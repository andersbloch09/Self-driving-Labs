#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "database_service/database_functions.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

using namespace BT;

// =============================================================================
// Condition Node: Check if there are materials still pending transport
// =============================================================================
class HasPendingMaterialsCondition : public ConditionNode
{
public:
    HasPendingMaterialsCondition(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config)
    {
        auto node = rclcpp::Node::make_shared("has_pending_materials");
        logger_ = node->get_logger();
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("materials_json", "JSON array of material names"),
            InputPort<std::string>("destination_map_json", "JSON object mapping materials to destinations"),
            OutputPort<int>("pending_count", "Number of materials still pending transport")
        };
    }

    NodeStatus tick() override
    {
        // Get inputs
        auto materials_json_str = getInput<std::string>("materials_json");
        auto dest_map_json_str = getInput<std::string>("destination_map_json");

        if (!materials_json_str || !dest_map_json_str) {
            RCLCPP_ERROR(logger_, "Missing required inputs: materials_json or destination_map_json");
            return NodeStatus::FAILURE;
        }

        try {
            // Parse JSON inputs
            auto materials_json = nlohmann::json::parse(materials_json_str.value());
            std::vector<std::string> materials = materials_json.get<std::vector<std::string>>();

            auto dest_map_json = nlohmann::json::parse(dest_map_json_str.value());
            std::map<std::string, std::string> destination_map = 
                dest_map_json.get<std::map<std::string, std::string>>();

            // Query database for materials needing transport
            auto pending = database_lib::getMaterialsNeedingTransport(materials, destination_map);

            setOutput("pending_count", static_cast<int>(pending.size()));

            if (pending.empty()) {
                RCLCPP_INFO(logger_, "✅ No pending materials - all at their destinations!");
                return NodeStatus::FAILURE;  // No more work to do
            } else {
                RCLCPP_INFO(logger_, "📦 %zu materials still need transport", pending.size());
                return NodeStatus::SUCCESS;  // More work needed
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error checking pending materials: %s", e.what());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Logger logger_;
};

// =============================================================================
// Action Node: Plan next batch of materials to load
// =============================================================================
class PlanNextBatchAction : public SyncActionNode
{
public:
    PlanNextBatchAction(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    {
        auto node = rclcpp::Node::make_shared("plan_next_batch");
        logger_ = node->get_logger();
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("materials_json", "JSON array of material names"),
            InputPort<std::string>("destination_map_json", "JSON object mapping materials to destinations"),
            InputPort<std::string>("transport_storage", "storage_mir", "Transport vehicle storage"),
            OutputPort<std::string>("batch_json", "JSON array of materials for this batch"),
            OutputPort<std::string>("source_location", "Current source location for pickup"),
            OutputPort<int>("batch_size", "Number of materials in this batch")
        };
    }

    NodeStatus tick() override
    {
        auto materials_json_str = getInput<std::string>("materials_json");
        auto dest_map_json_str = getInput<std::string>("destination_map_json");
        auto transport_storage = getInput<std::string>("transport_storage").value();

        if (!materials_json_str || !dest_map_json_str) {
            RCLCPP_ERROR(logger_, "Missing required inputs");
            return NodeStatus::FAILURE;
        }

        try {
            // Parse inputs
            auto materials_json = nlohmann::json::parse(materials_json_str.value());
            std::vector<std::string> materials = materials_json.get<std::vector<std::string>>();

            auto dest_map_json = nlohmann::json::parse(dest_map_json_str.value());
            std::map<std::string, std::string> destination_map = 
                dest_map_json.get<std::map<std::string, std::string>>();

            // Get materials needing transport (checks database for current locations)
            auto pending = database_lib::getMaterialsNeedingTransport(materials, destination_map);

            if (pending.empty()) {
                RCLCPP_WARN(logger_, "No pending materials to plan");
                return NodeStatus::FAILURE;
            }

            // Check available slots on transport vehicle
            int available_slots = database_lib::getAvailableSlotsCount(transport_storage);

            if (available_slots == 0) {
                RCLCPP_ERROR(logger_, "No available slots on transport vehicle!");
                return NodeStatus::FAILURE;
            }

            // Get containers for pending materials
            auto container_names = database_lib::getContainersWithMaterials(pending);
            
            if (container_names.empty()) {
                RCLCPP_ERROR(logger_, "No containers found for pending materials");
                return NodeStatus::FAILURE;
            }

            // Get current location of first container (assume all at same source for now)
            std::string source_location = database_lib::getContainerStorageObjectByContainerName(container_names[0]);
            if (source_location.empty()) {
                RCLCPP_ERROR(logger_, "Cannot determine source location for container: %s", 
                           container_names[0].c_str());
                return NodeStatus::FAILURE;
            }

            // Group materials by current location  
            std::vector<std::string> materials_at_source;
            for (size_t i = 0; i < container_names.size(); ++i) {
                std::string current_loc = database_lib::getContainerStorageObjectByContainerName(container_names[i]);
                if (current_loc == source_location) {
                    materials_at_source.push_back(pending[i]);
                }
            }

            if (materials_at_source.empty()) {
                RCLCPP_ERROR(logger_, "No materials found at source location: %s", source_location.c_str());
                return NodeStatus::FAILURE;
            }

            // Plan batch: take min(materials at source, available slots)
            int batch_size = std::min(static_cast<int>(materials_at_source.size()), available_slots);
            std::vector<std::string> batch(materials_at_source.begin(), materials_at_source.begin() + batch_size);

            // Output results
            nlohmann::json batch_json = batch;
            setOutput("batch_json", batch_json.dump());
            setOutput("source_location", source_location);
            setOutput("batch_size", batch_size);

            RCLCPP_INFO(logger_, "📋 Planned batch from '%s': %d materials (available slots: %d)", 
                       source_location.c_str(), batch_size, available_slots);
            for (const auto& material : batch) {
                RCLCPP_INFO(logger_, "   - %s", material.c_str());
            }

            return NodeStatus::SUCCESS;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error planning next batch: %s", e.what());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Logger logger_;
};

// =============================================================================
// Action Node: Load materials onto transport vehicle
// =============================================================================
class LoadMaterialsAction : public SyncActionNode
{
public:
    LoadMaterialsAction(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    {
        auto node = rclcpp::Node::make_shared("load_materials");
        logger_ = node->get_logger();
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("batch_json", "JSON array of materials to load"),
            InputPort<std::string>("transport_storage", "storage_mir", "Transport vehicle storage")
        };
    }

    NodeStatus tick() override
    {
        auto batch_json_str = getInput<std::string>("batch_json");
        auto transport_storage = getInput<std::string>("transport_storage").value();

        if (!batch_json_str) {
            RCLCPP_ERROR(logger_, "Missing required input: batch_json");
            return NodeStatus::FAILURE;
        }

        try {
            auto batch_json = nlohmann::json::parse(batch_json_str.value());
            std::vector<std::string> batch = batch_json.get<std::vector<std::string>>();

            RCLCPP_INFO(logger_, "🔄 Loading %zu materials onto %s", 
                       batch.size(), transport_storage.c_str());

            // Get containers for materials to load
            auto container_names = database_lib::getContainersWithMaterials(batch);
            
            if (container_names.size() != batch.size()) {
                RCLCPP_ERROR(logger_, "Expected %zu containers, found %zu", 
                           batch.size(), container_names.size());
                return NodeStatus::FAILURE;
            }

            // Move each container from current location to transport vehicle
            for (size_t i = 0; i < container_names.size(); ++i) {
                const auto& container_name = container_names[i];
                const auto& material_name = batch[i];

                bool success = database_lib::moveContainerToStorageObjectByName(
                    container_name, transport_storage);

                if (!success) {
                    RCLCPP_ERROR(logger_, "Failed to load container '%s' (material: %s)", 
                               container_name.c_str(), material_name.c_str());
                    return NodeStatus::FAILURE;
                }

                RCLCPP_INFO(logger_, "✅ Loaded: container '%s' (material: %s)", 
                          container_name.c_str(), material_name.c_str());
            }

            RCLCPP_INFO(logger_, "✅ All materials loaded successfully");
            return NodeStatus::SUCCESS;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error loading materials: %s", e.what());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Logger logger_;
};

// =============================================================================
// Action Node: Unload materials from transport vehicle to their destinations
// =============================================================================
class UnloadMaterialsAction : public SyncActionNode
{
public:
    UnloadMaterialsAction(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    {
        auto node = rclcpp::Node::make_shared("unload_materials");
        logger_ = node->get_logger();
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("materials_json", "JSON array of all material names"),
            InputPort<std::string>("destination_map_json", "JSON object mapping materials to destinations"),
            InputPort<std::string>("transport_storage", "storage_mir", "Transport vehicle storage")
        };
    }

    NodeStatus tick() override
    {
        auto materials_json_str = getInput<std::string>("materials_json");
        auto dest_map_json_str = getInput<std::string>("destination_map_json");
        auto transport_storage = getInput<std::string>("transport_storage").value();

        if (!materials_json_str || !dest_map_json_str) {
            RCLCPP_ERROR(logger_, "Missing required inputs");
            return NodeStatus::FAILURE;
        }

        try {
            auto materials_json = nlohmann::json::parse(materials_json_str.value());
            std::vector<std::string> materials = materials_json.get<std::vector<std::string>>();

            auto dest_map_json = nlohmann::json::parse(dest_map_json_str.value());
            std::map<std::string, std::string> destination_map = 
                dest_map_json.get<std::map<std::string, std::string>>();

            // Get all containers in transport storage
            auto all_on_transport = database_lib::getContainersInStorageObjectByName(transport_storage);

            if (all_on_transport.empty()) {
                RCLCPP_WARN(logger_, "No containers on transport vehicle to unload");
                return NodeStatus::SUCCESS;  // Nothing to do, but not a failure
            }

            // Filter to only materials we're tracking
            std::vector<std::string> materials_to_unload;
            for (const auto& container_name : all_on_transport) {
                // Check if this container contains any of our tracked materials
                for (const auto& material : materials) {
                    auto containers_with_material = database_lib::getContainersWithMaterials({material});
                    if (!containers_with_material.empty() && 
                        containers_with_material[0] == container_name) {
                        materials_to_unload.push_back(material);
                        break;
                    }
                }
            }

            if (materials_to_unload.empty()) {
                RCLCPP_WARN(logger_, "No tracked materials on transport vehicle to unload");
                return NodeStatus::SUCCESS;
            }

            RCLCPP_INFO(logger_, "📤 Unloading %zu materials from %s", 
                       materials_to_unload.size(), transport_storage.c_str());

            // Move each material's container from transport to its destination
            for (const auto& material : materials_to_unload) {
                // Get destination for this material
                auto dest_it = destination_map.find(material);
                if (dest_it == destination_map.end()) {
                    RCLCPP_ERROR(logger_, "No destination specified for material: %s", material.c_str());
                    return NodeStatus::FAILURE;
                }
                std::string destination = dest_it->second;

                // Get container name for this material
                auto container_names = database_lib::getContainersWithMaterials({material});
                if (container_names.empty()) {
                    RCLCPP_ERROR(logger_, "Cannot find container for material: %s", material.c_str());
                    return NodeStatus::FAILURE;
                }
                std::string container_name = container_names[0];

                RCLCPP_INFO(logger_, "Moving container '%s' (material '%s') to '%s'", 
                          container_name.c_str(), material.c_str(), destination.c_str());

                bool success = database_lib::moveContainerToStorageObjectByName(
                    container_name, destination);

                if (!success) {
                    RCLCPP_ERROR(logger_, "Failed to unload container: %s", container_name.c_str());
                    return NodeStatus::FAILURE;
                }

                RCLCPP_INFO(logger_, "✅ Unloaded: container '%s' → %s", 
                          container_name.c_str(), destination.c_str());
            }

            RCLCPP_INFO(logger_, "✅ All materials unloaded successfully");
            return NodeStatus::SUCCESS;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error unloading materials: %s", e.what());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Logger logger_;
};

// =============================================================================
// Action Node: Get mission ID for a storage location
// =============================================================================
class GetMissionForLocationAction : public SyncActionNode
{
public:
    GetMissionForLocationAction(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    {
        auto node = rclcpp::Node::make_shared("get_mission_for_location");
        logger_ = node->get_logger();
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("storage_location", "Storage location name"),
            InputPort<std::string>("mission_map_json", "JSON object mapping storage names to mission IDs"),
            OutputPort<std::string>("mission_id", "Mission ID for the location")
        };
    }

    NodeStatus tick() override
    {
        auto storage_location = getInput<std::string>("storage_location");
        auto mission_map_json_str = getInput<std::string>("mission_map_json");

        if (!storage_location || !mission_map_json_str) {
            RCLCPP_ERROR(logger_, "Missing required inputs");
            return NodeStatus::FAILURE;
        }

        try {
            auto mission_map_json = nlohmann::json::parse(mission_map_json_str.value());
            std::map<std::string, std::string> mission_map = 
                mission_map_json.get<std::map<std::string, std::string>>();

            auto mission_it = mission_map.find(storage_location.value());
            if (mission_it == mission_map.end()) {
                RCLCPP_ERROR(logger_, "No mission ID found for storage location: %s", 
                           storage_location.value().c_str());
                return NodeStatus::FAILURE;
            }

            std::string mission_id = mission_it->second;
            setOutput("mission_id", mission_id);

            RCLCPP_INFO(logger_, "📍 Storage '%s' → Mission '%s'", 
                       storage_location.value().c_str(), mission_id.c_str());

            return NodeStatus::SUCCESS;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error getting mission for location: %s", e.what());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Logger logger_;
};
