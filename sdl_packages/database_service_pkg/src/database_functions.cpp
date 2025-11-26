#include "database_service/database_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pqxx/pqxx>
#include <memory>
#include <stdexcept>

namespace database_lib {

DatabaseHelpers::DatabaseHelpers() 
{
    // Initialize connection parameters with defaults
    db_name_ = "postgres";
    db_user_ = "postgres.mytenant";
    db_password_ = "your-super-secret-and-long-postgres-password";
    db_host_ = "172.17.0.1";
    db_port_ = "5432";
}

DatabaseHelpers::DatabaseHelpers(const std::string& db_name, const std::string& db_user,
                                 const std::string& db_password, const std::string& db_host,
                                 const std::string& db_port)
    : db_name_(db_name), db_user_(db_user), db_password_(db_password), 
      db_host_(db_host), db_port_(db_port)
{
}

void* DatabaseHelpers::createConnection()
{
    return createConnection(db_name_, db_user_, db_password_, db_host_, db_port_);
}

void* DatabaseHelpers::createConnection(const std::string& db_name, const std::string& db_user,
                                       const std::string& db_password, const std::string& db_host,
                                       const std::string& db_port)
{
    try {
        std::string connection_string = "dbname=" + db_name + 
                                       " user=" + db_user + 
                                       " password=" + db_password + 
                                       " host=" + db_host + 
                                       " port=" + db_port;
        return new pqxx::connection(connection_string);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error creating connection: %s", e.what());
        return nullptr;
    }
}

std::string DatabaseHelpers::getFreeSlot(const std::string& storage_object_name)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return "";
        }
        pqxx::work transaction(*connection);
        
        // Query to find the first available slot in the specified storage object
        std::string query = R"(
            SELECT s.transform_to_object
            FROM slots s
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE so.name = )" + transaction.quote(storage_object_name) + R"(
            AND s.container_id IS NULL
            ORDER BY s.name
            LIMIT 1;
        )";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "No free slots found in storage object: %s", storage_object_name.c_str());
            return "";
        }
        // return the transform of the first free slot found
        std::string free_slot = result[0][0].as<std::string>();

        transaction.commit();
        
        return free_slot;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error finding free slot in %s: %s", storage_object_name.c_str(), e.what());
        return "";
    }
}

bool DatabaseHelpers::updateContainerLocation(const std::string& container_id, const std::string& slot_name)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return false;
        }
        pqxx::work transaction(*connection);
        
        // First, get the current location of the container for logging (both name and UUID)
        std::string current_location_query = R"(
            SELECT s.name, s.id
            FROM containers c
            LEFT JOIN slots s ON c.current_slot_id = s.id
            WHERE c.id = )" + transaction.quote(container_id) + ";";
        
        pqxx::result current_result = transaction.exec(current_location_query);
        std::string from_slot = "";
        std::string from_slot_id = "";
        
        if (!current_result.empty() && !current_result[0][0].is_null()) {
            from_slot = current_result[0][0].as<std::string>();
            from_slot_id = current_result[0][1].as<std::string>();
        } else {
            from_slot = "unassigned"; // Container was not previously assigned to any slot
            from_slot_id = ""; // No previous slot ID
        }
        
        // Get the slot_id and storage_object_id for the new location
        std::string slot_query = "SELECT id, storage_object_id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        std::string storage_object_id = slot_result[0][1].as<std::string>();
        
        // Get the storage object name for the containers table
        std::string storage_query = "SELECT name FROM storage_objects WHERE id = " + transaction.quote(storage_object_id) + ";";
        pqxx::result storage_result = transaction.exec(storage_query);
        std::string storage_object_name = storage_result[0][0].as<std::string>();
        
        // First, clear the old slot if container was previously assigned
        if (from_slot != "unassigned") {
            std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
            transaction.exec(clear_old_slot_query);
        }
        
        // Update container location in containers table
        std::string update_container_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                           ", current_storage_object = " + transaction.quote(storage_object_name) +
                                           ", last_update = NOW()" +
                                           " WHERE id = " + transaction.quote(container_id) + ";";
        
        pqxx::result update_result = transaction.exec(update_container_query);
        
        // Check if the update actually affected any rows
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_id.c_str());
            return false;
        }
        
        // Update the slot to reference this container
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        // Automatically log the movement using UUIDs for database compatibility
        std::string log_query;
        if (!from_slot_id.empty()) {
            // Moving from one slot to another
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote("robot_system") + ", NOW());";
        } else {
            // Moving from unassigned to a slot
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", NULL, " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote("robot_system") + ", NOW());";
        }
        
        transaction.exec(log_query);
        transaction.commit();
        
        return true;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error updating container location: %s", e.what());
        return false;
    }
}

bool DatabaseHelpers::updateContainerLocation(const std::string& container_id, const std::string& slot_name, const std::string& moved_by)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return false;
        }
        pqxx::work transaction(*connection);
        
        // First, get the current location of the container for logging (both name and UUID)
        std::string current_location_query = R"(
            SELECT s.name, s.id
            FROM containers c
            LEFT JOIN slots s ON c.current_slot_id = s.id
            WHERE c.id = )" + transaction.quote(container_id) + ";";
        
        pqxx::result current_result = transaction.exec(current_location_query);
        std::string from_slot = "";
        std::string from_slot_id = "";
        
        if (!current_result.empty() && !current_result[0][0].is_null()) {
            from_slot = current_result[0][0].as<std::string>();
            from_slot_id = current_result[0][1].as<std::string>();
        } else {
            from_slot = "unassigned"; // Container was not previously assigned to any slot
            from_slot_id = ""; // No previous slot ID
        }
        
        // Get the slot_id and storage_object_id for the new location
        std::string slot_query = "SELECT id, storage_object_id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        std::string storage_object_id = slot_result[0][1].as<std::string>();
        
        // Get the storage object name for the containers table
        std::string storage_query = "SELECT name FROM storage_objects WHERE id = " + transaction.quote(storage_object_id) + ";";
        pqxx::result storage_result = transaction.exec(storage_query);
        std::string storage_object_name = storage_result[0][0].as<std::string>();
        
        // First, clear the old slot if container was previously assigned
        if (from_slot != "unassigned") {
            std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
            transaction.exec(clear_old_slot_query);
        }
        
        // Update container location in containers table
        std::string update_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                  ", current_storage_object = " + transaction.quote(storage_object_name) +
                                  ", last_update = NOW()" +
                                  " WHERE id = " + transaction.quote(container_id) + ";";
        
        pqxx::result update_result = transaction.exec(update_query);
        
        // Check if the update actually affected any rows
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_id.c_str());
            return false;
        }
        
        // Update the slot to reference this container
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        // Check if the update actually affected any rows
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_id.c_str());
            return false;
        }
        
        // Automatically log the movement with the provided moved_by parameter using UUIDs
        std::string log_query;
        if (!from_slot_id.empty()) {
            // Moving from one slot to another
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote(moved_by) + ", NOW());";
        } else {
            // Moving from unassigned to a slot
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", NULL, " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote(moved_by) + ", NOW());";
        }
        
        transaction.exec(log_query);
        transaction.commit();
        
        return true;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error updating container location: %s", e.what());
        return false;
    }
}

std::string DatabaseHelpers::getContainerLocation(const std::string& container_id)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        std::string query = R"(
            SELECT s.name 
            FROM containers c
            JOIN slots s ON c.current_slot_id = s.id
            WHERE c.id = )" + transaction.quote(container_id) + ";";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "Container '%s' not found or not assigned to any slot", container_id.c_str());
            return "";
        }
        
        std::string location = result[0][0].as<std::string>();
        transaction.commit();
        
        return location;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting container location: %s", e.what());
        return "";
    }
}

std::string DatabaseHelpers::getContainerLocationByName(const std::string& container_name)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        std::string query = R"(
            SELECT s.name 
            FROM containers c
            JOIN slots s ON c.current_slot_id = s.id
            WHERE c.name = )" + transaction.quote(container_name) + ";";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "Container '%s' not found or not assigned to any slot", container_name.c_str());
            return "";
        }
        
        std::string location = result[0][0].as<std::string>();
        transaction.commit();
        
        return location;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting container location by name: %s", e.what());
        return "";
    }
}

std::string DatabaseHelpers::getContainerLocationTransform(const std::string& container_name)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        std::string query = R"(
            SELECT s.transform_to_object 
            FROM containers c
            JOIN slots s ON c.current_slot_id = s.id
            WHERE c.name = )" + transaction.quote(container_name) + ";";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "Container '%s' not found or not assigned to any slot", container_name.c_str());
            return "";
        }
        
        std::string location_transform = result[0][0].as<std::string>();
        transaction.commit();
        
        return location_transform;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting container location transform: %s", e.what());
        return "";
    }
}

StorageObjectInfo DatabaseHelpers::getStorageObjectInfo(const std::string& storage_object_name)
{
    StorageObjectInfo info;
    
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        std::string query = R"(
            SELECT id, name, type, parent_id, transform_to_parent, description
            FROM storage_objects 
            WHERE name = )" + transaction.quote(storage_object_name) + ";";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "Storage object '%s' not found", storage_object_name.c_str());
            return info;
        }
        
        info.id = result[0]["id"].as<std::string>();
        info.name = result[0]["name"].as<std::string>();
        info.type = result[0]["type"].is_null() ? "" : result[0]["type"].as<std::string>();
        info.parent_id = result[0]["parent_id"].is_null() ? "" : result[0]["parent_id"].as<std::string>();
        info.transform_json = result[0]["transform_to_parent"].is_null() ? "" : result[0]["transform_to_parent"].as<std::string>();
        info.description = result[0]["description"].is_null() ? "" : result[0]["description"].as<std::string>();
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                    "Retrieved info for storage object '%s'", storage_object_name.c_str());
                    
        transaction.commit();
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting storage object info: %s", e.what());
    }
    
    return info;
}

bool DatabaseHelpers::logMovement(const std::string& container_id, const std::string& from_slot, 
                                 const std::string& to_slot, const std::string& moved_by)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        // Log movement using slot names for human readability
        std::string query = R"(
            INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
            VALUES ()" + transaction.quote(container_id) + ", " +
                       transaction.quote(from_slot) + ", " +
                       transaction.quote(to_slot) + ", " +
                       transaction.quote(moved_by) + ", NOW());";
        
        transaction.exec(query);
        transaction.commit();
        
        return true;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error logging movement: %s", e.what());
        return false;
    }
}

std::vector<std::string> DatabaseHelpers::getAllContainersInStorageObject(const std::string& storage_object_name)
{
    std::vector<std::string> containers;
    
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        std::string query = R"(
            SELECT c.id 
            FROM containers c
            JOIN slots s ON c.current_slot_id = s.id
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE so.name = )" + transaction.quote(storage_object_name) + 
            " ORDER BY s.name;";
        
        pqxx::result result = transaction.exec(query);
        
        for (const auto& row : result) {
            containers.push_back(row[0].as<std::string>());
        }
        
        transaction.commit();
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting containers in storage object: %s", e.what());
    }
    
    return containers;
}

std::string DatabaseHelpers::getContainerStorageObjectByContainerName(const std::string& container_name)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        std::string query = R"(
            SELECT so.name 
            FROM containers c
            JOIN slots s ON c.current_slot_id = s.id
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE c.name = )" + transaction.quote(container_name) + ";";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "Container '%s' not found or not assigned to any slot", container_name.c_str());
            return "";
        }
        
        std::string storage_object_name = result[0][0].as<std::string>();
        transaction.commit();
        
        return storage_object_name;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting container storage object by name: %s", e.what());
        return "";
    }
}

bool DatabaseHelpers::updateContainerLocationByName(const std::string& container_name, const std::string& slot_name)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        // First, get the current location of the container for logging (both name and UUID)
        std::string current_location_query = R"(
            SELECT s.name, s.id
            FROM containers c
            LEFT JOIN slots s ON c.current_slot_id = s.id
            WHERE c.name = )" + transaction.quote(container_name) + ";";
        
        pqxx::result current_result = transaction.exec(current_location_query);
        
        if (current_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_name.c_str());
            return false;
        }
        
        std::string from_slot = "";
        std::string from_slot_id = "";
        if (!current_result[0][0].is_null()) {
            from_slot = current_result[0][0].as<std::string>();
            from_slot_id = current_result[0][1].as<std::string>();
        } else {
            from_slot = "unassigned"; // Container was not previously assigned to any slot
            from_slot_id = ""; // No previous slot ID
        }
        
        // Get the slot_id for the new location
        std::string slot_query = "SELECT id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        
        // Update container location
        std::string update_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                  " WHERE name = " + transaction.quote(container_name) + ";";
        
        pqxx::result update_result = transaction.exec(update_query);
        
        // Check if the update actually affected any rows
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Failed to update container '%s' - no rows affected", container_name.c_str());
            return false;
        }
        
        // Get container ID for slot update and logging
        std::string container_id_query = "SELECT id FROM containers WHERE name = " + transaction.quote(container_name) + ";";
        pqxx::result container_id_result = transaction.exec(container_id_query);
        std::string container_id = container_id_result[0][0].as<std::string>();
        
        // Clear the old slot (free up previous location)
        std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
        transaction.exec(clear_old_slot_query);
        
        // Update the slot to reference this container and mark as occupied
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        // Automatically log the movement using UUIDs for database compatibility
        std::string log_query;
        if (!from_slot_id.empty()) {
            // Moving from one slot to another
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote("robot_system") + ", NOW());";
        } else {
            // Moving from unassigned to a slot
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", NULL, " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote("robot_system") + ", NOW());";
        }
        
        transaction.exec(log_query);
        transaction.commit();
        
        return true;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error updating container location by name: %s", e.what());
        return false;
    }
}

bool DatabaseHelpers::updateContainerLocationByName(const std::string& container_name, const std::string& slot_name, const std::string& moved_by)
{
    try {
        auto connection = static_cast<pqxx::connection*>(createConnection());
        pqxx::work transaction(*connection);
        
        // First, get the current location of the container for logging (both name and UUID)
        std::string current_location_query = R"(
            SELECT s.name, s.id
            FROM containers c
            LEFT JOIN slots s ON c.current_slot_id = s.id
            WHERE c.name = )" + transaction.quote(container_name) + ";";
        
        pqxx::result current_result = transaction.exec(current_location_query);
        
        if (current_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_name.c_str());
            return false;
        }
        
        std::string from_slot = "";
        std::string from_slot_id = "";
        if (!current_result[0][0].is_null()) {
            from_slot = current_result[0][0].as<std::string>();
            from_slot_id = current_result[0][1].as<std::string>();
        } else {
            from_slot = "unassigned";
            from_slot_id = "";
        }
        
        // Get the slot_id for the new location
        std::string slot_query = "SELECT id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        
        // Update container location
        std::string update_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                  " WHERE name = " + transaction.quote(container_name) + ";";
        
        pqxx::result update_result = transaction.exec(update_query);
        
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Failed to update container '%s' - no rows affected", container_name.c_str());
            return false;
        }
        
        // Get container ID for logging
        std::string container_id_query = "SELECT id FROM containers WHERE name = " + transaction.quote(container_name) + ";";
        pqxx::result container_id_result = transaction.exec(container_id_query);
        std::string container_id = container_id_result[0][0].as<std::string>();
        
        // Clear the old slot (free up previous location)
        std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
        transaction.exec(clear_old_slot_query);
        
        // Update the slot to reference this container and mark as occupied
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        // Automatically log the movement using UUIDs for database compatibility
        std::string log_query;
        if (!from_slot_id.empty()) {
            // Moving from one slot to another
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote(moved_by) + ", NOW());";
        } else {
            // Moving from unassigned to a slot
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", NULL, " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote(moved_by) + ", NOW());";
        }
        
        transaction.exec(log_query);
        transaction.commit();
        
        return true;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error updating container location by name: %s", e.what());
        return false;
    }
}

bool DatabaseHelpers::moveContainerToStorageObject(const std::string& container_id, const std::string& storage_object_name)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return false;
        }
        pqxx::work transaction(*connection);
        
        // First, find a free slot in the specified storage object
        std::string free_slot_query = R"(
            SELECT s.name 
            FROM slots s
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE so.name = )" + transaction.quote(storage_object_name) + R"(
            AND s.container_id IS NULL
            ORDER BY s.name
            LIMIT 1;
        )";
        
        pqxx::result free_slot_result = transaction.exec(free_slot_query);
        
        if (free_slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "No free slots available in storage object '%s'", storage_object_name.c_str());
            return false;
        }
        
        std::string free_slot_name = free_slot_result[0][0].as<std::string>();
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Found free slot '%s' in storage object '%s' for container '%s'", 
                   free_slot_name.c_str(), storage_object_name.c_str(), container_id.c_str());
        
        transaction.commit();
        
        // Now use the existing updateContainerLocation function to move the container
        return updateContainerLocation(container_id, free_slot_name, "auto_placement");
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error moving container to storage object: %s", e.what());
        return false;
    }
}

bool DatabaseHelpers::moveContainerToStorageObjectByName(const std::string& container_name, const std::string& storage_object_name)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return false;
        }
        pqxx::work transaction(*connection);
        
        // First, find a free slot in the specified storage object
        std::string free_slot_query = R"(
            SELECT s.name 
            FROM slots s
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE so.name = )" + transaction.quote(storage_object_name) + R"(
            AND s.container_id IS NULL
            ORDER BY s.name
            LIMIT 1;
        )";
        
        pqxx::result free_slot_result = transaction.exec(free_slot_query);
        
        if (free_slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "No free slots available in storage object '%s'", storage_object_name.c_str());
            return false;
        }
        
        std::string free_slot_name = free_slot_result[0][0].as<std::string>();
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Found free slot '%s' in storage object '%s' for container '%s'", 
                   free_slot_name.c_str(), storage_object_name.c_str(), container_name.c_str());
        
        transaction.commit();
        
        // Now use the existing updateContainerLocationByName function to move the container
        return updateContainerLocationByName(container_name, free_slot_name, "auto_placement");
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error moving container to storage object by name: %s", e.what());
        return false;
    }
}

std::string DatabaseHelpers::getSlotTransform(const std::string& slot_name)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return "";
        }
        pqxx::work transaction(*connection);
        
        // Query to get the transform for a specific slot
        std::string query = R"(
            SELECT s.transform_to_object
            FROM slots s
            WHERE s.name = )" + transaction.quote(slot_name) + ";";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                       "Slot '%s' not found in database", slot_name.c_str());
            return "";
        }
        
        // Return the transform
        std::string slot_transform = result[0][0].as<std::string>();
        
        transaction.commit();
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Retrieved transform for slot '%s'", slot_name.c_str());
        
        return slot_transform;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error getting slot transform for '%s': %s", slot_name.c_str(), e.what());
        return "";
    }
}

// Singleton instance getter
DatabaseHelpers& DatabaseHelpers::getInstance()
{
    static DatabaseHelpers instance;
    return instance;
}

// Convenience functions for easy use
std::string getFreeSlot(const std::string& storage_object_name)
{
    return DatabaseHelpers::getInstance().getFreeSlot(storage_object_name);
}

bool updateContainerLocation(const std::string& container_id, const std::string& slot_name)
{
    return DatabaseHelpers::getInstance().updateContainerLocation(container_id, slot_name);
}

std::string getContainerLocation(const std::string& container_id)
{
    return DatabaseHelpers::getInstance().getContainerLocation(container_id);
}

std::string getContainerLocationByName(const std::string& container_name)
{
    return DatabaseHelpers::getInstance().getContainerLocationByName(container_name);
}

StorageObjectInfo getStorageObjectInfo(const std::string& storage_object_name)
{
    return DatabaseHelpers::getInstance().getStorageObjectInfo(storage_object_name);
}

bool logMovement(const std::string& container_id, const std::string& from_slot, 
                const std::string& to_slot, const std::string& moved_by)
{
    return DatabaseHelpers::getInstance().logMovement(container_id, from_slot, to_slot, moved_by);
}

bool updateContainerLocation(const std::string& container_id, const std::string& slot_name, const std::string& moved_by)
{
    return DatabaseHelpers::getInstance().updateContainerLocation(container_id, slot_name, moved_by);
}

bool updateContainerLocationByName(const std::string& container_name, const std::string& slot_name)
{
    return DatabaseHelpers::getInstance().updateContainerLocationByName(container_name, slot_name);
}

bool updateContainerLocationByName(const std::string& container_name, const std::string& slot_name, const std::string& moved_by)
{
    return DatabaseHelpers::getInstance().updateContainerLocationByName(container_name, slot_name, moved_by);
}

std::vector<std::string> getAllContainersInStorageObject(const std::string& storage_object_name)
{
    return DatabaseHelpers::getInstance().getAllContainersInStorageObject(storage_object_name);
}

std::string getContainerLocationTransform(const std::string& container_name)
{
    return DatabaseHelpers::getInstance().getContainerLocationTransform(container_name);
}

std::string getContainerStorageObjectByContainerName(const std::string& container_name)
{
    return DatabaseHelpers::getInstance().getContainerStorageObjectByContainerName(container_name);
}

bool moveContainerToStorageObject(const std::string& container_id, const std::string& storage_object_name)
{
    return DatabaseHelpers::getInstance().moveContainerToStorageObject(container_id, storage_object_name);
}

bool moveContainerToStorageObjectByName(const std::string& container_name, const std::string& storage_object_name)
{
    return DatabaseHelpers::getInstance().moveContainerToStorageObjectByName(container_name, storage_object_name);
}

std::string getSlotTransform(const std::string& slot_name)
{
    return DatabaseHelpers::getInstance().getSlotTransform(slot_name);
}

// Material tracking functions
int DatabaseHelpers::getAvailableSlotsCount(const std::string& storage_object_name)
{
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return 0;
        }
        pqxx::work transaction(*connection);
        
        // Count available (empty) slots in the storage object
        std::string query = R"(
            SELECT COUNT(*) 
            FROM slots s
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE so.name = )" + transaction.quote(storage_object_name) + R"(
            AND s.container_id IS NULL;
        )";
        
        pqxx::result result = transaction.exec(query);
        
        if (result.empty()) {
            return 0;
        }
        
        int available_count = result[0][0].as<int>();
        transaction.commit();
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Storage object '%s' has %d available slots", 
                   storage_object_name.c_str(), available_count);
        
        return available_count;
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error counting available slots: %s", e.what());
        return 0;
    }
}

std::vector<std::string> DatabaseHelpers::getContainersInStorageObjectByNames(
    const std::string& storage_object_name, 
    const std::vector<std::string>& container_names)
{
    std::vector<std::string> found_containers;
    
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return found_containers;
        }
        pqxx::work transaction(*connection);
        
        // Build IN clause for container names
        std::string names_list;
        for (size_t i = 0; i < container_names.size(); ++i) {
            if (i > 0) names_list += ", ";
            names_list += transaction.quote(container_names[i]);
        }
        
        // Find which of the specified containers are in this storage object
        std::string query = R"(
            SELECT c.name 
            FROM containers c
            JOIN slots s ON c.current_slot_id = s.id
            JOIN storage_objects so ON s.storage_object_id = so.id
            WHERE so.name = )" + transaction.quote(storage_object_name) + R"(
            AND c.name IN ()" + names_list + R"()
            ORDER BY c.name;
        )";
        
        pqxx::result result = transaction.exec(query);
        
        for (const auto& row : result) {
            found_containers.push_back(row[0].as<std::string>());
        }
        
        transaction.commit();
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Found %zu containers from material list in '%s'", 
                   found_containers.size(), storage_object_name.c_str());
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error finding containers in storage object: %s", e.what());
    }
    
    return found_containers;
}

std::vector<std::string> DatabaseHelpers::getContainersWithMaterials(
    const std::vector<std::string>& material_names)
{
    std::vector<std::string> container_names;
    
    try {
        std::unique_ptr<pqxx::connection> connection(static_cast<pqxx::connection*>(createConnection()));
        if (!connection) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Failed to create database connection");
            return container_names;
        }
        pqxx::work transaction(*connection);
        
        // For each material, find containers that have it in their contents
        for (const auto& material : material_names) {
            std::string query = R"(
                SELECT name 
                FROM containers 
                WHERE contents ? )" + transaction.quote(material) + R"(
                LIMIT 1;
            )";
            
            pqxx::result result = transaction.exec(query);
            
            if (!result.empty()) {
                std::string container_name = result[0][0].as<std::string>();
                container_names.push_back(container_name);
                RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                           "Material '%s' found in container '%s'", 
                           material.c_str(), container_name.c_str());
            } else {
                RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                           "No container found containing material '%s'", material.c_str());
            }
        }
        
        transaction.commit();
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error finding containers with materials: %s", e.what());
    }
    
    return container_names;
}

std::vector<std::string> DatabaseHelpers::getMaterialsNeedingTransport(
    const std::vector<std::string>& material_names,
    const std::map<std::string, std::string>& destination_map)
{
    std::vector<std::string> needs_transport;
    
    try {
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Checking which of %zu materials need transport...", material_names.size());
        
        for (const auto& material : material_names) {
            // Get destination for this material
            auto dest_it = destination_map.find(material);
            if (dest_it == destination_map.end()) {
                RCLCPP_WARN(rclcpp::get_logger("database_helpers"), 
                           "No destination specified for material '%s', skipping", material.c_str());
                continue;
            }
            std::string destination = dest_it->second;
            
            // Find container with this material and get its current location
            auto containers = getContainersWithMaterials({material});
            if (containers.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                            "Cannot find container with material '%s' in database", material.c_str());
                continue;
            }
            
            std::string container_name = containers[0];
            std::string current_location = getContainerStorageObjectByContainerName(container_name);
            
            if (current_location.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                            "Cannot determine location for container '%s'", container_name.c_str());
                continue;
            }
            
            // Check if already at destination
            if (current_location == destination) {
                RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                           "✅ Material '%s' (in '%s') already at destination '%s'", 
                           material.c_str(), container_name.c_str(), destination.c_str());
            } else if (current_location == "storage_mir") {
                RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                           "🚚 Material '%s' (in '%s') currently on transport vehicle", 
                           material.c_str(), container_name.c_str());
            } else {
                RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                           "📦 Material '%s' (in '%s') needs transport: %s → %s", 
                           material.c_str(), container_name.c_str(), 
                           current_location.c_str(), destination.c_str());
                needs_transport.push_back(material);
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                   "Result: %zu materials need transport", needs_transport.size());
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error determining materials needing transport: %s", e.what());
    }
    
    return needs_transport;
}

// Convenience wrappers
int getAvailableSlotsCount(const std::string& storage_object_name)
{
    return DatabaseHelpers::getInstance().getAvailableSlotsCount(storage_object_name);
}

std::vector<std::string> getContainersInStorageObjectByNames(
    const std::string& storage_object_name, 
    const std::vector<std::string>& container_names)
{
    return DatabaseHelpers::getInstance().getContainersInStorageObjectByNames(storage_object_name, container_names);
}

std::vector<std::string> getContainersWithMaterials(
    const std::vector<std::string>& material_names)
{
    return DatabaseHelpers::getInstance().getContainersWithMaterials(material_names);
}

std::vector<std::string> getMaterialsNeedingTransport(
    const std::vector<std::string>& material_names,
    const std::map<std::string, std::string>& destination_map)
{
    return DatabaseHelpers::getInstance().getMaterialsNeedingTransport(material_names, destination_map);
}

} // namespace database_lib
