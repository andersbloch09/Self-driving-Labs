#include "database_service/database_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pqxx/pqxx>
#include <memory>
#include <stdexcept>
#include <vector>
#include <string>

namespace database_lib {

// NOTE: The struct StorageObjectInfo is defined only in the header file.

// Helper function to create the connection string, now including the application name
std::string create_conn_string(const std::string& db_name, const std::string& db_user,
                               const std::string& db_password, const std::string& db_host,
                               const std::string& db_port)
{
    std::string conn_str = "dbname=" + db_name + 
                           " user=" + db_user + 
                           " password=" + db_password + 
                           " host=" + db_host + 
                           " port=" + db_port;
    
    // FIX 1: Add application_name to the connection string to avoid set_session_var error
    conn_str += " application_name='ROS2RobotSystem'"; 
    return conn_str;
}

// Macro for repetitive connection check 
#define CHECK_CONNECTION_AND_RETURN(ret_val) \
    if (!connection_ || !connection_->is_open()) { \
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), "Database connection is not open. Function aborted."); \
        return ret_val; \
    }

// --- REFACTORED CONSTRUCTORS ---

DatabaseHelpers::DatabaseHelpers() 
{
    db_name_ = "postgres";
    db_user_ = "postgres.mytenant";
    db_password_ = "your-super-secret-and-long-postgres-password";
    db_host_ = "172.17.0.1";
    db_port_ = "5432";

    try {
        std::string connection_string = create_conn_string(db_name_, db_user_, db_password_, db_host_, db_port_);
        connection_ = std::make_unique<pqxx::connection>(connection_string);
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                    "Long-lived database connection established successfully.");
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("database_helpers"), 
                     "FATAL: Error establishing database connection: %s", e.what());
    }
}

DatabaseHelpers::DatabaseHelpers(const std::string& db_name, const std::string& db_user,
                                 const std::string& db_password, const std::string& db_host,
                                 const std::string& db_port)
    : db_name_(db_name), db_user_(db_user), db_password_(db_password), 
      db_host_(db_host), db_port_(db_port)
{
    try {
        std::string connection_string = create_conn_string(db_name_, db_user_, db_password_, db_host_, db_port_);
        connection_ = std::make_unique<pqxx::connection>(connection_string);
        RCLCPP_INFO(rclcpp::get_logger("database_helpers"), 
                    "Long-lived database connection established successfully.");
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("database_helpers"), 
                     "FATAL: Error establishing database connection: %s", e.what());
    }
}

// --- REFACTORED METHODS ---

std::string DatabaseHelpers::getFreeSlot(const std::string& storage_object_name)
{
    CHECK_CONNECTION_AND_RETURN("");
    
    try {
        pqxx::work transaction(*connection_);
        
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
    CHECK_CONNECTION_AND_RETURN(false);
    
    try {
        pqxx::work transaction(*connection_);
        
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
            from_slot = "unassigned";
            from_slot_id = "";
        }
        
        std::string slot_query = "SELECT id, storage_object_id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        std::string storage_object_id = slot_result[0][1].as<std::string>();
        
        std::string storage_query = "SELECT name FROM storage_objects WHERE id = " + transaction.quote(storage_object_id) + ";";
        pqxx::result storage_result = transaction.exec(storage_query);
        std::string storage_object_name = storage_result[0][0].as<std::string>();
        
        if (from_slot != "unassigned") {
            std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
            transaction.exec(clear_old_slot_query);
        }
        
        std::string update_container_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                           ", current_storage_object = " + transaction.quote(storage_object_name) +
                                           ", last_update = NOW()" +
                                           " WHERE id = " + transaction.quote(container_id) + ";";
        
        pqxx::result update_result = transaction.exec(update_container_query);
        
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_id.c_str());
            return false;
        }
        
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        std::string log_query;
        if (!from_slot_id.empty()) {
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote("robot_system") + ", NOW());";
        } else {
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
    CHECK_CONNECTION_AND_RETURN(false);

    try {
        pqxx::work transaction(*connection_);
        
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
            from_slot = "unassigned"; 
            from_slot_id = ""; 
        }
        
        std::string slot_query = "SELECT id, storage_object_id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        std::string storage_object_id = slot_result[0][1].as<std::string>();
        
        std::string storage_query = "SELECT name FROM storage_objects WHERE id = " + transaction.quote(storage_object_id) + ";";
        pqxx::result storage_result = transaction.exec(storage_query);
        std::string storage_object_name = storage_result[0][0].as<std::string>();
        
        if (from_slot != "unassigned") {
            std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
            transaction.exec(clear_old_slot_query);
        }
        
        std::string update_container_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                           ", current_storage_object = " + transaction.quote(storage_object_name) +
                                           ", last_update = NOW()" +
                                           " WHERE id = " + transaction.quote(container_id) + ";";
        
        pqxx::result update_result = transaction.exec(update_container_query);
        
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_id.c_str());
            return false;
        }
        
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Container '%s' not found in database", container_id.c_str());
            return false;
        }
        
        std::string log_query;
        if (!from_slot_id.empty()) {
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote(moved_by) + ", NOW());";
        } else {
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
    CHECK_CONNECTION_AND_RETURN("");
    
    try {
        pqxx::work transaction(*connection_);
        
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
    CHECK_CONNECTION_AND_RETURN("");
    
    try {
        pqxx::work transaction(*connection_);
        
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
    CHECK_CONNECTION_AND_RETURN("");
    
    try {
        pqxx::work transaction(*connection_);
        
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
    
    CHECK_CONNECTION_AND_RETURN(info);
    
    try {
        pqxx::work transaction(*connection_);
        
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
    CHECK_CONNECTION_AND_RETURN(false);
    
    try {
        pqxx::work transaction(*connection_);
        
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
    
    CHECK_CONNECTION_AND_RETURN(containers);
    
    try {
        pqxx::work transaction(*connection_);
        
        std::string query = R"(
            SELECT c.name 
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
    CHECK_CONNECTION_AND_RETURN("");
    
    try {
        pqxx::work transaction(*connection_);
        
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
    CHECK_CONNECTION_AND_RETURN(false);
    
    try {
        pqxx::work transaction(*connection_);
        
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
        
        std::string slot_query = "SELECT id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        
        std::string update_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                  " WHERE name = " + transaction.quote(container_name) + ";";
        
        pqxx::result update_result = transaction.exec(update_query);
        
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Failed to update container '%s' - no rows affected", container_name.c_str());
            return false;
        }
        
        std::string container_id_query = "SELECT id FROM containers WHERE name = " + transaction.quote(container_name) + ";";
        pqxx::result container_id_result = transaction.exec(container_id_query);
        std::string container_id = container_id_result[0][0].as<std::string>();
        
        std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
        transaction.exec(clear_old_slot_query);
        
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        std::string log_query;
        if (!from_slot_id.empty()) {
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote("robot_system") + ", NOW());";
        } else {
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
    CHECK_CONNECTION_AND_RETURN(false);
    
    try {
        pqxx::work transaction(*connection_);
        
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
        
        std::string slot_query = "SELECT id FROM slots WHERE name = " + transaction.quote(slot_name) + ";";
        pqxx::result slot_result = transaction.exec(slot_query);
        
        if (slot_result.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Slot '%s' not found in database", slot_name.c_str());
            return false;
        }
        
        std::string slot_id = slot_result[0][0].as<std::string>();
        
        std::string update_query = "UPDATE containers SET current_slot_id = " + transaction.quote(slot_id) + 
                                  " WHERE name = " + transaction.quote(container_name) + ";";
        
        pqxx::result update_result = transaction.exec(update_query);
        
        if (update_result.affected_rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                        "Failed to update container '%s' - no rows affected", container_name.c_str());
            return false;
        }
        
        std::string container_id_query = "SELECT id FROM containers WHERE name = " + transaction.quote(container_name) + ";";
        pqxx::result container_id_result = transaction.exec(container_id_query);
        std::string container_id = container_id_result[0][0].as<std::string>();
        
        std::string clear_old_slot_query = "UPDATE slots SET container_id = NULL, status = 'available' WHERE container_id = " + transaction.quote(container_id) + ";";
        transaction.exec(clear_old_slot_query);
        
        std::string update_slot_query = "UPDATE slots SET container_id = " + 
                                       transaction.quote(container_id) + 
                                       ", status = 'occupied'" +
                                       " WHERE id = " + transaction.quote(slot_id) + ";";
        
        transaction.exec(update_slot_query);
        
        std::string log_query;
        if (!from_slot_id.empty()) {
            log_query = R"(
                INSERT INTO movements (container_id, from_slot, to_slot, moved_by, timestamp)
                VALUES ()" + transaction.quote(container_id) + ", " +
                           transaction.quote(from_slot_id) + ", " +
                           transaction.quote(slot_id) + ", " +
                           transaction.quote(moved_by) + ", NOW());";
        } else {
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
    CHECK_CONNECTION_AND_RETURN(false);
    
    try {
        pqxx::work transaction(*connection_);
        
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
        
        return updateContainerLocation(container_id, free_slot_name, "auto_placement");
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error moving container to storage object: %s", e.what());
        return false;
    }
}

bool DatabaseHelpers::moveContainerToStorageObjectByName(const std::string& container_name, const std::string& storage_object_name)
{
    CHECK_CONNECTION_AND_RETURN(false);
    
    try {
        pqxx::work transaction(*connection_);
        
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
        
        return updateContainerLocationByName(container_name, free_slot_name, "auto_placement");
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("database_helpers"), 
                    "Error moving container to storage object by name: %s", e.what());
        return false;
    }
}

std::string DatabaseHelpers::getSlotTransform(const std::string& slot_name)
{
    CHECK_CONNECTION_AND_RETURN("");
    
    try {
        pqxx::work transaction(*connection_);
        
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

} // namespace database_lib