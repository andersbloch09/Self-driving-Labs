#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace database_lib {

struct StorageObjectInfo {
    std::string id;
    std::string name;
    std::string type;
    std::string parent_id;
    std::string transform_json;
    std::string description;
};

class DatabaseHelpers {
public:
    // Default constructor using default connection parameters
    DatabaseHelpers();
    
    // Constructor with custom connection parameters
    DatabaseHelpers(const std::string& db_name, const std::string& db_user, 
                   const std::string& db_password, const std::string& db_host, 
                   const std::string& db_port);
    
    // Singleton access
    static DatabaseHelpers& getInstance();
    
    // Core database functions
    std::string getFreeSlot(const std::string& storage_object_name);

    bool updateContainerLocation(const std::string& container_id, const std::string& slot_name);

    std::string getContainerLocation(const std::string& container_id);

    std::string getContainerLocationByName(const std::string& container_name);

    std::string getContainerLocationTransform(const std::string& container_name);

    StorageObjectInfo getStorageObjectInfo(const std::string& storage_object_name);

    bool logMovement(const std::string& container_id, const std::string& from_slot, 
                    const std::string& to_slot, const std::string& moved_by = "Franka Robot");

    std::vector<std::string> getAllContainersInStorageObject(const std::string& storage_object_name);

    bool updateContainerLocation(const std::string& container_id, const std::string& slot_name, const std::string& moved_by);

    std::string getContainerStorageObjectByContainerName(const std::string& container_name);

    bool updateContainerLocationByName(const std::string& container_name, const std::string& slot_name);

    bool updateContainerLocationByName(const std::string& container_name, const std::string& slot_name, const std::string& moved_by);

    // Auto-placement functions - find free slot and move container
    bool moveContainerToStorageObject(const std::string& container_id, const std::string& storage_object_name);
    
    bool moveContainerToStorageObjectByName(const std::string& container_name, const std::string& storage_object_name);

    // Material tracking functions for multi-trip transport
    int getAvailableSlotsCount(const std::string& storage_object_name);
    
    std::vector<std::string> getContainersInStorageObjectByNames(
        const std::string& storage_object_name, 
        const std::vector<std::string>& container_names);
    
    std::vector<std::string> getContainersWithMaterials(
        const std::vector<std::string>& material_names);
    
    std::vector<std::string> getMaterialsNeedingTransport(
        const std::vector<std::string>& material_names,
        const std::map<std::string, std::string>& destination_map);

private:
    std::string db_name_;
    std::string db_user_;
    std::string db_password_;
    std::string db_host_;
    std::string db_port_;
    
    // Internal helper methods - implementation details hidden in .cpp
    void* createConnection();
    void* createConnection(const std::string& db_name, const std::string& db_user,
                          const std::string& db_password, const std::string& db_host,
                          const std::string& db_port);
};

// Convenience functions for easy import and use
std::string getFreeSlot(const std::string& storage_object_name);

bool updateContainerLocation(const std::string& container_id, const std::string& slot_name);

std::string getContainerLocation(const std::string& container_id);

StorageObjectInfo getStorageObjectInfo(const std::string& storage_object_name);

bool logMovement(const std::string& container_id, const std::string& from_slot, 
                const std::string& to_slot, const std::string& moved_by = "robot");

bool updateContainerLocation(const std::string& container_id, const std::string& slot_name, const std::string& moved_by);

std::string getContainerLocationByName(const std::string& container_name);

bool updateContainerLocationByName(const std::string& container_name, const std::string& slot_name);

bool updateContainerLocationByName(const std::string& container_name, const std::string& slot_name, const std::string& moved_by);

std::vector<std::string> getAllContainersInStorageObject(const std::string& storage_object_name);

std::string getContainerLocationTransform(const std::string& container_name);

std::string getContainerStorageObjectByContainerName(const std::string& container_name);

// Auto-placement convenience functions
bool moveContainerToStorageObject(const std::string& container_id, const std::string& storage_object_name);

bool moveContainerToStorageObjectByName(const std::string& container_name, const std::string& storage_object_name);

// Material tracking convenience functions
int getAvailableSlotsCount(const std::string& storage_object_name);

std::vector<std::string> getContainersInStorageObjectByNames(
    const std::string& storage_object_name, 
    const std::vector<std::string>& container_names);

std::vector<std::string> getContainersWithMaterials(
    const std::vector<std::string>& material_names);

std::vector<std::string> getMaterialsNeedingTransport(
    const std::vector<std::string>& material_names,
    const std::map<std::string, std::string>& destination_map);

} // namespace database_lib