#include <rclcpp/rclcpp.hpp>
#include "database_service/database_functions.hpp"
#include <iostream>
#include <string>

class InteractiveDatabaseTester : public rclcpp::Node
{
public:
    InteractiveDatabaseTester() : Node("interactive_database_tester")
    {
        RCLCPP_INFO(this->get_logger(), "Interactive Database Tester Started");
        runInteractiveTests();
    }

private:
    void runInteractiveTests()
    {
        while (rclcpp::ok()) {
            std::cout << "\n=== Database Function Tester ===" << std::endl;
            std::cout << "1. Get storage object info" << std::endl;
            std::cout << "2. Find free slot" << std::endl;
            std::cout << "3. Get container location (by name)" << std::endl;
            std::cout << "4. List containers in storage object" << std::endl;
            std::cout << "5. Update container location (by name)" << std::endl;
            std::cout << "6. Get container location (by ID)" << std::endl;
            std::cout << "7. Auto-move container to storage object" << std::endl;
            std::cout << "8. Get container location transform (by name)" << std::endl;
            std::cout << "9. Get container storage object (by name)" << std::endl;
            std::cout << "0. Exit" << std::endl;
            std::cout << "Choice: ";
            
            int choice;
            std::cin >> choice;
            
            switch (choice) {
                case 1: testStorageObjectInfo(); break;
                case 2: testFreeSlot(); break;
                case 3: testGetContainerLocationByName(); break;
                case 4: testListContainers(); break;
                case 5: testUpdateContainerLocation(); break;
                case 6: testGetContainerLocationById(); break;
                case 7: testAutoMoveContainer(); break;
                case 8: testGetContainerLocationTransform(); break;
                case 9: testGetContainerStorageObject(); break;
                case 0: return;
                default: std::cout << "Invalid choice!" << std::endl;
            }
        }
    }
    
    void testStorageObjectInfo()
    {
        std::string storage_name;
        std::cout << "Enter storage object name: ";
        std::cin >> storage_name;
        
        auto info = database_lib::getStorageObjectInfo(storage_name);
        if (!info.id.empty()) {
            std::cout << "Found: " << info.name << " (Type: " << info.type << ")" << std::endl;
            std::cout << "ID: " << info.id << std::endl;
            std::cout << "Description: " << info.description << std::endl;
            std::cout << "Parent ID: " << info.parent_id << std::endl;
        } else {
            std::cout << "Storage object not found!" << std::endl;
        }
    }
    
    void testFreeSlot()
    {
        std::string storage_name;
        std::cout << "Enter storage object name: ";
        std::cin >> storage_name;
        
        std::string free_slot = database_lib::getFreeSlot(storage_name);
        if (!free_slot.empty()) {
            std::cout << "Free slot found: " << free_slot << std::endl;
        } else {
            std::cout << "No free slots available!" << std::endl;
        }
    }
    
    void testGetContainerLocationByName()
    {
        std::string container_name;
        std::cout << "Enter container name: ";
        std::cin >> container_name;
        
        std::string location = database_lib::getContainerLocationByName(container_name);
        if (!location.empty()) {
            std::cout << "Container location: " << location << std::endl;
        } else {
            std::cout << "Container not found or not assigned!" << std::endl;
        }
    }
    
    void testGetContainerLocationById()
    {
        std::string container_id;
        std::cout << "Enter container ID: ";
        std::cin >> container_id;
        
        std::string location = database_lib::getContainerLocation(container_id);
        if (!location.empty()) {
            std::cout << "Container location: " << location << std::endl;
        } else {
            std::cout << "Container not found or not assigned!" << std::endl;
        }
    }
    
    void testListContainers()
    {
        std::string storage_name;
        std::cout << "Enter storage object name: ";
        std::cin >> storage_name;
        
        auto containers = database_lib::getAllContainersInStorageObject(storage_name);
        std::cout << "Found " << containers.size() << " containers:" << std::endl;
        for (const auto& container : containers) {
            std::cout << "  - " << container << std::endl;
        }
    }
    
    void testUpdateContainerLocation()
    {
        std::string container_name, slot_name, moved_by;
        std::cout << "Enter container name: ";
        std::cin >> container_name;
        std::cout << "Enter target slot name: ";
        std::cin >> slot_name;
        std::cout << "Enter who/what moved it (optional, press enter for default): ";
        std::cin.ignore(); // Clear the newline from previous input
        std::getline(std::cin, moved_by);
        
        bool success;
        if (moved_by.empty()) {
            success = database_lib::updateContainerLocationByName(container_name, slot_name);
        } else {
            success = database_lib::updateContainerLocationByName(container_name, slot_name, moved_by);
        }
        
        if (success) {
            std::cout << "Container moved successfully!" << std::endl;
        } else {
            std::cout << "Failed to move container!" << std::endl;
        }
    }

    void testAutoMoveContainer()
    {
        std::string container_name, storage_object_name;
        std::cout << "Enter container name: ";
        std::cin >> container_name;
        std::cout << "Enter target storage object name: ";
        std::cin >> storage_object_name;
        
        if (database_lib::moveContainerToStorageObjectByName(container_name, storage_object_name)) {
            std::cout << "Container automatically placed in free slot!" << std::endl;
        } else {
            std::cout << "Failed to auto-move container!" << std::endl;
        }
    }

    void testGetContainerLocationTransform()
    {
        std::string container_name;
        std::cout << "Enter container name: ";
        std::cin >> container_name;
        
        std::string transform = database_lib::getContainerLocationTransform(container_name);
        if (!transform.empty()) {
            std::cout << "Container transform: " << transform << std::endl;
        } else {
            std::cout << "Container not found or not assigned to any slot!" << std::endl;
        }
    }

    void testGetContainerStorageObject()
    {
        std::string container_name;
        std::cout << "Enter container name: ";
        std::cin >> container_name;
        
        std::string storage_object = database_lib::getContainerStorageObjectByContainerName(container_name);
        if (!storage_object.empty()) {
            std::cout << "Container is in storage object: " << storage_object << std::endl;
        } else {
            std::cout << "Container not found or not assigned to any slot!" << std::endl;
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveDatabaseTester>();
    rclcpp::shutdown();
    return 0;
}