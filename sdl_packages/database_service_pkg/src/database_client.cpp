#include "rclcpp/rclcpp.hpp"
#include "database_service_pkg/srv/get_ot_tray_slot.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("database_client");
    
    // Create clients for both services
    auto tray_slot_client = node->create_client<database_service_pkg::srv::GetOTTraySlot>("get_ot_tray_slot");

    // Test the tray slot service (new functionality)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Testing tray slot service...");
    auto tray_request = std::make_shared<database_service_pkg::srv::GetOTTraySlot::Request>();
    tray_request->tray_id = 1;  // Example tray ID - change this to test different trays
    
    while (!tray_slot_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for tray slot service...");
    }

    auto tray_future = tray_slot_client->async_send_request(tray_request);
    if (rclcpp::spin_until_future_complete(node, tray_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = tray_future.get();
        if (response->success) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                       "Tray ID %d is in slot %d. Message: %s", 
                       tray_request->tray_id, response->slot, response->message.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                        "Failed to get tray slot: %s", response->message.c_str());
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call tray slot service");
    }
    
    rclcpp::shutdown();
    return 0;
}