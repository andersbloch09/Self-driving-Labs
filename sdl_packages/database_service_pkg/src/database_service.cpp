#include "rclcpp/rclcpp.hpp"
#include <pqxx/pqxx>  // PostgreSQL C++ API
// SRV - Ignore error
#include "database_service_pkg/srv/get_chemical.hpp"
#include "database_service_pkg/srv/get_ot_tray_slot.hpp"

using namespace std::placeholders;

class DatabaseService : public rclcpp::Node
{
public:
    DatabaseService() : Node("database_service")
    {
        this->db_name = this->declare_parameter<std::string>("db_name", "postgres");
        this->db_user = this->declare_parameter<std::string>("db_user", "postgres.mytenant");
        this->db_password = this->declare_parameter<std::string>("db_password", "your-super-secret-and-long-postgres-password");
        this->db_host = this->declare_parameter<std::string>("db_host", "172.17.0.1");
        this->db_port = this->declare_parameter<std::string>("db_port", "5432");

        get_chemical_service_ = this->create_service<database_service_pkg::srv::GetChemical>(
            "get_chemical",
            std::bind(&DatabaseService::get_chemical_service, this, _1, _2));

        get_ot_tray_slot_service_ = this->create_service<database_service_pkg::srv::GetOTTraySlot>(
            "get_ot_tray_slot",
            std::bind(&DatabaseService::get_ot_tray_slot_service, this, _1, _2));
        
        RCLCPP_INFO(this->get_logger(), "Database service ready.");
    }

private:
    std::string db_name;
    std::string db_user;
    std::string db_password;
    std::string db_host;
    std::string db_port;

    void get_chemical_service(
        const std::shared_ptr<database_service_pkg::srv::GetChemical::Request> request,
        std::shared_ptr<database_service_pkg::srv::GetChemical::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming Request: %ld + %ld = %ld", 
                                    request->a, request->b, response->sum);
    }

    void get_ot_tray_slot_service(
        const std::shared_ptr<database_service_pkg::srv::GetOTTraySlot::Request> request,
        std::shared_ptr<database_service_pkg::srv::GetOTTraySlot::Response> response)
    {
        try {
            pqxx::connection C("dbname=" + db_name + " user=" + db_user +
                            " password=" + db_password + " hostaddr=" + db_host +
                            " port=" + db_port);
            pqxx::work W(C); // Start a transaction

            
            int tray_id = static_cast<int>(request->tray_id);
            std::string query = "SELECT \"HandoverSlot\" FROM \"OpenTrons\" WHERE \"TrayID\" = " + W.quote(tray_id) + ";";
            pqxx::result R = W.exec(query);

            if (R.empty()) {
                response->success = false;
                response->message = "Tray not found in database.";
                response->slot = -1;  // Indicate invalid slot
                RCLCPP_WARN(this->get_logger(), "Tray ID %d not found in database", static_cast<int>(request->tray_id));
                return;
            }
                
            // Get the slot value using column index instead of name
            int slot_value = R[0][0].as<int>();  // Use index 0 instead of column name
            response->slot = static_cast<int8_t>(slot_value);
            response->success = true;
            response->message = "Tray slot retrieved successfully.";
            
            RCLCPP_INFO(this->get_logger(), "Tray ID %s is in slot %s", 
                        std::to_string(static_cast<int>(request->tray_id)).c_str(),
                        std::to_string(static_cast<int>(response->slot)).c_str());
        
            
        } catch (const std::exception &e) {
            response->success = false;
            response->message = "Database error: " + std::string(e.what());
            response->slot = -1;
            RCLCPP_ERROR(this->get_logger(), "Database error: %s", e.what());
        } catch (...) {
            response->success = false;
            response->message = "Caught unknown exception";
            response->slot = -1;
        }
    }

    rclcpp::Service<database_service_pkg::srv::GetChemical>::SharedPtr get_chemical_service_;
    rclcpp::Service<database_service_pkg::srv::GetOTTraySlot>::SharedPtr get_ot_tray_slot_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DatabaseService>());
    rclcpp::shutdown();
    return 0;
}