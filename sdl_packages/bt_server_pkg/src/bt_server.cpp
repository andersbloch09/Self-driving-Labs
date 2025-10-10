#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

// Include our action node headers
#include "bt_server_pkg/mir_mission_action.hpp"

class MirBehaviorTreeServer : public BT::TreeExecutionServer
{
public:
  MirBehaviorTreeServer(const rclcpp::NodeOptions& options) 
    : TreeExecutionServer(std::make_shared<rclcpp::Node>("bt_action_server", options))
  {
    RCLCPP_INFO(node()->get_logger(), "MiR Behavior Tree Server initialized");
    
    // Let the TreeExecutionServer handle everything - don't interfere with its initialization
    RCLCPP_INFO(node()->get_logger(), "TreeExecutionServer will handle plugin and BT file loading");
  }

protected:
  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override
  {
    // The action nodes are already registered via plugins (CreateRosNodePlugin)
    // So we don't need to register them manually here
    RCLCPP_INFO(node()->get_logger(), "Action nodes will be loaded automatically via plugins");
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    // Add a console logger to see what's happening
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    
    // Skip Groot2 publisher - let external client handle monitoring
    // External Groot2 client can connect directly to the behavior tree
    
    RCLCPP_INFO(node()->get_logger(), "Behavior Tree created with console logging enabled");
    RCLCPP_INFO(node()->get_logger(), "External Groot2 client should connect to port 1666 for monitoring");
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                      bool was_cancelled) override
  {
    // Clean up loggers
    logger_cout_.reset();
    
    std::string result_msg;
    if (was_cancelled) {
      result_msg = "Tree execution was cancelled";
      RCLCPP_WARN(node()->get_logger(), "%s", result_msg.c_str());
    } else {
      switch (status) {
        case BT::NodeStatus::SUCCESS:
          result_msg = "Tree execution completed successfully - All missions executed!";
          RCLCPP_INFO(node()->get_logger(), "%s", result_msg.c_str());
          break;
        case BT::NodeStatus::FAILURE:
          result_msg = "Tree execution failed";
          RCLCPP_ERROR(node()->get_logger(), "%s", result_msg.c_str());
          break;
        default:
          result_msg = "Tree execution finished with unknown status";
          RCLCPP_WARN(node()->get_logger(), "%s", result_msg.c_str());
          break;
      }
    }
    
    return result_msg;
  }

private:
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<MirBehaviorTreeServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "MiR Behavior Tree Server starting...");
  RCLCPP_INFO(action_server->node()->get_logger(), "Groot2 monitoring will be available once tree execution starts");

  // Use MultiThreadedExecutor to handle action clients properly
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
  return 0;
}