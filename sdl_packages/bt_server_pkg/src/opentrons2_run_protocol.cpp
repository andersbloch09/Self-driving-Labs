#include "bt_server_pkg/opentrons2_run_protocol.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool OpenTrons2RunProtocol::setGoal(Goal& goal)
{
  auto protocol_path = getInput<std::string>("protocol_path");
  if (!protocol_path)
  {
    RCLCPP_ERROR(logger(), "Missing required input [protocol_path]");
    return false;
  }

  goal.protocol_path = protocol_path.value();
  
  // Optional custom labware folder
  auto custom_labware = getInput<std::string>("custom_labware_folder");
  if (custom_labware)
  {
    goal.custom_labware_folder = custom_labware.value();
  }
  else
  {
    goal.custom_labware_folder = "";  // Default empty
  }
  
  RCLCPP_INFO(logger(), "Setting OpenTrons2 goal with protocol: %s", goal.protocol_path.c_str());
  return true;
}

BT::NodeStatus OpenTrons2RunProtocol::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: Protocol execution result - Success: %s, Message: %s", 
              name().c_str(), wr.result->success ? "true" : "false", wr.result->message.c_str());

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus OpenTrons2RunProtocol::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void OpenTrons2RunProtocol::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt - Protocol execution was halted", name().c_str());
}

// Register OpenTrons2RunProtocol as a behavior tree plugin
CreateRosNodePlugin(OpenTrons2RunProtocol, "OpenTrons2RunProtocol");