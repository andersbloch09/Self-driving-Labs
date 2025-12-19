#include "bt_server_pkg/opentrons2_run_protocol.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool OpenTrons2RunProtocol::setGoal(Goal& goal)
{
  // === Required: protocol path ===
  auto protocol_path = getInput<std::string>("protocol_path");
  if (!protocol_path)
  {
    RCLCPP_ERROR(logger(), "Missing required input [protocol_path]");
    return false;
  }
  goal.protocol_path = protocol_path.value();

  // === Optional: custom labware folder ===
  auto custom_labware = getInput<std::string>("custom_labware_folder");
  if (custom_labware)
  {
    goal.custom_labware_folder = custom_labware.value();
  }
  else
  {
    goal.custom_labware_folder = "";
  }

  // === Optional: JSON parameters (THIS WAS MISSING) ===
  auto parameters = getInput<std::string>("parameters_json");
  if (parameters)
  {
    goal.parameters_json = parameters.value();
    RCLCPP_INFO(logger(), 
                "parameters_json received: %s", 
                parameters.value().c_str());
  }
  else
  {
    goal.parameters_json = "";
    RCLCPP_WARN(logger(), "parameters_json not provided, using empty string");
  }

  // === Final goal logging ===
  RCLCPP_INFO(logger(),
              "OpenTrons2RunProtocol goal set:\n"
              "  protocol_path: %s\n"
              "  custom_labware_folder: %s\n"
              "  parameters_json: %s",
              goal.protocol_path.c_str(),
              goal.custom_labware_folder.c_str(),
              goal.parameters_json.c_str());

  return true;
}

BT::NodeStatus OpenTrons2RunProtocol::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(logger(),
              "%s: protocol completed (success=%s, message=\"%s\")",
              name().c_str(),
              wr.result->success ? "true" : "false",
              wr.result->message.c_str());

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus OpenTrons2RunProtocol::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(),
               "%s: action failed with error: %s",
               name().c_str(),
               toStr(error));

  return NodeStatus::FAILURE;
}

void OpenTrons2RunProtocol::onHalt()
{
  RCLCPP_INFO(logger(),
              "%s: protocol execution halted",
              name().c_str());
}

// Register the BT plugin
CreateRosNodePlugin(OpenTrons2RunProtocol, "OpenTrons2RunProtocol");
