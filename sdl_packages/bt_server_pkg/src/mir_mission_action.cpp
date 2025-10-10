#include "bt_server_pkg/mir_mission_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool MirMissionAction::setGoal(RosActionNode::Goal& goal)
{
  auto mission_id = getInput<std::string>("mission_id");
  if (!mission_id)
  {
    RCLCPP_ERROR(logger(), "Missing required input [mission_id]");
    return false;
  }
  
  goal.mission_id = mission_id.value();
  RCLCPP_INFO(logger(), "Setting goal with mission_id: %s", goal.mission_id.c_str());
  return true;
}

NodeStatus MirMissionAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Status = %d, Message = %s", 
              name().c_str(), (int)wr.result->node_status.status, wr.result->return_message.c_str());

  // Convert NodeStatus from action result to BT NodeStatus
  switch(wr.result->node_status.status)
  {
    case btcpp_ros2_interfaces::msg::NodeStatus::SUCCESS:
      return NodeStatus::SUCCESS;
    case btcpp_ros2_interfaces::msg::NodeStatus::FAILURE:
      return NodeStatus::FAILURE;
    default:
      return NodeStatus::FAILURE;
  }
}

NodeStatus MirMissionAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void MirMissionAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Base class implementation - no plugin registration needed
// Individual derived classes handle their own registration