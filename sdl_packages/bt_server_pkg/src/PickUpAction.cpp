#include "bt_server_pkg/franka_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool PickUpAction::setGoal(Goal& goal)
{
    // Extract container_name from input port
    auto container_name = getInput<std::string>("container_name");
    if (!container_name) {
        logError(logger(), "Missing required input [container_name]");
        return false;
    }
    
    goal.container_name = container_name.value();
    logInfo(logger(), "Setting goal to pick up container: " + goal.container_name);
    return true;
}

NodeStatus PickUpAction::onResultReceived(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        logInfo(logger(), "Pick up action succeeded: " + result.result->message);
        return NodeStatus::SUCCESS;
    } else {
        logError(logger(), "Pick up action failed: " + result.result->message);
        return NodeStatus::FAILURE;
    }
}

NodeStatus PickUpAction::onFailure(ActionNodeErrorCode error)
{
    logError(logger(), "Pick up action encountered an error");
    return NodeStatus::FAILURE;
}

void PickUpAction::onHalt()
{
    logInfo(logger(), "Pick up action halted");
}

// Plugin registration
CreateRosNodePlugin(PickUpAction, "PickUpAction");
