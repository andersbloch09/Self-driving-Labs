#include "bt_server_pkg/franka_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool PlaceAction::setGoal(Goal& goal)
{
    // Extract container_name from input port
    auto container_name = getInput<std::string>("container_name");
    if (!container_name) {
        logError(logger(), "Missing required input [container_name]");
        return false;
    }
    
    goal.container_name = container_name.value();
    logInfo(logger(), "Setting goal to place container: " + goal.container_name);
    return true;
}

NodeStatus PlaceAction::onResultReceived(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        logInfo(logger(), "Place action succeeded: " + result.result->message);
        return NodeStatus::SUCCESS;
    } else {
        logError(logger(), "Place action failed: " + result.result->message);
        return NodeStatus::FAILURE;
    }
}

NodeStatus PlaceAction::onFailure(ActionNodeErrorCode error)
{
    logError(logger(), "Place action encountered an error");
    return NodeStatus::FAILURE;
}

void PlaceAction::onHalt()
{
    logInfo(logger(), "Place action halted");
}

// Plugin registration
CreateRosNodePlugin(PlaceAction, "PlaceAction");
