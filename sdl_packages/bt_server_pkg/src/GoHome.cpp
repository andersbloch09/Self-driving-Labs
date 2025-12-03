#include "bt_server_pkg/franka_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool GoHomeAction::setGoal(Goal& goal)
{
    // Extract container_name from input port
    auto clear_planning_scene = getInput<bool>("clear_planning_scene");
    if (!clear_planning_scene) {
        logError(logger(), "Missing required input [clear_planning_scene]");
        return false;
    }
    
    goal.clear_planning_scene = clear_planning_scene.value();
    logInfo(logger(), "Setting goal to go home with clear_planning_scene: " + std::to_string(goal.clear_planning_scene));
    return true;
}

NodeStatus GoHomeAction::onResultReceived(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        logInfo(logger(), "Go home action succeeded: " + result.result->message);
        return NodeStatus::SUCCESS;
    } else {
        logError(logger(), "Go home action failed: " + result.result->message);
        return NodeStatus::FAILURE;
    }
}

NodeStatus GoHomeAction::onFailure(ActionNodeErrorCode error)
{
    logError(logger(), "Go home action encountered an error");
    return NodeStatus::FAILURE;
}

void GoHomeAction::onHalt()
{
    logInfo(logger(), "Go home action halted");
}

// Plugin registration
CreateRosNodePlugin(GoHomeAction, "GoHomeAction");
