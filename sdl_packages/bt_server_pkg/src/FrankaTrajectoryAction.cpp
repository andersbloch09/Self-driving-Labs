#include "bt_server_pkg/franka_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool FrankaTrajectoryAction::setGoal(Goal& goal)
{
    // This action doesn't require specific goal parameters
    // The action server determines what to do based on the action name
    logInfo(logger(), "Setting goal for Franka trajectory action");
    return true;
}

NodeStatus FrankaTrajectoryAction::onResultReceived(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        logInfo(logger(), "Franka trajectory action succeeded");
        return NodeStatus::SUCCESS;
    } else {
        logError(logger(), "Franka trajectory action failed");
        return NodeStatus::FAILURE;
    }
}

NodeStatus FrankaTrajectoryAction::onFailure(ActionNodeErrorCode error)
{
    logError(logger(), "Franka trajectory action encountered an error");
    return NodeStatus::FAILURE;
}

void FrankaTrajectoryAction::onHalt()
{
    logInfo(logger(), "Franka trajectory action halted");
}

// Plugin registration
CreateRosNodePlugin(FrankaTrajectoryAction, "FrankaTrajectoryAction");
