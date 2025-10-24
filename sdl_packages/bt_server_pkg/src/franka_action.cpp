#include "bt_server_pkg/franka_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool FrankaAction::setGoal(Goal& goal)
{
    // FollowJointTrajectory action - the manipulation server will handle the square movement
    // No specific goal parameters needed as the server has predefined trajectories
    RCLCPP_INFO(logger(), "FrankaAction: Setting goal for move_in_square action");
    
    // The goal is empty as the manipulation server handles the predefined square movement
    // goal.trajectory is set by the action server itself
    return true;
}

NodeStatus FrankaAction::onResultReceived(const WrappedResult& result)
{
    if (result.result->error_code == 0)
    {
        RCLCPP_INFO(logger(), "FrankaAction: Move in square completed successfully");
        return NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(logger(), "FrankaAction: Move in square failed: %s", 
                     result.result->error_string.c_str());
        return NodeStatus::FAILURE;
    }
}

NodeStatus FrankaAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "FrankaAction: onFailure with error: %d", static_cast<int>(error));
    return NodeStatus::FAILURE;
}

void FrankaAction::onHalt()
{
    RCLCPP_INFO(logger(), "FrankaAction: onHalt called");
}

// Register the plugin
CreateRosNodePlugin(FrankaAction, "FrankaAction");