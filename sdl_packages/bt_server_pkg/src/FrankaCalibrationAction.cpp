#include "bt_server_pkg/franka_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool FrankaCalibrationAction::setGoal(Goal& goal)
{
    auto side = getInput<std::string>("side");
    if (!side) {
        logError(logger(), "Missing required input [side]");
        return false;
    }
    
    goal.side = side.value();
    logInfo(logger(), "Setting calibration goal with side: " + goal.side);
    return true;
}

NodeStatus FrankaCalibrationAction::onResultReceived(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        logInfo(logger(), "Franka calibration action succeeded");
        return NodeStatus::SUCCESS;
    } else {
        logError(logger(), "Franka calibration action failed");
        return NodeStatus::FAILURE;
    }
}

NodeStatus FrankaCalibrationAction::onFailure(ActionNodeErrorCode error)
{
    logError(logger(), "Franka calibration action encountered an error");
    return NodeStatus::FAILURE;
}

void FrankaCalibrationAction::onHalt()
{
    logInfo(logger(), "Franka calibration action halted");
}

// Plugin registration
CreateRosNodePlugin(FrankaCalibrationAction, "FrankaCalibrationAction");
