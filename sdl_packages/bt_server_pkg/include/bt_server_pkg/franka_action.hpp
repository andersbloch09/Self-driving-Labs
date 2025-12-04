#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "btcpp_ros2_interfaces/action/cube_visual_calibration.hpp"
#include "btcpp_ros2_interfaces/action/pick_up.hpp"
#include "btcpp_ros2_interfaces/action/place.hpp"
#include "btcpp_ros2_interfaces/action/go_home.hpp"

using namespace BT;

// =============================================================================
// Base class for shared Franka logic
// =============================================================================
class FrankaActionBase {
protected:
    void logInfo(const rclcpp::Logger& logger, const std::string& message) {
        RCLCPP_INFO(logger, "%s", message.c_str());
    }
    
    void logError(const rclcpp::Logger& logger, const std::string& message) {
        RCLCPP_ERROR(logger, "%s", message.c_str());
    }
};

// =============================================================================
// Franka Trajectory Action (for go_home, pick, place, etc.)
// =============================================================================
class FrankaTrajectoryAction 
    : public RosActionNode<control_msgs::action::FollowJointTrajectory>,
      public FrankaActionBase
{
public:
    FrankaTrajectoryAction(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosActionNode<control_msgs::action::FollowJointTrajectory>(name, config, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("action_name", "Name of the action to call"),
            InputPort<std::string>("container_name", "", "Container name for pick/place operations")
        });
    }

    bool setGoal(Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& result) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;
};

// =============================================================================
// Franka Calibration Action (for cube visual calibration)
// =============================================================================
class FrankaCalibrationAction 
    : public RosActionNode<btcpp_ros2_interfaces::action::CubeVisualCalibration>,
      public FrankaActionBase
{
public:
    FrankaCalibrationAction(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::CubeVisualCalibration>(name, config, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("side", "left", "Side of the cube for calibration (left/right)")
        });
    }

    bool setGoal(Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& result) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;
};

// =============================================================================
// Pick Up Action (for picking up containers)
// =============================================================================
class PickUpAction 
    : public RosActionNode<btcpp_ros2_interfaces::action::PickUp>,
      public FrankaActionBase
{
public:
    PickUpAction(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::PickUp>(name, config, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("container_name", "Container name for pick operation"),
            InputPort<bool>("is_ot", "Is the container going to be placed in the OT")
        });
    }

    bool setGoal(Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& result) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;
};


// =============================================================================
//  Go Home Action
// =============================================================================
class GoHomeAction 
    : public RosActionNode<btcpp_ros2_interfaces::action::GoHome>,
      public FrankaActionBase
{
public:
    GoHomeAction(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::GoHome>(name, config, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<bool>("clear_planning_scene", "Whether to clear the planning scene before going home")
        });
    }

    bool setGoal(Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& result) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;
};


// =============================================================================
// Place Action (for placing containers)
// =============================================================================
class PlaceAction 
    : public RosActionNode<btcpp_ros2_interfaces::action::Place>,
      public FrankaActionBase
{
public:
    PlaceAction(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::Place>(name, config, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("container_name", "Container name for place operation"),
            InputPort<std::string>("slot_name", "Slot name where the container should be placed")
        });
    }

    bool setGoal(Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& result) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;
};