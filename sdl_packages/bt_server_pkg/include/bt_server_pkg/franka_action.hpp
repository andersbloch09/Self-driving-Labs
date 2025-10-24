#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace BT;

class FrankaAction : public RosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
    FrankaAction(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosActionNode<control_msgs::action::FollowJointTrajectory>(name, config, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setGoal(Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& result) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;
};