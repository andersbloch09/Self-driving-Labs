#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/go_to_mission.hpp"
#include "btcpp_ros2_interfaces/msg/node_status.hpp"

using namespace BT;

class MirMissionAction : public RosActionNode<btcpp_ros2_interfaces::action::GoToMission>
{
public:
  MirMissionAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::GoToMission>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ InputPort<std::string>("mission_id") });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};