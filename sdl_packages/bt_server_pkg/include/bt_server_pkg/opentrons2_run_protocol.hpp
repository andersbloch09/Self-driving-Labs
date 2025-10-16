#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "ot2_interfaces/action/run_protocol.hpp"

using namespace BT;

/**
 * @brief OpenTrons2 action for executing protocols
 */
class OpenTrons2RunProtocol : public RosActionNode<ot2_interfaces::action::RunProtocol>
{
public:
  using Goal = ot2_interfaces::action::RunProtocol::Goal;
  using WrappedResult = RosActionNode<ot2_interfaces::action::RunProtocol>::WrappedResult;

  OpenTrons2RunProtocol(const std::string& name, const NodeConfig& conf,
                        const RosNodeParams& params)
    : RosActionNode<ot2_interfaces::action::RunProtocol>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<std::string>("protocol_path"),
      InputPort<std::string>("custom_labware_folder", "", "Optional custom labware folder")
    });
  }

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
  void onHalt() override;
};