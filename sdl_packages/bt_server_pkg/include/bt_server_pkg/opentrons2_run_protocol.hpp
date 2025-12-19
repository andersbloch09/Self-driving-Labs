#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "ot2_interfaces/action/run_protocol.hpp"

using namespace BT;

/**
 * @brief BehaviorTree.CPP ROS2 action node for running an OT-2 protocol.
 * This node sends the protocol path, optional labware folder, and optional
 * parameters JSON string to the OT-2 via the RunProtocol action.
 */
class OpenTrons2RunProtocol 
    : public RosActionNode<ot2_interfaces::action::RunProtocol>
{
public:
  using Goal = ot2_interfaces::action::RunProtocol::Goal;
  using WrappedResult = 
        RosActionNode<ot2_interfaces::action::RunProtocol>::WrappedResult;

  OpenTrons2RunProtocol(const std::string& name,
                        const NodeConfig& conf,
                        const RosNodeParams& params)
    : RosActionNode<ot2_interfaces::action::RunProtocol>(name, conf, params)
  {}

  /**
   * @brief Declare the input ports available to the BehaviorTree.
   * protocol_path          - required
   * custom_labware_folder  - optional
   * parameters_json        - optional JSON string
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<std::string>("protocol_path", 
                             "", 
                             "Path to the OT-2 protocol file"),
      InputPort<std::string>("custom_labware_folder", 
                             "", 
                             "Optional folder for custom OT-2 labware"),
      InputPort<std::string>("parameters_json", 
                             "", 
                             "Optional JSON parameters passed to the protocol")
    });
  }

  /// Fill the ROS2 Action goal based on BT input ports.
  bool setGoal(Goal& goal) override;

  /// Called when the action succeeds and a result is returned.
  NodeStatus onResultReceived(const WrappedResult& wr) override;

  /// Called when an action fails before completion.
  NodeStatus onFailure(ActionNodeErrorCode error) override;

  /// Called when the BT halts this node.
  void onHalt() override;
};
