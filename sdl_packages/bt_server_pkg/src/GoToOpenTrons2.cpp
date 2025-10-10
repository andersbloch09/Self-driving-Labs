#include "bt_server_pkg/mir_mission_action.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/plugins.hpp"

class GoToOpenTrons2 : public MirMissionAction
{
public:
    GoToOpenTrons2(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : MirMissionAction(name, conf, params) {}
    
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }
    
    bool setGoal(Goal& goal) override
    {
        // Use the hardcoded mission ID for GoToOpenTrons2
        goal.mission_id = "a5c110d4-a4f7-11f0-b2e5-000e8e984489";
        RCLCPP_INFO(logger(), "GoToOpenTrons2: Setting mission_id: %s", goal.mission_id.c_str());
        return true;
    }
};

// Plugin registration
CreateRosNodePlugin(GoToOpenTrons2, "GoToOpenTrons2");