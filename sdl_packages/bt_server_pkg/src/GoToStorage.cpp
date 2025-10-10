#include "bt_server_pkg/mir_mission_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

class GoToStorage : public MirMissionAction
{
public:
    GoToStorage(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : MirMissionAction(name, conf, params) {}
    
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }
    
    bool setGoal(Goal& goal) override
    {
        // Use the hardcoded mission ID for GoToStorage
        goal.mission_id = "94c9f0cf-a4f7-11f0-b2e5-000e8e984489";
        RCLCPP_INFO(logger(), "GoToStorage: Setting mission_id: %s", goal.mission_id.c_str());
        return true;
    }
};

// Plugin registration
CreateRosNodePlugin(GoToStorage, "GoToStorage");

