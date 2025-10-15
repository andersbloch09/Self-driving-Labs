#include "bt_server_pkg/mir_mission_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

class GoToMission : public MirMissionAction
{
public:
    GoToMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : MirMissionAction(name, conf, params) {}
    
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("mission_id", "Mission ID to execute")
        });
    }
    
    bool setGoal(Goal& goal) override
    {
        // Get mission_id from the behavior tree XML
        auto mission_id = getInput<std::string>("mission_id");
        if (!mission_id)
        {
            RCLCPP_ERROR(logger(), "Missing required input [mission_id]");
            return false;
        }
        
        goal.mission_id = mission_id.value();
        //RCLCPP_INFO(logger(), "GoToMission: Setting mission_id: %s", goal.mission_id.c_str());
        return true;
    }
};

// Plugin registration
CreateRosNodePlugin(GoToMission, "GoToMission");