#include "bt_server_pkg/material_tracking_nodes.hpp"
#include "behaviortree_ros2/plugins.hpp"

// Register all material tracking plugins
CreateRosNodePlugin(HasPendingMaterialsCondition, "HasPendingMaterialsCondition");
CreateRosNodePlugin(PlanNextBatchAction, "PlanNextBatchAction");
CreateRosNodePlugin(LoadMaterialsAction, "LoadMaterialsAction");
CreateRosNodePlugin(UnloadMaterialsAction, "UnloadMaterialsAction");
CreateRosNodePlugin(GetMissionForLocationAction, "GetMissionForLocationAction");
