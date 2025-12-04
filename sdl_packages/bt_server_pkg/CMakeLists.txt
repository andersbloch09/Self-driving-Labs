cmake_minimum_required(VERSION 3.16)
project(bt_server_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(ot2_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(database_service REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(database_service REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

# Helper function to create plugins (DRY principle)
function(create_bt_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file} src/mir_mission_action.cpp)
  set_target_properties(${plugin_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
  )
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${plugin_name} 
    rclcpp rclcpp_action behaviortree_ros2 btcpp_ros2_interfaces pluginlib
  )
endfunction()

# Helper function for OpenTrons2 plugins
function(create_ot2_bt_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file})
  set_target_properties(${plugin_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
  )
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${plugin_name} 
    rclcpp rclcpp_action behaviortree_ros2 ot2_interfaces pluginlib
  )
endfunction()

# Helper function for Franka plugins
function(create_franka_bt_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file})
  set_target_properties(${plugin_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
  )
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${plugin_name} 
    rclcpp rclcpp_action behaviortree_ros2 control_msgs btcpp_ros2_interfaces pluginlib
  )
endfunction()

# Helper function for material tracking plugins
function(create_material_tracking_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file})
  set_target_properties(${plugin_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
  )
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${plugin_name} 
    rclcpp behaviortree_ros2 database_service nlohmann_json pluginlib
  )
endfunction()

# Helper function for material tracking plugins
function(create_material_tracking_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file})
  set_target_properties(${plugin_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
  )
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${plugin_name} 
    rclcpp behaviortree_ros2 database_service nlohmann_json pluginlib
  )
endfunction()

# Create MiR plugins
create_bt_plugin(GoToMission src/GoToMission.cpp)

# Create OpenTrons2 plugins
create_ot2_bt_plugin(OpenTrons2RunProtocol src/opentrons2_run_protocol.cpp)

# Create Franka plugins
create_franka_bt_plugin(FrankaTrajectoryAction src/FrankaTrajectoryAction.cpp)
create_franka_bt_plugin(FrankaCalibrationAction src/FrankaCalibrationAction.cpp)
create_franka_bt_plugin(PickUpAction src/PickUpAction.cpp)
create_franka_bt_plugin(PlaceAction src/PlaceAction.cpp)

# Create Material Tracking plugins
create_material_tracking_plugin(MaterialTrackingNodes src/material_tracking_nodes.cpp)
create_franka_bt_plugin(GoHome src/GoHome.cpp)

# Create Material Tracking plugins
create_material_tracking_plugin(MaterialTrackingNodes src/material_tracking_nodes.cpp)

# Main executable (simplified dependencies)
add_executable(bt_server src/bt_server.cpp)
ament_target_dependencies(bt_server
  rclcpp behaviortree_ros2 btcpp_ros2_interfaces
)

# Install targets
install(TARGETS bt_server DESTINATION lib/${PROJECT_NAME})

# Install plugin libraries
install(TARGETS GoToMission OpenTrons2RunProtocol FrankaTrajectoryAction FrankaCalibrationAction PickUpAction PlaceAction MaterialTrackingNodes GoHome MaterialTrackingNodes
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS GoToMission OpenTrons2RunProtocol FrankaTrajectoryAction FrankaCalibrationAction PickUpAction PlaceAction MaterialTrackingNodes
install(TARGETS GoToMission OpenTrons2RunProtocol FrankaTrajectoryAction FrankaCalibrationAction PickUpAction PlaceAction MaterialTrackingNodes
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS GoToMission OpenTrons2RunProtocol FrankaTrajectoryAction FrankaCalibrationAction PickUpAction PlaceAction MaterialTrackingNodes MaterialTrackingNodes
  DESTINATION share/${PROJECT_NAME}/plugins
)

install(PROGRAMS src/mir_mission_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config behavior_trees include/
  DESTINATION share/${PROJECT_NAME}/
)

# Plugin export
pluginlib_export_plugin_description_file(behaviortree_ros2 plugins.xml)

ament_package()