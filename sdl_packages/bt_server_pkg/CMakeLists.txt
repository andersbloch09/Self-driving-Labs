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
find_package(pluginlib REQUIRED)

# Include directories
include_directories(include)

# Create action nodes as separate shared libraries (plugins)
# Include the base class implementation in each plugin
add_library(GoToStorage SHARED 
  src/GoToStorage.cpp
  src/mir_mission_action.cpp
)
add_library(GoToOpenTrons2 SHARED 
  src/GoToOpenTrons2.cpp
  src/mir_mission_action.cpp
)
add_library(GoToHome SHARED 
  src/GoToHome.cpp
  src/mir_mission_action.cpp
)

# Set visibility for plugin exports
set_target_properties(GoToStorage PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)
set_target_properties(GoToOpenTrons2 PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)
set_target_properties(GoToHome PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

# Define BT_PLUGIN_EXPORT for proper symbol export
target_compile_definitions(GoToStorage PRIVATE BT_PLUGIN_EXPORT)
target_compile_definitions(GoToOpenTrons2 PRIVATE BT_PLUGIN_EXPORT)
target_compile_definitions(GoToHome PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(GoToStorage
  rclcpp
  rclcpp_action
  behaviortree_ros2
  btcpp_ros2_interfaces
  pluginlib
)

ament_target_dependencies(GoToOpenTrons2
  rclcpp
  rclcpp_action
  behaviortree_ros2
  btcpp_ros2_interfaces
  pluginlib
)

ament_target_dependencies(GoToHome
  rclcpp
  rclcpp_action
  behaviortree_ros2
  btcpp_ros2_interfaces
  pluginlib
)

# Create the main BT server executable
add_executable(bt_server src/bt_server.cpp)

ament_target_dependencies(bt_server
  rclcpp
  behaviortree_ros2
  btcpp_ros2_interfaces
)

# Link the action nodes libraries to the server
target_link_libraries(bt_server GoToStorage GoToOpenTrons2 GoToHome)

# Install targets
install(TARGETS
  bt_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugins in share directory for TreeExecutionServer discovery
install(TARGETS
  GoToStorage
  GoToOpenTrons2
  GoToHome
  DESTINATION share/${PROJECT_NAME}/plugins
)

# Install Python scripts
install(PROGRAMS
  src/mir_mission_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Plugin export
pluginlib_export_plugin_description_file(behaviortree_ros2 plugins.xml)

ament_package()