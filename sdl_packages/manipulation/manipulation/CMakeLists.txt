cmake_minimum_required(VERSION 3.8)
project(manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(control_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(aruco_interfaces REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(database_service_pkg REQUIRED)
find_package(nlohmann_json REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(manipulation src/manipulation.cpp)


ament_target_dependencies(manipulation
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_visual_tools
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  control_msgs
  btcpp_ros2_interfaces
  aruco_interfaces
  franka_msgs
  database_service_pkg
  nlohmann_json
)

install(TARGETS
  manipulation
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
