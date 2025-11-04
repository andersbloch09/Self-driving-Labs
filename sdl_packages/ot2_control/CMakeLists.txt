cmake_minimum_required(VERSION 3.5)
project(ot2_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


find_package(ot2_interfaces REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Make your Python node executable
install(PROGRAMS
  ot2_control/ot2_manager.py
  ot2_control/action_caller.py
  ot2_control/init_caller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
