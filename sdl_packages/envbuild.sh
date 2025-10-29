#!/bin/bash
set -e  # Exit on error
set -o pipefail

echo "Building sdl_ws..."
cd ~/sdl_ws

# Download dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build libfranka
export CMAKE_PREFIX_PATH=~/sdl_ws/src/franka/libfranka/build/lib/
colcon build --packages-select libfranka --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash

# Specific message packages
colcon build --packages-up-to franka_msgs btcpp_ros2_interfaces ot2_interfaces
source install/setup.bash

# Full build, use other command for lower memory
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
#colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1

source install/setup.bash

echo "All workspaces built successfully."