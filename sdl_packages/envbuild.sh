#!/bin/bash
set -e  # Exit on error
set -o pipefail

do
  echo "Building sdl_ws..."
  cd ~/sdl_ws
  rosdep update
  rosdep install --from-paths src --ignore-src --rosdistro humble -y
  
  colcon build --merge-install
  source install/setup.bash
done

echo "All workspaces built successfully."
