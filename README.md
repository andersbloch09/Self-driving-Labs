# Self-Driving Labs

This repository contains a project for self-driving labs, focusing on automated laboratory systems and intelligent experimentation workflows.

This project uses a `.env.example` file to define required environment variables. To run the application, copy it to `.env` and fill in your own values.


Once ready to build the devcontainer image, use visual studio code and run the folder with the container library or run:
```bash
docker buildx build -f .devcontainer/Dockerfile.sdl -t sdl_dev .
```

To build the added packages from the web: 
```bash
vcs import src/franka < src/franka.repos --recursive --skip-existing
```
For to full install libfranka follow instructions from https://github.com/tenfoldpaper/multipanda_ros2 though this should not be needed.  

To build the entire workspace, run: 
```bash 
cd ~/sdl/src
bash envbuild.sh
```

For manual build do: 

Download dependcies: 
```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

```bash
export CMAKE_PREFIX_PATH=~/sdl_ws/src/franka/libfranka/build/lib/
colcon build --packages-select libfranka --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash
```

After this run the following one of the following line from the outer directory: 
```bash 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
For lower memory: 
```bash 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### Database Setup

To manage the Supabase database:

```bash
# Start Supabase services
cd supabase
bash start.sh

# Stop Supabase services
cd supabase
bash stop.sh
```


# Mir BT example execution:

Start by building the following packages using this command. The order is important:

```bash
colcon build --packages-select btcpp_ros2_interfaces behaviortree_ros2 mir bt_server_pkg
```

```bash
source install/setup.bash && ros2 launch bt_server_pkg bt_server.launch.py
```

Make sure that the MiR is started, has sufficient battery life and is set to "Ready" with a green light emitting from the base. Make sure that the mir_api.py is configured correctly with authorization headers, and that the correct IP address is in Mir_mission_action_server.py

```bash
source install/setup.bash && ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'BT_Test'}" --feedback
```

# Test database interaction functions

Start by ensuring that the supabase docker has been started by running start.sh in the supabase directory

Then build the database_service_pkg with the following command:
```bash
colcon build --packages-select database_service_pkg
```

Then run the node with:
```bash
source install/setup.bash && ros2 run database_service_pkg interactive_database_test
```

There are instructions printed in the terminal for usage of the testing tool, but it shows how the general implementation.
It is recommended to open the database og http://localhost:8000 to monitor how the rows are manipulated through the functions.