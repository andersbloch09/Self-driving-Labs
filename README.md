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

# Starting, stopping and transferring data
This project runs a containerized Supabase setup to track the positions of vessels and reactants throughout the lab. Below is listed some of the essential commands needed to use it:

Start Supabase services
```bash
cd supabase
bash start.sh
```

Stop Supabase database using:
```bash
cd supabase
bash stop.sh
```

The contents of the database are not automatically synchronized with git, so in order to preserve the data from instance to instance, it is necessary to dump the contents into a seed file that git can track. With the supabase containers running, run the following command in root directory of the repository where the supabase files are located.
```bash
docker exec -t supabase-db pg_dump \
  -U supabase_admin \
  -d postgres \
  -n public \
  --column-inserts \
  > supabase/seed.sql
```
This should create a seed.sql file that is commitable through git.

When you need to load the contents of the seed file into a different instance of supabase, you have to run this command from the root directory of the repository. Ensure that the containers are running.
```bash
docker exec -i supabase-db psql -U supabase_admin -d postgres < supabase/seed.sql
```

To drop the database first run: 
```bash
docker exec -i supabase-db psql -U supabase_admin -d postgres \
-c "DROP SCHEMA public CASCADE; CREATE SCHEMA public; GRANT ALL ON SCHEMA public TO supabase_admin; GRANT ALL ON SCHEMA public to public;" \
&& docker exec -i supabase-db psql -U supabase_admin -d postgres < supabase/seed.sql
```

# Test database interaction functions
Start by ensuring that POOLER_TENANT_ID is set to "mytenant" in your .env file, and that the supabase docker has been started. This can be done by running start.sh in the supabase directory or you can check whether it is already running by executing:
```bash
docker ps
```

Then build the database_service_pkg with the following command:
```bash
colcon build --packages-select database_service_pkg
```
Then run the node with:
```bash
source install/setup.bash && ros2 run database_service_pkg interactive_database_test
```
There are instructions printed in the terminal for usage of the testing tool, but it shows how the general implementation works.
It is recommended to open the database og http://localhost:8000 to monitor how the rows are manipulated through the functions.

### Examples

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
The folder bt_server_pkg/behavior_trees is loaded in its entirety, and each behavior tree can be referenced as the target tree in the command above. The alias used for each tree should be found in the individual files. For instance: <BehaviorTree ID="BT_Test">

# Running the Franka
In order to run the franca, first ensure that the the desired host machine is connected to the manipulator through the ethernet cable. The host machine will have to be running the Real Time Kernel, a guide for which can be found at: https://frankarobotics.github.io/docs/libfranka/docs/installation_linux.html#setting-up-the-real-time-kernel

Ensure that the host machine has booted using the correct kernel, either by default or by choosing it in the GNU bootloader.
Then, on your own machine (not the host machine), run the following command to forward the franca control interface to be accessible from your own machine. Replace sdl@192.168.1.42 with the username and IP of the host machine connected to the franca.

```bash
ssh -L 8888:192.168.0.30:443 sdl@192.168.1.42
```
This SSH session should be running at all times.
Then open https://robot.franka.de:8888/desk/ using google chrome, ignoring the warnings and proceeding to the site. Run homing of the manipulator and activate FCI in the dropdown menu in the top right corner. This enables movements triggered by ROS.



```bash
ros2 action send_goal /bt_execution \
btcpp_ros2_interfaces/action/ExecuteTree \
"{target_tree: 'MaterialTransport', \
  payload: '{ \
     \"materials\": [\"FehlingsSolution\", \"GlucoseSolution\", \"PipetteTips\"], \
     \"destination_map\": { \
        \"FehlingsSolution\": \"storage_ot2\", \
        \"GlucoseSolution\": \"storage_ot2\", \
        \"PipetteTips\": \"storage_ot2\" \
     }, \
     \"mission_map\": { \
        \"storage_jig_A\": \"94c9f0cf-a4f7-11f0-b2e5-000e8e984489\", \
        \"storage_ot2\": \"a1b2c3d4-e5f6-7890-1234-567890abcdef\", \
        \"storage_mir\": \"76638485-a4f7-11f0-b2e5-000e8e984489\" \
     }, \
     \"transport_storage\": \"storage_mir\" \
  }' \
}" --feedback
```