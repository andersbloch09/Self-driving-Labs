# Self-Driving Labs

This repository contains a project for self-driving labs, focusing on automated laboratory systems and intelligent experimentation workflows.

This project uses a `.env.example` file to define required environment variables. To run the application, copy it to `.env` and fill in your own values.


Once ready to build the devcontainer image, run:
```bash
docker buildx build -f .devcontainer/Dockerfile.sdl -t sdl_dev .
```

To build the added packages from the web: 
```bash
vcs import src/franka < src/franka.repos --recursive --skip-existing
```

After downloading the packages, follow the instructions from https://github.com/tenfoldpaper/multipanda_ros2
When trying to build libfranka delete the build folder first and then afterwards run with 
```bash
sudo cmake --install .
```
inside the new build folder. 


```bash
export CMAKE_PREFIX_PATH=~/sdl_ws/src/franka/mujoco_install/lib/cmake:~/sdl_ws/src/franka/libfranka/build/lib/
```

If libfranka fails try: 
```bash 
colcon build --packages-select libfranka --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

Download dependcies: 
```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

After this run the following line from the source directory: 
```bash 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
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

