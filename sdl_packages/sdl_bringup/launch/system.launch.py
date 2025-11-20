from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # -----------------------------
    # 1. FRANKA MOVEIT LAUNCH
    # -----------------------------
    franka_moveit_pkg = get_package_share_directory('franka_moveit_config')

    franka_moveit_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('franka_moveit_config'),
            'launch',
            'moveit.launch.py'
        )
    ),
        launch_arguments={
            'robot_ip': '192.168.0.30',
            'load_gripper': 'true'
        }.items()
    )

    # -----------------------------
    # 2. REALSENSE CAMERA LAUNCH
    # -----------------------------
    realsense_pkg = get_package_share_directory('realsense2_camera')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_sync': 'false',
            'pointcloud.enable': 'false'
        }.items()
    )

    # -----------------------------
    # 3. ARUCO DETECTOR NODE
    # -----------------------------
    aruco_node = Node(
        package='aruco_detector',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen'
    )

    return LaunchDescription([
        franka_moveit_launch,
        realsense_launch,
        aruco_node
    ])
