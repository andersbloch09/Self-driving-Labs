from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('bt_server_pkg')
    
    return LaunchDescription([
        # MiR mission action server
        Node(
            package='bt_server_pkg',
            executable='mir_mission_action_server.py',
            name='mir_mission_action_server',
            output='screen'
        ),
        
        # OpenTrons2 Manager (action server)
        Node(
            package='ot2_control',
            executable='ot2_manager.py',
            name='ot2_manager',
            output='screen'
        ),
        
        # Franka MoveIt launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('franka_moveit_config'),
                    'launch',
                    'moveit.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_ip': '192.168.0.30',
                'load_gripper': 'true',
                'use_rviz': 'false'
            }.items()
        ),
        
        # Behavior tree server
        Node(
            package='bt_server_pkg', 
            executable='bt_server',
            name='bt_server',
            parameters=[
                os.path.join(package_dir, 'config', 'bt_executor.yaml')
            ],
            output='screen'
        )
    ])