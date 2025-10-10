from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('bt_server_pkg')
    
    # Launch configuration
    groot2_port_arg = DeclareLaunchArgument(
        'groot2_port',
        default_value='1666',
        description='Port for Groot2 monitoring connection'
    )
    
    return LaunchDescription([
        groot2_port_arg,
        
        # MiR mission action server
        Node(
            package='bt_server_pkg',
            executable='mir_mission_action_server.py',
            name='mir_mission_action_server',
            output='screen'
        ),
        
        # Behavior tree server with Groot2 monitoring
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