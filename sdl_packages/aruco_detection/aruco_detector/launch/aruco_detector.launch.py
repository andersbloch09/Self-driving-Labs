#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            name='aruco_detector_node',
            output='screen',
            parameters=[],
            remappings=[
                # Uncomment and adjust if your camera topic is different
                # ('/camera/camera/color/image_raw', '/your/camera/topic'),
            ]
        )
    ])
