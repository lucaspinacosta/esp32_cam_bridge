#!/usr/bin/env python3
"""Launch the ESP32 camera bridge and human follower nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the camera follow stack."""
    bridge_config = LaunchConfiguration('bridge_config')
    follower_config = LaunchConfiguration('follower_config')

    default_bridge_config = PathJoinSubstitution([
        FindPackageShare('esp32_cam_bridge'),
        'config',
        'esp32_cam_bridge.yaml',
    ])
    default_follower_config = PathJoinSubstitution([
        FindPackageShare('esp32_cam_bridge'),
        'config',
        'human_follower.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_config',
            default_value=default_bridge_config,
            description='Path to the ESP32 camera bridge parameter file.',
        ),
        DeclareLaunchArgument(
            'follower_config',
            default_value=default_follower_config,
            description='Path to the human follower parameter file.',
        ),
        Node(
            package='esp32_cam_bridge',
            executable='esp32_cam_bridge',
            name='esp32_cam_bridge',
            output='screen',
            parameters=[bridge_config],
        ),
        Node(
            package='esp32_cam_bridge',
            executable='human_follower',
            name='human_follower',
            output='screen',
            parameters=[follower_config],
        ),
    ])
