#!/usr/bin/env python3
"""
Launch file for projection_plane ROS2 node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the package share directory path
    from ament_index_python.packages import get_package_share_directory
    package_share_dir = get_package_share_directory('projection_plane')

    # Default config path
    config_file = os.path.join(package_share_dir, 'config', 'projection_params.yaml')

    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )

    # Create node
    projection_plane_node = Node(
        package='projection_plane',
        executable='projection_plane_node',
        name='projection_plane_node',
        parameters=[LaunchConfiguration('config_file')],
        output='both',
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_config_file,
        projection_plane_node,
    ])
