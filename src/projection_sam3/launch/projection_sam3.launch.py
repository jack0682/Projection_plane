#!/usr/bin/env python3
"""Launch file for projection_sam3 node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""

    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/jack/ros2_ws/sam_3d_test/models/sam3.pt',
        description='Path to SAM3 model file'
    )

    max_fps_arg = DeclareLaunchArgument(
        'max_fps',
        default_value='2.0',
        description='Maximum inference FPS (throttle)'
    )

    # Create node
    projection_sam3_node = Node(
        package='projection_sam3',
        executable='projection_sam3_node',
        name='projection_sam3_node',
        parameters=[
            {'model_path': LaunchConfiguration('model_path')},
            {'max_fps': LaunchConfiguration('max_fps')},
        ],
        output='screen',
    )

    # Launch description
    ld = LaunchDescription([
        model_path_arg,
        max_fps_arg,
        projection_sam3_node,
    ])

    return ld
