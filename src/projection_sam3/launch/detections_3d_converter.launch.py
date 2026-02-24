#!/usr/bin/env python3
"""Launch file for 3D detections converter node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    # Create 3D converter node (T6)
    detections_3d_converter_node = Node(
        package='projection_sam3',
        executable='detections_3d_converter',
        name='detections_3d_converter',
        output='screen',
    )

    # Launch description
    ld = LaunchDescription([
        detections_3d_converter_node,
    ])

    return ld
