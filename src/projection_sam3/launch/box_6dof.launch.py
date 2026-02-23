#!/usr/bin/env python3
"""Launch file for box 6DOF pose extraction node."""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for 6DOF node."""

    # Declare launch arguments
    ply_path_arg = DeclareLaunchArgument(
        'ply_path',
        default_value='/home/jack/Last_point/pcd_file/241108_converted - Cloud.ply',
        description='Path to point cloud PLY file'
    )

    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera image width (pixels)'
    )

    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera image height (pixels)'
    )

    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='6.0',
        description='Maximum depth for estimation (D455 spec: 6.0m)'
    )

    search_radius_arg = DeclareLaunchArgument(
        'search_radius',
        default_value='1.5',
        description='KD-tree search radius around camera origin (meters)'
    )

    # Create 6DOF node
    box_6dof_node = Node(
        package='projection_sam3',
        executable='box_6dof_node',
        name='box_6dof_node',
        parameters=[
            {'ply_path': LaunchConfiguration('ply_path')},
            {'camera_width': LaunchConfiguration('camera_width')},
            {'camera_height': LaunchConfiguration('camera_height')},
            {'max_depth': LaunchConfiguration('max_depth')},
            {'search_radius': LaunchConfiguration('search_radius')},
        ],
        output='screen',
    )

    # Launch description
    ld = LaunchDescription([
        ply_path_arg,
        camera_width_arg,
        camera_height_arg,
        max_depth_arg,
        search_radius_arg,
        box_6dof_node,
    ])

    return ld
