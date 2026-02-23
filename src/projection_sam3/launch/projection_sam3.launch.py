#!/usr/bin/env python3
"""Launch file for projection_sam3 node and tracker."""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""

    # Declare launch arguments for detector
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

    # Declare launch arguments for tracker
    enable_tracker_arg = DeclareLaunchArgument(
        'enable_tracker',
        default_value='true',
        description='Enable tracking node'
    )

    tracker_min_hits_arg = DeclareLaunchArgument(
        'tracker_min_hits',
        default_value='2',
        description='Minimum hits to confirm track'
    )

    tracker_ttl_arg = DeclareLaunchArgument(
        'tracker_ttl_sec',
        default_value='2.0',
        description='Track time-to-live (seconds)'
    )

    tracker_max_dist_arg = DeclareLaunchArgument(
        'tracker_max_3d_distance',
        default_value='0.25',
        description='Max 3D distance for association (meters)'
    )

    tracker_merge_radius_arg = DeclareLaunchArgument(
        'tracker_merge_radius_3d',
        default_value='0.2',
        description='3D merge radius for inventory (meters)'
    )

    # Create detector node
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

    # Create tracker node (optional)
    projection_tracker_node = Node(
        package='projection_sam3',
        executable='projection_sam3_tracker_node',
        name='projection_tracker_node',
        parameters=[
            {'min_hits': LaunchConfiguration('tracker_min_hits')},
            {'ttl_sec': LaunchConfiguration('tracker_ttl_sec')},
            {'max_3d_distance': LaunchConfiguration('tracker_max_3d_distance')},
            {'merge_radius_3d': LaunchConfiguration('tracker_merge_radius_3d')},
            {'ema_alpha': 0.3},
            {'allow_unconfirmed_output': False},
        ],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_tracker')),
    )

    # Launch description
    ld = LaunchDescription([
        model_path_arg,
        max_fps_arg,
        enable_tracker_arg,
        tracker_min_hits_arg,
        tracker_ttl_arg,
        tracker_max_dist_arg,
        tracker_merge_radius_arg,
        projection_sam3_node,
        projection_tracker_node,
    ])

    return ld
