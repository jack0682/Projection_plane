#!/usr/bin/env python3
"""
Launch file for AprilTag-based 6DOF box pose detection pipeline.

This launch file starts:
1. apriltag_ros detector (configured for tag36h11, size=0.083m)
2. realtime_detect node (transforms poses to map frame)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description."""

    # Launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Image topic from camera'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='CameraInfo topic from camera'
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='TF frame of camera optical center'
    )
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Target TF frame for output poses'
    )
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag family'
    )
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.083',
        description='AprilTag size in meters (edge length)'
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Enable debug logging'
    )

    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    map_frame = LaunchConfiguration('map_frame')
    tag_family = LaunchConfiguration('tag_family')
    tag_size = LaunchConfiguration('tag_size')
    debug = LaunchConfiguration('debug')

    # AprilTag ROS detector node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', image_topic),
            ('camera_info', camera_info_topic),
        ],
        parameters=[
            {
                'family': tag_family,
                'size': tag_size,
                'max_hamming': 0,
                'profile': 'quality',
                'nthreads': 4,
                'decimate': 1.0,
                'blur': 0,
                'refine_edges': 1,
                'refine_decode': 0,
                'refine_pose': 0,
                'debug': False,
                'publish_tf': False,
            }
        ],
        output='screen',
        namespace='',
    )

    # realtime_detect node
    realtime_detect_node = Node(
        package='realtime_detect',
        executable='apriltag_box_pose_node',
        name='apriltag_box_pose_node',
        parameters=[
            {
                'map_frame': map_frame,
                'camera_frame': camera_frame,
                'detection_topic': '/tag_detections',
                'publish_rate_hz': 20,
                'pose_topic': '/realtime_detect/box_poses',
                'marker_topic': '/realtime_detect/box_pose_markers',
                'tag_ids_whitelist': [-1],
                'tag_size_m': tag_size,
                'decision_margin_min': 0.0,
                'debug': debug,
            }
        ],
        output='screen',
    )

    launch_description = LaunchDescription(
        [
            image_topic_arg,
            camera_info_topic_arg,
            camera_frame_arg,
            map_frame_arg,
            tag_family_arg,
            tag_size_arg,
            debug_arg,
            apriltag_node,
            realtime_detect_node,
        ]
    )

    return launch_description
