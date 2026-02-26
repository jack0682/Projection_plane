#!/usr/bin/env python3
"""
Launch file for detections_6dof_converter node
Extracts 6DOF poses from 2D detections using ray-casting + PCA
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 6DOF converter"""

    # Node: detections_6dof_converter
    detections_6dof_node = Node(
        package='projection_sam3',
        executable='detections_6dof_converter',
        name='detections_6dof_converter',
        output='screen',
        parameters=[
            {
                'max_queue_size': 10,
                'ray_k_neighbors': 100,
                'ray_tolerance_m': 0.05,
                'inlier_threshold_m': 0.1,
                'min_points_for_pca': 5,
                'csv_output_dir': '',  # Empty = auto-detect latest predict dir
            }
        ],
        remappings=[
            # Input topics (using actual topic names)
            ('/projection/cloud_raw', '/projection/cloud_raw'),
            ('/projection/contract', '/projection/contract'),
            ('/projection/sam3/detections', '/projection/sam3/detections'),
            # Output topic
            ('/projection/detections_6dof', '/projection/detections_6dof'),
        ]
    )

    return LaunchDescription([
        detections_6dof_node,
    ])


if __name__ == '__main__':
    # Allow running directly for testing
    import sys
    try:
        from launch.launch_service import LaunchService
        service = LaunchService()
        service.include_launch_description(generate_launch_description())
        sys.exit(service.run())
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
