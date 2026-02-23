#!/usr/bin/env python3
"""
Phase 5: 6DOF Pose Extraction Node for ROS2
Integrates Phases 1-4 to extract complete 6DOF poses (position + orientation + size)
for all detected boxes.

Data Flow:
  Phase 1 Output (Confidence Maps) + Phase 2 (Weighted Centers)
    ↓
  Phase 3 (Camera Ray Directions)
    ↓
  Phase 4 (Depth Estimation)
    ↓
  Phase 5 (6DOF: Position + Orientation + Size)
    ↓
  /projection/sam3/box_6dof (ROS2 Pose Array)
  CSV Log: runs/segment/predictN/box_6dof.csv
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import threading
import time
import csv
import os
from pathlib import Path

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String, Float64MultiArray
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

# Import Phase 2-4 components
from .mask_analysis import get_weighted_mask_center, estimate_mask_statistics
from .camera_model import D455CameraModel
from .depth_estimation import DepthEstimator, load_point_cloud_ply


class Box6DOFNode(Node):
    """
    Extract complete 6DOF poses for detected boxes.

    Input: Detection masks from Phase 2
    Output: 6DOF poses (position + orientation + size)

    Processing:
      1. Get weighted mask center (Phase 2)
      2. Convert to camera ray direction (Phase 3)
      3. Estimate depth using point cloud (Phase 4)
      4. Estimate orientation from mask PCA
      5. Estimate size from mask area + depth
      6. Publish 6DOF pose
      7. Log to CSV
    """

    def __init__(self):
        super().__init__('box_6dof_node')

        # Declare parameters
        self.declare_parameter('model_path', '/home/jack/ros2_ws/sam_3d_test/models/sam3.pt')
        self.declare_parameter('ply_path', '/home/jack/Last_point/pcd_file/241108_converted - Cloud.ply')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('max_depth', 6.0)
        self.declare_parameter('search_radius', 0.5)

        # Get parameters
        self.ply_path = self.get_parameter('ply_path').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.max_depth = self.get_parameter('max_depth').value
        self.search_radius = self.get_parameter('search_radius').value

        self.bridge = CvBridge()

        # Initialize camera model (Phase 3)
        self.get_logger().info('[Phase5] Initializing D455 camera model...')
        self.camera_model = D455CameraModel(
            image_width=self.camera_width,
            image_height=self.camera_height
        )

        # Load point cloud and initialize depth estimator (Phase 4)
        self.get_logger().info('[Phase5] Loading point cloud for depth estimation...')
        try:
            point_cloud = load_point_cloud_ply(self.ply_path)
            self.depth_estimator = DepthEstimator(point_cloud, max_depth=self.max_depth)
            self.get_logger().info(f'[Phase5] ✓ Depth estimator ready ({len(point_cloud):,} points)')
        except Exception as e:
            self.get_logger().error(f'[Phase5] Failed to load point cloud: {e}')
            self.depth_estimator = None

        # Camera pose (will be updated from /projection/camera_pose)
        self.camera_origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.R_cam_to_world = np.eye(3, dtype=np.float32)
        self.t_cam_to_world = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # CSV logging
        self.csv_base_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws/runs/segment')
        self.csv_path = None

        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_box_6dof = self.create_publisher(PoseArray, '/projection/sam3/box_6dof', qos)
        self.pub_debug = self.create_publisher(String, '/projection/sam3/box_6dof_debug', qos)

        # Frame counter for logging
        self.frame_count = 0

        # Subscriptions
        self.create_subscription(
            Detection2DArray,
            '/projection/sam3/detections',
            self._detections_callback,
            qos
        )

        self.create_subscription(
            Float64MultiArray,
            '/projection/camera_pose',
            self._camera_pose_callback,
            qos
        )

        self.get_logger().info('[Phase5] Box6DOFNode initialized ✓')

    def _camera_pose_callback(self, msg):
        """Update camera pose from /projection/camera_pose"""
        try:
            if len(msg.data) >= 12:
                # Assume format: [R11, R12, R13, R21, R22, R23, R31, R32, R33, tx, ty, tz]
                R_flat = msg.data[:9]
                t = msg.data[9:12]
                self.R_cam_to_world = np.array(R_flat, dtype=np.float32).reshape(3, 3)
                self.t_cam_to_world = np.array(t, dtype=np.float32)
                self.camera_origin = self.t_cam_to_world.copy()
        except Exception as e:
            self.get_logger().debug(f'[Phase5] Could not parse camera pose: {e}')

    def _detections_callback(self, msg):
        """Process detections and extract 6DOF poses"""
        if not msg.detections or self.depth_estimator is None:
            return

        self.frame_count += 1

        poses_array = PoseArray()
        poses_array.header = msg.header

        box_6dof_data = []

        for idx, det in enumerate(msg.detections):
            try:
                # Extract detection info from bounding box
                bbox = det.bbox

                # Phase 2: Get center from bounding box
                # Since Detection2D doesn't include mask, use bbox center instead
                center_x = bbox.center.position.x
                center_y = bbox.center.position.y

                # Get confidence from detection results
                mean_conf = 0.5  # Default
                if det.results:
                    mean_conf = float(det.results[0].hypothesis.score)

                # Validate center
                if center_x is None or center_y is None:
                    self.get_logger().debug(f'[Phase5] Box {idx}: Invalid bbox center')
                    continue

                # Phase 3: Camera ray direction
                direction_cam = self.camera_model.pixel_to_camera_direction(center_x, center_y)

                # Phase 4: Depth estimation
                depth, dist_from_ray = self.depth_estimator.estimate_depth(
                    self.camera_origin,
                    direction_cam,
                    self.search_radius
                )

                # Phase 5: Compute 6DOF pose
                # Position
                point_camera = direction_cam * depth
                point_world = self.camera_model.camera_to_world(
                    point_camera, self.R_cam_to_world, self.t_cam_to_world
                )

                # Orientation (from bbox aspect ratio)
                # Since Detection2D doesn't include mask, use simple aspect ratio-based orientation
                orientation_quat = self._estimate_orientation_from_bbox(bbox, direction_cam)

                # Size (from bbox + depth)
                size_3d = self._estimate_size_from_bbox_and_depth(bbox, depth)

                # Create Pose message
                pose = Pose()
                pose.position = Point(
                    x=float(point_world[0]),
                    y=float(point_world[1]),
                    z=float(point_world[2])
                )
                pose.orientation = Quaternion(
                    x=float(orientation_quat[0]),
                    y=float(orientation_quat[1]),
                    z=float(orientation_quat[2]),
                    w=float(orientation_quat[3])
                )

                poses_array.poses.append(pose)

                # Log data
                box_6dof_data.append({
                    'box_id': idx,
                    'pos_x': float(point_world[0]),
                    'pos_y': float(point_world[1]),
                    'pos_z': float(point_world[2]),
                    'quat_x': float(orientation_quat[0]),
                    'quat_y': float(orientation_quat[1]),
                    'quat_z': float(orientation_quat[2]),
                    'quat_w': float(orientation_quat[3]),
                    'size_x': float(size_3d[0]),
                    'size_y': float(size_3d[1]),
                    'size_z': float(size_3d[2]),
                    'depth': float(depth),
                    'mask_center_x': float(center_x),
                    'mask_center_y': float(center_y),
                    'confidence': float(mean_conf)
                })

            except Exception as e:
                self.get_logger().error(f'[Phase5] Box {idx} processing failed: {e}')
                continue

        # Publish 6DOF poses
        if poses_array.poses:
            self.pub_box_6dof.publish(poses_array)

            # Save to CSV
            self._save_to_csv(box_6dof_data, msg.header.stamp)

            # Debug output
            debug_msg = String()
            debug_msg.data = f'[Phase5] Extracted 6DOF for {len(poses_array.poses)}/{len(msg.detections)} boxes'
            self.pub_debug.publish(debug_msg)

            self.get_logger().info(
                f'[Phase5] Frame {self.frame_count}: {len(poses_array.poses)} 6DOF poses extracted'
            )

    def _estimate_orientation_from_bbox(self, bbox, camera_direction):
        """
        Estimate orientation from bbox aspect ratio.

        Since Detection2D doesn't include mask data, use bounding box dimensions
        to estimate a simple orientation (Z-axis rotation only).

        Args:
          bbox: Detection2D bbox with size_x, size_y
          camera_direction: Camera ray direction (not used for bbox, but for compatibility)

        Returns: quaternion [qx, qy, qz, qw]
        """
        try:
            # Compute angle from bbox aspect ratio
            aspect_ratio = bbox.size_y / (bbox.size_x + 1e-6)  # Avoid division by zero

            # If width > height, object is horizontal (angle ≈ 0)
            # If height > width, object is vertical (angle ≈ π/2)
            if bbox.size_x > bbox.size_y:
                angle = 0.0  # Horizontal
            else:
                angle = np.pi / 4  # 45° for vertical

            # Convert angle to quaternion (Z-axis rotation)
            half_angle = angle / 2.0
            quat = np.array([
                0.0,
                0.0,
                np.sin(half_angle),
                np.cos(half_angle)
            ], dtype=np.float32)

            return quat

        except Exception as e:
            # Fallback
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    def _estimate_orientation_from_mask(self, mask, camera_direction):
        """
        Estimate orientation from mask using PCA.

        Simple approach: Use principal axes of mask to estimate 2D orientation,
        then lift to 3D using camera ray direction.

        Returns: quaternion [qx, qy, qz, qw]
        """
        try:
            y_coords, x_coords = np.where(mask > 0)
            if len(x_coords) < 3:
                # Fallback: align with camera direction
                return self._direction_to_quaternion(camera_direction)

            # Compute covariance in 2D mask space
            points_2d = np.column_stack([x_coords, y_coords]).astype(np.float32)
            mean_2d = points_2d.mean(axis=0)
            centered = points_2d - mean_2d

            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            # Primary axis (largest eigenvalue)
            primary_axis_2d = eigenvectors[:, 1]  # Column for largest eigenvalue

            # Lift to 3D: project to camera direction plane
            # This is a simplified approach
            angle = np.arctan2(primary_axis_2d[1], primary_axis_2d[0])
            quat = self._angle_to_quaternion_around_z(angle)

            return quat

        except Exception as e:
            # Fallback
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    def _estimate_size_from_bbox_and_depth(self, bbox, depth):
        """
        Estimate 3D size from bounding box and depth.

        Approach:
          - BBox width/height in pixels
          - Pixel size at depth D using camera FOV
          - Size in 3D: [width_m, height_m, depth_scale_m]

        Args:
          bbox: Detection2D bbox object with size_x, size_y (pixel dimensions)
          depth: Depth value in meters

        Returns: [size_x, size_y, size_z] in meters
        """
        try:
            if bbox.size_x <= 0 or bbox.size_y <= 0:
                return np.array([0.0, 0.0, 0.0], dtype=np.float32)

            width_px = bbox.size_x
            height_px = bbox.size_y

            # Compute FOV at depth
            fov_width_m, fov_height_m = self.camera_model.compute_fov_at_distance(depth)

            # Scale to bbox size
            size_x = (width_px / self.camera_width) * fov_width_m
            size_y = (height_px / self.camera_height) * fov_height_m
            size_z = depth * 0.1  # Rough estimate: 10% of depth as z-extent

            return np.array([size_x, size_y, size_z], dtype=np.float32)

        except Exception as e:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)

    @staticmethod
    def _direction_to_quaternion(direction):
        """Convert 3D direction vector to quaternion"""
        direction = direction / np.linalg.norm(direction)
        z_axis = np.array([0.0, 0.0, 1.0])

        # Compute rotation from z-axis to direction
        if np.allclose(direction, z_axis):
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        elif np.allclose(direction, -z_axis):
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        axis = np.cross(z_axis, direction)
        axis = axis / np.linalg.norm(axis)
        angle = np.arccos(np.dot(z_axis, direction))

        # Axis-angle to quaternion
        half_angle = angle / 2.0
        quat = np.array([
            axis[0] * np.sin(half_angle),
            axis[1] * np.sin(half_angle),
            axis[2] * np.sin(half_angle),
            np.cos(half_angle)
        ], dtype=np.float32)

        return quat

    @staticmethod
    def _angle_to_quaternion_around_z(angle):
        """Convert 2D angle (rotation around Z) to quaternion"""
        half_angle = angle / 2.0
        return np.array([
            0.0,
            0.0,
            np.sin(half_angle),
            np.cos(half_angle)
        ], dtype=np.float32)

    def _save_to_csv(self, box_data, timestamp):
        """Save 6DOF data to CSV"""
        if not box_data:
            return

        try:
            # Find or create CSV file
            if self.csv_path is None:
                self.csv_path = self._get_or_create_csv_path()

            # Append rows
            with open(self.csv_path, 'a', newline='') as f:
                fieldnames = [
                    'timestamp', 'box_id', 'pos_x', 'pos_y', 'pos_z',
                    'quat_x', 'quat_y', 'quat_z', 'quat_w',
                    'size_x', 'size_y', 'size_z', 'depth',
                    'mask_center_x', 'mask_center_y', 'confidence'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)

                for box in box_data:
                    row = {'timestamp': timestamp.sec + timestamp.nanosec / 1e9}
                    row.update(box)
                    writer.writerow(row)

        except Exception as e:
            self.get_logger().error(f'[Phase5] CSV save failed: {e}')

    def _get_or_create_csv_path(self):
        """Find latest predictN folder and create box_6dof.csv"""
        try:
            Path(self.csv_base_dir).mkdir(parents=True, exist_ok=True)

            # Find latest predictN folder
            predict_dirs = sorted([
                d for d in os.listdir(self.csv_base_dir)
                if d.startswith('predict')
            ])

            if predict_dirs:
                latest_dir = os.path.join(self.csv_base_dir, predict_dirs[-1])
            else:
                latest_dir = os.path.join(self.csv_base_dir, 'predict')
                Path(latest_dir).mkdir(parents=True, exist_ok=True)

            csv_path = os.path.join(latest_dir, 'box_6dof.csv')

            # Write header if new file
            if not os.path.exists(csv_path):
                with open(csv_path, 'w', newline='') as f:
                    fieldnames = [
                        'timestamp', 'box_id', 'pos_x', 'pos_y', 'pos_z',
                        'quat_x', 'quat_y', 'quat_z', 'quat_w',
                        'size_x', 'size_y', 'size_z', 'depth',
                        'mask_center_x', 'mask_center_y', 'confidence'
                    ]
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()

            self.get_logger().info(f'[Phase5] CSV path: {csv_path}')
            return csv_path

        except Exception as e:
            self.get_logger().error(f'[Phase5] Failed to create CSV path: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = Box6DOFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
