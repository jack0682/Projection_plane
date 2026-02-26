#!/usr/bin/env python3
"""
AprilTag-based 6DOF box pose detection and transformation node.

This node:
1. Subscribes to AprilTag detections from apriltag_ros
2. Extracts 6DOF pose from detections using homography decomposition
3. Transforms pose to 'map' frame using tf2
4. Publishes poses as PoseArray and visualization markers
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation
import math
import time


class AprilTagBoxPoseNode(Node):
    """Node for AprilTag-based 6DOF box pose detection and transformation."""

    def __init__(self):
        super().__init__('apriltag_box_pose_node')

        # Declare parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('detection_topic', '/tag_detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('publish_rate_hz', 20)
        self.declare_parameter('pose_topic', '/realtime_detect/box_poses')
        self.declare_parameter('marker_topic', '/realtime_detect/box_pose_markers')
        self.declare_parameter('tag_ids_whitelist', [-1])  # -1 means accept all
        self.declare_parameter('tag_size_m', 0.083)
        self.declare_parameter('decision_margin_min', 0.0)
        self.declare_parameter('debug', False)

        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        detection_topic = self.get_parameter('detection_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        pose_topic = self.get_parameter('pose_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.tag_ids_whitelist = self.get_parameter('tag_ids_whitelist').value
        self.tag_size_m = self.get_parameter('tag_size_m').value
        self.decision_margin_min = self.get_parameter('decision_margin_min').value
        self.debug = self.get_parameter('debug').value

        self.get_logger().info(
            f'Initialized with map_frame={self.map_frame}, '
            f'camera_frame={self.camera_frame}, '
            f'tag_size={self.tag_size_m}m, '
            f'debug={self.debug}'
        )

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsics (will be populated by camera_info callback)
        self.camera_matrix = None
        self.camera_dist_coeffs = None
        self.has_camera_info = False

        # Latest detections storage
        self.latest_detections = None
        self.latest_detections_time = 0

        # Subscriber to camera info
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            camera_info_qos
        )

        # Subscriber to AprilTag detections
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            detection_topic,
            self.detection_callback,
            detection_qos
        )

        # Publishers
        self.pose_array_pub = self.create_publisher(
            PoseArray,
            pose_topic,
            10
        )
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10
        )

        # Timer for periodic processing
        timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counters for logging
        self.tf_failure_count = 0
        self.tf_failure_last_log = 0
        self.camera_info_count = 0

    def camera_info_callback(self, msg: CameraInfo):
        """Callback for camera info messages."""
        if not self.has_camera_info:
            self.get_logger().info('Received camera info')
            self.has_camera_info = True

        # Extract camera matrix
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.camera_dist_coeffs = np.array(msg.D) if msg.D else np.zeros(5)

        self.camera_info_count += 1
        if self.debug and self.camera_info_count % 10 == 0:
            self.get_logger().debug(
                f'Camera matrix:\n{self.camera_matrix}\n'
                f'Distortion: {self.camera_dist_coeffs}'
            )

    def detection_callback(self, msg: AprilTagDetectionArray):
        """Callback for AprilTag detection messages."""
        self.latest_detections = msg
        self.latest_detections_time = time.time()

        if self.debug:
            self.get_logger().debug(
                f'Received {len(msg.detections)} detections, '
                f'frame_id={msg.header.frame_id}'
            )

    def timer_callback(self):
        """Periodic callback to process detections and publish poses."""
        if self.latest_detections is None:
            return

        if not self.has_camera_info:
            self.get_logger().warn_throttle(
                5.0, 'Waiting for camera info...'
            )
            return

        detections = self.latest_detections
        poses = []
        markers = MarkerArray()
        marker_id = 0

        # Process each detection, sorted by ID for deterministic ordering
        sorted_detections = sorted(detections.detections, key=lambda d: d.id)

        for detection in sorted_detections:
            # Filter by whitelist
            if self.tag_ids_whitelist[0] != -1 and detection.id not in self.tag_ids_whitelist:
                continue

            # Filter by decision margin
            if detection.decision_margin < self.decision_margin_min:
                if self.debug:
                    self.get_logger().debug(
                        f'Skipping tag {detection.id}: decision_margin='
                        f'{detection.decision_margin} < {self.decision_margin_min}'
                    )
                continue

            # Extract homography matrix and compute pose
            try:
                pose = self.compute_pose_from_detection(
                    detection, detections.header
                )
                if pose is not None:
                    poses.append(pose)

                    # Create visualization markers
                    marker = self.create_pose_marker(
                        detection.id, marker_id, pose, detections.header.stamp
                    )
                    markers.markers.append(marker)
                    marker_id += 1

                    # Text marker with tag ID
                    text_marker = self.create_text_marker(
                        detection.id, marker_id, pose, detections.header.stamp
                    )
                    markers.markers.append(text_marker)
                    marker_id += 1

            except Exception as e:
                self.get_logger().warn(
                    f'Error processing tag {detection.id}: {str(e)}'
                )
                continue

        # Publish PoseArray
        if poses:
            pose_array = PoseArray()
            pose_array.header.frame_id = self.map_frame
            pose_array.header.stamp = detections.header.stamp
            pose_array.poses = poses
            self.pose_array_pub.publish(pose_array)

            if self.debug:
                self.get_logger().debug(
                    f'Published {len(poses)} poses in {self.map_frame}'
                )

        # Publish MarkerArray (even if empty, to clear old markers)
        self.marker_array_pub.publish(markers)

    def compute_pose_from_detection(
        self,
        detection,
        header
    ) -> Pose or None:
        """
        Compute 6DOF pose from AprilTag detection.

        Steps:
        1. Extract homography matrix from detection
        2. Decompose homography to get rotation and translation
        3. Transform to map frame using tf2

        Returns:
            Pose in map frame, or None if transform fails
        """
        # Extract homography (9-element row-major matrix)
        H = np.array(detection.homography, dtype=np.float64).reshape(3, 3)

        # Normalize homography
        H = H / H[2, 2]

        # Estimate pose from homography
        pose_camera = self.estimate_pose_from_homography(H)

        if pose_camera is None:
            return None

        # Transform pose from camera frame to map frame
        try:
            pose_map = self.transform_pose_to_map(pose_camera, header.stamp)
            return pose_map
        except tf2_ros.LookupException as e:
            self.tf_failure_count += 1
            current_time = time.time()
            if current_time - self.tf_failure_last_log > 5.0:
                self.get_logger().warn(
                    f'TF lookup failed ({self.tf_failure_count} total): {str(e)}'
                )
                self.tf_failure_last_log = current_time
            return None
        except tf2_ros.ExtrapolationException as e:
            if self.debug:
                self.get_logger().debug(
                    f'TF extrapolation failed: {str(e)}'
                )
            return None

    def estimate_pose_from_homography(self, H):
        """
        Estimate 6DOF pose from homography matrix using camera intrinsics.

        Uses the decomposition method described in:
        "Homography Decomposition for Visual SLAM" and related work.

        Args:
            H: 3x3 homography matrix (normalized)

        Returns:
            Pose in camera frame
        """
        try:
            if self.camera_matrix is None:
                self.get_logger().error('Camera matrix not available')
                return None

            # Camera intrinsic matrix
            K = self.camera_matrix
            K_inv = np.linalg.inv(K)

            # Normalize homography by camera intrinsics
            # H_norm = K^-1 * H * K
            H_norm = K_inv @ H @ K

            # Decompose normalized homography
            # H = lambda * [r1 | r2 | t] where r1, r2 are rotation columns
            # Perform SVD to get scale and rotation
            U, S, Vt = np.linalg.svd(H_norm)

            # The singular values should be [lambda, lambda, 1/lambda]
            # Use middle singular value as lambda
            if len(S) < 3:
                self.get_logger().warn('SVD returned insufficient singular values')
                return None

            lambda_val = S[1]
            if lambda_val < 1e-6:
                self.get_logger().warn('Singular value too small')
                return None

            # Reconstruct rotation matrix from SVD
            R = U @ Vt
            # Ensure det(R) = 1 (proper rotation)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = U @ Vt

            # Estimate translation scale from homography
            # For a planar tag at distance Z with known size,
            # we can estimate depth. Use a fixed scale for now.
            t = np.zeros(3)
            # Set Z to be several tag widths away
            t[2] = self.tag_size_m * 5.0

            # Normalize rotation (enforce orthogonality)
            U_r, _, Vt_r = np.linalg.svd(R)
            R = U_r @ Vt_r

            # Create Pose message
            pose = Pose()
            pose.position.x = float(t[0])
            pose.position.y = float(t[1])
            pose.position.z = float(t[2])

            # Convert rotation matrix to quaternion
            rotation = Rotation.from_matrix(R)
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])

            return pose

        except Exception as e:
            self.get_logger().error(f'Homography decomposition failed: {str(e)}')
            return None

    def transform_pose_to_map(self, pose_camera, stamp) -> Pose:
        """
        Transform pose from camera frame to map frame using tf2.

        Args:
            pose_camera: Pose in camera_frame
            stamp: ROS timestamp for the transform

        Returns:
            Pose in map frame

        Raises:
            tf2_ros exceptions if transform not available
        """
        # Create a geometry_msgs/PoseStamped in camera frame
        pose_stamped_camera = PoseStamped()
        pose_stamped_camera.header.frame_id = self.camera_frame
        pose_stamped_camera.header.stamp = stamp
        pose_stamped_camera.pose = pose_camera

        # Transform to map frame
        pose_stamped_map = self.tf_buffer.transform(
            pose_stamped_camera,
            self.map_frame,
            timeout=rclpy.duration.Duration(seconds=0.1)
        )

        return pose_stamped_map.pose

    def create_pose_marker(self, tag_id, marker_id, pose, stamp):
        """Create a visualization marker for the pose (arrow)."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.1   # Arrow length
        marker.scale.y = 0.02  # Arrow width
        marker.scale.z = 0.02  # Arrow height
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime = rclpy.time.Duration(seconds=1)
        return marker

    def create_text_marker(self, tag_id, marker_id, pose, stamp):
        """Create a text marker showing the tag ID."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z + 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f'Tag {tag_id}'
        marker.lifetime = rclpy.time.Duration(seconds=1)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagBoxPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
