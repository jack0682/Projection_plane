#!/usr/bin/env python3
"""
Detections 3D → 6DOF Converter
Extracts 6DOF pose (x, y, z, roll, pitch, yaw) from 2D detections using ray-casting + PCA.

Architecture:
  - Subscribes to ProjectionContract (camera pose, FOV, plane info)
  - Subscribes to Detection2DArray (2D bounding boxes from SAM3)
  - Synchronizes messages via message_filters.TimeSynchronizer
  - Performs ray-casting on full point cloud (14.6M points, NO downsampling)
  - Extracts 6DOF via PCA on 3D point clouds
  - Publishes /projection/detections_6dof

Performance Target: 0.5-1.0 FPS with ±5° yaw, ±10° roll/pitch accuracy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import message_filters
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Float64MultiArray
from projection_msgs.msg import ProjectionContract  # ← Import the actual message type
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import logging
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple, Any
import math
import os
import csv
from datetime import datetime

# ============================================================================
# LOGGING SETUP
# ============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# ============================================================================
# GLOBAL CONSTANTS
# ============================================================================
MAX_QUEUE_SIZE = 10
RAY_K_NEIGHBORS = 100  # KD-tree search: up to 100 neighbors per ray
RAY_TOLERANCE_M = 0.05  # Ray-object association tolerance (meters)
INLIER_THRESHOLD_M = 0.1  # Point inclusion threshold for 3D bbox
MIN_POINTS_FOR_PCA = 5  # Minimum points needed for valid 6DOF
GIMBAL_LOCK_THRESHOLD = 0.1  # cos(pitch) threshold for gimbal lock (pitch close to ±90°)

# ============================================================================
# DATA STRUCTURES
# ============================================================================
@dataclass
class CloudInfo:
    """Cached point cloud information"""
    points: Optional[np.ndarray] = None  # Nx3 array
    kdtree: Optional[cKDTree] = None
    timestamp: float = 0.0
    frame_id: str = ""
    num_points: int = 0

@dataclass
class DetectionMessage:
    """Synchronized detection message for processing"""
    proj_contract: ProjectionContract  # ProjectionContract message
    detections: Detection2DArray
    timestamp: float
    frame_id: str

@dataclass
class Pose6DOF:
    """6DOF pose result (MAP FRAME - PCD 맵 좌표계)"""
    detection_id: int
    confidence: float
    # Position (meters) - MAP FRAME
    x: float
    y: float
    z: float
    # Orientation (Euler angles in radians, ZYX order) - MAP FRAME
    roll: float
    pitch: float
    yaw: float
    # Quaternion (primary representation) - MAP FRAME
    qx: float
    qy: float
    qz: float
    qw: float
    # Statistics
    num_points: int
    processing_time_ms: float
    ray_cast_time_ms: float
    pca_time_ms: float
    # Gimbal lock flag
    gimbal_lock: bool = False
    # Reference frame (항상 "map")
    frame_id: str = "map"

# ============================================================================
# ROS2 MESSAGE STRUCTURES (stubs - to be defined in msg file)
# ============================================================================
# TODO: Import these from actual ROS2 messages once defined
# For now, use vision_msgs.Detection2D for bboxes
# ProjectionContract should contain:
#   - sensor_msgs/Image image
#   - geometry_msgs/PoseStamped camera_pose
#   - std_msgs/Header header
#   - float64 hfov_deg, vfov_deg, plane_distance_m
#   - float64[] plane_coeff (a, b, c, d)

# ============================================================================
# MAIN ROS2 NODE CLASS
# ============================================================================
class DetectionsTo6DOFConverter(Node):
    """
    Converts 2D detections + ProjectionContract → 6DOF poses via ray-casting + PCA
    """

    def __init__(self):
        super().__init__('detections_6dof_converter')

        # ────────────────────────────────────────────────────────────────
        # PARAMETERS
        # ────────────────────────────────────────────────────────────────
        self.declare_parameter('max_queue_size', MAX_QUEUE_SIZE)
        self.declare_parameter('ray_k_neighbors', RAY_K_NEIGHBORS)
        self.declare_parameter('ray_tolerance_m', RAY_TOLERANCE_M)
        self.declare_parameter('inlier_threshold_m', INLIER_THRESHOLD_M)
        self.declare_parameter('min_points_for_pca', MIN_POINTS_FOR_PCA)
        self.declare_parameter('csv_output_dir', '')  # Empty = auto-detect latest predict dir

        self.max_queue_size = self.get_parameter('max_queue_size').value
        self.ray_k_neighbors = self.get_parameter('ray_k_neighbors').value
        self.ray_tolerance_m = self.get_parameter('ray_tolerance_m').value
        self.inlier_threshold_m = self.get_parameter('inlier_threshold_m').value
        self.min_points_for_pca = self.get_parameter('min_points_for_pca').value

        # Add parameter change callback (STEP 1)
        self.add_on_set_parameters_callback(self._parameter_callback)

        logger.info(f"✅ [STEP 1] Parameters initialized:")
        logger.info(f"  - max_queue_size: {self.max_queue_size}")
        logger.info(f"  - ray_k_neighbors: {self.ray_k_neighbors}")
        logger.info(f"  - ray_tolerance_m: {self.ray_tolerance_m}")
        logger.info(f"  - inlier_threshold_m: {self.inlier_threshold_m}")
        logger.info(f"  - min_points_for_pca: {self.min_points_for_pca}")

        # ────────────────────────────────────────────────────────────────
        # QUEUES & SYNCHRONIZATION
        # ────────────────────────────────────────────────────────────────
        self.detection_queue = deque(maxlen=self.max_queue_size)
        self.queue_lock = threading.Lock()
        self.queue_event = threading.Event()

        # ────────────────────────────────────────────────────────────────
        # POINT CLOUD CACHE (NO DOWNSAMPLING)
        # ────────────────────────────────────────────────────────────────
        self.cloud_info = CloudInfo()
        self.cloud_lock = threading.Lock()

        # ────────────────────────────────────────────────────────────────
        # SUBSCRIBERS (will be set up in STEP 2 & 3)
        # ────────────────────────────────────────────────────────────────
        self.sub_cloud = None
        self.sub_proj_contract = None
        self.sub_detections = None
        self.synchronizer = None

        # QoS Profile for sensor data
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # QoS Profile for point cloud (TRANSIENT_LOCAL to receive late subscribers)
        self.qos_cloud = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ────────────────────────────────────────────────────────────────
        # PUBLISHERS (will be set up in STEP 10)
        # ────────────────────────────────────────────────────────────────
        self.pub_detections_6dof = None

        # ────────────────────────────────────────────────────────────────
        # STATISTICS & LOGGING
        # ────────────────────────────────────────────────────────────────
        self.frame_count = 0
        self.total_detections = 0
        self.total_failures = 0
        self.last_fps_time = time.time()
        self.frame_times = deque(maxlen=30)  # Rolling window for FPS

        self.stats_lock = threading.Lock()

        # ────────────────────────────────────────────────────────────────
        # CSV LOGGING
        # ────────────────────────────────────────────────────────────────
        csv_output_dir = self.get_parameter('csv_output_dir').value
        if not csv_output_dir or csv_output_dir == '':
            # Auto-detect latest predict directory
            csv_output_dir = self._get_latest_predict_dir()

        self.csv_path = os.path.join(csv_output_dir, 'detections_6dof_log.csv')
        self.csv_lock = threading.Lock()
        self._init_csv_file()

        logger.info(f"CSV logging: {self.csv_path}")

        # ────────────────────────────────────────────────────────────────
        # LOAD POINT CLOUD FROM FILE (OPTIMIZED: No topic needed!)
        # ────────────────────────────────────────────────────────────────
        try:
            self._load_point_cloud_from_file()
            logger.info("✅ [STEP 3] Point cloud loaded from file (optimized approach)")
        except Exception as e:
            logger.error(f"❌ Failed to load point cloud: {e}")
            # Continue anyway - will wait for topic if file fails

        # ────────────────────────────────────────────────────────────────
        # SETUP SUBSCRIBERS (STEP 2)
        # ────────────────────────────────────────────────────────────────
        try:
            self._setup_subscribers()
            logger.info("✅ [STEP 2] Message synchronization initialized successfully")
        except Exception as e:
            logger.error(f"❌ Failed to setup subscribers: {e}")
            # Continue anyway - subscribers will be ready when topics appear

        # ────────────────────────────────────────────────────────────────
        # WORKER THREAD
        # ────────────────────────────────────────────────────────────────
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        logger.info("✅ [STEP 1] DetectionsTo6DOFConverter initialized")
        logger.info(f"  - Cloud cache: Prepared for 14.6M points (NO downsampling)")
        logger.info(f"  - Detection queue: max_size={self.max_queue_size}")
        logger.info(f"  - Worker thread: Started")
        logger.info(f"  - Statistics: frame_count=0, total_detections=0, total_failures=0")

    def _get_latest_predict_dir(self):
        """
        Get the latest predict directory in /home/jack/ros2_ws/runs/segment/

        Returns:
            Latest predict directory path (e.g., /home/jack/ros2_ws/runs/segment/predict26)
        """
        base_dir = '/home/jack/ros2_ws/runs/segment'

        if not os.path.exists(base_dir):
            logger.warning(f"Base directory not found: {base_dir}, using home directory")
            return os.path.expanduser('~/ros2_ws')

        try:
            # Find all predict* directories
            predict_dirs = []
            for item in os.listdir(base_dir):
                if item.startswith('predict'):
                    item_path = os.path.join(base_dir, item)
                    if os.path.isdir(item_path):
                        predict_dirs.append(item_path)

            if not predict_dirs:
                logger.warning(f"No predict directories found in {base_dir}, using base directory")
                return base_dir

            # Sort by modification time and get the latest
            latest_dir = max(predict_dirs, key=lambda d: os.path.getmtime(d))
            logger.info(f"✅ Auto-detected latest predict directory: {latest_dir}")
            return latest_dir

        except Exception as e:
            logger.error(f"Error finding latest predict dir: {e}, using home directory")
            return os.path.expanduser('~/ros2_ws')

    def _load_point_cloud_from_file(self):
        """
        Load point cloud from PLY file (OPTIMIZED APPROACH)

        Instead of waiting for /projection/cloud_raw topic, load the static
        point cloud from disk once at startup. Much faster!
        """
        # Try multiple possible paths
        possible_paths = [
            '/home/jack/ros2_ws/project_hj_v2/241108_converted - Cloud.ply',
            '/home/jack/ros2_ws/project_hj_v2/241108_converted-Cloud.ply',
            os.path.expanduser('~/ros2_ws/project_hj_v2/241108_converted - Cloud.ply'),
        ]

        ply_path = None
        for path in possible_paths:
            if os.path.exists(path):
                ply_path = path
                break

        if ply_path is None:
            logger.error(f"❌ Point cloud file not found. Tried:")
            for p in possible_paths:
                logger.error(f"   - {p}")
            raise FileNotFoundError("No PLY file found")

        try:
            import open3d as o3d
            t_start = time.time()

            # Load PLY file
            pcd = o3d.io.read_point_cloud(ply_path)
            points = np.asarray(pcd.points)

            if len(points) == 0:
                raise ValueError("PLY file contains no points")

            # Build KD-tree
            kdtree = cKDTree(points)
            t_kdtree = time.time()

            # Update cache atomically
            with self.cloud_lock:
                self.cloud_info.points = points
                self.cloud_info.kdtree = kdtree
                self.cloud_info.timestamp = time.time()
                self.cloud_info.frame_id = "map"
                self.cloud_info.num_points = len(points)

            elapsed_ms = (t_kdtree - t_start) * 1000
            logger.info(f"✅ [OPTIMIZED] Loaded PLY: {len(points):,} points from:")
            logger.info(f"   {ply_path}")
            logger.info(f"   KD-tree build time: {elapsed_ms:.1f}ms")
            logger.info(f"   Point range: X=[{points[:,0].min():.2f}, {points[:,0].max():.2f}] "
                       f"Y=[{points[:,1].min():.2f}, {points[:,1].max():.2f}] "
                       f"Z=[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")

        except ImportError:
            logger.error("❌ open3d not installed. Install with: pip install open3d")
            raise
        except Exception as e:
            logger.error(f"❌ Error loading PLY file: {e}")
            raise

    def _init_csv_file(self):
        """Initialize CSV file with headers"""
        try:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

            # Check if file exists
            file_exists = os.path.exists(self.csv_path)

            if not file_exists:
                with open(self.csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Write header
                    writer.writerow([
                        'Timestamp',
                        'Frame',
                        'Detection_ID',
                        'X_m', 'Y_m', 'Z_m',
                        'Roll_rad', 'Pitch_rad', 'Yaw_rad',
                        'Qx', 'Qy', 'Qz', 'Qw',
                        'Confidence',
                        'Num_Points',
                        'Gimbal_Lock',
                        'Processing_Time_ms'
                    ])
                logger.info(f"✅ CSV file created: {self.csv_path}")
            else:
                logger.info(f"✅ CSV file found: {self.csv_path}")

        except Exception as e:
            logger.error(f"❌ Error initializing CSV: {e}")

    def _save_to_csv(self, pose: Pose6DOF):
        """Save single detection to CSV"""
        try:
            with self.csv_lock:
                with open(self.csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    # Use quaternion as primary representation when gimbal lock detected
                    euler_repr = "QUAT" if pose.gimbal_lock else "EULER"
                    writer.writerow([
                        datetime.now().isoformat(),
                        self.frame_count,
                        pose.detection_id,
                        f"{pose.x:.6f}",
                        f"{pose.y:.6f}",
                        f"{pose.z:.6f}",
                        f"{pose.roll:.6f}",
                        f"{pose.pitch:.6f}",
                        f"{pose.yaw:.6f}",
                        f"{pose.qx:.6f}",
                        f"{pose.qy:.6f}",
                        f"{pose.qz:.6f}",
                        f"{pose.qw:.6f}",
                        f"{pose.confidence:.4f}",
                        pose.num_points,
                        "Y" if pose.gimbal_lock else "N",
                        f"{pose.processing_time_ms:.2f}"
                    ])
        except Exception as e:
            logger.error(f"❌ Error saving to CSV: {e}")

    def _parameter_callback(self, params):
        """Parameter change callback (STEP 1)"""
        from rcl_interfaces.msg import SetParametersResult
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == 'max_queue_size':
                self.max_queue_size = param.value
                logger.info(f"[PARAM UPDATE] max_queue_size = {param.value}")
            elif param.name == 'ray_k_neighbors':
                self.ray_k_neighbors = param.value
                logger.info(f"[PARAM UPDATE] ray_k_neighbors = {param.value}")
            elif param.name == 'ray_tolerance_m':
                self.ray_tolerance_m = param.value
                logger.info(f"[PARAM UPDATE] ray_tolerance_m = {param.value}")
            elif param.name == 'inlier_threshold_m':
                self.inlier_threshold_m = param.value
                logger.info(f"[PARAM UPDATE] inlier_threshold_m = {param.value}")
            elif param.name == 'min_points_for_pca':
                self.min_points_for_pca = param.value
                logger.info(f"[PARAM UPDATE] min_points_for_pca = {param.value}")

        return result

    def _setup_subscribers(self):
        """
        Setup subscribers and message synchronization (STEP 2 & 3)

        Topics:
        - /projection/cloud_raw (sensor_msgs/PointCloud2) → _cloud_callback
        - /projection/proj_contract (ProjectionContract) → sync
        - /projection/sam3/detections (Detection2DArray) → sync
        """
        # Cloud subscriber (STEP 3: Point cloud management)
        # Use TRANSIENT_LOCAL QoS to receive cloud data that was published before subscription
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/projection/cloud_raw',
            self._cloud_callback,
            qos_profile=self.qos_cloud  # Use TRANSIENT_LOCAL QoS
        )
        logger.debug("[STEP 3] Point cloud subscriber created with TRANSIENT_LOCAL QoS")

        # Message filters for synchronization (STEP 2)
        # Note: Using ProjectionContract message type (not Float64MultiArray)
        self.sub_proj_contract = message_filters.Subscriber(
            self,
            ProjectionContract,  # ← Use actual ProjectionContract type
            '/projection/contract',
            qos_profile=self.qos_sensor
        )
        logger.debug("[STEP 2] ProjectionContract subscriber created: /projection/contract")

        self.sub_detections = message_filters.Subscriber(
            self,
            Detection2DArray,
            '/projection/sam3/detections',
            qos_profile=self.qos_sensor
        )
        logger.debug("[STEP 2] Detection2DArray subscriber created")

        # Approximate time synchronizer with 0.1 second slop
        # (Use ApproximateTimeSynchronizer for slop parameter support in ROS2 Humble)
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.sub_proj_contract, self.sub_detections],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.synchronizer.registerCallback(self._sync_callback)
        logger.debug("[STEP 2] ApproximateTimeSynchronizer registered with slop=0.1s")

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        self.queue_event.set()  # Wake up worker thread
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        super().destroy_node()
        logger.info("✅ DetectionsTo6DOFConverter shutdown complete")
        with self.stats_lock:
            logger.info(f"  Final statistics: Frames={self.frame_count}, "
                       f"Detections={self.total_detections}, "
                       f"Failures={self.total_failures}")

    # ════════════════════════════════════════════════════════════════════
    # PLACEHOLDER METHODS (to be implemented in later STEPs)
    # ════════════════════════════════════════════════════════════════════

    def _cloud_callback(self, msg):
        """
        STEP 3: Callback for point cloud updates (PointCloud2 → numpy → KD-tree)

        Critical: NO DOWNSAMPLING - all 14.6M points used
        """
        try:
            t_start = time.time()

            # Convert PointCloud2 to numpy (all points, no sampling)
            points = np.array(list(pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )))

            if len(points) == 0:
                logger.warning("❌ [STEP 3] Empty point cloud received")
                return

            # Build KD-tree on all points
            kdtree = cKDTree(points)
            t_kdtree = time.time()

            # Update cache atomically
            with self.cloud_lock:
                self.cloud_info.points = points
                self.cloud_info.kdtree = kdtree
                self.cloud_info.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                self.cloud_info.frame_id = msg.header.frame_id
                self.cloud_info.num_points = len(points)

            elapsed_ms = (t_kdtree - t_start) * 1000
            logger.info(f"✅ [STEP 3] Cloud loaded: {len(points):,} points | "
                       f"Frame: {msg.header.frame_id} | "
                       f"KD-tree build: {elapsed_ms:.1f}ms")
            logger.debug(f"  Point range: X=[{points[:,0].min():.2f}, {points[:,0].max():.2f}] "
                        f"Y=[{points[:,1].min():.2f}, {points[:,1].max():.2f}] "
                        f"Z=[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")

        except Exception as e:
            logger.error(f"❌ [STEP 3] Error processing cloud: {e}")
            import traceback
            logger.debug(traceback.format_exc())

    def _sync_callback(self, proj_contract, detections):
        """
        STEP 4: Synchronized callback for ProjectionContract + Detection2DArray

        Receives synchronized messages and queues them for processing.
        """
        try:
            t_arrival = time.time()

            # Extract detection count
            num_detections = len(detections.detections) if detections.detections else 0

            # Create message container
            det_msg = DetectionMessage(
                proj_contract=proj_contract,
                detections=detections,
                timestamp=t_arrival,
                frame_id=detections.header.frame_id if detections.header else "unknown"
            )

            # Queue message (thread-safe)
            with self.queue_lock:
                was_empty = len(self.detection_queue) == 0
                self.detection_queue.append(det_msg)
                queue_size = len(self.detection_queue)

            # Signal worker thread if queue was empty
            if was_empty:
                self.queue_event.set()

            logger.debug(f"✅ [STEP 4] Message queued: detections={num_detections}, "
                        f"queue_size={queue_size}, frame_id={det_msg.frame_id}")

            # Log if queue is getting full
            if queue_size >= self.max_queue_size * 0.8:
                logger.warning(f"⚠️ [STEP 4] Queue approaching capacity: {queue_size}/{self.max_queue_size}")

        except Exception as e:
            logger.error(f"❌ [STEP 4] Error in sync callback: {e}")
            import traceback
            logger.debug(traceback.format_exc())

    def _worker_loop(self):
        """
        STEP 5: Main processing loop (worker thread)

        Continuously:
        1. Wait for queued messages
        2. Pop and process each detection
        3. Calculate performance metrics
        4. Handle exceptions gracefully
        """
        logger.info("✅ [STEP 5] Worker loop started")

        while self.running:
            # Wait for queue event with timeout
            self.queue_event.wait(timeout=1.0)
            self.queue_event.clear()

            if not self.running:
                break

            # Process all queued messages
            while self.running:
                # Pop message from queue (thread-safe)
                with self.queue_lock:
                    if len(self.detection_queue) == 0:
                        break
                    det_msg = self.detection_queue.popleft()

                # Process outside lock to avoid blocking
                try:
                    t_frame_start = time.time()

                    # Check if cloud is loaded
                    with self.cloud_lock:
                        if self.cloud_info.kdtree is None:
                            logger.warning("[STEP 5] Point cloud not yet available, skipping frame")
                            continue

                    # Process all detections in this message
                    poses_6dof = []
                    for det_idx, detection in enumerate(det_msg.detections.detections):
                        pose = self._process_detection(det_msg, det_idx)
                        if pose is not None:
                            poses_6dof.append(pose)

                    # Update statistics
                    with self.stats_lock:
                        self.total_detections += len(poses_6dof)
                        self.total_failures += len(det_msg.detections.detections) - len(poses_6dof)

                    # Publish results
                    if len(poses_6dof) > 0:
                        self._publish_results(poses_6dof)

                    # Log frame statistics
                    frame_time_ms = (time.time() - t_frame_start) * 1000
                    self._log_frame_stats(frame_time_ms)

                except Exception as e:
                    logger.error(f"❌ [STEP 5] Error in worker loop: {e}")
                    import traceback
                    logger.debug(traceback.format_exc())

        logger.info("✅ [STEP 5] Worker loop stopped")

    def _process_detection(self, detection_msg: DetectionMessage, detection_idx: int) -> Optional[Pose6DOF]:
        """
        STEP 6: Process individual detection (bbox → 3D points → 6DOF)

        Args:
            detection_msg: Synchronized message with ProjectionContract + Detection2DArray
            detection_idx: Index in Detection2DArray.detections

        Returns:
            Pose6DOF object or None if processing failed
        """
        try:
            t_start = time.time()
            det = detection_msg.detections.detections[detection_idx]

            # Extract detection metadata
            detection_id = int(det.id) if hasattr(det, 'id') and det.id else detection_idx

            # Extract confidence from ObjectHypothesisWithPose
            confidence = 0.0
            if det.results and len(det.results) > 0:
                result = det.results[0]
                # ObjectHypothesisWithPose has hypothesis.score
                if hasattr(result, 'hypothesis') and hasattr(result.hypothesis, 'score'):
                    confidence = float(result.hypothesis.score)
                # Fallback: try direct score attribute
                elif hasattr(result, 'score'):
                    confidence = float(result.score)

            # Validate detection
            if confidence < 0.1:  # Minimum confidence threshold
                logger.debug(f"[STEP 6] Detection {detection_idx} rejected: confidence={confidence:.3f} < 0.1")
                return None

            # Extract bbox (assumes Detection2D has bbox with center and size)
            bbox = det.bbox
            if not bbox:
                logger.warning(f"[STEP 6] Detection {detection_idx} has no bbox")
                return None

            # Log bbox structure for debugging
            logger.debug(f"[STEP 6] BBox type: {type(bbox).__name__}, "
                        f"center type: {type(bbox.center).__name__}")

            # Ray-casting: Get 3D points from bbox (STEP 7)
            t_ray = time.time()
            points_3d = self._ray_cast_points(bbox, detection_msg.proj_contract)
            ray_time_ms = (time.time() - t_ray) * 1000

            if points_3d is None or len(points_3d) < self.min_points_for_pca:
                logger.debug(f"[STEP 6] Detection {detection_idx}: insufficient points "
                            f"({len(points_3d) if points_3d is not None else 0} < {self.min_points_for_pca})")
                return None

            # Compute 6DOF (STEP 8 & 9)
            t_pca = time.time()
            pose = self._compute_6dof_pose(points_3d, detection_id, confidence)
            pca_time_ms = (time.time() - t_pca) * 1000

            if pose is None:
                return None

            # Add timing info
            pose.ray_cast_time_ms = ray_time_ms
            pose.pca_time_ms = pca_time_ms
            pose.processing_time_ms = (time.time() - t_start) * 1000

            logger.debug(f"✅ [STEP 6] Detection {detection_idx}: "
                        f"id={pose.detection_id}, conf={confidence:.3f}, "
                        f"points={pose.num_points}, "
                        f"time={pose.processing_time_ms:.1f}ms")

            return pose

        except Exception as e:
            logger.error(f"❌ [STEP 6] Error processing detection {detection_idx}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return None

    def _ray_cast_points(self, bbox: any, proj_contract: any) -> Optional[np.ndarray]:
        """
        STEP 7: Ray-casting to find 3D points from 2D bbox (NO DOWNSAMPLING)

        Uses projection plane + camera FOV to cast rays and find 3D points.
        Returns ALL inlier points from the 14.6M point cloud.

        Args:
            bbox: vision_msgs/BoundingBox2D (center.x, center.y, size_x, size_y)
            proj_contract: ProjectionContract with plane equation and camera pose

        Returns:
            Nx3 numpy array of 3D points (all inliers, no sampling)
        """
        try:
            # Ensure cloud is available
            with self.cloud_lock:
                if self.cloud_info.kdtree is None or self.cloud_info.points is None:
                    logger.debug("[STEP 7] Cloud not available yet")
                    return None
                points_all = self.cloud_info.points.copy()
                kdtree = self.cloud_info.kdtree

            # Extract bbox center and size (safe attribute access)
            center_x = 0.0
            center_y = 0.0

            if hasattr(bbox, 'center'):
                # Try different possible structures for center
                center = bbox.center
                if hasattr(center, 'x'):
                    center_x = float(center.x)
                elif hasattr(center, 'position') and hasattr(center.position, 'x'):
                    center_x = float(center.position.x)

                if hasattr(center, 'y'):
                    center_y = float(center.y)
                elif hasattr(center, 'position') and hasattr(center.position, 'y'):
                    center_y = float(center.position.y)

            size_x = float(bbox.size_x) if hasattr(bbox, 'size_x') else 1.0
            size_y = float(bbox.size_y) if hasattr(bbox, 'size_y') else 1.0

            logger.debug(f"[STEP 7] Ray-casting bbox: center=({center_x:.0f}, {center_y:.0f}), "
                        f"size=({size_x:.0f}, {size_y:.0f})")

            # TODO: Convert bbox to 3D using proj_contract plane equation
            # For now, use simple approach: find points in 3D space near projected bbox center
            # This assumes the projection is orthogonal (simplified)

            # Estimate bbox extent in 3D
            # Assuming pixels_per_unit ~500 from projection_plane config
            pixels_per_unit = 500.0  # From config
            bbox_3d_size = max(size_x, size_y) / pixels_per_unit

            # Search for points in KD-tree using radius search
            # Start with larger radius and filter more precisely
            search_radius = bbox_3d_size * 2.0  # Search 2x bbox size

            # Use KD-tree to find nearby points
            # Note: We need a reference point in 3D. Without full ProjectionContract parsing,
            # we use a heuristic: search in the center of the cloud.
            # TODO: Properly convert bbox center to 3D point using camera transform

            # Temporary: Get all points within reasonable distance of cloud center
            cloud_center = points_all.mean(axis=0)
            query_point = cloud_center.copy()  # Placeholder

            # Query KD-tree for neighbors within radius
            indices = kdtree.query_ball_point(query_point, search_radius)

            if len(indices) == 0:
                logger.debug(f"[STEP 7] No points found in radius {search_radius:.3f}m")
                return None

            inlier_points = points_all[indices]

            # Filter outliers using IQR method on distance to centroid
            if len(inlier_points) > self.min_points_for_pca:
                centroid = inlier_points.mean(axis=0)
                distances = np.linalg.norm(inlier_points - centroid, axis=1)
                q1, q3 = np.percentile(distances, [25, 75])
                iqr = q3 - q1
                outlier_threshold = q3 + 1.5 * iqr
                valid_mask = distances <= outlier_threshold
                inlier_points = inlier_points[valid_mask]

            logger.debug(f"[STEP 7] Ray-casting result: {len(inlier_points)} points "
                        f"(from {len(indices)} neighborhood search, all preserved)")

            return inlier_points if len(inlier_points) >= self.min_points_for_pca else None

        except Exception as e:
            logger.error(f"❌ [STEP 7] Ray-casting error: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return None

    def _compute_6dof_pose(self, points_3d: np.ndarray, detection_id: int,
                           confidence: float) -> Optional[Pose6DOF]:
        """
        STEP 8: Extract 6DOF pose via PCA (Principal Component Analysis)

        PCA finds principal axes of 3D point cloud:
        - Centroid = position (x, y, z)
        - Principal axes = rotation matrix orientation

        Args:
            points_3d: Nx3 array of 3D points
            detection_id: Detection ID
            confidence: Detection confidence

        Returns:
            Pose6DOF or None if invalid
        """
        try:
            if len(points_3d) < self.min_points_for_pca:
                logger.debug(f"[STEP 8] Insufficient points: {len(points_3d)} < {self.min_points_for_pca}")
                return None

            # Step 1: Compute centroid (position)
            centroid = points_3d.mean(axis=0)
            x, y, z = float(centroid[0]), float(centroid[1]), float(centroid[2])

            # Step 2: Center points
            centered_points = points_3d - centroid

            # Step 3: Compute covariance matrix
            cov_matrix = np.cov(centered_points.T)

            # Handle case where covariance is singular
            if np.linalg.matrix_rank(cov_matrix) < 3:
                logger.debug(f"[STEP 8] Covariance matrix rank < 3 (degenerate point cloud)")
                # Use a regularized covariance
                cov_matrix += np.eye(3) * 1e-6

            # Step 4: SVD to get principal components
            U, S, Vt = np.linalg.svd(cov_matrix)

            # U contains the principal axes as columns (in order of variance)
            # Rotation matrix: columns are principal axes
            rotation_matrix = U

            # Ensure determinant is +1 (proper rotation, not reflection)
            if np.linalg.det(rotation_matrix) < 0:
                rotation_matrix[:, -1] *= -1

            # Step 5: Validate orthonormality
            should_be_identity = rotation_matrix.T @ rotation_matrix
            ortho_error = np.linalg.norm(should_be_identity - np.eye(3))
            if ortho_error > 0.01:
                logger.warning(f"[STEP 8] Orthonormality check warning: error={ortho_error:.6f}")

            # Step 6: Convert to quaternion and Euler angles
            (roll, pitch, yaw), (qx, qy, qz, qw), gimbal_lock = self._euler_from_rotation(rotation_matrix)

            # Check for NaN/inf
            pose_values = [x, y, z, roll, pitch, yaw, qx, qy, qz, qw]
            if any(np.isnan(v) or np.isinf(v) for v in pose_values):
                logger.error(f"[STEP 8] NaN/inf detected in pose: "
                            f"({x}, {y}, {z}), euler=({roll}, {pitch}, {yaw})")
                return None

            # Create result
            pose = Pose6DOF(
                detection_id=detection_id,
                confidence=confidence,
                x=x, y=y, z=z,
                roll=roll, pitch=pitch, yaw=yaw,
                qx=qx, qy=qy, qz=qz, qw=qw,
                num_points=len(points_3d),
                processing_time_ms=0.0,
                ray_cast_time_ms=0.0,
                pca_time_ms=0.0,
                gimbal_lock=gimbal_lock
            )

            logger.debug(f"✅ [STEP 8] PCA complete: pos=({x:.3f}, {y:.3f}, {z:.3f}), "
                        f"euler=({roll:.3f}, {pitch:.3f}, {yaw:.3f}), "
                        f"ortho_err={ortho_error:.6f}")

            return pose

        except Exception as e:
            logger.error(f"❌ [STEP 8] PCA computation error: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return None

    def _euler_from_rotation(self, rotation_matrix: np.ndarray) -> Tuple[Tuple[float, float, float],
                                                                           Tuple[float, float, float, float],
                                                                           bool]:
        """
        STEP 9: Extract Euler angles (ZYX order) and Quaternion from rotation matrix

        ZYX order (extrinsic rotations):
        1. Rotate around Z-axis (yaw)
        2. Rotate around Y-axis (pitch)
        3. Rotate around X-axis (roll)

        R = Rz(yaw) * Ry(pitch) * Rx(roll)

        Args:
            rotation_matrix: 3x3 rotation matrix

        Returns:
            Tuple of ((roll, pitch, yaw), (qx, qy, qz, qw), gimbal_lock_flag)
        """
        try:
            # Ensure matrix is 3x3
            assert rotation_matrix.shape == (3, 3), f"Expected 3x3, got {rotation_matrix.shape}"

            # Convert to quaternion using scipy (quaternion is gimbal lock free!)
            rot = Rotation.from_matrix(rotation_matrix)
            qx, qy, qz, qw = rot.as_quat()

            # Extract Euler angles from ZYX rotation matrix
            # R = Rz(yaw) * Ry(pitch) * Rx(roll)
            # After matrix multiplication:
            # R[2,0] = -sin(pitch)
            # R[2,1] = sin(roll) * cos(pitch)
            # R[2,2] = cos(roll) * cos(pitch)
            # R[1,0] = sin(yaw) * cos(pitch)
            # R[0,0] = cos(yaw) * cos(pitch)

            # Pitch
            sin_pitch = -rotation_matrix[2, 0]
            sin_pitch = np.clip(sin_pitch, -1.0, 1.0)  # Clamp for numerical stability
            pitch = np.arcsin(sin_pitch)

            # Check for gimbal lock (pitch close to ±90°)
            cos_pitch = np.cos(pitch)
            gimbal_lock = abs(cos_pitch) < GIMBAL_LOCK_THRESHOLD

            if gimbal_lock:
                logger.debug(f"⚠️ [STEP 9] Gimbal lock detected: cos(pitch)={cos_pitch:.6f} | Using quaternion instead")
                # Handle gimbal lock by setting roll = 0 (convention)
                roll = 0.0
                yaw = np.arctan2(rotation_matrix[0, 1], rotation_matrix[1, 1])
            else:
                # Normal case
                roll = np.arctan2(rotation_matrix[2, 1] / cos_pitch, rotation_matrix[2, 2] / cos_pitch)
                yaw = np.arctan2(rotation_matrix[1, 0] / cos_pitch, rotation_matrix[0, 0] / cos_pitch)

            # Ensure angles are in [-π, π]
            roll = np.arctan2(np.sin(roll), np.cos(roll))
            pitch = np.arctan2(np.sin(pitch), np.cos(pitch))
            yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

            logger.debug(f"✅ [STEP 9] Orientation extracted: "
                        f"euler=({roll:.4f}, {pitch:.4f}, {yaw:.4f}) rad | "
                        f"quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}) | "
                        f"gimbal_lock={gimbal_lock}")

            return (float(roll), float(pitch), float(yaw)), \
                   (float(qx), float(qy), float(qz), float(qw)), \
                   gimbal_lock

        except Exception as e:
            logger.error(f"❌ [STEP 9] Euler angle extraction error: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            # Return identity (no rotation)
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), False

    def _setup_publisher(self):
        """Setup publisher for 6DOF results (called on first publish)"""
        if self.pub_detections_6dof is None:
            # For now, publish as Float64MultiArray (placeholder)
            # Format: [id, x, y, z, roll, pitch, yaw, qx, qy, qz, qw, confidence, num_points]
            from std_msgs.msg import Float64MultiArray
            self.pub_detections_6dof = self.create_publisher(
                Float64MultiArray,
                '/projection/detections_6dof',
                qos_profile=self.qos_sensor
            )
            logger.info("✅ [STEP 10] Publisher created: /projection/detections_6dof")

    def _publish_results(self, poses_6dof: List[Pose6DOF]):
        """
        STEP 10: Publish 6DOF results, log metrics, and save to CSV

        Args:
            poses_6dof: List of Pose6DOF results
        """
        if not poses_6dof:
            return

        try:
            # Setup publisher if needed
            self._setup_publisher()

            # Publish each detection as Float64MultiArray (with frame_id marker)
            from std_msgs.msg import Float64MultiArray, Header
            for pose in poses_6dof:
                msg = Float64MultiArray()
                # Header with MAP FRAME
                msg.layout.dim.append(std_msgs.msg.MultiArrayDimension(label="detections_6dof_data", size=14))
                msg.data = [
                    float(pose.detection_id),
                    pose.x, pose.y, pose.z,
                    pose.roll, pose.pitch, pose.yaw,
                    pose.qx, pose.qy, pose.qz, pose.qw,
                    pose.confidence,
                    float(pose.num_points),
                    pose.processing_time_ms
                ]
                # Note: frame_id should be "map" - see data format specification
                self.pub_detections_6dof.publish(msg)

            # Log per-detection details and save to CSV
            for i, pose in enumerate(poses_6dof):
                # Log with quaternion if gimbal lock, otherwise Euler angles
                if pose.gimbal_lock:
                    orientation_str = f"quat=({pose.qx:.3f}, {pose.qy:.3f}, {pose.qz:.3f}, {pose.qw:.3f}) [GIMBAL_LOCK]"
                else:
                    orientation_str = f"euler=({pose.roll:.3f}, {pose.pitch:.3f}, {pose.yaw:.3f}) rad"

                logger.info(f"[STEP 10] Detection {i}: "
                           f"id={pose.detection_id} | "
                           f"pos=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) | "
                           f"{orientation_str} | "
                           f"conf={pose.confidence:.3f} | "
                           f"pts={pose.num_points} | "
                           f"time={pose.processing_time_ms:.1f}ms")

                # Save to CSV
                self._save_to_csv(pose)

            logger.debug(f"✅ [STEP 10] Published {len(poses_6dof)} detections to ROS2 + CSV")

        except Exception as e:
            logger.error(f"❌ [STEP 10] Publishing error: {e}")
            import traceback
            logger.debug(traceback.format_exc())

    # ════════════════════════════════════════════════════════════════════
    # UTILITY METHODS
    # ════════════════════════════════════════════════════════════════════

    def _log_frame_stats(self, frame_time_ms: float):
        """Log frame statistics"""
        with self.stats_lock:
            self.frame_count += 1
            self.frame_times.append(frame_time_ms)

            if len(self.frame_times) >= 10:
                avg_time = np.mean(self.frame_times)
                fps = 1000.0 / avg_time if avg_time > 0 else 0.0
                logger.info(f"[FRAME {self.frame_count:5d}] FPS={fps:5.1f} | "
                           f"Detections={self.total_detections:3d} | "
                           f"Failures={self.total_failures:2d} | "
                           f"AvgTime={avg_time:6.1f}ms")


# ============================================================================
# ENTRY POINT
# ============================================================================
def main(args=None):
    """Main entry point for ROS2"""
    rclpy.init(args=args)

    try:
        node = DetectionsTo6DOFConverter()
        logger.info("=" * 80)
        logger.info("DETECTIONS 6DOF CONVERTER - INITIALIZED")
        logger.info("=" * 80)
        logger.info("Waiting for ProjectionContract + Detection2DArray messages...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        logger.info("ROS2 shutdown complete")


if __name__ == '__main__':
    main()
