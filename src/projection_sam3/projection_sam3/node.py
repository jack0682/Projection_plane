#!/usr/bin/env python3
"""
SAM3 Semantic Segmentation Node for ROS2.
Identical to sam3_ultralytics.py mechanics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import threading
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

try:
    from ultralytics.models.sam import SAM3SemanticPredictor
except ImportError:
    SAM3SemanticPredictor = None


class ProjectionSAM3Node(Node):
    """ROS2 node for SAM3 semantic segmentation."""

    def __init__(self):
        super().__init__('projection_sam3_node')

        # Declare parameters
        self.declare_parameter('model_path', '/home/jack/ros2_ws/sam_3d_test/models/sam3.pt')
        self.declare_parameter('max_fps', 2.0)

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.max_fps = self.get_parameter('max_fps').value

        self.bridge = CvBridge()
        self.predictor = None
        self._init_model()

        self.last_inference_time = 0
        self.inference_interval = 1.0 / self.max_fps

        # Thread-safe image buffer
        self.latest_image_cv = None
        self.latest_image_header = None
        self.image_lock = threading.Lock()
        self.new_image_event = threading.Event()

        # Create publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_detections = self.create_publisher(Detection2DArray, '/projection/sam3/detections', qos)
        self.pub_debug = self.create_publisher(String, '/projection/sam3/debug', qos)

        # Create subscription
        self.create_subscription(
            Image,
            '/projection/image',
            self._image_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        # Start worker thread
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info('ProjectionSAM3Node initialized')

    def _init_model(self):
        """Initialize SAM3 model - exactly like sam3_ultralytics.py"""
        if SAM3SemanticPredictor is None:
            self.get_logger().error('SAM3SemanticPredictor not available')
            return

        try:
            overrides = dict(
                conf=0.25,
                task="segment",
                mode="predict",
                model=self.model_path,
                half=True,
                save=True,
                imgsz=1088,
            )
            self.predictor = SAM3SemanticPredictor(overrides=overrides)
            self.get_logger().info(f'SAM3 model loaded: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load SAM3 model: {e}')
            self.predictor = None

    def _image_callback(self, msg: Image):
        """Callback for incoming image."""
        try:
            image_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        with self.image_lock:
            self.latest_image_cv = image_cv
            self.latest_image_header = msg.header

        self.new_image_event.set()

    def _worker_loop(self):
        """Worker thread for inference."""
        while rclpy.ok():
            self.new_image_event.wait()
            self.new_image_event.clear()

            # FPS throttle
            now = time.time()
            if now - self.last_inference_time < self.inference_interval:
                continue

            with self.image_lock:
                if self.latest_image_cv is None:
                    continue
                image_cv = self.latest_image_cv.copy()
                header = self.latest_image_header

            try:
                self._run_inference(image_cv, header)
                self.last_inference_time = time.time()
            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')

    def _run_inference(self, image_cv, header):
        """Run inference - exactly like sam3_ultralytics.py"""
        if self.predictor is None:
            return

        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB)

        detections_list = []

        try:
            # Set image and run inference - exactly like sam3_ultralytics.py
            self.predictor.set_image(image_rgb)
            results = self.predictor(text=["box", "magazine"])

            if results and len(results) > 0:
                res = results[0]

                # Extract masks
                if hasattr(res, 'masks') and res.masks is not None:
                    masks_data = res.masks.data
                    if hasattr(masks_data, 'cpu'):
                        masks_data = masks_data.cpu().numpy()
                    else:
                        masks_data = np.asarray(masks_data)

                    # Get confidence
                    conf_data = None
                    if hasattr(res, 'conf') and res.conf is not None:
                        conf_data = res.conf
                        if hasattr(conf_data, 'cpu'):
                            conf_data = conf_data.cpu().numpy()
                        else:
                            conf_data = np.asarray(conf_data)

                    # Process each mask
                    for mask_idx, mask in enumerate(masks_data):
                        points = np.where(mask > 0)
                        if len(points[0]) > 0:
                            ymin, ymax = points[0].min(), points[0].max()
                            xmin, xmax = points[1].min(), points[1].max()

                            conf = 1.0
                            if conf_data is not None and len(conf_data) > mask_idx:
                                conf = float(conf_data[mask_idx])

                            det = Detection2D()
                            center_x = (xmin + xmax) / 2.0
                            center_y = (ymin + ymax) / 2.0
                            size_x = xmax - xmin
                            size_y = ymax - ymin

                            det.bbox.center.position.x = float(center_x)
                            det.bbox.center.position.y = float(center_y)
                            det.bbox.size_x = float(size_x)
                            det.bbox.size_y = float(size_y)

                            hyp = ObjectHypothesisWithPose()
                            hyp.hypothesis.class_id = "detection"
                            hyp.hypothesis.score = float(conf)
                            det.results.append(hyp)

                            detections_list.append(det)

        except Exception as e:
            self.get_logger().error(f'SAM3 inference error: {e}')

        # Log and publish
        self.get_logger().info(f'Detections found: {len(detections_list)}')

        detection_array = Detection2DArray()
        detection_array.header = header
        detection_array.detections = detections_list
        self.pub_detections.publish(detection_array)

        debug_msg = String()
        debug_msg.data = f'Detections: {len(detections_list)}'
        self.pub_debug.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionSAM3Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
