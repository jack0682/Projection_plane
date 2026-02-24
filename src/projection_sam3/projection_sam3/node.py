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
# CSV 저장을 위한 코드
import csv
import os

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

try:
    from ultralytics.models.sam import SAM3SemanticPredictor
except ImportError:
    SAM3SemanticPredictor = None


class ProjectionSAM3Node(Node):
    """ROS2 node for SAM3 semantic segmentation.
    Two-Stage Inference: Rack Detection → Crop → Object Detection.
    """

    def __init__(self):
        super().__init__('projection_sam3_node')

        # Declare parameters
        self.declare_parameter('model_path', '/home/jack/ros2_ws/sam_3d_test/models/sam3.pt')
        self.declare_parameter('max_fps', 2.0)
        self.declare_parameter('conf_rack', 0.3)
        self.declare_parameter('conf_obj', 0.7)
        self.declare_parameter('crop_padding', 50)

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.max_fps = self.get_parameter('max_fps').value
        self.conf_rack = self.get_parameter('conf_rack').value
        self.conf_obj = self.get_parameter('conf_obj').value
        self.crop_padding = self.get_parameter('crop_padding').value

        # CSV 저장을 위한 코드 (runs/segment/predictN 에 저장)
        self.csv_base_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws/runs/segment')
        self.csv_path = None  # 첫 추론 시 최신 predictN 폴더에서 결정

        self.bridge = CvBridge()
        self.predictor_rack = None
        self.predictor_obj = None
        self._init_model()

        self.last_inference_time = 0
        self.inference_interval = 1.0 / self.max_fps

        # Thread-safe image buffer
        self.latest_image_cv = None
        self.latest_image_header = None
        self.image_lock = threading.Lock()
        self.new_image_event = threading.Event()

        # Mask buffer for Phase 5 (6DOF node)
        self.latest_masks = []
        self.masks_lock = threading.Lock()

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

        self.get_logger().info(
            f'ProjectionSAM3Node initialized '
            f'(conf_rack={self.conf_rack}, conf_obj={self.conf_obj}, '
            f'pad={self.crop_padding})'
        )

    def _init_model(self):
        """Initialize dual SAM3 predictors for two-stage inference."""
        if SAM3SemanticPredictor is None:
            self.get_logger().error('SAM3SemanticPredictor not available')
            return

        try:
            # Stage 1: Rack detection (low conf → high recall)
            overrides_rack = dict(
                conf=self.conf_rack,
                task="segment",
                mode="predict",
                model=self.model_path,
                half=True,
                save=False,  # Rack 결과는 저장하지 않음
                imgsz=1092,
            )
            self.predictor_rack = SAM3SemanticPredictor(overrides=overrides_rack)
            self.get_logger().info(f'Stage1 predictor loaded (conf={self.conf_rack})')

            # Stage 2: Object detection (high conf → high precision)
            overrides_obj = dict(
                conf=self.conf_obj,
                task="segment",
                mode="predict",
                model=self.model_path,
                half=True,
                save=True,
                imgsz=1092,
            )
            self.predictor_obj = SAM3SemanticPredictor(overrides=overrides_obj)
            self.get_logger().info(f'Stage2 predictor loaded (conf={self.conf_obj})')

        except Exception as e:
            self.get_logger().error(f'Failed to load SAM3 model: {e}')
            self.predictor_rack = None
            self.predictor_obj = None

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
        """Two-Stage Inference: Rack → Crop → Object Detection."""
        if self.predictor_rack is None or self.predictor_obj is None:
            return

        image_rgb = cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB)
        h_orig, w_orig = image_rgb.shape[:2]
        detections_list = []

        try:
            # ── Stage 1: Rack Detection ──
            self.predictor_rack.set_image(image_rgb)
            results_rack = self.predictor_rack(text=["rack", "shelf"])

            merged_mask = np.zeros((h_orig, w_orig), dtype=bool)
            n_rack_masks = 0

            if results_rack and len(results_rack) > 0:
                res_rack = results_rack[0]
                if hasattr(res_rack, 'masks') and res_rack.masks is not None:
                    masks_data = res_rack.masks.data
                    if hasattr(masks_data, 'cpu'):
                        masks_data = masks_data.cpu().numpy()
                    else:
                        masks_data = np.asarray(masks_data)
                    for mask in masks_data:
                        merged_mask = np.logical_or(merged_mask, mask.astype(bool))
                        n_rack_masks += 1

            # ── Crop & Upscale (or Fallback) ──
            if n_rack_masks > 0 and np.any(merged_mask):
                coords = np.argwhere(merged_mask)
                y0, x0 = coords.min(axis=0)
                y1, x1 = coords.max(axis=0) + 1

                pad = self.crop_padding
                x0 = max(0, x0 - pad)
                y0 = max(0, y0 - pad)
                x1 = min(w_orig, x1 + pad)
                y1 = min(h_orig, y1 + pad)

                self.get_logger().info(
                    f'Stage1: {n_rack_masks} rack masks -> ROI ({x0},{y0})->({x1},{y1})'
                )

                infer_img = image_rgb[y0:y1, x0:x1]
                use_crop = True
            else:
                self.get_logger().warn('Stage1: No rack found -> Fallback to full image')
                infer_img = image_rgb
                x0, y0 = 0, 0
                use_crop = False

            # ── Stage 2: Object Detection ──
            self.predictor_obj.set_image(infer_img)
            results_obj = self.predictor_obj(text=["a box"])

            n_total = 0
            n_valid = 0

            if results_obj and len(results_obj) > 0:
                res_obj = results_obj[0]

                if hasattr(res_obj, 'masks') and res_obj.masks is not None:
                    masks_data = res_obj.masks.data
                    if hasattr(masks_data, 'cpu'):
                        masks_data = masks_data.cpu().numpy()
                    else:
                        masks_data = np.asarray(masks_data)

                    # ── PHASE 1: Extract pixel-wise confidence maps ──
                    confidence_maps = None
                    try:
                        # Option 1: Try to get probability maps (preferred)
                        if hasattr(res_obj.masks, 'probs') and res_obj.masks.probs is not None:
                            confidence_maps = res_obj.masks.probs
                            if hasattr(confidence_maps, 'cpu'):
                                confidence_maps = confidence_maps.cpu().numpy()
                            else:
                                confidence_maps = np.asarray(confidence_maps)
                            self.get_logger().debug(f'[PHASE1] Extracted masks.probs: shape {confidence_maps.shape}')

                        # Option 2: Try to convert logits to probabilities
                        elif hasattr(res_obj.masks, 'logits') and res_obj.masks.logits is not None:
                            import torch
                            logits = res_obj.masks.logits
                            if hasattr(logits, 'cpu'):
                                logits = logits.cpu()
                            # Apply softmax across the first dimension
                            confidence_maps = torch.nn.functional.softmax(logits, dim=0).numpy()
                            self.get_logger().debug(f'[PHASE1] Extracted masks.logits + softmax: shape {confidence_maps.shape}')

                        # Fallback: Use uniform confidence
                        if confidence_maps is None:
                            self.get_logger().debug('[PHASE1] No pixel-wise confidence available, will use uniform weights')

                    except Exception as e:
                        self.get_logger().warn(f'[PHASE1] Error extracting confidence maps: {e}')
                        confidence_maps = None

                    # Get confidence and class info
                    conf_data = None
                    cls_data = None
                    names = getattr(res_obj, 'names', {})

                    if hasattr(res_obj, 'boxes') and res_obj.boxes is not None:
                        if hasattr(res_obj.boxes, 'conf') and res_obj.boxes.conf is not None:
                            conf_data = res_obj.boxes.conf
                            if hasattr(conf_data, 'cpu'):
                                conf_data = conf_data.cpu().numpy()
                            else:
                                conf_data = np.asarray(conf_data)
                        if hasattr(res_obj.boxes, 'cls') and res_obj.boxes.cls is not None:
                            cls_data = res_obj.boxes.cls
                            if hasattr(cls_data, 'cpu'):
                                cls_data = cls_data.cpu().numpy()
                            else:
                                cls_data = np.asarray(cls_data)

                    for mask_idx, mask in enumerate(masks_data):
                        n_total += 1

                        # Store mask and confidence for Phase 2 usage
                        # (will be used in mask_analysis.py for weighted center calculation)
                        mask_confidence_map = None
                        if confidence_maps is not None:
                            try:
                                if len(confidence_maps.shape) == 3:  # (n_det, H, W)
                                    mask_confidence_map = confidence_maps[mask_idx]  # (H, W)
                            except Exception as e:
                                self.get_logger().debug(f'Could not extract confidence map for mask {mask_idx}: {e}')

                        # Confidence check (using per-detection confidence)
                        conf = 1.0
                        if conf_data is not None and len(conf_data) > mask_idx:
                            conf = float(conf_data[mask_idx])

                        if conf < self.conf_obj:
                            continue
                        n_valid += 1

                        # Class name
                        cls_name = "detection"
                        if cls_data is not None and len(cls_data) > mask_idx:
                            cls_id = int(cls_data[mask_idx])
                            if isinstance(names, dict):
                                cls_name = names.get(cls_id, f"class_{cls_id}")
                            elif isinstance(names, list) and cls_id < len(names):
                                cls_name = names[cls_id]
                            else:
                                cls_name = f"class_{cls_id}"

                        # BBox from mask
                        points = np.where(mask > 0)
                        if len(points[0]) == 0:
                            continue
                        ymin, ymax = points[0].min(), points[0].max()
                        xmin, xmax = points[1].min(), points[1].max()

                        # ── Coordinate Restoration ──
                        if use_crop:
                            xmin_orig = xmin + x0
                            xmax_orig = xmax + x0
                            ymin_orig = ymin + y0
                            ymax_orig = ymax + y0
                        else:
                            xmin_orig, xmax_orig = xmin, xmax
                            ymin_orig, ymax_orig = ymin, ymax

                        det = Detection2D()
                        det.bbox.center.position.x = float((xmin_orig + xmax_orig) / 2.0)
                        det.bbox.center.position.y = float((ymin_orig + ymax_orig) / 2.0)
                        det.bbox.size_x = float(xmax_orig - xmin_orig)
                        det.bbox.size_y = float(ymax_orig - ymin_orig)

                        hyp = ObjectHypothesisWithPose()
                        hyp.hypothesis.class_id = cls_name
                        hyp.hypothesis.score = float(conf)
                        det.results.append(hyp)

                        # Debug: Log if confidence map available for this detection
                        if mask_confidence_map is not None:
                            conf_map_mean = np.mean(mask_confidence_map)
                            self.get_logger().debug(
                                f'[PHASE1] Detection {mask_idx}: confidence_map shape {mask_confidence_map.shape}, mean={conf_map_mean:.3f}'
                            )

                        detections_list.append(det)

                        # Store mask for Phase 5 (6DOF node)
                        # Mask will be accessed by detection index
                        with self.masks_lock:
                            self.latest_masks.append(mask.astype(bool))

            self.get_logger().info(
                f'Stage2: {n_valid}/{n_total} objects (conf>={self.conf_obj})'
            )

            # PHASE 1 Status Summary
            if confidence_maps is not None:
                self.get_logger().info(
                    f'[PHASE1] ✓ Confidence maps extracted: shape {confidence_maps.shape}'
                )
            else:
                self.get_logger().info(
                    f'[PHASE1] Confidence maps not available - will use uniform weights in Phase 2'
                )

        except Exception as e:
            self.get_logger().error(f'SAM3 inference error: {e}')

        # Publish
        detection_array = Detection2DArray()
        detection_array.header = header
        detection_array.detections = detections_list

        # Store masks in sync with detections (for Phase 5/6DOF node)
        with self.masks_lock:
            self.latest_masks = getattr(self, 'current_batch_masks', [])

        self.pub_detections.publish(detection_array)

        debug_msg = String()
        debug_msg.data = f'Detections: {len(detections_list)}'
        self.pub_debug.publish(debug_msg)

        # CSV 저장
        self._save_csv(header, detections_list)


    # CSV 저장을 위한 코드
    def _save_csv(self, header, detections_list):
        """Save detections to CSV file in runs/segment/predictN."""
        if not detections_list:
            return
        try:
            # 첫 호출 시 최신 predictN 폴더 찾기
            if self.csv_path is None:
                self._init_csv_path()
                if self.csv_path is None:
                    return

            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                for idx, det in enumerate(detections_list):
                    conf = det.results[0].hypothesis.score if det.results else 0.0
                    cls_id = det.results[0].hypothesis.class_id if det.results else ''
                    writer.writerow([
                        f'{stamp:.6f}',
                        idx,
                        f'{det.bbox.center.position.x:.2f}',
                        f'{det.bbox.center.position.y:.2f}',
                        f'{det.bbox.size_x:.2f}',
                        f'{det.bbox.size_y:.2f}',
                        f'{conf:.4f}',
                        cls_id
                    ])
            self.get_logger().debug(f'CSV saved: {len(detections_list)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'CSV save error: {e}')

    # CSV 저장을 위한 코드
    def _init_csv_path(self):
        """Find latest runs/segment/predictN and create CSV there."""
        if not os.path.isdir(self.csv_base_dir):
            self.get_logger().warn(f'Segment dir not found: {self.csv_base_dir}')
            return
        # predictN 폴더 중 가장 최신(숫자가 큰) 것 찾기
        predict_dirs = []
        for d in os.listdir(self.csv_base_dir):
            full = os.path.join(self.csv_base_dir, d)
            if os.path.isdir(full) and d.startswith('predict'):
                suffix = d[len('predict'):]
                num = int(suffix) if suffix.isdigit() else 0
                predict_dirs.append((num, full))
        if not predict_dirs:
            self.get_logger().warn('No predictN folders found yet')
            return
        predict_dirs.sort(key=lambda x: x[0], reverse=True)
        latest_dir = predict_dirs[0][1]
        self.csv_path = os.path.join(latest_dir, 'detections.csv')
        # 헤더 작성
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'detection_id', 'center_x', 'center_y',
                             'size_x', 'size_y', 'confidence', 'class_id'])
        self.get_logger().info(f'CSV initialized: {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionSAM3Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
