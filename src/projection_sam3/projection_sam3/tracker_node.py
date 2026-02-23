#!/usr/bin/env python3
"""
ROS2 Tracker Node: 3D geometric tracking with TTL and accumulation.

Subscribes to:
  - /projection/sam3/detections (Detection2DArray)
  - /projection/proj_meta (Float64MultiArray)

Publishes:
  - /projection/sam3/tracks (Detection2DArray with track IDs)
  - /projection/sam3/tracks_debug (String)
  - /projection/sam3/inventory_debug (String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time
import threading
from typing import Optional

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from .track_manager import TrackManager, Track, TrackState
from .geometry_utils import (
    compute_bbox_center_3d,
    extract_metadata,
    ema_smooth,
)


class ProjectionTrackerNode(Node):
    """ROS2 node for 3D geometric tracking of 2D detections."""

    def __init__(self):
        super().__init__("projection_tracker_node")

        # Declare parameters
        self.declare_parameter("min_hits", 2)
        self.declare_parameter("ttl_sec", 2.0)
        self.declare_parameter("max_3d_distance", 0.25)
        self.declare_parameter("merge_radius_3d", 0.2)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("allow_unconfirmed_output", False)

        # Get parameters
        min_hits = self.get_parameter("min_hits").value
        ttl_sec = self.get_parameter("ttl_sec").value
        max_3d_distance = self.get_parameter("max_3d_distance").value
        merge_radius_3d = self.get_parameter("merge_radius_3d").value
        ema_alpha = self.get_parameter("ema_alpha").value
        allow_unconfirmed_output = self.get_parameter("allow_unconfirmed_output").value

        # Initialize track manager
        self.track_manager = TrackManager(
            min_hits=min_hits,
            ttl_sec=ttl_sec,
            max_3d_distance=max_3d_distance,
            merge_radius_3d=merge_radius_3d,
            ema_alpha=ema_alpha,
            allow_unconfirmed_output=allow_unconfirmed_output,
        )

        # Thread-safe buffers
        self.latest_detections: Optional[Detection2DArray] = None
        self.latest_metadata: Optional[dict] = None
        self.buffer_lock = threading.Lock()

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create subscribers
        self.create_subscription(
            Detection2DArray,
            "/projection/sam3/detections",
            self._detections_callback,
            qos,
        )

        self.create_subscription(
            Float64MultiArray,
            "/projection/proj_meta",
            self._metadata_callback,
            qos,
        )

        # Create publishers
        self.pub_tracks = self.create_publisher(Detection2DArray, "/projection/sam3/tracks", qos)
        self.pub_tracks_debug = self.create_publisher(String, "/projection/sam3/tracks_debug", qos)
        self.pub_inventory_debug = self.create_publisher(String, "/projection/sam3/inventory_debug", qos)

        # Timer for processing
        self.timer = self.create_timer(0.1, self._process_callback)  # 10 Hz

        self.get_logger().info(
            f"ProjectionTrackerNode initialized "
            f"(min_hits={min_hits}, ttl={ttl_sec}s, max_dist={max_3d_distance}m)"
        )

    def _detections_callback(self, msg: Detection2DArray):
        """Handle incoming detections."""
        with self.buffer_lock:
            self.latest_detections = msg

    def _metadata_callback(self, msg: Float64MultiArray):
        """Handle incoming projection metadata."""
        try:
            metadata = extract_metadata(msg)
            with self.buffer_lock:
                self.latest_metadata = metadata
        except ValueError as e:
            self.get_logger().error(f"Failed to parse metadata: {e}")

    def _process_callback(self):
        """Process detections and update tracks."""
        with self.buffer_lock:
            detections = self.latest_detections
            metadata = self.latest_metadata

        # Skip if missing data
        if detections is None or metadata is None:
            return

        current_time = time.time()

        try:
            # Convert 2D detections to 3D points
            detections_3d = self._convert_detections_to_3d(detections, metadata)

            if len(detections_3d) == 0 and len(detections.detections) > 0:
                self.get_logger().warn(
                    f"All {len(detections.detections)} detections failed 3D conversion"
                )

            # Update tracks
            active_tracks, inventory = self.track_manager.update(detections_3d, current_time)

            # Publish results
            self._publish_tracks(active_tracks, detections.header)
            self._publish_debug_info(active_tracks, inventory)

        except Exception as e:
            import traceback
            self.get_logger().error(f"Tracking error: {e}\n{traceback.format_exc()}")

    def _convert_detections_to_3d(
        self, detections: Detection2DArray, metadata: dict
    ) -> list:
        """Convert 2D detections to 3D points using metadata."""
        detections_3d = []
        failed_count = 0

        for det_idx, det_2d in enumerate(detections.detections):
            try:
                # Extract detection properties
                if not det_2d.results:
                    failed_count += 1
                    continue

                hyp = det_2d.results[0]
                class_name = hyp.hypothesis.class_id
                confidence = hyp.hypothesis.score

                # Validate bbox
                pixel_x = det_2d.bbox.center.position.x
                pixel_y = det_2d.bbox.center.position.y

                if not (0 <= pixel_x < metadata["width"] and 0 <= pixel_y < metadata["height"]):
                    self.get_logger().warn(
                        f"Detection {det_idx}: bbox center ({pixel_x:.1f}, {pixel_y:.1f}) "
                        f"outside image bounds ({metadata['width']}x{metadata['height']})"
                    )
                    failed_count += 1
                    continue

                # Compute 3D center
                center_3d = compute_bbox_center_3d(
                    det_2d,
                    metadata["width"],
                    metadata["height"],
                    metadata["u_min"],
                    metadata["u_max"],
                    metadata["v_min"],
                    metadata["v_max"],
                    metadata["origin"],
                    metadata["t1"],
                    metadata["t2"],
                )

                # Validate 3D point
                import numpy as np
                if np.any(np.isnan(center_3d)) or np.any(np.isinf(center_3d)):
                    self.get_logger().warn(
                        f"Detection {det_idx}: invalid 3D center {center_3d}"
                    )
                    failed_count += 1
                    continue

                detections_3d.append((det_2d, center_3d, class_name, confidence))

            except Exception as e:
                self.get_logger().warn(f"Detection {det_idx}: conversion failed: {e}")
                failed_count += 1

        if failed_count > 0 and len(detections.detections) > 0:
            self.get_logger().debug(
                f"Converted {len(detections_3d)}/{len(detections.detections)} detections "
                f"({failed_count} failed)"
            )

        return detections_3d

    def _publish_tracks(self, tracks: list, header):
        """Publish tracked detections."""
        track_array = Detection2DArray()
        track_array.header = header

        for track in tracks:
            # Reconstruct Detection2D with track ID
            det = Detection2D()
            det.header.frame_id = header.frame_id
            det.header.stamp = header.stamp

            # Set ID from track
            det.id = str(track.track_id)

            # Copy bbox from last detection if available
            if track.last_2d_det is not None:
                det.bbox = track.last_2d_det.bbox
            else:
                # Fallback: set center
                det.bbox.center.position.x = track.center_3d[0]
                det.bbox.center.position.y = track.center_3d[1]

            # Set results
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = track.class_name
            hyp.hypothesis.score = track.confidence
            det.results.append(hyp)

            track_array.detections.append(det)

        self.pub_tracks.publish(track_array)

    def _publish_debug_info(self, tracks: list, inventory: list):
        """Publish debug information."""
        # Tracks debug
        tracks_info = {
            "active_tracks": len(tracks),
            "confirmed": sum(1 for t in tracks if t.state == TrackState.CONFIRMED),
            "tentative": sum(1 for t in tracks if t.state == TrackState.TENTATIVE),
            "details": [
                {
                    "id": t.track_id,
                    "class": t.class_name,
                    "state": t.state.name,
                    "hits": t.hit_count,
                    "conf": round(t.confidence, 3),
                }
                for t in tracks[:10]  # Limit to first 10
            ],
        }

        import json

        tracks_msg = String()
        tracks_msg.data = json.dumps(tracks_info, indent=2)
        self.pub_tracks_debug.publish(tracks_msg)

        # Inventory debug
        inventory_info = {
            "total_items": len(inventory),
            "items": [
                {
                    "id": inv.inventory_id,
                    "class": inv.class_name,
                    "seen_count": inv.seen_count,
                    "last_confidence": round(inv.last_confidence, 3),
                }
                for inv in inventory[:10]  # Limit to first 10
            ],
        }

        inventory_msg = String()
        inventory_msg.data = json.dumps(inventory_info, indent=2)
        self.pub_inventory_debug.publish(inventory_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ProjectionTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
