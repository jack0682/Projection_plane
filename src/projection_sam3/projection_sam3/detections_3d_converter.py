#!/usr/bin/env python3
"""
3D Detections Converter: Back-project 2D detections to 3D world coordinates.

Subscribes to:
  - /projection/contract: Projection geometry (basis vectors, scale factors)
  - /projection/sam3/detections: 2D object detections

Publishes:
  - /projection/detections_3d: 3D centers as Float64MultiArray
    Format per detection: [id, x, y, z, confidence]
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import message_filters

from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from projection_msgs.msg import ProjectionContract


class Detections3DConverter(Node):
    """Convert 2D projections to 3D detections using ProjectionContract."""

    def __init__(self):
        super().__init__('detections_3d_converter')

        qos = QoSProfile(depth=10)

        # Subscribers with message filters for synchronization
        self.sub_contract_ = message_filters.Subscriber(
            self, ProjectionContract, '/projection/contract', qos_profile=qos)
        self.sub_detections_2d_ = message_filters.Subscriber(
            self, Detection2DArray, '/projection/sam3/detections', qos_profile=qos)

        # Time synchronizer
        self.ts_ = message_filters.TimeSynchronizer(
            [self.sub_contract_, self.sub_detections_2d_], queue_size=5)
        self.ts_.registerCallback(self.sync_callback)

        # Publisher for 3D detections (as Float64MultiArray)
        self.pub_detections_3d_ = self.create_publisher(
            Float64MultiArray, '/projection/detections_3d', qos_profile=qos)

        self.detection_id_counter_ = 0

        self.get_logger().info('Detections3DConverter node initialized')

    def sync_callback(self, contract_msg: ProjectionContract, detections_2d_msg: Detection2DArray):
        """
        Synchronized callback for contract + 2D detections.
        Back-project 2D detections to 3D using contract geometry.
        """
        try:
            # Extract basis vectors and scale factors from contract
            basis_u = np.array([contract_msg.basis_u.x, contract_msg.basis_u.y, contract_msg.basis_u.z])
            basis_v = np.array([contract_msg.basis_v.x, contract_msg.basis_v.y, contract_msg.basis_v.z])
            basis_n = np.array([contract_msg.basis_n.x, contract_msg.basis_n.y, contract_msg.basis_n.z])

            plane_center = np.array([
                contract_msg.plane_center.x,
                contract_msg.plane_center.y,
                contract_msg.plane_center.z
            ])

            sx_px_per_m = contract_msg.sx_px_per_m
            sy_px_per_m = contract_msg.sy_px_per_m
            ox_px = contract_msg.ox_px
            oy_px = contract_msg.oy_px

            img_width = contract_msg.image_width_px
            img_height = contract_msg.image_height_px

            # Collect 3D detections: [id, x, y, z, confidence]
            detections_3d = []

            for det_idx, det_2d in enumerate(detections_2d_msg.detections):
                center_3d = self._project_detection_to_3d(
                    det_2d, plane_center, basis_u, basis_v, basis_n,
                    sx_px_per_m, sy_px_per_m, ox_px, oy_px, img_width, img_height)

                if center_3d is not None:
                    x_3d, y_3d, z_3d, confidence = center_3d
                    detections_3d.extend([
                        float(det_idx),                 # detection ID
                        x_3d, y_3d, z_3d,              # 3D center
                        confidence                      # confidence score
                    ])

            # Create output message
            msg = Float64MultiArray()
            msg.data = detections_3d
            self.pub_detections_3d_.publish(msg)

            if len(detections_3d) > 0:
                self.get_logger().debug(
                    f"Converted {len(detections_3d) // 5} detections to 3D")

        except Exception as e:
            self.get_logger().error(f"Projection error: {e}")

    def _project_detection_to_3d(
        self,
        det_2d,
        plane_center: np.ndarray,
        basis_u: np.ndarray,
        basis_v: np.ndarray,
        basis_n: np.ndarray,
        sx_px_per_m: float,
        sy_px_per_m: float,
        ox_px: float,
        oy_px: float,
        img_width: int,
        img_height: int,
    ):
        """
        Project a 2D detection to 3D using basis vectors and scale factors.

        Args:
            det_2d: Detection2D message with bbox center and results
            plane_center: 3D plane center point
            basis_u, basis_v, basis_n: Orthonormal basis vectors
            sx_px_per_m, sy_px_per_m: Scale factors (px/m)
            ox_px, oy_px: Pixel origins (center-based)
            img_width, img_height: Image dimensions

        Returns:
            Tuple of (x_3d, y_3d, z_3d, confidence) or None if projection fails
        """
        try:
            # Extract 2D bbox center (in pixel coordinates)
            px_x = det_2d.bbox.center.position.x
            px_y = det_2d.bbox.center.position.y

            # Convert pixel coordinates to plane coordinates (meters)
            # Center-based convention: (0,0) at image center
            dx_px = px_x - ox_px  # pixel offset from center
            dy_px = px_y - oy_px

            # Convert to meters on plane
            u_m = dx_px / sx_px_per_m  # u direction (basis_u)
            v_m = dy_px / sy_px_per_m  # v direction (basis_v)

            # Back-project to 3D
            center_3d = plane_center + u_m * basis_u + v_m * basis_v

            # Extract confidence from detection results
            confidence = 0.5
            if hasattr(det_2d, 'results') and len(det_2d.results) > 0:
                if hasattr(det_2d.results[0], 'hypothesis'):
                    if hasattr(det_2d.results[0].hypothesis, 'score'):
                        confidence = float(det_2d.results[0].hypothesis.score)

            return (float(center_3d[0]), float(center_3d[1]), float(center_3d[2]), confidence)

        except Exception as e:
            self.get_logger().warn(f"Failed to project detection: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = Detections3DConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
