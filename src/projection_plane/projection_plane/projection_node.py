
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import datetime
import os

from sensor_msgs.msg import PointCloud2, PointField
from shape_msgs.msg import Plane
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, DurabilityPolicy

from projection_plane.core import (
    validate_plane,
    normalize,
    choose_up_hint,
    build_basis,
    load_point_cloud,
    project_points,
    compute_origin,
    map_uv,
    compute_depth,
    compute_image_size,
    rasterize
)

class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        
        # Declare parameters
        self.declare_parameter('plane_n', [0.0, 0.0, 1.0])
        self.declare_parameter('plane_d', 0.0)
        self.declare_parameter('input_file', '/home/jack/ros2_ws/project_hj_v2/241108_converted - Cloud.ply')
        self.declare_parameter('output_file', '/home/jack/ros2_ws/output/projection_ros{timestamp}.png')
        self.declare_parameter('pixels_per_unit', 500.0)
        self.declare_parameter('origin_mode', 'mean')
        self.declare_parameter('depth_priority_far', False)
        
        # Get parameters
        self.plane_n = self.get_parameter('plane_n').value
        self.plane_d = self.get_parameter('plane_d').value
        self.input_file = self.get_parameter('input_file').value
        self.output_file = self.get_parameter('output_file').value
        self.pixels_per_unit = self.get_parameter('pixels_per_unit').value
        self.origin_mode = self.get_parameter('origin_mode').value
        self.depth_priority_far = self.get_parameter('depth_priority_far').value
        
        if not self.input_file:
            self.get_logger().error("Parameter 'input_file' is required.")
            return

        # Publisher for input cloud
        # Use TRANSIENT_LOCAL durability so late joiners (like RViz) get the message
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_cloud = self.create_publisher(PointCloud2, 'input_cloud', qos_profile)

        # Subscriber for plane
        self.sub_plane = self.create_subscription(
            Plane,
            'input_plane',
            self.plane_callback,
            10
        )

        self.run_projection()

    def plane_callback(self, msg):
        self.get_logger().info(f"Received new plane: {msg.coef}")
        # msg.coef is [a, b, c, d] for ax + by + cz + d = 0
        self.plane_n = [msg.coef[0], msg.coef[1], msg.coef[2]]
        self.plane_d = msg.coef[3]
        self.run_projection()

    def run_projection(self):
        self.get_logger().info(f"Starting projection for {self.input_file}")
        
        # 1. Plane setup
        try:
            n_vec = np.array(self.plane_n, dtype=np.float64)
            if n_vec.shape != (3,):
                raise ValueError(f"Normal vector must be 3 elements, got {n_vec.shape}")
            
            n = validate_plane(n_vec[0], n_vec[1], n_vec[2], self.plane_d)
            n_hat = normalize(n)
            
            self.get_logger().info(f"Plane: {n_vec[0]}x + {n_vec[1]}y + {n_vec[2]}z + {self.plane_d} = 0")
            
            # 2. Basis
            up = choose_up_hint(n_hat)
            t1, t2 = build_basis(n_hat, up)
            
            # 3. Load cloud
            points, colors = load_point_cloud(self.input_file)
            self.get_logger().info(f"Loaded {len(points)} points")
            
            # Publish PointCloud2
            try:
                header = Header()
                header.frame_id = "map"
                header.stamp = self.get_clock().now().to_msg()
                
                # Convert to float32 for standard PointCloud2
                points_f32 = points.astype(np.float32)
                pc2_msg = point_cloud2.create_cloud_xyz32(header, points_f32)
                
                self.pub_cloud.publish(pc2_msg)
                self.get_logger().info(f"Published {len(points)} points to /input_cloud")
            except Exception as e:
                self.get_logger().error(f"Failed to publish point cloud: {e}")
            
            # 4. Project
            points_proj = project_points(points, n, self.plane_d)
            
            # 5. Origin
            origin = compute_origin(points_proj, n, self.plane_d, self.origin_mode)
            
            # 6. UV mapping
            u, v = map_uv(points_proj, origin, t1, t2)
            
            # 7. Depth
            depth = compute_depth(points, n, self.plane_d, 'abs')
            
            # 8. Image size
            W, H, u_min, u_max, v_min, v_max = compute_image_size(
                u, v, self.pixels_per_unit
            )
            self.get_logger().info(f"Image size: {W}x{H}")
            
            # 9. Rasterize
            image, pixels_written = rasterize(
                u, v, depth, colors,
                W, H, u_min, u_max, v_min, v_max,
                self.pixels_per_unit, self.depth_priority_far
            )
            
            coverage = pixels_written / (W * H) * 100
            self.get_logger().info(f"Coverage: {coverage:.2f}%")
            
            # 10. Save
            if not self.output_file:
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                final_output_file = f"projection_ros_{timestamp}.png"
            else:
                # Handle {timestamp} placeholder if present
                if '{timestamp}' in self.output_file:
                    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                    try:
                        final_output_file = self.output_file.format(timestamp=timestamp)
                    except Exception:
                        # Fallback if format fails (e.g. other braces)
                        final_output_file = self.output_file.replace('{timestamp}', timestamp)
                else:
                    final_output_file = self.output_file

            # Ensure directory exists
            output_dir = os.path.dirname(final_output_file)
            if output_dir and not os.path.exists(output_dir):
                try:
                    os.makedirs(output_dir)
                    self.get_logger().info(f"Created output directory: {output_dir}")
                except OSError as e:
                    self.get_logger().error(f"Failed to create output directory: {e}")

            # cv2.imwrite might fail if extension is missing or invalid
            if not os.path.splitext(final_output_file)[1]:
                 final_output_file += ".png"
            
            success = cv2.imwrite(final_output_file, image)
            if success:
                self.get_logger().info(f"Saved output to {final_output_file}")
            else:
                self.get_logger().error(f"Failed to save image to {final_output_file}. check path permissions or extension.")
            
        except Exception as e:
            self.get_logger().error(f"Projection failed: {str(e)}")
            import traceback
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    node = ProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
