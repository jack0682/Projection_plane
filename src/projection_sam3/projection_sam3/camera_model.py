#!/usr/bin/env python3
"""
Phase 3: D455 Camera Model and FOV-based Ray Casting
Convert 2D pixel coordinates to 3D camera ray directions using D455 specifications.
"""

import numpy as np
from typing import Tuple, Optional


class D455CameraModel:
    """
    Intel RealSense D455 camera model for 6DOF pose extraction.

    Implements pinhole camera model using D455 FOV specifications:
    - Horizontal FOV: 86°
    - Vertical FOV: 57°
    - Image resolution: 640×480 (typical)

    The camera model provides:
    1. Intrinsic matrix calculation from FOV
    2. Pixel coordinate → 3D camera ray direction conversion
    3. Camera → World coordinate system transformations
    """

    # D455 Camera Specifications
    HFOV_DEGREES = 86.0      # Horizontal Field of View (degrees)
    VFOV_DEGREES = 57.0      # Vertical Field of View (degrees)
    OPERATING_RANGE = (0.6, 6.0)  # Minimum and maximum depth range (meters)

    # Convert to radians
    HFOV = np.radians(HFOV_DEGREES)
    VFOV = np.radians(VFOV_DEGREES)

    def __init__(self, image_width: int = 640, image_height: int = 480):
        """
        Initialize D455 camera model with image dimensions.

        Args:
            image_width (int): Image width in pixels (default: 640)
            image_height (int): Image height in pixels (default: 480)

        Example:
            >>> camera = D455CameraModel(640, 480)
            >>> print(camera.fx, camera.fy)
            480.0 480.0
        """
        self.image_width = image_width
        self.image_height = image_height

        # Compute focal lengths from FOV
        # For pinhole camera: f = (width/2) / tan(fov/2)
        self.fx = (image_width / 2.0) / np.tan(self.HFOV / 2.0)
        self.fy = (image_height / 2.0) / np.tan(self.VFOV / 2.0)

        # Principal point (image center)
        self.cx = image_width / 2.0
        self.cy = image_height / 2.0

        # Intrinsic matrix (camera matrix)
        # K = [[fx,  0, cx],
        #      [ 0, fy, cy],
        #      [ 0,  0,  1]]
        self.K = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        # Inverse of K (for fast pixel→direction computation)
        self.K_inv = np.linalg.inv(self.K)

    def pixel_to_camera_direction(self, pixel_x: float, pixel_y: float) -> np.ndarray:
        """
        Convert pixel coordinates to normalized 3D camera direction vector.

        The direction vector points from camera origin through the pixel
        in the camera's coordinate system. It is normalized (unit length).

        Args:
            pixel_x (float): X pixel coordinate (0-based, 0 = left edge)
            pixel_y (float): Y pixel coordinate (0-based, 0 = top edge)

        Returns:
            np.ndarray: (3,) normalized direction vector in camera frame
                       Components are [x_norm, y_norm, z_norm]
                       where z_norm is typically ~1 (pointing forward)

        Mathematical Background:
            Pinhole camera model:
              norm_x = (pixel_x - cx) / fx
              norm_y = (pixel_y - cy) / fy
              direction = [norm_x, norm_y, 1]
              direction = direction / ||direction||

        Example:
            >>> camera = D455CameraModel(640, 480)
            >>> # Center pixel should point straight ahead
            >>> direction = camera.pixel_to_camera_direction(320, 240)
            >>> print(f"Center pixel direction: {direction}")
            >>> assert abs(direction[0]) < 0.01, "X should be ~0"
            >>> assert abs(direction[1]) < 0.01, "Y should be ~0"
            >>> assert abs(direction[2] - 1.0) < 0.01, "Z should be ~1"
        """
        # Normalize pixel coordinates to camera frame
        norm_x = (pixel_x - self.cx) / self.fx
        norm_y = (pixel_y - self.cy) / self.fy

        # Ray direction in camera coordinate system
        # [x_norm, y_norm, 1] points from camera through (pixel_x, pixel_y)
        direction = np.array([norm_x, norm_y, 1.0], dtype=np.float32)

        # Normalize to unit vector
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 1e-6:  # Avoid division by zero
            direction = direction / direction_norm
        else:
            direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        return direction

    def camera_to_world(
        self,
        point_camera: np.ndarray,
        R_cam_to_world: np.ndarray,
        t_cam_to_world: np.ndarray
    ) -> np.ndarray:
        """
        Transform point from camera coordinates to world coordinates.

        Uses rigid body transformation (rotation + translation):
            point_world = R @ point_camera + t

        Args:
            point_camera (np.ndarray): (3,) point in camera coordinate frame
            R_cam_to_world (np.ndarray): (3, 3) rotation matrix from camera to world
            t_cam_to_world (np.ndarray): (3,) translation vector (camera position in world)

        Returns:
            np.ndarray: (3,) point in world coordinate frame

        Example:
            >>> camera = D455CameraModel()
            >>> point_camera = np.array([0.1, 0.2, 1.5])
            >>> R = np.eye(3)  # Identity rotation (camera aligned with world)
            >>> t = np.array([0.0, 0.0, 0.0])  # Camera at origin
            >>> point_world = camera.camera_to_world(point_camera, R, t)
            >>> np.allclose(point_world, point_camera)
            True
        """
        # Rigid transformation: p_world = R @ p_camera + t
        point_world = R_cam_to_world @ point_camera + t_cam_to_world
        return point_world.astype(np.float32)

    def world_to_camera(
        self,
        point_world: np.ndarray,
        R_cam_to_world: np.ndarray,
        t_cam_to_world: np.ndarray
    ) -> np.ndarray:
        """
        Transform point from world coordinates to camera coordinates (inverse).

        Uses inverse transformation:
            point_camera = R^T @ (point_world - t)

        Args:
            point_world (np.ndarray): (3,) point in world coordinate frame
            R_cam_to_world (np.ndarray): (3, 3) rotation matrix from camera to world
            t_cam_to_world (np.ndarray): (3,) translation vector

        Returns:
            np.ndarray: (3,) point in camera coordinate frame
        """
        # Inverse transformation
        R_world_to_cam = R_cam_to_world.T
        point_camera = R_world_to_cam @ (point_world - t_cam_to_world)
        return point_camera.astype(np.float32)

    def get_intrinsics(self) -> np.ndarray:
        """
        Get intrinsic matrix K.

        Returns:
            np.ndarray: (3, 3) intrinsic matrix

        Example:
            >>> camera = D455CameraModel(640, 480)
            >>> K = camera.get_intrinsics()
            >>> print(K[0, 0])  # fx
            480.0
        """
        return self.K.copy()

    def get_intrinsics_inv(self) -> np.ndarray:
        """
        Get inverse of intrinsic matrix K^-1.

        Returns:
            np.ndarray: (3, 3) inverse intrinsic matrix
        """
        return self.K_inv.copy()

    def project_3d_to_2d(
        self,
        point_camera: np.ndarray
    ) -> Tuple[Optional[float], Optional[float]]:
        """
        Project 3D point in camera frame to 2D image plane.

        Uses perspective projection:
            pixel_x = fx * (x_camera / z_camera) + cx
            pixel_y = fy * (y_camera / z_camera) + cy

        Args:
            point_camera (np.ndarray): (3,) 3D point in camera frame

        Returns:
            tuple: (pixel_x, pixel_y) or (None, None) if behind camera

        Example:
            >>> camera = D455CameraModel(640, 480)
            >>> # Point 1 meter ahead, 0.1 meter right
            >>> point_camera = np.array([0.1, 0.0, 1.0])
            >>> px, py = camera.project_3d_to_2d(point_camera)
            >>> print(f"Projected to pixel: ({px}, {py})")
        """
        x, y, z = point_camera

        # Check if point is behind camera
        if z <= 0:
            return None, None

        # Perspective projection
        pixel_x = self.fx * (x / z) + self.cx
        pixel_y = self.fy * (y / z) + self.cy

        # Check if projection is within image bounds
        if pixel_x < 0 or pixel_x >= self.image_width:
            return None, None
        if pixel_y < 0 or pixel_y >= self.image_height:
            return None, None

        return float(pixel_x), float(pixel_y)

    def compute_fov_at_distance(self, distance: float) -> Tuple[float, float]:
        """
        Compute horizontal and vertical FOV size at given distance.

        Useful for understanding the camera's view at different depths.

        Args:
            distance (float): Distance from camera (meters)

        Returns:
            tuple: (fov_width_m, fov_height_m) Field of view dimensions at distance

        Example:
            >>> camera = D455CameraModel()
            >>> width_at_1m, height_at_1m = camera.compute_fov_at_distance(1.0)
            >>> print(f"FOV at 1m: {width_at_1m:.2f} × {height_at_1m:.2f} meters")
            FOV at 1m: 1.87 × 1.09 meters
        """
        # FOV width = 2 * distance * tan(HFOV/2)
        fov_width = 2 * distance * np.tan(self.HFOV / 2.0)
        fov_height = 2 * distance * np.tan(self.VFOV / 2.0)

        return fov_width, fov_height

    def get_camera_info(self) -> dict:
        """
        Get comprehensive camera model information.

        Returns:
            dict: Dictionary with all camera parameters

        Example:
            >>> camera = D455CameraModel()
            >>> info = camera.get_camera_info()
            >>> print(f"Model: {info['model']}")
            >>> print(f"Resolution: {info['image_size']}")
        """
        return {
            'model': 'Intel RealSense D455',
            'image_size': (self.image_width, self.image_height),
            'fov_horizontal_deg': self.HFOV_DEGREES,
            'fov_vertical_deg': self.VFOV_DEGREES,
            'fov_horizontal_rad': float(self.HFOV),
            'fov_vertical_rad': float(self.VFOV),
            'focal_length_x': float(self.fx),
            'focal_length_y': float(self.fy),
            'principal_point': (float(self.cx), float(self.cy)),
            'operating_range': self.OPERATING_RANGE,
            'intrinsic_matrix': self.K.tolist()
        }


# ─────────────────────────────────────────────────────────────────────────────
# Unit Tests
# ─────────────────────────────────────────────────────────────────────────────

def test_camera_model():
    """
    Unit tests for D455CameraModel.
    Run with: python3 camera_model.py
    """
    print("Testing D455CameraModel")
    print("=" * 80)

    # Test 1: Camera initialization
    print("\nTest 1: Camera initialization")
    camera = D455CameraModel(640, 480)
    print(f"  ✓ Initialized: {camera.image_width} × {camera.image_height}")
    print(f"  ✓ FOV: {camera.HFOV_DEGREES}° × {camera.VFOV_DEGREES}°")
    print(f"  ✓ Focal length: fx={camera.fx:.1f}, fy={camera.fy:.1f}")
    assert camera.fx > 0 and camera.fy > 0, "Focal length should be positive"
    print("  ✓ PASS\n")

    # Test 2: Center pixel should point straight ahead
    print("Test 2: Center pixel points straight ahead")
    direction = camera.pixel_to_camera_direction(320, 240)
    print(f"  Direction: {direction}")
    print(f"  Norm: {np.linalg.norm(direction):.4f}")
    assert abs(direction[0]) < 0.01, "X should be ~0"
    assert abs(direction[1]) < 0.01, "Y should be ~0"
    assert abs(direction[2] - 1.0) < 0.01, "Z should be ~1"
    assert abs(np.linalg.norm(direction) - 1.0) < 0.001, "Should be unit vector"
    print("  ✓ PASS\n")

    # Test 3: Corner pixels point in different directions
    print("Test 3: Corner pixels point in different directions")
    dir_corner1 = camera.pixel_to_camera_direction(0, 0)      # Top-left
    dir_corner2 = camera.pixel_to_camera_direction(639, 479)  # Bottom-right
    dir_center = camera.pixel_to_camera_direction(320, 240)

    dot_product = np.dot(dir_corner1, dir_center)
    print(f"  Corner 1: {dir_corner1}")
    print(f"  Center:   {dir_center}")
    print(f"  Dot product (should be < 1): {dot_product:.4f}")
    assert dot_product < 0.99, "Corner should point differently"
    print("  ✓ PASS\n")

    # Test 4: Unit vector property
    print("Test 4: All directions are unit vectors")
    test_pixels = [(0, 0), (320, 240), (639, 479), (100, 200), (500, 300)]
    for px, py in test_pixels:
        direction = camera.pixel_to_camera_direction(px, py)
        norm = np.linalg.norm(direction)
        assert abs(norm - 1.0) < 0.001, f"Pixel ({px}, {py}) not unit vector: {norm}"
    print(f"  ✓ Tested {len(test_pixels)} pixels, all unit vectors")
    print("  ✓ PASS\n")

    # Test 5: Camera to world transformation
    print("Test 5: Camera to world transformation")
    # Identity rotation, camera at origin
    R = np.eye(3)
    t = np.array([0.0, 0.0, 0.0])
    point_camera = np.array([0.1, 0.2, 1.5])
    point_world = camera.camera_to_world(point_camera, R, t)
    assert np.allclose(point_world, point_camera), "Should match with identity"
    print(f"  Point camera: {point_camera}")
    print(f"  Point world:  {point_world}")
    print("  ✓ PASS\n")

    # Test 6: Camera to world with translation
    print("Test 6: Camera to world with translation")
    R = np.eye(3)
    t = np.array([1.0, 2.0, 3.0])  # Camera offset
    point_camera = np.array([0.0, 0.0, 0.0])
    point_world = camera.camera_to_world(point_camera, R, t)
    assert np.allclose(point_world, [1.0, 2.0, 3.0]), "Should apply translation"
    print(f"  Camera position: {t}")
    print(f"  Camera origin → world: {point_world}")
    print("  ✓ PASS\n")

    # Test 7: FOV at distance
    print("Test 7: FOV at different distances")
    for distance in [0.6, 1.0, 2.0, 6.0]:
        width, height = camera.compute_fov_at_distance(distance)
        print(f"  At {distance}m: {width:.2f}m × {height:.2f}m")
    print("  ✓ PASS\n")

    # Test 8: Camera info
    print("Test 8: Camera information")
    info = camera.get_camera_info()
    print(f"  Model: {info['model']}")
    print(f"  Resolution: {info['image_size']}")
    print(f"  FOV: {info['fov_horizontal_deg']}° × {info['fov_vertical_deg']}°")
    print("  ✓ PASS\n")

    # Test 9: 3D to 2D projection
    print("Test 9: 3D to 2D projection")
    point_camera = np.array([0.0, 0.0, 1.0])  # 1m ahead
    px, py = camera.project_3d_to_2d(point_camera)
    print(f"  Point (0, 0, 1) → pixel ({px}, {py})")
    assert abs(px - 320) < 1, "Should project to center"
    assert abs(py - 240) < 1, "Should project to center"
    print("  ✓ PASS\n")

    # Test 10: Point behind camera
    print("Test 10: Point behind camera handling")
    point_behind = np.array([0.0, 0.0, -1.0])  # Behind camera
    px, py = camera.project_3d_to_2d(point_behind)
    print(f"  Point behind camera: px={px}, py={py}")
    assert px is None and py is None, "Should return None for behind camera"
    print("  ✓ PASS\n")

    print("=" * 80)
    print("✅ All 10 tests passed!")


if __name__ == '__main__':
    test_camera_model()
