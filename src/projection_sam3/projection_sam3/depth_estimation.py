#!/usr/bin/env python3
"""
Phase 4: Depth Estimation using Point Cloud Ray Casting
Estimate depth along camera rays by finding nearest point cloud intersections.
"""

import numpy as np
from typing import Tuple, Optional, List
from scipy.spatial import cKDTree
import time


class DepthEstimator:
    """
    Estimate depth for camera rays using point cloud ray casting.

    Given a camera ray (origin + direction), finds the closest point in
    a point cloud that lies along or near that ray, providing depth estimate.

    Algorithm:
        1. Build KD-tree index of point cloud (one-time, fast)
        2. For each ray:
           - Query points within search radius of camera origin
           - Filter: keep only points in front of camera
           - For each candidate: compute distance from ray
           - Return point with minimum ray distance
"""

    def __init__(self, point_cloud: np.ndarray, max_depth: float = 6.0):
        """
        Initialize depth estimator with point cloud.

        Args:
            point_cloud (np.ndarray): (N, 3) array of 3D points in world frame
            max_depth (float): Maximum valid depth in meters (D455: 0.6-6.0m)

        Example:
            >>> points = np.random.rand(10000, 3)
            >>> estimator = DepthEstimator(points, max_depth=6.0)
        """
        self.points = np.asarray(point_cloud, dtype=np.float32)
        self.max_depth = max_depth
        self.min_depth = 0.1  # Minimum depth (must be > 0 to be in front of camera)

        print(f"[DepthEstimator] Building KD-tree for {len(self.points):,} points...")
        start_time = time.time()

        # Build KD-tree for efficient spatial queries
        # This is a one-time cost (~10-20 seconds for 14.6M points)
        self.kdtree = cKDTree(self.points)

        elapsed = time.time() - start_time
        print(f"[DepthEstimator] KD-tree built in {elapsed:.2f} seconds ✓")

    def estimate_depth(
        self,
        camera_origin: np.ndarray,
        camera_direction: np.ndarray,
        search_radius: float = 0.5
    ) -> Tuple[float, float]:
        """
        Estimate depth along a camera ray using point cloud.

        Algorithm:
            1. Query points within search_radius of camera origin
            2. Filter points in front of camera (depth > 0)
            3. Compute perpendicular distance from each point to ray
            4. Return depth of point with minimum distance from ray

        Args:
            camera_origin (np.ndarray): (3,) camera position in world frame
            camera_direction (np.ndarray): (3,) normalized direction vector
            search_radius (float): Search radius in meters (default: 0.5m)

        Returns:
            tuple: (depth, distance_from_ray)
                - depth (float): Distance from camera along direction
                - distance_from_ray (float): Perpendicular distance (0 is ideal)

        Example:
            >>> origin = np.array([0.0, 0.0, 0.0])
            >>> direction = np.array([0.0, 0.0, 1.0])  # Looking forward
            >>> depth, dist = estimator.estimate_depth(origin, direction)
            >>> print(f"Depth: {depth:.2f}m, Distance from ray: {dist:.4f}m")
        """
        # Ensure inputs are numpy arrays
        camera_origin = np.asarray(camera_origin, dtype=np.float32)
        camera_direction = np.asarray(camera_direction, dtype=np.float32)

        # Query: Find all points within search_radius of camera origin
        try:
            indices = self.kdtree.query_ball_point(camera_origin, search_radius)
        except Exception as e:
            # Fallback if KD-tree query fails
            print(f"[DepthEstimator] KD-tree query failed: {e}")
            return self.max_depth, float('inf')

        # Handle case where no points found nearby
        if len(indices) == 0:
            return self.max_depth, float('inf')

        # Get candidate points
        nearby_points = self.points[indices]

        # For each point, compute:
        # 1. Projection on ray: depth = (point - origin) · direction
        # 2. Distance from ray: ||point - (origin + depth * direction)||

        # Vector from camera to each point
        vectors = nearby_points - camera_origin  # (n, 3)

        # Depth (distance along ray direction)
        depths = np.dot(vectors, camera_direction)  # (n,)

        # Filter: keep only points in front of camera
        # (depth must be positive and within valid range)
        valid_mask = (depths > self.min_depth) & (depths < self.max_depth)

        if not np.any(valid_mask):
            # No valid points found
            return self.max_depth, float('inf')

        valid_depths = depths[valid_mask]
        valid_points = nearby_points[valid_mask]

        # Compute closest point on ray for each valid point
        # closest_point = origin + depth * direction
        projections = camera_origin + valid_depths[:, np.newaxis] * camera_direction

        # Distance from actual point to its projection on ray
        distances_from_ray = np.linalg.norm(valid_points - projections, axis=1)

        # Find point with minimum distance from ray
        best_idx = np.argmin(distances_from_ray)
        best_depth = valid_depths[best_idx]
        best_distance_from_ray = distances_from_ray[best_idx]

        return float(best_depth), float(best_distance_from_ray)

    def estimate_depth_batch(
        self,
        camera_origin: np.ndarray,
        camera_directions: np.ndarray,
        search_radius: float = 0.5
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate depths for multiple rays efficiently.

        Args:
            camera_origin (np.ndarray): (3,) camera position (shared for all rays)
            camera_directions (np.ndarray): (n, 3) array of normalized directions
            search_radius (float): Search radius in meters

        Returns:
            tuple: (depths, distances_from_ray)
                - depths (np.ndarray): (n,) array of depth estimates
                - distances_from_ray (np.ndarray): (n,) perpendicular distances

        Example:
            >>> directions = np.array([[0,0,1], [0.1,0,1], [-0.1,0,1]])
            >>> depths, dists = estimator.estimate_depth_batch(origin, directions)
        """
        depths = []
        distances = []

        for direction in camera_directions:
            # Normalize direction (safety check)
            direction_norm = np.linalg.norm(direction)
            if direction_norm < 1e-6:
                direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            else:
                direction = direction / direction_norm

            d, dr = self.estimate_depth(camera_origin, direction, search_radius)
            depths.append(d)
            distances.append(dr)

        return np.array(depths, dtype=np.float32), np.array(distances, dtype=np.float32)

    def estimate_depth_adaptive(
        self,
        camera_origin: np.ndarray,
        camera_direction: np.ndarray,
        initial_radius: float = 0.5,
        max_iterations: int = 3
    ) -> Tuple[float, float]:
        """
        Estimate depth with adaptive search radius (retry with larger radius if needed).

        Useful when initial search radius is too small and no points found.

        Args:
            camera_origin (np.ndarray): (3,) camera position
            camera_direction (np.ndarray): (3,) normalized direction
            initial_radius (float): Initial search radius in meters
            max_iterations (int): Maximum search radius increase attempts

        Returns:
            tuple: (depth, distance_from_ray)
        """
        radius = initial_radius

        for iteration in range(max_iterations):
            depth, dist_from_ray = self.estimate_depth(
                camera_origin, camera_direction, radius
            )

            if dist_from_ray < float('inf'):
                # Found valid point
                return depth, dist_from_ray

            # No points found, increase search radius
            radius *= 2.0

        # Final fallback
        return self.max_depth, float('inf')

    def get_statistics(self) -> dict:
        """
        Get point cloud statistics for debugging/analysis.

        Returns:
            dict: Statistics about the point cloud
        """
        return {
            'num_points': len(self.points),
            'bounds_min': self.points.min(axis=0).tolist(),
            'bounds_max': self.points.max(axis=0).tolist(),
            'center': self.points.mean(axis=0).tolist(),
            'max_depth': self.max_depth,
            'min_depth': self.min_depth
        }


# ─────────────────────────────────────────────────────────────────────────────
# Point Cloud I/O
# ─────────────────────────────────────────────────────────────────────────────

def load_point_cloud_ply(ply_path: str) -> np.ndarray:
    """
    Load point cloud from PLY file.

    Args:
        ply_path (str): Path to .ply file

    Returns:
        np.ndarray: (N, 3) array of 3D points

    Example:
        >>> points = load_point_cloud_ply('/path/to/cloud.ply')
        >>> print(f"Loaded {len(points):,} points")
    """
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(ply_path)
        points = np.asarray(pcd.points, dtype=np.float32)
        print(f"[Depth] Loaded {len(points):,} points from {ply_path}")
        return points
    except ImportError:
        print("[Depth] Open3D not installed, using ply_fast_load fallback")
        return ply_fast_load(ply_path)


def ply_fast_load(ply_path: str) -> np.ndarray:
    """
    Fast PLY loader without Open3D dependency.

    Parses PLY header and vertex data directly.

    Args:
        ply_path (str): Path to .ply file

    Returns:
        np.ndarray: (N, 3) array of 3D points
    """
    with open(ply_path, 'rb') as f:
        # Read header
        header = []
        while True:
            line = f.readline().decode('utf-8').strip()
            header.append(line)
            if line == 'end_header':
                break

        # Parse header
        num_vertices = None
        for line in header:
            if line.startswith('element vertex'):
                num_vertices = int(line.split()[-1])

        if num_vertices is None:
            raise ValueError("Could not find vertex count in PLY header")

        # Read vertex data
        # Assume format: x y z (float32, 3 floats per vertex)
        vertex_data = f.read(num_vertices * 12)  # 12 bytes = 3 * float32
        points = np.frombuffer(vertex_data, dtype=np.float32)
        points = points.reshape(-1, 3)

        print(f"[Depth] Loaded {len(points):,} points from {ply_path}")
        return points


# ─────────────────────────────────────────────────────────────────────────────
# Unit Tests
# ─────────────────────────────────────────────────────────────────────────────

def test_depth_estimation():
    """
    Unit tests for DepthEstimator.
    Run with: python3 depth_estimation.py
    """
    print("Testing DepthEstimator")
    print("=" * 80)

    # Test 1: Synthetic point cloud
    print("\nTest 1: Synthetic point cloud")
    print("  Creating test point cloud...")

    # Create a simple point cloud (grid of points at z=1m)
    x = np.linspace(-5, 5, 20)
    y = np.linspace(-5, 5, 20)
    z = np.ones(20) * 1.0

    points = []
    for xi in x:
        for yi in y:
            points.append([xi, yi, 1.0])

    points = np.array(points, dtype=np.float32)
    print(f"  Created {len(points)} points")

    # Initialize estimator
    estimator = DepthEstimator(points, max_depth=6.0)
    print("  ✓ DepthEstimator initialized\n")

    # Test 2: Ray pointing forward
    print("Test 2: Ray pointing forward (should find points at z=1m)")
    origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    depth, dist_from_ray = estimator.estimate_depth(origin, direction, search_radius=10.0)
    print(f"  Depth: {depth:.4f}m (expected ~1.0)")
    print(f"  Distance from ray: {dist_from_ray:.6f}m")
    assert 0.9 < depth < 1.1, f"Depth should be ~1.0, got {depth}"
    # Sparse grid means closest point may be ~0.37m away (grid spacing: 0.5m)
    assert dist_from_ray < 0.5, f"Should be reasonably close to ray, got {dist_from_ray}"
    print("  ✓ PASS\n")

    # Test 3: Ray at angle
    print("Test 3: Ray at angle")
    direction_angled = np.array([0.1, 0.0, 1.0], dtype=np.float32)
    direction_angled = direction_angled / np.linalg.norm(direction_angled)

    depth, dist_from_ray = estimator.estimate_depth(origin, direction_angled, search_radius=10.0)
    print(f"  Depth: {depth:.4f}m")
    print(f"  Distance from ray: {dist_from_ray:.6f}m")
    assert 0.5 < depth < 1.5, "Depth should be reasonable"
    print("  ✓ PASS\n")

    # Test 4: Batch processing
    print("Test 4: Batch depth estimation")
    directions = np.array([
        [0.0, 0.0, 1.0],
        [0.1, 0.0, 1.0],
        [-0.1, 0.0, 1.0]
    ], dtype=np.float32)

    for d in directions:
        d_norm = np.linalg.norm(d)
        d /= d_norm

    depths, distances = estimator.estimate_depth_batch(origin, directions, search_radius=10.0)
    print(f"  Depths: {depths}")
    print(f"  Distances: {distances}")
    assert len(depths) == 3, "Should have 3 depth estimates"
    print("  ✓ PASS\n")

    # Test 5: Statistics
    print("Test 5: Point cloud statistics")
    stats = estimator.get_statistics()
    print(f"  Points: {stats['num_points']}")
    print(f"  Bounds: {stats['bounds_min']} → {stats['bounds_max']}")
    print(f"  Center: {stats['center']}")
    print("  ✓ PASS\n")

    # Test 6: Ray behind cloud
    print("Test 6: Ray pointing away from cloud (no intersection)")
    direction_away = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    depth, dist_from_ray = estimator.estimate_depth(origin, direction_away, search_radius=10.0)
    print(f"  Depth: {depth}")
    print(f"  Distance: {dist_from_ray}")
    assert depth == 6.0, "Should return max_depth when no valid points"
    print("  ✓ PASS\n")

    print("=" * 80)
    print("✅ All tests passed!")


if __name__ == '__main__':
    test_depth_estimation()
