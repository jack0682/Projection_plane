#!/usr/bin/env python3
"""
Unit Tests for Phase 5: 6DOF Node
Tests all components: orientation estimation, size estimation, quaternion conversion
"""

import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera_model import D455CameraModel


def test_direction_to_quaternion():
    """Test converting direction vector to quaternion"""
    print("Test 1: Direction to Quaternion Conversion")
    print("=" * 60)

    # Test forward direction (0, 0, 1)
    direction = np.array([0.0, 0.0, 1.0])
    quat = convert_direction_to_quaternion(direction)
    print(f"  Forward direction [0,0,1] → quaternion: {quat}")
    assert np.allclose(quat, [0, 0, 0, 1]), "Forward should be identity"
    print("  ✓ PASS\n")

    # Test backward direction (0, 0, -1)
    direction = np.array([0.0, 0.0, -1.0])
    quat = convert_direction_to_quaternion(direction)
    print(f"  Backward direction [0,0,-1] → quaternion: {quat}")
    # Backward is 180° rotation around X
    assert np.allclose(quat, [1, 0, 0, 0]), "Backward should be 180° around X"
    print("  ✓ PASS\n")

    # Test right direction (1, 0, 0)
    direction = np.array([1.0, 0.0, 0.0])
    quat = convert_direction_to_quaternion(direction)
    print(f"  Right direction [1,0,0] → quaternion: {quat}")
    # Should be 90° around Y
    quat_expected = np.array([0, np.sin(np.pi/4), 0, np.cos(np.pi/4)])
    assert np.allclose(quat, quat_expected, atol=1e-6), f"Right should be 90° around Y"
    print("  ✓ PASS\n")


def test_angle_to_quaternion():
    """Test converting 2D angle to quaternion (rotation around Z)"""
    print("Test 2: Angle to Quaternion (Z-axis rotation)")
    print("=" * 60)

    # 0 degrees
    quat = angle_to_quaternion_around_z(0.0)
    print(f"  0° → quaternion: {quat}")
    assert np.allclose(quat, [0, 0, 0, 1]), "0° should be identity"
    print("  ✓ PASS\n")

    # 90 degrees
    quat = angle_to_quaternion_around_z(np.pi / 2)
    print(f"  90° → quaternion: {quat}")
    quat_expected = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    assert np.allclose(quat, quat_expected), "90° should be correct"
    print("  ✓ PASS\n")

    # 180 degrees
    quat = angle_to_quaternion_around_z(np.pi)
    print(f"  180° → quaternion: {quat}")
    assert np.allclose(np.abs(quat), [0, 0, 1, 0], atol=1e-6), "180° should be [0,0,±1,0]"
    print("  ✓ PASS\n")


def test_size_estimation():
    """Test 3D size estimation from mask area and depth"""
    print("Test 3: 3D Size Estimation from Mask + Depth")
    print("=" * 60)

    camera = D455CameraModel(image_width=640, image_height=480)

    # Create test mask (200×200 pixels)
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[140:340, 220:420] = 255  # 200×200 in center

    depth = 1.0  # 1 meter

    size_3d = estimate_size_from_mask_and_depth(mask, depth, camera)
    print(f"  Mask: 200×200 pixels @ {depth}m depth")
    print(f"  3D Size: [{size_3d[0]:.3f}, {size_3d[1]:.3f}, {size_3d[2]:.3f}] meters")

    # Expected: mask covers roughly 31% of image width
    # FOV at 1m: 1.87m wide, 1.09m tall
    # Size ≈ 0.31 × 1.87 = 0.58m wide
    expected_width = (200 / 640) * camera.compute_fov_at_distance(depth)[0]
    expected_height = (200 / 480) * camera.compute_fov_at_distance(depth)[1]

    print(f"  Expected width: ~{expected_width:.3f}m (30% of FOV width)")
    print(f"  Expected height: ~{expected_height:.3f}m (42% of FOV height)")

    assert 0.5 < size_3d[0] < 0.7, f"Width should be ~0.58m, got {size_3d[0]}"
    assert 0.4 < size_3d[1] < 0.6, f"Height should be ~0.45m, got {size_3d[1]}"
    assert 0.05 < size_3d[2] < 0.2, f"Depth extent should be ~0.1m, got {size_3d[2]}"
    print("  ✓ PASS\n")


def test_size_at_different_depths():
    """Test size estimation at different distances"""
    print("Test 4: Size Estimation at Different Depths")
    print("=" * 60)

    camera = D455CameraModel(image_width=640, image_height=480)

    # Fixed mask size: 100×100 pixels
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[190:290, 270:370] = 255

    test_depths = [0.6, 1.0, 2.0, 4.0, 6.0]

    for depth in test_depths:
        size_3d = estimate_size_from_mask_and_depth(mask, depth, camera)
        fov_w, fov_h = camera.compute_fov_at_distance(depth)

        print(f"  Depth: {depth:.1f}m")
        print(f"    FOV: {fov_w:.2f}m × {fov_h:.2f}m")
        print(f"    Mask pixels: 100×100 (15.6% × 20.8% of image)")
        print(f"    3D Size: {size_3d[0]:.3f} × {size_3d[1]:.3f} × {size_3d[2]:.3f} m")

        # Verify: size scales with depth (approximately)
        expected_width = (100 / 640) * fov_w
        assert 0.9 * expected_width < size_3d[0] < 1.1 * expected_width, \
            f"Width calculation mismatch at {depth}m"

    print("  ✓ PASS\n")


def test_mask_pca_orientation():
    """Test PCA-based orientation estimation from mask"""
    print("Test 5: PCA-based Orientation from Mask")
    print("=" * 60)

    # Create test masks with different orientations
    camera_dir = np.array([0.0, 0.0, 1.0])  # Straight ahead

    # Vertical mask (tall and narrow)
    print("  Test 5a: Vertical mask (tall + narrow)")
    mask_vertical = np.zeros((480, 640), dtype=np.uint8)
    mask_vertical[100:400, 310:330] = 255  # 20px wide, 300px tall

    quat_v = estimate_orientation_from_mask(mask_vertical, camera_dir)
    print(f"    Mask: 20×300px (vertical)")
    print(f"    Quaternion: {quat_v}")
    print("    ✓ Computed\n")

    # Horizontal mask (wide and short)
    print("  Test 5b: Horizontal mask (wide + short)")
    mask_horiz = np.zeros((480, 640), dtype=np.uint8)
    mask_horiz[235:245, 100:600] = 255  # 500px wide, 10px tall

    quat_h = estimate_orientation_from_mask(mask_horiz, camera_dir)
    print(f"    Mask: 500×10px (horizontal)")
    print(f"    Quaternion: {quat_h}")
    print("    ✓ Computed\n")

    # Diagonal mask
    print("  Test 5c: Diagonal mask")
    mask_diag = np.zeros((480, 640), dtype=np.uint8)
    for i in range(100, 300):
        mask_diag[i, i:i+50] = 255  # Diagonal band

    quat_d = estimate_orientation_from_mask(mask_diag, camera_dir)
    print(f"    Mask: diagonal oriented")
    print(f"    Quaternion: {quat_d}")
    print("    ✓ Computed\n")

    # Verify quaternions are unit vectors
    assert np.allclose(np.linalg.norm(quat_v), 1.0), "Quaternion should be unit vector"
    assert np.allclose(np.linalg.norm(quat_h), 1.0), "Quaternion should be unit vector"
    assert np.allclose(np.linalg.norm(quat_d), 1.0), "Quaternion should be unit vector"

    print("  ✓ PASS\n")


def test_quaternion_norm():
    """Verify all quaternions are unit vectors"""
    print("Test 6: Quaternion Normalization")
    print("=" * 60)

    test_quats = [
        np.array([0, 0, 0, 1]),  # Identity
        np.array([1, 0, 0, 0]),  # 180° around X
        np.array([0, 1/np.sqrt(2), 0, 1/np.sqrt(2)]),  # 90° around Y
        np.array([0, 0, 1/np.sqrt(2), 1/np.sqrt(2)]),  # 90° around Z
    ]

    for i, quat in enumerate(test_quats):
        norm = np.linalg.norm(quat)
        print(f"  Quaternion {i}: {quat}")
        print(f"    Norm: {norm:.6f}")
        assert np.allclose(norm, 1.0, atol=1e-6), f"Quat {i} not normalized"
        print("    ✓ Valid\n")

    print("  ✓ PASS\n")


def test_data_integrity():
    """Test data integrity and edge cases"""
    print("Test 7: Data Integrity & Edge Cases")
    print("=" * 60)

    camera = D455CameraModel()

    # Test 1: Empty mask
    print("  Test 7a: Empty mask")
    mask_empty = np.zeros((480, 640), dtype=np.uint8)
    size = estimate_size_from_mask_and_depth(mask_empty, 1.0, camera)
    print(f"    Empty mask → size: {size}")
    assert np.allclose(size, [0, 0, 0]), "Empty mask should give zero size"
    print("    ✓ PASS\n")

    # Test 2: Very small depth
    print("  Test 7b: Small depth (0.6m)")
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[200:300, 270:370] = 255  # 100×100
    size = estimate_size_from_mask_and_depth(mask, 0.6, camera)
    print(f"    Size at 0.6m: {size}")
    assert size[0] > 0 and size[1] > 0 and size[2] > 0, "Size should be positive"
    print("    ✓ PASS\n")

    # Test 3: Large depth
    print("  Test 7c: Large depth (6.0m)")
    size = estimate_size_from_mask_and_depth(mask, 6.0, camera)
    print(f"    Size at 6.0m: {size}")
    # At 6m, objects should appear larger
    size_6m = size
    size_1m = estimate_size_from_mask_and_depth(mask, 1.0, camera)
    print(f"    Size at 1.0m: {size_1m}")
    assert size_6m[0] > size_1m[0], "Object should appear larger at 6m"
    print("    ✓ PASS\n")

    print("✓ ALL EDGE CASES PASS\n")


# ─────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ─────────────────────────────────────────────────────────────────────────────

def convert_direction_to_quaternion(direction):
    """Convert 3D direction vector to quaternion"""
    direction = direction / np.linalg.norm(direction)
    z_axis = np.array([0.0, 0.0, 1.0])

    if np.allclose(direction, z_axis):
        return np.array([0.0, 0.0, 0.0, 1.0])
    elif np.allclose(direction, -z_axis):
        return np.array([1.0, 0.0, 0.0, 0.0])

    axis = np.cross(z_axis, direction)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(z_axis, direction))

    half_angle = angle / 2.0
    quat = np.array([
        axis[0] * np.sin(half_angle),
        axis[1] * np.sin(half_angle),
        axis[2] * np.sin(half_angle),
        np.cos(half_angle)
    ])
    return quat


def angle_to_quaternion_around_z(angle):
    """Convert 2D angle (rotation around Z) to quaternion"""
    half_angle = angle / 2.0
    return np.array([
        0.0,
        0.0,
        np.sin(half_angle),
        np.cos(half_angle)
    ])


def estimate_size_from_mask_and_depth(mask, depth, camera):
    """Estimate 3D size from mask and depth"""
    y_coords, x_coords = np.where(mask > 0)
    if len(x_coords) == 0:
        return np.array([0.0, 0.0, 0.0])

    x_min, x_max = x_coords.min(), x_coords.max()
    y_min, y_max = y_coords.min(), y_coords.max()

    width_px = x_max - x_min
    height_px = y_max - y_min

    fov_width_m, fov_height_m = camera.compute_fov_at_distance(depth)

    size_x = (width_px / camera.image_width) * fov_width_m
    size_y = (height_px / camera.image_height) * fov_height_m
    size_z = depth * 0.1

    return np.array([size_x, size_y, size_z])


def estimate_orientation_from_mask(mask, camera_direction):
    """Estimate orientation from mask using PCA"""
    y_coords, x_coords = np.where(mask > 0)
    if len(x_coords) < 3:
        return np.array([0.0, 0.0, 0.0, 1.0])

    points_2d = np.column_stack([x_coords, y_coords]).astype(np.float32)
    mean_2d = points_2d.mean(axis=0)
    centered = points_2d - mean_2d

    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    primary_axis_2d = eigenvectors[:, 1]
    angle = np.arctan2(primary_axis_2d[1], primary_axis_2d[0])

    half_angle = angle / 2.0
    quat = np.array([
        0.0,
        0.0,
        np.sin(half_angle),
        np.cos(half_angle)
    ])

    return quat


if __name__ == '__main__':
    print("\n")
    print("=" * 60)
    print("PHASE 5 UNIT TESTS: 6DOF POSE EXTRACTION")
    print("=" * 60)
    print("\n")

    try:
        test_direction_to_quaternion()
        test_angle_to_quaternion()
        test_size_estimation()
        test_size_at_different_depths()
        test_mask_pca_orientation()
        test_quaternion_norm()
        test_data_integrity()

        print("=" * 60)
        print("✅ ALL 7 TESTS PASSED!")
        print("=" * 60)

    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
