#!/usr/bin/env python3
"""Geometry utilities for 3D tracking from 2D projections."""

import numpy as np
from typing import Tuple


def unproject_pixel_to_3d(
    pixel_x: float,
    pixel_y: float,
    width: int,
    height: int,
    u_min: float,
    u_max: float,
    v_min: float,
    v_max: float,
    origin: np.ndarray,
    t1: np.ndarray,
    t2: np.ndarray,
) -> np.ndarray:
    """
    Reproject a 2D pixel coordinate to 3D world coordinates on the projection plane.

    Args:
        pixel_x: x-coordinate in image (0 to width-1)
        pixel_y: y-coordinate in image (0 to height-1)
        width: image width (pixels)
        height: image height (pixels)
        u_min, u_max: min/max u coordinates on projection plane
        v_min, v_max: min/max v coordinates on projection plane
        origin: 3D origin point of projection basis [x, y, z]
        t1: first tangent vector of projection plane (u direction)
        t2: second tangent vector of projection plane (v direction)

    Returns:
        3D point on projection plane as np.ndarray([x, y, z])

    Formula:
        u = u_min + (pixel_x / (width - 1)) * (u_max - u_min)
        v = v_max - (pixel_y / (height - 1)) * (v_max - v_min)
        p3d = origin + u*t1 + v*t2
    """
    if width <= 1 or height <= 1:
        raise ValueError(f"Invalid image dimensions: {width}x{height}")

    # Normalize pixel coordinates to [0, 1] range
    u_norm = pixel_x / (width - 1)
    v_norm = pixel_y / (height - 1)

    # Map to plane coordinates
    # Note: v is inverted (y increases downward in images, but upward in world)
    u = u_min + u_norm * (u_max - u_min)
    v = v_max - v_norm * (v_max - v_min)

    # Project to 3D
    p3d = origin + u * t1 + v * t2

    return p3d


def extract_metadata(metadata_msg) -> dict:
    """
    Extract projection metadata from Float64MultiArray message.

    Expected order: [width, height,
                     a, b, c, d,
                     origin_x, origin_y, origin_z,
                     t1_x, t1_y, t1_z,
                     t2_x, t2_y, t2_z,
                     u_min, u_max, v_min, v_max,
                     stamp_sec]

    Returns:
        dict with keys: width, height, a, b, c, d, origin, t1, t2,
                        u_min, u_max, v_min, v_max, stamp_sec
    """
    if len(metadata_msg.data) < 20:
        raise ValueError(
            f"Expected at least 20 metadata values, got {len(metadata_msg.data)}"
        )

    data = metadata_msg.data
    return {
        "width": int(data[0]),
        "height": int(data[1]),
        "a": data[2],
        "b": data[3],
        "c": data[4],
        "d": data[5],
        "origin": np.array([data[6], data[7], data[8]]),
        "t1": np.array([data[9], data[10], data[11]]),
        "t2": np.array([data[12], data[13], data[14]]),
        "u_min": data[15],
        "u_max": data[16],
        "v_min": data[17],
        "v_max": data[18],
        "stamp_sec": int(data[19]),
    }


def compute_bbox_center_3d(
    detection_2d,
    width: int,
    height: int,
    u_min: float,
    u_max: float,
    v_min: float,
    v_max: float,
    origin: np.ndarray,
    t1: np.ndarray,
    t2: np.ndarray,
) -> np.ndarray:
    """
    Compute 3D center of a 2D detection bounding box.

    Args:
        detection_2d: vision_msgs/Detection2D message
        width, height: image dimensions
        u_min, u_max, v_min, v_max: projection plane bounds
        origin, t1, t2: projection basis

    Returns:
        3D center point as np.ndarray([x, y, z])
    """
    # Get 2D center from detection
    pixel_x = detection_2d.bbox.center.position.x
    pixel_y = detection_2d.bbox.center.position.y

    # Unproject to 3D
    center_3d = unproject_pixel_to_3d(
        pixel_x, pixel_y, width, height, u_min, u_max, v_min, v_max, origin, t1, t2
    )

    return center_3d


def compute_3d_distance(p1: np.ndarray, p2: np.ndarray) -> float:
    """Compute Euclidean distance between two 3D points."""
    return float(np.linalg.norm(p1 - p2))


def ema_smooth(current_value: np.ndarray, smoothed_value: np.ndarray, alpha: float) -> np.ndarray:
    """
    Exponential moving average.

    Args:
        current_value: new value (N-dim array)
        smoothed_value: previously smoothed value
        alpha: EMA coefficient (0 < alpha <= 1)

    Returns:
        Updated smoothed value
    """
    if smoothed_value is None:
        return np.array(current_value, dtype=np.float64)
    return alpha * np.array(current_value, dtype=np.float64) + (1 - alpha) * smoothed_value
