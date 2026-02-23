#!/usr/bin/env python3
"""
Phase 2: Confidence-Weighted Mask Center Calculation
Calculate mask center points using confidence weighting for more stable detection.
"""

import numpy as np


def get_weighted_mask_center(mask, confidence_map=None):
    """
    Extract confidence-weighted center point from mask.

    Weighted averaging gives more importance to high-confidence pixels,
    resulting in a more stable center estimate than simple geometric center.

    Args:
        mask: (H, W) binary array where mask > 0 indicates object pixels
        confidence_map: (H, W) float array with values in [0, 1] or None
                       If None, uses uniform weighting

    Returns:
        tuple: (center_x, center_y, mean_confidence)
            - center_x (float): Weighted x-coordinate (in pixels)
            - center_y (float): Weighted y-coordinate (in pixels)
            - mean_confidence (float): Mean confidence of all mask pixels

    Example:
        >>> mask = np.zeros((480, 640))
        >>> mask[200:250, 300:350] = 1
        >>> confidence_map = np.random.rand(480, 640)
        >>> cx, cy, conf = get_weighted_mask_center(mask, confidence_map)
        >>> print(f"Center: ({cx:.1f}, {cy:.1f}), Confidence: {conf:.3f}")
    """
    # Get all pixels belonging to the mask
    y_coords, x_coords = np.where(mask > 0)

    # Handle empty mask
    if len(x_coords) == 0:
        return None, None, 0.0

    # Get weights for each pixel
    if confidence_map is not None and confidence_map.shape == mask.shape:
        # Use pixel-wise confidence from confidence_map
        weights = confidence_map[y_coords, x_coords].astype(np.float32)
    else:
        # Fallback: Uniform weighting (all pixels equally important)
        weights = np.ones(len(x_coords), dtype=np.float32)

    # Ensure weights are valid (no NaN, no inf)
    if np.any(np.isnan(weights)) or np.any(np.isinf(weights)):
        weights = np.ones(len(x_coords), dtype=np.float32)

    # Calculate weighted average center
    # Using numpy.average with weights parameter
    center_x = np.average(x_coords, weights=weights)
    center_y = np.average(y_coords, weights=weights)

    # Calculate mean confidence
    mean_confidence = np.mean(weights)

    return float(center_x), float(center_y), float(mean_confidence)


def get_simple_mask_center(mask):
    """
    Extract simple (non-weighted) geometric center from mask.

    This is the baseline comparison method - calculates the arithmetic mean
    of all mask pixel coordinates without any weighting.

    Args:
        mask: (H, W) binary array where mask > 0 indicates object pixels

    Returns:
        tuple: (center_x, center_y)
            - center_x (float): Arithmetic mean x-coordinate (in pixels)
            - center_y (float): Arithmetic mean y-coordinate (in pixels)
            - Returns (None, None) if mask is empty

    Example:
        >>> mask = np.zeros((480, 640))
        >>> mask[200:250, 300:350] = 1
        >>> cx, cy = get_simple_mask_center(mask)
        >>> print(f"Simple center: ({cx:.1f}, {cy:.1f})")
    """
    # Get all pixels belonging to the mask
    y_coords, x_coords = np.where(mask > 0)

    # Handle empty mask
    if len(x_coords) == 0:
        return None, None

    # Calculate simple arithmetic mean (no weighting)
    center_x = np.mean(x_coords)
    center_y = np.mean(y_coords)

    return float(center_x), float(center_y)


def compare_mask_centers(mask, confidence_map=None):
    """
    Compare weighted and simple mask centers and return statistics.

    Useful for validation and understanding the difference between
    weighted and simple center estimates.

    Args:
        mask: (H, W) binary array
        confidence_map: (H, W) float array or None

    Returns:
        dict: Dictionary with comparison results
            {
                'weighted_center': (cx_weighted, cy_weighted),
                'simple_center': (cx_simple, cy_simple),
                'difference': (dx, dy),
                'distance': euclidean_distance,
                'mean_confidence': mean_confidence
            }

    Example:
        >>> comparison = compare_mask_centers(mask, confidence_map)
        >>> print(f"Center difference: {comparison['distance']:.2f} pixels")
    """
    # Get weighted center
    cx_weighted, cy_weighted, mean_conf = get_weighted_mask_center(mask, confidence_map)

    # Get simple center
    cx_simple, cy_simple = get_simple_mask_center(mask)

    # Handle cases where mask is empty
    if cx_weighted is None or cx_simple is None:
        return {
            'weighted_center': (None, None),
            'simple_center': (None, None),
            'difference': (None, None),
            'distance': None,
            'mean_confidence': 0.0
        }

    # Calculate differences
    dx = cx_weighted - cx_simple
    dy = cy_weighted - cy_simple
    distance = np.sqrt(dx**2 + dy**2)

    return {
        'weighted_center': (float(cx_weighted), float(cy_weighted)),
        'simple_center': (float(cx_simple), float(cy_simple)),
        'difference': (float(dx), float(dy)),
        'distance': float(distance),
        'mean_confidence': float(mean_conf)
    }


def estimate_mask_statistics(mask, confidence_map=None):
    """
    Calculate comprehensive statistics about a mask.

    Useful for understanding mask properties and quality assessment.

    Args:
        mask: (H, W) binary array
        confidence_map: (H, W) float array or None

    Returns:
        dict: Dictionary with mask statistics
    """
    y_coords, x_coords = np.where(mask > 0)

    if len(x_coords) == 0:
        return {
            'pixel_count': 0,
            'area': 0.0,
            'width_pixels': 0,
            'height_pixels': 0,
            'bounding_box': None,
            'center': (None, None),
            'mean_confidence': 0.0,
            'min_confidence': 0.0,
            'max_confidence': 0.0
        }

    # Get confidence statistics
    if confidence_map is not None and confidence_map.shape == mask.shape:
        confidences = confidence_map[y_coords, x_coords]
        mean_conf = np.mean(confidences)
        min_conf = np.min(confidences)
        max_conf = np.max(confidences)
    else:
        mean_conf = 1.0
        min_conf = 1.0
        max_conf = 1.0

    # Bounding box
    y_min, y_max = y_coords.min(), y_coords.max()
    x_min, x_max = x_coords.min(), x_coords.max()

    return {
        'pixel_count': len(x_coords),
        'area': float(len(x_coords)),
        'width_pixels': int(x_max - x_min + 1),
        'height_pixels': int(y_max - y_min + 1),
        'bounding_box': (int(x_min), int(y_min), int(x_max), int(y_max)),
        'center': get_weighted_mask_center(mask, confidence_map)[:2],
        'mean_confidence': float(mean_conf),
        'min_confidence': float(min_conf),
        'max_confidence': float(max_conf)
    }


# ─────────────────────────────────────────────────────────────────────────────
# Unit Tests (for validation)
# ─────────────────────────────────────────────────────────────────────────────

def test_mask_analysis():
    """
    Unit tests for mask analysis functions.
    Run with: python3 -m pytest mask_analysis.py -v
    """
    # Test 1: Simple rectangular mask
    print("Test 1: Simple rectangular mask")
    mask = np.zeros((480, 640))
    mask[200:250, 300:350] = 1

    cx_w, cy_w, conf_w = get_weighted_mask_center(mask)
    cx_s, cy_s = get_simple_mask_center(mask)

    print(f"  Weighted center: ({cx_w:.1f}, {cy_w:.1f}), conf={conf_w:.3f}")
    print(f"  Simple center:   ({cx_s:.1f}, {cy_s:.1f})")
    assert abs(cx_w - cx_s) < 1.0, "Weighted and simple should be similar for uniform mask"
    print("  ✓ PASS\n")

    # Test 2: Mask with confidence map
    print("Test 2: Mask with confidence weighting")
    mask = np.zeros((480, 640))
    mask[100:200, 200:300] = 1

    # Confidence higher on right side
    confidence_map = np.zeros((480, 640))
    confidence_map[100:200, 200:250] = 0.3  # Left side: low confidence
    confidence_map[100:200, 250:300] = 0.9  # Right side: high confidence

    cx_w, cy_w, conf_w = get_weighted_mask_center(mask, confidence_map)
    cx_s, cy_s = get_simple_mask_center(mask)

    print(f"  Weighted center: ({cx_w:.1f}, {cy_w:.1f}), conf={conf_w:.3f}")
    print(f"  Simple center:   ({cx_s:.1f}, {cy_s:.1f})")
    assert cx_w > cx_s, "Weighted center should be shifted right (higher confidence)"
    print("  ✓ PASS\n")

    # Test 3: Empty mask
    print("Test 3: Empty mask")
    mask = np.zeros((480, 640))

    cx_w, cy_w, conf_w = get_weighted_mask_center(mask)
    cx_s, cy_s = get_simple_mask_center(mask)

    print(f"  Weighted center: ({cx_w}, {cy_w}), conf={conf_w}")
    print(f"  Simple center:   ({cx_s}, {cy_s})")
    assert cx_w is None and cy_w is None, "Should return None for empty mask"
    assert cx_s is None and cy_s is None, "Should return None for empty mask"
    print("  ✓ PASS\n")

    # Test 4: Comparison function
    print("Test 4: Comparison function")
    mask = np.zeros((480, 640))
    mask[150:250, 250:350] = 1
    confidence_map = np.random.rand(480, 640) * 0.5 + 0.5

    comparison = compare_mask_centers(mask, confidence_map)
    print(f"  Weighted center: {comparison['weighted_center']}")
    print(f"  Simple center:   {comparison['simple_center']}")
    print(f"  Distance diff:   {comparison['distance']:.2f} pixels")
    print(f"  Mean confidence: {comparison['mean_confidence']:.3f}")
    assert comparison['distance'] is not None, "Should calculate distance"
    print("  ✓ PASS\n")

    # Test 5: Statistics function
    print("Test 5: Statistics function")
    mask = np.zeros((480, 640))
    mask[100:300, 200:400] = 1

    stats = estimate_mask_statistics(mask)
    print(f"  Pixel count:     {stats['pixel_count']}")
    print(f"  Area:            {stats['area']}")
    print(f"  Bounding box:    {stats['bounding_box']}")
    print(f"  Center:          {stats['center']}")
    assert stats['pixel_count'] > 0, "Should count pixels"
    print("  ✓ PASS\n")

    print("✅ All tests passed!")


if __name__ == '__main__':
    test_mask_analysis()
