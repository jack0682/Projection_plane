
import numpy as np
import open3d as o3d
import cv2
import os

# =============================================================================
# GEOMETRY FUNCTIONS
# =============================================================================


def normalize(v: np.ndarray) -> np.ndarray:
    """
    Normalize a vector.
    
    Args:
        v: Input vector
        
    Returns:
        Unit vector
        
    Raises:
        ValueError: If vector norm is near zero
    """
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        raise ValueError(f"Cannot normalize near-zero vector: norm={norm}")
    return v / norm


def validate_plane(a: float, b: float, c: float, d: float) -> np.ndarray:
    """
    Validate plane equation and return normal vector.
    
    Args:
        a, b, c, d: Plane equation coefficients
        
    Returns:
        Normal vector n = (a, b, c)
        
    Raises:
        ValueError: If plane is degenerate (||(a,b,c)|| < 1e-12)
    """
    n = np.array([a, b, c], dtype=np.float64)
    n_norm = np.linalg.norm(n)
    
    if n_norm < 1e-12:
        raise ValueError(
            f"Degenerate plane: normal vector (a,b,c) = ({a},{b},{c}) has near-zero magnitude. "
            f"Plane equation ax+by+cz+d=0 requires non-zero normal."
        )
    
    return n


def choose_up_hint(n_hat: np.ndarray, user_up_hint: np.ndarray = None) -> np.ndarray:
    """
    Choose an appropriate up_hint vector for basis construction.
    
    The up_hint should not be parallel to the plane normal.
    
    Args:
        n_hat: Normalized plane normal
        user_up_hint: User-specified up hint (optional)
        
    Returns:
        Appropriate up_hint vector
    """
    if user_up_hint is not None:
        up = np.array(user_up_hint, dtype=np.float64)
        if np.linalg.norm(up) > 1e-12:
            up = normalize(up)
            if abs(np.dot(n_hat, up)) < 0.95:
                # User hint is not too parallel
                return up
            # User hint is too parallel, fall through to defaults
    
    # Default: try (0,0,1), then (0,1,0), then (1,0,0)
    candidates = [
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    ]
    
    for up in candidates:
        if abs(np.dot(n_hat, up)) < 0.95:
            return up
    
    # Fallback (should never reach here for valid planes)
    return candidates[0]


def build_basis(n_hat: np.ndarray, up_hint: np.ndarray) -> tuple:
    """
    Build orthonormal basis (t1, t2) for the plane.
    
    Args:
        n_hat: Normalized plane normal
        up_hint: Hint vector for orientation
        
    Returns:
        Tuple (t1, t2) where t1, t2 are orthonormal vectors on the plane
        
    Raises:
        ValueError: If basis construction fails
    """
    # t1 = normalize(cross(n_hat, up_hint))
    t1 = np.cross(n_hat, up_hint)
    t1_norm = np.linalg.norm(t1)
    
    if t1_norm < 1e-12:
        raise ValueError(
            f"Failed to build basis: n_hat and up_hint are parallel. "
            f"n_hat={n_hat}, up_hint={up_hint}"
        )
    
    t1 = t1 / t1_norm
    
    # t2 = cross(n_hat, t1) - should be unit already
    t2 = np.cross(n_hat, t1)
    t2_norm = np.linalg.norm(t2)
    
    if t2_norm < 1e-12:
        raise ValueError("Failed to build basis: t2 is near-zero")
    
    t2 = t2 / t2_norm  # Normalize for safety
    
    return t1, t2


def project_points(points: np.ndarray, n: np.ndarray, d: float) -> np.ndarray:
    """
    Orthographically project points onto the plane.
    
    Projection formula:
        p_proj = p - ((n·p + d) / ||n||^2) * n
    
    Args:
        points: (N, 3) array of 3D points
        n: Normal vector (a, b, c)
        d: Plane equation constant
        
    Returns:
        (N, 3) array of projected points
    """
    n_sq = np.dot(n, n)  # ||n||^2
    
    # Signed distance to plane (scaled by ||n||): n·p + d
    distances = np.dot(points, n) + d
    
    # Projection: p - ((n·p + d) / ||n||^2) * n
    # Shape: (N,1) * (3,) -> (N,3)
    projections = points - (distances[:, np.newaxis] / n_sq) * n
    
    return projections


def compute_origin(points_proj: np.ndarray, n: np.ndarray, d: float, mode: str) -> np.ndarray:
    """
    Compute UV origin point on the plane.
    
    Args:
        points_proj: Projected points
        n: Normal vector
        d: Plane constant
        mode: 'mean' or 'closest'
        
    Returns:
        Origin point on plane
    """
    if mode == 'closest':
        # Closest point on plane to world origin: -(d / ||n||^2) * n
        n_sq = np.dot(n, n)
        return -(d / n_sq) * n
    else:  # 'mean'
        return np.mean(points_proj, axis=0)


def map_uv(points_proj: np.ndarray, origin: np.ndarray, t1: np.ndarray, t2: np.ndarray) -> tuple:
    """
    Map projected points to UV coordinates on the plane.
    
    Args:
        points_proj: (N, 3) projected points
        origin: Origin point for UV system
        t1, t2: Basis vectors
        
    Returns:
        Tuple (u, v) where each is (N,) array
    """
    # Relative positions
    rel = points_proj - origin
    
    # u = dot(t1, rel), v = dot(t2, rel)
    u = np.dot(rel, t1)
    v = np.dot(rel, t2)
    
    return u, v


def compute_depth(points: np.ndarray, n: np.ndarray, d: float, mode: str) -> np.ndarray:
    """
    Compute depth values for z-buffer.
    
    Signed distance to plane: dist = (n·p + d) / ||n||
    
    Args:
        points: Original 3D points
        n: Normal vector
        d: Plane constant
        mode: 'abs' or 'signed'
        
    Returns:
        (N,) array of depth values
    """
    n_norm = np.linalg.norm(n)
    signed_dist = (np.dot(points, n) + d) / n_norm
    
    if mode == 'abs':
        return np.abs(signed_dist)
    else:  # 'signed'
        return signed_dist


def compute_image_size(
    u: np.ndarray, 
    v: np.ndarray, 
    pixels_per_unit: float,
    width_override: int = None,
    height_override: int = None,
    robust_range: bool = False,
    percentiles: tuple = (1, 99)
) -> tuple:
    """
    Compute image dimensions and UV bounds.
    
    Args:
        u, v: UV coordinates
        pixels_per_unit: Resolution factor
        width_override, height_override: Optional fixed dimensions
        robust_range: Use percentile-based range
        percentiles: (low, high) percentiles for robust range
        
    Returns:
        Tuple (width, height, u_min, u_max, v_min, v_max)
    """
    if robust_range:
        u_min = np.percentile(u, percentiles[0])
        u_max = np.percentile(u, percentiles[1])
        v_min = np.percentile(v, percentiles[0])
        v_max = np.percentile(v, percentiles[1])
    else:
        u_min, u_max = u.min(), u.max()
        v_min, v_max = v.min(), v.max()
    
    # Compute auto dimensions
    u_range = u_max - u_min
    v_range = v_max - v_min
    
    # Handle degenerate cases
    if u_range < 1e-9:
        u_range = 1.0
    if v_range < 1e-9:
        v_range = 1.0
    
    auto_width = int(np.ceil(u_range * pixels_per_unit)) + 1
    auto_height = int(np.ceil(v_range * pixels_per_unit)) + 1
    
    # Apply limits
    auto_width = max(100, min(8192, auto_width))
    auto_height = max(100, min(8192, auto_height))
    
    # Apply overrides
    width = width_override if width_override is not None else auto_width
    height = height_override if height_override is not None else auto_height
    
    return width, height, u_min, u_max, v_min, v_max


def rasterize(
    u: np.ndarray,
    v: np.ndarray,
    depth: np.ndarray,
    colors: np.ndarray,
    width: int,
    height: int,
    u_min: float,
    u_max: float,
    v_min: float,
    v_max: float,
    pixels_per_unit: float,
    depth_priority_far: bool,
    point_size: int = 1
) -> tuple:
    """
    Rasterize points to image with z-buffer.
    
    Args:
        u, v: UV coordinates
        depth: Depth values
        colors: (N, 3) BGR colors (uint8) or None
        width, height: Image dimensions
        u_min, u_max, v_min, v_max: UV bounds
        pixels_per_unit: Resolution factor
        depth_priority_far: If True, farther points win
        point_size: Pixel size per point (1 = single pixel)
        
    Returns:
        Tuple (image, pixels_written):
            - image: (height, width, 3) uint8 image
            - pixels_written: Count of pixels written
    """
    # Initialize image and depth buffer
    image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Initialize depth buffer
    if depth_priority_far:
        zbuf = np.full((height, width), -np.inf, dtype=np.float32)
    else:
        zbuf = np.full((height, width), np.inf, dtype=np.float32)
    
    # Compute pixel coordinates
    # x = (u - u_min) * scale
    # y = (v_max - v) * scale  (y increases downward)
    u_range = u_max - u_min
    v_range = v_max - v_min
    
    if u_range < 1e-9:
        u_range = 1.0
    if v_range < 1e-9:
        v_range = 1.0
    
    scale_u = (width - 1) / u_range
    scale_v = (height - 1) / v_range
    
    px = np.round((u - u_min) * scale_u).astype(np.int32)
    py = np.round((v_max - v) * scale_v).astype(np.int32)
    
    # Clamp to bounds
    px = np.clip(px, 0, width - 1)
    py = np.clip(py, 0, height - 1)
    
    # Default color if no colors provided
    if colors is None:
        colors = np.full((len(u), 3), 128, dtype=np.uint8)  # Gray
    
    # Rasterize with z-buffer
    n_points = len(u)
    pixels_written = 0
    
    for i in range(n_points):
        x, y = px[i], py[i]
        d = depth[i]
        
        # Determine if this point should replace current pixel
        if depth_priority_far:
            should_write = d > zbuf[y, x]
        else:
            should_write = d < zbuf[y, x]
        
        if should_write:
            zbuf[y, x] = d
            image[y, x] = colors[i]
            pixels_written += 1
            
            # Multi-pixel rendering
            if point_size > 1:
                half = point_size // 2
                for dy in range(-half, half + 1):
                    for dx in range(-half, half + 1):
                        if dy == 0 and dx == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if depth_priority_far:
                                if d > zbuf[ny, nx]:
                                    zbuf[ny, nx] = d
                                    image[ny, nx] = colors[i]
                            else:
                                if d < zbuf[ny, nx]:
                                    zbuf[ny, nx] = d
                                    image[ny, nx] = colors[i]
    
    return image, pixels_written


# =============================================================================
# POINT CLOUD I/O
# =============================================================================

def load_point_cloud(filepath: str) -> tuple:
    """
    Load point cloud from PLY/PCD file.
    
    Args:
        filepath: Path to point cloud file
        
    Returns:
        Tuple (points, colors) where:
            - points: (N, 3) float64 array
            - colors: (N, 3) uint8 BGR array or None
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Input file not found: {filepath}")
    
    # print(f"Loading point cloud from {filepath}...")
    pcd = o3d.io.read_point_cloud(filepath)
    
    points = np.asarray(pcd.points, dtype=np.float64)
    
    if len(points) == 0:
        raise ValueError("Point cloud is empty")
    
    # Handle colors
    colors_raw = np.asarray(pcd.colors)
    if len(colors_raw) > 0:
        # Convert from [0,1] RGB to [0,255] BGR
        colors_bgr = (colors_raw[:, [2, 1, 0]] * 255).astype(np.uint8)
        # print(f"Loaded {len(points)} points with colors")
    else:
        colors_bgr = None
        # print(f"Loaded {len(points)} points (no color)")
    
    return points, colors_bgr
