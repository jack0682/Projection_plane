# realtime_detect: AprilTag-based 6DOF Box Pose Detection

A ROS 2 Python package for real-time detection and tracking of AprilTags on box fronts, with 6DOF pose estimation in the 'map' frame using TF2 transforms.

## Features

- **AprilTag Detection**: Integrates with `apriltag_ros` for tag36h11 family detection
- **6DOF Pose Estimation**: Extracts position and orientation from AprilTag detections using homography decomposition
- **Map Frame Transform**: Automatically transforms detected poses from camera frame to map frame via TF2
- **RViz Visualization**: Publishes PoseArray and MarkerArray for RViz visualization
- **Robust Error Handling**: Gracefully handles missing TF transforms and missing camera info
- **Configurable Filtering**: Support for tag ID whitelisting and decision margin filtering
- **Debug Logging**: Optional debug output for troubleshooting

## Installation

### Prerequisites

- ROS 2 Humble (or compatible version)
- `apriltag_ros` package installed
- Python packages: `rclpy`, `numpy`, `scipy`

### Building

```bash
cd ~/ros2_ws
colcon build --packages-select realtime_detect
source install/setup.bash
```

## Running

### Quick Start (with remappings)

If your camera topics are not the default names, use remappings:

```bash
ros2 launch realtime_detect apriltag_box_pose.launch.py \
    image_topic:=/your_camera/image_raw \
    camera_info_topic:=/your_camera/camera_info \
    camera_frame:=your_camera_optical_frame
```

### With Custom Parameters

```bash
ros2 launch realtime_detect apriltag_box_pose.launch.py \
    tag_size:=0.083 \
    tag_family:=tag36h11 \
    map_frame:=map \
    debug:=True
```

### Verifying the System

**Check if all nodes are running:**
```bash
ros2 node list
# Should show:
# /apriltag_node (from apriltag_ros)
# /apriltag_box_pose_node
```

**Check published topics:**
```bash
ros2 topic list | grep -E '(tag_detections|box_poses|box_pose_markers)'
# Should show:
# /tag_detections (AprilTagDetectionArray)
# /realtime_detect/box_poses (PoseArray)
# /realtime_detect/box_pose_markers (MarkerArray)
```

**Inspect detection data:**
```bash
ros2 topic echo /tag_detections --once
```

**Inspect output poses:**
```bash
ros2 topic echo /realtime_detect/box_poses --once
```

## Configuration

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `image_topic` | `/camera/image_raw` | Input image topic |
| `camera_info_topic` | `/camera/camera_info` | Camera calibration topic |
| `camera_frame` | `camera_color_optical_frame` | Camera optical frame name |
| `map_frame` | `map` | Target global frame for poses |
| `tag_family` | `tag36h11` | AprilTag family |
| `tag_size` | `0.083` | Tag edge size in meters |
| `debug` | `False` | Enable debug logging |

### Node Parameters (in launch file)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_frame` | `map` | Target TF frame |
| `camera_frame` | `camera_color_optical_frame` | Camera optical frame |
| `detection_topic` | `/tag_detections` | Subscription topic for detections |
| `camera_info_topic` | `/camera/camera_info` | Subscription topic for camera calibration |
| `publish_rate_hz` | `20` | Publishing rate (Hz) |
| `pose_topic` | `/realtime_detect/box_poses` | Output PoseArray topic |
| `marker_topic` | `/realtime_detect/box_pose_markers` | Output MarkerArray topic |
| `tag_ids_whitelist` | `[-1]` | Tag IDs to accept (-1 = all) |
| `tag_size_m` | `0.083` | AprilTag size in meters |
| `decision_margin_min` | `0.0` | Minimum decision margin (0.0-1.0) |
| `debug` | `False` | Enable debug logging |

### Example: Filter Specific Tag IDs

Edit the launch file or pass parameters:

```bash
ros2 launch realtime_detect apriltag_box_pose.launch.py
# Then modify the tag_ids_whitelist in the launch file to e.g., [0, 1, 5]
```

### Example: Require High Detection Quality

```bash
ros2 launch realtime_detect apriltag_box_pose.launch.py \
    decision_margin_min:=0.3  # Only detections with >0.3 margin
```

## Architecture

### Data Flow

```
Camera Image + Calibration
    ↓
apriltag_ros (tag36h11 detector)
    ↓
AprilTagDetectionArray (/tag_detections)
    ↓
apriltag_box_pose_node
    ├─ Extract homography from detection
    ├─ Decompose homography → 6DOF in camera frame
    ├─ Query TF2 for camera → map transform
    ├─ Transform pose to map frame
    └─ Publish:
        ├─ PoseArray (/realtime_detect/box_poses)
        └─ MarkerArray (/realtime_detect/box_pose_markers)
    ↓
RViz / Downstream Applications
```

### Message Types

**Input:**
- `sensor_msgs/Image` — Camera images
- `sensor_msgs/CameraInfo` — Camera calibration
- `apriltag_msgs/AprilTagDetectionArray` — Detected tags with homography

**Output:**
- `geometry_msgs/PoseArray` — Detected box poses in map frame
- `visualization_msgs/MarkerArray` — RViz visualization (arrows + text)

## RViz Visualization

1. Open RViz:
   ```bash
   rviz2
   ```

2. Add displays:
   - **TF** — to see frame relationships
   - **PoseArray** — Display `/realtime_detect/box_poses`
   - **MarkerArray** — Display `/realtime_detect/box_pose_markers` (arrows + tag IDs)

3. Set fixed frame to `map` to see poses in map frame

4. Ensure TF tree is connected:
   ```bash
   ros2 run tf2_tools view_frames.py  # then open frames.pdf
   ```

## Troubleshooting

### No Detections Appearing

**Symptom:** `/tag_detections` topic is empty

**Causes & Fixes:**
1. **Camera not publishing:**
   ```bash
   ros2 topic list | grep camera
   # If empty, start your camera driver first
   ```

2. **Wrong image topic:**
   ```bash
   # Check available topics
   ros2 topic list | grep image
   # Launch with correct topic:
   ros2 launch realtime_detect apriltag_box_pose.launch.py image_topic:=/your_image_topic
   ```

3. **Camera image quality:**
   - Ensure good lighting
   - AprilTag should be >= 10 pixels/side in image
   - Try viewing raw camera feed in RViz

### Poses Not Transforming to Map Frame

**Symptom:** TF errors in node logs, poses not published

**Causes & Fixes:**
1. **Missing camera→map transform:**
   ```bash
   # Check TF tree
   ros2 run tf2_tools view_frames.py

   # Publish static transform if needed:
   ros2 run tf2_ros static_transform_publisher \
       0 0 0 0 0 0 map camera_color_optical_frame
   ```

2. **Wrong camera frame name:**
   ```bash
   # Find correct frame:
   ros2 topic echo /camera/camera_info | grep frame_id

   # Launch with correct frame:
   ros2 launch realtime_detect apriltag_box_pose.launch.py \
       camera_frame:=correct_frame_name
   ```

3. **Stale transforms:**
   ```bash
   # Increase TF buffer size or reduce timeout in node parameters
   # Default timeout is 0.1s; increase if transforms are delayed
   ```

### High Jitter in Detected Poses

**Symptom:** Poses oscillate or jump between frames

**Causes & Fixes:**
1. **Low decision margin detections:**
   ```bash
   ros2 launch realtime_detect apriltag_box_pose.launch.py \
       decision_margin_min:=0.2  # Filter weak detections
   ```

2. **Homography decomposition ambiguity:**
   - Ensure tag size is accurate (use `tag_size:=0.083`)
   - Verify camera calibration (check in `/camera/camera_info`)

3. **Check detection quality:**
   ```bash
   ros2 topic echo /tag_detections | grep -A2 decision_margin
   ```

### Poses Inverted/Flipped

**Symptom:** Orientation appears 180° rotated

**Causes & Fixes:**
1. **Homography sign ambiguity:**
   - Verify tag is oriented correctly in image
   - Check if camera→world transform is applied correctly

2. **Double-check TF chain:**
   ```bash
   ros2 run tf2_tools view_frames.py
   # Ensure camera→map is forward, not inverted
   ```

### Debug Logging

Enable verbose output:

```bash
ros2 launch realtime_detect apriltag_box_pose.launch.py debug:=True
```

Log output will show:
- Detection count per frame
- Camera matrix values
- TF transform requests
- Homography decomposition details

### Common Error Messages

| Error | Meaning | Fix |
|-------|---------|-----|
| `TF lookup failed` | Transform not yet available | Publish static transform or wait for TF tree |
| `TF extrapolation failed` | Transform too old | Increase TF buffer time or reduce inference latency |
| `Camera matrix not available` | No camera_info received | Check camera_info_topic parameter |
| `Waiting for camera info...` | Node started before camera | Normal on startup, should auto-resolve |

## Performance

- **Detection Rate:** ~20 Hz (configurable via `publish_rate_hz`)
- **Latency:** ~50-100ms (detection + pose estimation + TF transform)
- **CPU Usage:** ~5-10% (single-threaded Python)
- **Memory:** ~50-100 MB

## Development

### Code Structure

```
realtime_detect/
├── apriltag_box_pose_node.py
│   ├── AprilTagBoxPoseNode (main class)
│   ├── camera_info_callback() → Store intrinsics
│   ├── detection_callback() → Buffer detections
│   ├── timer_callback() → Main processing loop
│   ├── compute_pose_from_detection() → Extract 6DOF
│   ├── estimate_pose_from_homography() → SVD-based pose
│   ├── transform_pose_to_map() → TF2 transform
│   └── create_*_marker() → Visualization
└── launch/apriltag_box_pose.launch.py
    ├── Launch apriltag_ros detector
    └── Launch realtime_detect node
```

### Extending the Node

1. **Add pose filtering (e.g., Kalman filter):**
   - Modify `timer_callback()` to smooth poses over time

2. **Add per-tag tracking:**
   - Store detection history per tag ID
   - Implement temporal coherence checks

3. **Add confidence metrics:**
   - Use detection.decision_margin and hamming distance
   - Publish confidence alongside poses

## License

Apache-2.0

## References

- [AprilTag Homepage](https://april.eecs.umich.edu/software/apriltag/)
- [apriltag_ros GitHub](https://github.com/christianrauch/apriltag_ros)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [TF2 User Guide](https://docs.ros.org/en/humble/Concepts/Advanced/Tf2/Main.html)
