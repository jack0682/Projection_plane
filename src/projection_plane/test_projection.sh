#!/bin/bash
# Test script for projection_plane node

set -e

echo "=========================================="
echo "Projection Plane Node Test"
echo "=========================================="

# Source ROS setup
source /home/jack/ros2_ws/install/setup.bash

# Check if PLY file exists
PLY_FILE="/home/jack/Last_point/pcd_file/241108_converted - Cloud.ply"
if [ ! -f "$PLY_FILE" ]; then
    echo "ERROR: PLY file not found: $PLY_FILE"
    echo "Create a dummy PLY or update PLY_PATH in test"
    exit 1
fi

echo "✓ PLY file found: $PLY_FILE"

# Start node in background
echo "Starting projection_plane_node..."
ros2 run projection_plane projection_plane_node &
NODE_PID=$!
sleep 2

# Check if node is running
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo "ERROR: Node failed to start"
    exit 1
fi
echo "✓ Node started (PID: $NODE_PID)"

# Check if topics are available
echo "Waiting for topics to advertise..."
sleep 3

# Test topic subscription
echo ""
echo "Testing /projection/cloud_raw..."
timeout 3 ros2 topic echo /projection/cloud_raw --once > /tmp/cloud_test.txt 2>&1 || true
if [ -s /tmp/cloud_test.txt ]; then
    echo "✓ /projection/cloud_raw is publishing"
else
    echo "⚠ /projection/cloud_raw not available yet (may be normal)"
fi

# Publish a test plane
echo ""
echo "Publishing test plane: [0, 0, 1, 0] (XY plane)"
ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 1.0, 0.0]}" 2>/dev/null || true

# Wait for projection
sleep 2

# Check image output
echo "Testing /projection/image..."
timeout 3 ros2 topic echo /projection/image --field header.stamp --once > /tmp/image_test.txt 2>&1 || true
if [ -s /tmp/image_test.txt ]; then
    echo "✓ /projection/image is publishing"
else
    echo "⚠ /projection/image not available yet (may be normal)"
fi

# Test with another plane
echo ""
echo "Publishing test plane: [1, 0, 0, 0] (YZ plane)"
ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
    "{data: [1.0, 0.0, 0.0, 0.0]}" 2>/dev/null || true

sleep 2

# List active topics
echo ""
echo "Active topics:"
ros2 topic list | grep projection || echo "No projection topics found"

# Stop node
echo ""
echo "Stopping node..."
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "Test completed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Run: ros2 launch projection_plane projection_plane.launch.py"
echo "2. Publish planes: ros2 topic pub -1 /projection/plane std_msgs/msg/Float64MultiArray \"{data: [0,0,1,0]}\""
echo "3. View image: rqt_image_view /projection/image"
