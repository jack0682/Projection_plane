#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/jack/ros2_ws/install/setup.bash

echo "🚀 Random Plane Publisher Started"
echo "📡 Publishing to /projection/plane every 2 seconds"
echo "Press Ctrl+C to stop"
echo ""

count=0

while true; do
  count=$((count + 1))

  # awk를 사용한 랜덤 값 생성
  read a b c d < <(awk 'BEGIN {
    srand();
    a = int(rand() * 200 - 100) / 100.0;
    b = int(rand() * 200 - 100) / 100.0;
    c = int(rand() * 200 - 100) / 100.0;
    d = int(rand() * 400 - 200) / 100.0;
    printf "%.4f %.4f %.4f %.4f", a, b, c, d;
  }')

  printf "#%-3d: [%7.4f, %7.4f, %7.4f, %7.4f]\n" "$count" "$a" "$b" "$c" "$d"

  # 평면 발행
  ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
    "{data: [$a, $b, $c, $d]}" > /dev/null 2>&1

  sleep 2
done
