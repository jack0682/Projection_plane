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

  # 가우시안 노이즈로 [1, 0, 0, 1] 근처의 약간 틀어진 평면 생성
  # sigma_n: 법선 벡터 흔들림 (작을수록 [1,0,0]에 가까움)
  # sigma_d: 거리 d 흔들림
  read a b c d < <(awk 'BEGIN {
    srand();
    sigma_n = 0.05;
    sigma_d = 0.1;
    # Box-Muller transform for Gaussian noise
    u1 = rand(); u2 = rand();
    g1 = sqrt(-2*log(u1)) * cos(2*3.14159265*u2);
    g2 = sqrt(-2*log(u1)) * sin(2*3.14159265*u2);
    u3 = rand(); u4 = rand();
    g3 = sqrt(-2*log(u3)) * cos(2*3.14159265*u4);
    g4 = sqrt(-2*log(u3)) * sin(2*3.14159265*u4);
    a = 1.0 + sigma_n * g1;
    b = 0.0 + sigma_n * g2;
    c = 0.0 + sigma_n * g3;
    d = 1.0 + sigma_d * g4;
    # normalize normal vector
    len = sqrt(a*a + b*b + c*c);
    a = a / len; b = b / len; c = c / len;
    printf "%.4f %.4f %.4f %.4f", a, b, c, d;
  }')

  printf "#%-3d: [%7.4f, %7.4f, %7.4f, %7.4f]\n" "$count" "$a" "$b" "$c" "$d"

  # 평면 발행
  ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
    "{data: [$a, $b, $c, $d]}" > /dev/null 2>&1

  sleep 2
done
