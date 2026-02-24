#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/jack/ros2_ws/install/setup.bash

echo "🚀 Plane Publisher - Yaw Only Oscillation"
echo "📡 Publishing to /projection/plane every 2 seconds"
echo "Press Ctrl+C to stop"
echo ""

count=0

while true; do
  count=$((count + 1))

  # 정면 기준 (Z축 중심 회전)
  # 기준 평면: XZ 평면 (법선 = [0, 1, 0])
  # Yaw 각도: 가우시안 노이즈 (Gaussian Noise) 적용
  read a b c d < <(awk -v seed="$RANDOM" 'BEGIN {
    srand(seed);

    # Box-Muller 변환으로 가우시안 분포 생성 (평균 0, 표준편차 1)
    u1 = rand();
    while (u1 == 0) u1 = rand();
    u2 = rand();
    z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265 * u2);

    # Yaw 노이즈 (표준편차 = 0.05 라디안, 약 2.8도)
    # 조금 더 크게 흔들리게 하려면 std_dev 값을 키우면 됩니다.
    std_dev = 0.05;
    yaw = z0 * std_dev;

    # 회전된 법선 벡터 (Z축 중심 회전)
    a = sin(yaw);
    b = cos(yaw);
    c = 0.0;
    d = 0.0;

    # 법선 벡터 정규화
    len = sqrt(a*a + b*b + c*c);
    a = a / len;
    b = b / len;
    c = c / len;

    printf "%.4f %.4f %.4f %.4f", a, b, c, d;
  }')

  printf "#%-3d: Yaw Oscillation - [%7.4f, %7.4f, %7.4f, %7.4f]\n" "$count" "$a" "$b" "$c" "$d"

  # 평면 발행
  ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
    "{data: [$a, $b, $c, $d]}" > /dev/null 2>&1

  sleep 2
done
