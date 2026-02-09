# Projection Plane - ROS2 Package

PLY 포인트 클라우드를 임의의 평면에 정사영(Orthographic Projection)하여 2D 이미지로 변환하는 ROS2 패키지입니다.

## Features

- **PLY 파일 로드 및 평면 투영**: 임의의 평면 방정식(`ax + by + cz + d = 0`)에 대한 정사영
- **ROS2 토픽 발행**:
  - `/input_cloud` (`sensor_msgs/PointCloud2`): 입력 포인트 클라우드
  - `/projection_image` (`sensor_msgs/Image`): Raw BGR8 투영 이미지
  - `/projection_pose` (`geometry_msgs/PoseStamped`): 카메라 pose (placeholder)
- **동적 평면 구독**: `/input_plane` 토픽으로 평면 파라미터 동적 변경
- **TRANSIENT_LOCAL QoS**: 늦게 구독하는 노드도 최신 메시지 수신 가능

## Directory Structure

```
projection_plane/
├── config/
│   └── projection_params.yaml    # 기본 파라미터 설정
├── launch/
│   └── projection.launch.py      # 런치 파일
├── projection_plane/
│   ├── __init__.py
│   ├── core.py                   # 투영 알고리즘 (순수 Python)
│   └── projection_node.py        # ROS2 노드
├── package.xml
├── setup.py
└── setup.cfg
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select projection_plane --symlink-install
source install/setup.bash
```

## Usage

### 기본 실행
```bash
ros2 launch projection_plane projection.launch.py
```

### 파라미터 오버라이드
```bash
ros2 launch projection_plane projection.launch.py \
  input_file:="/path/to/your.ply" \
  plane_n:="[1.0, 0.0, 0.0]" \
  plane_d:="0.0" \
  pixels_per_unit:="500.0"
```

### 파일 저장 활성화
```bash
ros2 launch projection_plane projection.launch.py save_image:=True
```

### Plane토픽 발행 형식
```bash
ros2 topic pub /input_plane shape_msgs/msg/Plane "{coef: [1.2, -5.3, 0.3, 5.0]}" 
```


## Parameters

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `input_file` | string | (필수) | PLY 파일 경로 |
| `output_file` | string | `output/projection_{timestamp}.png` | 출력 파일 경로 |
| `plane_n` | float[] | `[0, 0, 1]` | 평면 법선 벡터 |
| `plane_d` | float | `0.0` | 평면 방정식의 d 값 |
| `pixels_per_unit` | float | `500.0` | 해상도 (pixels/meter) |
| `origin_mode` | string | `"mean"` | 원점 계산 방식 (`mean` or `closest`) |
| `depth_priority_far` | bool | `False` | 깊이 우선순위 (True: 먼 점 우선) |
| `save_image` | bool | `False` | 파일 저장 여부 |

## Topics

### Published
| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/input_cloud` | `PointCloud2` | TRANSIENT_LOCAL | 입력 포인트 클라우드 |
| `/projection_image` | `Image` | TRANSIENT_LOCAL | Raw BGR8 투영 이미지 |
| `/projection_pose` | `PoseStamped` | TRANSIENT_LOCAL | 카메라 pose (placeholder) |

### Subscribed
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/input_plane` | `shape_msgs/Plane` | 평면 파라미터 동적 업데이트 |

## Configuration File

`config/projection_params.yaml`:
```yaml
projection_node:
  ros__parameters:
    plane_n: [0.0, 0.0, 1.0]
    plane_d: 0.0
    pixels_per_unit: 500.0
    depth_priority_far: false
    origin_mode: "mean"
    output_file: "/home/jack/ros2_ws/output/projection_{timestamp}.png"
    input_file: "/path/to/input.ply"
```

## Troubleshooting


### 이미지가 RViz에서 안 보임
QoS 설정 확인:
```bash
ros2 topic info /projection_image --verbose
```
<img width="1310" height="508" alt="image" src="https://github.com/user-attachments/assets/19fa7165-1142-456a-ba09-36a93ecb4a1a" />


잘보이는것같기도 하지만 왼쪽 아래의 평면방정식을 바굴때마다 이미지의 모양이 달라져야하는데 그게 안된다. 퍼블리시 속도가 매우느리고 왜 최신화가 느린지는 모르겠다. 

## Dependencies

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `shape_msgs`
- `python3-opencv`, `python3-numpy`, `python3-scipy`
- `sensor_msgs_py`, `open3d`

## License

MIT License
