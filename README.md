# Projection Plane - C++ ROS2 Implementation

**상태**: ✅ COMPLETE (2026-02-10)
**버전**: 0.1.0
**언어**: C++17, Python3

---

## 📋 개요

PLY 포인트 클라우드(14,640,946 포인트)를 임의의 평면에 **정사영(Orthographic Projection)**하여 2D 이미지로 변환하는 고성능 ROS2 노드입니다.

### 핵심 특징
- ✅ **원본 Python 알고리즘과 동일한 구현**
- ✅ **비동기 처리**: 워커 스레드로 논블로킹 투영
- ✅ **평면 업데이트 합치기**: 빠른 업데이트 대응
- ✅ **두 가지 래스터 모드**: Baseline (정확성) / Fast-Stable (성능)
- ✅ **실시간 이미지 발행**: 2.5 FPS with 14M points

---

## 🚀 빠른 시작

### 1. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select projection_plane
source install/setup.bash
```

### 2. 노드 실행 (터미널 1)

```bash
ros2 launch projection_plane projection_plane.launch.py
```

### 3. 평면 발행 (터미널 2)

```bash
# XY 평면 (Z축 법선)
ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 1.0, 0.0]}"

# 랜덤 평면 계속 발행 (0.5초 간격)
bash /home/jack/ros2_ws/random_plane_publisher.sh
```

### 4. 결과 확인

```bash
# 이미지 보기
rqt_image_view /projection/image &

# 포인트 클라우드 확인
ros2 topic echo /projection/cloud_raw --once | head -20

# 토픽 정보
ros2 topic info /projection/image
```

---

## 📊 성능 벤치마크

**테스트 환경**:
- 포인트 클라우드: 14,640,946 points
- 평면 업데이트: 0.5초 간격
- 모드: Baseline (기본)

**측정 결과**:

| 항목 | 값 |
|------|-----|
| **프레임 레이트** | 2.49 FPS |
| **처리량** | 36,446,244 points/sec |
| **응답률** | 132.2% (버퍼링) |
| **총 처리** | 1,141,993,788 points |

---

## 🔧 매개변수

**주요 파라미터** (`config/projection_params.yaml`):

```yaml
# 입출력
ply_path: "/home/jack/Last_point/pcd_file/241108_converted - Cloud.ply"
pixels_per_unit: 500.0  # 해상도 (픽셀/단위)

# 이미지 크기
width: -1   # -1 = 자동
height: -1  # -1 = 자동

# 투영 설정
origin_mode: "mean"        # "mean" 또는 "closest"
depth_mode: "abs"          # "abs" 또는 "signed"
depth_priority_far: false  # true = 먼 점 우선

# 렌더링
point_size: 1              # 픽셀당 크기
raster_mode: "baseline"    # "baseline" 또는 "fast_stable"
publish_rate_hz: 10.0      # 발행 빈도

# 범위 처리
robust_range: false        # 백분위수 기반 범위
percentile_low: 1.0
percentile_high: 99.0

# Up hint (선택사항)
up_hint_x: 0.0
up_hint_y: 0.0
up_hint_z: 1.0

# 디버그
save_png_path: ""  # PNG 저장 경로 (비움 = 저장 안함)
```

---

## 🔌 ROS2 토픽

### 구독 (Input)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/projection/plane` | `std_msgs/Float64MultiArray` | 평면 [a, b, c, d] |
| `/camera/pose_in` | `geometry_msgs/PoseStamped` | 카메라 포즈 (선택사항) |

### 발행 (Output)

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/projection/image` | `sensor_msgs/Image` (BGR8) | 10 | 투영 이미지 |
| `/projection/cloud_raw` | `sensor_msgs/PointCloud2` | Transient Local | 원본 클라우드 |
| `/projection/camera_pose` | `geometry_msgs/PoseStamped` | 10 | 릴레이된 포즈 |

---

## 📁 파일 구조

```
/home/jack/ros2_ws/
├── PROJECTION_PLANE_GUIDE.md          # 완전한 가이드 (755줄)
├── README.md                          # 이 파일
├── random_plane_publisher.sh          # 랜덤 평면 발행 스크립트
├── benchmark_projection.py            # 성능 측정 스크립트
└── src/projection_plane/
    ├── CMakeLists.txt                 # 빌드 설정
    ├── package.xml                    # 패키지 메타
    ├── include/projection_plane/
    │   ├── projection_math.hpp        # 기하학 함수 (277줄)
    │   └── rasterizer.hpp             # Baseline/Fast-Stable (295줄)
    ├── src/
    │   └── projection_plane_node.cpp   # 메인 노드 (400줄)
    ├── launch/
    │   └── projection_plane.launch.py  # 런치 파일
    └── config/
        └── projection_params.yaml      # 기본 파라미터
```

---

## 🛠️ 알고리즘

### 투영 파이프라인 (10단계)

1. **평면 검증**: 법선 (a,b,c) 비퇴화 확인
2. **정규화**: `n_hat = n / ||n||`
3. **Up Hint 선택**: 평면과 평행하지 않은 벡터
4. **정규직교 기저**: `t1 = normalize(cross(n_hat, up))`, `t2 = cross(n_hat, t1)`
5. **정사영**: `p_proj = p - ((n·p + d) / ||n||²) * n`
6. **원점 계산**: mean 또는 closest
7. **UV 매핑**: `u = (p_proj - origin)·t1`, `v = (p_proj - origin)·t2`
8. **깊이 계산**: `depth = (n·p + d) / ||n||`
9. **이미지 크기**: 자동 또는 명시적
10. **래스터화**: Z-버퍼 또는 정렬

### 래스터화 모드

**Baseline** (기본, 정확성 보장):
```
순차적 Z-버퍼, O(N) 시간
```

**Fast-Stable** (성능 최적화, point_size=1일 때 동일):
```
정렬 기반, O(N log N) 시간
```

---

## 🧪 테스트

### 자동 테스트

```bash
python3 /home/jack/ros2_ws/benchmark_projection.py
```

출력 예시:
```
Frame rate: 2.49 FPS
Points processed: 36,446,244 points/sec
Response rate: 132.2%
```

---

## 🐛 트러블슈팅

### 이미지가 검은색이면

```bash
# 평면 확인 (XY 평면 시도)
ros2 topic pub --once /projection/plane std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 1.0, 0.0]}"

# pixels_per_unit 감소
ros2 param set /projection_plane_node pixels_per_unit 100.0

# 명시적 크기 설정
ros2 param set /projection_plane_node width 512
ros2 param set /projection_plane_node height 512
```

### 느린 처리

```bash
# Fast-Stable 모드로 전환 (point_size=1일 때)
ros2 param set /projection_plane_node raster_mode fast_stable

# 발행 빈도 감소
ros2 param set /projection_plane_node publish_rate_hz 5.0
```

---

## 📈 추가 리소스

- **완전한 가이드**: `/home/jack/ros2_ws/PROJECTION_PLANE_GUIDE.md` (755줄)
- **성능 측정**: `python3 benchmark_projection.py`
- **평면 발행**: `bash random_plane_publisher.sh`

---

## 📝 라이선스

Apache License 2.0

---

## 👤 Contact

Jack <jack0682@github.com>
