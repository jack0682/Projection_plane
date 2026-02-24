# 최종 2.5D 6DOF 파이프라인 설계 문서

**작성일**: 2026-02-23
**상태**: 🚀 구현 준비 완료
**목표**: Map frame 기준 multi-view 2.5D 포즈 추적

---

## 1️⃣ 원하는 것 (Vision)

평면 방정식(map frame)으로부터 가상 카메라 포즈를 **일관되게 정의**하고, 그 가상 카메라의 **pinhole 모델**로 image/depth/xyz를 동시에 렌더링한 뒤, SAM3 마스크를 xyz에 역매핑하여 물체의 **front-face 중심과 yaw(2.5D pose)**를 map frame에서 추정한다.

여러 평면(다중 뷰)에서 얻은 map-pose 관측을 **거리+yaw 기반**으로 연관시켜 TTL/누적/연속성 보정으로 **"놓치지 않는" 트랙**을 유지한다.

---

## 2️⃣ 우려하는 것 (Risk)

| # | 위험 | 영향 | 대책 |
|---|------|------|------|
| (A) | FOV + 1088×1088 혼동 | 거리/스케일 오추정 | ✅ xyz_image 필수사용 |
| (B) | Plane만으로 pose 비유일 | yaw/roll 튀기 | ✅ 게이지 규칙 명시 |
| (C) | 마스크 오염/희박 점군 | front-face 불안정 | ✅ 강건 통계 추가 |
| (D) | 깊이 0.1 고정의 모순 | position 왜곡 | ✅ size.z만 고정 |
| (E) | 토픽 동기화 실패 | 마스크↔xyz 불일치 | ✅ exact 필터 적용 |
| (F) | 단순 매칭 비용 | 유사 객체 ID 스왑 | ✅ 비용식 확장 |

---

## 3️⃣ 상세 대책

### (A) FOV + 1088×1088 혼동 방지

**문제**:
```
❌ "밑면이 1088 픽셀" ≠ "1.5m 거리"
❌ FOV 각도를 직접 거리 계산에 사용하면 안 됨
```

**해결**:
```cpp
// ✅ 반드시 intrinsics + xyz_image 사용
double fx = 385.0, fy = 385.0;  // focal length (pixels)
double cx = 320.0, cy = 240.0;  // principal point

// 3D 복원: pinhole 역투영
x_cam = (pixel_x - cx) * depth / fx
y_cam = (pixel_y - cy) * depth / fy
z_cam = depth

// 절대 하지 말 것:
// ❌ size_world = bbox_width * (some_distance / 1088)
```

**구현 체크**:
- [ ] `perspective_projection.hpp`: intrinsics만 사용 (확인)
- [ ] `front_face_6dof.py`: xyz_image → camera_frame 3D (확인)
- [ ] ROS 토픽: `/projection/xyz` 발행 (TODO)

---

### (B) Plane → Camera Pose 게이지 고정

**문제**:
```
평면 ax + by + cz + d = 0은 2개 자유도를 고정하지 못함:
  - h: 평면으로부터 거리 (0.1m vs 1.0m)
  - φ: 평면 내 roll (카메라 x축 방향)
```

**규칙 (명시적 정의)**:
```cpp
// z_cam: 평면 법선 (시야 방향)
Vec3d z_cam = n_normalized;

// world_up으로 roll 고정
Vec3d up_hint = (|n.z| < 0.95) ? [0,0,1] : [1,0,0];
Vec3d x_cam = normalize(up_hint × z_cam);
Vec3d y_cam = z_cam × x_cam;

// Gauge 1: 거리 h (상수로 고정)
double h = 1.0;  // 미터 (튜닝 가능)
cam_pos = anchor + h * z_cam;

// Gauge 2: roll = 0 (위 규칙으로 자동 고정)
R = [x_cam | y_cam | z_cam];
assert(det(R) > 0);  // right-handed 확인
```

**구현 체크**:
- [ ] `perspective_projection.hpp`: up_hint fallback (확인)
- [ ] Gauge 주석 추가 (TODO)
- [ ] 특이 케이스(n과 up 평행) 처리 (확인)

---

### (C) Front-face 추정 강건성

**현재 (취약)**:
```python
# ❌ 모든 3D 점에서 min(depth) 사용 → 노이즈/이상치에 민감
front_center = points_map[np.argmin(depths)]
```

**개선**:
```python
# ✅ 강건 추정
def estimate_front_face_robust(points_map, depths, confidence_mask):
    # 1. 이상치 제거 (RANSAC 또는 percentile)
    valid_mask = depths < np.percentile(depths, 90)  # 상위 10% 제외
    points_clean = points_map[valid_mask]

    # 2. 깊이 기반 평면 피팅 (z 최소 영역)
    z_min_idx = np.argsort(points_clean[:, 2])[:int(0.2*len(points_clean))]
    front_plane_pts = points_clean[z_min_idx]

    # 3. 가장 밀도 높은 영역 선택 (DBSCAN)
    # 또는 simple: 상위 20%의 평균
    front_center = np.mean(front_plane_pts, axis=0)

    return front_center, confidence=0.8
```

**구현 체크**:
- [ ] `front_face_6dof.py`: percentile 필터 추가 (TODO)
- [ ] 점군 밀도 체크 추가 (TODO)

---

### (D) Position의 깊이는 고정 금지 (Size의 깊이만 고정)

**문제**:
```
❌ 고정된 깊이로 인한 모순:
  - 평면1에서: position.z = 1.0
  - 평면2에서: 같은 물체인데 position.z = 1.5?
  → map-frame 추적이 깨짐
```

**해결**:
```python
# ✅ Position: xyz_image에서 실제값 사용
position = xyz_image[mask_pixels] @ cam_rot.T + cam_pos
position = np.mean(position, axis=0)  # 앞면 중심

# ✅ Size: 깊이만 고정
size = [width_m, height_m, 0.1]  # 0.1은 상수, position.z와 무관
```

**의미**:
- `position.z`: 앞면까지의 실제 거리 (map frame)
- `size[2]`: 물체 두께 추정값 (grasp에 사용)

**구현 체크**:
- [ ] `front_face_6dof.py`: position.z 실제값 (확인)
- [ ] size.z만 상수 처리 (확인)

---

### (E) 토픽 동기화 (Critical!)

**문제**:
```
image → SAM3 (100ms)
depth → 발행된 지 이미 50ms 전
xyz → 또 다른 시점
→ 마스크는 t=100인데 xyz는 t=50

결과: mask 픽셀과 xyz_image가 엇갈림 → 3D 오류
```

**해결**:
```cpp
// projection_plane_node.cpp
{
  auto now = this->get_clock()->now();

  // 1. 같은 렌더 패스에서 생성
  render_image_and_depth_and_xyz(plane);

  // 2. 같은 timestamp로 발행
  msg_image.header.stamp = now;
  msg_depth.header.stamp = now;
  msg_xyz.header.stamp = now;
  msg_meta.header.stamp = now;

  publisher_image->publish(msg_image);
  publisher_depth->publish(msg_depth);
  publisher_xyz->publish(msg_xyz);
}
```

**Python 구독**:
```python
# node.py
def sync_callback(image, depth, xyz, meta):
    # ✅ message_filters.Synchronizer로 exact 매칭
    # 같은 timestamp만 처리
    assert image.header.stamp == depth.header.stamp
    assert depth.header.stamp == xyz.header.stamp

    process(image, depth, xyz, meta)

sync = message_filters.Synchronizer(
    message_filters.TimeSynchronizer(
        [sub_image, sub_depth, sub_xyz, sub_meta],
        queue_size=5
    ),
    sync_callback
)
```

**구현 체크**:
- [ ] C++: 동일 timestamp 발행 (TODO in projection_plane_node.cpp)
- [ ] Python: message_filters.Synchronizer 사용 (TODO in node.py)
- [ ] 차이 > 10ms 드롭 로직 (TODO)

---

### (F) 매칭 비용식 확장

**현재 (단순)**:
```python
cost = ||pos - pos_t|| + 0.5 * |yaw - yaw_t|
```

**개선**:
```python
def compute_association_cost(det, track):
    # 위치 거리
    pos_dist = np.linalg.norm(det['position'] - track.pos)

    # Yaw 차이 (각도)
    yaw_diff = abs(det['yaw'] - track.yaw)
    yaw_diff = min(yaw_diff, 2*np.pi - yaw_diff)

    # Size 유사도 (width, height)
    size_sim = 1 - np.dot(det['size'][:2], track.size[:2]) / (
        np.linalg.norm(det['size'][:2]) * np.linalg.norm(track.size[:2]) + 1e-6
    )

    # View count 가중 (자주 본 것을 더 믿기)
    view_weight = 1.0 / (1.0 + 0.1 * track.view_count)

    # Confidence 히스테리시스 (기존 트랙을 선호)
    conf_bonus = 0.1 if track.is_alive() else 0.0

    # 결합 비용
    cost = (
        pos_dist * 1.0 +
        yaw_diff * 0.5 +
        size_sim * 0.3 +
        conf_bonus
    ) * view_weight

    return cost
```

**구현 체크**:
- [ ] `multiview_tracker.py`: 비용식 확장 (TODO)
- [ ] 가중치 튜닝 (TODO)

---

## 4️⃣ 3개 결의문 (필수)

### 결의문 1: FOV 기반 밑면 추정 금지

**선언**: 3D 좌표 복원은 **반드시 `xyz_image` 또는 `intrinsics 기반 ray + PLY 교차`**에서 수행한다. 해상도(1088)와 FOV를 혼동하여 직접 거리를 계산하지 않는다.

### 결의문 2: Plane→Cam 게이지 규칙 명시

**선언**: 평면 방정식만으로는 카메라 포즈가 유일하지 않다. 다음 규칙을 모든 구현에서 명시한다:
- `z_cam = -n_hat` (평면 법선 바라보기)
- `x_cam ∝ up_hint × z_cam` (world_up으로 roll=0 고정)
- 특이 케이스(n ∥ up): fallback 정의

### 결의문 3: Position의 깊이는 고정하지 않는다

**선언**: `position.z`는 xyz_image에서 실제값을 사용한다. **고정은 `size[2]`에만 적용**한다.
- ✅ `position = [x_world, y_world, z_world_actual]`
- ✅ `size = [width, height, 0.1]` (0.1은 상수)

---

## 5️⃣ 구현 체크리스트

### Phase 1: Core 모듈
- [ ] `perspective_projection.hpp` (평면 → camera, pinhole 투영)
- [ ] `front_face_6dof.py` (mask → 앞면 2.5D, 강건성 추가)
- [ ] `multiview_tracker.py` (map 기준, 비용식 확장)

### Phase 2: ROS2 노드
- [ ] `projection_plane_node.cpp` (perspective 렌더링 + xyz_image)
- [ ] `node.py` (depth + xyz 구독, 동기화, front_face 호출)
- [ ] `tracker_node.py` (NEW, detections 추적)

### Phase 3: 토픽 & 동기화
- [ ] `/projection/image`, `/projection/depth`, `/projection/xyz`, `/projection/proj_meta` (동일 timestamp)
- [ ] `message_filters.Synchronizer` (Python)
- [ ] 동기화 실패 드롭 로직

### Phase 4: 검증
- [ ] 단일 평면 테스트 (정적)
- [ ] 다중 평면 테스트 (뷰 변경)
- [ ] ID 지속성 검증
- [ ] 통계 (age, view_count 분포)

---

## 6️⃣ 성공 기준 (현실적)

| 기준 | 설명 | 확인 방법 |
|-----|------|---------|
| **위치 일관성** | 다른 평면에서도 같은 박스의 position 차이 < 0.2m | plot(pos_plane1 - pos_plane2) |
| **ID 지속성** | 20프레임 이상 추적 시 ID 변경 0회 | track.age histogram |
| **Yaw 연속성** | 인접 프레임 yaw 변화 < 0.3rad | plot(Δyaw) |
| **거짓 추적** | 실제 없는 객체 생성 0회 | track.age < 3 개수 |
| **놓친 객체** | 실제 보인 객체 > 50% 추적 | manual validation |

---

## 📌 최종 아키텍처 다이어그램

```
[Plane eq]
    ↓
[Virtual Cam Pose (게이지 고정)]
    ↓
[Pinhole Projection (intrinsics)]
    ├─ image (1088×1088, RGB8)
    ├─ depth (1088×1088, FLOAT32)
    └─ xyz (1088×1088, FLOAT32×3, camera frame)
    ↓ [✅ 동일 timestamp/frame_id]
[SAM3 Detection + Mask]
    ↓
[Front-face 6DOF (강건 통계)]
    ├─ position [x, y, z_actual]
    ├─ yaw
    └─ size [w, h, 0.1]
    ↓
[Map-frame Tracker (확장 비용)]
    ├─ 거리 + yaw + size + view_count
    ├─ TTL + 누적
    └─ /projection/6dof/tracks (최종 2.5D)
```

---

## 🎯 최종 말

이 문서가 "우려"를 명시한 이유는, 구현할 때 이 6개 지점에서 "대충 하면 작동하는 것 같은데 추적이 흔들려"가 되기 때문이다.

**반드시 확인**:
1. xyz_image를 사용하는가?
2. 게이지 규칙이 명시되어 있는가?
3. position.z는 고정하지 않았는가?
4. 토픽 동기화를 정확히 했는가?
5. 매칭 비용에 size, view_count를 포함했는가?
6. Front-face가 이상치 필터를 하는가?

6개 모두 YES면 → "놓치지 않는 2.5D 추적" ✅
