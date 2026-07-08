# dolbotz 실행 방법

ROS2 (Humble) 패키지. 이 디렉터리(`/home/j/dolbotZ`) 자체가 워크스페이스 겸 패키지 루트입니다
(`package.xml`이 루트에 있고, `colcon build`도 이 디렉터리에서 실행합니다).

## 0. 사전 준비

```bash
source /opt/ros/humble/setup.bash
```

필요 의존성 (`package.xml` 기준):
- rclpy, sensor_msgs, std_msgs, nav_msgs, geometry_msgs, visualization_msgs, cv_bridge, image_transport
- python3-numpy, python3-opencv, python3-scipy
- RealSense 카메라 사용 시 `realsense2_camera` 패키지 (`ros2 pkg list | grep realsense`로 설치 확인)
- `arm_pickup` 노드는 추가로 `ultralytics`(YOLO), `message_filters` 필요 — 없으면 해당 노드만 비활성 처리됨
- `flat_drive` 노드는 추가로 `ultralytics`, `openvino`(OpenVINO IR 세그멘테이션 모델 추론용) 필요
  — `pip install ultralytics openvino` — 없으면 해당 노드만 비활성 처리됨

## 1. 빌드

```bash
cd /home/j/dolbotZ
colcon build --symlink-install --packages-select dolbotz
source install/setup.bash
```

`--symlink-install`을 쓰면 `src/dolbotz/*.py` 수정이 재빌드 없이 바로 반영됩니다.

## 2. 카메라 드라이버 실행

각 노드가 구독하는 토픽에 맞게 필요한 스트림만 켜서 실행합니다.

```bash
# 컬러만 필요할 때 (flat_drive)
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=false \
  -p rgb_camera.profile:=640x480x30

# 뎁스만 필요할 때 (slope_decision)
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=false -p enable_depth:=true
```

## 3. 노드 실행

### flat_drive (평지 주행가능영역 추종 — 세그멘테이션 → BEV투영 → centerline 경로 발행)

`gradient_map`(경사 구간)과 역할이 대칭이다: 이 노드도 실제 속도/조향 명령이
아니라 좌표(경로)만 발행하고 끝난다. 평지/경사 전환 판단은 별도 decision
노드(Phase F)가 담당한다.

```bash
ros2 run dolbotz flat_drive --ros-args \
  -p model_path:=/home/jecs/dolbotZ/runs/segment/dolbotz_seg_v1/weights/best_openvino_model \
  -p camera_height_m:=0.5 \
  -p camera_pitch_offset_deg:=10.0 \
  -p camera_roll_offset_deg:=0.0 \
  -p bev_meters_per_pixel:=0.03 \
  -p bev_img_width:=200 \
  -p bev_img_height:=200 \
  -p conf_threshold:=0.5 \
  -p min_row_pixels:=5
```

구독: `/camera/camera/color/image_raw`, `.../camera_info`, `/camera/camera/imu`
발행: `/flatdrive/planned_path`(`nav_msgs/Path`, x=전방/y=좌측 m — 실제 출력),
`/planning/target_point`, `/bev/image`, `/bev/mask`, `/bev/debug_overlay`,
`/bev/centerline_overlay`, `/bev/H` (디버그용 중간 결과)

> **주의**: `model_path`는 상대경로면 프로세스 작업 디렉터리 기준이므로, 워크스페이스
> 루트가 아닌 곳에서 실행할 경우 절대경로로 지정하세요. `camera_height_m`,
> `camera_pitch_offset_deg`, `camera_roll_offset_deg`, `bev_meters_per_pixel`,
> `bev_img_width/height`, `min_row_pixels`, `complementary_filter_alpha`는 실측
> 전 임시값입니다 (`src/dolbotz/flat_drive.py`의 PLACEHOLDER 주석 참고). 카메라
> 마운트 파라미터는 `elevation_map`과 이름이 같으므로(동일 카메라) launch에서
> 값을 공유하세요.

### slope_decision (뎁스 카메라로 좌우 기울기(roll) 추정)

```bash
ros2 run dolbotz slope_decision --ros-args \
  -p track_width_m:=0.45
```

구독: `/camera/camera/depth/camera_info`, `/camera/camera/depth/image_rect_raw`
발행: `/terrain/side_slope_angle_deg`
OpenCV 창(`SlopeVisualizer`)으로 깊이 ROI/기울기 값을 표시하므로 헤드리스 환경에서는 X 디스플레이 필요.

### elevation_map (depth+IMU → 고도맵 게시, gradient_map의 입력을 만듦)

```bash
ros2 run dolbotz elevation_map --ros-args \
  -p depth_topic:=/camera/camera/depth/image_rect_raw \
  -p camera_info_topic:=/camera/camera/depth/camera_info \
  -p imu_topic:=/camera/camera/imu \
  -p camera_height_m:=0.5 \
  -p camera_pitch_offset_deg:=10.0 \
  -p camera_roll_offset_deg:=0.0 \
  -p resolution_m:=0.15 \
  -p min_depth_m:=0.5 \
  -p max_depth_m:=4.0 \
  -p blind_fill_forward_m:=0.6
```

구독: `/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/camera_info`, `/camera/camera/imu`
발행: `/terrain/elevation_map` (32FC1, m 단위; NaN=미관측)

> **주의**: `camera_height_m`, `camera_pitch_offset_deg`, `camera_roll_offset_deg`,
> `min_depth_m`, `blind_fill_forward_m`, `complementary_filter_alpha`는 실측 전
> 임시값입니다 (`src/dolbotz/elevation_map.py`의 PLACEHOLDER 주석 참고). 실제
> 하드웨어(장착 각도, D435i 최소 인식거리 등)에 맞춰 조정이 필요합니다.

### gradient_map (고도맵 → 경사 필드 → 슬로프 제한 경로 계획)

```bash
ros2 run dolbotz gradient_map --ros-args \
  -p resolution_m:=0.15 \
  -p max_slope_deg:=30.0
```

구독: `/terrain/elevation_map` (32FC1, m 단위)
발행: `/terrain/gradient_x`, `/terrain/gradient_y`, `/terrain/gradient_magnitude`,
`/terrain/gradient_direction`, `/terrain/slope_deg`, `/terrain/planned_path` (`nav_msgs/Path`)

> **주의**: `max_slope_deg`는 실측 전 임시값(30°)입니다
> (`src/dolbotz/gradient_map.py`의 `MAX_SLOPE_DEG_PLACEHOLDER` 참고).
> `/terrain/elevation_map`을 게시하는 `elevation_map` 노드를 먼저(또는 함께) 띄워야 합니다.

### arm_pickup (YOLO로 박스 탐지 → 3D 좌표 퍼블리시)

```bash
ros2 run dolbotz arm_pickup --ros-args \
  -p model_path:=/path/to/best.pt \
  -p target_class:=supply_box
```

`model_path`는 필수 파라미터. `ultralytics`가 설치되어 있지 않으면 탐지 기능이 비활성화됩니다.

## 4. 테스트 실행

`gradient_map.py`/`elevation_map.py`/`flat_drive.py`가 최상단에서
`rclpy`/`nav_msgs`/`geometry_msgs`/`cv_bridge` 등을 import하므로, ROS 환경을
source한 뒤 실행해야 합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/j/dolbotZ
python3 -m pytest test/ -v
```

- `test_gradient_field.py` — `compute_gradient_field`, `plan_path_on_slope_field`
- `test_elevation_map.py` — depth 역투영, IMU 상보필터, `camera_body_to_level_matrix`,
  고도 그리드 투영 (blind-fill 포함)
- `test_flat_drive.py` — 지면 호모그래피 재유도(`ground_to_image_homography` 등,
  마운트/섀시 기울기 조합에 대한 독립 물리 검증 포함), 세그멘테이션 마스크 →
  BEV → centerline 경로 추출

순수 함수에 대한 단위/벤치마크 테스트만 포함되어 있고, ROS 노드
(`GradientMapNode`/`ElevationMapNode`/`UnifiedDriveNode`) 자체는 테스트 대상이
아닙니다. `dolbotz.utils.attitude`(`roll_pitch_from_accel_body`,
`update_complementary_filter`, `R_BODY_TO_OPTICAL`)는 `elevation_map.py`와
`flat_drive.py`가 공유하는 IMU 자세 추정 공용 모듈입니다.
