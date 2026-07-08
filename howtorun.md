# dolbotz 실행 방법

ROS2 (Humble) 패키지. 이 디렉터리(`/home/j/dolbotZ`) 자체가 워크스페이스 겸 패키지 루트입니다
(`package.xml`이 루트에 있고, `colcon build`도 이 디렉터리에서 실행합니다).

## 0. 사전 준비

```bash
source /opt/ros/humble/setup.bash
```

필요 의존성 (`package.xml` 기준):
- rclpy, sensor_msgs, std_msgs, nav_msgs, geometry_msgs, visualization_msgs, cv_bridge, image_transport
- python3-numpy, python3-opencv (`cv2.ximgproc` 포함), python3-scipy
- RealSense 카메라 사용 시 `realsense2_camera` 패키지 (`ros2 pkg list | grep realsense`로 설치 확인)
- `arm_pickup` 노드는 추가로 `ultralytics`(YOLO), `message_filters` 필요 — 없으면 해당 노드만 비활성 처리됨

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
# 컬러만 필요할 때 (slope_drive)
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=false \
  -p rgb_camera.profile:=640x480x30

# 뎁스만 필요할 때 (slope_decision)
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=false -p enable_depth:=true
```

## 3. 노드 실행

### slope_drive (카메라 기반 차선 추종 주행 — BEV → 마스크 → 스켈레톤 → 가지치기 → lookahead 조향)

```bash
ros2 run dolbotz slope_drive --ros-args \
  -p camera_height_m:=0.5 \
  -p lookahead_distance_pixels:=50 \
  -p max_speed:=0.2
```

구독: `/camera/camera/color/image_raw`, `.../camera_info`, `/camera/camera/imu`
발행: `/cmd_vel`, `/planning/target_point`, `/bev/image`, `/skeleton_image`, `/pruned_skeleton` 등 (디버그용 중간 결과 포함)

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

`gradient_map.py`가 최상단에서 `rclpy`/`nav_msgs`/`geometry_msgs`를 import하므로,
ROS 환경을 source한 뒤 실행해야 합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/j/dolbotZ
python3 -m pytest test/test_gradient_field.py -v
```

`compute_gradient_field`, `plan_path_on_slope_field` 순수 함수에 대한 단위/벤치마크 테스트만
포함되어 있고, ROS 노드(`GradientMapNode`) 자체는 테스트 대상이 아닙니다.
