



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
  -p bev_meters_per_pixel:=0.03 \
  -p bev_img_width:=200 \
  -p bev_img_height:=200 \
  -p conf_threshold:=0.5 \
  -p min_row_pixels:=5
```

`model_path`와 카메라 마운트 파라미터(`camera_height_m` 등)는 기본값이 있으므로
위 예시에는 생략했다 — 아래 참고.

구독: `/camera/camera/color/image_raw/compressed`, `.../camera_info`,
`/camera/camera/imu`
발행: `/flatdrive/planned_path`(`nav_msgs/Path`, x=전방/y=좌측 m — 실제 출력),
`/planning/target_point`, `/bev/image`, `/bev/mask`, `/bev/debug_overlay`,
`/bev/centerline_overlay`, `/bev/H` (디버그용 중간 결과)



>
> **주의 — 카메라 마운트 파라미터**: `camera_height_m`, `camera_pitch_offset_deg`,
> `camera_roll_offset_deg`, `complementary_filter_alpha`의 기본값은
> `src/dolbotz/utils/attitude.py`의 `MOUNT_*_PLACEHOLDER` /
> `COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER` 상수다 (실측 전 임시값 — 대회장에서
> 자주 바뀔 수 있어 의도적으로 config/*.yaml이 아니라 코드 상수로 둔다).
> `elevation_map`과 파라미터 이름이 같다(동일 카메라). 값을 바꾸는 세 가지 방법:
>   (a) 한 번만 다르게 실행 — `--ros-args -p camera_height_m:=X`로 즉석 오버라이드
>   (b) 이후 계속 이 값을 쓰기 — `attitude.py`의 상수 자체를 수정
>   (c) 카메라별로 실측값을 자동 적용 — `config/calibration/`에 해당 카메라
>       `camera_serial_no`의 캘리브레이션 피클을 넣어두고 `-p
>       camera_serial_no:=117222251401`처럼 지정하면, 그 피클에 있는 값이
>       `attitude.py` 상수보다 우선 적용된다 (피클 스키마는
>       `config/calibration/README.md` 참고 — 아직 피클을 만드는 스크립트는
>       없고 스키마만 정해져 있다).

### slope_decision (뎁스 카메라로 좌우 기울기(roll) 추정)

```bash
ros2 run dolbotz slope_decision --ros-args \
  -p track_width_m:=0.45
```

구독: `/camera/camera/depth/camera_info`,
`/camera/camera/depth/image_rect_raw/compressedDepth`
발행: `/terrain/side_slope_angle_deg`
OpenCV 창(`SlopeVisualizer`)으로 깊이 ROI/기울기 값을 표시하므로 헤드리스 환경에서는 X 디스플레이 필요.

### elevation_map (depth+IMU → 고도맵 게시, gradient_map의 입력을 만듦)

```bash
ros2 run dolbotz elevation_map --ros-args \
  -p depth_topic:=/camera/camera/depth/image_rect_raw/compressedDepth \
  -p camera_info_topic:=/camera/camera/depth/camera_info \
  -p imu_topic:=/camera/camera/imu \
  -p resolution_m:=0.15 \
  -p min_depth_m:=0.5 \
  -p max_depth_m:=4.0 \
  -p blind_fill_forward_m:=0.6
```

구독: `/camera/camera/depth/image_rect_raw/compressedDepth`,
`/camera/camera/depth/camera_info`, `/camera/camera/imu`
발행: `/terrain/elevation_map` (32FC1, m 단위; NaN=미관측)

> **주의 — min_depth_m/blind_fill_forward_m**: 실측 전 임시값입니다
> (`src/dolbotz/elevation_map.py`의 PLACEHOLDER 주석 참고). 실제 하드웨어
> (D455 최소 인식거리 등)에 맞춰 조정이 필요합니다.
>
> **주의 — 카메라 마운트 파라미터**: `camera_height_m`, `camera_pitch_offset_deg`,
> `camera_roll_offset_deg`, `complementary_filter_alpha`의 기본값/오버라이드
> 방법은 `flat_drive` 섹션의 안내와 동일합니다 (같은 상수, 같은 카메라 —
> `src/dolbotz/utils/attitude.py` 참고).

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
- `test_paths.py` — `dolbotz.utils.paths`의 리포/패키지 경로 해석(`get_repo_root`,
  `get_package_share_dir`의 ament_index/폴백 하이브리드, `get_models_dir`,
  `load_calibration`)
- `test_attitude.py` — `dolbotz.utils.attitude`의 마운트 파라미터 상수,
  `resolve_mount_defaults`의 캘리브레이션 피클 오버라이드 우선순위, 그리고
  `ElevationMapNode`/`FlatDriveNode`의 `declare_parameter` 기본값이 실제로 이
  상수를 참조하는지에 대한 회귀 테스트(두 노드를 실제로 생성함)

순수 함수 위주의 단위/벤치마크 테스트이며, `test_attitude.py`의 노드 생성
테스트를 제외하면 ROS 노드(`GradientMapNode`/`ElevationMapNode`/`FlatDriveNode`)
자체는 테스트 대상이 아닙니다. `dolbotz.utils.attitude`(`roll_pitch_from_accel_body`,
`update_complementary_filter`, `R_BODY_TO_OPTICAL`, `resolve_mount_defaults`,
`MOUNT_*_PLACEHOLDER` 등)는 `elevation_map.py`와 `flat_drive.py`가 공유하는
IMU 자세 추정 + 마운트 파라미터 공용 모듈입니다.

## 5. config/ 디렉토리

리포 루트의 `config/`에는 모델 파일과 카메라 캘리브레이션 자리만 둔다 (마운트
파라미터는 위에서 설명한 대로 `attitude.py` 코드 상수로 관리하며, 이번
정리로 `config/camera_extrinsics.yaml`은 제거했다):

- `config/models/` — 학습된 모델 가중치 (`supplybest.pt`, `dolbotz_seg_v1/`).
  `dolbotz.utils.paths.get_models_dir()`로 코드가 실행 환경과 무관하게 찾는다.
  자세한 이동 내역은 `config/models/README.md` 참고. 새로 재학습하려면
  `train_drive_area.py` 실행 전에 `export ROBOFLOW_API_KEY=...`로 환경변수를
  설정해야 한다(코드에 키를 하드코딩하지 말 것).
- `config/model_paths.yaml` — 문서화 + override 템플릿 (실제 로딩에는 쓰이지 않음).
- `config/calibration/` — 카메라별 실측 캘리브레이션 피클이 들어갈 자리
  (아직 생성 스크립트 없음). 스키마/네이밍은 `config/calibration/README.md` 참고.




# 로봇 수동 조종 실행 가이드

## 사전 준비 (최초 1회 또는 재부팅 후 매번)

### CAN 인터페이스 활성화
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive

# 확인
ip -details link show can_drive

## 1. CAN 드라이버 (터미널 1)
cd ~/manual2_ws
source install/setup.bash
ros2 run can_driver can_driver_node --ros-args -p can_channel:=can_drive

## 2. 조이스틱 수동 조종 (터미널 2)
cd ~/manual2_ws
source install/setup.bash
ros2 launch manual_joy_control manual_control.launch.py

## 확인용 (선택)

### 터미널 3 - 조이스틱 raw 입력 확인
source ~/manual2_ws/install/setup.bash
ros2 topic echo /joy

### 터미널 4 - 최종 모터 속도 명령 확인
source ~/manual2_ws/install/setup.bash
ros2 topic echo /motor_speed_cmd





# 로봇팔 D435I — arm_pickup 전용
ros2 run realsense2_camera realsense2_camera_node --ros-args -p serial_no:="'339222071362'" -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true -p enable_infra1:=false -p enable_infra2:=false -p enable_gyro:=false -p enable_accel:=false

ros2 run dolbotz arm_pickup
ros2 run dolbotz arm_visualizer

# 로봇팔 rosbag
ros2 bag record -o arm_pickup_bag \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
  /camera/camera/aligned_depth_to_color/camera_info

ros2 bag record -o arm_pickup_bag -a


# 주행 D455
ros2 run realsense2_camera realsense2_camera_node --ros-args -p serial_no:="'117222251401'" -p enable_color:=true -p enable_depth:=true -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 -p enable_infra1:=false -p enable_infra2:=false