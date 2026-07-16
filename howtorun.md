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
  -p bev_meters_per_pixel:=0.03 \
  -p bev_img_width:=200 \
  -p bev_img_height:=200 \
  -p conf_threshold:=0.5 \
  -p min_row_pixels:=5
```

`model_path`와 카메라 마운트 파라미터(`camera_height_m` 등)는 기본값이 있으므로
위 예시에는 생략했다 — 아래 참고.

구독: `/camera/camera/color/image_raw`, `.../camera_info`, `/camera/camera/imu`
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

구독: `/camera/camera/depth/camera_info`, `/camera/camera/depth/image_rect_raw`
발행: `/terrain/side_slope_angle_deg`
OpenCV 창(`SlopeVisualizer`)으로 깊이 ROI/기울기 값을 표시하므로 헤드리스 환경에서는 X 디스플레이 필요.

### elevation_map (depth+IMU → 고도맵 게시, gradient_map의 입력을 만듦)

```bash
ros2 run dolbotz elevation_map --ros-args \
  -p depth_topic:=/camera/camera/depth/image_rect_raw \
  -p camera_info_topic:=/camera/camera/depth/camera_info \
  -p imu_topic:=/camera/camera/imu \
  -p resolution_m:=0.15 \
  -p min_depth_m:=0.5 \
  -p max_depth_m:=4.0 \
  -p blind_fill_forward_m:=0.6
```

구독: `/camera/camera/depth/image_rect_raw`, `/camera/camera/depth/camera_info`, `/camera/camera/imu`
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

### arm_pickup (YOLO로 박스 탐지 → 3D 좌표 퍼블리시)

```bash
ros2 run dolbotz arm_pickup --ros-args \
  -p target_class:=supply_box
```

`model_path` 기본값은 `dolbotz.utils.paths.get_models_dir()` 기준
`config/models/supplybest.pt`다 (flat_drive와 동일한 방식으로 cwd/사용자 홈
경로 무관하게 해석됨). 다른 가중치를 쓰려면 `-p model_path:=/abs/path`로
오버라이드하세요. `ultralytics`가 설치되어 있지 않으면 탐지 기능이 비활성화됩니다.

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

## 6. manual_ws 실행 가이드 (수동 주행 전용, can_drive)

이 워크스페이스는 **조이스틱 수동 주행에 필요한 패키지만** 모아둔 순수 manual 전용
워크스페이스입니다. 플리퍼(MD400T)와 Nav2 자율주행 스택(controller_server, EKF,
robot_state_publisher 등)은 코드/설정/의존성까지 전부 빠져 있습니다.

### 구성 패키지 (3개)

- `rmd_x8_driver` — RMD-X8-120 구동모터 CAN 드라이버
- `myahrs_driver` — IMU 드라이버
- `robot_bringup` — `joy_mux_node`, `current_ramp_node`, `stability_monitor_node`
  + `manual_drive.launch.py`

### 구성 노드 (6개)

`joy_node` → `joy_mux_node` → `current_ramp_node` → `rmd_x8_driver` +
`myahrs_driver`(IMU) → `stability_monitor_node`

---

### 6.1 빌드 - 터미널 1

```bash
source /opt/ros/humble/setup.bash
cd ~/manual_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 6.2 can_drive 연결 (USB-CAN 어댑터) - 터미널 2


```bash
ip link show can0                                        # 어댑터가 실제로 잡힌 이름 확인 (다르면 아래 can0를 그 이름으로 교체)
sudo ip link set can0 down
sudo ip link set can0 name can_drive                     # can_drive로 리네임
sudo ip link set can_drive up type can bitrate 1000000   # 1Mbps로 업
ip link show can_drive                                    # state UP 확인
```

### 6.3 실행 전 정리 (중복 노드 방지) - 터미널 3

```bash
source /opt/ros/humble/setup.bash
source ~/manual_ws/install/setup.bash
pkill -f "ros2 launch robot_bringup"
pkill -f "joy_node"; pkill -f "joy_mux_node"
pkill -f "current_ramp_node"; pkill -f "stability_monitor_node"
pkill -f "rmd_x8_driver_node"; pkill -f "myahrs_driver_node"
sleep 1
ros2 node list   # 아무것도 안 나와야 정상
```

### 6.4 실행

```bash
ros2 launch robot_bringup manual_drive.launch.py
```

(`can_interface` 기본값이 이미 `can_drive`라 별도 인자 없이 그대로 실행하면 됩니다.)

### 6.5 확인 - 터미널 4

```bash
source /opt/ros/humble/setup.bash
ros2 node list     # 6개 노드 정상 기동 확인
ros2 topic list    # /flipper_*, /cmd_vel_auto 등 자율주행/플리퍼 토픽 없어야 정상
```

### 6.6 구동 속도 프리셋 (조이스틱 L1/R1)

- **R1**: 한 단계 가속 (40% → 70% → 100%, 정격 700dps 기준)
- **L1**: 한 단계 감속
- 기본값은 시작 시 자동으로 40%(280dps)로 설정됨
- 확인/수동 조정:
  ```bash
  ros2 param get /rmd_x8_driver max_wheel_speed_dps
  ros2 param set /rmd_x8_driver max_wheel_speed_dps 700.0   # 100% 강제 지정 시
  ```

### 6.7 종료

```bash
pkill -f "ros2 launch robot_bringup"
ros2 daemon stop   # 노드 목록이 stale하게 남을 때만
```

---

### 안전 주의사항 (manual_ws)

- **100% 프리셋(정격 700dps)은 안전장치 없이 즉시 전환됩니다.** 처음 가동 시
  반드시 리프트 위에서 40%/70%부터 단계적으로 확인 후 100%를 시도하세요.
- `/cmd_vel_safety` 경로(자세 임계각·과전류 비상 개입)는 `joy_mux_node`와
  무관하게 항상 활성 상태입니다.
- 조이스틱 SHARE 버튼(자율/수동 전환)은 **누르지 마세요.** 이 워크스페이스엔
  자율주행 명령 소스가 아예 없어서, AUTONOMOUS로 전환되면 조이스틱 입력이
  무시됩니다 (0.3초 후 안전상 속도 0 고정). 다시 SHARE를 누르면 복구됩니다.
- can_drive 해제: `sudo ip link set can_drive down`




로봇팔 depth cam

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p serial_no:="'339222071362'" \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p align_depth.enable:=true
```

> **주의**: `-p` 옵션 사이에 빈 줄을 넣으면 안 됩니다. bash의 `\` 줄이음이
> 빈 줄에서 끊겨서 `align_depth.enable:=true`가 실제로 적용되지 않고,
> `aligned_depth_to_color` 토픽이 발행되지 않는 원인이 됩니다. 위 코드블럭을
> 그대로 복사하거나, 아래처럼 한 줄로 실행해도 됩니다.

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p serial_no:="'339222071362'" -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true
```

```bash
ros2 run dolbotz arm_pickup
```

```bash
ros2 run dolbotz arm_visualizer
```