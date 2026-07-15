# rmd_x8_driver

RMD-X8-120 좌/우 구동모터용 ROS2 CAN 드라이버. MYACTUATOR의 RMD-X 프로토콜(`Servo Motor Control Protocol V4.01`)의 속도 폐루프 제어 커맨드(`0xA2`)를 사용합니다.

이 패키지는 국방로봇경진대회 제어 가이드(`autonomous_control_guide.md`) 3.1절(구동 역기구학) / 3.3절(전류 보호)의 하위 구현체입니다.

## 프로토콜 검증

`rmd_x8_protocol.py`의 프레임 인코딩/디코딩은 매뉴얼 2.20.4절의 실제 예제 값과 바이트 단위로 일치하는지 단위 테스트로 검증되어 있습니다:

```bash
cd rmd_x8_driver
python3 test/test_protocol.py
```

CAN 하드웨어 없이도 이 테스트는 통과해야 합니다 (순수 인코딩/디코딩 로직만 검증).

## 설치

```bash
pip install python-can --break-system-packages
```

`python3-can`은 보통 rosdep으로 해결되지 않으므로 위처럼 직접 설치해야 합니다.

## CAN 인터페이스 준비 (실물 테스트 전 필수)

```bash
sudo ip link set can0 up type can bitrate 1000000   # RMD-X 매뉴얼: CAN 1Mbps 고정
candump can0                                          # 두 모터가 정상 응답하는지 먼저 확인
```

## 빌드 및 실행

```bash
colcon build --packages-select rmd_x8_driver
source install/setup.bash
ros2 launch rmd_x8_driver rmd_x8_driver.launch.py
```

또는 직접 파라미터를 override:

```bash
ros2 run rmd_x8_driver rmd_x8_driver_node --ros-args \
  -p can_interface:=can0 \
  -p left_motor_can_id:=1 \
  -p right_motor_can_id:=2
```

## ⚠️ 실물에서 반드시 확인/캘리브레이션해야 하는 파라미터

`config/rmd_x8_params.yaml`에 있는 아래 값들은 **기본값(placeholder)이며 실물 확인 없이 신뢰하면 안 됩니다**:

| 파라미터 | 이유 |
|---|---|
| `left_direction_sign` / `right_direction_sign` | 좌우 모터가 미러링 마운트되어 있으면 한쪽 부호를 반전해야 함. 벤치에서 `cmd_vel linear.x = 0.1` 명령 시 양쪽 바퀴가 같은 방향(전진)으로 도는지 육안 확인 필수 |
| `wheel_radius_m` | 실측 필요 |
| `external_gear_ratio` | RMD-X8 출력축에 바퀴/스프로킷이 직결이면 1.0, 추가 감속단이 있으면 그 비율 |
| `effective_track_width_m` | **기하학적 실측값이 아니라 제어 가이드 3.1절의 제자리 N회전 캘리브레이션 값**을 넣어야 함 |
| `max_wheel_speed_dps` | 실제 모터/기구 한계에 맞게 조정 |

## 전류 보호(제어 가이드 3.3절)와의 연동

이 노드는 **전류 컷오프 로직 자체를 구현하지 않습니다** (의도적 — 3.3절 설계상 그 로직은 별도의 `current_monitor_node`가 담당). 대신 이 노드는:

- `/wheel/joint_states`의 `effort` 필드에 각 모터의 토크전류(iq, A 단위)를 발행
- `/wheel/motor_status`(DiagnosticArray)에 전압/온도/에러 플래그를 발행 (2Hz로 Motor Status 1, `0x9A` 폴링)

`current_monitor_node`는 이 두 토픽, 특히 `/wheel/joint_states.effort`를 구독해서 20A/45A/3초 로직을 구현하면 됩니다.

**주의**: `iq`(토크전류, d-q축 전류)는 각 상전류(phase current, `0x9D` 명령으로 읽는 A/B/C상 전류)와 다릅니다. 정격/피크 임계값을 iq 기준으로 잡을지 상전류 기준으로 잡을지는 RMD-X8 데이터시트의 정격 표기 기준과 맞춰서 확인이 필요합니다. 필요하면 `0x9D` 폴링을 이 노드에 추가하는 것도 어렵지 않습니다 (요청하시면 추가해드릴 수 있습니다).

## 안전 관련 설계 노트

- RMD-X 드라이버 자체에 **500ms 하트비트 보호**가 있어, 이 노드가 죽어도 하드웨어가 결국 정지합니다. 다만 그 전에 부드럽게 서지 않고 갑자기 멈추므로, 이 노드는 `cmd_vel_timeout_s`(기본 0.3s, 하드웨어 컷오프보다 짧음)로 먼저 감속 정지시킵니다.
- `destroy_node()`에서 종료 시 `0x81`(모터 정지) 명령을 먼저 보내려고 시도합니다 (best-effort).
- 각도 언랩(`AngleUnwrapper`)은 매뉴얼상 응답 각도 필드가 `int16`(±32767°)라는 하드웨어 제약을 소프트웨어로 보정한 것입니다. 매우 긴 주행에서는 주기적으로 `0x92`(멀티턴 절대각, int32) 명령으로 재동기화하는 걸 추가로 권장하나, 현재 버전에는 포함되어 있지 않습니다.

## 다음 단계

이 드라이버 노드가 실물에서 검증되면, 제어 가이드의 빌드 순서(이전 대화 참고)대로 다음은 myAHRS+ IMU 드라이버 → `robot_localization` EKF(wheel odom만) → Nav2 MPPI 순서입니다.