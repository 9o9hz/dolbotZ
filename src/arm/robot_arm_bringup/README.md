# robot_arm_bringup

ros2_control, 게임패드 입력, Manual100 명령, 운용 모드와 보호 정지 노드를 실행합니다.

## 권장 실행 진입점

```bash
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py
```

`robot_arm_bringup.launch.py`는 현재 `manual_total_control.launch.py`를 포함하는 대표 진입점입니다. RMD 단독 시험에는 `test_rmd_only.launch.py`를 사용합니다.

## 실행되는 노드

| 노드 | 역할 |
|---|---|
| `ros2_control_node` | 하드웨어와 controller 관리 |
| `robot_state_publisher` | URDF 기반 TF 발행 |
| `joy_node` | 게임패드 입력 발행 |
| `safety_manager` | OFF/SEMIAUTO/MANUAL 상태와 보호 정지 관리 |
| `gamepad_position_controller` | Manual100 관절 위치 명령 생성 |
| `semiauto_placeholder` | 향후 반자동 제어용 빈 자리 |

## 기본 게임패드 버튼

Linux `/joy` 배열 기준이며 컨트롤러 종류나 드라이버에 따라 번호가 다를 수 있습니다.

| 기능 | 기본 index |
|---|---:|
| 제어 ON/OFF | 9 |
| SEMIAUTO/MANUAL 전환 | 5 |
| 보호 정지 | 10 |

현재 Manual100 축은 shoulder index 3, elbow index 4입니다. 실제 매핑은 다음 명령으로 먼저 확인합니다.

```bash
ros2 topic echo /joy
```

## 제어 상태

```text
OFF --제어 버튼--> SEMIAUTO --모드 버튼--> MANUAL
 ^                      ^                       |
 └──── 제어 버튼 ───────┘                       |
                                                v
                               보호 정지 --> ESTOP_LATCHED
```

보호 정지는 현재 프로세스를 재시작해야 해제됩니다. 이 로직은 제어 명령을 차단하고 감속·위치 유지를 수행하는 소프트웨어 보호 기능이며, 안전 인증 E-stop은 아닙니다.

## 주요 토픽

| 토픽 | 형식 | 용도 |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | 운용자 입력 |
| `/joint_states` | `sensor_msgs/JointState` | 관절 측정값 |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | 관절 위치 명령 |
| `/control/mode` | `std_msgs/String` | 현재 운용 모드 |
| `/control/manual_enabled` | `std_msgs/Bool` | Manual100 활성 상태 |
| `/control/semiauto_enabled` | `std_msgs/Bool` | 반자동 활성 상태 |
| `/control/protective_stop` | `std_msgs/Bool` | 래치된 보호 정지 |

## 관절 추가 시 주의

`manual_total_position_node.py`의 `JOINTS`, `rates`, `lower_limits`, `upper_limits`는 controller YAML과 길이 및 순서가 같아야 합니다. 자세한 활성화 순서는 [description README](../robot_arm_description/README.md)를 참고합니다.

