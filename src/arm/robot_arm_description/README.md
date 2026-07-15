# robot_arm_description

로봇팔의 링크·관절 형상과 ros2_control 하드웨어 선언, controller 설정을 관리합니다.

## 주요 파일

- `urdf/manual_total_robot.urdf.xacro`: 통합 로봇 모델과 실제 하드웨어 선언
- `urdf/test_urdf.urdf.xacro`: RMD 단독 시험 모델
- `config/manual_total_controllers.yaml`: Manual100 controller 관절 배열
- `config/robot_arm_rmd_controllers.yaml`: RMD 단독 시험 controller 설정
- `launch/robot_arm.launch.py`: robot_state_publisher 표시용 launch

## 관절명

| 순번 | 관절명 | 구동기 | 현재 상태 |
|---|---|---|---|
| 1 | `base_rotate_joint` | Dynamixel | 비활성 |
| 2 | `shoulder_lift_joint` | RMD, CAN ID 4 | 활성 |
| 3 | `elbow_joint` | RMD, CAN ID 5 | 활성 |
| 4 | `wrist_joint` | RMD, CAN ID 6 | 비활성 |
| 5 | `gripper_joint` | Dynamixel | 비활성 |

CAN ID는 launch 인자로 변경할 수 있으며 표의 값은 기본값입니다.

## wrist 활성화 체크리스트

1. `manual_total_robot.urdf.xacro`에서 `wrist_rmd` actuator 선언을 활성화합니다.
2. `manual_total_controllers.yaml`의 동일한 위치에 `wrist_joint`를 추가합니다.
3. `manual_total_position_node.py`의 `JOINTS`, `rates`, `lower_limits`, `upper_limits`를 같은 순서로 확장합니다.
4. 낮은 `max_velocity`에서 관절 방향과 영점을 검증합니다.

## Dynamixel 활성화 체크리스트

1. `manual_total_robot.urdf.xacro`의 `dynamixel_system` 블록을 활성화합니다.
2. 실제 포트, baud rate, 모터 ID와 model 파일을 확인합니다.
3. `manual_total_controllers.yaml`에 `gripper_joint`, `base_rotate_joint`를 원하는 명령 배열 순서로 추가합니다.
4. `manual_total_position_node.py`의 관절 배열과 입력·limit를 정확히 같은 순서로 확장합니다.
5. launch의 Dynamixel 인자 전달을 복구합니다.
6. 모터를 무부하 상태에서 하나씩 검증한 뒤 통합합니다.

관절 순서가 controller와 명령 배열에서 다르면 다른 모터가 움직일 수 있으므로 이름뿐 아니라 배열 순서까지 검토해야 합니다.

## 모델 검사

```bash
xacro src/robot_arm_description/urdf/manual_total_robot.urdf.xacro > /tmp/manual_total_robot.urdf
check_urdf /tmp/manual_total_robot.urdf
```
