"""
manual_drive.launch.py

manual_ws: manual-joystick-driving-only workspace. Nav2/MPPI autonomous
stack (controller_server, lifecycle_manager, local_costmap, EKF,
robot_state_publisher) and the flipper (MD400T) subsystem have both been
stripped out of this workspace entirely -- this covers only the RMD-X8
drive motors and their manual safety chain.

Includes only what manual driving + its safety net needs:
    joy_node -> joy_mux_node -> current_ramp_node -> rmd_x8_driver_node
    myahrs_driver_node (IMU) -> stability_monitor_node

can_interface defaults to the real bus ('can_drive'); pass
'can_interface:=vcan0' to dry-run against a virtual bus.

The DRIVE MOTOR intervention path (/cmd_vel_safety, subscribed to
directly by rmd_x8_driver_node, bypassing joy_mux_node entirely) is the
only automatic safety intervention path and remains fully active
regardless of mode.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')

    return LaunchDescription([

        DeclareLaunchArgument(
            'can_interface',
            default_value='can_drive',
            description=(
                "SocketCAN interface for rmd_x8_driver_node. Default is "
                "the real bus ('can_drive'); override with "
                "'can_interface:=vcan0' on the ros2 launch command line "
                "(see RUNNING.md) to dry-run against a virtual bus with "
                "no motors attached."
            ),
        ),

        # [조이스틱 드라이버] 실제 PS4 DualShock 장치를 읽어 /joy 발행
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'device_name': '',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen',
        ),

        # [수동 전용 MUX] /cmd_vel 출력을 current_ramp_node로 우회시켜서
        # 전류 소프트 램프를 거치도록 함 (joy_mux_node.cpp 자체는 수정하지 않음)
        Node(
            package='robot_bringup',
            executable='joy_mux_node',
            name='joy_mux_node',
            remappings=[
                ('/cmd_vel', '/cmd_vel_manual_raw'),
            ],
            output='screen',
        ),

        # [전류 소프트 램프] 22A가 정상 운용 소프트 개입 임계값 (45A/3초
        # 하드컷오프와 별개 레이어 -- 이 값은 "최대로 낼 수 있는 전류"가
        # 아니라 "언제부터 회전을 미리 줄이기 시작할지"의 트리거임).
        # TEMP: 무한궤도 체인 장착 후 첫 실차 테스트라 20A로 더 낮춰서
        # 시작 -- 실측 회전 마찰 전류 스파이크 크기가 아직 검증되지
        # 않았기 때문. 정격 연속전류(17.6A) 위 여유는 남겨서 정상
        # 주행에서 상시 개입하지 않도록 함. 실제 거동 확인 후 22A로 복귀 검토.
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[{
                'soft_current_limit_a': 20.0,
                'ramp_down_gain': 0.05,
                'ramp_up_rate': 0.2,
                'control_rate_hz': 50.0,
            }],
            output='screen',
        ),

        # [RMD-X8 CAN 구동 드라이버] can_interface 인자로 can_drive/vcan0 전환
        Node(
            package='rmd_x8_driver',
            executable='rmd_x8_driver_node',
            name='rmd_x8_driver',
            parameters=[{
                'can_interface': can_interface,
                'left_motor_can_id': 1,
                'right_motor_can_id': 2,
                'effective_track_width_m': 0.4904,
                'wheel_radius_m': 0.1125,
                'external_gear_ratio': 1.0,
                # Rated speed 127 RPM = 762 dps output shaft => ~1.50 m/s
                # (MyActuator X Series-V4 product manual). Rated-based cap
                # would be 700 dps (~1.37 m/s); TEMP further lowered to
                # ~40% of that (280 dps, ~0.55 m/s) for the first real
                # chain-on, real can_drive test. Raise gradually only after
                # confirming clean low-speed behavior, with user approval.
                'max_wheel_speed_dps': 280.0,
            }],
            output='screen',
        ),

        # [myAHRS+ IMU 드라이버]
        Node(
            package='myahrs_driver',
            executable='myahrs_driver_node',
            name='myahrs_driver',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 460800,
                'frame_id': 'imu_link',
                'orientation_covariance_roll': 0.00000594,
                'orientation_covariance_pitch': 0.00003487,
                'orientation_covariance_yaw': 0.00051956,
            }],
            output='screen',
        ),

        # [안정성 감시] 전복 방지 + 45A/3초 구동계 과전류 하드컷오프
        # (자율주행 전용 기능이 아니라 수동 조종 중에도 유효한 안전망)
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[{
                'critical_pitch_deg': 25.0,
                'critical_roll_deg': 20.0,
            }],
            output='screen',
        ),
    ])
