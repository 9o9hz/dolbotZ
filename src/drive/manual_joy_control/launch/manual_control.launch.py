from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # [하림 수정] drive/arm이 조이스틱 하나를 토글로 공유하는 구조로 바뀌면서,
    # joy_node는 둘 중 한쪽에서만 띄우면 된다. arm 쪽(robot_arm_bringup)을 먼저
    # 띄웠다면 launch_joy:=false로 이 launch의 joy_node는 꺼야 함.
    joy_dev_cmd = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='공유 조이스틱 장치 경로'
    )
    launch_joy_cmd = DeclareLaunchArgument(
        'launch_joy',
        default_value='true',
        description='이 launch 파일에서 joy_node를 함께 실행할지 여부 (arm 쪽에서 이미 띄웠다면 false)'
    )
    joy_dev = LaunchConfiguration('joy_dev')
    launch_joy = LaunchConfiguration('launch_joy')

    return LaunchDescription([
        joy_dev_cmd,
        launch_joy_cmd,
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(launch_joy),
        ),
        Node(
            package='manual_joy_control',
            executable='manual_joy_control_node',
            name='manual_joy_control_node',
            output='screen',
        ),
    ])
