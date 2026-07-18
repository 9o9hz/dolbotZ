"""Canonical, role-oriented entry point for the complete robot arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    legacy_launch = os.path.join(
        get_package_share_directory('robot_arm_bringup'),
        'launch', 'manual_total_control.launch.py')
    # [하림 수정] launch_joy를 이 wrapper에서 선언해서 manual_total_control.launch.py로
    # 명시적으로 전달한다. drive 쪽(manual_joy_control)에서 joy_node를 이미 띄웠다면
    # `launch_joy:=false`로 여기서는 중복 실행을 막아야 한다.
    launch_joy_cmd = DeclareLaunchArgument('launch_joy', default_value='true')
    return LaunchDescription([
        launch_joy_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(legacy_launch),
            launch_arguments={'launch_joy': LaunchConfiguration('launch_joy')}.items(),
        )
    ])
