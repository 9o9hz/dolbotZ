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
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_joy',
            default_value='true',
            description='Launch joy_node from the arm bringup'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(legacy_launch),
            launch_arguments={
                'launch_joy': LaunchConfiguration('launch_joy'),
            }.items())
    ])
