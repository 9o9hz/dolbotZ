"""Explicitly named RMD-only hardware and manual-control test entry point."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    legacy_launch = os.path.join(
        get_package_share_directory('robot_arm_bringup'),
        'launch', 'robot_arm_rmd_control.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(legacy_launch))
    ])
