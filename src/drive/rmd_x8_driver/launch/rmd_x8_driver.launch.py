import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rmd_x8_driver"),
        "config",
        "rmd_x8_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="rmd_x8_driver",
            executable="rmd_x8_driver_node",
            name="rmd_x8_driver",
            output="screen",
            parameters=[config],
        ),
    ])