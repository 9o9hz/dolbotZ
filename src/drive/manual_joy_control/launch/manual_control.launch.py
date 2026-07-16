from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='manual_joy_control',
            executable='manual_joy_control_node',
            name='manual_joy_control_node',
            output='screen',
        ),
    ])