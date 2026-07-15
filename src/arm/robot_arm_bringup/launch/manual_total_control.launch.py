from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('ifname', default_value='can_arm'),
        DeclareLaunchArgument('shoulder_actuator_id', default_value='4'),
        DeclareLaunchArgument('elbow_actuator_id', default_value='5'),
        DeclareLaunchArgument('wrist_actuator_id', default_value='6'),
        # Per-joint RMD operation limits in degrees per second.
        # DeclareLaunchArgument('shoulder_max_velocity', default_value='15.0'),
        # DeclareLaunchArgument('elbow_max_velocity', default_value='20.0'),
        # DeclareLaunchArgument('wrist_max_velocity', default_value='30.0'),
        # Low-speed calibration/test limits.
        DeclareLaunchArgument('shoulder_max_velocity', default_value='5.0'),
        DeclareLaunchArgument('elbow_max_velocity', default_value='5.0'),
        DeclareLaunchArgument('wrist_max_velocity', default_value='5.0'),
        DeclareLaunchArgument('timeout', default_value='0'),
        DeclareLaunchArgument('dxl_port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('dxl_baud_rate', default_value='1000000'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('control_toggle_button', default_value='9'),
        DeclareLaunchArgument('manual_mode_button', default_value='5'),
        DeclareLaunchArgument('emergency_stop_button', default_value='10'),
        DeclareLaunchArgument('wrist_axis', default_value='1'),
        DeclareLaunchArgument(
            'rmd_hardware_components',
            default_value="['shoulder_rmd', 'elbow_rmd', 'wrist_rmd']"),
    ]
    description_share = get_package_share_directory('robot_arm_description')
    xacro_file = description_share + '/urdf/manual_total_robot.urdf.xacro'
    controllers = description_share + '/config/manual_total_controllers.yaml'
    xacro_command = [FindExecutable(name='xacro'), ' ', xacro_file,
                     ' ifname:=', LaunchConfiguration('ifname'),
                     ' shoulder_actuator_id:=', LaunchConfiguration('shoulder_actuator_id'),
                     ' elbow_actuator_id:=', LaunchConfiguration('elbow_actuator_id'),
                     ' wrist_actuator_id:=', LaunchConfiguration('wrist_actuator_id'),
                     ' shoulder_max_velocity:=', LaunchConfiguration('shoulder_max_velocity'),
                     ' elbow_max_velocity:=', LaunchConfiguration('elbow_max_velocity'),
                     ' wrist_max_velocity:=', LaunchConfiguration('wrist_max_velocity'),
                     ' timeout:=', LaunchConfiguration('timeout'),
                     ' dxl_port_name:=', LaunchConfiguration('dxl_port_name'),
                     ' dxl_baud_rate:=', LaunchConfiguration('dxl_baud_rate')]
    robot_description = {'robot_description': ParameterValue(Command(xacro_command), value_type=str)}
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[controllers, robot_description], output='screen')
    state_spawner = Node(package='controller_manager', executable='spawner',
                         arguments=['joint_state_broadcaster', '-c', '/controller_manager'])
    position_spawner = Node(package='controller_manager', executable='spawner',
                            arguments=['position_controller', '-c', '/controller_manager'])
    spawn_in_order = RegisterEventHandler(OnProcessExit(target_action=state_spawner,
                                                        on_exit=[position_spawner]))
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
        control, state_spawner, spawn_in_order,
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'dev': LaunchConfiguration('joy_dev'), 'deadzone': 0.08}]),
        Node(package='robot_arm_bringup', executable='safety_manager.py',
             name='safety_manager', output='screen', parameters=[{
                 'control_toggle_button': LaunchConfiguration('control_toggle_button'),
                 'manual_mode_button': LaunchConfiguration('manual_mode_button'),
                 'emergency_stop_button': LaunchConfiguration('emergency_stop_button')}]),
        Node(package='robot_arm_bringup', executable='gamepad_position_controller.py',
             name='gamepad_position_controller', output='screen', parameters=[{
                 'wrist_axis': LaunchConfiguration('wrist_axis')}]),
        Node(package='robot_arm_bringup', executable='semiauto_placeholder.py',
             name='semiauto_placeholder', output='screen'),
    ]
    return LaunchDescription(arguments + nodes)
