from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    shoulder_actuator_id_parameter_name = 'shoulder_actuator_id'
    shoulder_actuator_id = LaunchConfiguration(shoulder_actuator_id_parameter_name)
    elbow_actuator_id_parameter_name = 'elbow_actuator_id'
    elbow_actuator_id = LaunchConfiguration(elbow_actuator_id_parameter_name)
    wrist_actuator_id_parameter_name = 'wrist_actuator_id'
    wrist_actuator_id = LaunchConfiguration(wrist_actuator_id_parameter_name)
    ifname_parameter_name = 'ifname'
    ifname = LaunchConfiguration(ifname_parameter_name)
    joy_dev_parameter_name = 'joy_dev'
    joy_dev = LaunchConfiguration(joy_dev_parameter_name)
    launch_joy_parameter_name = 'launch_joy'
    launch_joy = LaunchConfiguration(launch_joy_parameter_name)
    max_velocity_parameter_name = 'max_velocity'
    max_velocity = LaunchConfiguration(max_velocity_parameter_name)
    timeout_parameter_name = 'timeout'
    timeout = LaunchConfiguration(timeout_parameter_name)
    xacro_file_parameter_name = 'xacro_file'
    xacro_file = LaunchConfiguration(xacro_file_parameter_name)

    shoulder_actuator_id_cmd = DeclareLaunchArgument(
        shoulder_actuator_id_parameter_name,
        default_value='4',
        description='숄더 관절 RMD CAN ID'
    )
    elbow_actuator_id_cmd = DeclareLaunchArgument(
        elbow_actuator_id_parameter_name,
        default_value='5',
        description='엘보 관절 RMD CAN ID'
    )
    wrist_actuator_id_cmd = DeclareLaunchArgument(
        wrist_actuator_id_parameter_name,
        default_value='6',
        description='손목 관절 RMD CAN ID'
    )
    ifname_cmd = DeclareLaunchArgument(
        ifname_parameter_name,
        default_value='can_arm',
        description='CAN 인터페이스 이름'
    )
    launch_joy_cmd = DeclareLaunchArgument(
        launch_joy_parameter_name,
        default_value='true',
        description='이 launch 파일에서 joy_node를 함께 실행할지 여부'
    )
    joy_dev_cmd = DeclareLaunchArgument(
        joy_dev_parameter_name,
        default_value='/dev/input/js0',
        description='PlayStation 조이스틱 장치 경로'
    )
    max_velocity_cmd = DeclareLaunchArgument(
        max_velocity_parameter_name,
        default_value='30.0',
        description='RMD position 명령의 최대 속도(deg/s)'
    )
    timeout_cmd = DeclareLaunchArgument(
        timeout_parameter_name,
        default_value='0',
        description='액추에이터 동작 timeout 시간'
    )
    default_xacro_file = PathJoinSubstitution(
        [
            get_package_share_directory('robot_arm_description'),
            'urdf', 'test_urdf.urdf.xacro'
        ]
    )
    xacro_file_parameter_cmd = DeclareLaunchArgument(
        xacro_file_parameter_name,
        default_value=default_xacro_file,
        description='사용할 xacro URDF 파일. 기본값은 작동 테스트용 임시 URDF'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([xacro_file]), ' ',
            'ifname:=', ifname, ' ',
            'shoulder_actuator_id:=', shoulder_actuator_id, ' ',
            'elbow_actuator_id:=', elbow_actuator_id, ' ',
            'wrist_actuator_id:=', wrist_actuator_id, ' ',
            'max_velocity:=', max_velocity, ' ',
            'timeout:=', timeout
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    controllers = PathJoinSubstitution(
        [
            get_package_share_directory('robot_arm_description'),
            'config',
            'robot_arm_rmd_controllers.yaml',
        ]
    )
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen'
    )
    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    position_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '-c', '/controller_manager']
    )
    controller_spawner_sequence = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[position_controller_spawner_node],
        )
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': joy_dev}],
        condition=IfCondition(launch_joy)
    )
    ld = LaunchDescription()
    ld.add_action(shoulder_actuator_id_cmd)
    ld.add_action(elbow_actuator_id_cmd)
    ld.add_action(wrist_actuator_id_cmd)
    ld.add_action(ifname_cmd)
    ld.add_action(launch_joy_cmd)
    ld.add_action(joy_dev_cmd)
    ld.add_action(max_velocity_cmd)
    ld.add_action(timeout_cmd)
    ld.add_action(xacro_file_parameter_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner_node)
    ld.add_action(controller_spawner_sequence)
    ld.add_action(joy_node)
    return ld
