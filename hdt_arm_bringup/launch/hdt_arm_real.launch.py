"""
hdt_arm_real.launch.py
======================
ควบคุม Dynamixel จริง ไม่มี Gazebo

วิธีใช้:
  ros2 launch hdt_arm_bringup hdt_arm_real.launch.py
  ros2 launch hdt_arm_bringup hdt_arm_real.launch.py start_imu_teleop:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('port_name',            default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_mock_hardware',    default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('start_rviz',           default_value='true'),
        DeclareLaunchArgument('init_position',        default_value='true'),
        DeclareLaunchArgument('start_imu_teleop',     default_value='false'),
    ]

    port_name            = LaunchConfiguration('port_name')
    use_mock_hardware    = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    start_rviz           = LaunchConfiguration('start_rviz')
    init_position        = LaunchConfiguration('init_position')
    start_imu_teleop     = LaunchConfiguration('start_imu_teleop')

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('hdt_arm_description'),
                'urdf', 'hdt_arm.urdf.xacro',
            ]),
            ' use_sim:=false',
            ' ros_namespace:=real',
            ' use_mock_hardware:=',    use_mock_hardware,
            ' mock_sensor_commands:=', mock_sensor_commands,
            ' port_name:=',            port_name,
            ' initial_positions_file:=',
            PathJoinSubstitution([
                FindPackageShare('hdt_arm_description'),
                'config', 'initial_positions.yaml',
            ]),
        ]),
        value_type=str,
    )

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'config', 'ros2_controllers_real.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'rviz', 'hdt_arm.rviz',
    ])
    initial_positions_yaml = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'config', 'home_position_params.yaml',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='real',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
            'publish_frequency': 50.0,
        }],
        remappings=[('robot_description', '/real/robot_description')],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='real',
        parameters=[
            controllers_yaml,
            {'use_sim_time': False},
        ],
        remappings=[('~/robot_description', '/real/robot_description')],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/real/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/real/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    home_position_node = Node(
        package='hdt_arm_teleop',
        executable='home_position',
        namespace='real',
        parameters=[
            initial_positions_yaml,
            {'use_sim_time': False},
        ],
        output='screen',
        condition=IfCondition(init_position),
    )

    imu_kinematics_node = Node(
        package='hdt_arm_teleop',
        executable='imu_kinematics_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(start_imu_teleop),
    )

    on_jsb_done_start_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    on_arm_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[rviz_node, home_position_node, imu_kinematics_node],
        )
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        on_jsb_done_start_arm,
        on_arm_done,
    ])