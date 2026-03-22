"""
hdt_arm_sim.launch.py
=====================
ใช้ Gazebo แทน Dynamixel ทั้งหมด

วิธีใช้:
  ros2 launch hdt_arm_bringup hdt_arm_sim.launch.py
  ros2 launch hdt_arm_bringup hdt_arm_sim.launch.py start_bridge:=true

Digital twin (รันพร้อมกัน 2 terminal):
  terminal 1: ros2 launch hdt_arm_bringup hdt_arm_real.launch.py
  terminal 2: ros2 launch hdt_arm_bringup hdt_arm_sim.launch.py start_bridge:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    desc_pkg = get_package_share_directory('hdt_arm_description')

    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(desc_pkg, '..'), ':' + desc_pkg],
    )
    gz_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_MODEL_PATH',
        value=[os.path.join(desc_pkg, '..')],
    )

    declared_arguments = [
        DeclareLaunchArgument('start_rviz',       default_value='true'),
        DeclareLaunchArgument('init_position',    default_value='true'),
        DeclareLaunchArgument('start_bridge',     default_value='false',
            description='เปิด joint_state_bridge เพื่อให้ sim follow real'),
        DeclareLaunchArgument('start_imu_teleop', default_value='false'),
    ]

    start_rviz       = LaunchConfiguration('start_rviz')
    init_position    = LaunchConfiguration('init_position')
    start_bridge     = LaunchConfiguration('start_bridge')
    start_imu_teleop = LaunchConfiguration('start_imu_teleop')

    # process xacro ด้วย ros_namespace=sim
    xacro_file = os.path.join(desc_pkg, 'urdf', 'hdt_arm.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={
        'use_sim':       'true',
        'ros_namespace': 'sim',
    })
    robot_desc_sim = doc.toprettyxml(indent='  ')

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'config', 'ros2_controllers_sim.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'rviz', 'hdt_arm.rviz',
    ])
    initial_positions_yaml = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'config', 'home_position_params.yaml',
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'
            ),
            '/gz_sim.launch.py',
        ]),
        launch_arguments={
            'gz_args': [
                os.path.join(desc_pkg, 'worlds', 'default.sdf'),
                ' -v 1 -r',
            ],
        }.items(),
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='sim',
        parameters=[{
            'robot_description': robot_desc_sim,
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }],
        remappings=[('robot_description', '/sim/robot_description')],
        output='screen',
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc_sim,
            '-name',   'hdt_arm',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-allow_renaming', 'true',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/sim/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/sim/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    home_position_node = Node(
        package='hdt_arm_teleop',
        executable='home_position',
        namespace='sim',
        parameters=[
            initial_positions_yaml,
            {'use_sim_time': True},
        ],
        output='screen',
        condition=IfCondition(init_position),
    )

    imu_kinematics_node = Node(
        package='hdt_arm_teleop',
        executable='imu_kinematics_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(start_imu_teleop),
    )

    joint_state_bridge = Node(
        package='hdt_arm_teleop',
        executable='joint_state_bridge',
        output='screen',
        condition=IfCondition(start_bridge),
    )

    # gz_spawn เสร็จ → jsb
    on_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # jsb เสร็จ → arm_controller
    on_jsb_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # arm_controller เสร็จ → rviz + home + imu + bridge
    on_arm_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[rviz_node, home_position_node, imu_kinematics_node, joint_state_bridge],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        gz_model_path,
        *declared_arguments,
        gazebo,
        bridge_clock,
        robot_state_publisher,
        gz_spawn,
        on_spawn_done,
        on_jsb_done,
        on_arm_done,
    ])