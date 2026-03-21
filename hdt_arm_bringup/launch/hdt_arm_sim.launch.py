#!/usr/bin/env python3
"""
hdt_arm_sim.launch.py
=====================
Launch file สำหรับ Gazebo Harmonic simulation
ทุก node และ topic จะอยู่ใน namespace /sim

วิธีใช้:
  ros2 launch hdt_arm_bringup hdt_arm_sim.launch.py
  ros2 launch hdt_arm_bringup hdt_arm_sim.launch.py start_bridge:=true
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

    # Gazebo Harmonic ใช้ GZ_SIM_* ไม่ใช่ IGN_GAZEBO_*
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(desc_pkg, '..'), ':' + desc_pkg],
    )
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_MODEL_PATH',
        value=[os.path.join(desc_pkg, '..')],
    )

    declared_arguments = [
        DeclareLaunchArgument('start_rviz',   default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='false',
            description='เปิด joint_state_bridge เพื่อให้ sim follow real'),
    ]

    start_rviz   = LaunchConfiguration('start_rviz')
    start_bridge = LaunchConfiguration('start_bridge')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'hdt_arm.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    # yaml สำหรับ sim namespace
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('hdt_arm_description'),
        'config', 'ros2_controllers_sim.yaml',
    ])

    # Gazebo Harmonic ใช้ gz_args ไม่ใช่ ign_args
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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='sim',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'sim/',
        }],
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name',   'hdt_arm',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-allow_renaming', 'true',
        ],
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ส่ง --param-file เพื่อให้ controller_manager รู้จัก type ของ controller
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
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'hdt_arm.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    joint_state_bridge = Node(
        package='hdt_arm_teleop',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen',
        condition=IfCondition(start_bridge),
    )

    on_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    on_jsb_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    on_arm_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[rviz_node, joint_state_bridge],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        gz_model_path,
        *declared_arguments,
        gazebo,
        robot_state_publisher,
        gz_spawn,
        bridge_clock,
        on_spawn_done,
        on_jsb_done,
        on_arm_done,
    ])