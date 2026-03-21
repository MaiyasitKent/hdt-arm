"""
hdt_arm_real.launch.py
======================
โหมด 1 — use_sim_time:=false  (default)
  ควบคุม Dynamixel จริง ไม่มี Gazebo
  ros2 launch hdt_arm_bringup hdt_arm_real.launch.py

โหมด 2 — use_sim_time:=true
  ใช้ Gazebo แทน Dynamixel ทั้งหมด + แสดงผลใน Gazebo
  ros2 launch hdt_arm_bringup hdt_arm_real.launch.py use_sim_time:=true

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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='false=Dynamixel จริง | true=Gazebo แทน Dynamixel',
        ),
        DeclareLaunchArgument('port_name',           default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_mock_hardware',   default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands',default_value='false'),
        DeclareLaunchArgument('start_rviz',          default_value='true'),
        DeclareLaunchArgument('init_position',       default_value='true'),
        DeclareLaunchArgument('start_imu_teleop',    default_value='false',
            description='เปิด imu_kinematics_node หรือไม่'),
    ]

    use_sim_time         = LaunchConfiguration('use_sim_time')
    port_name            = LaunchConfiguration('port_name')
    use_mock_hardware    = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    start_rviz           = LaunchConfiguration('start_rviz')
    init_position        = LaunchConfiguration('init_position')
    start_imu_teleop     = LaunchConfiguration('start_imu_teleop')

    # ============================================================
    # URDF — process ตอน launch time ด้วย xacro.process_file
    # รองรับทั้ง mock hardware และ real hardware
    # หมายเหตุ: use_sim_time ถูก evaluate ตอน launch จริง
    #           จึงต้องใช้ LaunchConfiguration แบบ string แทน
    # ============================================================
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('hdt_arm_description'),
                'urdf', 'hdt_arm.urdf.xacro',
            ]),
            ' use_sim:=',              use_sim_time,
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

    # ============================================================
    # Gazebo Fortress (sim mode only)
    # ============================================================
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
        condition=IfCondition(use_sim_time),
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_sim_time),
    )

    # [แก้ 3] spawn ใน namespace real เพื่อให้ controller_manager
    # ขึ้นที่ /real/controller_manager เหมือน real mode
    xacro_file = os.path.join(desc_pkg, 'urdf', 'hdt_arm.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc_sim = doc.toprettyxml(indent='  ')

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
        condition=IfCondition(use_sim_time),
    )

    # ============================================================
    # Nodes
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='real',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }],
        remappings=[
            ('robot_description', '/real/robot_description'),
        ],
        output='screen',
    )

    # real mode เท่านั้น — sim mode ใช้ Gazebo plugin จัดการแทน
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='real',
        parameters=[
            controllers_yaml,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('~/robot_description', '/real/robot_description'),
        ],
        output='screen',
        condition=UnlessCondition(use_sim_time),
    )

    # [แก้ 2] แยก spawner เป็น 2 ตัวตาม mode
    # real mode: รันทันที (control_node พร้อมแล้ว)
    # sim mode:  รอ gz_spawn เสร็จก่อน (ผ่าน on_spawn_done_start_jsb)
    joint_state_broadcaster_spawner_real = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/real/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
        condition=UnlessCondition(use_sim_time),
    )

    joint_state_broadcaster_spawner_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/real/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(use_sim_time),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    home_position_node = Node(
        package='hdt_arm_teleop',
        executable='home_position',
        namespace='real',
        parameters=[
            initial_positions_yaml,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(init_position),
    )

    # [แก้ 1] ใส่ imu_kinematics_node ใน LaunchDescription ด้วย
    imu_kinematics_node = Node(
        package='hdt_arm_teleop',
        executable='imu_kinematics_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(start_imu_teleop),
    )

    # ============================================================
    # Event handlers
    # ============================================================

    # sim mode: gz_spawn เสร็จ → jsb_sim
    on_spawn_done_start_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn,
            on_exit=[joint_state_broadcaster_spawner_sim],
        )
    )

    # jsb (ทั้ง 2 mode) เสร็จ → arm_controller
    on_jsb_real_done_start_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_real,
            on_exit=[arm_controller_spawner],
        )
    )

    on_jsb_sim_done_start_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_sim,
            on_exit=[arm_controller_spawner],
        )
    )

    # arm_controller เสร็จ → rviz + home + imu
    on_arm_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[rviz_node, home_position_node, imu_kinematics_node],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        gz_model_path,
        *declared_arguments,
        # Gazebo (sim mode only)
        gazebo,
        bridge_clock,
        gz_spawn,
        # Nodes
        robot_state_publisher,
        control_node,
        # real mode: jsb รันทันที
        joint_state_broadcaster_spawner_real,
        # Event handlers
        on_spawn_done_start_jsb,      # sim: spawn → jsb
        on_jsb_real_done_start_arm,   # real: jsb → arm
        on_jsb_sim_done_start_arm,    # sim: jsb → arm
        on_arm_done,                  # arm → rviz + home + imu
    ])