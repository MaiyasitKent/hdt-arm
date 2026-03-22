import os
import xacro

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
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _pkg(name: str) -> str:
    """Return share directory path for a package."""
    return get_package_share_directory(name)


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():

    desc_pkg = _pkg('hdt_arm_description')

    # ── Environment ─────────────────────────────────────────────────────────
    env_actions = [
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            [os.path.join(desc_pkg, '..'), ':', desc_pkg],
        ),
        SetEnvironmentVariable(
            'IGN_GAZEBO_MODEL_PATH',
            [os.path.join(desc_pkg, '..')],
        ),
    ]

    # ── Launch arguments ────────────────────────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument('port_name',            default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_mock_hardware',    default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('start_rviz',           default_value='true'),
        DeclareLaunchArgument('init_position',        default_value='true'),
        DeclareLaunchArgument('start_imu_teleop',     default_value='false'),
    ]

    cfg = {k: LaunchConfiguration(k) for k in [
        'port_name', 'use_mock_hardware', 'mock_sensor_commands',
        'start_rviz', 'init_position', 'start_imu_teleop',
    ]}

    # ── Paths (declared once) ───────────────────────────────────────────────
    desc_share       = FindPackageShare('hdt_arm_description')
    controllers_yaml = PathJoinSubstitution([desc_share, 'config', 'ros2_controllers_sim.yaml'])
    rviz_config      = PathJoinSubstitution([desc_share, 'rviz',   'hdt_arm.rviz'])
    initial_pos_yaml = PathJoinSubstitution([desc_share, 'config', 'home_position_params.yaml'])
    init_pos_file    = PathJoinSubstitution([desc_share, 'config', 'initial_positions.yaml'])
    xacro_file       = os.path.join(desc_pkg, 'urdf', 'hdt_arm.urdf.xacro')

    # ── Robot description ───────────────────────────────────────────────────
    # (1) สำหรับ robot_state_publisher + control_node  → ประกอบตอน launch
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ', xacro_file,
            ' use_sim:=true',
            ' ros_namespace:=sim',
            ' use_mock_hardware:=',    cfg['use_mock_hardware'],
            ' mock_sensor_commands:=', cfg['mock_sensor_commands'],
            ' port_name:=',            cfg['port_name'],
            ' initial_positions_file:=', init_pos_file,
        ]),
        value_type=str,
    )

    # (2) สำหรับ gz_spawn → ประกอบตอน build time (xacro.process_file)
    robot_desc_sim = xacro.process_file(
        xacro_file,
        mappings={'use_sim': 'true', 'ros_namespace': 'sim'},
    ).toprettyxml(indent='  ')

    # ── Nodes ───────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_pkg('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
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
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time':      False,
            'publish_frequency': 50.0,
        }],
        remappings=[('robot_description', '/sim/robot_description')],
        output='screen',
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc_sim,
            '-name',   'hdt_arm',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='sim',
        parameters=[controllers_yaml, {'use_sim_time': False}],
        remappings=[('~/robot_description', '/sim/robot_description')],
        output='screen',
    )

    def spawner(controller: str) -> Node:
        """Factory: สร้าง controller spawner node."""
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--controller-manager', '/sim/controller_manager',
                '--param-file', controllers_yaml,
            ],
            output='screen',
        )

    jsb_spawner = spawner('joint_state_broadcaster')
    arm_spawner = spawner('arm_controller')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(cfg['start_rviz']),
        output='screen',
    )

    home_position_node = Node(
        package='hdt_arm_teleop',
        executable='home_position',
        namespace='sim',
        parameters=[initial_pos_yaml, {'use_sim_time': False}],
        condition=IfCondition(cfg['init_position']),
        output='screen',
    )

    imu_kinematics_node = Node(
    package='hdt_arm_teleop',
    executable='imu_kinematics_node',
    parameters=[{
        'use_sim_time': True,
        'action_name': '/sim/arm_controller/follow_joint_trajectory', 
    }],
    condition=IfCondition(cfg['start_imu_teleop']),
    output='screen',
)

    # ── Startup sequence ────────────────────────────────────────────────────
    # gz_spawn done → jsb → arm_controller → rviz / home / imu
    event_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=gz_spawn,
            on_exit=[jsb_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner,
            on_exit=[rviz_node, home_position_node, imu_kinematics_node],
        )),
    ]

    return LaunchDescription([
        *env_actions,
        *declared_arguments,
        gazebo,
        robot_state_publisher,
        control_node,
        gz_spawn,
        *event_handlers,
    ])