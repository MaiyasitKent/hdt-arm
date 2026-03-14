from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
    generate_moveit_rviz_launch,
)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_description = get_package_share_directory('hdt_arm_description')
    pkg_description_parent = os.path.dirname(pkg_description)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_description, 'worlds', 'default.sdf')

    use_sim_time = {"use_sim_time": True}

    set_env_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_description_parent, os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    moveit_config = MoveItConfigsBuilder(
        "hdt_arm",
        package_name="hdt_arm_moveit_config"
    ).to_moveit_configs()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hdt_arm',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # -------------------------------
    # MoveIt launches
    # -------------------------------
    # demo_launch = generate_demo_launch(moveit_config)
    move_group_launch = generate_move_group_launch(moveit_config)
    rsp = generate_rsp_launch(moveit_config)
    rviz = generate_moveit_rviz_launch(moveit_config)

    # apply use_sim_time to all nodes
    for launch_set in [move_group_launch, rsp, rviz]:
        for entity in launch_set.entities:
            if hasattr(entity, "parameters"):
                entity.parameters = entity.parameters or []
                entity.parameters.append(use_sim_time)

    # -------------------------------

    return LaunchDescription(

        [
            SetParameter(name="use_sim_time", value=True),
        ]

        +
        # demo_launch.entities +
        move_group_launch.entities +
        rsp.entities +
        rviz.entities +

        [
            set_env_gz_path,

            gazebo,

            bridge,

            TimerAction(period=3.0, actions=[spawn_robot]),

            TimerAction(period=5.0, actions=[joint_state_broadcaster]),

            TimerAction(period=6.0, actions=[arm_controller]),
        ]
    )