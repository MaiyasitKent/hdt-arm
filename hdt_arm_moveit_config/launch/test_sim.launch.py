# sim.launch.py  ──────────────────────────────────────────────────────────────
"""
HDT Arm — Gazebo Simulation Launch
====================================
Starts the full simulation stack:
  - Gazebo (gz_sim) with custom world
  - MoveIt (move_group, RSP, RViz)  — use_sim_time=True
  - ROS ↔ Gazebo clock bridge
  - Robot spawner (t=3s)
  - joint_state_broadcaster (t=5s)
  - arm_controller (t=6s)

Usage:
    ros2 launch hdt_arm_moveit_config sim.launch.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))  # make common.py importable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

from test_common import (
    DESCRIPTION_PKG,
    WORLD_FILE,
    CLOCK_BRIDGE_TOPIC,
    ROBOT_NAME,
    get_moveit_config,
    get_moveit_entities,
    controller_spawner,
)

# Timing
SPAWN_DELAY_S                   = 3.0
JOINT_STATE_BROADCASTER_DELAY_S = 5.0
ARM_CONTROLLER_DELAY_S          = 6.0

CLOCK_BRIDGE_TOPIC = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"


def generate_launch_description() -> LaunchDescription:

    # ── Paths ────────────────────────────────────────────────────────────────
    pkg_description        = get_package_share_directory(DESCRIPTION_PKG)
    pkg_description_parent = os.path.dirname(pkg_description)
    pkg_ros_gz_sim         = get_package_share_directory("ros_gz_sim")
    world_file_abs         = os.path.join(pkg_description, WORLD_FILE)

    # ── MoveIt ──────────────────────────────────────────────────────────────
    moveit_config   = get_moveit_config()
    moveit_entities = get_moveit_entities(moveit_config, use_sim_time=True)

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file_abs}"}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[CLOCK_BRIDGE_TOPIC],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{ROBOT_NAME}",
        arguments=[
            "-name",  ROBOT_NAME,
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0",
        ],
        output="screen",
    )

    # ── Assemble ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                pkg_description_parent,
                os.pathsep,
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ],
        ),
        *moveit_entities,
        gz_sim,
        clock_bridge,
        TimerAction(period=SPAWN_DELAY_S,
                    actions=[spawn_robot]),
        TimerAction(period=JOINT_STATE_BROADCASTER_DELAY_S,
                    actions=[controller_spawner("joint_state_broadcaster")]),
        TimerAction(period=ARM_CONTROLLER_DELAY_S,
                    actions=[controller_spawner("arm_controller")]),
    ])