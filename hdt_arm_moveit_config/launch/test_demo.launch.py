#!/usr/bin/env python3
"""
HDT Arm Launch File — Simulation + Real Hardware (switchable)
=============================================================
Controls via top-level constant:
    USE_SIM = True   → Full Gazebo simulation stack
    USE_SIM = False  → Real hardware via ros2_control + Dynamixel

Delay sequence (simulation):
    t=0s → Gazebo + MoveIt stack + clock bridge
    t=3s → Spawn robot URDF into Gazebo
    t=5s → joint_state_broadcaster
    t=6s → arm_controller

Delay sequence (real hardware):
    t=0s → ros2_control_node + MoveIt stack
    t=2s → joint_state_broadcaster
    t=3s → arm_controller

To switch to real hardware:
    1. Set USE_SIM = False
    2. Uncomment all blocks marked # [REAL HARDWARE]
    3. Comment out all blocks marked   # [SIMULATION]
    4. Verify DYNAMIXEL_PORT and DYNAMIXEL_BAUD match your setup
    5. Ensure your URDF <ros2_control> uses dynamixel_hardware plugin
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    # ExecuteProcess,    # [REAL HARDWARE] uncomment when needed
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)


# ===========================================================================
# ⚙️  TOP-LEVEL CONFIGURATION — edit here only
# ===========================================================================

USE_SIM = True   # True = Gazebo | False = real Dynamixel hardware

# Robot / package identity
ROBOT_NAME      = "hdt_arm"
MOVEIT_PKG      = "hdt_arm_moveit_config"
DESCRIPTION_PKG = "hdt_arm_description"
WORLD_FILE      = os.path.join("worlds", "default.sdf")

# Timing (seconds)
SPAWN_DELAY_S                    = 3.0   # [SIMULATION]  wait for Gazebo world to load
JOINT_STATE_BROADCASTER_DELAY_S  = 5.0   # [SIMULATION]  after spawn
ARM_CONTROLLER_DELAY_S           = 6.0   # [SIMULATION]  after joint_state_broadcaster

# REAL_JOINT_STATE_BROADCASTER_DELAY_S = 2.0   # [REAL HARDWARE]
# REAL_ARM_CONTROLLER_DELAY_S          = 3.0   # [REAL HARDWARE]

# Dynamixel hardware settings                   # [REAL HARDWARE]
# DYNAMIXEL_PORT = "/dev/ttyUSB0"               # [REAL HARDWARE] check: ls /dev/ttyUSB*
# DYNAMIXEL_BAUD = 57600                        # [REAL HARDWARE] match Dynamixel Wizard setting
# CONTROLLERS_YAML = os.path.join(              # [REAL HARDWARE]
#     get_package_share_directory(MOVEIT_PKG),  # [REAL HARDWARE]
#     "config",                                 # [REAL HARDWARE]
#     "ros2_controllers.yaml",                  # [REAL HARDWARE]
# )                                             # [REAL HARDWARE]

# Gazebo bridge topic
CLOCK_BRIDGE_TOPIC = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"


# ===========================================================================
# Helpers
# ===========================================================================

def _apply_sim_time(launch_set) -> None:
    """Patch every node in a LaunchDescription to honour use_sim_time."""
    for entity in launch_set.entities:
        if hasattr(entity, "parameters"):
            entity.parameters = (entity.parameters or []) + [{"use_sim_time": USE_SIM}]


def _controller_spawner(controller_name: str) -> Node:
    """Return a controller_manager spawner node for the given controller."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name],
        output="screen",
    )


# ===========================================================================
# Launch description
# ===========================================================================

def generate_launch_description() -> LaunchDescription:

    # ── Package paths ────────────────────────────────────────────────────────
    pkg_description        = get_package_share_directory(DESCRIPTION_PKG)
    pkg_description_parent = os.path.dirname(pkg_description)
    pkg_ros_gz_sim         = get_package_share_directory("ros_gz_sim")  # [SIMULATION]
    world_file_abs         = os.path.join(pkg_description, WORLD_FILE)  # [SIMULATION]

    # ── MoveIt configuration ─────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_PKG)
        .to_moveit_configs()
    )

    # ── MoveIt sub-launches (sim-time patched) ───────────────────────────────
    moveit_launches = [
        generate_move_group_launch(moveit_config),
        generate_rsp_launch(moveit_config),
        generate_moveit_rviz_launch(moveit_config),
    ]
    for launch_set in moveit_launches:
        _apply_sim_time(launch_set)

    moveit_entities = [
        entity
        for launch_set in moveit_launches
        for entity in launch_set.entities
    ]

    # =========================================================================
    # [SIMULATION] — comment this entire block out when switching to real hw
    # =========================================================================
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
    # =========================================================================
    # [END SIMULATION]
    # =========================================================================

    # =========================================================================
    # [REAL HARDWARE] — uncomment this entire block when switching to real hw
    # =========================================================================

    # Step 1: lower USB latency (critical for Dynamixel reliability)
    # set_usb_latency = ExecuteProcess(
    #     cmd=["bash", "-c",
    #          f"echo 1 | sudo tee /sys/bus/usb-serial/devices/"
    #          f"{os.path.basename(DYNAMIXEL_PORT)}/latency_timer"],
    #     output="screen",
    # )

    # Step 2: ros2_control node — loads the Dynamixel hardware plugin
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     name="ros2_control_node",
    #     parameters=[
    #         moveit_config.robot_description,   # URDF must use dynamixel_hardware plugin
    #         CONTROLLERS_YAML,
    #         {
    #             "use_sim_time": False,
    #             # Dynamixel hardware plugin params (also settable in URDF xacro args):
    #             # "dynamixel_hardware/usb_port": DYNAMIXEL_PORT,
    #             # "dynamixel_hardware/baud_rate": DYNAMIXEL_BAUD,
    #         },
    #     ],
    #     output="screen",
    # )
    # =========================================================================
    # [END REAL HARDWARE]
    # =========================================================================

    # ── Assemble LaunchDescription ───────────────────────────────────────────
    return LaunchDescription(
        [
            # ── Global parameters ────────────────────────────────────────────
            SetParameter(name="use_sim_time", value=USE_SIM),

            # ── [SIMULATION] Gazebo resource path ────────────────────────────
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    pkg_description_parent,
                    os.pathsep,
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ],
            ),
            # # [REAL HARDWARE] Remove SetEnvironmentVariable above and
            # # uncomment USB latency + ros2_control_node instead:
            # set_usb_latency,
            # ros2_control_node,

            # ── MoveIt stack (always present) ─────────────────────────────────
            *moveit_entities,

            # ── [SIMULATION] Gazebo + clock bridge ───────────────────────────
            gz_sim,
            clock_bridge,

            # ── [SIMULATION] Timed actions ────────────────────────────────────
            TimerAction(period=SPAWN_DELAY_S,
                        actions=[spawn_robot]),
            TimerAction(period=JOINT_STATE_BROADCASTER_DELAY_S,
                        actions=[_controller_spawner("joint_state_broadcaster")]),
            TimerAction(period=ARM_CONTROLLER_DELAY_S,
                        actions=[_controller_spawner("arm_controller")]),

            # # [REAL HARDWARE] Replace the three TimerActions above with these:
            # TimerAction(period=REAL_JOINT_STATE_BROADCASTER_DELAY_S,
            #             actions=[_controller_spawner("joint_state_broadcaster")]),
            # TimerAction(period=REAL_ARM_CONTROLLER_DELAY_S,
            #             actions=[_controller_spawner("arm_controller")]),
        ]
    )