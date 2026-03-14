# common.py  ──────────────────────────────────────────────────────────────────
"""
Shared constants, helpers, and MoveIt configuration
used by both sim.launch.py and real_world.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)

CLOCK_BRIDGE_TOPIC = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
# ===========================================================================
# Identity
# ===========================================================================
ROBOT_NAME      = "hdt_arm"
MOVEIT_PKG      = "hdt_arm_moveit_config"
DESCRIPTION_PKG = "hdt_arm_description"
WORLD_FILE      = os.path.join("worlds", "default.sdf")

# ===========================================================================
# Dynamixel (real hardware only)
# ===========================================================================
DYNAMIXEL_PORT = "/dev/ttyUSB0"
DYNAMIXEL_BAUD = 57600
CONTROLLERS_YAML = os.path.join(
    get_package_share_directory(MOVEIT_PKG),
    "config",
    "ros2_controllers.yaml",
)


# ===========================================================================
# Helpers
# ===========================================================================

def get_moveit_config():
    """Build and return the shared MoveIt configuration object."""
    return (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_PKG)
        .to_moveit_configs()
    )


def get_moveit_entities(moveit_config, use_sim_time: bool) -> list:
    """
    Generate move_group, RSP, and RViz launch entities,
    patched with the correct use_sim_time value.
    """
    launches = [
        generate_move_group_launch(moveit_config),
        generate_rsp_launch(moveit_config),
        generate_moveit_rviz_launch(moveit_config),
    ]
    for launch_set in launches:
        for entity in launch_set.entities:
            if hasattr(entity, "parameters"):
                entity.parameters = (entity.parameters or []) + [
                    {"use_sim_time": use_sim_time}
                ]
    return [entity for ls in launches for entity in ls.entities]


def controller_spawner(controller_name: str) -> Node:
    """Return a controller_manager spawner node for the given controller."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name],
        output="screen",
    )