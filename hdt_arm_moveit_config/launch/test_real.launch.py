# real_world.launch.py  ───────────────────────────────────────────────────────
"""
HDT Arm — Real Hardware Launch
================================
Starts the full real hardware stack:
  - USB latency tuning for Dynamixel reliability
  - ros2_control_node with Dynamixel hardware plugin
  - MoveIt (move_group, RSP, RViz)  — use_sim_time=False
  - joint_state_broadcaster (t=2s)
  - arm_controller (t=3s)

Pre-flight checklist:
  [ ] URDF <ros2_control> uses 'dynamixel_hardware' plugin (not gz_ros2_control)
  [ ] DYNAMIXEL_PORT in common.py matches your device (ls /dev/ttyUSB*)
  [ ] DYNAMIXEL_BAUD in common.py matches Dynamixel Wizard setting
  [ ] ros2_controllers.yaml exists in hdt_arm_moveit_config/config/
  [ ] User has permission to access serial port (sudo usermod -aG dialout $USER)

Usage:
    ros2 launch hdt_arm_moveit_config real_world.launch.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))  # make common.py importable
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, SetParameter

from test_common import (
    DYNAMIXEL_PORT,
    CONTROLLERS_YAML,
    get_moveit_config,
    get_moveit_entities,
    controller_spawner,
)

# Timing (shorter than sim — no Gazebo world load to wait for)
JOINT_STATE_BROADCASTER_DELAY_S = 2.0
ARM_CONTROLLER_DELAY_S          = 3.0


def generate_launch_description() -> LaunchDescription:

    # ── MoveIt ───────────────────────────────────────────────────────────────
    moveit_config   = get_moveit_config()
    moveit_entities = get_moveit_entities(moveit_config, use_sim_time=False)

    # ── USB latency (critical for Dynamixel timing) ──────────────────────────
    # Reduces USB round-trip from ~16ms → ~1ms.
    # Requires sudo — run once manually if this fails:
    #   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    set_usb_latency = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"echo 1 | sudo tee /sys/bus/usb-serial/devices/"
            f"{DYNAMIXEL_PORT.split('/')[-1]}/latency_timer",
        ],
        output="screen",
    )

    # ── ros2_control node ─────────────────────────────────────────────────────
    # Loads the Dynamixel hardware plugin defined in your URDF <ros2_control>.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        parameters=[
            moveit_config.robot_description,  # URDF with dynamixel_hardware plugin
            CONTROLLERS_YAML,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # ── Assemble ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),
        set_usb_latency,
        ros2_control_node,
        *moveit_entities,
        TimerAction(period=JOINT_STATE_BROADCASTER_DELAY_S,
                    actions=[controller_spawner("joint_state_broadcaster")]),
        TimerAction(period=ARM_CONTROLLER_DELAY_S,
                    actions=[controller_spawner("arm_controller")]),
    ])