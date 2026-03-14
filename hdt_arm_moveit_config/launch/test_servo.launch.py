from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("hdt_arm")
        .robot_description(file_path="config/hdt_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/hdt_arm.srdf")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("hdt_arm_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,                                    # ← แรกสุด!
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])