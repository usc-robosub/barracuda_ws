import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("barracuda_control"),
        "config",
        "thruster_params.yaml"
    )

    node_joystick_to_wrench = Node(
        package="barracuda_control",
        namespace="barracuda",
        executable="joystick_to_wrench.py",
        output="screen",
        parameters=[config_file]
    )

    return LaunchDescription(
        [
            node_joystick_to_wrench,
        ]
    )