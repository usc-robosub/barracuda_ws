import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_joystick_to_pose = Node(
        package="barracuda_control",
        namespace="barracuda",
        executable="joystick_to_pose.py",
        output="screen",
        
    )

    return LaunchDescription([
        node_joystick_to_pose,
    ])