#!/usr/bin/env python3
"""Barracuda localization launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulated time, e.g. during rosbag replay.",
        ),
        Node(
            package="barracuda_localization",
            executable="zed_localization",
            name="zed_localization",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
