from os import getenv

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch_description(package, launchfile):
    return IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare(package),
                "launch",
                launchfile,
            ]
        )
    )


def generate_launch_description():
    if getenv("PLATFORM") == "jetson":
        return LaunchDescription(
            [
                include_launch_description(
                    "foxglove_bridge", "foxglove_bridge_launch.xml"
                )
            ]
        )
    elif getenv("PLATFORM") == "pi":
        return LaunchDescription(
            [
                include_launch_description(
                    "foxglove_bridge", "foxglove_bridge_launch.xml"
                ),
                include_launch_description("barracuda_thrusters", "barracuda_thrusters_launch.py")
            ]
        )
