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
    return LaunchDescription(
        [include_launch_description("foxglove_bridge", "foxglove_bridge_launch.xml")]
    )
