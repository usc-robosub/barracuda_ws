from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("barracuda_thermal_warning"),
            "config",
            "thermal_params.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="barracuda_thermal_warning",
                namespace="barracuda",
                executable="jetson_thermal_warning",
                name="jetson_thermal_warning",
                output="screen",
                emulate_tty=True,
                parameters=[config_path],
            ),
        ]
    )
