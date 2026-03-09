from os import getenv

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

thruster_manager_params = {
    "tam.thrusters": [
        "barracuda/thruster_0_joint",
        "barracuda/thruster_1_joint",
        "barracuda/thruster_2_joint",
        "barracuda/thruster_3_joint",
        "barracuda/thruster_4_joint",
        "barracuda/thruster_5_joint",
        "barracuda/thruster_6_joint",
        "barracuda/thruster_7_joint",
    ],
    "tam.min_thrust": -4.0,
    "tam.max_thrust": 4.0,
    "control_frame": "barracuda/base_link",
}
# create thruster_manager node
node_thruster_manager = Node(
    package="thruster_manager",
    namespace="barracuda",
    executable="thruster_manager_node",
    output="screen",
    parameters=[thruster_manager_params],
)


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
                include_launch_description("foxglove_bridge", "foxglove_bridge_launch.xml"),
                include_launch_description("barracuda_description", "rsp.launch.py"),
                include_launch_description("barracuda_control", "joystick_wrench_controller.launch.py"),
                include_launch_description("barracuda_dvl", "dvl.launch.py"),
                node_thruster_manager,
            ]
        )
    elif getenv("PLATFORM") == "pi":
        return LaunchDescription(
            [
                include_launch_description("foxglove_bridge", "foxglove_bridge_launch.xml"),
                include_launch_description("barracuda_thrusters", "barracuda_thrusters_launch.py"),
            ]
        )

    elif getenv("PLATFORM") == "laptop":
        return LaunchDescription(
            [
                include_launch_description("foxglove_bridge", "foxglove_bridge_launch.xml"),
                include_launch_description("barracuda_thrusters", "barracuda_thrusters_launch.py"),
                include_launch_description("barracuda_description", "rsp.launch.py"),
                include_launch_description("barracuda_control", "joystick_wrench_controller.launch.py"),
                include_launch_description("barracuda_dvl", "dvl.launch.py"),
                node_thruster_manager,
            ]
        )
