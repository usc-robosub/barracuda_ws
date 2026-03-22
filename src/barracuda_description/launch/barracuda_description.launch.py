# Adapted from Articulated Robotics' URDF example launch file (https://github.com/joshnewans/urdf_example/blob/main/launch/rsp.launch.py)

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory("barracuda_description"))
    xacro_file = os.path.join(pkg_path, "urdf", "barracuda.xacro")
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {
        "robot_description": robot_description_config.toxml(),
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace="barracuda",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        namespace="barracuda",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
           node_joint_state_publisher,
           node_robot_state_publisher,
        ]
    )
