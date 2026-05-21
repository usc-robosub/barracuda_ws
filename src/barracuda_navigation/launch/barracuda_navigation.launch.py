#!/usr/bin/env python3
"""Barracuda Nav2 MPPI navigation launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("barracuda_navigation"), "config", "nav2_params.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=params_file,
            description="Nav2 parameters file",
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[("/cmd_vel", "/barracuda/cmd_vel")],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),

        # Goal bridge: converts /target_pose → NavigateToPose action goals
        Node(
            package="barracuda_navigation",
            executable="goal_bridge",
            name="goal_bridge",
            output="screen",
        ),

            # Hardcoded map + TF + goal publisher for testing without real sensors
            Node(
                package="barracuda_navigation",
                executable="hardcoded_nav2_publisher",
                name="hardcoded_nav2_publisher",
                output="screen",
            ),
    ])
