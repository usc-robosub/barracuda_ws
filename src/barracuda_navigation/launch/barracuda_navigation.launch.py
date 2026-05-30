#!/usr/bin/env python3
"""Barracuda Nav2 MPPI navigation launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulated time, e.g. during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "use_hardcoded_test_data",
            default_value="true",
            description="Launch the hardcoded fake map/pose/goal publisher for testing.",
        ),
        DeclareLaunchArgument(
            "lifecycle_manager_delay_sec",
            default_value="0.0",
            description="Delay starting Nav2 lifecycle manager to let inputs come online.",
        ),
        DeclareLaunchArgument(
            "use_lifecycle_manager",
            default_value="true",
            description="Use nav2_lifecycle_manager for bringup; disable for manual replay activation.",
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[("/cmd_vel", "/barracuda/cmd_vel")],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),

        TimerAction(
            period=LaunchConfiguration("lifecycle_manager_delay_sec"),
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[
                        LaunchConfiguration("params_file"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    ],
                    condition=IfCondition(LaunchConfiguration("use_lifecycle_manager")),
                )
            ],
        ),

        Node(
            package="barracuda_navigation",
            executable="manual_nav_lifecycle_bringup",
            name="manual_nav_lifecycle_bringup",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {
                    "node_names": [
                        "planner_server",
                        "controller_server",
                        "behavior_server",
                        "bt_navigator",
                    ]
                },
            ],
            condition=UnlessCondition(LaunchConfiguration("use_lifecycle_manager")),
        ),

        # Goal bridge: converts /target_pose → NavigateToPose action goals
        Node(
            package="barracuda_navigation",
            executable="goal_bridge",
            name="goal_bridge",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),

        Node(
            package="barracuda_navigation",
            executable="replay_tf_bridge",
            name="replay_tf_bridge",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=UnlessCondition(LaunchConfiguration("use_hardcoded_test_data")),
        ),

        # Hardcoded map + TF + goal publisher for testing without real sensors.
        Node(
            package="barracuda_navigation",
            executable="hardcoded_nav2_publisher",
            name="hardcoded_nav2_publisher",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("use_hardcoded_test_data")),
        ),
    ])
