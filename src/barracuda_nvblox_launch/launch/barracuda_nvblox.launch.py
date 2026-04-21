#!/usr/bin/env python3
"""Barracuda nvblox launch."""

from launch import Action, LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CONTAINER_NAME = "nvblox_container"

ZED_REMAPPINGS = [
    ("camera_0/depth/image", "/barracuda/zed_node/depth/depth_registered"),
    ("camera_0/depth/camera_info", "/barracuda/zed_node/depth/camera_info"),
    ("camera_0/color/image", "/barracuda/zed_node/rgb/color/rect/image"),
    ("camera_0/color/camera_info", "/barracuda/zed_node/rgb/color/rect/camera_info"),
    ("pose", "/barracuda/zed_node/pose"),
]


def generate_launch_description() -> LaunchDescription:
    nvblox_node = ComposableNode(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        remappings=ZED_REMAPPINGS,
        parameters=[
            {"num_cameras": 1},
            {"use_lidar": False},
            {"use_tf_transforms": False},
            {"use_topic_transforms": True},
            {
                "map_clearing_frame_id": "zed_camera_link",
                "pose_frame": "zed_camera_link",
                "esdf_slice_bounds_visualization_attachment_frame_id": "zed_camera_link",
            },
        ],
    )

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[nvblox_node],
        output="screen",
    )

    return LaunchDescription([container])
