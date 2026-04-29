#!/usr/bin/env python3
"""Barracuda nvblox launch."""

from launch import Action, LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CONTAINER_NAME = "nvblox_container"

ZED_REMAPPINGS = [
    ("camera_0/depth/image", "/barracuda/zed_node/depth/depth_registered"),
    ("camera_0/depth/camera_info", "/barracuda/zed_node/depth/depth_registered/camera_info"),
    ("camera_0/color/image", "/barracuda/zed_node/rgb/color/rect/image"),
    ("camera_0/color/camera_info", "/barracuda/zed_node/rgb/color/rect/image/camera_info"),
    ("pose", "/barracuda/zed_node/pose"),
    ("transform", "/barracuda/transform"),
]


def generate_launch_description() -> LaunchDescription:
    nvblox_node = ComposableNode(
        name="nvblox_node",
        namespace="barracuda",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        remappings=ZED_REMAPPINGS,
        parameters=[
            {"num_cameras": 1},
            {"use_lidar": False},
            {"use_tf_transforms": True},
            {"use_topic_transforms": False},
            {"global_frame": "map"},
            {"static_mapper.workspace_bounds_type": "kUnbounded"},
            {"dynamic_mapper.workspace_bounds_type": "kUnbounded"},
            # Explicit workspace bounds: ±10m in XY, 0-10m height
            {"static_mapper.workspace_bounds_min_corner_x_m": -10.0},
            {"static_mapper.workspace_bounds_max_corner_x_m": 10.0},
            {"static_mapper.workspace_bounds_min_corner_y_m": -10.0},
            {"static_mapper.workspace_bounds_max_corner_y_m": 10.0},
            {"static_mapper.workspace_bounds_min_height_m": 0.0},
            {"static_mapper.workspace_bounds_max_height_m": 10.0},
            {"dynamic_mapper.workspace_bounds_min_corner_x_m": -10.0},
            {"dynamic_mapper.workspace_bounds_max_corner_x_m": 10.0},
            {"dynamic_mapper.workspace_bounds_min_corner_y_m": -10.0},
            {"dynamic_mapper.workspace_bounds_max_corner_y_m": 10.0},
            {"dynamic_mapper.workspace_bounds_min_height_m": 0.0},
            {"dynamic_mapper.workspace_bounds_max_height_m": 10.0},
            {
                "map_clearing_frame_id": "map",
                "pose_frame": "barracuda_camera_center",
                "esdf_slice_bounds_visualization_attachment_frame_id": "barracuda_camera_center",
            },
        ],
    )

    pose_to_transform = Node(
        package="barracuda_nvblox",
        executable="pose_to_transform",
        namespace="barracuda",
        output="screen",
        parameters=[{"child_frame_id": "barracuda_camera_center"}],
    )

    # Static transform to alias the depth frame names
    # ZED publishes depth with frame: barracuda_left_camera_frame_optical
    # TF tree has: barracuda_left_camera_optical_frame
    static_frame_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "barracuda_left_camera_optical_frame",
            "barracuda_left_camera_frame_optical",
        ],
        output="screen",
    )

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[nvblox_node],
        output="screen",
    )

    return LaunchDescription([pose_to_transform, static_frame_alias, container])
