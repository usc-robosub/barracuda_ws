import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch_ros.actions import Node


def generate_launch_description():
    logger = get_logger("barracuda_camera.launch")

    description_share_dir = get_package_share_directory("barracuda_description")
    description_launch_file = os.path.join(
        description_share_dir, "launch", "barracuda_description.launch.py"
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file)
    )

    try:
        zed_wrapper_share_dir = get_package_share_directory("zed_wrapper")
    except PackageNotFoundError:
        logger.warning("zed_wrapper is not installed; skipping ZED camera launch.")
        return LaunchDescription([description_launch])

    zed_launch_file = os.path.join(zed_wrapper_share_dir, "launch", "zed_camera.launch.py")

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            "camera_model": "zedm",
            "camera_name": "barracuda",
            "enable_positional_tracking": "true",
            "publish_tf": "false",
            "publish_map_tf": "false",
        }.items(),
    )

    # Graft ZED's static camera subtree (rooted at barracuda_camera_link) onto
    # the URDF tree by making barracuda_camera_link a child of zedm_base_link.
    # This is the single connection needed — ZED publishes the rest of the
    # camera frame chain (camera_center, left/right frames) itself.
    graft_zed_to_urdf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0",
                   "barracuda/zedm_base_link", "barracuda_camera_link"],
        output="screen",
    )

    return LaunchDescription([description_launch, zed_camera_launch, graft_zed_to_urdf])
