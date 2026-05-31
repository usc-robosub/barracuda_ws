import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch_ros.actions import Node


def generate_launch_description():
    logger = get_logger("barracuda_camera.launch")

    try:
        zed_wrapper_share_dir = get_package_share_directory("zed_wrapper")
    except PackageNotFoundError:
        logger.warning("zed_wrapper is not installed; skipping ZED camera launch.")
        return LaunchDescription([])

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

    # Connect ZED wrapper frame names to the robot TF tree.
    # barracuda_description (robot_state_publisher) is expected to be launched
    # by another bringup, not by this camera launch file.
    graft_zed_to_urdf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0",
                   "barracuda/zedm_base_link", "barracuda_camera_link"],
        output="screen",
    )

    return LaunchDescription([zed_camera_launch, graft_zed_to_urdf])
