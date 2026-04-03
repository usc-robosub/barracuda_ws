import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger


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
        }.items(),
    )

    return LaunchDescription([zed_camera_launch])
