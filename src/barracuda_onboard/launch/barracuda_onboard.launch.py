import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.logging import get_logger

logger = get_logger("barracuda_onboard.launch.py")
default_excluded_pkgs = {"barracuda_onboard"}


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


src_root = "/root/barracuda_ws/src"
pkgs = [
    pkg
    for pkg in os.listdir(src_root)
    if pkg not in default_excluded_pkgs
    and os.path.isfile(os.path.join(src_root, pkg, "package.xml"))
]

if selected_pkgs_str := os.getenv("PKG_SEL"):
    selected_pkgs = [
        pkg for pkg in selected_pkgs_str.split(" ") if pkg and pkg != "barracuda_onboard"
    ]
    pkgs = [pkg for pkg in selected_pkgs if pkg in pkgs]

    if selected_pkgs and not pkgs:
        logger.warn("selected package(s) don't exist")

launch_inclusion_list = [include_launch_description(pkg, f"{pkg}.launch.py") for pkg in pkgs]


def generate_launch_description():
    return LaunchDescription(
        [include_launch_description("foxglove_bridge", "foxglove_bridge_launch.xml")]
        + launch_inclusion_list
    )
