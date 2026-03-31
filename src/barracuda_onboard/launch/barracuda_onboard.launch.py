import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.logging import get_logger





logger = get_logger("barracuda_onboard.launch.py")
workspace_src = "/home/ros/barracuda_ws/src"


# helper to include a given external launch file
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


# scan workspace src dir, make a list of all package names except for barracuda_onboard
pkgs = [pkg for pkg in os.listdir(workspace_src) if pkg != "barracuda_onboard"]


# if PKG_SEL env var is set, only include packages specified by PKG_SEL in pkg list
if selected_pkgs_str := os.getenv("PKG_SEL"):
    pkgs = [pkg for pkg in selected_pkgs_str.split(" ") if pkg in pkgs]

    # show a warning if the pkg list ends up being empty (this could happen if invalid packages are specified in PKG_SEL)
    if not pkgs:
        logger.warn("selected package(s) don't exist") 
     
# for packages in barracuda_ws/src, naming convention is pkg_name.launch.py, so it's easy to include all of them in the
# list below - this way we don't have to update this launch file (barracuda_onboard.launch.py) manually every time a new
# package is added to barracuda_ws/src
launch_inclusion_list = [include_launch_description(pkg, f"{pkg}.launch.py") for pkg in pkgs]


def generate_launch_description():
    return LaunchDescription(

        # always launch foxglove bridge
        [include_launch_description("foxglove_bridge", "foxglove_bridge_launch.xml")] + launch_inclusion_list
   )
