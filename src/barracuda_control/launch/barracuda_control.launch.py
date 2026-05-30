from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("barracuda_control"),
                        "launch",
                        "joystick_pose_controller.launch.py"
                    ]
                )
            )
        ]
    )
