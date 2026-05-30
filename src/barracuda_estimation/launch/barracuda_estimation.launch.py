import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("barracuda_estimation")
    ekf_odom_config = os.path.join(pkg_share, "config", "ekf_odom.yaml")
    ekf_map_config = os.path.join(pkg_share, "config", "ekf_map.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                namespace="barracuda",
                name="ekf_odom",
                output="screen",
                parameters=[ekf_odom_config],
                remappings=[
                    ("odometry/filtered", "odometry/filtered/local"),
                    ("accel/filtered", "accel_map/filtered/local"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                namespace="barracuda",
                name="ekf_map",
                output="screen",
                parameters=[ekf_map_config],
                remappings=[
                    ("odometry/filtered", "odometry/filtered/global"),
                    ("accel/filtered", "accel_map/filtered/global"),
                ],
            ),
            Node(
                package="barracuda_estimation",
                executable="estimator_node",
                namespace="barracuda",
                name="estimator_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "topics.imu": "/barracuda/zed_node/imu/data",
                        "topics.camera_image": "/barracuda/zed_node/rgb/color/rect/image",
                    }
                ],
            ),
        ]
    )
