from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
                        "topics.point_cloud": "/barracuda/zed_node/point_cloud/cloud_registered",
                    }
                ],
            ),
        ]
    )
