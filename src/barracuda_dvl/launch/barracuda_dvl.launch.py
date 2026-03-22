import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('barracuda_dvl'),
        'config',
        'dvl_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='barracuda_dvl',
            executable='waterlinked_dvl_ros_driver.py',
            name='dvl_publisher',
            namespace='barracuda',
            output='screen',
            parameters=[config_file]
        ),
    ])
