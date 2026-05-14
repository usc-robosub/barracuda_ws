from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='barracuda_trajectory',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[],
        ),
    ])
