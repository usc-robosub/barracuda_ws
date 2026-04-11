from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='barracuda_thrusters',
            namespace='barracuda',
            executable='teensy_comms',
            output='screen',
            emulate_tty=True
        ),
    ])
