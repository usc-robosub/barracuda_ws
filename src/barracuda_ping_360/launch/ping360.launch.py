#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Ping360 sonar node with parameters."""

    # Declare arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description='Serial device port'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )
    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='barracuda',
        description='Node namespace'
    )

    # Ping360 node
    ping360_node = Node(
        package='barracuda_ping_360',
        executable='ping360_node',
        namespace=LaunchConfiguration('namespace'),
        name='ping360_node',
        output='screen',
        parameters=[
            {
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'debug': False,
                'range_max': 10,
                'angle_step': 1,
                'gain': 0,
                'frequency': 740,
                'speed_of_sound': 1500,
                'frame': 'sonar_frame',
                'publish_image': True,
                'publish_scan': True,
                'publish_echo': False,
                'fallback_emulated': False,
                'angle_sector': 360,
            }
        ]
    )

    return LaunchDescription([
        device_arg,
        baudrate_arg,
        namespace,
        ping360_node,
    ])

