import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    config_file = os.path.join( get_package_share_directory('vectornav'),'config','vectornav.yaml' ) 
    
    # Create the launch description and populate 
    return LaunchDescription([ 
        GroupAction([
            PushRosNamespace("barracuda"),
            Node( package='vectornav',
                executable='vectornav', 
                output='screen', 
                parameters=[config_file]), 
                
            Node( package='vectornav', 
                executable='vn_sensor_msgs', 
                output='screen', 
                parameters=[config_file])
        ])
    ])