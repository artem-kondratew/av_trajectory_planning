import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'manual_ackerman_control'

parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')]


def generate_launch_description():
    ld = LaunchDescription()
    
    publisher = Node(
        package=package_name,
        executable='carla_ackermann_publisher',
        parameters=[parameters],
        output='screen',
    )
    
    
    ld.add_action(publisher)
    
    return ld