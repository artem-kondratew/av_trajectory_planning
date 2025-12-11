import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'adas'

parameters = os.path.join(get_package_share_directory(package_name), 'config', 'cruise_control_params.yaml')

def generate_launch_description():
    ld = LaunchDescription()

    # json_path = DeclareLaunchArgument(
    #     'json_path',
    #     default_value=os.path.join(get_package_share_directory(package_name), 'config', 'stack.json'),
    #     description='Path to file with vehicle/sensors setup'
    # )
    
    adas_node = Node(
        package=package_name,
        executable='adas_node',
        parameters=[parameters],
        output='screen',
    )
    
    # ld.add_action(json_path)
    ld.add_action(adas_node)
    
    return ld
