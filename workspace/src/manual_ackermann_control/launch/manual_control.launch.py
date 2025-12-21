import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'manual_ackermann_control'


def generate_launch_description():
    ld = LaunchDescription()

    input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/cmd_vel'
    )

    output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/carla/hero/vehicle_control_cmd'
    )
    
    manual_ackermann_control = Node(
        package=package_name,
        executable='manual_ackermann_control',
        output='screen',
        remappings=[
            ('input_topic', LaunchConfiguration('input_topic')),
            ('output_topic', LaunchConfiguration('output_topic')),
        ]
    )
    
    ld.add_action(input_topic)
    ld.add_action(output_topic)
    ld.add_action(manual_ackermann_control)
    
    return ld