import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


package_name = 'carla_ros'

parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')]
rviz2_config = os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')


def generate_launch_description():
    ld = LaunchDescription()

    json_path = DeclareLaunchArgument(
        'json_path',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'stack.json'),
        description='Path to file with vehicle/sensors setup'
    )

    json_dummy_path = DeclareLaunchArgument(
        'json_dummy_path',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'stack_dummy.json'),
        description='Path to file with vehicle/sensors setup'
    )
    
    carla_ros = Node(
        package=package_name,
        executable='carla_ros',
        parameters=[parameters, {'json_path': LaunchConfiguration('json_path')}, {'json_dummy_path': LaunchConfiguration('json_dummy_path')}],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_config],
        output='screen',
    )

    manual_ackermann_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('manual_ackermann_control'), 'launch', 'manual_control.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    ld.add_action(json_path)
    ld.add_action(json_dummy_path)
    ld.add_action(carla_ros)
    ld.add_action(rviz2)
    ld.add_action(manual_ackermann_control)
    
    return ld
