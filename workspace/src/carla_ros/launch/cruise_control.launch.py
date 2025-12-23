import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


package_name = 'carla_ros'

world_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_world.yaml')]
host_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_host.yaml')]
rviz2_config = os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')


def generate_launch_description():

    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info'
    )

    json_path = DeclareLaunchArgument(
        'json_path',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'stack_host.json'),
    )

    record_video = DeclareLaunchArgument(
        'record_video',
        default_value='false',
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/carla/hero/back_viewer/image',
    )

    record_dir_arg = DeclareLaunchArgument(
        'record_dir',
        default_value='/data/default'
    )
    
    world_node = Node(
        package=package_name,
        executable='world_node',
        parameters=[world_params],
        output='screen',
    )

    host_node = Node(
        package=package_name,
        executable='vehicle_node',
        name='carla_host_node',
        parameters=[host_params, {'json_path': LaunchConfiguration('json_path')}],
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
        launch_arguments={
            'input_topic': '/cmd_vel',
            'output_topic': '/carla/hero/vehicle_control_cmd',
        }.items()
    )

    cc_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('adas'), 'launch', 'cruise_control.launch.py')]
        ),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    recorder_node = Node(
        package=package_name,
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        parameters=[
            host_params,
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'record_dir': LaunchConfiguration('record_dir'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('record_video')),
    )

    ld = LaunchDescription()
    
    ld.add_action(log_level)
    ld.add_action(json_path)
    ld.add_action(record_video)
    ld.add_action(image_topic_arg)
    ld.add_action(record_dir_arg)
    ld.add_action(world_node)
    ld.add_action(host_node)
    ld.add_action(rviz2)
    ld.add_action(manual_ackermann_control)
    ld.add_action(cc_node)
    ld.add_action(recorder_node)
    
    return ld
