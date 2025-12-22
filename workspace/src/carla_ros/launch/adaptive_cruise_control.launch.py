import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


package_name = 'carla_ros'

world_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_world.yaml')]
host_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_host.yaml')]
dummy_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_dummy.yaml')]
rviz2_config = os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')


def generate_launch_description():
    
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info'
    )

    json_host = DeclareLaunchArgument(
        'json_host',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'stack_host.json'),
    )

    json_dummy = DeclareLaunchArgument(
        'json_dummy',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'stack_dummy.json'),
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
        parameters=[host_params, {'json_path': LaunchConfiguration('json_host')}],
        output='screen',
    )

    dummy_node = Node(
        package=package_name,
        executable='vehicle_node',
        name='carla_dummy_node',
        parameters=[dummy_params, {'json_path': LaunchConfiguration('json_dummy')}],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_config],
        output='screen',
    )

    manual_ackermann_control_host = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('manual_ackermann_control'), 'launch', 'manual_control.launch.py')]
        ),
        launch_arguments={
            'input_topic': '/carla/hero/cmd_vel',
            'output_topic': '/carla/hero/vehicle_control_cmd',
        }.items()
    )

    manual_ackermann_control_dummy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('manual_ackermann_control'), 'launch', 'manual_control.launch.py')]
        ),
        launch_arguments={
            'input_topic': '/carla/dummy/cmd_vel',
            'output_topic': '/carla/dummy/vehicle_control_cmd',
        }.items()
    )

    cc_node_dummy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('adas'), 'launch', 'cruise_control.launch.py')]
        ),
        launch_arguments=
            {
                'log_level': LaunchConfiguration('log_level'),
                'params_file': os.path.join(get_package_share_directory('adas'), 'config', 'params_cc_dummy.yaml'),
                'cc_node_name': 'cc_node_dummy',
                'host_velocity_topic': '/carla/dummy/velocity',
                'imu_topic': '/carla/dummy/imu',
                'control_topic': '/carla/dummy/vehicle_control_cmd',
                'v_ref_topic': '/carla/dummy/v_ref',
                'y_vector_topic': '/carla/dummy/y_vector',
            }.items()
    )

    # acc_nodes = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(get_package_share_directory('adas'), 'launch', 'adaptive_cruise_control.launch.py')]
    #     ),
    #     launch_arguments=
    #         {
    #             'params_file': os.path.join(get_package_share_directory('adas'), 'config', 'params_acc.yaml'),
    #             'cc_node_name': 'cc_node',
    #         }.items()
    # )

    ld = LaunchDescription()
    
    ld.add_action(log_level)
    ld.add_action(json_host)
    ld.add_action(json_dummy)
    ld.add_action(world_node)
    ld.add_action(host_node)
    ld.add_action(dummy_node)
    ld.add_action(rviz2)
    ld.add_action(manual_ackermann_control_host)
    ld.add_action(manual_ackermann_control_dummy)
    ld.add_action(cc_node_dummy)
    # ld.add_action(acc_nodes)
    
    return ld
