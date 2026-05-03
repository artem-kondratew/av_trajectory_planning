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
adas_package = 'adas'

world_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_world.yaml')]
host_params = [os.path.join(get_package_share_directory(package_name), 'config', 'params_host_lateral_mpc.yaml')]
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

    allow_driving_arg = DeclareLaunchArgument(
        'allow_driving',
        default_value='false'
    )

    cc_file_arg = DeclareLaunchArgument(
        'cc_file',
        default_value=os.path.join(
            get_package_share_directory(adas_package),
            'config',
            'params_cc.yaml'
        ),
    )

    lateral_mpc_file_arg = DeclareLaunchArgument(
        'lateral_mpc_file',
        default_value=os.path.join(
            get_package_share_directory(adas_package),
            'config',
            'params_lateral_mpc.yaml'
        ),
    )

    host_velocity_topic_arg = DeclareLaunchArgument(
        'host_velocity_topic',
        default_value='/carla/hero/velocity'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/carla/hero/imu'
    )

    control_in_topic_arg = DeclareLaunchArgument(
        'control_in_topic',
        default_value='/carla/hero/vehicle_control_cmd_cc'
    )

    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='/carla/hero/vehicle_control_cmd'
    )

    v_ref_topic_arg = DeclareLaunchArgument(
        'v_ref_topic',
        default_value='/carla/hero/v_ref'
    )

    y_vector_topic_arg = DeclareLaunchArgument(
        'y_vector_topic',
        default_value='/carla/hero/y_vector'
    )

    lateral_mpc_state_topic_arg = DeclareLaunchArgument(
        'lateral_mpc_state_topic',
        default_value='/carla/hero/lateral_mpc_state'
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

    cc_node = Node(
        package=adas_package,
        executable='cc_node',
        name='cc_node',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('cc_file'),
            {
                'allow_driving': False,
                'host_velocity_topic': LaunchConfiguration('host_velocity_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'control_topic': LaunchConfiguration('control_in_topic'),
                'v_ref_topic': LaunchConfiguration('v_ref_topic'),
                'y_vector_topic': LaunchConfiguration('y_vector_topic'),
            }
        ],
    )

    lateral_mpc_node = Node(
        package=adas_package,
        executable='lateral_mpc_node',
        name='lateral_mpc_node',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('lateral_mpc_file'),
            {
                'allow_driving': False,
                'lateral_mpc_state_topic': LaunchConfiguration('lateral_mpc_state_topic'),
                'control_in_topic': LaunchConfiguration('control_in_topic'),
                'control_topic': LaunchConfiguration('control_topic'),
            }
        ],
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
    ld.add_action(allow_driving_arg)
    ld.add_action(cc_file_arg)
    ld.add_action(lateral_mpc_file_arg)
    ld.add_action(host_velocity_topic_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(control_in_topic_arg)
    ld.add_action(control_topic_arg)
    ld.add_action(v_ref_topic_arg)
    ld.add_action(y_vector_topic_arg)
    ld.add_action(lateral_mpc_state_topic_arg)
    ld.add_action(world_node)
    ld.add_action(host_node)
    ld.add_action(rviz2)
    ld.add_action(cc_node)
    ld.add_action(lateral_mpc_node)
    ld.add_action(recorder_node)

    return ld
