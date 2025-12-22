import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'adas'


def generate_launch_description():

    log_level = DeclareLaunchArgument(name='log_level', default_value='info')

    cc_node_name = DeclareLaunchArgument(
        'cc_node_name',
        default_value='cc_node'
    )

    cc_params_file_arg = DeclareLaunchArgument(
        'cc_params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'params_cc.yaml'
        ),
    )

    acc_params_file_arg = DeclareLaunchArgument(
        'acc_params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'params_acc.yaml'
        ),
    )

    host_position_topic_arg = DeclareLaunchArgument(
        'host_position_topic',
        default_value='/carla/hero/position'
    )

    dummy_position_topic_arg = DeclareLaunchArgument(
        'dummy_position_topic',
        default_value='/carla/dummy/position'
    )

    host_velocity_topic_arg = DeclareLaunchArgument(
        'host_velocity_topic',
        default_value='/carla/hero/velocity'
    )

    dummy_velocity_topic_arg = DeclareLaunchArgument(
        'dummy_velocity_topic',
        default_value='/carla/hero/velocity'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/carla/hero/imu'
    )

    cc_control_topic_arg = DeclareLaunchArgument(
        'cc_control_topic',
        default_value='/carla/hero/vehicle_control_cmd/cc_control'
    )

    acc_control_topic_arg = DeclareLaunchArgument(
        'acc_control_topic',
        default_value='/carla/hero/vehicle_control_cmd/acc_control'
    )

    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='/carla/hero/vehicle_control_cmd'
    )
    
    v_ref_topic_arg = DeclareLaunchArgument(
        'v_ref_topic',
        default_value='/carla/hero/v_ref'
    )

    dist_ref_topic_arg = DeclareLaunchArgument(
        'dist_ref_topic',
        default_value='/carla/hero/dist_ref'
    )

    dist_topic_arg = DeclareLaunchArgument(
        'dist_topic',
        default_value='/carla/hero/dist'
    )

    y_vector_topic_arg = DeclareLaunchArgument(
        'y_vector_topic',
        default_value='/carla/hero/y_vector'
    )

    cc_node = Node(
        package=package_name,
        executable='cc_node',
        name=LaunchConfiguration('cc_node_name'),
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('cc_params_file'),
            {
                'host_velocity_topic': LaunchConfiguration('host_velocity_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'control_topic': LaunchConfiguration('cc_control_topic'),
                'v_ref_topic': LaunchConfiguration('v_ref_topic'),
                'y_vector_topic': LaunchConfiguration('y_vector_topic'),
            }
        ],
    )

    acc_node = Node(
        package=package_name,
        executable='acc_node',
        name='acc_node',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('acc_params_file'),
            {
                'host_position_topic': LaunchConfiguration('host_position_topic'),
                'dummy_position_topic': LaunchConfiguration('dummy_position_topic'),
                'host_velocity_topic': LaunchConfiguration('host_velocity_topic'),
                'dummy_velocity_topic': LaunchConfiguration('dummy_velocity_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'control_topic': LaunchConfiguration('acc_control_topic'),
                'dist_ref_topic': LaunchConfiguration('dist_ref_topic'),
                'dist_topic': LaunchConfiguration('dist_topic'),
                'y_vector_topic': LaunchConfiguration('y_vector_topic'),
            }
        ],
    )

    control_wrapper_node = Node(
        package=package_name,
        executable='control_wrapper',
        name='vehicle_control_wrapper',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('acc_params_file'),
            {
                'cc_topic': LaunchConfiguration('cc_control_topic'),
                'acc_topic': LaunchConfiguration('acc_control_topic'),
                'control_topic': LaunchConfiguration('control_topic'),
                'dist_ref_topic': LaunchConfiguration('dist_ref_topic'),
                'dist_topic': LaunchConfiguration('dist_topic'),
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(log_level)
    ld.add_action(cc_node_name)
    ld.add_action(cc_params_file_arg)
    ld.add_action(acc_params_file_arg)
    ld.add_action(host_position_topic_arg)
    ld.add_action(dummy_position_topic_arg)
    ld.add_action(host_velocity_topic_arg)
    ld.add_action(dummy_velocity_topic_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(cc_control_topic_arg)
    ld.add_action(acc_control_topic_arg)
    ld.add_action(control_topic_arg)
    ld.add_action(v_ref_topic_arg)
    ld.add_action(dist_ref_topic_arg)
    ld.add_action(dist_topic_arg)
    ld.add_action(y_vector_topic_arg)
    ld.add_action(cc_node)
    ld.add_action(acc_node)
    ld.add_action(control_wrapper_node)

    return ld
