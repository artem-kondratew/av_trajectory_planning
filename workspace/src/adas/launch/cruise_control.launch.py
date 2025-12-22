import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'adas'


def generate_launch_description():

    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info'
    )

    cc_node_name = DeclareLaunchArgument(
        'cc_node_name',
        default_value='cc_node'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'params_cc.yaml'
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

    cc_node = Node(
        package=package_name,
        executable='cc_node',
        name=LaunchConfiguration('cc_node_name'),
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'host_velocity_topic': LaunchConfiguration('host_velocity_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'control_topic': LaunchConfiguration('control_topic'),
                'v_ref_topic': LaunchConfiguration('v_ref_topic'),
                'y_vector_topic': LaunchConfiguration('y_vector_topic'),
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(log_level)
    ld.add_action(cc_node_name)
    ld.add_action(params_file_arg)
    ld.add_action(host_velocity_topic_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(control_topic_arg)
    ld.add_action(v_ref_topic_arg)
    ld.add_action(y_vector_topic_arg)
    ld.add_action(cc_node)

    return ld
