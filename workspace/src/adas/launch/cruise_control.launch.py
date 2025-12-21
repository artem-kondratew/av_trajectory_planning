import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'adas'

params_file = os.path.join(get_package_share_directory(package_name), 'config', 'params_cc.yaml')


def generate_launch_description():

    host_velocity_topic_arg = DeclareLaunchArgument(
        'host_velocity_topic',
        default_value='/carla/hero/local_velocity'
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

    adas_node = Node(
        package=package_name,
        executable='cc_node',
        name='cruise_control_node',
        output='screen',
        parameters=[
            params_file,
            {
                'host_velocity_topic': LaunchConfiguration('host_velocity_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'control_topic': LaunchConfiguration('control_topic'),
                'v_ref_topic': LaunchConfiguration('v_ref_topic'),
                'y_vector_topic': LaunchConfiguration('y_vector_topic'),
            }
        ],
    )

    return LaunchDescription([
        host_velocity_topic_arg,
        imu_topic_arg,
        control_topic_arg,
        v_ref_topic_arg,
        y_vector_topic_arg,
        adas_node,
    ])
