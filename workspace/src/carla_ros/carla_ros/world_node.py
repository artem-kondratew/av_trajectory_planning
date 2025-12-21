#!/usr/bin/env python

import os

import carla
import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class CarlaRos(Node):
    def __init__(self):
        super().__init__('carla_world_node')

        parameters = [
            ('dt', -0.1),
            ('map_name', ''),
            ('use_custom_map', False),
            ('host', ''),
            ('port', -1),
        ]

        self.declare_parameters(namespace='', parameters=parameters)

        dt = self.get_parameter('dt').get_parameter_value().double_value
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        use_custom_map = self.get_parameter('use_custom_map').get_parameter_value().bool_value
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f'map_name: {map_name}')
        self.get_logger().info(f'use_custom_map: {use_custom_map}')
        self.get_logger().info(f'host: {host}')
        self.get_logger().info(f'port: {port}')

        client = carla.Client(host, port)
        client.set_timeout(10.0)

        if not use_custom_map:
            client.load_world(map_name)
            self.world = client.get_world()
        else:
            map_name = os.path.join(get_package_share_directory('carla_ros'), 'maps', map_name)
            with open(map_name) as f:
                xodr = f.read()

            self.world = client.generate_opendrive_world(
                xodr,
                carla.OpendriveGenerationParameters(
                    vertex_distance=2.0,
                    smooth_junctions=True
                )
            )

        self.map = self.world.get_map()

        self.original_settings = self.world.get_settings()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        self.world.apply_settings(settings)

        self.traffic_manager = client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(True)

        self.world.tick()

        self.create_timer(dt, self.world.tick)

    def destroy(self):
        if self.original_settings:
            self.world.apply_settings(self.original_settings)
        self.get_logger().info('carla destroy: ok')

def main(args=None):
    rclpy.init(args=args)
    node = CarlaRos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\ncancelled by user. bye!')
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
