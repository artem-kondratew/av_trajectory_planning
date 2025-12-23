#!/usr/bin/env python

import os
import xml.etree.ElementTree as ET
import numpy as np

import carla
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class WorldNode(Node):
    def __init__(self):
        super().__init__('carla_world_node')

        parameters = [
            ('dt', rclpy.Parameter.Type.DOUBLE),
            ('map_name', rclpy.Parameter.Type.STRING),
            ('use_custom_map', rclpy.Parameter.Type.BOOL),
            ('host', rclpy.Parameter.Type.STRING),
            ('port', rclpy.Parameter.Type.INTEGER),
            ('spawn_trees', rclpy.Parameter.Type.BOOL),
            ('enable_lane_markings', rclpy.Parameter.Type.BOOL),
        ]

        self.declare_parameters(namespace='', parameters=parameters)

        self.dt = self.get_parameter('dt').value
        self.map_name = self.get_parameter('map_name').value
        self.use_custom_map = self.get_parameter('use_custom_map').value
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.spawn_trees = self.get_parameter('spawn_trees').value
        self.enable_lane_markings = self.get_parameter('enable_lane_markings').value

        self.lanes_drawn = False

        client = carla.Client(self.host, self.port)
        client.set_timeout(10.0)

        self.xodr_path = None

        if not self.use_custom_map:
            client.load_world(self.map_name)
            self.world = client.get_world()
        else:
            self.xodr_path = os.path.join(
                get_package_share_directory('carla_ros'),
                'maps',
                self.map_name
            )

            with open(self.xodr_path) as f:
                xodr = f.read()

            self.world = client.generate_opendrive_world(
                xodr,
                carla.OpendriveGenerationParameters(
                    vertex_distance=2.0,
                    smooth_junctions=True
                )
            )
        
        weather = carla.WeatherParameters(
            sun_altitude_angle=30.0,
            sun_azimuth_angle=0.0,
            cloudiness=50.0,
            precipitation=0.0,
            fog_density=0.0
        )
        self.world.set_weather(weather)

        self.map = self.world.get_map()

        self.original_settings = self.world.get_settings()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        self.world.apply_settings(settings)

        self.traffic_manager = client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(True)

        self.world.tick()

        if self.spawn_trees and self.xodr_path:
            self.spawn_trees_from_xodr(self.xodr_path)

        if self.enable_lane_markings:
            self.draw_lane_markings_once()

        self.create_timer(self.dt, self.world.tick)

    def spawn_trees_from_xodr(self, xodr_path):
        tree = ET.parse(xodr_path)
        root = tree.getroot()

        bp_lib = self.world.get_blueprint_library()
        tree_bps = bp_lib.filter('static.prop.*tree*')
        if not tree_bps:
            self.get_logger().warn('No tree blueprints found')
            return

        tree_bp = tree_bps[0]
        spawned = 0

        for road in root.findall('road'):
            road_id = int(road.attrib['id'])
            road_length = float(road.attrib['length'])

            objects = road.find('objects')
            if objects is None:
                continue

            for obj in objects.findall('object'):
                if obj.attrib.get('type') != 'tree':
                    continue

                s0 = float(obj.attrib['s'])
                t = float(obj.attrib['t'])
                repeat = obj.find('repeat')
                if repeat is None:
                    continue

                distance = float(repeat.attrib['distance'])

                for s in np.arange(s0, road_length, distance):
                    wp = self.map.get_waypoint_xodr(road_id, 0, s)
                    if wp is None:
                        continue

                    loc = wp.transform.transform(carla.Location(y=t))
                    tr = carla.Transform(loc, wp.transform.rotation)

                    if self.world.try_spawn_actor(tree_bp, tr):
                        spawned += 1

        self.get_logger().info(f'Spawned trees: {spawned}')

    def draw_lane_markings_once(self):
        if self.lanes_drawn:
            return

        debug = self.world.debug
        c = 5
        color = carla.Color(c, c, c)
        thickness = 0.06
        z = 0.04
        step = 1.0

        dash_on = 1
        dash_off = 4
        dash_period = dash_on + dash_off

        for wp in self.map.generate_waypoints(step):
            if wp.lane_type != carla.LaneType.Driving:
                continue

            if wp.lane_id != 1:
                continue

            nxt = wp.next(step)
            if not nxt:
                continue

            wp2 = nxt[0]

            i = int(wp.s / step)
            if (i % dash_period) >= dash_on:
                continue

            offset = wp.lane_width * 0.5

            p1 = wp.transform.transform(carla.Location(y=-offset, z=z))
            p2 = wp2.transform.transform(carla.Location(y=-offset, z=z))

            debug.draw_line(
                p1, p2,
                thickness=thickness,
                color=color,
                life_time=0.0,
                persistent_lines=True
            )

        self.lanes_drawn = True

    def destroy(self):
        if self.original_settings:
            self.world.apply_settings(self.original_settings)
        self.get_logger().info('carla destroy: ok')


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\ncancelled by user. bye!')
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()
