#!/usr/bin/env python

import os
import json
import numpy as np

import carla
import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from .submodules.transform_3d import Transform3D


MAGENTA = (1.0, 0.0, 1.0)
GREEN = (0.0, 1.0, 0.0)
BLUE = (0.0, 0.0, 1.0)


class CarlaRos(Node):
    def __init__(self):
        super().__init__('vehicle_node')

        parameters = [
            ('callback_period', 0.05),
            ('host', ''),
            ('port', -1),
            ('json_path', ''),
            ('id', ''),
            ('color', ''),
            ('spawn_point', 0),
            ('x_offset', 0.0),
            ('y_offset', 0.0),
            ('create_markers', True),
        ]

        self.declare_parameters(namespace='', parameters=parameters)

        callback_period = self.get_parameter('callback_period').get_parameter_value().double_value
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        json_path = self.get_parameter('json_path').get_parameter_value().string_value
        self.id = self.get_parameter('id').get_parameter_value().string_value
        color = self.get_parameter('color').get_parameter_value().string_value
        spawn_point = self.get_parameter('spawn_point').get_parameter_value().integer_value
        x_offset = self.get_parameter('x_offset').get_parameter_value().double_value
        y_offset = self.get_parameter('y_offset').get_parameter_value().double_value
        self.create_markers = self.get_parameter('create_markers').get_parameter_value().bool_value

        self.get_logger().info(f'callback_period: {callback_period}')
        self.get_logger().info(f'host: {host}')
        self.get_logger().info(f'port: {port}')
        self.get_logger().info(f'json_path: {json_path}')
        self.get_logger().info(f'id: {self.id}')
        self.get_logger().info(f'color: {color}')
        self.get_logger().info(f'spawn_point: {spawn_point}')
        self.get_logger().info(f'x_offset: {x_offset}')
        self.get_logger().info(f'y_offset: {y_offset}')
        self.get_logger().info(f'create_markers: {self.create_markers}')

        client = carla.Client(host, port)
        client.set_timeout(10.0)

        self.world = client.get_world()
        self.map = self.world.get_map()

        with open(json_path) as f:
            config = json.load(f)
            self.vehicle = self.setup_vehicle(self.world, config, self.id, color, spawn_point, x_offset, y_offset)
            self.sensors = self.setup_sensors(self.world, self.vehicle, config.get('sensors', []))

        self.create_timer(callback_period, self.callback)

        self.velocity_pub = self.create_publisher(Twist, f'carla/{self.id}/local_velocity', 10)

        if self.create_markers:
            self.marker_pub = self.create_publisher(MarkerArray, f'carla/{self.id}/waypoints', 10)
        
    def location_to_list(self, location : carla.Location) -> list:
        return [location.x, location.y, location.z]

    def transform_lane_to_vehicle(self, t : np.ndarray):
        tf = self.vehicle.get_transform()
        t_global_vehicle = Transform3D.translation(tf.location.x, tf.location.y, tf.location.z)
        yaw = np.radians(tf.rotation.yaw)
        Rz = Transform3D.Rz(yaw)

        T_global_vehicle = Transform3D(Rz, t_global_vehicle)

        return Transform3D.from_homogeneous(T_global_vehicle.inv @ t)
    
    def waypoint_in_vehicle_frame(self, lane : carla.Waypoint) -> np.ndarray:
        loc = lane.transform.location
        t = Transform3D.translation(loc.x, loc.y, loc.z, homogeneous=True)
        return self.transform_lane_to_vehicle(t)
    
    def coords_in_vehicle_frame(self, t : np.ndarray) -> np.ndarray:
        return self.transform_lane_to_vehicle(Transform3D.homogeneous(t))
    
    def location2ndarray(self, location : carla.Location) -> np.ndarray:
        return Transform3D.translation(location.x, location.y, location.z)

    def callback(self):
        vehicle_location = self.vehicle.get_location()
        current_lane = self.map.get_waypoint(vehicle_location, project_to_road=False)

        velocity = self.vehicle.get_velocity()
        velocity_msg = Twist()
        velocity_msg.linear.x = np.sqrt(velocity.x**2 + velocity.y**2)
        self.velocity_pub.publish(velocity_msg)

        if not self.create_markers:
            return

        if current_lane is None:
            return

        lanes = [current_lane]
        colors = [MAGENTA]

        left = current_lane.get_left_lane()
        right = current_lane.get_right_lane()

        # self.get_logger().info(f'current: {current_lane, current_lane.lane_id, current_lane.lane_type, self.location_to_list(current_lane.transform.location)}')

        # if left is not None:
        #     self.get_logger().info(f'left: {left, left.lane_id, left.lane_type, self.location_to_list(left.transform.location)}')

        # if right is not None:
        #     self.get_logger().info(f'right: {right, right.lane_id, right.lane_type, self.location_to_list(right.transform.location)}')

        for lane, color in zip([left, right], [GREEN, BLUE]):
            if lane is not None and lane.lane_type == carla.libcarla.LaneType.Driving:
                lanes.append(lane)
                colors.append(color)

        marker_array = MarkerArray()

        for i, (lane, color) in enumerate(zip(lanes, colors)):
            lane_local_point = self.waypoint_in_vehicle_frame(lane)

            lane_local_point[1, 0] = -lane_local_point[1, 0]
            
            marker = self.create_marker(i, lane_local_point, color)

            marker_array.markers.append(marker)

        next_current_lane_waypoints = []
        wp = current_lane
        for _ in range(100):
            wp_list = wp.next(0.2)
            if not wp_list:
                continue
            wp = wp_list[0]
            next_current_lane_waypoints.append(wp)

        offset_y = current_lane.lane_width/2
        idx = marker_array.markers[-1].id + 1

        offset_vec_l = Transform3D.translation(0, -offset_y, 0)
        offset_vec_r = Transform3D.translation(0, +offset_y, 0)

        for waypoint in next_current_lane_waypoints:
            waypoint_yaw = waypoint.transform.rotation.yaw
            Rz = Transform3D.Rz(waypoint_yaw)
            waypoint_coords = self.location2ndarray(waypoint.transform.location)

            pt0 = waypoint_coords
            pt1 = waypoint_coords# + Rz @ offset_vec_l
            pt2 = waypoint_coords# + Rz @ offset_vec_r

            pt0 = self.coords_in_vehicle_frame(pt0)
            pt1 = self.coords_in_vehicle_frame(pt1) + offset_vec_l
            pt2 = self.coords_in_vehicle_frame(pt2) + offset_vec_r
            
            pt0[1, 0] = -pt0[1, 0]
            pt1[1, 0] = -pt1[1, 0]
            pt2[1, 0] = -pt2[1, 0]

            pt0[2, 0] = 0.05
            pt1[2, 0] = 0.05
            pt2[2, 0] = 0.05

            marker_array.markers.append(self.create_marker(idx, pt0, MAGENTA, 0.1))
            idx += 1
            marker_array.markers.append(self.create_marker(idx, pt1, GREEN, 0.1))
            idx += 1
            marker_array.markers.append(self.create_marker(idx, pt2, BLUE, 0.1))
            idx += 1

        # self.get_logger().info(f'{marker_array.markers[0].pose.position}')
        # self.get_logger().info(f'{marker_array.markers[-1].pose.position}')

        self.marker_pub.publish(marker_array)

    def create_marker(self, id : int, t : np.ndarray, color, marker_size=0.3) -> Marker:
        m = Marker()
        m.header.frame_id = self.id
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = t[0, 0]
        m.pose.position.y = t[1, 0]
        m.pose.position.z = t[2, 0]

        m.pose.orientation.w = 1.0

        m.scale.x = marker_size
        m.scale.y = marker_size
        m.scale.z = marker_size

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0

        return m
    
    def setup_vehicle(self, world : carla.World, config, id, color, spawn_point_idx, x_offset, y_offset):
        bp_library = world.get_blueprint_library()
        map = world.get_map()

        spawn_point = map.get_spawn_points()[spawn_point_idx]
        spawn_point.location.x += x_offset
        spawn_point.location.y += y_offset
            
        bp = bp_library.filter(config.get('type'))[0]
        bp.set_attribute('color', color)
        bp.set_attribute('role_name', id)
        bp.set_attribute('ros_name', id)

        return  world.spawn_actor(
            bp,
            spawn_point,
            attach_to=None)

    def setup_sensors(self, world, vehicle, sensors_config):
        bp_library = world.get_blueprint_library()

        sensors = []
        for sensor in sensors_config:
            bp = bp_library.filter(sensor.get('type'))[0]
            bp.set_attribute('ros_name', sensor.get('id')) 
            bp.set_attribute('role_name', sensor.get('id')) 
            for key, value in sensor.get('attributes', {}).items():
                bp.set_attribute(str(key), str(value))

            wp = carla.Transform(
                location=carla.Location(x=sensor['spawn_point']['x'], y=-sensor['spawn_point']['y'], z=sensor['spawn_point']['z']),
                rotation=carla.Rotation(roll=sensor['spawn_point']['roll'], pitch=-sensor['spawn_point']['pitch'], yaw=-sensor['spawn_point']['yaw'])
            )

            sensors.append(
                world.spawn_actor(
                    bp,
                    wp,
                    attach_to=vehicle
                )
            )

            sensors[-1].enable_for_ros()

        return sensors
    
    def destroy(self):
        for sensor in self.sensors:
            sensor.destroy()
        if self.vehicle:
            self.vehicle.destroy()
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
