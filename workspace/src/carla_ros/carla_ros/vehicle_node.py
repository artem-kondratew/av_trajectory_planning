#!/usr/bin/env python

import os
import json
import numpy as np

import carla
import rclpy

from scipy.signal import savgol_filter

from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl as Control
from carla_msgs.msg import PPState
from carla_msgs.msg import StanleyState
from carla_msgs.msg import LateralMpcState
from geometry_msgs.msg import Vector3, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

from .submodules.transform_3d import Transform3D


MAGENTA = (1.0, 0.0, 1.0)
GREEN = (0.0, 1.0, 0.0)
BLUE = (0.0, 0.0, 1.0)
RED = (1.0, 0.0, 0.0)


class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')

        parameters = [
            ('callback_period', rclpy.Parameter.Type.DOUBLE),
            ('host', rclpy.Parameter.Type.STRING),
            ('port', rclpy.Parameter.Type.INTEGER),
            ('json_path', rclpy.Parameter.Type.STRING),
            ('id', rclpy.Parameter.Type.STRING),
            ('color', rclpy.Parameter.Type.STRING),
            ('spawn_point', rclpy.Parameter.Type.INTEGER),
            ('x_offset', rclpy.Parameter.Type.DOUBLE),
            ('y_offset', rclpy.Parameter.Type.DOUBLE),
            ('yaw', 0.0),
            ('use_yaw', False),
            ('create_markers', rclpy.Parameter.Type.BOOL),
            ('generate_imu_pub', rclpy.Parameter.Type.BOOL),
            ('generate_control_sub', rclpy.Parameter.Type.BOOL),
            ('fix_y', rclpy.Parameter.Type.BOOL),
            ('lookahead_dist', 8.0),
            ('p', 10),
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
        yaw = self.get_parameter('yaw').get_parameter_value().double_value
        use_yaw = self.get_parameter('use_yaw').get_parameter_value().bool_value
        yaw = yaw if use_yaw else None
        self.create_markers = self.get_parameter('create_markers').get_parameter_value().bool_value
        self.generate_imu_pub = self.get_parameter('generate_imu_pub').get_parameter_value().bool_value
        self.generate_control_sub = self.get_parameter('generate_control_sub').get_parameter_value().bool_value
        self.fix_y = self.get_parameter('fix_y').get_parameter_value().bool_value
        self.lookahead_dist = self.get_parameter('lookahead_dist').get_parameter_value().double_value
        self.p = self.get_parameter('p').get_parameter_value().integer_value

        self.get_logger().info(f'callback_period: {callback_period}')
        self.get_logger().info(f'host: {host}')
        self.get_logger().info(f'port: {port}')
        self.get_logger().info(f'json_path: {json_path}')
        self.get_logger().info(f'id: {self.id}')
        self.get_logger().info(f'color: {color}')
        self.get_logger().info(f'spawn_point: {spawn_point}')
        self.get_logger().info(f'x_offset: {x_offset}')
        self.get_logger().info(f'y_offset: {y_offset}')
        self.get_logger().info(f'yaw: {yaw}')
        self.get_logger().info(f'use_yaw: {use_yaw}')
        self.get_logger().info(f'create_markers: {self.create_markers}')
        self.get_logger().info(f'generate_imu_pub: {self.generate_imu_pub}')
        self.get_logger().info(f'generate_control_sub: {self.generate_control_sub}')
        self.get_logger().info(f'fix_y: {self.fix_y}')
        self.get_logger().info(f'lookahead_dist: {self.lookahead_dist}')
        self.get_logger().info(f'p: {self.p}')

        client = carla.Client(host, port)
        client.set_timeout(10.0)

        self.world = client.get_world()
        self.map = self.world.get_map()

        with open(json_path) as f:
            config = json.load(f)
            self.vehicle = self.setup_vehicle(self.world, config, self.id, color, spawn_point, x_offset, y_offset, yaw)
            self.sensors = self.setup_sensors(self.world, self.vehicle, config.get('sensors', []))

        self.create_timer(callback_period, self.callback)

        self.position_pub = self.create_publisher(Vector3, f'carla/{self.id}/position', 10)
        self.position_stamped_pub = self.create_publisher(Vector3Stamped, f'carla/{self.id}/position_stamped', 10)
        self.velocity_pub = self.create_publisher(Float64, f'carla/{self.id}/velocity', 10)
        self.velocity_stamped_pub = self.create_publisher(Vector3Stamped, f'carla/{self.id}/velocity_stamped', 10)
        self.yaw_pub = self.create_publisher(Vector3Stamped, f'carla/{self.id}/yaw', 10)
        self.rear_axle_lane_offset_pub = self.create_publisher(Float64, f'carla/{self.id}/rear_axle_lane_offset', 10)
        self.pp_state_pub = self.create_publisher(PPState, f'carla/{self.id}/pp_state', 10)
        self.stanley_state_pub = self.create_publisher(
            StanleyState,
            f'carla/{self.id}/stanley_state',
            10,
        )
        self.lateral_mpc_state_pub = self.create_publisher(
            LateralMpcState,
            f'carla/{self.id}/lateral_mpc_state',
            10,
        )

        if self.create_markers:
            self.marker_pub = self.create_publisher(MarkerArray, f'carla/{self.id}/waypoints', 10)

        if self.generate_imu_pub:
            self.imu_pub = self.create_publisher(Imu, f'carla/{self.id}/imu', 10)

        self.world.on_tick(self.on_carla_tick)

        if self.generate_control_sub:
            self.control_sub = self.create_subscription(Control, f'/carla/{self.id}/vehicle_control_cmd', self.control_callback, 10)
        
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

    def lane_waypoints_and_lookahead(self, current_lane: carla.Waypoint):
        """Forward waypoints on current lane and PP/Stanley lookahead (falls back to current_lane)."""
        next_wps = []
        wp = current_lane
        for _ in range(100):
            wp_list = wp.next(0.2)
            if not wp_list:
                continue
            wp = wp_list[0]
            next_wps.append(wp)
        lookahead_wp = current_lane
        if not next_wps:
            return next_wps, lookahead_wp
        lookahead_wp = next_wps[-1]
        accum_dist = 0.0
        prev = current_lane.transform.location
        for w in next_wps:
            loc = w.transform.location
            dx = loc.x - prev.x
            dy = loc.y - prev.y
            ds = np.hypot(dx, dy)
            accum_dist += ds
            if accum_dist >= self.lookahead_dist:
                lookahead_wp = w
                break
            prev = loc
        return next_wps, lookahead_wp

    def callback(self):
        vehicle_location = self.vehicle.get_location()
        bbox = self.vehicle.bounding_box
        vehicle_tf = self.vehicle.get_transform()
        wheels = self.vehicle.get_physics_control().wheels

        wheels_world = [
            carla.Location(
                w.position.x / 100.0,
                w.position.y / 100.0,
                w.position.z / 100.0,
            )
            for w in wheels
        ]

        front_left  = wheels_world[0]
        front_right = wheels_world[1]
        rear_left   = wheels_world[2]
        rear_right  = wheels_world[3]

        front_axle = carla.Location(
            (front_left.x + front_right.x) / 2.0,
            (front_left.y + front_right.y) / 2.0,
            (front_left.z + front_right.z) / 2.0,
        )

        rear_axle = carla.Location(
            (rear_left.x + rear_right.x) / 2.0,
            (rear_left.y + rear_right.y) / 2.0,
            (rear_left.z + rear_right.z) / 2.0,
        )

        L = ((front_axle.x - rear_axle.x) ** 2 +
            (front_axle.y - rear_axle.y) ** 2 +
            (front_axle.z - rear_axle.z) ** 2) ** 0.5

        wheels_info = ', '.join(
            f'{i}=({w.x:.3f}, {w.y:.3f}, {w.z:.3f})'
            for i, w in enumerate(wheels_world)
        )

        # self.get_logger().info(
        #     f'wheels_pos = {wheels_info}'
        # )

        # self.get_logger().info(
        #     f'rear_axle = ({rear_axle.x:.3f}, {rear_axle.y:.3f}, {rear_axle.z:.3f})'
        # )

        # self.get_logger().info(
        #     f'wheelbase L = {L:.3f} m'
        # )

        # self.get_logger().info(
        #     f'vehicle_location = ({vehicle_location.x:.3f}, {vehicle_location.y:.3f}, {vehicle_location.z:.3f})'
        # )

        position_msg = Vector3()
        position_msg.x = vehicle_location.x
        position_msg.y = vehicle_location.y
        position_msg.z = vehicle_location.z
        self.position_pub.publish(position_msg)

        position_stamped_msg = Vector3Stamped()
        position_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        position_stamped_msg.header.frame_id = 'map'
        position_stamped_msg.vector.x = vehicle_location.x
        position_stamped_msg.vector.y = vehicle_location.y
        position_stamped_msg.vector.z = vehicle_location.z
        self.position_stamped_pub.publish(position_stamped_msg)

        velocity = self.vehicle.get_velocity()
        speed = np.sqrt(velocity.x**2 + velocity.y**2)
        velocity_msg = Float64()
        velocity_msg.data = speed
        self.velocity_pub.publish(velocity_msg)

        velocity_stamped_msg = Vector3Stamped()
        velocity_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_stamped_msg.header.frame_id = 'map'
        velocity_stamped_msg.vector.x = speed
        self.velocity_stamped_pub.publish(velocity_stamped_msg)

        yaw_msg = Vector3Stamped()
        yaw_msg.header.stamp = self.get_clock().now().to_msg()
        yaw_msg.header.frame_id = 'map'
        yaw_msg.vector.z = np.deg2rad(vehicle_tf.rotation.yaw)
        self.yaw_pub.publish(yaw_msg)

        current_lane = self.map.get_waypoint(vehicle_location, project_to_road=False)
        next_current_lane_waypoints = []
        lookahead_wp = None
        if current_lane is not None:
            next_current_lane_waypoints, lookahead_wp = self.lane_waypoints_and_lookahead(
                current_lane
            )
            yaw_vehicle = np.deg2rad(vehicle_tf.rotation.yaw)
            yaw_path = np.deg2rad(lookahead_wp.transform.rotation.yaw)
            wp_loc = lookahead_wp.transform.location
            xf = rear_axle.x + L * np.cos(yaw_vehicle)
            yf = rear_axle.y + L * np.sin(yaw_vehicle)
            dx = xf - wp_loc.x
            dy = yf - wp_loc.y
            nx = -np.sin(yaw_path)
            ny = np.cos(yaw_path)
            e = dx * nx + dy * ny
            v = float(np.sqrt(velocity.x**2 + velocity.y**2))
            stanley_msg = StanleyState()
            stanley_msg.header.stamp = self.get_clock().now().to_msg()
            stanley_msg.header.frame_id = 'map'
            stanley_msg.xr = xf
            stanley_msg.yr = yf
            stanley_msg.yaw_vehicle = yaw_vehicle
            stanley_msg.yaw_path = yaw_path
            stanley_msg.e = -e
            stanley_msg.v = v
            self.stanley_state_pub.publish(stanley_msg)

            theta = yaw_path - yaw_vehicle
            theta = float(np.arctan2(np.sin(theta), np.cos(theta)))

            control = self.vehicle.get_control()
            max_steer_angle = np.deg2rad(
                self.vehicle.get_physics_control().wheels[0].max_steer_angle
            )
            delta = float(control.steer * max_steer_angle)

            kappa_seq = []

            p = self.p
            wps = next_current_lane_waypoints

            assert len(wps) > p
            
            for i in range(len(wps) - 1):
                wp0 = wps[i]
                wp1 = wps[i + 1]

                yaw0 = np.deg2rad(wp0.transform.rotation.yaw)
                yaw1 = np.deg2rad(wp1.transform.rotation.yaw)

                loc0 = wp0.transform.location
                loc1 = wp1.transform.location

                ds = np.hypot(loc1.x - loc0.x, loc1.y - loc0.y)

                if ds > 1e-6:
                    dyaw = np.arctan2(np.sin(yaw1 - yaw0), np.cos(yaw1 - yaw0))
                    kappa = dyaw / ds
                else:
                    kappa = 0.0

                if abs(kappa) < 1e-3:
                    kappa = 0.0

                kappa_seq.append(float(kappa))


            kappa_seq = kappa_seq[:p]
            kappa_seq = savgol_filter(kappa_seq, 5, 2)

            vx = max(v, 1e-3)

            lat_msg = LateralMpcState()
            lat_msg.header.stamp = self.get_clock().now().to_msg()
            lat_msg.header.frame_id = 'map'
            lat_msg.e = -e
            lat_msg.theta = theta
            lat_msg.delta = delta
            lat_msg.vx = vx
            lat_msg.kappa_ref = list(kappa_seq)
            self.lateral_mpc_state_pub.publish(lat_msg)

            # self.get_logger().info(
            #     f'Stanley: e={e:.3f}, psi={(yaw_vehicle - yaw_path):.3f}, v={v:.3f}'
            # )

        if not self.create_markers:
            return

        marker_array = MarkerArray()
        marker_array.markers.append(self.create_bbox_marker_world(0, bbox, vehicle_tf, RED))
        idx = 1

        if current_lane is None:
            self.marker_pub.publish(marker_array)
            return

        rear_axle_local = self.coords_in_vehicle_frame(self.location2ndarray(rear_axle))
        lane_local = self.waypoint_in_vehicle_frame(current_lane)
        rear_axle_local[1, 0] = -rear_axle_local[1, 0]  # CARLA (LHS) → ROS (RHS)
        lane_local[1, 0] = -lane_local[1, 0]  # CARLA (LHS) → ROS (RHS)
        offset_msg = Float64()
        offset_msg.data = rear_axle_local[1, 0] - lane_local[1, 0]
        self.rear_axle_lane_offset_pub.publish(offset_msg)

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

        for i, (lane, color) in enumerate(zip(lanes, colors)):
            lane_local_point = self.waypoint_in_vehicle_frame(lane)

            lane_local_point[1, 0] = -lane_local_point[1, 0]  # CARLA (LHS) → ROS (RHS)
            
            marker = self.create_marker(idx + i, lane_local_point, color)

            marker_array.markers.append(marker)

        # -------- PURE PURSUIT DATA --------
        if next_current_lane_waypoints:
            xr = rear_axle.x
            yr = rear_axle.y
            yaw = np.deg2rad(vehicle_tf.rotation.yaw)

            lookahead_loc = lookahead_wp.transform.location
            xt = lookahead_loc.x
            yt = lookahead_loc.y

            # self.get_logger().info(
            #     f'PP state: xr={xr:.3f}, yr={yr:.3f}, yaw={yaw:.3f}, '
            #     f'xt={xt:.3f}, yt={yt:.3f}, L={L:.3f}'
            # )

            pp_msg = PPState()
            pp_msg.header.stamp = self.get_clock().now().to_msg()
            pp_msg.header.frame_id = 'map'
            pp_msg.xr = xr
            pp_msg.yr = yr
            pp_msg.yaw = yaw
            pp_msg.xt = xt
            pp_msg.yt = yt
            pp_msg.wheelbase = L
            self.pp_state_pub.publish(pp_msg)

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

    def create_bbox_marker_world(self, id : int, bbox : carla.BoundingBox, vehicle_tf : carla.Transform, color, alpha=0.3) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = id
        m.type = Marker.CUBE
        m.action = Marker.ADD

        bbox_world = vehicle_tf.transform(bbox.location)
        m.pose.position.x = bbox_world.x
        m.pose.position.y = -bbox_world.y  # CARLA (LHS) → ROS (RHS)
        m.pose.position.z = bbox_world.z

        m.pose.orientation.w = 1.0

        m.scale.x = bbox.extent.x * 2.0
        m.scale.y = bbox.extent.y * 2.0
        m.scale.z = bbox.extent.z * 2.0

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = alpha

        return m
    
    def on_carla_tick(self, snapshot: carla.WorldSnapshot):
        if self.generate_imu_pub:
            self.publish_imu(snapshot)

        if self.fix_y:
            tf = self.vehicle.get_transform()
            tf.location.y = self.fixed_y
            self.vehicle.set_transform(tf)
        
    def publish_imu(self, snapshot: carla.WorldSnapshot):
        imu_msg = Imu()

        t = snapshot.timestamp.elapsed_seconds
        imu_msg.header.stamp.sec = int(t)
        imu_msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
        imu_msg.header.frame_id = 'imu'

        rot = self.vehicle.get_transform().rotation
        roll  = np.deg2rad(rot.roll)
        pitch = np.deg2rad(rot.pitch)
        yaw   = np.deg2rad(rot.yaw)

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        ang = self.vehicle.get_angular_velocity()
        imu_msg.angular_velocity.x = ang.x
        imu_msg.angular_velocity.y = -ang.y  # CARLA (LHS) → ROS (RHS)
        imu_msg.angular_velocity.z = ang.z

        acc = self.vehicle.get_acceleration()

        imu_msg.linear_acceleration.x = acc.x
        imu_msg.linear_acceleration.y = -acc.y
        imu_msg.linear_acceleration.z = acc.z + 9.81

        self.imu_pub.publish(imu_msg)


    def control_callback(self, msg: Control):
        control = carla.VehicleControl()

        control.throttle = float(msg.throttle)
        control.steer = float(msg.steer)
        control.brake = float(msg.brake)
        control.hand_brake = bool(msg.hand_brake)
        control.reverse = bool(msg.reverse)
        control.manual_gear_shift = bool(msg.manual_gear_shift)
        control.gear = int(msg.gear)

        self.vehicle.apply_control(control)
    
    def setup_vehicle(self, world : carla.World, config, id, color, spawn_point_idx, x_offset, y_offset, yaw):
        bp_library = world.get_blueprint_library()
        map = world.get_map()

        spawn_point = map.get_spawn_points()[spawn_point_idx]
        spawn_point.location.x += x_offset
        spawn_point.location.y += y_offset
        if yaw is not None:
            spawn_point.rotation.yaw = yaw
        
        if self.fix_y:
            # self.fixed_y = spawn_point.location.y
            self.fixed_y = round(spawn_point.location.y, 1)
            
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

            actor = world.spawn_actor(bp, wp, attach_to=vehicle)
            actor.enable_for_ros()

            sensors.append(actor)

        return sensors
    
    def destroy(self):
        for sensor in self.sensors:
            sensor.destroy()
        if self.vehicle:
            self.vehicle.destroy()
        self.get_logger().info('carla destroy: ok')


def main(args=None):
    rclpy.init(args=args)
    node = VehicleNode()
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
