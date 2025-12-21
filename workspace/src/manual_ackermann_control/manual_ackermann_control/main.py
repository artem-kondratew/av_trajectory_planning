#!/usr/bin/env python3

import numpy as np

import rclpy

from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import Twist


class CarlaAckermannPublisher(Node):
    def __init__(self):
        super().__init__('manual_ackermann_controller')

        self.create_subscription(Twist, 'input_topic', self.callback, 10)

        self.publisher_ = self.create_publisher(CarlaEgoVehicleControl, 'output_topic', 10)

        self.callback(Twist())

    def callback(self, msg : Twist):
        v = msg.linear.x
        w = msg.angular.z

        msg = CarlaEgoVehicleControl()

        throttle = 0.0
        brake = 0.0
        reverse = False

        if v == 0.0:
            throttle = 0.0
            brake = 1.0
        
        if v < 0:
            v = -v
            reverse = True

        throttle = np.clip(v, 0, 1)
        steer = -np.clip(w, -1, 1)

        msg.throttle = throttle
        msg.steer = steer
        msg.brake = brake
        msg.reverse = reverse

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: throttle = {throttle}, steer = {steer}, brake = {brake}, reverse = {reverse}')


def main(args=None):
    rclpy.init(args=args)
    node = CarlaAckermannPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
