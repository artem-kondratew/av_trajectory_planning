#!/usr/bin/env python3

import os
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RecorderNode(Node):
    def __init__(self):
        super().__init__('image_recorder_node')

        parameters = [
            ('image_topic', rclpy.Parameter.Type.STRING),
            ('record_dir', rclpy.Parameter.Type.STRING),
        ]

        self.declare_parameters(namespace='', parameters=parameters)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.record_dir = self.get_parameter('record_dir').get_parameter_value().string_value

        self.get_logger().info(f'image_topic: {self.image_topic}')
        self.get_logger().info(f'record_dir: {self.record_dir}')

        os.makedirs(self.record_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=200)
        self.shutdown_flag = False

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.worker = threading.Thread(
            target=self.writer_loop,
            daemon=True
        )
        self.worker.start()

    def image_callback(self, msg: Image):
        try:
            stamp = (
                msg.header.stamp.sec +
                msg.header.stamp.nanosec * 1e-9
            )
            self.image_queue.put_nowait((stamp, msg))
        except queue.Full:
            self.get_logger().warning('Image queue full, dropping frame')

    def writer_loop(self):
        while not self.shutdown_flag or not self.image_queue.empty():
            try:
                stamp, msg = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                cv_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8'
                )

                ts = f'{stamp:.6f}'.replace('.', '_')
                filename = os.path.join(
                    self.record_dir, f't_{ts}.png'
                )

                cv2.imwrite(filename, cv_img)

            except Exception as e:
                self.get_logger().error(
                    f'Failed to write image: {e}'
                )

            self.image_queue.task_done()

    # ------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Stopping image recorder')
        self.shutdown_flag = True
        self.worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCancelled by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
