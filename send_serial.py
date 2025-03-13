#!/usr/bin/env python3
# To test, use 'demo_nodes_py listener' and 'teleop_twist_keyboard'

import rclpy
import time
import struct
import rclpy.executors
from rclpy.node import Node
from serial import Serial

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SubscribeNode(Node):
    def __init__(self, serial_port: Serial):
        super().__init__('serial_read')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            10
        )
        self.serial_port = serial_port
        self.get_logger().info("Subscribed to /cmd_vel topic")
    
    def vel_callback(self, msg: Twist):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        data = struct.pack('ff', linear_vel, angular_vel)
        self.serial_port.write(data)
        self.get_logger().debug(f"Sent to ESP32: linear={linear_vel:.2f}, angular={angular_vel:.2f}")

class PublishNode(Node):
    def __init__(self, serial_port: Serial):
        super().__init__('serial_write')
        self.publisher = self.create_publisher(
            String,
            "chatter",
            10
        )
        self.serial_port = serial_port
        self.timer = self.create_timer(0.1, self.encode_callback)
        self.get_logger().info("Publishing ESP32 responses to /chatter topic")
    
    def encode_callback(self):
        if self.serial_port.in_waiting:
            serial_data = self.serial_port.readline().decode()
            msg = String()
            msg.data = serial_data
            self.publisher.publish(msg)
            self.get_logger().debug(f"Received from ESP32: {serial_data.strip()}")

def main(args=None):
    rclpy.init(args=args)

    try:
        serial_port = Serial(
            port="/dev/ttyTHS0",
            baudrate=115200,
            timeout=0.5
        )
        time.sleep(1)
        print(f"Serial port opened: {serial_port.name}")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    subscribe_node = SubscribeNode(serial_port)
    publish_node = PublishNode(serial_port)

    node_executors = rclpy.executors.MultiThreadedExecutor()
    node_executors.add_node(subscribe_node)
    node_executors.add_node(publish_node)

    try:
        print("Spinning up send_serial.py nodes")
        node_executors.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupted...")
    finally:
        print("Stopping send_serial.py")
        serial_port.close()
        subscribe_node.destroy_node()
        publish_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
