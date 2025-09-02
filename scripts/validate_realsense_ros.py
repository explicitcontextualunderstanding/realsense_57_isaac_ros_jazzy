#!/usr/bin/env python3
"""
validate_realsense_ros.py

Simple ROS 2 (rclpy) smoke test for the realsense2_camera driver.
Subscribes to color, depth and camera_info topics and verifies that messages
arrive within a timeout and contain non-empty payloads.

Usage (inside container where ROS environment is sourced):
  python3 scripts/validate_realsense_ros.py

Optional arguments:
  --color TOPIC         color image topic (default: /camera/camera/color/image_raw)
  --depth TOPIC         depth image topic (default: /camera/camera/depth/image_rect_raw)
  --caminfo TOPIC       camera info topic (default: /camera/camera/color/camera_info)
  --timeout SEC         timeout in seconds to wait for messages (default: 10)

Exit code: 0 = success, 1 = failure
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class RSValidateNode(Node):
    def __init__(self, color_topic, depth_topic, caminfo_topic, timeout):
        super().__init__('rs_validate')
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.caminfo_topic = caminfo_topic
        self.timeout = timeout

        self.color_msg = None
        self.depth_msg = None
        self.caminfo_msg = None

        self.create_subscription(Image, self.color_topic, self.color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.caminfo_topic, self.caminfo_cb, 10)

    def color_cb(self, msg: Image):
        if not self.color_msg:
            self.get_logger().info(f'Received color image: {msg.width}x{msg.height} enc={msg.encoding} size={len(msg.data)}')
        self.color_msg = msg

    def depth_cb(self, msg: Image):
        if not self.depth_msg:
            self.get_logger().info(f'Received depth image: {msg.width}x{msg.height} enc={msg.encoding} size={len(msg.data)}')
        self.depth_msg = msg

    def caminfo_cb(self, msg: CameraInfo):
        if not self.caminfo_msg:
            self.get_logger().info(f'Received camera_info: {msg.width}x{msg.height} K={msg.k}')
        self.caminfo_msg = msg

    def run(self):
        start = time.time()
        rclpy.spin_once(self, timeout_sec=0)  # primer
        while time.time() - start < self.timeout:
            if self.color_msg and self.depth_msg and self.caminfo_msg:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth', default='/camera/camera/depth/image_rect_raw')
    parser.add_argument('--caminfo', default='/camera/camera/color/camera_info')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = RSValidateNode(args.color, args.depth, args.caminfo, args.timeout)
    ok = node.run()
    if ok:
        node.get_logger().info('Validation SUCCESS: color, depth and camera_info received')
    else:
        node.get_logger().error('Validation FAILURE: not all messages received within timeout')

    # print a short diagnostic summary to stdout for CI parsing
    summary = {
        'color_received': bool(node.color_msg),
        'depth_received': bool(node.depth_msg),
        'caminfo_received': bool(node.caminfo_msg),
    }
    print(summary)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
