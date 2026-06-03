#!/usr/bin/env python3
"""
Script to verify camera configuration and depth map factor
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import sys


class CameraVerifier(Node):
    def __init__(self):
        super().__init__('camera_verifier')
        self.bridge = CvBridge()
        self.depth_received = False
        self.color_received = False

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )

        self.get_logger().info('Waiting for camera images...')

    def depth_callback(self, msg):
        if not self.depth_received:
            self.depth_received = True

            # Convert to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Get valid depth values (non-zero)
            valid_depths = depth_image[depth_image > 0]

            if len(valid_depths) > 0:
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                mean_depth = np.mean(valid_depths)
                median_depth = np.median(valid_depths)

                self.get_logger().info('\n' + '='*70)
                self.get_logger().info('Depth Image Analysis:')
                self.get_logger().info(f'  Encoding: {msg.encoding}')
                self.get_logger().info(f'  Resolution: {msg.width} x {msg.height}')
                self.get_logger().info(f'  Valid pixels: {len(valid_depths)} / {depth_image.size}')
                self.get_logger().info(f'  Min depth: {min_depth}')
                self.get_logger().info(f'  Max depth: {max_depth}')
                self.get_logger().info(f'  Mean depth: {mean_depth:.2f}')
                self.get_logger().info(f'  Median depth: {median_depth:.2f}')
                self.get_logger().info('='*70)

                # Determine depth map factor
                if mean_depth > 100:  # Likely in millimeters
                    suggested_factor = 1000.0
                    unit = 'millimeters'
                else:  # Likely in meters
                    suggested_factor = 1.0
                    unit = 'meters'

                self.get_logger().info(f'\nDepth values appear to be in {unit}')
                self.get_logger().info(f'Suggested RGBD.DepthMapFactor: {suggested_factor}')

                if suggested_factor == 1000.0:
                    self.get_logger().info(f'Example: depth value {int(median_depth)} = {median_depth/1000.0:.3f} meters')

                self.get_logger().info('='*70 + '\n')
            else:
                self.get_logger().warn('No valid depth values found! Make sure camera can see objects.')

    def color_callback(self, msg):
        if not self.color_received:
            self.color_received = True
            self.get_logger().info(f'Color image: {msg.width}x{msg.height}, encoding: {msg.encoding}')

    def has_received_all(self):
        return self.depth_received and self.color_received


def main():
    rclpy.init()

    verifier = CameraVerifier()

    # Wait for messages
    import time
    start_time = time.time()
    timeout = 10.0

    while rclpy.ok() and not verifier.has_received_all():
        rclpy.spin_once(verifier, timeout_sec=0.1)

        if time.time() - start_time > timeout:
            verifier.get_logger().error(f'Timeout waiting for camera images after {timeout}s')
            verifier.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    verifier.get_logger().info('Verification complete!')
    verifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
