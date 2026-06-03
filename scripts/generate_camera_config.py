#!/usr/bin/env python3
"""
Script to generate ORB-SLAM3 camera configuration from ROS2 camera_info topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import sys
import argparse


class CameraInfoCollector(Node):
    def __init__(self, color_topic, depth_topic):
        super().__init__('camera_info_collector')
        self.color_info = None
        self.depth_info = None

        self.color_sub = self.create_subscription(
            CameraInfo,
            color_topic,
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            CameraInfo,
            depth_topic,
            self.depth_callback,
            10
        )

        self.get_logger().info(f'Waiting for camera info on {color_topic} and {depth_topic}...')

    def color_callback(self, msg):
        if self.color_info is None:
            self.color_info = msg
            self.get_logger().info('Received color camera info')

    def depth_callback(self, msg):
        if self.depth_info is None:
            self.depth_info = msg
            self.get_logger().info('Received depth camera info')

    def has_both_info(self):
        return self.color_info is not None and self.depth_info is not None


def generate_config(color_info, depth_info, output_file, camera_name='Camera1'):
    """Generate ORB-SLAM3 configuration file from camera_info messages"""

    # Extract intrinsics from K matrix
    # K = [fx  0 cx]
    #     [ 0 fy cy]
    #     [ 0  0  1]
    fx = color_info.k[0]
    fy = color_info.k[4]
    cx = color_info.k[2]
    cy = color_info.k[5]

    # Extract distortion coefficients
    # D = [k1, k2, p1, p2, k3]
    k1 = color_info.d[0] if len(color_info.d) > 0 else 0.0
    k2 = color_info.d[1] if len(color_info.d) > 1 else 0.0
    p1 = color_info.d[2] if len(color_info.d) > 2 else 0.0
    p2 = color_info.d[3] if len(color_info.d) > 3 else 0.0
    k3 = color_info.d[4] if len(color_info.d) > 4 else 0.0

    width = color_info.width
    height = color_info.height

    # Calculate baseline for stereo (if applicable)
    # For RGBD, we use a nominal baseline
    baseline = 0.075  # 75mm nominal baseline for depth cameras

    config_content = f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV)
{camera_name}.fx: {fx:.6f}
{camera_name}.fy: {fy:.6f}
{camera_name}.cx: {cx:.6f}
{camera_name}.cy: {cy:.6f}

{camera_name}.k1: {k1:.6f}
{camera_name}.k2: {k2:.6f}
{camera_name}.p1: {p1:.6f}
{camera_name}.p2: {p2:.6f}
{camera_name}.k3: {k3:.6f}

Camera.width: {width}
Camera.height: {height}

# Camera frames per second
Camera.fps: 30

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
Stereo.ThDepth: 40.0
Stereo.b: {baseline:.5f}

# Depth map values factor
RGBD.DepthMapFactor: 1000.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
"""

    with open(output_file, 'w') as f:
        f.write(config_content)

    print(f"\n{'='*70}")
    print(f"Configuration file generated: {output_file}")
    print(f"{'='*70}")
    print(f"\nCamera Parameters:")
    print(f"  Resolution: {width} x {height}")
    print(f"  fx: {fx:.6f}")
    print(f"  fy: {fy:.6f}")
    print(f"  cx: {cx:.6f}")
    print(f"  cy: {cy:.6f}")
    print(f"\nDistortion Coefficients:")
    print(f"  k1: {k1:.6f}")
    print(f"  k2: {k2:.6f}")
    print(f"  p1: {p1:.6f}")
    print(f"  p2: {p2:.6f}")
    print(f"  k3: {k3:.6f}")
    print(f"{'='*70}\n")


def main():
    parser = argparse.ArgumentParser(description='Generate ORB-SLAM3 config from camera_info')
    parser.add_argument('--color-topic', default='/camera/color/camera_info',
                        help='Color camera info topic (default: /camera/color/camera_info)')
    parser.add_argument('--depth-topic', default='/camera/depth/camera_info',
                        help='Depth camera info topic (default: /camera/depth/camera_info)')
    parser.add_argument('--output', '-o', default='camera_config.yaml',
                        help='Output config file path (default: camera_config.yaml)')
    parser.add_argument('--camera-name', default='Camera1',
                        help='Camera parameter prefix in config (default: Camera1)')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Timeout in seconds (default: 10.0)')

    args = parser.parse_args()

    rclpy.init()

    collector = CameraInfoCollector(args.color_topic, args.depth_topic)

    # Wait for camera info messages
    import time
    start_time = time.time()

    while rclpy.ok() and not collector.has_both_info():
        rclpy.spin_once(collector, timeout_sec=0.1)

        if time.time() - start_time > args.timeout:
            collector.get_logger().error(f'Timeout waiting for camera info after {args.timeout}s')
            collector.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    if collector.has_both_info():
        generate_config(collector.color_info, collector.depth_info,
                       args.output, args.camera_name)
        collector.get_logger().info('Configuration generation complete!')

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
