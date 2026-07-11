#!/usr/bin/env python3
"""
Record and validate ROS2 camera and IMU topics.

Usage:
    python3 validate_topics.py --camera-topic /image --imu-topic /imu/data \
        --output-dir ./recordings --duration 10
"""

import argparse
import os
import signal
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, Imu


class TopicRecorder(Node):
    def __init__(self, camera_topic: str, imu_topic: str, output_dir: str):
        super().__init__('topic_recorder')

        # Output directory with timestamp subfolder
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(output_dir, timestamp_str)
        os.makedirs(self.output_dir, exist_ok=True)

        # Recording state
        self.image_timestamps = []
        self.imu_timestamps = []
        self.img_count = 0
        self.imu_count = 0

        # QoS: best-effort for images (large data), reliable for IMU
        best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # Subscriptions
        self.img_sub = self.create_subscription(
            Image, camera_topic, self._on_image, best_effort)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, reliable)

        self.get_logger().info(
            f'Recording started. Camera: {camera_topic}, IMU: {imu_topic}')
        self.get_logger().info(f'Saving to: {self.output_dir}')

    def _on_image(self, msg: Image):
        """Decode mono8 image, save as JPEG, record timestamp."""
        self.img_count += 1
        stamp_ns = msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000
        stamp_sec = stamp_ns / 1e9
        self.image_timestamps.append(stamp_sec)

        # Decode mono8 raw bytes to numpy array
        if msg.encoding == 'mono8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
        else:
            # Fallback for other encodings — try to interpret as mono8
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
            self.get_logger().warn(
                f'Unexpected encoding {msg.encoding}, treating as mono8')

        # Save as JPEG
        filename = f'image_{self.img_count:04d}_{stamp_ns}.jpg'
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, arr)
        # cv2.imwrite expects BGR for 3-channel, but for single-channel
        # it writes grayscale JPEG correctly.

        print(f'[IMG #{self.img_count:04d}] stamp={stamp_sec:.3f} '
              f'size={msg.width}x{msg.height}')

    def _on_imu(self, msg: Imu):
        """Record IMU sample and print summary."""
        self.imu_count += 1
        stamp_ns = msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000
        stamp_sec = stamp_ns / 1e9
        self.imu_timestamps.append(stamp_sec)

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        print(f'[IMU #{self.imu_count:04d}] stamp={stamp_sec:.3f} '
              f'accel=({ax:.2f},{ay:.2f},{az:.2f}) '
              f'gyro=({gx:.2f},{gy:.2f},{gz:.2f})')

    def print_summary(self):
        """Compute and print statistics for both streams."""
        print()
        print('=' * 60)
        print('=== Image Statistics ===')
        self._print_stream_stats(
            self.image_timestamps, self.img_count, 'image')

        print()
        print('=== IMU Statistics ===')
        self._print_stream_stats(
            self.imu_timestamps, self.imu_count, 'IMU')

        print(f'\nSaved images to: {self.output_dir}')
        print('=' * 60)

    def _print_stream_stats(
            self, timestamps: list, count: int, label: str):
        """Print statistics for a single stream."""
        if count == 0:
            print(f'No {label} data received.')
            return

        total_time = timestamps[-1] - timestamps[0]
        actual_rate = (count - 1) / total_time if total_time > 0 else 0.0

        intervals = np.diff(timestamps) * 1000.0  # convert to ms

        print(f'Total frames:      {count}')
        print(f'Actual rate:       {actual_rate:.1f} Hz')
        print(f'Interval min/mean/max: '
              f'{intervals.min():.1f} / {intervals.mean():.1f} / '
              f'{intervals.max():.1f} ms')
        print(f'First stamp:       {timestamps[0]:.3f}')
        print(f'Last stamp:        {timestamps[-1]:.3f}')
        print(f'Total duration:    {total_time:.1f} s')


def main():
    parser = argparse.ArgumentParser(
        description='Record and validate ROS2 camera and IMU topics')
    parser.add_argument(
        '--camera-topic', default='/image',
        help='Camera image topic (default: /image)')
    parser.add_argument(
        '--imu-topic', default='/imu/data',
        help='IMU topic (default: /imu/data)')
    parser.add_argument(
        '--output-dir', default='./recordings',
        help='Output directory (default: ./recordings)')
    parser.add_argument(
        '--duration', type=float, default=0.0,
        help='Recording duration in seconds (0 = until Ctrl+C)')
    args = parser.parse_args()

    rclpy.init(args=sys.argv)

    recorder = TopicRecorder(args.camera_topic, args.imu_topic, args.output_dir)

    # Set up signal handlers for clean shutdown
    running = True

    def shutdown(signum=None, frame=None):
        nonlocal running
        running = False
        recorder.print_summary()
        recorder.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    start_time = time.monotonic()

    try:
        while running and rclpy.ok():
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if args.duration > 0 and (
                    time.monotonic() - start_time) >= args.duration:
                shutdown()
                break
    except KeyboardInterrupt:
        pass
    finally:
        if running:
            shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
