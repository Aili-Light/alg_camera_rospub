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
from sensor_msgs.msg import CompressedImage, Image, Imu


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

        # IMU text file output (opened on first message)
        self.imu_file = None

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
        self._resolve_camera_subscription(camera_topic, best_effort)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, reliable)

        self.get_logger().info(
            f'Recording started. Camera: {camera_topic}, IMU: {imu_topic}')
        self.get_logger().info(f'Saving to: {self.output_dir}')

    def _resolve_camera_subscription(self, camera_topic: str, qos: QoSProfile):
        """Inspect topic type and subscribe to Image or CompressedImage."""
        max_retries = 5
        for attempt in range(max_retries):
            topics_and_types = self.get_topic_names_and_types()
            topic_map = {name: types for name, types in topics_and_types}
            if camera_topic in topic_map:
                type_list = topic_map[camera_topic]
                if 'sensor_msgs/msg/CompressedImage' in type_list:
                    self.get_logger().info(
                        f'Detected compressed image topic: {camera_topic}')
                    self.img_sub = self.create_subscription(
                        CompressedImage, camera_topic,
                        self._on_compressed_image, qos)
                    return
                elif 'sensor_msgs/msg/Image' in type_list:
                    self.get_logger().info(
                        f'Detected raw image topic: {camera_topic}')
                    self.img_sub = self.create_subscription(
                        Image, camera_topic, self._on_image, qos)
                    return
                else:
                    raise RuntimeError(
                        f'Topic {camera_topic} has unexpected type(s): {type_list}. '
                        f'Expected sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage.')
            self.get_logger().warn(
                f'Waiting for camera topic "{camera_topic}" '
                f'(attempt {attempt + 1}/{max_retries})...')
            time.sleep(1.0)
        raise RuntimeError(
            f'Camera topic "{camera_topic}" not found after {max_retries} retries')

    def _on_image(self, msg: Image):
        """Decode raw image (multi-encoding), save as JPEG, record timestamp."""
        self.img_count += 1
        stamp_ns = msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000
        stamp_sec = stamp_ns / 1e9
        self.image_timestamps.append(stamp_sec)

        # Decode raw image bytes based on encoding
        encoding = msg.encoding
        if encoding == 'mono8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
        elif encoding == 'bgr8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        elif encoding == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif encoding == 'yuv422_yuy2':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 2)
            arr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
        elif encoding == 'yuv422':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 2)
            arr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_UYVY)
        else:
            # Fallback for unrecognized encodings — try to interpret as mono8
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
            self.get_logger().warn(
                f'Unexpected encoding "{encoding}", treating as mono8')

        self._save_image(arr, stamp_ns, stamp_sec,
                         f'size={msg.width}x{msg.height}')

    def _on_compressed_image(self, msg: CompressedImage):
        """Decode compressed JPEG image, save, record timestamp."""
        self.img_count += 1
        stamp_ns = msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000
        stamp_sec = stamp_ns / 1e9
        self.image_timestamps.append(stamp_sec)

        # Decode JPEG bytes to numpy array
        jpeg_bytes = np.frombuffer(msg.data, dtype=np.uint8)
        arr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)

        if arr is None:
            self.get_logger().error(
                f'Failed to decode compressed image #{self.img_count}')
            return

        self._save_image(arr, stamp_ns, stamp_sec,
                         f'size={arr.shape[1]}x{arr.shape[0]} compressed')

    def _save_image(self, arr: np.ndarray, stamp_ns: int,
                    stamp_sec: float, label: str):
        """Write numpy array as JPEG and print status line."""
        filename = f'image_{self.img_count:04d}_{stamp_ns}.jpg'
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, arr)
        # cv2.imwrite expects BGR for 3-channel, but for single-channel
        # it writes grayscale JPEG correctly.

        print(f'[IMG #{self.img_count:04d}] stamp={stamp_sec:.3f} '
              f'{label}')

    def _on_imu(self, msg: Imu):
        """Record IMU sample, print summary, and append to text file."""
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

        # Open IMU text file on first message
        if self.imu_file is None:
            filepath = os.path.join(self.output_dir, 'imu_data.txt')
            self.imu_file = open(filepath, 'w')
            self.imu_file.write(
                '# timestamp_ns gx gy gz ax ay az\n')

        # Append one space-delimited line: ts_ns gx gy gz ax ay az
        self.imu_file.write(
            f'{stamp_ns} {gx:.4f} {gy:.4f} {gz:.4f} '
            f'{ax:.4f} {ay:.4f} {az:.4f}\n')
        self.imu_file.flush()

        print(f'[IMU #{self.imu_count:04d}] stamp={stamp_sec:.3f} '
              f'accel=({ax:.2f},{ay:.2f},{az:.2f}) '
              f'gyro=({gx:.2f},{gy:.2f},{gz:.2f})')

    def print_summary(self):
        """Compute and print statistics for both streams."""
        # Close IMU text file if open
        if self.imu_file is not None:
            self.imu_file.close()
            self.imu_file = None
            imu_path = os.path.join(self.output_dir, 'imu_data.txt')
            print(f'\nIMU data saved to: {imu_path}')

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
