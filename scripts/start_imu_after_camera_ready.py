#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

STANDARD_GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0


class CameraReadyGate(Node):
    def __init__(self) -> None:
        super().__init__('camera_ready_gate')
        self.declare_parameter('left_topic', '/image/left')
        self.declare_parameter('right_topic', '/image/right')
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('imu_executable', '')
        self.declare_parameter('imu_ld_library_path', '')
        self.declare_parameter('i2c_bus', '/dev/i2c-3')
        self.declare_parameter('imu_address', 105)
        self.declare_parameter('topic_name', '_imu/data_raw')
        self.declare_parameter('frequency', 200.0)
        self.declare_parameter('output_imu_topic', 'imu/data')

        self.left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self.imu_executable = self.get_parameter('imu_executable').get_parameter_value().string_value
        self.imu_ld_library_path = self.get_parameter('imu_ld_library_path').get_parameter_value().string_value
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().string_value
        self.imu_address = self.get_parameter('imu_address').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        output_imu_topic = self.get_parameter('output_imu_topic').get_parameter_value().string_value

        self.left_received = False
        self.right_received = False
        self.imu_process = None
        self.shutdown_requested = False
        self.exit_code = 0

        self.left_sub = self.create_subscription(
            Image, self.left_topic, self._on_left_image, qos_profile_sensor_data)
        self.right_sub = self.create_subscription(
            Image, self.right_topic, self._on_right_image, qos_profile_sensor_data)

        self._imu_pub = self.create_publisher(Imu, output_imu_topic, qos_profile_sensor_data)
        self._imu_sub = self.create_subscription(
            Imu, self.topic_name, self._on_raw_imu, qos_profile_sensor_data)

        self.get_logger().info(
            f'Waiting for first frames on {self.left_topic} and {self.right_topic} before starting IMU')

        self.deadline = time.monotonic() + self.timeout_sec
        self.timer = self.create_timer(0.2, self._check_status)

    def _on_left_image(self, _msg: Image) -> None:
        if not self.left_received:
            self.left_received = True
            self.get_logger().info(f'Received first frame on {self.left_topic}')

    def _on_right_image(self, _msg: Image) -> None:
        if not self.right_received:
            self.right_received = True
            self.get_logger().info(f'Received first frame on {self.right_topic}')

    def _check_status(self) -> None:
        if self.shutdown_requested:
            return

        if self.left_received and self.right_received:
            self.timer.cancel()
            self._start_imu()
            return

        if time.monotonic() >= self.deadline:
            self.get_logger().error(
                f'Timed out after {self.timeout_sec:.1f}s waiting for camera frames')
            self.exit_code = 1
            self.shutdown_requested = True
            rclpy.shutdown()

    def _on_raw_imu(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity.x = msg.angular_velocity.x * DEG_TO_RAD
        out.angular_velocity.y = msg.angular_velocity.y * DEG_TO_RAD
        out.angular_velocity.z = msg.angular_velocity.z * DEG_TO_RAD
        out.linear_acceleration.x = msg.linear_acceleration.x * STANDARD_GRAVITY
        out.linear_acceleration.y = msg.linear_acceleration.y * STANDARD_GRAVITY
        out.linear_acceleration.z = msg.linear_acceleration.z * STANDARD_GRAVITY
        self._imu_pub.publish(out)

    def _start_imu(self) -> None:
        if not self.imu_executable:
            self.get_logger().error('IMU executable path is empty')
            self.exit_code = 1
            self.shutdown_requested = True
            rclpy.shutdown()
            return

        cmd = [
            self.imu_executable,
            '--ros-args',
            '-p', f'i2c_bus:={self.i2c_bus}',
            '-p', f'imu_address:={self.imu_address}',
            '-p', f'topic_name:={self.topic_name}',
            '-p', f'frequency:={self.frequency}',
        ]

        env = os.environ.copy()
        if self.imu_ld_library_path:
            existing = env.get('LD_LIBRARY_PATH', '')
            env['LD_LIBRARY_PATH'] = (
                f'{self.imu_ld_library_path}:{existing}' if existing else self.imu_ld_library_path)

        self.get_logger().info('Camera ready, starting IMU process')
        self.get_logger().info(f'IMU command: {cmd}')

        try:
            self.imu_process = subprocess.Popen(cmd, env=env)
        except Exception as exc:
            self.get_logger().error(f'Failed to start IMU process: {exc}')
            self.exit_code = 1
            self.shutdown_requested = True
            rclpy.shutdown()
            return

        threading.Thread(target=self._wait_for_imu_process, daemon=True).start()

    def _wait_for_imu_process(self) -> None:
        if self.imu_process is None:
            return
        return_code = self.imu_process.wait()
        if not self.shutdown_requested:
            if return_code == 0:
                self.get_logger().info('IMU process exited normally')
            else:
                self.get_logger().error(f'IMU process exited with code {return_code}')
                self.exit_code = return_code
            self.shutdown_requested = True
            rclpy.shutdown()

    def request_shutdown(self) -> None:
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        if self.imu_process and self.imu_process.poll() is None:
            self.get_logger().info('Stopping IMU process')
            self.imu_process.terminate()
            try:
                self.imu_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('IMU process did not exit after SIGTERM; sending SIGKILL')
                self.imu_process.kill()
                self.imu_process.wait()


def main() -> int:
    rclpy.init()
    gate = CameraReadyGate()

    def _handle_signal(signum, _frame) -> None:
        gate.get_logger().info(f'Received signal {signum}, shutting down')
        gate.request_shutdown()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        rclpy.spin(gate)
    except KeyboardInterrupt:
        pass
    finally:
        gate.request_shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        gate.destroy_node()

    return gate.exit_code


if __name__ == '__main__':
    sys.exit(main())
