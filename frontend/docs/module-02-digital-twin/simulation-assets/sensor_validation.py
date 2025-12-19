#!/usr/bin/env python3
"""
Sensor Validation Script for Module 2: The Digital Twin

This script validates that simulated sensors are publishing data with realistic values
and appropriate data formats in the Gazebo simulation environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Header
import numpy as np
import math

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # LiDAR validation parameters
        self.lidar_valid = False
        self.lidar_data_received = False
        self.lidar_ranges_valid = False

        # Camera validation parameters
        self.camera_valid = False
        self.camera_data_received = False
        self.camera_resolution_valid = False

        # IMU validation parameters
        self.imu_valid = False
        self.imu_data_received = False
        self.imu_orientation_valid = False

        # Set up subscribers
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for validation
        self.timer = self.create_timer(1.0, self.validate_sensors)

        self.get_logger().info('Sensor Validator Node Started')

    def lidar_callback(self, msg):
        """Callback function for LiDAR sensor data"""
        self.lidar_data_received = True

        # Validate range values
        if len(msg.ranges) > 0:
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max and not math.isnan(r)]
            if len(valid_ranges) > len(msg.ranges) * 0.5:  # At least 50% valid ranges
                self.lidar_ranges_valid = True

        # Validate timestamp
        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            self.lidar_timestamp_valid = True
        else:
            self.lidar_timestamp_valid = False

    def camera_callback(self, msg):
        """Callback function for camera sensor data"""
        self.camera_data_received = True

        # Validate image resolution
        if msg.width == 640 and msg.height == 480:
            self.camera_resolution_valid = True
        elif msg.width >= 320 and msg.height >= 240:  # Minimum acceptable resolution
            self.camera_resolution_valid = True
        else:
            self.camera_resolution_valid = False

        # Validate image format
        if msg.encoding in ['rgb8', 'bgr8', 'mono8']:
            self.camera_format_valid = True
        else:
            self.camera_format_valid = False

    def imu_callback(self, msg):
        """Callback function for IMU sensor data"""
        self.imu_data_received = True

        # Validate orientation quaternion (should be normalized)
        norm = math.sqrt(
            msg.orientation.x**2 +
            msg.orientation.y**2 +
            msg.orientation.z**2 +
            msg.orientation.w**2
        )

        if 0.99 < norm < 1.01:  # Quaternion should be normalized
            self.imu_orientation_valid = True
        else:
            self.imu_orientation_valid = False

        # Validate that we have reasonable values
        if (-1.0 <= msg.orientation.x <= 1.0 and
            -1.0 <= msg.orientation.y <= 1.0 and
            -1.0 <= msg.orientation.z <= 1.0 and
            -1.0 <= msg.orientation.w <= 1.0):
            self.imu_orientation_reasonable = True
        else:
            self.imu_orientation_reasonable = False

    def validate_sensors(self):
        """Periodic validation of all sensors"""
        self.get_logger().info('--- Sensor Validation Report ---')

        # LiDAR validation
        lidar_all_valid = all([
            self.lidar_data_received,
            self.lidar_ranges_valid,
            hasattr(self, 'lidar_timestamp_valid') and self.lidar_timestamp_valid
        ])

        self.get_logger().info(f'LiDAR Sensor: {"VALID" if lidar_all_valid else "INVALID"}')
        if self.lidar_data_received:
            self.get_logger().info('  - Data received: YES')
        else:
            self.get_logger().info('  - Data received: NO')

        if self.lidar_ranges_valid:
            self.get_logger().info('  - Range values valid: YES')
        else:
            self.get_logger().info('  - Range values valid: NO')

        # Camera validation
        camera_all_valid = all([
            self.camera_data_received,
            self.camera_resolution_valid,
            self.camera_format_valid
        ])

        self.get_logger().info(f'Camera Sensor: {"VALID" if camera_all_valid else "INVALID"}')
        if self.camera_data_received:
            self.get_logger().info('  - Data received: YES')
        else:
            self.get_logger().info('  - Data received: NO')

        if self.camera_resolution_valid:
            self.get_logger().info('  - Resolution valid: YES')
        else:
            self.get_logger().info('  - Resolution valid: NO')

        # IMU validation
        imu_all_valid = all([
            self.imu_data_received,
            self.imu_orientation_valid,
            self.imu_orientation_reasonable
        ])

        self.get_logger().info(f'IMU Sensor: {"VALID" if imu_all_valid else "INVALID"}')
        if self.imu_data_received:
            self.get_logger().info('  - Data received: YES')
        else:
            self.get_logger().info('  - Data received: NO')

        if self.imu_orientation_valid:
            self.get_logger().info('  - Orientation normalized: YES')
        else:
            self.get_logger().info('  - Orientation normalized: NO')

        self.get_logger().info('--- End Validation Report ---')


def main(args=None):
    rclpy.init(args=args)

    sensor_validator = SensorValidator()

    try:
        rclpy.spin(sensor_validator)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()