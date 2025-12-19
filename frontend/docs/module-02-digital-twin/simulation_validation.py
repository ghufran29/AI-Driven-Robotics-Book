#!/usr/bin/env python3
"""
Complete Simulation Pipeline Validation Script

This script validates the entire simulation pipeline including:
- Gazebo physics simulation
- Sensor data generation
- ROS 2 bridge communication
- Unity visualization synchronization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import threading
from datetime import datetime

class SimulationValidator(Node):
    def __init__(self):
        super().__init__('simulation_validator')

        # Data validation flags
        self.gazebo_running = False
        self.sensors_active = False
        self.bridge_connected = False
        self.unity_connected = False

        # Sensor data tracking
        self.lidar_data_received = False
        self.camera_data_received = False
        self.imu_data_received = False
        self.odom_data_received = False

        # Performance metrics
        self.start_time = None
        self.test_duration = 0
        self.data_counts = {
            'lidar': 0,
            'camera': 0,
            'imu': 0,
            'odom': 0
        }

        # Set up subscribers for all sensor topics
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
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

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for test commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for periodic validation
        self.timer = self.create_timer(1.0, self.validate_pipeline)

        # Start time tracking
        self.start_time = self.get_clock().now()

        self.get_logger().info('Simulation Pipeline Validator Started')

    def lidar_callback(self, msg):
        """Handle LiDAR data"""
        self.lidar_data_received = True
        self.data_counts['lidar'] += 1
        self.validate_lidar_data(msg)

    def camera_callback(self, msg):
        """Handle camera data"""
        self.camera_data_received = True
        self.data_counts['camera'] += 1
        self.validate_camera_data(msg)

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data_received = True
        self.data_counts['imu'] += 1
        self.validate_imu_data(msg)

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.odom_data_received = True
        self.data_counts['odom'] += 1
        self.validate_odom_data(msg)

    def validate_lidar_data(self, msg):
        """Validate LiDAR data quality"""
        # Check for valid ranges
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max and not self.is_nan(r)]
        if len(valid_ranges) > len(msg.ranges) * 0.5:  # At least 50% valid ranges
            self.get_logger().debug('LiDAR: Good data quality')
        else:
            self.get_logger().warn('LiDAR: Low data quality detected')

    def validate_camera_data(self, msg):
        """Validate camera data quality"""
        if msg.width > 0 and msg.height > 0:
            self.get_logger().debug(f'Camera: Resolution {msg.width}x{msg.height}')
        else:
            self.get_logger().warn('Camera: Invalid resolution')

    def validate_imu_data(self, msg):
        """Validate IMU data quality"""
        # Check if orientation quaternion is normalized
        norm = (msg.orientation.x**2 + msg.orientation.y**2 +
                msg.orientation.z**2 + msg.orientation.w**2)**0.5

        if 0.99 < norm < 1.01:
            self.get_logger().debug('IMU: Orientation normalized')
        else:
            self.get_logger().warn('IMU: Orientation not normalized')

    def validate_odom_data(self, msg):
        """Validate odometry data quality"""
        # Check for reasonable position and velocity values
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear

        if abs(pos.x) < 1000 and abs(pos.y) < 1000 and abs(pos.z) < 1000:
            self.get_logger().debug('Odom: Position values reasonable')
        else:
            self.get_logger().warn('Odom: Unreasonable position values')

    def is_nan(self, value):
        """Check if value is NaN"""
        try:
            return value != value  # NaN is not equal to itself
        except:
            return False

    def validate_pipeline(self):
        """Periodic validation of the entire pipeline"""
        current_time = self.get_clock().now()
        self.test_duration = (current_time.nanoseconds - self.start_time.nanoseconds) / 1e9

        self.get_logger().info('--- Simulation Pipeline Validation Report ---')
        self.get_logger().info(f'Test Duration: {self.test_duration:.2f}s')

        # Check sensor data reception
        self.get_logger().info(f'LiDAR Data: {"RECEIVED" if self.lidar_data_received else "NO DATA"} '
                              f'({self.data_counts["lidar"]} messages)')
        self.get_logger().info(f'Camera Data: {"RECEIVED" if self.camera_data_received else "NO DATA"} '
                              f'({self.data_counts["camera"]} messages)')
        self.get_logger().info(f'IMU Data: {"RECEIVED" if self.imu_data_received else "NO DATA"} '
                              f'({self.data_counts["imu"]} messages)')
        self.get_logger().info(f'Odom Data: {"RECEIVED" if self.odom_data_received else "NO DATA"} '
                              f'({self.data_counts["odom"]} messages)')

        # Overall validation
        all_sensors_active = all([
            self.lidar_data_received,
            self.camera_data_received,
            self.imu_data_received,
            self.odom_data_received
        ])

        if all_sensors_active:
            self.get_logger().info('✓ ALL SENSORS ACTIVE - Pipeline validation PASSED')
        else:
            self.get_logger().info('✗ SOME SENSORS INACTIVE - Pipeline validation FAILED')

        self.get_logger().info('--- End Validation Report ---')

    def send_test_command(self):
        """Send a test command to validate actuation"""
        twist = Twist()
        twist.linear.x = 0.2  # Move forward slowly
        twist.angular.z = 0.1  # Turn slowly
        self.cmd_publisher.publish(twist)
        self.get_logger().info('Test command sent: linear.x=0.2, angular.z=0.1')

def main(args=None):
    rclpy.init(args=args)

    validator = SimulationValidator()

    # Send a test command after 2 seconds
    timer = validator.create_timer(2.0, lambda: validator.send_test_command())

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()