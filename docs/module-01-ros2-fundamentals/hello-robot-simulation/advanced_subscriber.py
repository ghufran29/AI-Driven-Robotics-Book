#!/usr/bin/env python3

"""
Advanced ROS 2 Subscriber Example

This example demonstrates advanced features of ROS 2 subscribers using rclpy,
including multiple subscriptions, Quality of Service (QoS) profiles, and
message filtering.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import statistics
import math


class AdvancedSubscriber(Node):
    """
    An advanced subscriber node that demonstrates various ROS 2 features.
    """

    def __init__(self):
        # Initialize the node with the name 'advanced_subscriber'
        super().__init__('advanced_subscriber')

        # Declare parameters with default values
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('robot_name', 'turtlebot')

        # Get parameter values
        self.safety_distance = self.get_parameter('safety_distance').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create a QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create subscriptions for different message types
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            qos_profile
        )

        # Create a subscription for simulated sensor data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            qos_profile
        )

        # Store recent sensor readings for analysis
        self.recent_ranges = []
        self.collision_warning_issued = False

        self.get_logger().info(
            f'Advanced Subscriber initialized for robot: {self.robot_name}, '
            f'safety distance: {self.safety_distance}m'
        )

    def cmd_vel_callback(self, msg):
        """
        Callback for receiving movement commands.
        """
        self.get_logger().info(
            f'Received movement command - Linear: {msg.linear.x:.2f}, '
            f'Angular: {msg.angular.z:.2f}'
        )

        # Check if movement command is too aggressive
        if abs(msg.linear.x) > 1.0 or abs(msg.angular.z) > 1.0:
            self.get_logger().warn(
                f'Aggressive movement command detected: '
                f'Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}'
            )

    def status_callback(self, msg):
        """
        Callback for receiving status messages.
        """
        self.get_logger().info(f'Status: {msg.data}')

        # Check for error conditions in status messages
        if 'error' in msg.data.lower():
            self.get_logger().error(f'Error detected in status: {msg.data}')
        elif 'warning' in msg.data.lower():
            self.get_logger().warn(f'Warning in status: {msg.data}')

    def laser_callback(self, msg):
        """
        Callback for processing laser scan data.
        """
        # Store the ranges for analysis
        self.recent_ranges = list(msg.ranges)

        # Filter out invalid range values (inf, nan)
        valid_ranges = [r for r in self.recent_ranges if r != float('inf') and not math.isnan(r)]

        if valid_ranges:
            # Calculate minimum distance to obstacles
            min_distance = min(valid_ranges)

            # Check for collision risk
            if min_distance < self.safety_distance:
                if not self.collision_warning_issued:
                    self.get_logger().warn(
                        f'Collision risk detected! Distance: {min_distance:.2f}m < '
                        f'Safety distance: {self.safety_distance}m'
                    )
                    self.collision_warning_issued = True
            else:
                self.collision_warning_issued = False

            # Calculate average distance for analysis
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            self.get_logger().info(
                f'Laser scan - Min: {min_distance:.2f}m, '
                f'Avg: {avg_distance:.2f}m, '
                f'Sample count: {len(valid_ranges)}'
            )
        else:
            self.get_logger().warn('No valid laser ranges received')

    def get_sensor_statistics(self):
        """
        Get statistics about recent sensor readings.
        """
        if not self.recent_ranges:
            return None

        # Filter out invalid values
        valid_ranges = [r for r in self.recent_ranges if r != float('inf') and not r != r]

        if not valid_ranges:
            return None

        stats = {
            'min': min(valid_ranges),
            'max': max(valid_ranges),
            'mean': statistics.mean(valid_ranges),
            'median': statistics.median(valid_ranges),
            'count': len(valid_ranges)
        }

        return stats


def main(args=None):
    """
    Main function that initializes the node and starts spinning.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the AdvancedSubscriber node
    advanced_subscriber = AdvancedSubscriber()

    # Start spinning the node - this keeps the node running and processes callbacks
    try:
        rclpy.spin(advanced_subscriber)
    except KeyboardInterrupt:
        advanced_subscriber.get_logger().info('Shutting down advanced subscriber...')
    finally:
        # Destroy the node explicitly
        advanced_subscriber.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()