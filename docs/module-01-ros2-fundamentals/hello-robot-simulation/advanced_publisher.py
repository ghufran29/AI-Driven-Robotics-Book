#!/usr/bin/env python3

"""
Advanced ROS 2 Publisher Example

This example demonstrates advanced features of ROS 2 publishers using rclpy,
including custom message types, Quality of Service (QoS) profiles, and
parameter configuration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time


class AdvancedPublisher(Node):
    """
    An advanced publisher node that demonstrates various ROS 2 features.
    """

    def __init__(self):
        # Initialize the node with the name 'advanced_publisher'
        super().__init__('advanced_publisher')

        # Declare parameters with default values
        self.declare_parameter('publish_frequency', 2)  # Hz
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('movement_pattern', 'circle')  # circle, square, or line

        # Get parameter values
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.robot_name = self.get_parameter('robot_name').value
        self.movement_pattern = self.get_parameter('movement_pattern').value

        # Create a QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create publishers for different message types
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)

        # Create a timer with the specified frequency
        timer_period = 1.0 / self.publish_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables for movement patterns
        self.time_counter = 0.0
        self.get_logger().info(
            f'Advanced Publisher initialized for robot: {self.robot_name}, '
            f'pattern: {self.movement_pattern}, frequency: {self.publish_frequency}Hz'
        )

    def timer_callback(self):
        """
        This method is called every time the timer expires.
        It publishes messages based on the selected movement pattern.
        """
        # Create and publish a status message
        status_msg = String()
        status_msg.data = f'{self.robot_name} status: operating normally at t={self.time_counter:.2f}s'
        self.status_publisher.publish(status_msg)

        # Create and publish a movement command based on the pattern
        cmd_vel_msg = Twist()

        if self.movement_pattern == 'circle':
            # Circular movement: linear velocity forward, angular velocity to turn
            cmd_vel_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_vel_msg.angular.z = 0.5  # Turn at 0.5 rad/s
        elif self.movement_pattern == 'square':
            # Square movement: alternate between moving forward and turning
            phase = int(self.time_counter * 2) % 4  # 4 phases for square: 2s forward, 2s turn
            if phase in [0, 2]:  # Moving forward phases
                cmd_vel_msg.linear.x = 0.5
                cmd_vel_msg.angular.z = 0.0
            else:  # Turning phases
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.785  # 45 degrees per second
        else:  # Default to linear movement
            cmd_vel_msg.linear.x = 0.3
            cmd_vel_msg.angular.z = 0.0

        # Publish the movement command
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Log the published commands
        self.get_logger().info(
            f'Published - Linear: {cmd_vel_msg.linear.x:.2f}, '
            f'Angular: {cmd_vel_msg.angular.z:.2f}'
        )

        # Increment the time counter
        self.time_counter += 1.0 / self.publish_frequency


def main(args=None):
    """
    Main function that initializes the node and starts spinning.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the AdvancedPublisher node
    advanced_publisher = AdvancedPublisher()

    # Start spinning the node - this keeps the node running and processes callbacks
    try:
        rclpy.spin(advanced_publisher)
    except KeyboardInterrupt:
        advanced_publisher.get_logger().info('Shutting down advanced publisher...')
    finally:
        # Destroy the node explicitly
        advanced_publisher.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()