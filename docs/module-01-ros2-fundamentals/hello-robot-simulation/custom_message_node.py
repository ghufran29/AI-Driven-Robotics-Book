#!/usr/bin/env python3

"""
Custom Message Node Example

This example demonstrates how to work with custom message types in ROS 2.
Since we're creating this for educational purposes, we'll simulate a custom
message using Python classes that follow the same structure as a real message.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json


# Simulate a custom message class (in a real ROS 2 package, this would be
# generated from a .msg file in the msg/ directory)
class RobotControlCommand:
    """
    Simulated custom message class representing a robot control command.
    In a real ROS 2 package, this would be generated from a .msg file.

    Equivalent to:
    string robot_id
    float64 linear_velocity
    float64 angular_velocity
    string command_type
    float64[] parameters
    """

    def __init__(self):
        self.robot_id = ""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.command_type = ""
        self.parameters = []

    def to_string(self):
        """Convert the custom message to a string representation."""
        return json.dumps({
            'robot_id': self.robot_id,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
            'command_type': self.command_type,
            'parameters': self.parameters
        })

    @classmethod
    def from_string(cls, data_str):
        """Create a custom message from a string representation."""
        data = json.loads(data_str)
        msg = cls()
        msg.robot_id = data.get('robot_id', '')
        msg.linear_velocity = data.get('linear_velocity', 0.0)
        msg.angular_velocity = data.get('angular_velocity', 0.0)
        msg.command_type = data.get('command_type', '')
        msg.parameters = data.get('parameters', [])
        return msg


class CustomMessageNode(Node):
    """
    A node that demonstrates working with custom message types.
    """

    def __init__(self):
        # Initialize the node with the name 'custom_message_node'
        super().__init__('custom_message_node')

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_001')
        self.robot_id = self.get_parameter('robot_id').value

        # Create a QoS profile
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create publisher and subscriber for our custom message
        # (using String messages to simulate custom message transport)
        self.custom_cmd_publisher = self.create_publisher(
            String, 'custom_robot_command', qos_profile
        )
        self.custom_cmd_subscription = self.create_subscription(
            String,
            'custom_robot_command',
            self.custom_cmd_callback,
            qos_profile
        )

        # Create a timer to periodically send custom commands
        self.timer = self.create_timer(2.0, self.send_custom_command)

        self.command_counter = 0
        self.get_logger().info(f'Custom Message Node initialized for {self.robot_id}')

    def send_custom_command(self):
        """
        Send a custom robot command message.
        """
        # Create a custom message
        custom_msg = RobotControlCommand()
        custom_msg.robot_id = self.robot_id
        custom_msg.linear_velocity = 0.5 + (self.command_counter * 0.1)
        custom_msg.angular_velocity = 0.2 * (self.command_counter % 4 - 2)
        custom_msg.command_type = "navigation"
        custom_msg.parameters = [1.0, 2.0, 3.0, float(self.command_counter)]

        # Convert to string for transport (simulating serialization)
        msg_str = String()
        msg_str.data = custom_msg.to_string()

        # Publish the message
        self.custom_cmd_publisher.publish(msg_str)
        self.get_logger().info(
            f'Sent custom command #{self.command_counter}: {custom_msg.command_type}'
        )
        self.command_counter += 1

    def custom_cmd_callback(self, msg):
        """
        Callback for receiving custom robot command messages.
        """
        try:
            # Parse the custom message from the string
            custom_msg = RobotControlCommand.from_string(msg.data)

            self.get_logger().info(
                f'Received custom command for robot {custom_msg.robot_id}: '
                f'linear={custom_msg.linear_velocity:.2f}, '
                f'angular={custom_msg.angular_velocity:.2f}, '
                f'type={custom_msg.command_type}'
            )

            # Process the command based on type
            if custom_msg.command_type == "navigation":
                self.execute_navigation_command(custom_msg)
            elif custom_msg.command_type == "action":
                self.execute_action_command(custom_msg)
            else:
                self.get_logger().warn(f'Unknown command type: {custom_msg.command_type}')

        except Exception as e:
            self.get_logger().error(f'Error parsing custom message: {e}')

    def execute_navigation_command(self, cmd):
        """
        Execute a navigation command.
        """
        self.get_logger().info(
            f'Executing navigation command: '
            f'v={cmd.linear_velocity:.2f}, w={cmd.angular_velocity:.2f}'
        )

        # In a real robot, this would publish to /cmd_vel
        # For simulation, we just log the action
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = cmd.linear_velocity
        cmd_vel_msg.angular.z = cmd.angular_velocity

        self.get_logger().info(
            f'Simulated sending to /cmd_vel: linear.x={cmd_vel_msg.linear.x:.2f}, '
            f'angular.z={cmd_vel_msg.angular.z:.2f}'
        )

    def execute_action_command(self, cmd):
        """
        Execute an action command.
        """
        self.get_logger().info(
            f'Executing action command with parameters: {cmd.parameters}'
        )


def main(args=None):
    """
    Main function that initializes the node and starts spinning.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the CustomMessageNode
    custom_node = CustomMessageNode()

    # Start spinning the node
    try:
        rclpy.spin(custom_node)
    except KeyboardInterrupt:
        custom_node.get_logger().info('Shutting down custom message node...')
    finally:
        # Destroy the node explicitly
        custom_node.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()