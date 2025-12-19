#!/usr/bin/env python3

"""
Basic ROS 2 Publisher Example

This example demonstrates how to create a simple publisher node in ROS 2 using rclpy.
The publisher sends "Hello World" messages to a topic called "chatter" at a rate of 2 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that publishes messages to a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages to the 'chatter' topic
        # with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Set the timer period in seconds (0.5 seconds = 2 Hz)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of the number of messages published
        self.i = 0

    def timer_callback(self):
        """
        This method is called every time the timer expires.
        It creates and publishes a message to the 'chatter' topic.
        """
        # Create a String message
        msg = String()
        # Set the data field to "Hello World" with a counter
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function that initializes the node and starts spinning.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    # Start spinning the node - this keeps the node running and processes callbacks
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()