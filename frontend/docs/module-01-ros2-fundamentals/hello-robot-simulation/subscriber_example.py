#!/usr/bin/env python3

"""
Basic ROS 2 Subscriber Example

This example demonstrates how to create a simple subscriber node in ROS 2 using rclpy.
The subscriber listens to messages on the "chatter" topic and prints them to the console.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber node that subscribes to messages from a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'chatter' topic with String messages
        # with a queue size of 10, and specify the callback function
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription  # This line is to prevent the linter from warning about unused variable

    def listener_callback(self, msg):
        """
        This method is called every time a message is received on the 'chatter' topic.
        It logs the received message to the console.
        """
        # Log the received message to the console
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function that initializes the node and starts spinning.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    # Start spinning the node - this keeps the node running and processes callbacks
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()