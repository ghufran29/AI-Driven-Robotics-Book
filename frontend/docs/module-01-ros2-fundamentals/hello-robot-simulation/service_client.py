#!/usr/bin/env python3

"""
ROS 2 Service Client Example

This example demonstrates how to create a service client in ROS 2 using rclpy.
The client calls the robot movement service to request the robot to move to a location.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Using standard service for example


class RobotMovementClient(Node):
    """
    A service client that sends requests to the robot movement service.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_movement_client'
        super().__init__('robot_movement_client')

        # Create a client for the service
        # The service type must match the service server's interface
        self.cli = self.create_client(AddTwoInts, 'add_two_ints_service')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service and return the future object.
        """
        # Set the request parameters
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)
        return self.future


def main(args=None):
    """
    Main function that initializes the client and sends a request.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the service client
    robot_client = RobotMovementClient()

    # Check if command line arguments are provided
    if len(sys.argv) != 3:
        print('Usage: ros2 run python3 service_client.py <int1> <int2>')
        print('Using default values: 1 2')
        a, b = 1, 2
    else:
        a, b = int(sys.argv[1]), int(sys.argv[2])

    # Send the request
    future = robot_client.send_request(a, b)

    try:
        # Wait for the result (blocking until the service responds)
        rclpy.spin_until_future_complete(robot_client, future)

        # Check if the service call was successful
        if future.result() is not None:
            # Get the response
            response = future.result()
            robot_client.get_logger().info(
                f'Result of {a} + {b} = {response.sum}'
            )
        else:
            # Handle the case where the service call failed
            robot_client.get_logger().error(
                f'Exception while calling service: {future.exception()}'
            )
    except KeyboardInterrupt:
        robot_client.get_logger().info('Client interrupted by user')
    finally:
        # Destroy the node explicitly
        robot_client.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()