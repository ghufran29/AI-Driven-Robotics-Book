#!/usr/bin/env python3

"""
ROS 2 Service Server Example

This example demonstrates how to create a service server in ROS 2 using rclpy.
The service allows clients to request the robot to move to a specific location.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Using standard service for example
import time


class RobotMovementService(Node):
    """
    A service server that handles robot movement requests.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_movement_service'
        super().__init__('robot_movement_service')

        # Create a service with the AddTwoInts interface, service name, and callback
        # In a real robot application, you would define a custom service interface
        # for movement commands (e.g., MoveToPosition.srv)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints_service',  # Service name - in real usage would be 'move_to_position'
            self.add_two_ints_callback
        )

        self.get_logger().info('Robot movement service server started')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that processes the service request and returns a response.
        """
        # Simulate some processing time
        time.sleep(0.5)

        # Perform the operation (in this example, addition)
        # In a real robot service, this would involve:
        # 1. Validating the requested position
        # 2. Planning a path to the destination
        # 3. Executing the movement
        # 4. Returning success/failure status
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}'
        )

        # Return the response (automatically sent back to the client)
        return response


def main(args=None):
    """
    Main function that initializes the service server and starts processing requests.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the service server
    robot_service = RobotMovementService()

    try:
        # Keep the service server running and processing requests
        rclpy.spin(robot_service)
    except KeyboardInterrupt:
        robot_service.get_logger().info('Shutting down service server...')
    finally:
        # Destroy the node explicitly (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        robot_service.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()