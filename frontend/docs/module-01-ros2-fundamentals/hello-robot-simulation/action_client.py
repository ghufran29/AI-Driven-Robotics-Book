#!/usr/bin/env python3

"""
ROS 2 Action Client Example

This example demonstrates how to create an action client in ROS 2 using rclpy.
The client sends goals to the robot movement action server and receives feedback
during execution, with the ability to cancel the operation if needed.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
import sys

# For this example, we'll use the same mock action interface as the server
class MoveToPosition:
    class Goal:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.tolerance = 0.1

    class Result:
        def __init__(self):
            self.success = False
            self.message = ""

    class Feedback:
        def __init__(self):
            self.current_x = 0.0
            self.current_y = 0.0
            self.distance_to_goal = 0.0


class RobotMoveActionClient(Node):
    """
    An action client that sends goals to the robot movement action server.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_move_action_client'
        super().__init__('robot_move_action_client')

        # Create an action client for the 'move_to_position' action
        self._action_client = ActionClient(
            self,
            MoveToPosition,  # In real usage: would be a defined action interface
            'move_to_position'
        )

    def send_goal(self, x, y, tolerance=0.1):
        """
        Send a goal to the action server and handle responses.
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = MoveToPosition.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.tolerance = tolerance

        # Send the goal asynchronously and register callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add a callback for when the goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response (accepted or rejected).
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the final result of the action.
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')

        # Shutdown after receiving the result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback messages during action execution.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: Current position ({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
            f'Distance to goal: {feedback.distance_to_goal:.2f}'
        )

    def send_cancel_request(self):
        """
        Send a cancel request for the current goal.
        """
        # Note: This is a simplified version - in practice, you'd need to keep track
        # of the goal handle to send a cancel request
        self.get_logger().info('Cancel request would be sent here')


def main(args=None):
    """
    Main function that initializes the action client and sends a goal.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the action client
    robot_action_client = RobotMoveActionClient()

    # Parse command line arguments for target position
    if len(sys.argv) >= 3:
        try:
            target_x = float(sys.argv[1])
            target_y = float(sys.argv[2])
        except ValueError:
            print("Invalid coordinates provided, using default (1.0, 1.0)")
            target_x, target_y = 1.0, 1.0
    else:
        print("No coordinates provided, using default (1.0, 1.0)")
        target_x, target_y = 1.0, 1.0

    # Send a goal to move to the specified position
    robot_action_client.send_goal(target_x, target_y)

    try:
        # Keep the client running to receive feedback and result
        rclpy.spin(robot_action_client)
    except KeyboardInterrupt:
        robot_action_client.get_logger().info('Action client interrupted by user')
        # In a real application, you might want to send a cancel request here
    finally:
        # Destroy the node explicitly
        robot_action_client.destroy_node()
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()