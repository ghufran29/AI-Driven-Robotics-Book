#!/usr/bin/env python3

"""
ROS 2 Action Server Example

This example demonstrates how to create an action server in ROS 2 using rclpy.
The action allows clients to request the robot to move to a specific location
with feedback during the movement and the ability to cancel the operation.
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# For this example, we'll create a mock action interface since the real one
# might not be available in this environment
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


class RobotMoveActionServer(Node):
    """
    An action server that handles robot movement requests with feedback and cancellation.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_move_action_server'
        super().__init__('robot_move_action_server')

        # Create an action server with the interface type, action name, and callback
        self._action_server = ActionServer(
            self,
            MoveToPosition,  # In real usage: would be a defined action interface
            'move_to_position',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Simulate robot state
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info('Robot move action server started')

    def destroy(self):
        """
        Clean up the action server before destroying the node.
        """
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """
        Accept or reject a goal request.
        """
        self.get_logger().info(
            f'Received goal request to move to ({goal_request.x}, {goal_request.y})'
        )
        # Accept all goals for this example
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept or reject a cancel request.
        """
        self.get_logger().info('Received cancel request')
        # Accept all cancel requests for this example
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute the goal and provide feedback during execution.
        """
        self.get_logger().info('Executing goal...')

        # Get the goal parameters
        goal = goal_handle.request
        target_x = goal.x
        target_y = goal.y
        tolerance = goal.tolerance

        # Create result message
        result = MoveToPosition.Result()

        # Create feedback message
        feedback_msg = MoveToPosition.Feedback()

        # Simulate the movement process
        step_size = 0.1  # Move in small increments
        distance_to_goal = self.calculate_distance(self.current_x, self.current_y, target_x, target_y)

        while distance_to_goal > tolerance and not goal_handle.is_cancel_requested:
            # Calculate movement direction
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = self.calculate_distance(self.current_x, self.current_y, target_x, target_y)

            # Move one step toward the goal
            if distance > step_size:
                self.current_x += (dx / distance) * step_size
                self.current_y += (dy / distance) * step_size
            else:
                self.current_x = target_x
                self.current_y = target_y

            # Update feedback
            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.distance_to_goal = self.calculate_distance(
                self.current_x, self.current_y, target_x, target_y
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'Moving... Current: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Distance to goal: {feedback_msg.distance_to_goal:.2f}'
            )

            # Sleep to simulate real robot movement
            time.sleep(0.5)

            # Recalculate distance
            distance_to_goal = self.calculate_distance(self.current_x, self.current_y, target_x, target_y)

        # Check if the goal was canceled
        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = 'Goal was canceled'
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
        elif distance_to_goal <= tolerance:
            # Goal completed successfully
            result.success = True
            result.message = f'Reached position ({self.current_x:.2f}, {self.current_y:.2f})'
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
        else:
            # Unexpected termination
            result.success = False
            result.message = 'Goal failed for unknown reason'
            goal_handle.abort()
            self.get_logger().info('Goal aborted')

        # Return the result
        return result

    def calculate_distance(self, x1, y1, x2, y2):
        """
        Calculate the Euclidean distance between two points.
        """
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def main(args=None):
    """
    Main function that initializes the action server and starts processing goals.
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the action server
    robot_action_server = RobotMoveActionServer()

    try:
        # Use a multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(robot_action_server)
        executor.spin()
    except KeyboardInterrupt:
        robot_action_server.get_logger().info('Shutting down action server...')
    finally:
        robot_action_server.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()