# Chapter 3: Services & Actions

## Introduction to Advanced Communication Patterns

While topics provide an excellent mechanism for continuous data exchange between ROS 2 nodes, many robotics applications require more sophisticated communication patterns. Services and actions provide request-response interactions and goal-oriented communication respectively, enabling complex robot behaviors that require coordination, confirmation, and feedback.

### Accessibility Considerations for Communication Patterns

When implementing services and actions in robotics applications, consider these accessibility aspects:

- **Clear Service/Action Names**: Use descriptive names that clearly indicate the function (e.g., `move_to_position` instead of `mtop`)
- **Comprehensive Documentation**: Document request/response parameters and their meanings clearly
- **Consistent Parameter Types**: Use standard message types when possible to reduce cognitive load
- **Feedback Clarity**: Provide clear, meaningful feedback messages during action execution
- **Error Reporting**: Return informative error messages that help users understand what went wrong
- **Progress Indicators**: For long-running actions, provide meaningful progress indicators
- **Cancellation Options**: Always provide clear ways to cancel long-running operations
- **State Transparency**: Make action states easily observable and understandable

## Services: Synchronous Request-Response Communication

Services provide a synchronous request-response communication pattern in ROS 2. Unlike topics which are asynchronous, services guarantee that a request is processed and a response is returned before the client continues execution.

### Service Architecture

A service system consists of:
- **Service Server**: Node that provides the service and processes requests
- **Service Client**: Node that calls the service and waits for a response
- **Service Interface**: Defines the request and response message types

### Creating a Service

First, let's look at how to create a service interface. In a real ROS 2 package, you would define this in a `.srv` file, but for this educational example, we'll use standard ROS 2 service types:

```python
# Example service interface (in .srv file format):
# Request message
# string name
# float64 value
# ---
# Response message
# bool success
# string message
```

### Service Server Implementation

Here's how to implement a service server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Standard ROS 2 service example


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service with the interface type, service name, and callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        """Process the service request and return a response."""
        # Perform the service operation
        response.sum = request.a + request.b

        # Log the request
        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )

        # Return the response (automatically sent back to client)
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.get_logger().info('Shutting down service server...')
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client Implementation

Here's how to implement a service client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Standard ROS 2 service example
import sys


class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create a client for the service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """Send a request to the service and return the future."""
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()

    # Send a request
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    try:
        # Wait for the result
        rclpy.spin_until_future_complete(minimal_client, future)

        if future.result() is not None:
            response = future.result()
            minimal_client.get_logger().info(
                f'Result of {sys.argv[1]} + {sys.argv[2]} = {response.sum}'
            )
        else:
            minimal_client.get_logger().error('Exception while calling service: %r' % future.exception())
    except KeyboardInterrupt:
        minimal_client.get_logger().info('Shutting down client...')
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Custom Service Example

Here's an example of a custom service for robot control:

```python
import rclpy
from rclpy.node import Node
# In a real implementation, you would use a custom service type
# For this example, we'll use SetBool which is similar to what we need
from std_srvs.srv import SetBool


class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')

        # Create a service to control robot movement
        self.srv = self.create_service(
            SetBool,
            'robot_move_to_location',
            self.move_to_location_callback
        )

        # Simulate robot state
        self.robot_position = [0.0, 0.0]
        self.robot_moving = False

    def move_to_location_callback(self, request, response):
        """Handle move to location requests."""
        if self.robot_moving:
            response.success = False
            response.message = 'Robot is currently moving, cannot accept new commands'
            return response

        # In a real implementation, this would trigger actual movement
        # For simulation, we'll just update the position
        try:
            # Parse target coordinates from the request data
            # (In a custom service, we'd have proper fields for x, y coordinates)
            target_x, target_y = 1.0, 1.0  # Example coordinates

            self.get_logger().info(
                f'Moving robot from ({self.robot_position[0]}, {self.robot_position[1]}) '
                f'to ({target_x}, {target_y})'
            )

            # Simulate movement
            self.robot_moving = True
            self.robot_position = [target_x, target_y]
            self.robot_moving = False

            response.success = True
            response.message = f'Robot successfully moved to ({target_x}, {target_y})'

        except Exception as e:
            response.success = False
            response.message = f'Error moving robot: {str(e)}'

        return response


def main(args=None):
    rclpy.init(args=args)
    robot_service = RobotControlService()

    try:
        rclpy.spin(robot_service)
    except KeyboardInterrupt:
        robot_service.get_logger().info('Shutting down robot control service...')
    finally:
        robot_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions: Asynchronous Goal-Oriented Communication

Actions are designed for long-running tasks that require feedback, status updates, and the ability to cancel operations. They're perfect for complex robot behaviors like navigation, manipulation, or any task that takes a significant amount of time to complete.

### Action Architecture

An action system consists of:
- **Action Server**: Node that executes the action and provides feedback
- **Action Client**: Node that sends goals and receives feedback/results
- **Action Interface**: Defines goal, feedback, and result message types

### Action States

Actions have several states that track the progress of a goal:
- **PENDING**: Goal accepted but not yet started
- **ACTIVE**: Goal is currently being processed
- **FEEDBACK**: Goal is active and feedback is being sent
- **SUCCEEDED**: Goal completed successfully
- **ABORTED**: Goal execution failed
- **CANCELED**: Goal was canceled by the client

### Action Server Implementation

Here's how to implement an action server:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Using a standard action type for example
from irobot_msgs.action import DriveDistance  # This would be from irobot package in real usage
# For our example, we'll use a mock action since the actual action type might not be available
import time
from std_msgs.msg import Empty


class MockDriveDistance:  # This is a mock since the real action might not be available
    class Goal:
        def __init__(self):
            self.distance = 0.0
            self.speed = 0.0

    class Result:
        def __init__(self):
            self.success = False

    class Feedback:
        def __init__(self):
            self.distance_traveled = 0.0


class DriveActionServer(Node):
    def __init__(self):
        super().__init__('drive_action_server')

        # Create an action server
        self._action_server = ActionServer(
            self,
            MockDriveDistance,  # In real usage: DriveDistance
            'drive_distance',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Get the goal parameters
        goal = goal_handle.request
        distance = goal.distance
        speed = goal.speed

        # Create a result message
        result = MockDriveDistance.Result()  # In real usage: DriveDistance.Result()

        # Simulate driving with feedback
        feedback_msg = MockDriveDistance.Feedback()  # In real usage: DriveDistance.Feedback()

        # Simulate the driving process
        traveled = 0.0
        while traveled < distance and not goal_handle.is_cancel_requested:
            # Update feedback
            traveled += speed * 0.1  # Simulate incremental progress
            feedback_msg.distance_traveled = traveled
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Driving... {traveled:.2f}/{distance:.2f}m')

            # Sleep to simulate real robot movement
            time.sleep(0.1)

        # Check if the goal was canceled
        if goal_handle.is_cancel_requested:
            result.success = False
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
        else:
            result.success = True
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')

        return result


def main(args=None):
    rclpy.init(args=args)

    # Create the action server node
    drive_server = DriveActionServer()

    try:
        # Use a multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(drive_server)
        executor.spin()
    except KeyboardInterrupt:
        drive_server.get_logger().info('Shutting down action server...')
    finally:
        drive_server.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Action Client Implementation

Here's how to implement an action client:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Using mock action since real action type might not be available
class MockDriveDistance:
    class Goal:
        def __init__(self):
            self.distance = 0.0
            self.speed = 0.0

    class Result:
        def __init__(self):
            self.success = False

    class Feedback:
        def __init__(self):
            self.distance_traveled = 0.0


class DriveActionClient(Node):
    def __init__(self):
        super().__init__('drive_action_client')

        # Create an action client
        self._action_client = ActionClient(
            self,
            MockDriveDistance,  # In real usage: DriveDistance
            'drive_distance'
        )

    def send_goal(self, distance, speed):
        """Send a goal to the action server."""
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = MockDriveDistance.Goal()  # In real usage: DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.speed = speed

        # Send the goal and register callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback messages."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.distance_traveled:.2f}m')


def main(args=None):
    rclpy.init(args=args)

    action_client = DriveActionClient()

    # Send a goal
    action_client.send_goal(2.0, 0.5)  # Drive 2 meters at 0.5 m/s

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Shutting down action client...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## When to Use Services vs Actions vs Topics

### Use Topics when:
- You need continuous data flow (sensor data, robot state)
- Communication can be asynchronous
- You have many-to-many relationships
- Real-time performance is critical

### Use Services when:
- You need request-response communication
- The operation is relatively quick (under a few seconds)
- You need guaranteed delivery and response
- You want simple, synchronous communication

### Use Actions when:
- The operation takes a long time to complete
- You need feedback during execution
- You need the ability to cancel operations
- You need to track the state of long-running tasks

## Service Server Example

Here's a complete example of a service server that handles robot movement requests:

```python
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
```

## Service Client Example

Here's a complete example of a service client that sends requests to the robot movement service:

```python
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
```

## Action Server Example

Here's a complete example of an action server that handles robot movement with feedback:

```python
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
```

## Action Client States and Transitions

Actions in ROS 2 have a well-defined state machine that tracks the progress of goals. Understanding these states is crucial for implementing robust action clients and servers.

### Action Goal States

An action goal transitions through the following states:

1. **PENDING**: The goal has been received by the action server but has not started processing yet
2. **ACTIVE**: The goal has been accepted and is currently being processed by the action server
3. **FEEDBACK**: While active, the goal may publish feedback messages to update the client on progress
4. **SUCCEEDED**: The goal has completed successfully and the result is available
5. **ABORTED**: The goal was terminated due to a failure during execution
6. **CANCELED**: The goal was canceled either by the client or the server

### State Transition Diagram

```
[Client sends goal] → PENDING → ACTIVE → FEEDBACK → SUCCEEDED
                          ↓         ↓         ↓         ↓
                      [Cancel]    [Cancel]  [Cancel]  [Get Result]
                          ↓         ↓         ↓         ↓
                      CANCELING   CANCELING CANCELING    ↓
                          ↓         ↓         ↓         ↓
                      CANCELED ← [Cancel] [Cancel] [Cancel]
                          ↓
                      [Get Result]
```

### Handling Action States in Code

In your action client code, you'll handle these states through various callbacks:

- **Goal Response Callback**: Handles the response when the server accepts or rejects the goal (PENDING state)
- **Feedback Callback**: Processes feedback messages while the goal is ACTIVE
- **Result Callback**: Processes the final result when the goal reaches SUCCEEDED, ABORTED, or CANCELED state

### Example: State Handling

```python
def goal_response_callback(self, future):
    """
    Handle the goal response (PENDING state).
    """
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected :(')
        return

    self.get_logger().info('Goal accepted, now ACTIVE :)')
    # The goal is now in ACTIVE state
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)

def feedback_callback(self, feedback_msg):
    """
    Handle feedback messages (FEEDBACK state).
    """
    # The goal is in ACTIVE state with feedback being published
    feedback = feedback_msg.feedback
    self.get_logger().info(f'Feedback: {feedback.distance_to_goal:.2f}m remaining')

def get_result_callback(self, future):
    """
    Handle the final result (SUCCEEDED, ABORTED, or CANCELED state).
    """
    result = future.result().result
    if result.success:
        self.get_logger().info(f'Goal SUCCEEDED: {result.message}')
    else:
        self.get_logger().info(f'Goal did not succeed: {result.message}')
```

### Cancellation

Cancellation can be initiated by either the client or the server:
- **Client-initiated**: The client can request to cancel a goal that is in PENDING or ACTIVE state
- **Server-initiated**: The server may cancel a goal due to an error or other condition

When a cancel request is accepted, the goal transitions to CANCELING state before reaching CANCELED state.

## Hands-On Exercise: Implementing Complex Robot Tasks with Actions

### Exercise Goals:
- Implement a complex robot task using actions with feedback and cancellation
- Create an action server for a multi-step robot operation
- Develop an action client that monitors progress and can cancel operations
- Handle different action states and transitions appropriately

### Exercise Overview:
In this exercise, you'll implement a "Robot Patrol" action that moves the robot through multiple waypoints, providing feedback on progress and allowing cancellation if needed.

### Steps:

1. **Design the Patrol Action Interface**:
   - Create a custom action definition for patrol tasks
   - Define goal parameters (list of waypoints, speed, safety parameters)
   - Define feedback (current waypoint, progress percentage, obstacles detected)
   - Define result (success, failure reason, final position)

2. **Implement the Patrol Action Server**:
   - Create a server that accepts patrol goals
   - Implement navigation to each waypoint in sequence
   - Publish feedback during the patrol
   - Handle cancellation requests appropriately
   - Return appropriate results

3. **Implement the Patrol Action Client**:
   - Create a client that sends patrol goals
   - Handle feedback to monitor progress
   - Implement cancellation capability
   - Process the final result

4. **Test and Validate**:
   - Test with different waypoint sequences
   - Verify feedback is published correctly
   - Test cancellation functionality
   - Validate error handling

### Sample Implementation Structure:

```python
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# Define mock action interface for patrol
class Patrol:
    class Goal:
        def __init__(self):
            self.waypoints = []  # List of (x, y) tuples
            self.speed = 0.5
            self.tolerance = 0.2

    class Result:
        def __init__(self):
            self.success = False
            self.completed_waypoints = 0
            self.error_message = ""

    class Feedback:
        def __init__(self):
            self.current_waypoint_index = 0
            self.progress_percentage = 0.0
            self.current_position_x = 0.0
            self.current_position_y = 0.0
            self.obstacles_detected = False

class PatrolActionServer(Node):
    def __init__(self):
        super().__init__('patrol_action_server')

        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol_robot',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.current_x = 0.0
        self.current_y = 0.0

    def goal_callback(self, goal_request):
        """Accept patrol goals if robot is available."""
        self.get_logger().info(f'Received patrol request with {len(goal_request.waypoints)} waypoints')
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info('Received cancel request for patrol')
        return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the patrol task with feedback and cancellation."""
        self.get_logger().info('Starting patrol execution')

        result = Patrol.Result()
        feedback_msg = Patrol.Feedback()

        waypoints = goal_handle.request.waypoints
        total_waypoints = len(waypoints)
        completed_waypoints = 0

        # Patrol through each waypoint
        for i, (target_x, target_y) in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                result.success = False
                result.completed_waypoints = completed_waypoints
                result.error_message = "Patrol canceled by request"
                goal_handle.canceled()
                self.get_logger().info('Patrol canceled')
                return result

            # Simulate navigation to waypoint
            self.get_logger().info(f'Navigating to waypoint {i+1}/{total_waypoints}: ({target_x}, {target_y})')

            # Move toward the waypoint with feedback
            distance = self.calculate_distance(self.current_x, self.current_y, target_x, target_y)
            step_size = 0.1

            while distance > goal_handle.request.tolerance and not goal_handle.is_cancel_requested:
                # Move toward waypoint
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dist = self.calculate_distance(self.current_x, self.current_y, target_x, target_y)

                if dist > step_size:
                    self.current_x += (dx / dist) * step_size
                    self.current_y += (dy / dist) * step_size
                else:
                    self.current_x = target_x
                    self.current_y = target_y

                # Update feedback
                feedback_msg.current_waypoint_index = i
                feedback_msg.progress_percentage = (completed_waypoints + (1 - distance / dist)) * 100 / total_waypoints
                feedback_msg.current_position_x = self.current_x
                feedback_msg.current_position_y = self.current_y
                feedback_msg.obstacles_detected = False  # Simplified

                goal_handle.publish_feedback(feedback_msg)

                # Simulate movement time
                time.sleep(0.2)

                distance = self.calculate_distance(self.current_x, self.current_y, target_x, target_y)

            if goal_handle.is_cancel_requested:
                break

            completed_waypoints += 1
            self.get_logger().info(f'Reached waypoint {i+1}/{total_waypoints}')

        # Return result
        if not goal_handle.is_cancel_requested:
            result.success = True
            result.completed_waypoints = completed_waypoints
            result.error_message = ""
            goal_handle.succeed()
            self.get_logger().info('Patrol completed successfully')
        else:
            result.success = False
            result.completed_waypoints = completed_waypoints
            result.error_message = "Patrol canceled"
            goal_handle.canceled()
            self.get_logger().info('Patrol canceled')

        return result

    def calculate_distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

def main(args=None):
    rclpy.init(args=args)
    patrol_server = PatrolActionServer()

    try:
        rclpy.spin(patrol_server)
    except KeyboardInterrupt:
        patrol_server.get_logger().info('Shutting down patrol server')
    finally:
        patrol_server.destroy_node()
        rclpy.shutdown()
```

### Exercise Enhancement Ideas:
- Add obstacle detection and avoidance during patrol
- Implement path optimization for the waypoints
- Add multiple robot coordination
- Include sensor data collection during patrol
- Add dynamic replanning capabilities

### Validation:
Your implementation should successfully:
- Navigate to all waypoints in sequence
- Provide continuous feedback on progress
- Handle cancellation requests appropriately
- Return proper results based on execution outcome

## Comparison: Topics vs Services vs Actions

Understanding when to use each communication pattern is crucial for effective ROS 2 design. Here's a comprehensive comparison:

### Topics (Publish/Subscribe)

**Characteristics:**
- Asynchronous communication
- Many-to-many relationships
- Continuous data flow
- No guaranteed delivery
- No response mechanism

**Best for:**
- Sensor data streams (camera images, laser scans, IMU data)
- Robot state publishing (odometry, joint states)
- Continuous control commands (velocity commands)
- Broadcasting information to multiple subscribers

**Example Use Cases:**
- Publishing camera images for computer vision processing
- Broadcasting robot odometry for localization
- Sending continuous velocity commands to robot base
- Broadcasting system status to monitoring nodes

### Services (Request/Response)

**Characteristics:**
- Synchronous communication
- One-to-one relationships
- Request-response pattern
- Guaranteed delivery and response
- Quick operations (typically under a few seconds)

**Best for:**
- Configuration changes
- Triggering specific operations
- Querying current state
- Operations that should complete quickly

**Example Use Cases:**
- Saving a map (`/map_server/save_map`)
- Changing robot mode (e.g., from autonomous to manual)
- Querying robot battery level
- Triggering calibration procedures
- Requesting a path plan for navigation

### Actions (Goal-Based Communication)

**Characteristics:**
- Asynchronous long-running operations
- Goal-feedback-result pattern
- Support for cancellation
- State tracking (pending, active, succeeded, aborted, canceled)
- Suitable for complex tasks

**Best for:**
- Navigation to a specific location
- Manipulation tasks
- Calibration procedures
- Any task that takes significant time
- Tasks requiring feedback during execution

**Example Use Cases:**
- Moving robot to a specific location (`/navigate_to_pose`)
- Performing a complex manipulation task
- Running a system calibration procedure
- Executing a multi-step inspection routine
- Performing a long-running data collection task

### Decision Matrix

| Requirement | Topics | Services | Actions |
|-------------|--------|----------|---------|
| Continuous data | ✅ | ❌ | ❌ |
| Request-response | ❌ | ✅ | ✅* |
| Long-running tasks | ❌ | ❌ | ✅ |
| Feedback during execution | ❌ | ❌ | ✅ |
| Cancellation | ❌ | ❌ | ✅ |
| Multiple consumers | ✅ | ❌ | ❌ |
| Guaranteed delivery | ❌ | ✅ | ✅ |
| Quick operations | ✅ | ✅ | ❌ |

*Actions provide result, not immediate response like services

### When to Use Each Pattern

**Use Topics when:**
- You need to broadcast information continuously
- Multiple nodes need to receive the same data
- Real-time performance is critical
- The operation is stateless

**Use Services when:**
- You need guaranteed request-response communication
- The operation completes quickly (under a few seconds)
- You need to query or change state atomically
- You have a simple, synchronous operation

**Use Actions when:**
- The operation takes a long time to complete
- You need to monitor progress during execution
- You might need to cancel the operation
- The task involves multiple steps with potential for failure
- You need to track the state of the operation

### Real-World Examples

**Navigation System:**
- Topics: Robot odometry, sensor data, system status
- Services: Get map, clear costmaps, get path
- Actions: Navigate to pose, follow waypoint list

**Manipulation System:**
- Topics: Joint states, camera feeds, gripper position
- Services: Grasp object, open gripper, home manipulator
- Actions: Pick and place, move to pose, execute trajectory

## Action Client Example

Here's a complete example of an action client that sends goals to the robot movement action server:

```python
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
```

## Summary

In this chapter, you learned about advanced communication patterns in ROS 2. Services provide synchronous request-response communication for quick operations, while actions enable complex, long-running tasks with feedback and cancellation capabilities. Understanding when to use each communication pattern is crucial for designing effective robot systems. The next chapter will cover robot description and visualization using URDF.

## Cross-References

- **Previous Chapter**: [Chapter 2: Speaking Robot (rclpy)](02-speaking-robot-rclpy.md) - Python-based robot communication
- **Next Chapter**: [Chapter 4: Anatomy of a Humanoid (URDF)](04-anatomy-urdf.md) - Robot description and visualization
- **Fundamental Concepts**: [Chapter 1: The ROS 2 Ecosystem](01-ros2-ecosystem.md) - Core ROS 2 architecture

## References and Further Reading

- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Tutorials/Services/Understanding-ROS2-Services.html)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Actions/Understanding-ROS2-Actions.html)
- [ROS 2 Action and Service Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Async-Callback-Group.html)
- [Creating Custom Message, Service and Action Files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 Action Client and Server Implementation Guide](https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html)
- [ROS 2 Communication Patterns Best Practices](https://docs.ros.org/en/humble/How-To-Guides/Node-design.html)