# Chapter 2: Speaking Robot (rclpy)

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, provide and use services, and work with actions. This chapter will guide you through the fundamentals of creating Python-based robot communication systems.

### Why Python for Robotics?

Python is an excellent choice for robotics development because of its:
- Simple and readable syntax
- Extensive library ecosystem
- Rapid prototyping capabilities
- Strong community support
- Integration with AI and machine learning frameworks

### Accessibility Considerations in Python Robotics

When developing with Python for robotics, consider these accessibility aspects:

- **Descriptive Variable Names**: Use meaningful names for nodes, topics, and parameters (e.g., `cmd_vel_publisher` instead of `pub1`)
- **Comprehensive Code Comments**: Explain complex logic and algorithms in your code
- **Structured Code Organization**: Use proper indentation and logical grouping of related functions
- **Error Messages**: Provide clear, informative error messages that help users understand what went wrong
- **Logging Levels**: Use appropriate logging levels (info, warn, error) to convey the importance of messages
- **Keyboard-Friendly Interfaces**: Ensure command-line tools can be operated without requiring mouse interaction
- **Text-Based Visualizations**: When possible, provide text-based alternatives to graphical representations

## Creating Your First Node with rclpy

A node is the fundamental building block of a ROS 2 system. Let's explore how to create a node using rclpy:

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my_first_node!')

def main(args=None):
    # Initialize ROS communications
    rclpy.init(args=args)

    # Create the node
    node = MyFirstNode()

    # Keep the node alive
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Lifecycle in rclpy

ROS 2 nodes have a well-defined lifecycle that allows for more robust robot applications:

### Lifecycle States
- **Unconfigured**: Initial state after node creation
- **Inactive**: After configuration, ready to be activated
- **Active**: Running and processing callbacks
- **Finalized**: Node is shut down and cleaned up

### Managing Node Lifecycle
```python
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleTestNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_test_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring lifecycle node...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating lifecycle node...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating lifecycle node...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up lifecycle node...')
        return TransitionCallbackReturn.SUCCESS
```

## Publishers in rclpy

Publishers allow nodes to send messages to topics. Here's how to create and use a publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create a publisher with a topic name and queue size
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create a timer to periodically publish messages
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscribers in rclpy

Subscribers receive messages from topics. Here's how to create and use a subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create a subscription with a topic name, message type, and callback
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription  # type: ignore

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting AI Logic to Robot Control

One of the key advantages of ROS 2 is the ability to connect high-level AI logic to robot control systems. Here's an example of how to do this:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # For robot movement commands
import numpy as np  # Example AI library

class AIBasedController(Node):
    def __init__(self):
        super().__init__('ai_controller')

        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for sensor data
        self.sensor_subscription = self.create_subscription(
            String,  # In practice, this would be a sensor message type
            '/sensor_data',
            self.sensor_callback,
            10)

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        self.latest_sensor_data = None

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.latest_sensor_data = msg.data
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def ai_processing_callback(self):
        """Run AI logic and generate robot commands"""
        if self.latest_sensor_data is not None:
            # Example AI processing
            cmd_vel = Twist()

            # Simple AI logic: if sensor detects obstacle, move away
            if 'obstacle' in self.latest_sensor_data.lower():
                cmd_vel.linear.x = -0.5  # Move backward
                cmd_vel.angular.z = 0.2  # Turn slightly
            else:
                cmd_vel.linear.x = 0.5   # Move forward
                cmd_vel.angular.z = 0.0  # Go straight

            # Publish the command
            self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = AIBasedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Python-based ROS 2 Development

### 1. Error Handling
Always include proper error handling in your nodes:

```python
def main(args=None):
    try:
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Keyboard interrupt received, shutting down')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
```

### 2. Parameter Management
Use ROS 2 parameters for configurable behavior:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.robot_name = self.get_parameter('robot_name').value
```

### 3. Logging
Use appropriate logging levels:

```python
# Informational messages
self.get_logger().info('Node initialized successfully')

# Warning messages
self.get_logger().warn('Sensor calibration may be needed')

# Error messages
self.get_logger().error('Failed to connect to sensor')
```

## Node Lifecycle Management

ROS 2 provides a lifecycle management system that allows for more robust robot applications. Lifecycle nodes go through well-defined states that can be managed programmatically.

### Lifecycle States

A lifecycle node transitions through the following states:

1. **Unconfigured (1)**: Initial state after node creation
2. **Inactive (2)**: After configuration, ready to be activated
3. **Active (3)**: Running and processing callbacks
4. **Finalized (4)**: Node is shut down and cleaned up

### Lifecycle Transitions

The possible transitions between states are:
- Unconfigured → Inactive: `configure()` transition
- Inactive → Active: `activate()` transition
- Active → Inactive: `deactivate()` transition
- Inactive → Unconfigured: `cleanup()` transition
- Inactive → Finalized: `shutdown()` transition
- Active → Finalized: `shutdown()` transition

### Implementing a Lifecycle Node

Here's how to create and use a lifecycle node:

```python
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

class LifecycleTestNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_test_node')
        self.get_logger().info('Lifecycle node created in unconfigured state')

    def on_configure(self, state):
        """Called when transitioning from unconfigured to inactive."""
        self.get_logger().info('Configuring lifecycle node...')

        # Create publishers, subscribers, timers, etc. here
        self.publisher = self.create_publisher(String, 'lifecycle_status', 10)

        # Return SUCCESS, FAILURE, or ERROR
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Called when transitioning from inactive to active."""
        self.get_logger().info('Activating lifecycle node...')

        # Activate any created entities
        self.publisher.on_activate()

        # Publish status
        msg = String()
        msg.data = 'Node activated'
        self.publisher.publish(msg)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Called when transitioning from active to inactive."""
        self.get_logger().info('Deactivating lifecycle node...')

        # Deactivate entities
        self.publisher.on_deactivate()

        msg = String()
        msg.data = 'Node deactivated'
        self.publisher.publish(msg)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Called when transitioning from inactive to unconfigured."""
        self.get_logger().info('Cleaning up lifecycle node...')

        # Destroy publishers, subscribers, timers, etc.
        self.destroy_publisher(self.publisher)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """Called when transitioning to finalized state."""
        self.get_logger().info('Shutting down lifecycle node...')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """Called when an error occurs during transition."""
        self.get_logger().error('Error occurred in lifecycle node!')
        return TransitionCallbackReturn.ERROR

def main(args=None):
    rclpy.init(args=args)

    # Create the lifecycle node
    node = LifecycleTestNode()

    # The node starts in the unconfigured state
    # You can then transition it through states:
    # node.configure()  # Transition to inactive
    # node.activate()   # Transition to active
    # node.deactivate() # Transition back to inactive
    # node.cleanup()    # Transition back to unconfigured
    # node.shutdown()   # Transition to finalized

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Managing Lifecycle from Command Line

You can manage lifecycle nodes from the command line:

```bash
# List lifecycle nodes
ros2 lifecycle list my_lifecycle_node

# Get current state
ros2 lifecycle get my_lifecycle_node

# Trigger transitions
ros2 lifecycle configure my_lifecycle_node
ros2 lifecycle activate my_lifecycle_node
ros2 lifecycle deactivate my_lifecycle_node
ros2 lifecycle cleanup my_lifecycle_node
ros2 lifecycle shutdown my_lifecycle_node
```

### When to Use Lifecycle Nodes

Lifecycle nodes are particularly useful for:

- Hardware interface nodes that need initialization
- Safety-critical systems that require controlled startup/shutdown
- Complex nodes with multiple initialization steps
- Nodes that need to be reconfigured at runtime
- Systems that require coordinated startup/shutdown sequences

## Hands-On Exercise: Connecting AI Logic to Robot Control Loops

### Exercise Goals:
- Create a robot controller that uses AI logic to navigate
- Implement a publisher for movement commands
- Create a subscriber for sensor data
- Add parameter configuration for AI behavior
- Connect AI decision-making to physical robot control

### Exercise Overview:
In this exercise, you'll build a complete AI-based robot controller that connects high-level decision making to low-level robot control. This demonstrates the bridge between AI logic and robot control loops.

### Steps:

1. **Create a sensor processing node**:
   - Subscribe to simulated laser scan data (`sensor_msgs/LaserScan`)
   - Process the data to identify obstacles and free space
   - Publish processed sensor information

2. **Implement AI decision-making**:
   - Create an AI node that subscribes to processed sensor data
   - Implement a simple navigation algorithm (e.g., wall following, obstacle avoidance)
   - Consider multiple factors like safety distance, efficiency, and goal direction

3. **Connect to robot control**:
   - Publish velocity commands (`geometry_msgs/Twist`) based on AI decisions
   - Implement safety checks before executing commands
   - Add feedback mechanisms to adjust behavior based on results

4. **Add parameter configuration**:
   - Set safety distance thresholds
   - Configure movement speeds
   - Adjust AI behavior parameters (e.g., aggression in obstacle avoidance)

5. **Test and validate**:
   - Test with various simulated environments
   - Verify safety constraints are respected
   - Evaluate navigation performance

### Sample Implementation Structure:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import math

class AINavigationNode(Node):
    def __init__(self):
        super().__init__('ai_navigation_node')

        # Declare parameters
        self.declare_parameter('safety_distance', 0.6)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_direction', 0.0)  # radians

        # Get parameter values
        self.safety_distance = self.get_parameter('safety_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_direction = self.get_parameter('goal_direction').value

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for laser scan data
        qos_profile = QoSProfile(depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.scan_callback, qos_profile)

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        # Store latest sensor data
        self.latest_scan = None
        self.get_logger().info('AI Navigation Node initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data."""
        self.latest_scan = msg

    def ai_processing_callback(self):
        """Main AI decision-making loop."""
        if self.latest_scan is None:
            return

        # Process laser scan to detect obstacles
        min_distance, direction_to_obstacle = self.find_nearest_obstacle()

        # Make navigation decision based on sensor data and goal
        cmd_vel = Twist()

        if min_distance < self.safety_distance:
            # Emergency maneuver: avoid obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.avoid_obstacle(direction_to_obstacle)
        else:
            # Normal navigation: move toward goal while avoiding distant obstacles
            cmd_vel.linear.x, cmd_vel.angular.z = self.navigate_toward_goal()

        # Apply speed limits
        cmd_vel.linear.x = max(-self.max_linear_speed,
                              min(cmd_vel.linear.x, self.max_linear_speed))
        cmd_vel.angular.z = max(-self.max_angular_speed,
                               min(cmd_vel.angular.z, self.max_angular_speed))

        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)

        # Log decision
        self.get_logger().info(
            f'AI Decision - Linear: {cmd_vel.linear.x:.2f}, '
            f'Angular: {cmd_vel.angular.z:.2f}, '
            f'Min distance: {min_distance:.2f}m'
        )

    def find_nearest_obstacle(self):
        """Find the nearest obstacle and its direction."""
        if not self.latest_scan:
            return float('inf'), 0.0

        # Convert index to angle: angle = start_angle + index * angle_increment
        min_dist = float('inf')
        min_idx = 0

        for i, distance in enumerate(self.latest_scan.ranges):
            if not (math.isnan(distance) or math.isinf(distance)) and distance < min_dist:
                min_dist = distance
                min_idx = i

        # Calculate angle to nearest obstacle
        angle_to_obstacle = (self.latest_scan.angle_min +
                           min_idx * self.latest_scan.angle_increment)

        return min_dist, angle_to_obstacle

    def avoid_obstacle(self, obstacle_direction):
        """Generate angular velocity to avoid obstacle."""
        # Turn away from obstacle (opposite direction)
        avoid_direction = obstacle_direction + math.pi
        # Normalize angle to [-π, π]
        avoid_direction = math.atan2(math.sin(avoid_direction),
                                   math.cos(avoid_direction))

        # Generate angular velocity proportional to direction error
        angular_vel = avoid_direction * 0.5  # Scale factor for responsiveness
        return angular_vel

    def navigate_toward_goal(self):
        """Generate linear and angular velocities toward goal."""
        # For simplicity, we'll use the stored goal direction
        # In a real system, this would be computed from current pose to goal
        desired_angular = self.goal_direction * 0.5  # P controller

        # Move forward at reduced speed when turning
        if abs(desired_angular) > 0.2:
            linear_speed = self.max_linear_speed * 0.5
        else:
            linear_speed = self.max_linear_speed

        return linear_speed, desired_angular

def main(args=None):
    rclpy.init(args=args)
    node = AINavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AI navigation node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise Enhancement Ideas:
- Implement a more sophisticated path planning algorithm (e.g., A* or Dijkstra)
- Add multiple sensor fusion (combine laser scan with camera data)
- Create a behavior tree for different navigation modes (explore, avoid, return to base)
- Add machine learning for adaptive behavior based on past experiences
- Implement state estimation using odometry and sensor data (localization)

### Validation:
Your implementation should successfully:
- Navigate around obstacles safely
- Show proper parameter configuration
- Handle sensor data appropriately
- Generate appropriate velocity commands
- Log AI decisions for debugging

## Advanced Publisher Example

Here's an advanced publisher that demonstrates multiple ROS 2 features including parameters, QoS profiles, and complex movement patterns:

```python
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
```

## Advanced Subscriber Example

Here's an advanced subscriber that demonstrates multiple subscriptions, QoS profiles, and message processing:

```python
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
        valid_ranges = [r for r in self.recent_ranges if r != float('inf') and not r != r]

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
```

## Custom Message Example

Here's an example of how to work with custom message types in ROS 2:

```python
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
```

## Best Practices for Python-based ROS 2 Development

When developing with rclpy, following best practices will help you create more robust, maintainable, and efficient robot applications:

### 1. Error Handling and Robustness
- Always implement proper exception handling, especially around network operations and sensor data processing
- Use try/finally blocks or context managers to ensure proper cleanup
- Implement timeouts for operations that could potentially block indefinitely

```python
def sensor_callback(self, msg):
    try:
        # Process sensor data
        processed_data = self.process_sensor_data(msg)
        self.publish_result(processed_data)
    except ValueError as e:
        self.get_logger().error(f'Sensor data processing error: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error in sensor callback: {e}')
```

### 2. Resource Management
- Always properly destroy ROS entities (publishers, subscribers, timers) when no longer needed
- Use appropriate QoS settings based on your application's requirements
- Be mindful of memory usage, especially when storing historical data

```python
def destroy_node(self):
    # Clean up all ROS entities before destroying the node
    self.destroy_publisher(self.publisher)
    self.destroy_subscription(self.subscriber)
    self.destroy_timer(self.timer)
    super().destroy_node()
```

### 3. Performance Optimization
- Use appropriate timer frequencies - don't update faster than necessary
- Consider using threading for CPU-intensive operations that shouldn't block the main ROS thread
- Use efficient data structures and algorithms for sensor processing

### 4. Parameter Validation
- Validate parameters at startup to catch configuration errors early
- Provide sensible defaults for all parameters
- Document parameter ranges and expected values

```python
def validate_parameters(self):
    safety_dist = self.get_parameter('safety_distance').value
    if safety_dist <= 0:
        self.get_logger().error('Safety distance must be positive')
        return False
    return True
```

### 5. Testing and Debugging
- Use logging extensively to track node behavior
- Implement unit tests for your robot logic
- Use appropriate log levels (info, warn, error) appropriately

### 6. Safety Considerations
- Implement safety checks before executing robot commands
- Use appropriate velocity and acceleration limits
- Include emergency stop functionality

## Summary

In this chapter, you learned how to use rclpy to create Python-based robot communication systems. You explored node creation, publishing and subscribing to topics, and how to connect AI logic to robot control systems. The next chapter will cover advanced communication patterns including services and actions.

## Cross-References

- **Previous Chapter**: [Chapter 1: The ROS 2 Ecosystem](01-ros2-ecosystem.md) - Fundamental concepts of ROS 2
- **Next Chapter**: [Chapter 3: Services & Actions](03-services-and-actions.md) - Advanced communication patterns in ROS 2
- **Practical Application**: [Chapter 4: Anatomy of a Humanoid (URDF)](04-anatomy-urdf.md) - Robot description and visualization

## References and Further Reading

- [ROS 2 Python Client Library (rclpy) Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Node Lifecycle Documentation](https://docs.ros.org/en/humble/Tutorials/Managed-Nodes.html)
- [ROS 2 Quality of Service Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS 2 Parameter System](https://docs.ros.org/en/humble/Tutorials/Parameters/Understanding-ROS2-Parameters.html)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Message and Service Definitions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)