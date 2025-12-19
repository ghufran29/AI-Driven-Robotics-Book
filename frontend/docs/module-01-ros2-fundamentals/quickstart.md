# Quickstart Guide: The Robotic Nervous System (ROS 2)

## Getting Started with ROS 2

This quickstart guide provides a rapid introduction to the core concepts covered in Module 1: The Robotic Nervous System (ROS 2). Follow these steps to quickly set up your ROS 2 environment and run your first robot communication examples.

## Prerequisites

Before starting, ensure you have:
- Ubuntu 22.04 LTS (or WSL2 with Ubuntu 22.04)
- ROS 2 Humble Hawksbill installed
- Basic Python 3.10+ knowledge
- Familiarity with terminal commands

### Installing ROS 2 Humble

If you haven't installed ROS 2 yet, follow these steps:

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-keyring.gpg | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release; echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list and install ROS 2 Humble Desktop
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-argcomplete
```

### Setting up the Environment

```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Install additional dependencies for Python development
sudo apt install -y python3-pip python3-colcon-common-extensions
pip3 install -U argcomplete
```

## Creating Your First ROS 2 Workspace

### 1. Create the Workspace

```bash
# Create a new ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build
```

### 2. Source the Workspace

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Add to your bashrc to source automatically
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Running Basic Publisher-Subscriber Examples

### 1. Create a New Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python hello_robot_py --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

### 2. Create a Publisher Node

Create the publisher file `~/ros2_ws/src/hello_robot_py/hello_robot_py/basic_publisher.py`:

```python
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
```

### 3. Create a Subscriber Node

Create the subscriber file `~/ros2_ws/src/hello_robot_py/hello_robot_py/basic_subscriber.py`:

```python
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
```

### 4. Update the Package Setup

Update the setup.py file in `~/ros2_ws/src/hello_robot_py/setup.py`:

```python
from setuptools import setup

package_name = 'hello_robot_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Basic ROS 2 publisher and subscriber examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_publisher = hello_robot_py.basic_publisher:main',
            'basic_subscriber = hello_robot_py.basic_subscriber:main',
        ],
    },
)
```

### 5. Build and Run the Examples

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Build your package
colcon build --packages-select hello_robot_py

# Source the newly built package
source install/setup.bash

# In one terminal, run the publisher:
ros2 run hello_robot_py basic_publisher

# In another terminal, run the subscriber:
ros2 run hello_robot_py basic_subscriber
```

## Working with Services

### 1. Create a Service Server

Create `~/ros2_ws/src/hello_robot_py/hello_robot_py/simple_service_server.py`:

```python
#!/usr/bin/env python3

"""
Simple ROS 2 Service Server Example

This example demonstrates how to create a service server in ROS 2 using rclpy.
The service adds two integers together.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):
    """
    A simple service server that adds two integers.
    """

    def __init__(self):
        super().__init__('simple_service_server')

        # Create a service with the AddTwoInts interface
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        """
        Callback function that processes the service request.
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)

    simple_service_server = SimpleServiceServer()

    try:
        rclpy.spin(simple_service_server)
    except KeyboardInterrupt:
        simple_service_server.get_logger().info('Shutting down service server...')
    finally:
        simple_service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Create a Service Client

Create `~/ros2_ws/src/hello_robot_py/hello_robot_py/simple_service_client.py`:

```python
#!/usr/bin/env python3

"""
Simple ROS 2 Service Client Example

This example demonstrates how to create a service client in ROS 2 using rclpy.
The client calls the AddTwoInts service to add two numbers.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceClient(Node):
    """
    A simple service client that calls the AddTwoInts service.
    """

    def __init__(self):
        super().__init__('simple_service_client')

        # Create a client for the AddTwoInts service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service.
        """
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.client.call_async(self.request)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    # Parse command line arguments
    if len(sys.argv) != 3:
        print('Usage: ros2 run hello_robot_py simple_service_client <int1> <int2>')
        print('Using default values: 1 2')
        a, b = 1, 2
    else:
        a, b = int(sys.argv[1]), int(sys.argv[2])

    simple_service_client = SimpleServiceClient()

    # Send the request
    future = simple_service_client.send_request(a, b)

    try:
        # Wait for the result
        rclpy.spin_until_future_complete(simple_service_client, future)

        if future.result() is not None:
            response = future.result()
            simple_service_client.get_logger().info(
                f'Result of {a} + {b} = {response.sum}'
            )
        else:
            simple_service_client.get_logger().error(
                f'Exception while calling service: {future.exception()}'
            )
    except KeyboardInterrupt:
        simple_service_client.get_logger().info('Client interrupted by user')
    finally:
        simple_service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Run the Service Example

```bash
# Terminal 1: Run the service server
ros2 run hello_robot_py simple_service_server

# Terminal 2: Call the service with different numbers
ros2 run hello_robot_py simple_service_client 5 3
ros2 run hello_robot_py simple_service_client 10 -2
```

## Working with Actions

### 1. Create an Action Server

Create `~/ros2_ws/src/hello_robot_py/hello_robot_py/simple_action_server.py`:

```python
#!/usr/bin/env python3

"""
Simple ROS 2 Action Server Example

This example demonstrates how to create an action server in ROS 2 using rclpy.
The action simulates a task that takes time and provides feedback.
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class SimpleActionServer(Node):
    """
    A simple action server that generates a Fibonacci sequence.
    """

    def __init__(self):
        super().__init__('simple_action_server')

        # Create an action server for the Fibonacci action
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        """
        Execute the action goal and provide feedback.
        """
        self.get_logger().info('Executing goal...')

        # Create result message
        result = Fibonacci.Result()
        result.sequence = [0]

        # Create feedback message
        feedback = Fibonacci.Feedback()

        # Generate Fibonacci sequence with feedback
        if goal_handle.request.order > 0:
            result.sequence = [0, 1]

            for i in range(1, goal_handle.request.order):
                # Check if the goal was canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.sequence = feedback.sequence
                    self.get_logger().info('Goal canceled')
                    return result

                # Publish feedback
                feedback.sequence = result.sequence[:]
                goal_handle.publish_feedback(feedback)

                # Calculate next Fibonacci number
                next_fib = result.sequence[i] + result.sequence[i-1]
                result.sequence.append(next_fib)

                # Simulate processing time
                time.sleep(0.5)

        # Complete the goal
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)

    simple_action_server = SimpleActionServer()

    try:
        rclpy.spin(simple_action_server)
    except KeyboardInterrupt:
        simple_action_server.get_logger().info('Shutting down action server...')
    finally:
        simple_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Create an Action Client

Create `~/ros2_ws/src/hello_robot_py/hello_robot_py/simple_action_client.py`:

```python
#!/usr/bin/env python3

"""
Simple ROS 2 Action Client Example

This example demonstrates how to create an action client in ROS 2 using rclpy.
The client sends a goal to the Fibonacci action server.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class SimpleActionClient(Node):
    """
    A simple action client that sends goals to the Fibonacci action server.
    """

    def __init__(self):
        super().__init__('simple_action_client')

        # Create an action client for the Fibonacci action
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        """
        Send a goal to the action server.
        """
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send the goal and register callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the final result.
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback messages.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)

    simple_action_client = SimpleActionClient()

    # Send a goal to generate a Fibonacci sequence of order 5
    simple_action_client.send_goal(5)

    try:
        rclpy.spin(simple_action_client)
    except KeyboardInterrupt:
        simple_action_client.get_logger().info('Client interrupted by user')
    finally:
        simple_action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Run the Action Example

```bash
# Terminal 1: Run the action server
ros2 run hello_robot_py simple_action_server

# Terminal 2: Send a goal to the action server
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
```

## Visualizing Robot Models with URDF

### 1. Create a Simple Robot Model

Create `~/ros2_ws/src/hello_robot_py/config/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0225" ixy="0.0" ixz="0.0" iyy="0.0225" iyz="0.0" izz="0.0225"/>
    </inertial>
  </link>

  <!-- Joint connecting base to head -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00292" ixy="0.0" ixz="0.0" iyy="0.00292" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00292" ixy="0.0" ixz="0.0" iyy="0.00292" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Joints connecting base to wheels -->
  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.25 -0.1" rpy="-1.57079632679 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.25 -0.1" rpy="-1.57079632679 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### 2. Visualize the Robot Model

```bash
# Set the robot description parameter
export ROBOT_DESCRIPTION="$(cat ~/ros2_ws/src/hello_robot_py/config/simple_robot.urdf)"

# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$ROBOT_DESCRIPTION"

# In another terminal, launch RViz2
rviz2
```

In RViz2:
1. Add a "RobotModel" display
2. Set "Robot Description" to "/robot_description"
3. You should see your robot model displayed

## Understanding TF (Transforms)

### 1. Create a TF Publisher Node

Create `~/ros2_ws/src/hello_robot_py/hello_robot_py/tf_publisher.py`:

```python
#!/usr/bin/env python3

"""
TF Publisher Example

This example demonstrates how to publish transforms in ROS 2 using tf2.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class TFPublisher(Node):
    """
    A node that publishes transforms between coordinate frames.
    """

    def __init__(self):
        super().__init__('tf_publisher')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to periodically publish transforms
        self.timer = self.create_timer(0.1, self.broadcast_transform)

        self.time = 0

    def broadcast_transform(self):
        """
        Broadcast transforms between coordinate frames.
        """
        # Current time
        current_time = self.get_clock().now()

        # Create a transform from base_link to laser_frame
        t = TransformStamped()

        # Set the header
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        # Set the transform (position and orientation)
        # Make the laser frame rotate around the base
        self.time += 0.1
        radius = 0.3  # 30 cm from base

        t.transform.translation.x = radius * math.cos(self.time)
        t.transform.translation.y = radius * math.sin(self.time)
        t.transform.translation.z = 0.2  # 20 cm above base

        # For rotation, we'll set a simple rotation around Z-axis
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.time / 2.0)
        t.transform.rotation.w = math.cos(self.time / 2.0)

        # Send the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    tf_publisher = TFPublisher()

    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        tf_publisher.get_logger().info('Shutting down TF publisher...')
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Run the TF Example

```bash
# Terminal 1: Run the TF publisher
ros2 run hello_robot_py tf_publisher

# Terminal 2: Check the TF tree
ros2 run tf2_tools view_frames

# Terminal 3: Echo a specific transform
ros2 run tf2_ros tf2_echo base_link laser_frame
```

## Summary of Commands

Here's a summary of the key commands you've learned:

1. **Environment Setup**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Package Creation**:
   ```bash
   ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs
   ```

3. **Building Packages**:
   ```bash
   colcon build --packages-select <package_name>
   ```

4. **Running Nodes**:
   ```bash
   ros2 run <package_name> <executable_name>
   ```

5. **Checking System State**:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 service list
   ros2 action list
   ```

6. **Visualizing**:
   ```bash
   rviz2
   ros2 run rqt_graph rqt_graph
   ```

## Next Steps

After completing this quickstart guide, you should:

1. **Explore More Complex Examples**: Look at the detailed chapters in this module for advanced topics
2. **Practice with Real Hardware**: If you have access to a physical robot, try deploying your nodes
3. **Learn About Navigation**: Study the ROS 2 Navigation2 stack for autonomous navigation
4. **Integrate AI/ML**: Connect your robot to AI models for perception and decision making
5. **Join the Community**: Participate in ROS discourse, answers.ros.org, and local ROS meetups

## Troubleshooting Common Issues

### 1. Package Not Found
- Ensure you've sourced your workspace: `source ~/ros2_ws/install/setup.bash`
- Check that the package was built successfully: `colcon build`

### 2. Permission Errors
- Make sure your Python files have execute permissions: `chmod +x script.py`
- Check that you're running commands with appropriate permissions

### 3. Network Communication Issues
- Verify that ROS_DOMAIN_ID is the same on all machines: `echo $ROS_DOMAIN_ID`
- Check that firewalls aren't blocking ROS 2 traffic

### 4. Missing Dependencies
- Install missing packages with: `sudo apt install ros-humble-<package-name>`
- Use rosdep to install dependencies: `rosdep install --from-paths src --ignore-src -r -y`

This quickstart guide provides the foundation for working with ROS 2. The examples here correspond to the detailed concepts covered in the full curriculum modules.