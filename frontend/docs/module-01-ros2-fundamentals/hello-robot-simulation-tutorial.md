# Hello Robot Simulation Tutorial

## Overview

This tutorial will guide you through creating and running a complete robot simulation that demonstrates all core ROS 2 concepts. You'll build a robot with multiple sensors, actuators, and communication patterns working together in a unified system.

## Prerequisites

Before starting this tutorial, ensure you have:

1. ROS 2 Humble Hawksbill installed on Ubuntu 22.04
2. Basic understanding of Python programming
3. Completed Modules 1-3 of this curriculum
4. Basic familiarity with terminal commands

## Learning Objectives

After completing this tutorial, you will be able to:

- Create a complete robot model using URDF
- Implement all three ROS 2 communication patterns (topics, services, actions)
- Visualize your robot in RViz2
- Simulate sensor data and robot movement
- Understand how all ROS 2 concepts work together in a unified system

## Step 1: Create the Robot URDF Model

First, let's create a comprehensive robot model that includes all the components we've learned about:

1. Create a directory for your robot model:
   ```bash
   mkdir -p ~/ros2_ws/src/hello_robot_description/urdf
   ```

2. Create the main URDF file `~/ros2_ws/src/hello_robot_description/urdf/hello_robot.urdf`:
   ```xml
   <?xml version="1.0"?>
   <robot name="hello_robot">
     <!-- Base link - the main body of the robot -->
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder radius="0.3" length="0.2"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 1"/>
         </material>
       </visual>

       <collision>
         <geometry>
           <cylinder radius="0.3" length="0.2"/>
         </geometry>
       </collision>

       <inertial>
         <mass value="10.0"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.6" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.9"/>
       </inertial>
     </link>

     <!-- Head link -->
     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
         <material name="white">
           <color rgba="1.0 1.0 1.0 1.0"/>
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

     <!-- Neck joint connecting base to head -->
     <joint name="neck_joint" type="revolute">
       <parent link="base_link"/>
       <child link="head"/>
       <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-0.785" upper="0.785" effort="10.0" velocity="1.0"/>
       <dynamics damping="0.1" friction="0.0"/>
     </joint>

     <!-- Left wheel -->
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder length="0.05" radius="0.1"/>
         </geometry>
         <material name="black">
           <color rgba="0.0 0.0 0.0 1.0"/>
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
           <color rgba="0.0 0.0 0.0 1.0"/>
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
       <origin xyz="0.0 0.25 -0.1" rpy="-1.57079632679 0.0 0.0"/>
       <axis xyz="0 0 1"/>
       <limit effort="10.0" velocity="1.0"/>
     </joint>

     <joint name="base_to_right_wheel" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0.0 -0.25 -0.1" rpy="-1.57079632679 0.0 0.0"/>
       <axis xyz="0 0 1"/>
       <limit effort="10.0" velocity="1.0"/>
     </joint>

     <!-- Front caster wheel -->
     <link name="caster_wheel">
       <visual>
         <geometry>
           <sphere radius="0.05"/>
         </geometry>
         <material name="black">
           <color rgba="0.0 0.0 0.0 1.0"/>
         </material>
       </visual>

       <collision>
         <geometry>
           <sphere radius="0.05"/>
         </geometry>
       </collision>

       <inertial>
         <mass value="0.1"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
       </inertial>
     </link>

     <joint name="base_to_caster" type="fixed">
       <parent link="base_link"/>
       <child link="caster_wheel"/>
       <origin xyz="0.2 0.0 -0.15" rpy="0 0 0"/>
     </joint>

     <!-- Sensor mast -->
     <link name="sensor_mast">
       <visual>
         <geometry>
           <cylinder radius="0.02" length="0.4"/>
         </geometry>
         <material name="green">
           <color rgba="0.0 1.0 0.0 1.0"/>
         </material>
       </visual>

       <collision>
         <geometry>
           <cylinder radius="0.02" length="0.4"/>
         </geometry>
       </collision>

       <inertial>
         <mass value="0.2"/>
         <origin xyz="0 0 0.2"/>
         <inertia ixx="0.0027" ixy="0.0" ixz="0.0" iyy="0.0027" iyz="0.0" izz="0.0001"/>
       </inertial>
     </link>

     <joint name="mast_joint" type="fixed">
       <parent link="base_link"/>
       <child link="sensor_mast"/>
       <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
     </joint>

     <!-- Camera -->
     <link name="camera">
       <visual>
         <geometry>
           <box size="0.05 0.08 0.05"/>
         </geometry>
         <material name="black">
           <color rgba="0.0 0.0 0.0 1.0"/>
         </material>
       </visual>

       <collision>
         <geometry>
           <box size="0.05 0.08 0.05"/>
         </geometry>
       </collision>

       <inertial>
         <mass value="0.05"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.00005" ixy="0.0" ixz="0.0" iyy="0.00007" iyz="0.0" izz="0.00005"/>
       </inertial>
     </link>

     <joint name="camera_joint" type="fixed">
       <parent link="sensor_mast"/>
       <child link="camera"/>
       <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
     </joint>

     <!-- Laser scanner -->
     <link name="laser_scanner">
       <visual>
         <geometry>
           <cylinder radius="0.03" length="0.05"/>
         </geometry>
         <material name="yellow">
           <color rgba="1.0 1.0 0.0 1.0"/>
         </material>
       </visual>

       <collision>
         <geometry>
           <cylinder radius="0.03" length="0.05"/>
         </geometry>
       </collision>

       <inertial>
         <mass value="0.1"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
       </inertial>
     </link>

     <joint name="laser_joint" type="fixed">
       <parent link="base_link"/>
       <child link="laser_scanner"/>
       <origin xyz="0.25 0.0 0.05" rpy="0 0 0"/>
     </joint>
   </robot>
   ```

## Step 2: Create the Robot Controller Node

Now let's create a comprehensive Python node that demonstrates all communication patterns:

1. Create the Python file `~/ros2_ws/src/hello_robot_controller/hello_robot_controller/hello_robot_node.py`:
   ```python
   #!/usr/bin/env python3

   """
   Hello Robot Controller - Comprehensive ROS 2 Robot Node

   This node demonstrates all core ROS 2 concepts including:
   - Publisher-Subscriber communication (topics)
   - Service-based communication (services)
   - Goal-oriented communication (actions)
   - Robot state management
   - TF transforms
   - URDF model integration
   """

   import rclpy
   from rclpy.node import Node
   from rclpy.action import ActionServer, GoalResponse, CancelResponse
   from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

   from std_msgs.msg import String, Bool, Float32
   from geometry_msgs.msg import Twist, Vector3, TransformStamped
   from sensor_msgs.msg import LaserScan, JointState
   from tf2_ros import TransformBroadcaster
   from builtin_interfaces.msg import Time

   from example_interfaces.srv import SetBool
   from example_interfaces.action import Fibonacci

   import time
   import math
   import threading
   from collections import deque


   class HelloRobotNode(Node):
       """
       A comprehensive robot node that demonstrates all core ROS 2 concepts.
       """

       def __init__(self):
           super().__init__('hello_robot_node')

           # Robot state variables
           self.robot_x = 0.0
           self.robot_y = 0.0
           self.robot_theta = 0.0  # Heading angle in radians
           self.robot_enabled = True
           self.joint_positions = {'neck_joint': 0.0, 'left_wheel_joint': 0.0, 'right_wheel_joint': 0.0}

           # Initialize command velocity
           self.cmd_vel = Twist()

           # QoS profile for reliable communication
           qos_profile = QoSProfile(
               depth=10,
               history=QoSHistoryPolicy.KEEP_LAST,
               reliability=QoSReliabilityPolicy.RELIABLE,
               durability=QoSDurabilityPolicy.VOLATILE
           )

           # ===== TOPICS (Publisher-Subscriber) =====

           # Publisher for robot velocity commands
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

           # Publisher for robot state
           self.state_pub = self.create_publisher(String, 'robot_state', qos_profile)

           # Publisher for sensor data
           self.laser_scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile)
           self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

           # Subscriber for velocity commands
           self.cmd_vel_sub = self.create_subscription(
               Twist,
               'cmd_vel',
               self.cmd_vel_callback,
               qos_profile
           )

           # Subscriber for robot enable/disable commands
           self.enable_sub = self.create_subscription(
               Bool,
               'robot_enable',
               self.enable_callback,
               qos_profile
           )

           # Create transform broadcaster for TF
           self.tf_broadcaster = TransformBroadcaster(self)

           # ===== SERVICES =====

           # Service to enable/disable the robot
           self.enable_service = self.create_service(
               SetBool,
               'enable_robot',
               self.enable_service_callback
           )

           # ===== ACTIONS =====

           # Action server for navigation
           self.nav_action_server = ActionServer(
               self,
               Fibonacci,  # Using standard action for example
               'fibonacci_action',  # Changed to more appropriate name
               execute_callback=self.navigate_execute_callback,
               goal_callback=self.navigate_goal_callback,
               cancel_callback=self.navigate_cancel_callback
           )

           # Timer for periodic updates
           self.update_timer = self.create_timer(0.1, self.update_robot_state)

           # Timer for publishing sensor data
           self.sensor_timer = self.create_timer(0.05, self.publish_sensor_data)

           # Timer for publishing TF transforms
           self.tf_timer = self.create_timer(0.05, self.publish_transforms)

           self.get_logger().info('Hello Robot Node initialized with all communication patterns')

       def cmd_vel_callback(self, msg):
           """
           Handle incoming velocity commands.
           """
           self.cmd_vel = msg
           self.get_logger().info(
               f'Received velocity command: linear.x={msg.linear.x}, '
               f'angular.z={msg.angular.z}'
           )

       def enable_callback(self, msg):
           """
           Handle robot enable/disable commands via topic.
           """
           self.robot_enabled = msg.data
           self.get_logger().info(f'Robot {"enabled" if self.robot_enabled else "disabled"} via topic')

       def enable_service_callback(self, request, response):
           """
           Handle robot enable/disable requests via service.
           """
           self.robot_enabled = request.data
           response.success = True
           response.message = f'Robot {"enabled" if self.robot_enabled else "disabled"} via service'
           self.get_logger().info(response.message)
           return response

       def navigate_goal_callback(self, goal_request):
           """
           Accept or reject navigation goals.
           """
           self.get_logger().info('Received fibonacci goal request')
           # Accept all goals for this example
           return GoalResponse.ACCEPT

       def navigate_cancel_callback(self, goal_handle):
           """
           Accept or reject navigation cancel requests.
           """
           self.get_logger().info('Received fibonacci cancel request')
           # Accept all cancel requests for this example
           return CancelResponse.ACCEPT

       def navigate_execute_callback(self, goal_handle):
           """
           Execute fibonacci goal and provide feedback.
           """
           self.get_logger().info('Executing fibonacci goal...')

           # Get the goal
           order = goal_handle.request.order

           # Create result message
           result = Fibonacci.Result()
           result.sequence = [0]

           # Create feedback message
           feedback_msg = Fibonacci.Feedback()

           # Generate fibonacci sequence with feedback
           if order == 0:
               result.sequence = [0]
           elif order == 1:
               result.sequence = [0, 1]
           else:
               result.sequence = [0, 1]
               for i in range(1, order):
                   if goal_handle.is_cancel_requested:
                       result.sequence = feedback_msg.sequence
                       goal_handle.canceled()
                       self.get_logger().info('Fibonacci goal canceled')
                       return result

                   # Calculate next fibonacci number
                   next_fib = result.sequence[i] + result.sequence[i-1]
                   result.sequence.append(next_fib)

                   # Publish feedback
                   feedback_msg.sequence = result.sequence[:]
                   goal_handle.publish_feedback(feedback_msg)

                   self.get_logger().info(f'Fibonacci feedback: {feedback_msg.sequence[-1]}')

                   # Simulate processing time
                   time.sleep(0.5)

           # Complete the goal
           goal_handle.succeed()
           self.get_logger().info('Fibonacci goal succeeded')

           return result

       def update_robot_state(self):
           """
           Update robot state based on velocity commands (simple simulation).
           """
           if not self.robot_enabled:
               self.cmd_vel.linear.x = 0.0
               self.cmd_vel.angular.z = 0.0

           # Simple kinematic model for differential drive
           dt = 0.1  # Time step (matches timer interval)

           # Update position based on velocity
           self.robot_x += self.cmd_vel.linear.x * math.cos(self.robot_theta) * dt
           self.robot_y += self.cmd_vel.linear.x * math.sin(self.robot_theta) * dt
           self.robot_theta += self.cmd_vel.angular.z * dt

           # Normalize theta to [-pi, pi]
           self.robot_theta = math.atan2(math.sin(self.robot_theta), math.cos(self.robot_theta))

           # Update joint positions (simple oscillation for demonstration)
           current_time = self.get_clock().now().nanoseconds / 1e9
           self.joint_positions['neck_joint'] = 0.5 * math.sin(current_time)
           self.joint_positions['left_wheel_joint'] = current_time * 2  # Continuous rotation
           self.joint_positions['right_wheel_joint'] = current_time * 2  # Continuous rotation

           # Publish robot state
           state_msg = String()
           state_msg.data = f'x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}, enabled={self.robot_enabled}'
           self.state_pub.publish(state_msg)

       def publish_sensor_data(self):
           """
           Publish simulated sensor data.
           """
           # Publish joint states
           joint_state_msg = JointState()
           joint_state_msg.header.stamp = self.get_clock().now().to_msg()
           joint_state_msg.header.frame_id = 'base_link'
           joint_state_msg.name = list(self.joint_positions.keys())
           joint_state_msg.position = list(self.joint_positions.values())
           self.joint_state_pub.publish(joint_state_msg)

           # Publish simulated laser scan
           scan_msg = LaserScan()
           scan_msg.header.stamp = self.get_clock().now().to_msg()
           scan_msg.header.frame_id = 'laser_scanner'

           # Laser scan parameters
```python
           scan_msg.angle_min = -math.pi / 2
           scan_msg.angle_max = math.pi / 2
           scan_msg.angle_increment = math.pi / 180  # 1 degree increments
           scan_msg.time_increment = 0.0
           scan_msg.scan_time = 0.1
           scan_msg.range_min = 0.1
           scan_msg.range_max = 10.0

           # Generate simulated ranges (with some obstacles)
           num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
           ranges = []

           current_time = self.get_clock().now().nanoseconds / 1e9
           for i in range(num_readings):
               angle = scan_msg.angle_min + i * scan_msg.angle_increment

               # Simulate some obstacles at specific angles
               distance = 5.0  # Default range

               # Add some simulated obstacles
               if abs(angle) < 0.2:
                   distance = 1.0 + 0.5 * math.sin(current_time * 2)  # Close obstacle ahead
               elif abs(angle - 0.5) < 0.1:
                   distance = 2.0  # Obstacle to the right
               elif abs(angle + 0.5) < 0.1:
                   distance = 2.5  # Obstacle to the left
               else:
                   distance = 3.0 + 1.0 * math.sin(current_time + angle)  # Varying background

               ranges.append(distance)

           scan_msg.ranges = ranges
           scan_msg.intensities = []  # No intensity data

           self.laser_scan_pub.publish(scan_msg)
```

       def publish_transforms(self):
           """
           Publish TF transforms for the robot.
           """
           # Robot base transform (simulated movement)
           t = TransformStamped()
           t.header.stamp = self.get_clock().now().to_msg()
           t.header.frame_id = 'odom'
           t.child_frame_id = 'base_link'

           t.transform.translation.x = self.robot_x
           t.transform.translation.y = self.robot_y
           t.transform.translation.z = 0.0

           # Convert orientation from euler to quaternion
           cy = math.cos(self.robot_theta * 0.5)
           sy = math.sin(self.robot_theta * 0.5)
           t.transform.rotation.w = cy
           t.transform.rotation.x = 0.0
           t.transform.rotation.y = 0.0
           t.transform.rotation.z = sy

           self.tf_broadcaster.sendTransform(t)

           # Head transform (relative to base)
           t_head = TransformStamped()
           t_head.header.stamp = self.get_clock().now().to_msg()
           t_head.header.frame_id = 'base_link'
           t_head.child_frame_id = 'head'
           t_head.transform.translation.x = 0.0
           t_head.transform.translation.y = 0.0
           t_head.transform.translation.z = 0.2
           t_head.transform.rotation.w = 1.0
           t_head.transform.rotation.x = 0.0
           t_head.transform.rotation.y = 0.0
           t_head.transform.rotation.z = 0.0

           self.tf_broadcaster.sendTransform(t_head)

           # Camera transform (relative to sensor mast)
           t_camera = TransformStamped()
           t_camera.header.stamp = self.get_clock().now().to_msg()
           t_camera.header.frame_id = 'sensor_mast'
           t_camera.child_frame_id = 'camera'
           t_camera.transform.translation.x = 0.0
           t_camera.transform.translation.y = 0.0
           t_camera.transform.translation.z = 0.4
           t_camera.transform.rotation.w = 1.0
           t_camera.transform.rotation.x = 0.0
           t_camera.transform.rotation.y = 0.0
           t_camera.transform.rotation.z = 0.0

           self.tf_broadcaster.sendTransform(t_camera)

           # Laser scanner transform (relative to base)
           t_laser = TransformStamped()
           t_laser.header.stamp = self.get_clock().now().to_msg()
           t_laser.header.frame_id = 'base_link'
           t_laser.child_frame_id = 'laser_scanner'
           t_laser.transform.translation.x = 0.25
           t_laser.transform.translation.y = 0.0
           t_laser.transform.translation.z = 0.05
           t_laser.transform.rotation.w = 1.0
           t_laser.transform.rotation.x = 0.0
           t_laser.transform.rotation.y = 0.0
           t_laser.transform.rotation.z = 0.0

           self.tf_broadcaster.sendTransform(t_laser)


   def main(args=None):
       """
       Main function to run the Hello Robot node.
       """
       rclpy.init(args=args)

       hello_robot = HelloRobotNode()

       try:
           rclpy.spin(hello_robot)
       except KeyboardInterrupt:
           hello_robot.get_logger().info('Shutting down Hello Robot Node...')
       finally:
           hello_robot.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

## Step 3: Create Package Files

1. Create the package.xml file `~/ros2_ws/src/hello_robot_controller/package.xml`:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>hello_robot_controller</name>
     <version>0.0.1</version>
     <description>Hello Robot Controller Package - Demonstrates all core ROS 2 concepts</description>
     <maintainer email="student@todo.ros">Student</maintainer>
     <license>Apache-2.0</license>

     <depend>rclpy</depend>
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <depend>sensor_msgs</depend>
     <depend>tf2_ros</depend>
     <depend>example_interfaces</depend>

     <exec_depend>ros2launch</exec_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

2. Create the setup.py file `~/ros2_ws/src/hello_robot_controller/setup.py`:
   ```python
   from setuptools import setup

   package_name = 'hello_robot_controller'

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
       maintainer='student',
       maintainer_email='student@todo.ros',
       description='Hello Robot Controller Package - Demonstrates all core ROS 2 concepts',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'hello_robot_node = hello_robot_controller.hello_robot_node:main',
           ],
       },
   )
   ```

## Step 4: Create Launch File

1. Create the launch directory and file:
   ```bash
   mkdir -p ~/ros2_ws/src/hello_robot_controller/launch
   ```

2. Create `~/ros2_ws/src/hello_robot_controller/launch/hello_robot.launch.py`:
   ```python
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
       return LaunchDescription([
           # Declare launch arguments
           DeclareLaunchArgument(
               'publish_frequency',
               default_value='2',
               description='Frequency of publishing messages'),

           DeclareLaunchArgument(
               'robot_name',
               default_value='hello_robot',
               description='Name of the robot'),

           # Robot state publisher node
           Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               parameters=[{
                   'robot_description': open(
                       '~/ros2_ws/src/hello_robot_description/urdf/hello_robot.urdf', 'r').read(),
                   'publish_frequency': LaunchConfiguration('publish_frequency'),
               }],
               arguments=['--ros-args', '--log-level', 'info']
           ),

           # Hello Robot Controller node
           Node(
               package='hello_robot_controller',
               executable='hello_robot_node',
               name='hello_robot_node',
               parameters=[
                   {'publish_frequency': LaunchConfiguration('publish_frequency')},
                   {'robot_name': LaunchConfiguration('robot_name')}
               ],
               arguments=['--ros-args', '--log-level', 'info']
           ),
       ])
   ```

## Step 5: Build and Run the Simulation

1. Build your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select hello_robot_description hello_robot_controller
   source install/setup.bash
   ```

2. Launch the robot simulation:
   ```bash
   ros2 launch hello_robot_controller hello_robot.launch.py
   ```

3. In a separate terminal, launch RViz2:
   ```bash
   rviz2
   ```

4. In RViz2, add the following displays:
   - RobotModel: Set "Robot Description" to "/robot_description"
   - TF: Shows the transform tree
   - LaserScan: Set "Topic" to "/scan"

5. To send velocity commands to the robot, use:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```

6. To test the service, use:
   ```bash
   ros2 service call /enable_robot example_interfaces/srv/SetBool '{data: true}'
   ```

7. To test the action, use:
   ```bash
   ros2 action send_goal /fibonacci_action example_interfaces/action/Fibonacci '{order: 5}'
   ```

## Step 6: Understanding the Integration

This complete simulation demonstrates how all ROS 2 concepts work together:

1. **Topics (Publish-Subscribe)**: Used for continuous communication like sensor data, velocity commands, and robot state
2. **Services (Request-Response)**: Used for discrete commands like enabling/disabling the robot
3. **Actions (Goal-Oriented)**: Used for long-running tasks like navigation goals
4. **TF (Transforms)**: Used to understand spatial relationships between robot components
5. **URDF (Robot Description)**: Used to define the robot's physical structure

## Troubleshooting

1. **Robot not appearing in RViz2**:
   - Verify that robot_state_publisher is running and publishing transforms
   - Check that the URDF file path is correct
   - Ensure all required packages are installed

2. **Nodes not communicating**:
   - Verify that all nodes are on the same ROS domain
   - Check that topic and service names match between publishers and subscribers
   - Ensure the correct message types are being used

3. **Performance issues**:
   - Reduce the frequency of high-bandwidth topics like laser scans
   - Optimize the complexity of visual meshes in URDF
   - Check for unnecessary repeated calculations in callbacks

## Next Steps

After completing this tutorial, you should:
1. Experiment with different movement patterns
2. Add more complex sensor models to your robot
3. Implement more sophisticated AI logic in your controller
4. Explore advanced features like navigation and manipulation

## Validation Steps

To verify that your complete "Hello Robot" simulation is working correctly, follow these validation steps:

### 1. URDF Model Validation

1. **Check URDF Syntax**:
   ```bash
   # Validate the URDF file syntax
   check_urdf ~/ros2_ws/src/hello_robot_description/urdf/hello_robot.urdf
   ```

2. **Verify Joint Limits and Ranges**:
   ```bash
   # Parse the URDF and check joint specifications
   ros2 run urdf_parser joint_limits_urdf hello_robot.urdf
   ```

3. **Visualize in RViz2**:
   - Launch RViz2: `rviz2`
   - Add a RobotModel display
   - Set "Robot Description" to `/robot_description`
   - Verify all links appear with correct colors and positions
   - Check that joint axes are displayed correctly

### 2. Node Communication Validation

1. **Check Active Nodes**:
   ```bash
   # List all active nodes
   ros2 node list
   # Should show your robot nodes (e.g., hello_robot_node, robot_state_publisher)
   ```

2. **Verify Topic Connections**:
   ```bash
   # List all active topics
   ros2 topic list
   # Should show topics like /cmd_vel, /scan, /joint_states, etc.

   # Check topic connections
   ros2 topic info /cmd_vel
   # Should show publishers and subscribers
   ```

3. **Test Topic Communication**:
   ```bash
   # Echo the robot state topic to verify it's publishing
   ros2 topic echo /robot_state

   # Publish a velocity command and observe response
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```

### 3. Service Validation

1. **Check Available Services**:
   ```bash
   # List all services
   ros2 service list
   # Should include your services like /enable_robot
   ```

2. **Test Service Calls**:
   ```bash
   # Call the enable robot service
   ros2 service call /enable_robot example_interfaces/srv/SetBool '{data: true}'
   # Should return success: true and a message
   ```

### 4. Action Validation

1. **Check Available Actions**:
   ```bash
   # List all actions
   ros2 action list
   # Should include your actions like /fibonacci_action
   ```

2. **Test Action Execution**:
   ```bash
   # Send a goal to the fibonacci action
   ros2 action send_goal /fibonacci_action example_interfaces/action/Fibonacci '{order: 5}'
   # Should return a sequence like [0, 1, 1, 2, 3, 5]
   ```

### 5. Transform (TF) Validation

1. **View the TF Tree**:
   ```bash
   # Visualize the TF tree
   ros2 run rqt_tf_tree rqt_tf_tree
   # Should show a proper tree with base_link as the root and all other frames connected
   ```

2. **Check Specific Transforms**:
   ```bash
   # Get the transform between base_link and laser_scanner
   ros2 run tf2_tools view_frames
   # Generates a PDF showing the frame relationships
   ```

3. **Echo Specific Transforms**:
   ```bash
   # Monitor the transform between two frames
   ros2 run tf2_ros tf2_echo base_link laser_scanner
   # Should show continuous updates of the transform
   ```

### 6. Sensor Data Validation

1. **Verify Laser Scan Data**:
   ```bash
   # Check if laser scan messages are being published
   ros2 topic echo /scan
   # Should show continuous LaserScan messages with ranges data
   ```

2. **Verify Joint States**:
   ```bash
   # Check if joint states are being published
   ros2 topic echo /joint_states
   # Should show continuous JointState messages with position data
   ```

### 7. Complete System Integration Test

1. **Launch Complete System**:
   ```bash
   # Build your workspace
   cd ~/ros2_ws
   colcon build --packages-select hello_robot_description hello_robot_controller
   source install/setup.bash

   # Launch the complete system
   ros2 launch hello_robot_controller hello_robot.launch.py
   ```

2. **Monitor All Components**:
   - In RViz2, verify the robot model appears correctly
   - Check that TF transforms are updating properly
   - Verify that sensor data (laser scan, joint states) is being published
   - Send commands via topics/services/actions and verify responses

3. **Performance Validation**:
   ```bash
   # Check the frequency of published topics
   ros2 topic hz /joint_states
   # Should match the expected publishing rate (e.g., ~50Hz)

   # Monitor node resource usage
   ros2 run topicos topicos
   # Should show reasonable CPU and memory usage
   ```

### 8. Exercise Completion Check

Validate that you've completed all aspects of the hands-on exercise:

1. **Custom Robot Model**:
   - [ ] URDF file created with base, wheels, and sensor mast
   - [ ] Proper visual and collision properties defined
   - [ ] Correct inertial properties calculated

2. **Communication Patterns**:
   - [ ] Publisher-subscriber pairs working correctly
   - [ ] Services responding to requests
   - [ ] Actions completing goals with feedback

3. **Visualization**:
   - [ ] Robot appears correctly in RViz2
   - [ ] All links and joints visible
   - [ ] TF transforms properly displayed

4. **AI Integration**:
   - [ ] AI logic connected to robot control
   - [ ] Sensor data feeding into AI processing
   - [ ] Appropriate responses to environmental stimuli

### Troubleshooting Checklist

If any validation steps fail, check:

- [ ] All required packages are installed and sourced
- [ ] URDF file is properly formatted and all referenced resources exist
- [ ] Node names don't conflict with existing nodes
- [ ] Topic/service/action names match between publishers and subscribers
- [ ] QoS profiles are compatible between publishers and subscribers
- [ ] All required dependencies are declared in package.xml
- [ ] Environment variables are properly set (ROS_DOMAIN_ID, RMW_IMPLEMENTATION)

### Expected Outcomes

Upon successful validation, you should observe:

1. A complete robot model visualized in RViz2 with all components
2. Responsive communication through all three patterns (topics, services, actions)
3. Proper TF transforms showing spatial relationships
4. Published sensor data matching simulated environment
5. All validation commands returning expected results without errors

## Summary

This tutorial has walked you through creating a complete robot simulation that demonstrates all core ROS 2 concepts working together. You've learned how to integrate URDF models, implement all communication patterns, and visualize your robot in RViz2. This forms the foundation for more advanced robotics applications.