#!/usr/bin/env python3

"""
Hello Robot Node - Comprehensive ROS 2 Robot Simulation

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

from std_msgs.msg import String, Bool
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
        self.joint_positions = {'neck_joint': 0.0, 'shoulder_joint': 0.0, 'elbow_joint': 0.0, 'wrist_joint': 0.0}

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
        self.joint_positions['shoulder_joint'] = 0.3 * math.sin(current_time * 0.7)
        self.joint_positions['elbow_joint'] = 0.4 * math.sin(current_time * 0.5)
        self.joint_positions['wrist_joint'] = 0.2 * math.sin(current_time * 1.2)

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

        # Camera transform (relative to base link)
        t_camera = TransformStamped()
        t_camera.header.stamp = self.get_clock().now().to_msg()
        t_camera.header.frame_id = 'base_link'
        t_camera.child_frame_id = 'camera'
        t_camera.transform.translation.x = 0.1
        t_camera.transform.translation.y = 0.0
        t_camera.transform.translation.z = 0.3
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