#!/usr/bin/env python3

"""
Transform (TF) Demonstration

This example demonstrates how to work with transforms in ROS 2 using the tf2 library.
It shows how to broadcast transforms and listen to transform data between coordinate frames.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import math
import time


class TFBroadcasterNode(Node):
    """
    A node that broadcasts transforms between coordinate frames.
    """

    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to periodically broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

        # Initialize time for transform timestamps
        self.time = self.get_clock().now()

        self.get_logger().info('TF Broadcaster node initialized')

    def broadcast_transforms(self):
        """
        Broadcast transforms between coordinate frames.
        """
        # Current time for the transform
        current_time = self.get_clock().now()

        # Create a transform from base_link to laser_frame
        t = TransformStamped()

        # Set the header
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        # Set the transform (position and orientation)
        # For demonstration, we'll make the laser frame rotate around the base
        angle = current_time.nanoseconds / 1e9  # Use time to create a changing angle
        radius = 0.3  # 30 cm from base

        t.transform.translation.x = radius * math.cos(angle)
        t.transform.translation.y = radius * math.sin(angle)
        t.transform.translation.z = 0.2  # 20 cm above base

        # For rotation, we'll set a simple rotation around Z-axis
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(angle / 2.0)
        t.transform.rotation.w = math.cos(angle / 2.0)

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

        # Create another transform: base_link to camera_frame
        t2 = TransformStamped()
        t2.header.stamp = current_time.to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'camera_frame'

        # Fixed transform for camera (mounted on top of base)
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.4  # 40 cm above base

        # Camera looking forward (no rotation)
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Send the transform
        self.tf_broadcaster.sendTransform(t2)

        # Log the transform
        self.get_logger().info(
            f'Broadcasting transform from {t.header.frame_id} to {t.child_frame_id} '
            f'at ({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}, {t.transform.translation.z:.2f})'
        )


class TFListenerNode(Node):
    """
    A node that listens to transforms between coordinate frames.
    """

    def __init__(self):
        super().__init__('tf_listener_node')

        # Create a transform buffer to store transforms
        self.tf_buffer = Buffer()

        # Create a transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a timer to periodically lookup transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

        self.get_logger().info('TF Listener node initialized')

    def lookup_transform(self):
        """
        Lookup and print the transform between two frames.
        """
        try:
            # Lookup the transform from base_link to laser_frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'laser_frame',
                now
            )

            # Print the transform
            self.get_logger().info(
                f'Transform from base_link to laser_frame: '
                f'({trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f}, {trans.transform.translation.z:.2f})'
            )

        except Exception as e:
            self.get_logger().error(f'Could not transform: {str(e)}')


def main_broadcaster(args=None):
    """
    Main function for the TF broadcaster node.
    """
    rclpy.init(args=args)

    node = TFBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TF broadcaster node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_listener(args=None):
    """
    Main function for the TF listener node.
    """
    rclpy.init(args=args)

    node = TFListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TF listener node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Example usage with a more complex TF tree
class ComplexTFDemoNode(Node):
    """
    A more complex example showing a hierarchical TF tree.
    """

    def __init__(self):
        super().__init__('complex_tf_demo')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to periodically broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_complex_transforms)

        self.get_logger().info('Complex TF Demo node initialized')

    def broadcast_complex_transforms(self):
        """
        Broadcast a more complex set of transforms forming a hierarchy.
        """
        current_time = self.get_clock().now()

        # Robot base frame
        base_transform = TransformStamped()
        base_transform.header.stamp = current_time.to_msg()
        base_transform.header.frame_id = 'odom'
        base_transform.child_frame_id = 'base_link'

        # Simulate robot moving in a circle
        angle = current_time.nanoseconds / 1e9
        base_transform.transform.translation.x = 2.0 * math.cos(angle / 5.0)
        base_transform.transform.translation.y = 2.0 * math.sin(angle / 5.0)
        base_transform.transform.translation.z = 0.0
        base_transform.transform.rotation.x = 0.0
        base_transform.transform.rotation.y = 0.0
        base_transform.transform.rotation.z = math.sin(angle / 10.0)
        base_transform.transform.rotation.w = math.cos(angle / 10.0)

        self.tf_broadcaster.sendTransform(base_transform)

        # Robot torso (child of base)
        torso_transform = TransformStamped()
        torso_transform.header.stamp = current_time.to_msg()
        torso_transform.header.frame_id = 'base_link'
        torso_transform.child_frame_id = 'torso'
        torso_transform.transform.translation.x = 0.0
        torso_transform.transform.translation.y = 0.0
        torso_transform.transform.translation.z = 0.5
        torso_transform.transform.rotation.x = 0.0
        torso_transform.transform.rotation.y = 0.0
        torso_transform.transform.rotation.z = 0.0
        torso_transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(torso_transform)

        # Robot head (child of torso)
        head_transform = TransformStamped()
        head_transform.header.stamp = current_time.to_msg()
        head_transform.header.frame_id = 'torso'
        head_transform.child_frame_id = 'head'
        head_transform.transform.translation.x = 0.0
        head_transform.transform.translation.y = 0.0
        head_transform.transform.translation.z = 0.3
        head_transform.transform.rotation.x = 0.0
        head_transform.transform.rotation.y = 0.0
        head_transform.transform.rotation.z = 0.0
        head_transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(head_transform)

        # Robot left arm (child of torso)
        arm_transform = TransformStamped()
        arm_transform.header.stamp = current_time.to_msg()
        arm_transform.header.frame_id = 'torso'
        arm_transform.child_frame_id = 'left_arm'
        arm_angle = math.sin(current_time.nanoseconds / 1e9)  # Oscillating
        arm_transform.transform.translation.x = 0.3 * math.cos(arm_angle)
        arm_transform.transform.translation.y = 0.3 * math.sin(arm_angle)
        arm_transform.transform.translation.z = 0.2
        arm_transform.transform.rotation.x = 0.0
        arm_transform.transform.rotation.y = 0.0
        arm_transform.transform.rotation.z = math.sin(arm_angle / 2.0)
        arm_transform.transform.rotation.w = math.cos(arm_angle / 2.0)

        self.tf_broadcaster.sendTransform(arm_transform)

        self.get_logger().info(
            f'Complex TF tree updated: odom->base_link->torso->head, torso->left_arm'
        )


def main_complex_demo(args=None):
    """
    Main function for the complex TF demo node.
    """
    rclpy.init(args=args)

    node = ComplexTFDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down complex TF demo node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Uncomment the one you want to run:
    # main_broadcaster()
    # main_listener()
    main_complex_demo()