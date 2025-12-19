# Foundational ROS 2 Concepts

This document covers the foundational concepts for ROS 2 that will be used throughout the module.

## Core Entities

### ROS 2 Node
A process that performs computation, participating in the ROS 2 communication graph with:
- node_name: string (unique identifier)
- namespace: string (optional grouping)
- lifecycle_state: enum (created, configured, activated, deactivated, cleaned_up, shutdown)

### ROS 2 Topic
A named bus over which nodes exchange messages using publisher-subscriber pattern with:
- topic_name: string (unique within namespace)
- message_type: string (e.g., std_msgs/String, sensor_msgs/LaserScan)
- qos_profile: object (quality of service settings)

### ROS 2 Service
A request-response communication pattern for synchronous interactions with:
- service_name: string (unique within namespace)
- service_type: string (e.g., std_srvs/SetBool, custom_srvs/CustomService)
- request_message: object (request structure)
- response_message: object (response structure)

### ROS 2 Action
A communication pattern for long-running tasks with feedback and goal management with:
- action_name: string (unique within namespace)
- action_type: string (e.g., nav_msgs/FollowPath, custom_actions/CustomAction)
- goal: object (action goal structure)
- feedback: object (progress updates)
- result: object (final outcome)

### URDF Model
XML-based description of robot structure, including links, joints, and physical properties with:
- robot_name: string (unique identifier)
- links: array[Link] (rigid bodies)
- joints: array[Joint] (connections between links)
- materials: array[Material] (visual properties)

### TF Transform
Coordinate transformation system for spatial relationships between robot components with:
- source_frame: string (origin coordinate frame)
- target_frame: string (destination coordinate frame)
- translation: object (x, y, z offset)
- rotation: object (quaternion or Euler angles)