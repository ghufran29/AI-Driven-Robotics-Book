# ROS 2 Core Entities

This document provides detailed information about the core entities in ROS 2 based on the data model.

## ROS 2 Node

A process that performs computation, participating in the ROS 2 communication graph.

### Attributes:
- **node_name**: string (unique identifier)
- **namespace**: string (optional grouping)
- **lifecycle_state**: enum (created, configured, activated, deactivated, cleaned_up, shutdown)

### Relationships:
- Connects to other nodes via topics, services, or actions

## ROS 2 Topic

A named bus over which nodes exchange messages using publisher-subscriber pattern.

### Attributes:
- **topic_name**: string (unique within namespace)
- **message_type**: string (e.g., std_msgs/String, sensor_msgs/LaserScan)
- **qos_profile**: object (quality of service settings)

### Relationships:
- Connects publisher nodes to subscriber nodes

## ROS 2 Service

A request-response communication pattern for synchronous interactions.

### Attributes:
- **service_name**: string (unique within namespace)
- **service_type**: string (e.g., std_srvs/SetBool, custom_srvs/CustomService)
- **request_message**: object (request structure)
- **response_message**: object (response structure)

### Relationships:
- Connects client nodes to server nodes

## ROS 2 Action

A communication pattern for long-running tasks with feedback and goal management.

### Attributes:
- **action_name**: string (unique within namespace)
- **action_type**: string (e.g., nav_msgs/FollowPath, custom_actions/CustomAction)
- **goal**: object (action goal structure)
- **feedback**: object (progress updates)
- **result**: object (final outcome)

### Relationships:
- Connects action client nodes to action server nodes

## URDF Model

XML-based description of robot structure, including links, joints, and physical properties.

### Attributes:
- **robot_name**: string (unique identifier)
- **links**: array[Link] (rigid bodies)
- **joints**: array[Joint] (connections between links)
- **materials**: array[Material] (visual properties)

### Relationships:
- Defines the physical structure of a robot for simulation and visualization

## Link (URDF Component)

A rigid body in the robot structure.

### Attributes:
- **link_name**: string (unique identifier)
- **visual**: object (visual representation)
- **collision**: object (collision properties)
- **inertial**: object (mass and inertia properties)

## Joint (URDF Component)

A connection between two links.

### Attributes:
- **joint_name**: string (unique identifier)
- **joint_type**: enum (revolute, continuous, prismatic, fixed, etc.)
- **parent_link**: string (name of parent link)
- **child_link**: string (name of child link)
- **limits**: object (movement constraints for movable joints)

## TF Transform

Coordinate transformation system for spatial relationships between robot components.

### Attributes:
- **source_frame**: string (origin coordinate frame)
- **target_frame**: string (destination coordinate frame)
- **translation**: object (x, y, z offset)
- **rotation**: object (quaternion or Euler angles)

### Relationships:
- Maintains spatial relationships between coordinate frames

## Validation Rules from Requirements

1. All ROS 2 entities must have proper documentation of concepts including nodes, topics, services, and actions
2. Installation guides must validate Ubuntu 22.04 and ROS 2 Humble compatibility
3. rclpy code examples must validate Python 3.10+ syntax and ROS 2 API compatibility
4. Zero-copy transfer explanations must reference actual ROS 2 implementation details
5. Publisher/subscriber exercises must validate proper message exchange
6. Service and action examples must validate request-response and feedback patterns
7. URDF tutorials must validate proper XML structure and visualization
8. TF usage must validate proper coordinate frame relationships
9. RViz2 visualization must validate proper display of robot models
10. "Hello Robot" simulation must demonstrate all core concepts working together
11. All content must be formatted as Docusaurus Markdown
12. Code examples must validate Python 3.10+ compatibility

## State Transitions

### Node Lifecycle States
- **Created** → **Configured**: After successful initialization
- **Configured** → **Activated**: When node becomes active
- **Activated** → **Deactivated**: When node is temporarily disabled
- **Deactivated** → **Activated**: When node is re-enabled
- **Activated/Deactivated** → **Cleaned Up**: Before shutdown
- **Cleaned Up** → **Shutdown**: Final state

### Action Client States
- **Pending** → **Active**: When goal is accepted
- **Active** → **Feedback**: During execution with progress updates
- **Active/Feedback** → **Succeeded**: When goal completed successfully
- **Active/Feedback** → **Aborted**: When goal execution failed
- **Active/Feedback** → **Canceled**: When goal was canceled