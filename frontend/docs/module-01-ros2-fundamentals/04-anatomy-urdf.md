# Chapter 4: Anatomy of a Humanoid (URDF)

## Introduction to Robot Description

The Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, inertial properties, and how they connect to form a complete robot model. Understanding URDF is crucial for robotics simulation, visualization, and control.

### Accessibility Considerations for Robot Models

When creating and working with URDF models, consider these accessibility aspects:

- **Descriptive Link and Joint Names**: Use clear, meaningful names that convey the component's function (e.g., `left_wheel_front` instead of `lwf`)
- **Clear Material Definitions**: Use color names or well-documented color values that are meaningful
- **Comprehensive Comments**: Include comments in URDF files that explain the purpose of each component
- **Logical Hierarchies**: Organize the robot structure in a logical, easy-to-follow manner
- **Consistent Units**: Always specify units clearly and use consistent measurement systems
- **Visual-Text Alternatives**: Provide text descriptions of robot appearance and structure for screen reader users
- **Simplified Models for Visualization**: When possible, provide simplified versions of complex models for better visualization
- **Proper Inertial Values**: Document how inertial values were calculated for transparency

### Why Robot Description Matters

Robot description serves several important purposes:
- **Simulation**: Enables physics simulation in environments like Gazebo
- **Visualization**: Allows proper visualization in tools like RViz
- **Kinematics**: Provides the geometric structure for forward and inverse kinematics
- **Collision Detection**: Defines collision properties for planning and safety
- **Standardization**: Creates a common format for robot models across ROS tools

## URDF Fundamentals

### Basic Structure

A URDF file is an XML document that describes a robot. The basic structure includes:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <!-- Visual properties for display -->
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>

    <!-- Collision properties for physics -->
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>

    <!-- Inertial properties for physics simulation -->
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

### Key Components

#### Links
Links represent rigid bodies in the robot. Each link has:
- A unique name
- Visual properties (how it appears)
- Collision properties (how it interacts physically)
- Inertial properties (mass, center of mass, moments of inertia)

**Detailed Link Structure:**
```xml
<link name="link_name">
  <!-- Visual properties for rendering in simulation/visualization -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Offset from link origin -->
    <geometry>
      <!-- One of: box, cylinder, sphere, mesh -->
      <box size="1 1 1"/>
      <!-- OR -->
      <cylinder radius="0.5" length="1.0"/>
      <!-- OR -->
      <sphere radius="0.5"/>
      <!-- OR -->
      <mesh filename="package://my_robot/meshes/link_mesh.stl"/>
    </geometry>
    <material name="mat_name">
      <color rgba="0.8 0.2 0.1 1.0"/>
      <!-- OR reference a predefined material -->
      <!-- <texture filename="path/to/texture.png"/> -->
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Offset from link origin -->
    <geometry>
      <!-- Same geometry types as visual, often simplified for performance -->
      <cylinder radius="0.5" length="1.0"/>
    </geometry>
  </collision>

  <!-- Inertial properties for physics simulation -->
  <inertial>
    <mass value="1.0"/>  <!-- Mass in kg -->
    <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Center of mass offset -->
    <!-- Inertia tensor (symmetric, so only 6 values needed) -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

#### Joints
Joints connect links together and define how they can move relative to each other. Joint types include:
- **Fixed**: No movement allowed (welded connection)
- **Revolute**: Rotational movement around an axis (like a hinge)
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Linear sliding movement along an axis
- **Planar**: Movement in a plane
- **Floating**: 6 degrees of freedom (rarely used)

**Detailed Joint Structure:**
```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Transform from parent to child when joint is at zero position -->
  <axis xyz="0 0 1"/>  <!-- Axis of rotation/translation (normalized) -->

  <!-- Joint limits (not applicable for fixed and continuous joints) -->
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>

  <!-- Joint dynamics -->
  <dynamics damping="0.1" friction="0.0"/>

  <!-- Joint safety limits -->
  <safety_controller k_position="10" k_velocity="10" soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
</joint>
```

**Joint Types in Detail:**

1. **Fixed Joint**: Connects two links rigidly with no relative motion
   ```xml
   <joint name="fixed_connection" type="fixed">
     <parent link="base"/>
     <child link="sensor_mount"/>
     <origin xyz="0.1 0.0 0.2" rpy="0 0 0"/>
   </joint>
   ```

2. **Revolute Joint**: Single degree of freedom rotational joint with position limits
   ```xml
   <joint name="elbow_joint" type="revolute">
     <parent link="upper_arm"/>
     <child link="lower_arm"/>
     <origin xyz="0 0 -0.3" rpy="0 0 0"/>
     <axis xyz="0 1 0"/>
     <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
     <dynamics damping="0.5" friction="0.1"/>
   </joint>
   ```

3. **Continuous Joint**: Single degree of freedom rotational joint without position limits
   ```xml
   <joint name="continuous_rotation" type="continuous">
     <parent link="base"/>
     <child link="rotating_plate"/>
     <origin xyz="0 0 0.1" rpy="0 0 0"/>
     <axis xyz="0 0 1"/>
     <limit effort="10.0" velocity="1.0"/>
     <dynamics damping="0.1" friction="0.0"/>
   </joint>
   ```

4. **Prismatic Joint**: Single degree of freedom linear sliding joint
   ```xml
   <joint name="slider_joint" type="prismatic">
     <parent link="base"/>
     <child link="slider"/>
     <origin xyz="0.1 0 0" rpy="0 0 0"/>
     <axis xyz="1 0 0"/>
     <limit lower="0" upper="0.5" effort="20.0" velocity="0.5"/>
     <dynamics damping="1.0" friction="0.5"/>
   </joint>
   ```

## Creating Your First URDF Model

Let's create a simple robot model to understand the basics:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link - this is the reference frame for the entire robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- A simple arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Visual and Collision Properties

### Visual Elements

The `<visual>` tag defines how a link appears in visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Choose one geometry type -->
    <box size="1 1 1"/>
    <!-- OR -->
    <cylinder radius="0.5" length="1.0"/>
    <!-- OR -->
    <sphere radius="0.5"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/link.stl"/>
  </geometry>
  <material name="my_material">
    <color rgba="0.8 0.2 0.1 1.0"/>
    <!-- OR reference a material -->
    <!-- <texture filename="package://my_robot/materials/textures/my_texture.png"/> -->
  </material>
</visual>
```

### Collision Elements

The `<collision>` tag defines the physical shape for collision detection:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Similar to visual, but often simplified for performance -->
    <box size="1 1 1"/>
    <!-- OR -->
    <cylinder radius="0.5" length="1.0"/>
    <!-- OR -->
    <sphere radius="0.5"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/collision_link.stl"/>
  </geometry>
</collision>
```

## Inertial Properties

Inertial properties are crucial for physics simulation:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

The inertia tensor values depend on the mass distribution:
- For a box: `ixx = m/12 * (h² + d²)`, `iyy = m/12 * (w² + d²)`, `izz = m/12 * (w² + h²)`
- For a cylinder: `ixx = iyy = m/12 * (3*r² + h²)`, `izz = m/2 * r²`
- For a sphere: `ixx = iyy = izz = 2/5 * m * r²`

## Joints in Detail

### Joint Types and Parameters

```xml
<!-- Revolute joint (rotational) -->
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>

<!-- Prismatic joint (linear) -->
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Linear motion axis -->
  <limit lower="0.0" upper="0.5" effort="100.0" velocity="0.5"/>
</joint>

<!-- Fixed joint (no movement) -->
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

### Joint Limits and Dynamics

Joint limits and dynamics parameters:
- `lower`, `upper`: Position limits (radians for revolute, meters for prismatic)
- `effort`: Maximum force/torque (N for prismatic, Nm for revolute)
- `velocity`: Maximum velocity (m/s for prismatic, rad/s for revolute)
- `damping`: Viscous damping coefficient
- `friction`: Coulomb friction coefficient

## Transforms and Coordinate Frames (TF)

The Transform (TF) system in ROS is fundamental for understanding spatial relationships between different parts of the robot and the environment.

### Understanding Coordinate Frames

Each link in a URDF model has its own coordinate frame. The relationship between frames is described by:
- **Position**: (x, y, z) translation
- **Orientation**: (roll, pitch, yaw) rotation

### Static and Dynamic Transforms

- **Static transforms**: Fixed relationship between frames (e.g., sensor mount)
- **Dynamic transforms**: Changing relationship (e.g., moving joints)

### TF Tree Structure

The TF tree is a directed graph with a single root frame. For a robot, this is typically `base_link`. All other frames are connected through this tree structure.

## Visualizing URDF Models

### Using RViz

To visualize your URDF model in RViz:
1. Launch your robot's state publisher
2. Add a RobotModel display
3. Set the Robot Description parameter to your URDF parameter name

### Using Robot State Publisher

The robot_state_publisher node automatically publishes static transforms based on your URDF and joint states:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = ['joint_name_1', 'joint_name_2']
        msg.position = [0.0, 0.5]  # Example positions
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        # Publish the message
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for URDF

### 1. Organize with Xacro

Xacro is a macro language that makes URDF files more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_xacro">
  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Define a macro for wheels -->
  <xacro:macro name="wheel" params="prefix parent x y z">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x} ${y} ${z}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <xacro:wheel prefix="front_left" parent="base_link" x="0.2" y="0.2" z="0"/>
  <xacro:wheel prefix="front_right" parent="base_link" x="0.2" y="-0.2" z="0"/>
  <xacro:wheel prefix="rear_left" parent="base_link" x="-0.2" y="0.2" z="0"/>
  <xacro:wheel prefix="rear_right" parent="base_link" x="-0.2" y="-0.2" z="0"/>
</robot>
```

### 2. Proper Inertial Values

Always calculate realistic inertial values for physics simulation:

```xml
<!-- For a solid cylinder rotating about its central axis -->
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.0125" ixy="0.0" ixz="0.0" iyy="0.0125" iyz="0.0" izz="0.025"/>
</inertial>
```

### 3. Collision Optimization

Use simplified geometries for collision to improve performance:

```xml
<!-- Visual can be complex -->
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/detailed_wheel.dae"/>
  </geometry>
</visual>

<!-- Collision uses simple shape -->
<collision>
  <geometry>
    <cylinder radius="0.1" length="0.05"/>
  </geometry>
</collision>
```

## Visualizing URDF Models in RViz2

### Loading and Visualizing URDF

To visualize your URDF models in RViz2, you need to:

1. Launch the robot_state_publisher to publish the static transforms from your URDF
2. Launch RViz2 and add the RobotModel display
3. Configure the display to show your robot model

### Basic Robot Model Visualization

Here's how to visualize the basic robot model we created:

```xml
<?xml version="1.0"?>
<robot name="basic_robot">
  <!-- Base link - this is the reference frame for the entire robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
      <material name="light_blue">
        <color rgba="0.5 0.5 1.0 1.0"/>
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
</robot>
```

### Humanoid Robot Model Visualization

Here's a more complex humanoid robot model with multiple joints:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.2604" ixy="0.0" ixz="0.0" iyy="0.3542" iyz="0.0" izz="0.1458"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.0038" ixy="0.0" ixz="0.0" iyy="0.0038" iyz="0.0" izz="0.0004"/>
    </inertial>
  </link>

  <link name="left_hand">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Left arm joints -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0.1" rpy="0 0 0.785"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="5.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Right arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.0038" ixy="0.0" ixz="0.0" iyy="0.0038" iyz="0.0" izz="0.0004"/>
    </inertial>
  </link>

  <link name="right_hand">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Right arm joints -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0.1 0.1" rpy="0 0 -0.785"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_lower_arm"/>
    <child link="right_hand"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="5.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Left leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 1.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.014" ixy="0.0" ixz="0.0" iyy="0.014" iyz="0.0" izz="0.0018"/>
    </inertial>
  </link>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 1.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.011" ixy="0.0" ixz="0.0" iyy="0.011" iyz="0.0" izz="0.0016"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00019" ixy="0.0" ixz="0.0" iyy="0.00057" iyz="0.0" izz="0.00071"/>
    </inertial>
  </link>

  <!-- Left leg joints -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0.0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0.0 0.0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0.0 0.0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Right leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 1.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.014" ixy="0.0" ixz="0.0" iyy="0.014" iyz="0.0" izz="0.0018"/>
    </inertial>
  </link>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 1.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.011" ixy="0.0" ixz="0.0" iyy="0.011" iyz="0.0" izz="0.0016"/>
    </inertial>
  </link>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00019" ixy="0.0" ixz="0.0" iyy="0.00057" iyz="0.0" izz="0.00071"/>
    </inertial>
  </link>

  <!-- Right leg joints -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0.0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0.0 0.0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0.0 0.0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>
</robot>
```

## Transforms and Spatial Reasoning

### Understanding Coordinate Frames

In robotics, coordinate frames are fundamental for describing positions and orientations of objects in space. A coordinate frame consists of:
- An origin point (x, y, z position)
- Three orthogonal axes (x, y, z directions)
- A reference point from which all measurements are taken

In ROS, each link in a URDF model has its own coordinate frame. The relationships between these frames are described by transforms.

### Transform Mathematics

A transform between two coordinate frames is represented by a 4×4 transformation matrix that combines:
- **Rotation**: A 3×3 rotation matrix describing the orientation relationship
- **Translation**: A 3×1 vector describing the positional offset

The transformation from frame A to frame B can be expressed as:
```
[x_B]   [Rxx Rxy Rxz Tx] [x_A]
[y_B] = [Ryx Ryy Ryz Ty] [y_A]
[z_B]   [Rzx Rzy Rzz Tz] [z_A]
[1  ]   [0   0   0   1 ] [1  ]
```

Where:
- [Rxx, Rxy, Rxz, ...] represents the rotation matrix
- [Tx, Ty, Tz] represents the translation vector

### Representing Rotations

Rotations can be represented in several ways:
1. **Roll-Pitch-Yaw (RPY)**: Three sequential rotations around the x, y, and z axes
2. **Quaternions**: Four values (x, y, z, w) that avoid gimbal lock and are computationally efficient
3. **Rotation Vector**: Three values representing an axis and angle of rotation

Quaternions are preferred in ROS because they:
- Don't suffer from gimbal lock
- Are more numerically stable
- Require fewer computations for chaining rotations

### TF Tree Structure

The TF system organizes all coordinate frames in a tree structure with a single root. For a mobile robot, this is typically the `base_link` frame. All other frames are connected through this tree structure, allowing any frame to be transformed to any other frame by following the chain of transforms.

For example, if you have:
- `odom` → `base_link` → `laser_frame`
- `odom` → `base_link` → `camera_frame`

You can transform from `laser_frame` to `camera_frame` by:
1. Transforming from `laser_frame` to `base_link`
2. Transforming from `base_link` to `camera_frame`

### Spatial Reasoning in Robotics

Spatial reasoning involves understanding the geometric relationships between objects in the robot's environment. This includes:
- **Forward Kinematics**: Calculating the position of the end effector given joint angles
- **Inverse Kinematics**: Calculating joint angles needed to achieve a desired end effector position
- **Collision Detection**: Determining if objects occupy overlapping space
- **Path Planning**: Finding trajectories that avoid obstacles

### Practical Example: Robot Arm Reachability

Consider a simple 2-link robot arm where:
- Link 1 has length L1 and rotates around the base
- Link 2 has length L2 and rotates around the end of Link 1

The reachable workspace is an annulus (ring) with:
- Inner radius: |L1 - L2|
- Outer radius: L1 + L2

Any point within this workspace can be reached by solving the inverse kinematics equations:
```
θ₂ = acos((x² + y² - L1² - L2²) / (2 × L1 × L2))
θ₁ = atan2(y, x) - atan2(L2 × sin(θ₂), L1 + L2 × cos(θ₂))
```

## Hands-On Exercise: Creating and Visualizing Robot Models

### Exercise Goals:
- Create a custom robot model using URDF
- Understand the relationship between links, joints, and coordinate frames
- Visualize your robot model in RViz2
- Implement simple transforms to understand spatial relationships

### Exercise Overview:
In this exercise, you'll build a simple wheeled robot with a sensor mast. This will help you understand how to structure a robot model with multiple links and joints.

### Steps:

1. **Create a New URDF File**:
   - Create a file named `my_robot.urdf` in your robot package
   - Define a base link as the main body of the robot
   - Add cylindrical geometry for the base (radius 0.3m, height 0.1m)

2. **Add Wheels to Your Robot**:
   - Create 4 wheel links (front-left, front-right, rear-left, rear-right)
   - Position wheels at the corners of the base (0.2m from center in x and y)
   - Use revolute joints to connect wheels to the base
   - Set the axis of rotation to be along the wheel's rotation axis (typically the Y-axis)

3. **Add a Sensor Mast**:
   - Create a vertical rod extending from the center of the base
   - Add a sensor (like a camera or LiDAR) at the top of the mast
   - Position the sensor 0.5m above the base

4. **Define Materials and Colors**:
   - Set the base to be blue
   - Set wheels to be black
   - Set the sensor mast to be gray

5. **Set Physical Properties**:
   - Define appropriate masses for each link
   - Calculate and set inertial properties (approximate as cylinders/spheres)

6. **Visualize in RViz2**:
   - Launch the robot_state_publisher with your URDF
   - Launch RViz2 and add a RobotModel display
   - Verify that your robot appears correctly with all links and joints

### Sample Implementation Structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot_exercise">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1458" ixy="0.0" ixz="0.0" iyy="0.1458" iyz="0.0" izz="0.225"/>
    </inertial>
  </link>

  <!-- Front left wheel -->
  <link name="wheel_fl">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.00125" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Front right wheel -->
  <link name="wheel_fr">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.00125" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Wheel joints -->
  <joint name="joint_wheel_fl" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_fl"/>
    <origin xyz="0.2 0.2 -0.05" rpy="1.57079632679 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <joint name="joint_wheel_fr" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_fr"/>
    <origin xyz="0.2 -0.2 -0.05" rpy="1.57079632679 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Add rear wheels similarly... -->

  <!-- Sensor mast -->
  <link name="mast">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0021" ixy="0.0" ixz="0.0" iyy="0.0021" iyz="0.0" izz="0.00002"/>
    </inertial>
  </link>

  <joint name="joint_base_mast" type="fixed">
    <parent link="base_link"/>
    <child link="mast"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Sensor at top of mast -->
  <link name="sensor">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00000417" ixy="0.0" ixz="0.0" iyy="0.00000417" iyz="0.0" izz="0.00000417"/>
    </inertial>
  </link>

  <joint name="joint_mast_sensor" type="fixed">
    <parent link="mast"/>
    <child link="sensor"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
</robot>
```

### Exercise Enhancement Ideas:
- Add a rotating sensor platform with a revolute joint
- Implement a simple arm with multiple joints
- Add realistic materials and textures
- Create a multi-robot scene with proper frame relationships
- Add sensor plugins to your robot model

### Validation:
Your implementation should successfully:
- Load without errors in robot_state_publisher
- Appear correctly in RViz2 with all links visible
- Show proper joint connections and transformations
- Allow you to visualize the TF tree in RViz2

### Command to Launch Visualization:
```bash
# Export your URDF as a parameter
export ROBOT_DESCRIPTION="$(cat my_robot.urdf)"

# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$ROBOT_DESCRIPTION"

# In another terminal, launch RViz2
rviz2
```

In RViz2, add a RobotModel display and set the Robot Description to `/robot_description`.

### Troubleshooting Tips:
- Check that all joint parents and children refer to existing links
- Verify that joint origins are in the correct coordinate frame
- Ensure all units are in meters and radians
- Check that inertial properties are physically plausible

## RViz2 Visualization Tutorials

### Setting Up RViz2 for Robot Visualization

RViz2 is the 3D visualization tool for ROS 2. It allows you to visualize robot models, sensor data, paths, and more in a 3D environment.

#### Launching RViz2 with Your Robot Model

To visualize your URDF robot model in RViz2, follow these steps:

1. **Prepare your URDF file**:
   - Ensure your URDF is properly formatted with all links, joints, and visual elements defined
   - Make sure all referenced meshes and materials are accessible

2. **Launch robot_state_publisher**:
   ```bash
   # Method 1: Using a parameter file
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat your_robot.urdf)"

   # Method 2: Using a launch file
   ros2 launch your_robot_package your_robot.launch.py
   ```

3. **Launch RViz2**:
   ```bash
   rviz2
   ```

#### Essential RViz2 Displays for Robot Visualization

Once RViz2 is running, you'll need to add the following displays:

1. **RobotModel Display**:
   - Add it via Panels → Displays → Add → RobotModel
   - Set "Robot Description" to the parameter name where your URDF is stored (usually `/robot_description`)
   - Adjust "TF Prefix" if needed
   - The robot model should now appear in the 3D view

2. **TF Display** (Transforms):
   - Add via Panels → Displays → Add → TF
   - This shows the coordinate frames and their relationships
   - Set "Frame Timeout" to a reasonable value (e.g., 15 seconds)
   - Enable "Show Names" and "Show Axes" to visualize the frame tree
   - This is crucial for understanding spatial relationships

3. **LaserScan Display** (for sensor data):
   - Add via Panels → Displays → Add → LaserScan
   - Set "Topic" to your laser scanner's topic (e.g., `/scan`)
   - Adjust "Size" for better visualization of points

4. **PointCloud Display** (for 3D sensor data):
   - Add via Panels → Displays → Add → PointCloud2
   - Set "Topic" to your point cloud topic (e.g., `/camera/depth/color/points`)
   - Adjust "Size" and "Style" for better visualization

#### Configuring the RobotModel Display

To properly configure the RobotModel display:

1. **Set the Robot Description Parameter**:
   - In the RobotModel display panel, set "Robot Description" to the name of the parameter where your URDF is stored
   - Common parameter names: `/robot_description`, `/my_robot_description`

2. **Adjust Visual Properties**:
   - Set "Visual Enabled" to true to show visual elements
   - Set "Collision Enabled" to true to show collision geometry (useful for debugging)
   - Set "Axes Enabled" to show coordinate frames for each link
   - Set "Names Enabled" to show link names in the 3D view

3. **Configure TF Prefix** (if applicable):
   - If your robot has a prefix in its TF tree, set "TF Prefix" accordingly
   - This is common when managing multiple robots

#### Viewing and Navigating the Robot Model

1. **Initial View Setup**:
   - Use the "Pan" tool (hand icon) to move the view around
   - Use the "Rotate" tool (curved arrow icon) to rotate the camera around the robot
   - Use the "Zoom" tool (magnifying glass icon) to zoom in/out
   - Press "F" to focus on a specific part of the robot
   - Press "Ctrl + Click + Drag" to orbit around a focal point

2. **Understanding the Grid**:
   - The grid represents the world coordinate frame (usually `map` or `odom`)
   - The red, green, and blue arrows represent the X, Y, and Z axes respectively
   - This helps orient yourself in 3D space

3. **Inspecting Individual Links**:
   - In the "RobotModel" display, you can expand the "Links" section
   - Each link shows its current position and orientation
   - You can enable/disable visualization of individual links

#### Troubleshooting Common Visualization Issues

1. **Robot Not Appearing**:
   - Verify that robot_state_publisher is running and publishing TF transforms
   - Check that the "Robot Description" parameter name matches what's published
   - Look for error messages in the RViz2 "Displays" panel
   - Ensure all mesh files referenced in the URDF are accessible

2. **Incorrect Joint Positions**:
   - Check that joint states are being published if you have moving joints
   - Verify that the TF tree is properly constructed
   - Use the TF display to visualize the frame relationships

3. **Missing or Incorrect Colors**:
   - Check that material definitions in your URDF are correct
   - Verify that color values are in the correct range (0.0-1.0 for RGBA)

4. **Performance Issues**:
   - Reduce the complexity of visual meshes
   - Limit the number of simultaneously displayed elements
   - Consider using simplified collision geometry for visualization

#### Advanced RViz2 Techniques

1. **Creating Saved Configurations**:
   - Use File → Save Config to save your display setup
   - This saves all display settings, view configuration, and panels
   - Load configurations with File → Open Config

2. **Using the Selection Tool**:
   - Select the "Selection" tool (cursor icon)
   - Click on elements in the 3D view to see their properties
   - This is useful for identifying specific links or frames

3. **Time Panel**:
   - Enable the Time panel to visualize data over time
   - Useful for replaying bag files or watching dynamic systems

4. **Custom Topics**:
   - Subscribe to custom topics published by your robot
   - Visualize paths, trajectories, or custom markers
   - Use the Marker display for custom visualization messages

### Working with Transforms (TF)

#### Transform (TF) Demonstration Code

Here's how to work with transforms in ROS 2 using the tf2 library:

```python
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
```

## Zero-Copy Data Transfer in ROS 2

### Understanding Zero-Copy Transfer

Zero-copy data transfer is an important performance optimization feature in ROS 2 that allows message data to be shared between processes without copying the actual data. This significantly reduces memory usage and improves communication performance, especially for large messages like images, point clouds, or laser scans.

### How Zero-Copy Works

In traditional message passing, when a publisher sends a message to a subscriber, the entire message data is copied from the publisher's memory space to the subscriber's memory space. With zero-copy transfer:

1. The publisher creates a message and stores it in shared memory
2. Instead of copying the data, a reference (pointer) to the shared memory location is sent
3. The subscriber accesses the data directly from the shared memory
4. Reference counting ensures the data remains valid until all subscribers are done

### Benefits of Zero-Copy

- **Reduced Memory Usage**: No duplicate copies of large messages
- **Improved Performance**: Faster message transmission and reduced CPU overhead
- **Better Real-Time Characteristics**: More predictable timing for large messages
- **Scalability**: Better performance when multiple subscribers receive the same message

### Implementation in ROS 2

ROS 2 implements zero-copy through the middleware layer, primarily using DDS (Data Distribution Service). When nodes are on the same machine, DDS can use shared memory for zero-copy transfers. The implementation varies by DDS vendor:

- **Fast DDS**: Supports zero-copy intraprocess and interprocess communication
- **Cyclone DDS**: Implements zero-copy using shared memory segments
- **RTI Connext**: Provides zero-copy through its shared memory transport

### Using Zero-Copy in Your Code

To take advantage of zero-copy transfer in ROS 2:

1. **Same Machine Communication**: Zero-copy only works when publisher and subscriber are on the same machine
2. **Large Messages**: Most beneficial for large messages like sensor_msgs/Image, sensor_msgs/PointCloud2
3. **Appropriate QoS Settings**: Use reliable reliability and transient local durability for better sharing

```python
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Create a QoS profile optimized for zero-copy with large messages
image_qos = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,  # For streaming data
    # In newer ROS 2 versions, you can also specify allocation settings for zero-copy
)

# Publisher with optimized QoS
image_pub = node.create_publisher(Image, 'camera/image_raw', image_qos)
```

### Memory Management Considerations

When using zero-copy, pay attention to:

- **Lifetime Management**: Ensure the publisher doesn't modify message data after publishing
- **Memory Allocation**: The middleware manages shared memory, not the application
- **Cross-Process Safety**: Zero-copy data should be treated as read-only by subscribers

### Performance Comparison

For a typical 640x480 RGB image (921,600 bytes):
- Traditional copy: ~2-5ms per message for serialization and copying
- Zero-copy: ~0.1-0.5ms for reference passing
- Memory usage: Half the memory footprint with zero-copy for multiple subscribers

### When to Use Zero-Copy

Zero-copy is most beneficial when:
- Publishing large messages (images, point clouds, maps)
- Multiple subscribers receive the same data
- Running nodes on the same machine
- Performance is critical (real-time applications)

Zero-copy may not be beneficial when:
- Messages are small (under 1KB)
- Network communication is required (between machines)
- Complex message transformations are needed

## Safety Considerations in Robot Simulation

### Physical Robot Deployment Warnings

⚠️ **CRITICAL SAFETY NOTICE**: The code examples and simulation environments described in this curriculum are designed exclusively for simulation and educational purposes. Before deploying any code to physical robots, the following safety measures must be implemented:

1. **Emergency Stop Systems**: All physical robots must have reliable emergency stop mechanisms accessible to operators
2. **Motion Limits**: Implement software and hardware limits for all joint movements
3. **Collision Detection**: Deploy robust collision detection and avoidance systems
4. **Safety-rated Hardware**: Use safety-certified controllers and drives for physical robot deployment
5. **Operational Boundaries**: Define and enforce operational boundaries for all robot movements

### Simulation vs. Reality Differences

When transitioning from simulation to real hardware, consider these critical differences:

- **Latency**: Real systems have communication and processing delays not present in simulation
- **Noise**: Sensor data contains noise and inaccuracies that simulation may not fully capture
- **Dynamics**: Real robot dynamics differ from simulation models
- **Environmental Factors**: Lighting, temperature, and surface conditions affect robot performance
- **Mechanical Wear**: Physical systems experience wear and tear over time

### Safe Development Practices

1. **Start in Simulation**: Validate all behaviors in simulation before physical deployment
2. **Progressive Testing**: Move from simulation to simple physical tests to complex operations
3. **Operator Training**: Ensure all operators understand robot behavior and emergency procedures
4. **Regular Inspections**: Schedule maintenance and safety checks for physical systems

## Summary

In this chapter, you learned about URDF (Unified Robot Description Format) and how to describe robots in ROS 2. You explored the fundamental components of URDF including links, joints, visual properties, and inertial properties. You also learned about the TF (Transform) system for spatial relationships and best practices for creating efficient robot models. The next chapter will cover the integration of all these concepts in a complete "Hello Robot" simulation.

## Cross-References

- **Previous Chapter**: [Chapter 3: Services & Actions](03-services-and-actions.md) - Advanced communication patterns in ROS 2
- **Foundational Concepts**: [Chapter 1: The ROS 2 Ecosystem](01-ros2-ecosystem.md) - Core ROS 2 architecture
- **Implementation Guide**: [Chapter 2: Speaking Robot (rclpy)](02-speaking-robot-rclpy.md) - Python-based robot communication

## References and Further Reading

- [ROS 2 URDF Documentation](https://docs.ros.org/en/humble/p/urdf/)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
- [Xacro Documentation](https://docs.ros.org/en/humble/p/xacro/)
- [ROS 2 TF2 (Transform Library) Documentation](https://docs.ros.org/en/humble/p/tf2/)
- [Robot State Publisher](https://docs.ros.org/en/humble/p/robot_state_publisher/)
- [RViz2 Visualization Tool](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/index.html)
- [ROS 2 Inertial Properties Guide](https://docs.ros.org/en/humble/Tutorials/URDF/Adding-Physical-Properties-to-URDF.html)
- [ROS 2 Joint Types and Parameters](https://docs.ros.org/en/humble/Tutorials/URDF/Building-a-Movable-Robot-Model-with-URDF.html)
- [ROS 2 Robot Description Best Practices](https://docs.ros.org/en/humble/Tutorials/URDF/Converting-A-Xacro-File-Into-A-URDF-File.html)
- [ROS 2 Visualization with RViz2](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html)