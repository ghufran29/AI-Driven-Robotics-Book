# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

**Created**: 2025-12-17
**Feature**: Module 2: The Digital Twin (Gazebo & Unity)

## Prerequisites

Before starting this module, ensure you have:
- Completed Module 1 (ROS 2 fundamentals)
- Ubuntu 22.04 LTS or equivalent environment
- ROS 2 Humble Hawksbill installed and configured
- Python 3.10+ with pip
- At least 16GB RAM and a modern GPU (NVIDIA RTX 3060 or equivalent)
- 20GB free disk space

## Environment Setup

### 1. Install Gazebo Harmonic

```bash
# Add ROS repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install Gazebo Harmonic
sudo apt install -y gazebo libgazebo-dev
```

### 2. Install Unity Hub and Unity 2022.3 LTS

1. Download Unity Hub from https://unity.com/download
2. Install Unity Hub and launch it
3. Sign in with a Unity ID (free account)
4. Install Unity 2022.3 LTS version
5. Install Unity Robotics Hub package through the Package Manager

### 3. Install ROS 2 Bridge Packages

```bash
# Install Gazebo bridge for ROS 2
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-bridge

# Install additional simulation packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

## Basic Simulation Setup

### 1. Create Simulation Workspace

```bash
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws
colcon build
source install/setup.bash
```

### 2. Launch Basic Gazebo World

```bash
# Launch Gazebo with a simple world
gz sim -r empty.sdf

# Or create and launch your own world
cd ~/simulation_ws/src
# Create your world file in a new package
```

### 3. Import URDF from Module 1

```bash
# Copy your URDF from Module 1 to the simulation workspace
cp ~/ros2_ws/src/your_robot_description/urdf/robot.urdf ~/simulation_ws/src/simulation_robot/urdf/

# Launch the robot in Gazebo
ros2 launch gazebo_ros spawn_entity.py entity:=robot_name file:=path/to/robot.urdf
```

## Sensor Integration

### 1. Add LiDAR to Your Robot

Create a Gazebo plugin configuration in your URDF/Xacro:

```xml
<!-- LiDAR sensor plugin -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin filename="libgazebo_ros_ray_sensor.so" name="gazebo_ros_lidar">
      <ros>
        <namespace>lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Add Camera and IMU

```xml
<!-- Depth camera plugin -->
<gazebo reference="camera_link">
  <sensor type="depth" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.089</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
      <ros>
        <namespace>camera</namespace>
        <remapping>image_raw:=image_color</remapping>
        <remapping>camera_info:=camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>

<!-- IMU plugin -->
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Unity Setup

### 1. Import Robot Model to Unity

1. Launch Unity Hub and create a new 3D project
2. Import your robot model (converted to FBX format) via Assets > Import Package > Custom Package
3. Set up the robot hierarchy with appropriate joints and colliders
4. Add ROS# components for ROS communication

### 2. Configure Unity Robotics Hub

```csharp
// Example Unity script for ROS communication
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string topicName = "/unity_robot/cmd_vel";

    void Start()
    {
        ros = ROSConnection.instance;
    }

    public void SendCommand(float linear, float angular)
    {
        var cmdMsg = new Unity.RosMessageTypes.Geometry.TwistMsg(
            new Vector3(linear, 0, 0),   // Linear velocity
            new Vector3(0, 0, angular)   // Angular velocity
        );

        ros.Send(topicName, cmdMsg);
    }
}
```

## Bridge Configuration

### 1. Configure ros_gz_bridge

Create a bridge configuration file:

```yaml
# bridge_config.yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: "BIDIRECTIONAL"

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: "ROS_TO_GZ"

- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: "ROS_TO_GZ"
```

### 2. Launch the Bridge

```bash
# Source your ROS 2 environment
source ~/ros2_ws/install/setup.bash
source ~/simulation_ws/install/setup.bash

# Launch the bridge with configuration
ros2 run ros_gz_bridge parameter_bridge --ros-args --params-file bridge_config.yaml
```

## Testing the Simulation

### 1. Verify Physics Simulation

```bash
# Launch Gazebo with your robot
gz sim -r your_world.sdf

# Check if the robot responds to physics
# Verify gravity is working by observing if the robot falls when unsupported
```

### 2. Test Sensor Data

```bash
# Monitor sensor topics
ros2 topic echo /scan
ros2 topic echo /camera/image_raw
ros2 topic echo /imu/data

# Verify data is being published at expected rates
```

### 3. Test Bridge Communication

```bash
# Send commands and verify they affect the simulation
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'

# Monitor bridge status
ros2 run rqt_graph rqt_graph
```

## Troubleshooting

### Common Issues

1. **Gazebo not launching**: Ensure proper graphics drivers are installed
2. **Sensor data not publishing**: Check plugin configuration in URDF
3. **Bridge connection issues**: Verify network settings and firewall
4. **Physics instability**: Adjust time step and solver parameters

### Performance Tips

- Reduce simulation update rate if experiencing performance issues
- Use simplified collision meshes for better performance
- Monitor GPU usage when running Unity visualization
- Consider using a dedicated simulation machine for complex scenarios

## Next Steps

1. Complete all four chapters of Module 2:
   - Laws of Physics (Gazebo Setup)
   - The Sensory Apparatus
   - High-Fidelity Rendering (Unity)
   - The Simulation Bridge

2. Integrate with Module 1 ROS 2 nodes for complete system testing

3. Explore advanced topics like dynamic environments and multi-robot simulation