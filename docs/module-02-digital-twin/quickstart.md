# Module 2 Quickstart Guide: The Digital Twin (Gazebo & Unity)

## Overview

This quickstart guide will help you set up and run the complete Digital Twin simulation environment combining Gazebo physics simulation with Unity visualization, connected through ROS 2 bridges.

## Prerequisites

Before starting, ensure you have:
- Completed Module 1 (ROS 2 fundamentals)
- Ubuntu 22.04 LTS with ROS 2 Humble installed
- Python 3.10+ with pip
- At least 16GB RAM and a modern GPU (NVIDIA RTX 3060 or equivalent)
- 20GB free disk space

## Environment Setup

### 1. Install Gazebo and Bridge Components

```bash
# Install Gazebo Harmonic
sudo apt update
sudo apt install -y gazebo libgazebo-dev

# Install Gazebo ROS packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install ros_gz bridge
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-bridge
```

### 2. Set up Simulation Workspace

```bash
# Create simulation workspace
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws

# Source ROS 2 environment (assuming Module 1 setup)
source /opt/ros/humble/setup.bash

# Build workspace (if you have custom packages)
colcon build
source install/setup.bash
```

## Launching the Complete Simulation

### 1. Start Gazebo Simulation

```bash
# Navigate to simulation workspace
cd ~/simulation_ws

# Source the environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Gazebo with your robot in the simple room world
gz sim -r simple_room.sdf
```

### 2. Launch the Gazebo-ROS Bridge

In a new terminal:

```bash
# Source the same environment
cd ~/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the bridge with configuration
ros2 run ros_gz_bridge parameter_bridge --ros-args --params-file docs/module-02-digital-twin/simulation-assets/bridge_config.yaml
```

### 3. Launch Unity Visualization (Conceptual)

Unity visualization would be launched separately:
1. Open Unity Hub
2. Launch your Unity project with the robot model
3. Configure ROS connection to the same IP/ports as the bridge

## Testing the Simulation

### 1. Verify Sensor Data

```bash
# Check available topics
ros2 topic list | grep -E "(scan|image|imu)"

# Monitor LiDAR data
ros2 topic echo /scan

# Monitor camera data
ros2 topic echo /camera/image_raw

# Monitor IMU data
ros2 topic echo /imu/data
```

### 2. Send Commands to the Robot

```bash
# Send a velocity command to move the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### 3. Use RViz2 for Visualization

```bash
# Launch RViz2 to visualize sensor data
rviz2
```

In RViz2, add displays for:
- Robot Model (showing your URDF)
- LaserScan (for LiDAR data)
- Image (for camera feed)
- TF (for coordinate frames)

## Running a Complete Example

### 1. Terminal 1 - Gazebo
```bash
cd ~/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
gz sim -r simple_room.sdf
```

### 2. Terminal 2 - Bridge
```bash
cd ~/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args --params-file docs/module-02-digital-twin/simulation-assets/bridge_config.yaml
```

### 3. Terminal 3 - Command Node
```bash
cd ~/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Send periodic commands to move the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" -r 1
```

### 4. Terminal 4 - Monitor Data
```bash
# Monitor sensor data
ros2 topic echo /scan
```

## Troubleshooting Quick Fixes

### No Sensor Data
- Verify Gazebo is running with your robot model
- Check that the bridge is properly configured and running
- Confirm sensor plugins are correctly defined in URDF

### Robot Not Responding to Commands
- Check that the command topic name matches the bridge configuration
- Verify the robot model has proper joint controllers
- Confirm the robot is not stuck or colliding with environment

### High Latency
- Check network connectivity between components
- Reduce simulation complexity if needed
- Monitor system resources (CPU, memory)

## Next Steps

Once you have the basic simulation running:

1. **Experiment with different worlds**: Try the obstacle course world included in the simulation assets
2. **Add more sensors**: Configure additional sensor types to your robot model
3. **Test navigation algorithms**: Use the simulation to develop and test path planning
4. **Connect Unity**: Integrate the Unity visualization component for high-fidelity rendering
5. **Performance testing**: Run longer simulations to validate stability

## Common Commands Reference

```bash
# List all ROS 2 topics
ros2 topic list

# Check topic types
ros2 topic info /topic_name

# Monitor a topic with specific frequency
ros2 topic echo /scan --field ranges -r 2

# Send a single command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"

# Check ROS 2 services
ros2 service list

# Visualize the ROS graph
rqt_graph
```

## Validation Checklist

Before proceeding to advanced features, verify:
- [ ] Gazebo launches with your robot model
- [ ] Sensor data is published on ROS topics
- [ ] Robot responds to velocity commands
- [ ] Bridge connects Gazebo and ROS 2 successfully
- [ ] Simulation runs stably for several minutes
- [ ] Sensor data quality matches expectations

## Getting Help

- Check the troubleshooting guide in `troubleshooting.md`
- Review the detailed documentation in each chapter
- Use ROS tools like `rqt_console` to check for error messages
- Verify all environment variables are properly set

---

This quickstart guide provides the essential steps to get your Digital Twin simulation running. For detailed configuration and advanced features, refer to the individual chapter documentation.