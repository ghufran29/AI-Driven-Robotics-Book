# Simulation Workflow Test Guide

## Test Objective
Validate the complete simulation workflow from ROS 2 commands to Unity visualization through the integrated Digital Twin architecture.

## Prerequisites
- Gazebo simulation running with robot model
- Unity visualization environment running
- ROS 2 bridge (ros_gz_bridge) operational
- Unity Robotics Hub connection established
- Robot model with sensors properly configured

## Test Setup
1. Launch Gazebo simulation with the robot in a test environment
2. Start Unity visualization with the same robot model
3. Establish bridge connections between ROS 2, Gazebo, and Unity
4. Verify all sensor topics are publishing data

## Test Procedure

### Step 1: Command Flow Validation
1. Send a movement command from ROS 2:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
   ```

2. Observe robot movement in Gazebo physics simulation
3. Observe synchronized robot movement in Unity visualization
4. Record response time from command to visual feedback

### Step 2: Sensor Data Flow Validation
1. Verify sensor data publication from Gazebo:
   - LiDAR: `/scan`
   - Camera: `/camera/image_raw`
   - IMU: `/imu/data`
   - Odometry: `/odom`

2. Confirm data reaches ROS 2 topics
3. Verify Unity receives relevant data for visualization
4. Check data consistency between Gazebo and Unity

### Step 3: Synchronization Validation
1. Monitor robot pose consistency between Gazebo and Unity
2. Verify timing synchronization between systems
3. Check that sensor timestamps align across platforms
4. Confirm coordinate frame consistency

### Step 4: Performance Validation
1. Measure command-to-response latency
2. Monitor system resource usage
3. Verify stable frame rates in Unity
4. Confirm physics simulation stability in Gazebo

## Expected Results
- Robot responds to ROS 2 commands in both Gazebo and Unity
- Sensor data flows correctly from Gazebo through bridge to ROS 2
- Unity visualization accurately reflects Gazebo physics state
- Latency remains under 100ms for real-time applications
- Simulation remains stable for extended periods

## Success Criteria
- Command execution: Robot moves as commanded in both environments
- Sensor feedback: All sensors publish valid data streams
- Synchronization: Robot states match between Gazebo and Unity
- Performance: Latency &lt; 100ms, stable operation
- Stability: No crashes or synchronization errors during 10-minute test

## Troubleshooting
If the workflow test fails:
1. Verify all bridge connections are active
2. Check network connectivity between systems
3. Confirm topic mappings are correct
4. Validate robot models are identical in both systems
5. Review log files for error messages