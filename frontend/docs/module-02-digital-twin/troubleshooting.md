# Troubleshooting Guide: Module 2 - The Digital Twin

## Common Gazebo Issues

### Robot Falls Through Ground
**Symptoms**: Robot model falls through the ground plane or other static objects.
**Solutions**:
1. Check URDF collision geometry - ensure proper size and position
2. Verify mass and inertia properties are defined
3. Check that `<collision>` tags have valid geometry
4. Reduce physics time step in world file if needed

### Robot Jitters or Vibrates
**Symptoms**: Robot joints oscillate rapidly or robot body vibrates.
**Solutions**:
1. Reduce physics time step (e.g., from 0.001 to 0.0005)
2. Increase solver iterations in physics configuration
3. Check for intersecting collision geometries
4. Verify joint limits and stiffness parameters

### Sensor Data Issues
**Symptoms**: No sensor data published or unrealistic values.
**Solutions**:
1. Check Gazebo plugin configuration in URDF/Xacro
2. Verify topic names match documentation
3. Confirm plugin libraries are properly loaded
4. Check sensor mounting position and orientation

## Common Unity Issues

### Model Import Problems
**Symptoms**: Missing textures, incorrect scale, or distorted geometry.
**Solutions**:
1. Verify FBX export settings from original CAD software
2. Check Unity import scale settings (usually 1.0 for meters)
3. Ensure texture files are in same directory or properly referenced
4. Check for missing materials in Inspector

### Performance Issues
**Symptoms**: Low frame rate, stuttering, or high CPU/GPU usage.
**Solutions**:
1. Reduce scene complexity or draw distance
2. Use Level of Detail (LOD) for complex models
3. Optimize materials and textures
4. Check for unnecessary physics calculations

### ROS Connection Problems
**Symptoms**: No communication between Unity and ROS 2.
**Solutions**:
1. Verify IP address and port settings in ROS Connection component
2. Check firewall settings for required ports (default 10000)
3. Confirm ROS-TCP-Connector is properly configured
4. Test network connectivity between systems

## Common Bridge Issues

### ros_gz_bridge Connection Failures
**Symptoms**: Bridge fails to start or loses connection.
**Solutions**:
1. Verify Gazebo and ROS 2 are running before starting bridge
2. Check topic names match between systems
3. Confirm message type compatibility
4. Check network configuration and permissions

### High Latency
**Symptoms**: Delay between ROS commands and simulation response.
**Solutions**:
1. Check network bandwidth and latency
2. Reduce message rates if too high
3. Optimize message sizes
4. Check CPU usage on all systems

### Synchronization Problems
**Symptoms**: Robot state differs between Gazebo and Unity.
**Solutions**:
1. Verify time synchronization between systems
2. Check that state updates are published at appropriate rates
3. Confirm coordinate frame consistency
4. Implement interpolation if needed

## Installation and Setup Issues

### Gazebo Installation Problems
**Symptoms**: Gazebo fails to launch or missing plugins.
**Solutions**:
1. Verify ROS 2 Humble installation is complete
2. Check for missing dependencies: `sudo apt install gazebo libgazebo-dev`
3. Verify Gazebo plugins are installed: `sudo apt install ros-humble-gazebo-*`
4. Check environment variables are properly sourced

### Unity Installation Problems
**Symptoms**: Unity Editor fails to launch or missing packages.
**Solutions**:
1. Verify system requirements are met
2. Check Unity Hub is properly installed and updated
3. Confirm Unity Robotics Hub package is installed
4. Check for disk space and permissions issues

## Sensor Integration Issues

### LiDAR Problems
**Symptoms**: Incomplete scans, incorrect ranges, or no data.
**Solutions**:
1. Check scan parameters (samples, min/max angle, range)
2. Verify sensor mounting position and orientation
3. Confirm collision geometry doesn't block sensor view
4. Check for proper noise parameters

### Camera Issues
**Symptoms**: Black images, wrong resolution, or no depth data.
**Solutions**:
1. Verify camera pose and orientation in URDF
2. Check image resolution and format settings
3. Confirm lighting in environment is adequate
4. Check depth camera plugin configuration

### IMU Issues
**Symptoms**: Constant values, drift, or incorrect orientation.
**Solutions**:
1. Verify IMU placement on robot model
2. Check update rate settings
3. Confirm coordinate frame alignment
4. Check for proper noise parameters

## Performance Optimization

### General Performance Tips
1. **Reduce physics update rate** if real-time performance isn't required
2. **Simplify collision geometry** for complex models
3. **Limit sensor update rates** to required frequency
4. **Use appropriate mesh resolutions** for visualization

### Memory Management
1. **Monitor memory usage** during simulation
2. **Unload unused assets** when possible
3. **Use object pooling** for frequently created/destroyed objects
4. **Optimize texture sizes** without sacrificing quality

## Network and Communication Issues

### Firewall Configuration
**Issue**: Bridge components cannot communicate across network.
**Solution**: Open required ports (typically 10000 for Unity, various for Gazebo)

### Topic Mapping Problems
**Issue**: ROS topics don't connect to Gazebo/Unity.
**Solution**:
1. Verify topic names match in bridge configuration
2. Check namespace consistency
3. Confirm message type compatibility

## Validation and Testing

### Physics Validation
- **Stability Test**: Run simulation for 30+ minutes to check for drift
- **Collision Test**: Verify robot properly interacts with environment
- **Joint Limits**: Confirm joints respect defined limits

### Sensor Validation
- **Range Check**: Verify sensor data within expected ranges
- **Resolution Check**: Confirm data resolution matches specifications
- **Timing Check**: Validate sensor update rates

### Bridge Validation
- **Command Test**: Send commands and verify robot response
- **Sensor Test**: Check sensor data flows correctly to ROS
- **Latency Test**: Measure and document communication delays

## Debugging Strategies

### Systematic Debugging
1. **Isolate the issue** to specific component (Gazebo, Unity, or bridge)
2. **Check logs** for error messages and warnings
3. **Test components individually** before integration
4. **Use simple test cases** to validate functionality

### Logging and Monitoring
- Enable detailed logging for each component
- Monitor system resources (CPU, memory, network)
- Use ROS tools like `rqt_graph` to visualize communication
- Check Gazebo console output for physics errors

## Hardware-Specific Issues

### GPU Problems
**Symptoms**: Rendering artifacts, crashes, or poor performance.
**Solutions**:
1. Update graphics drivers to latest version
2. Check GPU meets minimum requirements
3. Verify DirectX/OpenGL compatibility
4. Try different rendering settings

### Memory Issues
**Symptoms**: Crashes, slow performance, or out-of-memory errors.
**Solutions**:
1. Close unnecessary applications
2. Increase virtual memory if possible
3. Reduce simulation complexity
4. Check for memory leaks in custom code

## Getting Help

### Documentation Resources
- Gazebo documentation: http://gazebosim.org/
- Unity Robotics documentation: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- ROS 2 documentation: https://docs.ros.org/

### Community Support
- ROS Discourse: https://discourse.ros.org/
- Unity forums: https://forum.unity.com/
- Gazebo answers: https://answers.gazebosim.org/

### Diagnostic Commands
```bash
# Check ROS 2 topics
ros2 topic list

# Check Gazebo status
gz topic -l

# Monitor bridge communication
ros2 run rqt_graph rqt_graph
```