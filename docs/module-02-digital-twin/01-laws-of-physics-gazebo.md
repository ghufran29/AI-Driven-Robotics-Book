# Chapter 1: Laws of Physics (Gazebo Setup)

## Introduction

Welcome to the simulation environment setup for your robot! In this chapter, we'll establish a physically accurate simulation environment using Gazebo, which will serve as the foundation for all your virtual testing. Gazebo provides realistic physics simulation that allows you to test your robot's behavior in a controlled, repeatable environment before deploying to the physical world.

The "Digital Twin" concept we're building combines Gazebo's accurate physics and sensor simulation with Unity's high-fidelity visualization. In this chapter, we focus on the physics layer - the laws that govern how your robot interacts with the virtual world.

## Understanding Gazebo's Physics Engine

Gazebo uses a physics engine to simulate real-world physics. The most common engines are:
- **ODE (Open Dynamics Engine)**: Default for Gazebo Classic, excellent for ground vehicles
- **Bullet**: Good for general-purpose simulation
- **DART**: Advanced for humanoid robots
- **Simbody**: Biomechanics-focused

For our purposes, we'll use ODE as it's the default and provides excellent performance for most robotic applications.

## Installation Requirements

Before installing Gazebo, ensure you have:
- Ubuntu 22.04 LTS (recommended) or equivalent Linux distribution
- ROS 2 Humble Hawksbill installed and configured
- Python 3.10+ with pip
- At least 8GB RAM (16GB recommended)
- A modern GPU (for visualization)

### Installing Gazebo Harmonic

1. **Add the ROS repository** (if not already done from Module 1):
   ```bash
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   ```

2. **Install Gazebo Harmonic**:
   ```bash
   sudo apt install -y gazebo libgazebo-dev
   ```

3. **Install Gazebo ROS packages**:
   ```bash
   sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
   ```

4. **Install the bridge package**:
   ```bash
   sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-bridge
   ```

5. **Verify installation**:
   ```bash
   gz sim --version
   ```

## Configuring Physics Parameters

### Gravity Configuration

Gravity is a fundamental force in any physics simulation. The default gravity in Gazebo is set to Earth's gravity: (0, 0, -9.8) m/s².

In our `simple_room.sdf` world file, gravity is configured as:
```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <!-- ... other physics parameters ... -->
</physics>
```

To modify gravity for different scenarios:
- **Moon simulation**: Use (0, 0, -1.6) m/s²
- **Mars simulation**: Use (0, 0, -3.7) m/s²
- **Zero gravity**: Use (0, 0, 0) m/s²

### Time Step Configuration

The time step affects simulation stability and performance:
- **Max step size**: 0.001 seconds (1ms) provides good stability
- **Real time update rate**: 1000 Hz ensures real-time performance
- **Real time factor**: 1.0 for real-time simulation

### Friction and Contact Parameters

Friction parameters determine how objects interact with surfaces:

**Static Friction** - The force required to start motion between two surfaces
**Dynamic Friction** - The force required to maintain motion between surfaces

In Gazebo, friction is configured per material using the `<friction>` tag in model definitions:
```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>    <!-- Static friction coefficient -->
        <mu2>1.0</mu2>  <!-- Dynamic friction coefficient -->
      </ode>
    </friction>
  </surface>
</collision>
```

Common friction coefficients:
- **Rubber on concrete**: 0.9-1.0
- **Steel on steel**: 0.5-0.8
- **Ice on ice**: 0.1
- **Teflon on Teflon**: 0.04

## Creating Your First World

Let's create a basic world file with realistic physics parameters:

1. **Create the world directory**:
   ```bash
   mkdir -p ~/.gazebo/worlds
   ```

2. **Create a world file** (`simple_room.world`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="simple_room">
       <!-- Physics parameters -->
       <physics type="ode">
         <gravity>0 0 -9.8</gravity>
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <real_time_update_rate>1000</real_time_update_rate>
       </physics>

       <!-- Environment -->
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Your robot will be spawned here -->
     </world>
   </sdf>
   ```

## Importing Your Robot Model

To import your robot model from Module 1:

1. **Ensure your URDF is properly formatted** with collision and visual properties
2. **Convert to SDF if needed**: Gazebo can load URDF directly, but SDF is native
3. **Launch with your world**:
   ```bash
   gz sim -r simple_room.sdf
   ```

## Testing Physics Stability

### Stability Tests

1. **Static Stability**: Place your robot in the world and ensure it doesn't drift or fall through the ground
2. **Dynamic Stability**: Apply forces and verify realistic responses
3. **Long-term Stability**: Run simulations for extended periods to check for numerical drift

### Contact Detection

Verify that your robot properly detects contact with the ground and other objects:
- Check that wheels touch the ground without sinking
- Ensure no unintended collisions or jittering
- Verify that joints move freely without clipping through other parts

## Troubleshooting Common Physics Issues

### Robot Falls Through Ground
- Check collision geometry in your URDF
- Verify mass and inertia properties are set correctly
- Ensure `<collision>` tags have proper geometry

### Robot Jitters or Vibrates
- Reduce time step size (e.g., to 0.0005)
- Increase solver iterations
- Check for intersecting collision geometries

### Robot Slides Unrealistically
- Increase friction coefficients
- Check contact surfaces and materials
- Verify mass distribution

## Performance Optimization

### Physics Settings for Performance
- **Increase time step**: Higher values (e.g., 0.01) for less accuracy but better performance
- **Reduce solver iterations**: Fewer iterations for faster computation
- **Simplify collision geometry**: Use boxes and cylinders instead of complex meshes

### Visual Settings
- **Reduce rendering quality**: For headless simulations
- **Limit visual updates**: Reduce visual refresh rate when physics accuracy is more important

## Hands-on Exercise: Physics Validation

1. **Launch your world** with the robot model:
   ```bash
   gz sim -r simple_room.sdf
   ```

2. **Observe the robot's behavior**:
   - Does it maintain proper pose without drifting?
   - Do joints hold position without oscillating?
   - Does it respond realistically to gravity?

3. **Test stability** by applying small forces to the robot and observing the response

4. **Validate for 30+ minutes** to ensure long-term stability

## Next Steps

Once your physics environment is stable, you're ready to move to [Chapter 2: The Sensory Apparatus](02-sensory-apparatus.md), where we'll add realistic sensor simulation to your robot model. The stable physics foundation we've established here will ensure that your sensor data is accurate and representative of real-world conditions.

You may also want to explore:
- [Chapter 3: High-Fidelity Rendering (Unity)](03-high-fidelity-unity.md) for visualization
- [Chapter 4: The Simulation Bridge](04-simulation-bridge.md) for connecting all components
- [Module Summary and Next Steps](module-summary-next-steps.md) for an overview of the complete Digital Twin

## Hardware Requirements

For optimal performance:
- **GPU**: NVIDIA GeForce RTX 3060 or equivalent (for rendering)
- **RAM**: 16GB minimum, 32GB recommended
- **CPU**: Multi-core processor (4+ cores recommended)

## Accessibility Considerations

- All physics parameters are configurable through the SDF files for different simulation scenarios
- Documentation includes visual diagrams and text descriptions
- Console outputs provide detailed information for users with screen readers
- Alternative text provided for physics concept diagrams

---

## Summary

In this chapter, we've established a solid foundation for your simulation environment by:
- Installing and configuring Gazebo with ROS 2 integration
- Setting up realistic physics parameters including gravity and friction
- Creating a basic world with appropriate environmental conditions
- Validating physics stability for reliable simulation results

This physics foundation ensures that your robot's behavior in simulation will closely match its real-world performance, making your testing and development process both safe and effective.