# Chapter 3: High-Fidelity Rendering (Unity)

## Introduction

Welcome to the Unity visualization environment for your robot! In this chapter, we'll create a photorealistic visualization layer that complements the physics-accurate simulation from Gazebo. Unity provides exceptional rendering capabilities that enable high-fidelity visualization for human-robot interaction (HRI) research and advanced perception tasks.

The "Split-Brain" architecture we're implementing uses Gazebo for physics and sensor simulation while leveraging Unity's superior rendering capabilities for visualization. This combination provides the best of both worlds: accurate physics simulation and photorealistic rendering.

## Understanding Unity for Robotics

### Why Unity for Robotics?

Unity offers several advantages for robotics visualization:

- **Photorealistic Rendering**: Advanced lighting, materials, and effects
- **Human-Robot Interaction**: Intuitive interfaces and visualization
- **Flexible Environments**: Easy creation of complex, detailed scenes
- **Real-time Performance**: Optimized for interactive applications
- **Asset Ecosystem**: Extensive library of models, materials, and tools

### Unity Robotics Integration

Unity integrates with ROS 2 through the Unity Robotics Hub, which provides:
- ROS message serialization/deserialization
- Communication protocols for bidirectional data flow
- Sample projects and components for robotics applications
- Tools for developing and testing robot behaviors

## Unity Installation and Setup

### System Requirements

Before installing Unity, ensure you have:
- **Operating System**: Windows 10/11, macOS 10.14+, or Ubuntu 20.04+
- **GPU**: DirectX 10, 11, or 12 compatible graphics card
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free space for Unity installation
- **Additional**: Git for version control

### Installing Unity Hub

Unity Hub is the central application for managing Unity installations and projects:

1. **Download Unity Hub** from https://unity.com/download
2. **Install Unity Hub** using the installer for your platform
3. **Launch Unity Hub** and sign in with a Unity ID (free account)
4. **Verify installation** by checking the installed versions panel

### Installing Unity Editor

1. **Open Unity Hub** and navigate to the "Installs" tab
2. **Click "Add"** to install a new Unity version
3. **Select Unity 2022.3 LTS** (Long Term Support) - recommended for robotics
4. **Select modules**:
   - Unity Editor
   - Visual Studio Code Editor Package (or your preferred IDE)
   - Android Build Support (if needed)
   - Linux Build Support (if needed)
5. **Complete installation** - this may take 20-40 minutes depending on internet speed

## Installing Unity Robotics Hub

### What is Unity Robotics Hub?

Unity Robotics Hub is a package that enables communication between Unity and ROS 2. It includes:
- ROS-TCP-Connector for communication
- Sample robotics projects
- Robotics-specific components and tools

### Installation Steps

1. **Launch Unity Editor** through Unity Hub
2. **Create a new 3D project** or open an existing one
3. **Open Package Manager** (Window > Package Manager)
4. **Click the "+" button** in the top-left corner
5. **Select "Add package from git URL..."**
6. **Enter the Unity Robotics package URL**: `com.unity.robotics.ros-tcp-connector`
7. **Click "Add"** to install the package
8. **Verify installation** by checking the package list

## Importing Robot Models

### Converting Models for Unity

Unity uses different file formats than Gazebo. The most common formats are:
- **FBX**: Most versatile, supports geometry, materials, and animations
- **OBJ**: Basic geometry, good for static models
- **glTF**: Modern format, good for web deployment

### Converting from URDF to Unity

1. **Export from CAD** (if available) to FBX format
2. **Convert DAE to FBX** if using models from Gazebo
3. **Import FBX** into Unity project
4. **Set up materials** and textures
5. **Configure colliders** for physics interaction

### Importing Your Robot Model

1. **Prepare your robot model** in FBX format
2. **Create Assets folder** in your Unity project: `Assets/RobotModels/`
3. **Drag and drop** the FBX file into the folder
4. **Configure import settings**:
   - Scale Factor: 1 (adjust if needed)
   - Mesh Compression: Off for precision
   - Read/Write Enabled: Checked
   - Optimize Mesh: Checked
5. **Apply changes** and verify the model appears correctly

## Creating Unity Scenes

### Basic Scene Setup

1. **Create a new scene**: File > New Scene
2. **Save the scene**: File > Save Scene As > `Assets/Scenes/basic_scene.unity`
3. **Set up lighting**:
   - Add Directional Light for sun-like lighting
   - Adjust Intensity to 1.0
   - Set Color to warm white (255, 250, 240)
4. **Add environment**:
   - Create ground plane or import environment
   - Set up skybox for realistic background

### Adding Your Robot to the Scene

1. **Drag your robot model** from Assets into the scene hierarchy
2. **Position the robot** at the origin (0, 0, 0) or desired location
3. **Scale appropriately** if needed (Unity uses meters)
4. **Add colliders** for physical interaction:
   - Use appropriate collider types (Box, Sphere, Capsule, Mesh)
   - Set as "Is Trigger" if needed for sensor simulation

### Configuring Materials and Textures

1. **Create materials** for different robot parts:
   - Metallic surfaces: High metallic, low roughness
   - Plastic surfaces: Low metallic, medium roughness
   - Rubber surfaces: Low metallic, high roughness
2. **Apply textures** if available:
   - Albedo (color) maps
   - Normal maps for surface detail
   - Metallic/Roughness maps

## Setting up Human-Robot Interaction (HRI)

### Understanding HRI in Unity

Human-Robot Interaction interfaces allow users to:
- Control robot behavior through intuitive interfaces
- Visualize robot sensor data
- Monitor robot status and health
- Plan robot missions and tasks

### Creating HRI Interfaces

1. **Add Canvas** for UI elements (GameObject > UI > Canvas)
2. **Create control panels**:
   - Movement controls (joysticks, buttons)
   - Sensor data displays
   - Robot status indicators
3. **Implement interaction logic** using Unity's UI system
4. **Connect to ROS 2** for bidirectional communication

### Example HRI Controls

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotCommandTopic = "/cmd_vel";

    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button moveButton;

    void Start()
    {
        ros = ROSConnection.instance;

        // Set up button click listeners
        moveButton.onClick.AddListener(SendVelocityCommand);
    }

    void SendVelocityCommand()
    {
        // Create a Twist message
        var twist = new TwistMsg();
        twist.linear = new Vector3(linearVelocitySlider.value, 0, 0);
        twist.angular = new Vector3(0, 0, angularVelocitySlider.value);

        // Send the message to the robot
        ros.Send(robotCommandTopic, twist);
    }
}
```

## Optimizing for Performance

### Rendering Optimization

- **Use Level of Detail (LOD)** for complex models
- **Implement occlusion culling** for large environments
- **Use baked lighting** for static objects
- **Optimize textures** to appropriate resolution

### Physics Optimization

- **Use simple colliders** where possible
- **Disable unnecessary physics** on static objects
- **Optimize mesh colliders** for complex shapes

## Testing Visual Quality

### Quality Comparison

Unity rendering typically offers:
- **Better lighting models** (PBR materials)
- **More realistic shadows** (with proper setup)
- **Advanced post-processing effects**
- **Better texture filtering**

### Validation Steps

1. **Compare lighting** between Gazebo and Unity
2. **Check material accuracy** against real robot
3. **Verify color representation** matches physical robot
4. **Test rendering performance** at target frame rate

## Troubleshooting Common Issues

### Model Import Issues
- **Missing textures**: Ensure texture files are in the same folder or properly referenced
- **Incorrect scale**: Check import scale settings and adjust as needed
- **Missing geometry**: Verify FBX export settings in original tool

### Performance Issues
- **Slow rendering**: Reduce draw distance or optimize materials
- **High memory usage**: Compress textures and reduce polygon count
- **Frame rate drops**: Implement LOD or reduce rendering quality

## Hands-on Exercise: Basic Unity Scene

1. **Install Unity Hub and Editor** with Robotics Hub package
2. **Import your robot model** in FBX format
3. **Create a basic scene** with lighting and environment
4. **Add basic HRI controls** to test robot movement
5. **Compare visual quality** with Gazebo rendering

## Next Steps

Once your Unity visualization environment is set up, you'll be ready for [Chapter 4: The Simulation Bridge](04-simulation-bridge.md), where we'll connect both simulation environments to ROS 2, enabling seamless communication between your control algorithms and both the physics simulation and visualization.

You may also want to explore:
- [Chapter 1: Laws of Physics (Gazebo Setup)](01-laws-of-physics-gazebo.md) for the physics foundation
- [Chapter 2: The Sensory Apparatus](02-sensory-apparatus.md) for sensor integration
- [Module Summary and Next Steps](module-summary-next-steps.md) for an overview of the complete Digital Twin

## Accessibility Considerations

- All Unity interfaces can be customized for different accessibility needs
- Documentation includes visual diagrams and text descriptions
- UI elements can be made keyboard navigable
- Alternative text provided for visual concept diagrams

---

## Summary

In this chapter, we've established a high-fidelity visualization environment using Unity:
- Installed Unity Hub and Editor with Robotics Hub package
- Imported robot models with proper materials and textures
- Created Unity scenes with appropriate lighting and environments
- Implemented Human-Robot Interaction interfaces
- Optimized for performance and visual quality

This visualization layer complements the physics simulation from Gazebo, providing the "Split-Brain" architecture that leverages the strengths of both systems.