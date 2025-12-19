# Research Summary: Module 2 - The Digital Twin (Gazebo & Unity)

**Created**: 2025-12-17
**Feature**: Module 2: The Digital Twin (Gazebo & Unity)

## Decision: Gazebo Physics Engine Selection

**Rationale**: After evaluating options, Gazebo Classic (Harmonic) was selected for its mature ROS 2 Humble integration. While Gazebo Garden is the newer version, Gazebo Classic has more extensive documentation, community support, and proven stability for robotics applications. The ros_gz_bridge package has been extensively tested with Gazebo Classic, making it the safer choice for this curriculum.

**Alternatives considered**:
- Gazebo Garden: Newer but less ROS 2 integration testing
- Ignition Gazebo: Being superseded by Gazebo Garden
- Webots: Good alternative but different workflow than ROS 2 standard
- MuJoCo: Commercial solution with excellent physics but licensing costs

## Decision: Unity-ROS Bridge Technology

**Rationale**: Unity Robotics Hub was selected as the primary bridge technology due to its official support from Unity Technologies and integration with ROS 2. It provides pre-built components, message definitions, and sample projects that align perfectly with the curriculum's educational goals. The package includes comprehensive documentation and examples that make it accessible for learners.

**Alternatives considered**:
- ROS#: Community-developed bridge but less maintained
- Custom TCP/IP bridge: More complex to implement and maintain
- WebSocket bridge: Potential for custom solution but adds complexity
- ZeroMQ: Alternative messaging system but not ROS 2 native

## Decision: Asset Format Standards

**Rationale**: The implementation will use DAE (Collada) format for Gazebo due to its support for complex materials and textures that Gazebo's rendering pipeline can utilize effectively. For Unity, FBX format will be used as it preserves animation data, maintains material properties, and is the standard format for Unity asset import. This dual-format approach ensures optimal performance in both environments while requiring a conversion step that will be documented in the curriculum.

**Alternatives considered**:
- OBJ: Simpler but lacks material support
- STL: Good for collision geometry but poor for visualization
- glTF: Modern format with good tooling but less established in robotics
- PLY: Good for point clouds but not for complete models

## Decision: Sensor Simulation Fidelity

**Rationale**: The curriculum will balance computational efficiency with sensor accuracy by using realistic but not overly complex sensor models. This approach ensures that simulations run smoothly on standard hardware while providing sensor data that is representative of real-world sensors. The implementation will include configurable parameters to allow users to adjust fidelity based on their hardware capabilities.

**Alternatives considered**:
- Highly detailed simulation: More realistic but computationally expensive
- Simplified models: Faster but less representative of real sensors
- Ray-tracing for cameras: Photorealistic but excessive for robotics
- Custom sensor models: More control but increased complexity

## Decision: Split-Brain Architecture

**Rationale**: Using Gazebo for physics and sensor simulation while using Unity for visualization provides the optimal balance of accuracy and visual quality. Gazebo's physics engine (ODE, Bullet) is specifically designed for robotics simulation and provides accurate contact mechanics, while Unity's rendering pipeline offers superior visual quality for HRI applications. This separation allows each system to excel in its specialized domain.

**Alternatives considered**:
- Single environment approach: Either Gazebo or Unity alone (compromises either physics or visuals)
- Custom integrated solution: High complexity with uncertain benefits
- Third-party integrated tools: Limited flexibility and customization options