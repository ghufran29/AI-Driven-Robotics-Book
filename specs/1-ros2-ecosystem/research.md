# Research: Module 1: The Robotic Nervous System (ROS 2)

## Decision: Language Selection (rclpy vs rclcpp)
**Rationale**: Python (rclpy) chosen over C++ (rclcpp) for this module to prioritize accessibility for students transitioning from general programming to robotics. Python's simpler syntax and lower barrier to entry support the pedagogical excellence principle from the constitution.
**Alternatives considered**:
- C++ with rclcpp (higher performance, closer to hardware)
- Mixed approach (both languages)
**Trade-offs**: Performance vs. Learning curve - Python prioritizes educational goals over raw performance for this curriculum module.

## Decision: Version Pinning (ROS 2 Humble LTS)
**Rationale**: ROS 2 Humble Hawksbill is the Long Term Support (LTS) distribution with 5-year support until 2027, making it ideal for educational curriculum that needs stability. This aligns with the technical rigor and reproducibility principle.
**Alternatives considered**:
- Rolling distribution (newer features but less stability)
- Galactic/foxy (shorter support cycles)
**Trade-offs**: Newer features vs. Stability - LTS prioritizes consistent learning experience over cutting-edge features.

## Decision: Visualization Approach (Rviz2 screenshots vs Mermaid.js diagrams)
**Rationale**: Using Rviz2 screenshots for actual robot visualization and Mermaid.js diagrams for architectural concepts. This provides both realistic visualization (for URDF/TF concepts) and clear system diagrams (for node graphs).
**Alternatives considered**:
- Only code-based visualization
- Third-party diagram tools
**Trade-offs**: Realism vs. Clarity - Combining both approaches serves different pedagogical needs.

## Decision: Documentation Format (Docusaurus Markdown)
**Rationale**: Docusaurus provides excellent support for technical documentation with code snippets, versioning, and search capabilities. It supports the pedagogical excellence principle by providing a clean, accessible learning interface.
**Alternatives considered**:
- Sphinx documentation
- GitBook
- Custom static site
**Trade-offs**: Feature richness vs. Simplicity - Docusaurus provides the right balance for technical curriculum.

## Research: Zero-Copy Data Transfer in ROS 2
**Findings**: ROS 2 implements zero-copy through shared memory when nodes are on the same machine, significantly improving performance by avoiding data duplication. This is achieved through the rmw_implementation layer and requires specific configuration.
**Implementation**: Will explain concepts and provide configuration examples for optimal performance in educational context.

## Research: "Hello Robot" Simulation Requirements
**Findings**: The simulation needs to demonstrate all core ROS 2 concepts: nodes, topics, services, actions, and URDF visualization. This requires a simple robot model with basic movement capabilities.
**Implementation**: Create a minimal robot with publisher/subscriber for sensor data, service for simple commands, action for longer tasks, and URDF for visualization.

## Dependencies & Tools Research
- **Ubuntu 22.04**: Chosen for ROS 2 Humble compatibility and LTS support
- **Python 3.10+**: Required for ROS 2 Humble rclpy compatibility
- **Docusaurus 2.x**: Latest stable version for documentation
- **Node.js 16+**: Required for Docusaurus build process