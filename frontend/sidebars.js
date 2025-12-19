// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Manual sidebar structure for organized curriculum
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-01-ros2-fundamentals/foundational-concepts',
        'module-01-ros2-fundamentals/entities',
        'module-01-ros2-fundamentals/ros2-ecosystem',
        'module-01-ros2-fundamentals/speaking-robot-rclpy',
        'module-01-ros2-fundamentals/services-and-actions',
        'module-01-ros2-fundamentals/anatomy-urdf',
        'module-01-ros2-fundamentals/installation-guide',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-02-digital-twin/laws-of-physics-gazebo',
        'module-02-digital-twin/sensory-apparatus',
        'module-02-digital-twin/high-fidelity-unity',
        'module-02-digital-twin/simulation-bridge',
        'module-02-digital-twin/module-summary-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-03-isaac-brain/omniverse-isaac',
        'module-03-isaac-brain/synthetic-data-replicator',
        'module-03-isaac-brain/isaac-ros-vslam',
        'module-03-isaac-brain/nav2-path-planning',
        'module-03-isaac-brain/module-summary-next-steps',
        'module-03-isaac-brain/quickstart',
        'module-03-isaac-brain/glossary',
        'module-03-isaac-brain/safety',
        'module-03-isaac-brain/troubleshooting',
        'module-03-isaac-brain/implementation-history',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-04-vla-cognitive-pipeline/voice-interface',
        'module-04-vla-cognitive-pipeline/cognitive-brain',
        'module-04-vla-cognitive-pipeline/vision-grounding',
        'module-04-vla-cognitive-pipeline/capstone-autonomous',
        'module-04-vla-cognitive-pipeline/quickstart',
        'module-04-vla-cognitive-pipeline/glossary',
        'module-04-vla-cognitive-pipeline/safety',
        'module-04-vla-cognitive-pipeline/troubleshooting',
        'module-04-vla-cognitive-pipeline/module-summary-next-steps',
      ],
    },
  ],
};

export default sidebars;
