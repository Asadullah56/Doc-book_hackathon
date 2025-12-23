import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',

    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/ros2-communication-backbone',
        'module-1/python-agents-rclpy',
        'module-1/urdf-humanoid-body',
      ],
    },

    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        // 'module-2/module-2',
        'module-2/chapter-1-gazebo-physics',
        'module-2/chapter-2-unity-visualization',
        'module-2/chapter-3-sensor-simulation',
      ],
    },

    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        // 'module-3/module-3',
        'module-3/chapter-1-photorealistic-intelligence',
        'module-3/chapter-2-seeing-the-world',
        'module-3/chapter-3-navigating-physical-world',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) & Capstone Project',
      items: [
        'module-4/chapter-1-voice-to-action',
        'module-4/chapter-2-cognitive-planning',
        'module-4/chapter-3-capstone-humanoid',
      ],
    },
  ],
};

export default sidebars;
