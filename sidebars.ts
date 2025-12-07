import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '📖 Course Introduction',
      items: [
        'introduction',
        'course-overview',
        'learning-outcomes',
        'hardware-requirements',
      ],
    },
    {
      type: 'category',
      label: '🤖 Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module1/introduction-to-ros2',
          label: 'Ch 1: Introduction to ROS 2',
        },
        {
          type: 'doc',
          id: 'module1/custom-messages-urdf',
          label: 'Ch 2: Custom Messages & URDF',
        },
        {
          type: 'doc',
          id: 'module1/launch-files',
          label: 'Ch 3: Launch Files & Multi-Node Systems',
        },
        {
          type: 'doc',
          id: 'module1/ros2-advanced',
          label: 'Ch 4: Advanced ROS 2 Concepts',
        },
        {
          type: 'doc',
          id: 'module1/python-ros2-integration',
          label: 'Ch 5: Python Integration with ROS 2',
        },
      ],
    },
    {
      type: 'category',
      label: '🎮 Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module2/introduction-to-gazebo',
          label: 'Ch 1: Introduction to Gazebo Simulation',
        },
        {
          type: 'doc',
          id: 'module2/physics-simulation',
          label: 'Ch 2: Physics, Gravity & Collisions',
        },
        {
          type: 'doc',
          id: 'module2/sensor-simulation',
          label: 'Ch 3: Simulating Sensors (LiDAR, Cameras, IMU)',
        },
        {
          type: 'doc',
          id: 'module2/unity-integration',
          label: 'Ch 4: Unity for High-Fidelity Rendering',
        },
        {
          type: 'doc',
          id: 'module2/ros2-gazebo-bridge',
          label: 'Ch 5: ROS 2 - Gazebo Bridge',
        },
      ],
    },
    {
      type: 'category',
      label: '🚀 Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module3/nvidia-isaac-overview',
          label: 'Ch 1: NVIDIA Isaac Platform Overview',
        },
        {
          type: 'doc',
          id: 'module3/isaac-sim',
          label: 'Ch 2: Isaac Sim - Photorealistic Simulation',
        },
        {
          type: 'doc',
          id: 'module3/isaac-ros-perception',
          label: 'Ch 3: Isaac ROS - VSLAM & Perception',
        },
        {
          type: 'doc',
          id: 'module3/nav2-path-planning',
          label: 'Ch 4: Nav2 - Navigation for Humanoids',
        },
        {
          type: 'doc',
          id: 'module3/synthetic-data-generation',
          label: 'Ch 5: Synthetic Data Generation',
        },
      ],
    },
    {
      type: 'category',
      label: '🗣️ Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module4/vla-introduction',
          label: 'Ch 1: Introduction to Vision-Language-Action',
        },
        {
          type: 'doc',
          id: 'module4/voice-to-action',
          label: 'Ch 2: Voice-to-Action with OpenAI Whisper',
        },
        {
          type: 'doc',
          id: 'module4/llm-cognitive-planning',
          label: 'Ch 3: LLMs for Cognitive Planning',
        },
        {
          type: 'doc',
          id: 'module4/computer-vision-integration',
          label: 'Ch 4: Computer Vision for Object Detection',
        },
        {
          type: 'doc',
          id: 'module4/capstone-project',
          label: 'Ch 5: Capstone - Autonomous Humanoid',
        },
      ],
    },
    {
      type: 'category',
      label: '🔧 Additional Resources',
      items: [
        'resources/hardware-setup-guide',
        'resources/software-installation',
        'resources/troubleshooting',
        'resources/best-practices',
        'resources/further-reading',
      ],
    },
    {
      type: 'category',
      label: '📝 Assessments & Projects',
      items: [
        'assessments/ros2-package-project',
        'assessments/gazebo-simulation-project',
        'assessments/isaac-perception-project',
        'assessments/final-capstone-project',
      ],
    },
  ],
};

export default sidebars;