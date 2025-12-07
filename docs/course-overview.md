---
id: course-overview
title: Course Overview
sidebar_position: 2
---

# Course Overview

## Physical AI & Humanoid Robotics

### Course Vision

This capstone course integrates artificial intelligence, robotics, and simulation to prepare you for the emerging field of embodied AI. Unlike traditional software AI that exists only in digital space, Physical AI operates in the real world—navigating obstacles, manipulating objects, and collaborating with humans.

### Learning Journey

#### **Phase 1: Foundations (Weeks 1-5)**
**Module 1: ROS 2 - The Robotic Nervous System**

You'll master Robot Operating System 2, the industry standard for robot software architecture. Think of ROS 2 as the "nervous system" that connects a robot's "brain" (AI algorithms) to its "body" (sensors and actuators).

**Key Skills:**
- Node architecture and inter-process communication
- Custom message types for robot-specific data
- URDF robot description format
- Parameter management and launch files
- Integration with Python AI libraries

**Deliverable:** Multi-node robot system with sensors, perception, and control

---

#### **Phase 2: Digital Twins (Weeks 6-7)**
**Module 2: Gazebo & Unity Simulation**

Before deploying expensive robot hardware, you'll test everything in simulation. This module covers physics-accurate simulation environments.

**Key Skills:**
- Gazebo simulation environment setup
- Physics simulation (gravity, friction, collisions)
- Sensor simulation (LIDAR, cameras, depth sensors)
- Unity integration for photorealistic rendering
- ROS 2 ↔ Simulator bridges

**Deliverable:** Simulated robot navigating dynamic environment

---

#### **Phase 3: AI Perception (Weeks 8-10)**
**Module 3: NVIDIA Isaac Platform**

NVIDIA Isaac brings GPU-accelerated AI to robotics. You'll use photorealistic simulation, hardware-accelerated SLAM, and synthetic data generation.

**Key Skills:**
- Isaac Sim for photorealistic robot simulation
- Isaac ROS for perception pipelines (VSLAM, object detection)
- Nav2 navigation stack for autonomous movement
- Sim-to-real transfer techniques
- Reinforcement learning for robot control

**Deliverable:** AI-powered perception and navigation system

---

#### **Phase 4: Cognitive Robotics (Weeks 11-13)**
**Module 4: Vision-Language-Action (VLA)**

The culmination: integrating Large Language Models for cognitive planning. Your robot will understand natural language commands and translate them into physical actions.

**Key Skills:**
- Voice command processing (OpenAI Whisper)
- LLM-based task planning (GPT-4, Claude)
- Vision-language models for scene understanding
- Action primitive mapping
- Multi-modal robot interaction

**Deliverable:** Voice-controlled autonomous humanoid (CAPSTONE)

---

### Assessment Structure

**Continuous Assessment (60%)**
- Weekly coding assignments (20%)
- Module projects (40%)
  - ROS 2 Package Development
  - Gazebo Simulation Implementation
  - Isaac Perception Pipeline

**Capstone Project (40%)**
- Autonomous humanoid robot with conversational AI
- Must demonstrate:
  - Voice command understanding
  - Autonomous navigation
  - Object detection and manipulation
  - Learned behavior from demonstration

---

### Tools & Technologies

**Software Stack:**
- **OS:** Ubuntu 22.04 LTS
- **Middleware:** ROS 2 Humble
- **Simulators:** Gazebo Fortress, Unity 2022, NVIDIA Isaac Sim
- **AI Frameworks:** PyTorch, TensorFlow, Isaac Lab
- **Languages:** Python 3.10+, C++17
- **Voice:** OpenAI Whisper, Riva
- **LLMs:** GPT-4, Claude Sonnet

**Hardware (Required):**
- **Workstation:** RTX 4070 Ti+ GPU, 64GB RAM, Ubuntu 22.04
- **Edge Device:** NVIDIA Jetson Orin Nano (for deployment)
- **Sensors:** Intel RealSense D435i (depth camera)

---

### Weekly Time Commitment

- **Lectures:** 3 hours/week (video + live sessions)
- **Labs:** 4-6 hours/week (hands-on coding)
- **Reading:** 2-3 hours/week (documentation + papers)
- **Project Work:** 4-6 hours/week

**Total:** 13-18 hours/week

---

### Career Pathways

This course prepares you for:

**Roles:**
- Robotics Software Engineer
- AI Robotics Research Engineer
- Autonomous Systems Developer
- Simulation Engineer
- Physical AI Engineer

**Companies Hiring:**
- Tesla (Optimus), Boston Dynamics, Figure AI
- NVIDIA, Amazon Robotics, Sanctuary AI
- Agility Robotics, 1X Technologies
- Traditional manufacturers (BMW, Mercedes, Toyota)

**Salary Range:** $120k-$250k+ (US market, 2024)

---

### Support & Resources

**Office Hours:** Tuesdays & Thursdays, 5-6 PM PKT  
**Discussion Forum:** Available 24/7  
**Lab Access:** Remote GPU clusters provided  
**Cloud Credits:** $300 AWS/Azure credits per student  

Ready to transform from AI developer to robotics engineer? Let's begin!

**Next →** [Learning Outcomes](./learning-outcomes.md)