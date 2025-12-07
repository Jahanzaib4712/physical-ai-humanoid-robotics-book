---
id: hardware-requirements
title: Hardware Requirements
sidebar_position: 4
---

# Hardware Requirements

## Understanding the Computational Demands

Physical AI sits at the intersection of three computationally intensive tasks:

1. **Physics Simulation** (Isaac Sim, Gazebo) - Rigid body dynamics, collisions
2. **Visual Perception** (SLAM, Computer Vision) - Real-time sensor processing
3. **Generative AI** (LLMs, VLAs) - Large neural network inference

This creates unique hardware requirements compared to typical software development or even standard AI work.

---

## Option 1: High-Performance Local Workstation (Recommended)

### Minimum Specifications

**GPU** (Critical Bottleneck)
- NVIDIA RTX 4070 Ti (12GB VRAM) - **Minimum**
- NVIDIA RTX 4080 (16GB VRAM) - **Recommended**
- NVIDIA RTX 4090 (24GB VRAM) - **Ideal**

Why RTX specifically?
- Ray-tracing cores for photorealistic simulation
- Tensor cores for AI acceleration
- CUDA support for Isaac Sim/Gazebo

**CPU**
- Intel Core i7-13700K or AMD Ryzen 9 7900X - **Minimum**
- 8+ performance cores required
- Physics calculations are heavily CPU-bound

**RAM**
- 64GB DDR5 - **Minimum**
- 128GB - **Recommended** for complex scenes

Why so much?
- Isaac Sim loads entire USD scenes into memory
- Running simulation + perception + LLM simultaneously
- Gazebo with multiple robots requires 8-16GB per instance

**Storage**
- 1TB NVMe SSD - **Minimum** (Isaac Sim alone: ~200GB)
- 2TB - **Recommended** for datasets

**Operating System**
- Ubuntu 22.04 LTS - **Required**
- Dual-boot with Windows acceptable
- WSL2 NOT sufficient (graphics limitations)

### Estimated Cost
- **Budget Build:** $2,500-3,500 (RTX 4070 Ti)
- **Recommended Build:** $3,500-4,500 (RTX 4080)
- **Professional Build:** $5,000-7,000 (RTX 4090)

---

## Option 2: Cloud-Based Development

If you don't have high-end hardware, cloud instances are viable:

### AWS Configuration

**Instance Type:** g5.2xlarge (A10G GPU, 24GB VRAM)
- **CPU:** 8 vCPUs
- **RAM:** 32GB
- **GPU:** NVIDIA A10G (24GB)
- **Cost:** ~$1.50/hour (spot instances)

**Monthly Budget (10 hours/week):**
- 10 hours × 4 weeks = 40 hours
- 40 × $1.50 = **$60/month**
- **Quarter Total:** ~$180

### Azure Configuration

**Instance Type:** NC A10 v4
- Similar specs to AWS g5.2xlarge
- ~$1.70/hour

### Setup Steps
1. Create Isaac Sim AMI with dependencies pre-installed
2. Use VSCode Remote-SSH for development
3. Download trained models locally (avoid repeated inference costs)

### Cloud Limitations
- **Latency:** Not suitable for real-time hardware control
- **Data Transfer:** Large datasets expensive to upload/download
- **Pricing:** Can exceed local hardware cost over 1-2 years

---

## Option 3: Hybrid Approach (Best Value)

Combine local and cloud resources:

**Local Machine ($1,500-2,000):**
- Mid-range GPU (RTX 4060 Ti, 8GB)
- 32GB RAM
- For development, debugging, lightweight simulation

**Cloud When Needed:**
- Large-scale training runs
- Photorealistic Isaac Sim rendering
- Batch synthetic data generation

---

## Edge Computing Kit (For Physical Deployment)

Even if you primarily simulate, you'll need edge hardware to understand real-world constraints:

### NVIDIA Jetson Orin Nano Super Dev Kit

**Specs:**
- 8GB RAM
- 40 TOPS AI performance
- **Price:** $249 (official MSRP, 2024)

**What It's For:**
- Deploying trained models to robot hardware
- Understanding edge AI optimization
- Testing real sensor integration

### Sensor Kit

**Intel RealSense D435i**
- RGB + Depth camera with IMU
- **Price:** ~$350
- Industry standard for robotics

**Optional Additions:**
- USB Microphone array (ReSpeaker): $70
- LiDAR (RPLidar A1): $99

**Total Edge Kit Cost:** ~$700

---

## Software Costs

### Free (Open Source)
- ✅ ROS 2 Humble
- ✅ Gazebo Fortress
- ✅ NVIDIA Isaac Sim (free for students/educators)
- ✅ PyTorch, TensorFlow

### Paid Services (Optional)
- OpenAI API (GPT-4): ~$20-50/month for development
- Claude API: ~$20-40/month
- Cloud storage: ~$10-20/month

**Estimated Software Budget:** $50-100/month (if using paid LLM APIs)

---

## Recommended Configurations by Budget

### **Budget: Under $2,000**
- Cloud-based development (AWS/Azure)
- No local GPU workstation
- Borrow/rent Jetson for edge testing
- **Best for:** Short-term learning, exploring field

### **Budget: $3,000-4,000**
- RTX 4070 Ti local workstation
- 64GB RAM, 1TB SSD
- Jetson Orin Nano + RealSense
- **Best for:** Serious students, portfolio building

### **Budget: $5,000+**
- RTX 4080/4090 workstation
- 128GB RAM, 2TB SSD
- Full edge kit + multiple sensors
- **Best for:** Professional development, startup founders

---

## Installation Checklist

Once you have hardware, install:
```bash
# 1. Ubuntu 22.04 LTS
# 2. NVIDIA Drivers (550+)
# 3. ROS 2 Humble
sudo apt install ros-humble-desktop-full

# 4. Gazebo Fortress
sudo apt install ros-humble-gazebo-ros-pkgs

# 5. NVIDIA Isaac Sim (via Omniverse Launcher)
# Download from: developer.nvidia.com/isaac-sim

# 6. Python Environment
sudo apt install python3-pip
pip3 install torch torchvision numpy opencv-python

# 7. Development Tools
sudo apt install git build-essential cmake
```

---

## Performance Benchmarks

Expected performance on recommended hardware:

| Task | RTX 4070 Ti | RTX 4080 | RTX 4090 |
|------|-------------|----------|----------|
| Isaac Sim (1 robot) | 30-40 FPS | 50-60 FPS | 60+ FPS |
| Gazebo (5 robots) | 20-30 FPS | 40-50 FPS | 60 FPS |
| VLA Inference | 100ms | 80ms | 50ms |
| Synthetic Data Gen | 500 images/min | 800 images/min | 1200 images/min |

---

## Can I Use Mac?

**Short Answer:** No, not for this course.

**Why:**
- Isaac Sim requires NVIDIA RTX GPU
- ROS 2 native support best on Linux
- Metal (Apple GPU) not compatible with CUDA

**Alternatives:**
- Dual-boot Mac with Linux (if Intel-based)
- Use cloud workstations
- Wait for M-series ROS 2 support (experimental as of 2024)

---

## Questions?

**Q: Can I start without a GPU?**  
A: Yes, Modules 1-2 work on CPU-only, but Module 3-4 require GPU.

**Q: Will my gaming laptop work?**  
A: If it has RTX 3070+ and 32GB+ RAM, yes for learning. Not ideal for production.

**Q: Should I buy now or wait?**  
A: RTX 50-series expected mid-2025. Current gen (40-series) excellent value and sufficient.

**Next →** [Module 1: Introduction to ROS 2](./module1/introduction-to-ros2.md)