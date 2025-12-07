---
id: nvidia-isaac-overview
title: NVIDIA Isaac Platform Overview
sidebar_position: 1
---

# Chapter 1: NVIDIA Isaac Platform Overview

## What is NVIDIA Isaac?

NVIDIA Isaac is a complete robotics platform consisting of three main components:

**1. Isaac Sim** - Photorealistic robot simulator  
**2. Isaac ROS** - GPU-accelerated ROS 2 packages  
**3. Isaac Lab** - Reinforcement learning for robots  

### Why Isaac Over Gazebo?

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| Graphics | Good | Photorealistic (RTX) |
| Physics | CPU-based | GPU-accelerated |
| AI Integration | Manual | Native (TensorRT) |
| Synthetic Data | Limited | Production-ready |
| Speed | 1-10 FPS | 30-60 FPS |

**Isaac Sim = Gazebo + Unity + AI Tools combined**

---

## System Requirements

### Minimum Setup
- **GPU:** NVIDIA RTX 3070 (8GB VRAM)
- **CPU:** Intel i7 / AMD Ryzen 7
- **RAM:** 32GB
- **OS:** Ubuntu 22.04
- **Driver:** 550+

### Recommended Setup
- **GPU:** RTX 4080 (16GB VRAM)
- **RAM:** 64GB
- **Storage:** 500GB SSD

---

## Installation

### Method 1: Omniverse Launcher (Recommended)
```bash
# Download launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run
./omniverse-launcher-linux.AppImage
```

Inside Launcher:
1. Install **Omniverse Cache** (speeds up loading)
2. Install **Isaac Sim 4.5.0**
3. Launch Isaac Sim

### Method 2: Docker (Cloud/CI)
```bash
# Pull image
docker pull nvcr.io/nvidia/isaac-sim:4.5.0

# Run container
docker run --gpus all -it \
  -v ~/isaac-sim-cache:/root/.cache/ov:rw \
  nvcr.io/nvidia/isaac-sim:4.5.0
```

---

## Isaac Sim Interface

### Main Windows

**Stage** (left) - Scene hierarchy tree  
**Viewport** (center) - 3D visualization  
**Property** (right) - Object details  
**Content** (bottom) - Asset browser  

### Navigation
- **Orbit:** Right-click + drag
- **Pan:** Middle-click + drag
- **Zoom:** Scroll wheel
- **Select:** Left-click

---

## USD (Universal Scene Description)

Isaac Sim uses **OpenUSD** (Pixar's format):
```python
from pxr import Usd, UsdGeom

# Create stage
stage = Usd.Stage.CreateNew("my_scene.usd")

# Add cube
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.AddTranslateOp().Set((0, 0, 1))
cube.GetSizeAttr().Set(1.0)

# Save
stage.Save()
```

**USD advantages:**
- Non-destructive editing (layers)
- Massive scenes (lazy loading)
- Industry standard (used in movies)

---

## Your First Simulation

### Python Script
```python
from isaacsim import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from isaacsim.core import World
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add falling cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0, 0, 2.0]),
        size=0.5,
        color=np.array([1, 0, 0])
    )
)

# Reset and run
world.reset()

for i in range(1000):
    world.step(render=True)
    if i % 100 == 0:
        pos = cube.get_world_pose()[0]
        print(f"Cube position: {pos}")

simulation_app.close()
```

Run:
```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.5.0
./python.sh /path/to/script.py
```

---

## ROS 2 Integration

Isaac Sim includes ROS 2 bridge:

### Enable ROS 2
```python
from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.isaac.ros2_bridge")
```

### Publish Camera
```python
from isaacsim.sensors import Camera

camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2, 0, 1]),
    frequency=30
)

# Automatically publishes to /rgb
camera.initialize()
```

Subscribe in ROS 2:
```bash
ros2 topic echo /rgb
```

---

## Isaac Sim vs Isaac ROS

**Isaac Sim:**
- Simulation environment
- Runs locally or in cloud
- Generates data

**Isaac ROS:**
- ROS 2 packages (VSLAM, perception)
- Runs on Jetson/x86
- Processes real sensor data

**Together:** Train in Sim → Deploy with ROS

---

## Key Concepts

### Physics Engine
- **PhysX 5** - NVIDIA's GPU physics
- 10-100x faster than CPU
- Supports articulations (humanoids)

### RTX Rendering
- Ray-traced lighting
- Photorealistic shadows
- Path-traced global illumination

### Replicator
- Synthetic data generation
- Domain randomization
- Perfect labels (segmentation, depth)

---

## Practice Exercises

**1. Hello World:**
- Launch Isaac Sim GUI
- Add cube + sphere from Content
- Press Play, watch physics

**2. Python Script:**
- Create 10 random cubes
- Different colors and sizes
- Run simulation for 500 steps

**3. ROS Bridge:**
- Add camera to scene
- Enable ROS 2 extension
- Subscribe to /rgb in ROS 2

**Next →** [Chapter 2: Isaac Sim Deep Dive](./isaac-sim.md)