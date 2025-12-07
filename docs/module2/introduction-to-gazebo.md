---
id: introduction-to-gazebo
title: Introduction to Gazebo Simulation
sidebar_position: 1
---

# Chapter 1: Introduction to Gazebo Simulation

## Why Simulate Before Building?

### The Cost of Real Hardware

Building and testing physical robots is expensive:
- **Hardware costs:** $10k-100k+ for a humanoid robot
- **Damage risk:** Motors burn out, sensors break, frames crack
- **Iteration time:** Days to weeks per design change
- **Safety concerns:** Testing dangerous scenarios

**Simulation solves this** by providing a risk-free, cost-effective testing environment.

### What is Gazebo?

Gazebo is a 3D robot simulator that enables testing and debugging robot software and training robots in simulated environments. It's the industry standard for ROS-based robotics development.

**Key Features:**
- Physics-accurate simulation (gravity, friction, collisions)
- Sensor simulation (cameras, LIDAR, IMU, depth sensors)
- Plugin system for custom behaviors
- ROS 2 integration via `ros_gz_bridge`
- GPU acceleration for realistic rendering

### Gazebo History

**Gazebo Classic** (2004-2025)
- Original version, tightly coupled to ROS 1
- Being deprecated in 2025

**Gazebo Sim** (2019-present)
- Complete rewrite for modern robotics
- Also called "Ignition Gazebo" or just "Gazebo"
- We'll use **Gazebo Fortress** (compatible with ROS 2 Humble)

### Installation
```bash
# Install Gazebo Fortress
sudo apt update
sudo apt install ros-humble-ros-gz

# Verify installation
gz sim --version

# Should output: Gazebo Sim, version 7.x.x
```

### Your First Simulation

Launch a demo world with a robot that has visualized LIDAR:
```bash
# Start Gazebo with demo world
gz sim -v 4 -r visualize_lidar.sdf
```

**What you'll see:**
- 3D environment with ground plane
- Blue vehicle (diff-drive robot)
- LIDAR sensor visualization (red rays)
- Physics running in real-time

### Understanding SDF (Simulation Description Format)

Gazebo uses **SDF** (not URDF) for its internal format. SDF is more expressive than URDF.

**Basic SDF structure:**
```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="my_world">
    <!-- Lighting, physics settings -->
    
    <model name="my_robot">
      <!-- Links, joints, sensors -->
    </model>
    
  </world>
</sdf>
```

**URDF vs SDF:**
| Feature | URDF | SDF |
|---------|------|-----|
| Physics | Basic | Advanced (multiple engines) |
| Sensors | Limited | Comprehensive |
| Worlds | ❌ No | ✅ Yes |
| Nested Models | ❌ No | ✅ Yes |

**Good news:** You can convert URDF → SDF automatically!
```bash
gz sdf -p robot.urdf > robot.sdf
```

---

## Gazebo Topics and Communication

### Gazebo Transport Topics

Check topics provided by Gazebo using the ign command line tool:
```bash
gz topic -l
```

Output:

/clock
/model/vehicle_blue/odometry
/model/vehicle_blue/tf
/world/default/clock
/world/default/stats

### Echo Topic Data
```bash
# Monitor odometry
gz topic -e -t /model/vehicle_blue/odometry

# Monitor transforms
gz topic -t /model/vehicle_blue/tf -e
```

---

## Bridging Gazebo with ROS 2

### The ros_gz_bridge

The ros_gz_bridge package provides a network bridge enabling message exchange between ROS 2 and Gazebo Transport.

**Install:**
```bash
sudo apt install ros-humble-ros-gz-bridge
```

### Bridge Syntax

**Pattern:** `/TOPIC@ROS_MSG@GZ_MSG`
- `TOPIC` - Gazebo topic name
- `ROS_MSG` - ROS 2 message type
- `GZ_MSG` - Gazebo message type

**Direction markers:**
- `]` - ROS → Gazebo (publisher)
- `[` - Gazebo → ROS (subscriber)
- `@` - Bidirectional

### Example: Control Robot from ROS 2

Create a bridge for the cmd_vel topic to control the robot:
```bash
# Terminal 1: Start Gazebo
gz sim -v 4 -r visualize_lidar.sdf

# Terminal 2: Bridge cmd_vel (ROS → Gazebo)
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist

# Terminal 3: Control with keyboard
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel
```

**Now use keyboard to drive the robot:**
- `i` - Forward
- `k` - Stop  
- `j` - Turn left
- `l` - Turn right

### Example: LIDAR Data to ROS 2

Bridge the LIDAR sensor data from Gazebo to ROS 2:
```bash
# Bridge LIDAR topic
ros2 run ros_gz_bridge parameter_bridge \
  /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  --ros-args -r /lidar2:=/laser_scan

# Visualize in RViz2
rviz2
```

In RViz2:
1. Add → LaserScan
2. Topic: `/laser_scan`
3. Fixed Frame: `vehicle_blue`

---

## Gazebo GUI Basics

### Navigation
- **Orbit:** Right-click + drag
- **Pan:** Shift + right-click + drag
- **Zoom:** Scroll wheel
- **Select:** Left-click on models

### Top Toolbar
- **Play/Pause** - Control simulation time
- **Step** - Advance by one time step
- **Reset** - Return to initial state

### Left Panel - World Tree
- View all models in scene
- Expand to see links and joints
- Right-click for options (delete, rename)

### Plugins Panel (top-right icon grid)
- **Scene 3D** - Main view
- **Component Inspector** - Edit properties
- **Entity Tree** - Hierarchical view
- **Transform Control** - Move/rotate objects

---

## Creating Your First World

### Minimal Empty World

Create `empty_world.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty_world">
    
    <!-- Physics engine -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Sun light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

Launch:
```bash
gz sim empty_world.sdf
```

---

## Adding Models to World

### Insert Built-in Models

Gazebo comes with model libraries:
```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
</include>

<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
  <pose>1 0 0.5 0 0 0</pose>
</include>
```

### Fuel Library

Browse models at: **fuel.gazebosim.org**

Download model:
```bash
gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table
```

---

## Performance Tips

### Optimize Physics
```xml
<physics name="fast" type="ignored">
  <max_step_size>0.004</max_step_size>  <!-- Larger = faster -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### Disable Shadows
```xml
<light type="directional" name="sun">
  <cast_shadows>false</cast_shadows>  <!-- Faster rendering -->
</light>
```

### Use Simpler Collision Meshes
```xml
<collision name="collision">
  <geometry>
    <box><size>1 1 1</size></box>  <!-- Simple box instead of mesh -->
  </geometry>
</collision>
```

---

## Best Practices

✅ **Start simple** - Test with basic shapes before complex meshes  
✅ **Use realistic physics** - Match real-world parameters  
✅ **Profile performance** - Monitor FPS (should be >30)  
✅ **Version control SDF files** - Track changes with Git  
✅ **Modular worlds** - Use `<include>` for reusable components  

---

## Common Issues

**Problem:** Simulation runs slow  
**Solution:** Reduce physics step size, disable shadows, use simpler meshes

**Problem:** Models fall through ground  
**Solution:** Check ground plane has collision geometry

**Problem:** Camera view is black  
**Solution:** Add light source to world

**Problem:** Can't find models  
**Solution:** Set `GZ_SIM_RESOURCE_PATH` environment variable

---

## Practice Exercises

**1. Create Custom World:**
- Empty world with ground
- Add 3 obstacles (boxes, cylinders)
- Add lighting
- Test navigation space for robot

**2. Bridge Setup:**
- Launch your world
- Create bridge for odometry topic
- Echo in ROS 2 terminal
- Verify data flow

**3. Model Exploration:**
- Browse Fuel library
- Download 5 different models
- Create world with all 5
- Measure simulation performance (FPS)

**Next →** [Chapter 2: Physics Simulation](./physics-simulation.md)