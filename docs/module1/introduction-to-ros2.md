---
id: introduction-to-ros2
title: Introduction to ROS 2
sidebar_position: 1
---

# Chapter 1: Introduction to ROS 2

## The Brain Behind Modern Robots

### What is ROS 2?

Robot Operating System 2 (ROS 2) is the industry-standard middleware framework for building robot software. Despite its name, it's not an operating system—it's a communication layer that enables different parts of your robot to work together seamlessly.

**Think of it as:** The nervous system connecting your robot's brain (AI algorithms) to its body (sensors and actuators).

### Why ROS 2 Matters

Modern robots are complex distributed systems:
- **Sensors:** Cameras, LIDAR, IMU, force sensors
- **Processors:** Multiple computers (edge devices, GPUs, cloud)
- **Actuators:** Motors, grippers, displays, speakers
- **AI Models:** Perception, planning, control algorithms

ROS 2 provides:
✅ **Standardized communication** between components  
✅ **Language flexibility** (Python, C++, and more)  
✅ **Hardware abstraction** (same code works on different robots)  
✅ **Massive ecosystem** (thousands of existing packages)  
✅ **Industry adoption** (Tesla, BMW, NASA, Boston Dynamics)

### From ROS 1 to ROS 2

**ROS 1** (2007-2020):
- ❌ Single point of failure (master node required)
- ❌ Not real-time capable
- ❌ Poor security (no encryption)
- ❌ TCP-only networking

**ROS 2** (2017-present):
- ✅ Distributed architecture (no master)
- ✅ Real-time capable (DDS middleware)
- ✅ Built-in security (encryption, authentication)
- ✅ Multi-protocol networking (UDP, shared memory)
- ✅ Better Windows/embedded support

### Core Concepts

#### 1. Nodes - Independent Workers

A **node** is a single-purpose program that performs one task:
```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot')
        self.get_logger().info('Node initialized!')

def main():
    rclpy.init()
    node = MyRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

#### 2. Topics - Message Highways

Topics enable **publish-subscribe** communication:
```python
# Publisher
self.publisher = self.create_publisher(String, 'robot_status', 10)
msg = String()
msg.data = 'Robot operational'
self.publisher.publish(msg)

# Subscriber
self.subscription = self.create_subscription(
    String, 'robot_status', self.callback, 10)
```

#### 3. Services - Request-Response

Services provide **synchronous** communication:
```python
# Server
self.srv = self.create_service(AddTwoInts, 'add', self.callback)

# Client
self.client = self.create_client(AddTwoInts, 'add')
request = AddTwoInts.Request()
request.a = 5
request.b = 3
future = self.client.call_async(request)
```

#### 4. Actions - Long Tasks with Feedback

Actions handle tasks that take time:
```python
# Action for "Navigate to position X, Y"
# - Goal: Target coordinates
# - Feedback: Current progress (50% there...)
# - Result: Success or failure
```

### Installation (Ubuntu 22.04)
```bash
# Add ROS 2 repository
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS 2 Humble
sudo apt install ros-humble-desktop

# Source setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

### Your First ROS 2 System

Test with built-in demo:
```bash
# Terminal 1: Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener
ros2 run demo_nodes_py listener

# Terminal 3: Inspect system
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```

### Workspace Setup
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

# Source workspace
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Key Tools
```bash
# Node operations
ros2 node list          # List active nodes
ros2 node info /node    # Node details

# Topic operations
ros2 topic list         # List topics
ros2 topic echo /topic  # Monitor messages
ros2 topic pub /topic   # Publish manually

# Service operations
ros2 service list       # List services
ros2 service call /srv  # Call service

# Visualization
rqt_graph              # Node graph
rviz2                  # 3D visualization
```

### Real-World Architecture

Typical humanoid robot ROS 2 architecture:



Camera Node → Image Processing → Object Detection
↓
LIDAR Node → Point Cloud → Obstacle Detection → Path Planner → Motor Control
↓
IMU Node → Orientation Tracking → Balance Controller

### Best Practices

1. **One node, one purpose** - Keep nodes focused
2. **Use namespaces** - Organize multi-robot systems
3. **Parameterize** - Avoid hardcoding values
4. **Log properly** - Use logging levels (info, warn, error)
5. **Handle shutdown** - Clean up resources gracefully

### Common Mistakes

❌ **Too many topics** - Causes network congestion  
❌ **Blocking callbacks** - Slows entire system  
❌ **Ignoring QoS** - Leads to message loss  
❌ **No error handling** - Crashes entire robot  

### Industry Usage

**Tesla Bot (Optimus):** ROS 2 for perception and control  
**Boston Dynamics:** Custom ROS 2 packages  
**NASA Rovers:** ROS 2 for autonomy (future missions)  
**BMW Factories:** ROS 2-based assembly robots  

### What's Next

In Chapter 2, we'll create:
- Custom message types for robot data
- URDF robot descriptions
- Parameter management systems
- Multi-node launch files

### Practice Exercise

**Build a robot status monitor:**
1. Create a node that publishes battery level (0-100)
2. Create a node that subscribes and warns if < 20%
3. Visualize with `rqt_plot`

**Next →** [Chapter 2: Custom Messages & URDF](./custom-messages-urdf.md)