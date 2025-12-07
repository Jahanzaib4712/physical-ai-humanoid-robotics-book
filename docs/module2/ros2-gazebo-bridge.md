---
id: ros2-gazebo-bridge
title: ROS 2 - Gazebo Bridge
sidebar_position: 5
---

# Chapter 5: ROS 2 - Gazebo Bridge

## Understanding the Bridge Architecture

### Why a Bridge?

Gazebo and ROS 2 use different communication systems:
- **Gazebo:** Gazebo Transport (formerly Ignition Transport)
- **ROS 2:** DDS middleware

The **ros_gz_bridge** translates messages between these systems.

### Communication Flow

ROS 2 Node → DDS → ros_gz_bridge → Gazebo Transport → Gazebo Simulation
←               ←                   ←                ←

---

## Installation
```bash
# Install bridge package
sudo apt install ros-humble-ros-gz-bridge

# Install Gazebo packages
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces

# Verify installation
ros2 pkg list | grep ros_gz
```

---

## Bridge Syntax

### Basic Pattern
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /TOPIC@ROS_MSG@GZ_MSG[DIRECTION]
```

### Direction Markers

- `]` - **ROS → Gazebo** (ROS publishes, Gazebo subscribes)
- `[` - **Gazebo → ROS** (Gazebo publishes, ROS subscribes)
- `@` - **Bidirectional** (both directions)

### Common Message Mappings

| ROS 2 Message | Gazebo Message | Typical Use |
|---------------|----------------|-------------|
| `geometry_msgs/msg/Twist` | `gz.msgs.Twist` | Velocity commands |
| `sensor_msgs/msg/Image` | `gz.msgs.Image` | Camera images |
| `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` | 2D LIDAR |
| `sensor_msgs/msg/PointCloud2` | `gz.msgs.PointCloudPacked` | 3D LIDAR |
| `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | IMU data |
| `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | Robot odometry |
| `sensor_msgs/msg/JointState` | `gz.msgs.Model` | Joint states |

---

## Common Bridge Examples

### 1. Control Robot (cmd_vel)

**Gazebo → ROS (read velocity commands):**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

**Test with keyboard:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/model/robot/cmd_vel
```

### 2. Camera Image

**Gazebo → ROS (publish images):**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image[gz.msgs.Image
```

**View in ROS:**
```bash
ros2 run rqt_image_view rqt_image_view /camera/image
```

### 3. LIDAR Data

**Gazebo → ROS (publish scans):**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

**Visualize in RViz:**
```bash
rviz2
# Add → LaserScan → Topic: /lidar
# Fixed Frame: laser
```

### 4. IMU Data

**Gazebo → ROS:**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU
```

### 5. Odometry

**Gazebo → ROS:**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

---

## Launch File Integration

### Single Bridge Node
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    
    return LaunchDescription([bridge])
```

### Multiple Bridge Nodes

For better organization:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Sensor bridges (Gazebo → ROS)
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )
    
    # Control bridge (ROS → Gazebo)
    control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='control_bridge',
        arguments=[
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen'
    )
    
    # State bridge (bidirectional)
    state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='state_bridge',
        arguments=[
            '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        sensor_bridge,
        control_bridge,
        state_bridge,
    ])
```

---

## Complete Robot System Launch

### Full System with Gazebo + Bridges + RViz
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot')
    world_file = os.path.join(pkg_dir, 'worlds', 'robot_world.sdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robot.rviz')
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Bridge all topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(os.path.join(pkg_dir, 'urdf', 'robot.urdf')).read()}]
    )
    
    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        rviz,
    ])
```

Launch:
```bash
ros2 launch my_robot full_system.launch.py
```

---

## Clock Synchronization

### Why Clock Matters

ROS 2 nodes need synchronized time for:
- TF transforms (coordinate frames)
- Sensor fusion
- Recording/playback (rosbag)

### Bridge Clock Topic
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Use Simulation Time
```bash
ros2 param set /my_node use_sim_time true
```

Or in launch file:
```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{'use_sim_time': True}]
)
```

---

## TF Transforms Bridge

### Publishing TF from Gazebo

Gazebo can publish TF directly:
```xml
<!-- In your SDF model -->
<plugin
  filename="gz-sim-pose-publisher-system"
  name="gz::sim::systems::PosePublisher">
  <publish_link_pose>true</publish_link_pose>
  <publish_collision_pose>false</publish_collision_pose>
  <publish_visual_pose>false</publish_visual_pose>
  <publish_nested_model_pose>true</publish_nested_model_pose>
</plugin>
```

Bridge TF:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
```

---

## Topic Remapping

### Rename Topics in Bridge
```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/gz_camera@sensor_msgs/msg/Image[gz.msgs.Image',
    ],
    remappings=[
        ('/gz_camera', '/camera/image'),  # Rename to standard name
    ]
)
```

---

## Custom Message Bridges

### When Standard Mappings Don't Exist

Create custom bridge in C++:
```cpp
#include <ros_gz_bridge/convert.hpp>

namespace ros_gz_bridge
{
template<>
void convert_ros_to_gz(
  const my_msgs::msg::CustomMsg & ros_msg,
  gz::msgs::CustomMsg & gz_msg)
{
  gz_msg.set_field1(ros_msg.field1);
  gz_msg.set_field2(ros_msg.field2);
}

template<>
void convert_gz_to_ros(
  const gz::msgs::CustomMsg & gz_msg,
  my_msgs::msg::CustomMsg & ros_msg)
{
  ros_msg.field1 = gz_msg.field1();
  ros_msg.field2 = gz_msg.field2();
}
}
```

---

## Debugging Bridge Issues

### Check Available Topics

**Gazebo topics:**
```bash
gz topic -l
```

**ROS 2 topics:**
```bash
ros2 topic list
```

### Echo Messages

**Gazebo:**
```bash
gz topic -e -t /camera/image
```

**ROS 2:**
```bash
ros2 topic echo /camera/image
```

### Verify Message Types
```bash
# Gazebo
gz topic -i -t /camera/image

# ROS 2
ros2 topic info /camera/image
```

### Common Issues

**Problem:** Bridge starts but no data flows  
**Solution:** Check direction markers (`[`, `]`, `@`)

**Problem:** Message type mismatch  
**Solution:** Verify both message types exist and are compatible

**Problem:** High latency  
**Solution:** Use direct bridges (not multiple hops)

**Problem:** Bridge crashes on startup  
**Solution:** Ensure Gazebo is fully started first

---

## Performance Optimization

### 1. Bridge Only What You Need

❌ **Bad:**
```python
# Bridge everything (slow)
arguments=[
    '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
    '/camera/info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    # ... 20 more topics
]
```

✅ **Good:**
```python
# Bridge only used topics
arguments=[
    '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',  # Actually used
]
```

### 2. Use Lazy Bridges

Start bridge only when topic has subscribers:
```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'lazy': True}],
    arguments=[...]
)
```

### 3. Reduce Message Frequency

In Gazebo SDF:
```xml
<sensor name="camera" type="camera">
  <update_rate>10</update_rate>  <!-- 10 Hz instead of 30 Hz -->
</sensor>
```

---

## Practice Exercises

**1. Basic Bridge Setup:**
- Launch Gazebo with demo world
- Bridge cmd_vel, odometry, LIDAR
- Control robot from ROS 2
- Visualize in RViz2

**2. Multi-Sensor System:**
- Create robot with 3 cameras + LIDAR + IMU
- Bridge all sensors
- Display all in RViz2
- Measure total bandwidth

**3. Custom World:**
- Create warehouse world in Gazebo
- Add mobile robot
- Bridge all necessary topics
- Implement simple autonomous navigation

**4. Performance Testing:**
- Bridge 10 high-res cameras (1920x1080 @ 30 FPS)
- Monitor CPU/memory usage
- Optimize by reducing resolution/rate
- Compare performance before/after

**Next →** [Module 3: NVIDIA Isaac](../module3/nvidia-isaac-overview.md)