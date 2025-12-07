---
id: custom-messages-urdf
title: Custom Messages & URDF
sidebar_position: 2
---

# Chapter 2: Custom Messages & URDF

## Speaking Your Robot's Language

### Why Custom Messages?

Standard ROS 2 messages cover basic types (String, Int, Float), but real robots need structured data:

**Humanoid Status Message:**

Robot ID
Battery level
Joint temperatures (20+ joints)
Sensor health (camera, LIDAR, IMU)
Current mode (idle, walking, running)
Error codes


This doesn't fit into `std_msgs/String`!

### Creating Custom Messages

#### Step 1: Create Interface Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake robot_interfaces
cd robot_interfaces
mkdir msg srv action
```

#### Step 2: Define Message

Create `msg/RobotStatus.msg`:


Header with timestamp
std_msgs/Header header
Identification
string robot_id
string robot_name
Power system
float32 battery_level        # 0-100%
bool is_charging
float32 estimated_runtime    # minutes
Motion state
string current_mode          # IDLE, WALKING, RUNNING
geometry_msgs/Twist velocity
Sensors
bool camera_active
bool lidar_active
bool imu_active
Errors
int32 error_code
string error_message




#### Step 3: Configure CMakeLists.txt
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

#### Step 4: Configure package.xml
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

#### Step 5: Build
```bash
cd ~/ros2_ws
colcon build --packages-select robot_interfaces
source install/setup.bash

# Verify
ros2 interface show robot_interfaces/msg/RobotStatus
```

### Using Custom Messages
```python
from robot_interfaces.msg import RobotStatus

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.pub = self.create_publisher(RobotStatus, 'status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
    
    def publish_status(self):
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = 'HUMANOID_001'
        msg.battery_level = 75.5
        msg.is_charging = False
        msg.current_mode = 'WALKING'
        msg.camera_active = True
        self.pub.publish(msg)
```

---

## Understanding URDF

### What is URDF?

**Unified Robot Description Format** - XML file describing robot's physical structure:
- **Links** - Rigid body parts (torso, arms, legs)
- **Joints** - Connections (shoulder, elbow, knee)
- **Visual** - Appearance (colors, meshes)
- **Collision** - Simplified shapes for physics
- **Inertial** - Mass and inertia properties

### Why URDF Matters

**One URDF file works in:**
- RViz2 (visualization)
- Gazebo (simulation)
- MoveIt (motion planning)
- Nav2 (navigation)
- ros2_control (hardware interface)

### Basic Link
```xml
<link name="base_link">
  <!-- Visual: What you see -->
  <visual>
    <geometry>
      <box size="0.6 0.4 0.2"/>  <!-- width, depth, height (meters) -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>  <!-- RGBA -->
    </material>
  </visual>
  
  <!-- Collision: Simplified for physics -->
  <collision>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </collision>
  
  <!-- Inertial: Physical properties -->
  <inertial>
    <mass value="50.0"/>  <!-- kg -->
    <inertia ixx="1.0" ixy="0" ixz="0" 
             iyy="1.0" iyz="0" izz="1.0"/>
  </inertial>
</link>
```

### Joint Types

**1. Fixed** - No movement
```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

**2. Revolute** - Rotation with limits (elbow, knee)
```xml
<joint name="elbow" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <axis xyz="0 1 0"/>  <!-- Y-axis rotation -->
  <limit lower="0" upper="2.618" effort="100" velocity="2.0"/>
  <!-- 0 to 150 degrees -->
</joint>
```

**3. Continuous** - Unlimited rotation (wheels)
```xml
<joint name="wheel" type="continuous">
  <parent link="chassis"/>
  <child link="wheel_left"/>
  <axis xyz="0 0 1"/>
</joint>
```

**4. Prismatic** - Linear sliding
```xml
<joint name="telescope" type="prismatic">
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="1000" velocity="0.1"/>
</joint>
```

### Humanoid Arm Example
```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  
  <!-- Shoulder -->
  <link name="shoulder">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Upper Arm -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" 
               iyy="0.02" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  
  <joint name="shoulder_joint" type="revolute">
    <parent link="shoulder"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="2.0"/>
  </joint>
  
  <!-- Forearm -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.035"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.004"/>
    </inertial>
  </link>
  
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.618" effort="80" velocity="2.0"/>
  </joint>
  
</robot>
```

### Visualizing in RViz2
```bash
# Install tools
sudo apt install ros-humble-joint-state-publisher-gui

# Launch visualization
ros2 launch urdf_tutorial display.launch.py model:=/path/to/robot.urdf
```

This opens:
- RViz2 showing your robot
- GUI with sliders to move joints

### XACRO: Macros for URDF

**Problem:** Repetitive URDF code  
**Solution:** XACRO (XML Macros)
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Properties -->
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="arm_radius" value="0.04"/>
  
  <!-- Macro for reusable arm -->
  <xacro:macro name="arm" params="prefix reflect">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
      </visual>
    </link>
    
    <joint name="${prefix}_shoulder" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0 ${0.2 * reflect} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-3.14" upper="3.14" effort="100" velocity="2.0"/>
    </joint>
  </xacro:macro>
  
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.5"/>
      </geometry>
    </visual>
  </link>
  
  <!-- Create both arms -->
  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>
  
</robot>
```

Convert XACRO to URDF:
```bash
xacro robot.urdf.xacro > robot.urdf
```

### TF (Transform) System

TF manages coordinate frames. Every link has its own frame.

**Query transforms:**
```python
from tf2_ros import TransformListener, Buffer

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# Get hand position relative to base
transform = self.tf_buffer.lookup_transform(
    'base_link',    # target frame
    'left_hand',    # source frame
    rclpy.time.Time()
)

x = transform.transform.translation.x
y = transform.transform.translation.y
z = transform.transform.translation.z
```

### Best Practices

✅ **Use XACRO** for any robot with repeated structures  
✅ **Accurate inertias** - Calculate properly for simulation  
✅ **Collision != Visual** - Simplify collision meshes  
✅ **Proper joint limits** - Match real hardware  
✅ **Consistent naming** - `left_`, `right_` prefixes  

### Common Issues

❌ **Zero mass** - Simulation will crash  
❌ **Misaligned origins** - Robot will be twisted  
❌ **Missing collision** - Objects pass through  
❌ **Wrong joint axis** - Rotates in wrong direction  

### Practice Exercise

**Build a 4-wheeled robot:**
1. Create URDF with chassis + 4 wheels
2. Use continuous joints for wheels
3. Visualize in RViz2
4. Add a camera on top (fixed joint)

**Next →** [Chapter 3: Launch Files](./launch-files.md)