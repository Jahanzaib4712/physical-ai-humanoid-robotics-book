---
id: launch-files
title: Launch Files & Multi-Node Systems
sidebar_position: 3
---

# Chapter 3: Launch Files & Multi-Node Systems

## Orchestrating Complex Robot Systems

### The Problem

Starting a robot manually:
```bash
# Terminal 1
ros2 run camera_pkg camera_node

# Terminal 2
ros2 run lidar_pkg lidar_node

# Terminal 3
ros2 run perception_pkg detector

# Terminal 4... 5... 6... 20+
```

**This doesn't scale!**

### The Solution: Launch Files
```bash
# One command starts everything
ros2 launch my_robot robot.launch.py
```

Launch files:
- Start multiple nodes
- Set parameters
- Load configurations
- Handle dependencies
- Manage namespaces

### Launch File Formats

ROS 2 supports three formats:

**1. Python** (Most flexible, recommended)
**2. XML** (Declarative, simpler)
**3. YAML** (Minimal, rarely used)

We'll focus on Python launch files.

---

## Basic Python Launch File

Create `launch/simple.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            prefix='xterm -e',  # Opens in new terminal
            output='screen'
        ),
    ])
```

Run:
```bash
ros2 launch my_pkg simple.launch.py
```

---

## Advanced Launch Features

### 1. Parameters from File
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot'),
        'config',
        'robot_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='robot_node',
            name='robot',
            parameters=[config]
        ),
    ])
```

### 2. Remapping Topics
```python
Node(
    package='camera_pkg',
    executable='camera_node',
    name='front_camera',
    remappings=[
        ('/image_raw', '/front/image'),
        ('/camera_info', '/front/camera_info'),
    ]
)
```

### 3. Namespaces (Multi-Robot)
```python
# Robot 1
Node(
    package='robot_pkg',
    executable='controller',
    name='controller',
    namespace='robot1'
)

# Robot 2
Node(
    package='robot_pkg',
    executable='controller',
    name='controller',
    namespace='robot2'
)

# Topics become:
# /robot1/cmd_vel
# /robot2/cmd_vel
```

### 4. Conditionals
```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation'
        ),
        
        Node(
            package='gazebo_ros',
            executable='gazebo',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),
    ])
```

Run with argument:
```bash
ros2 launch my_pkg robot.launch.py use_sim:=true
```

### 5. Including Other Launch Files
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('sensors_pkg'),
            '/launch/sensors.launch.py'
        ])
    )
    
    return LaunchDescription([
        sensors_launch,
        Node(...),  # Your other nodes
    ])
```

---

## Real-World Example: Humanoid Robot Launch
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    pkg_dir = get_package_share_directory('humanoid_robot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Launch arguments
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Robot state publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
        output='screen'
    )
    
    # Joint state publisher (for moving joints in RViz)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_sim)
    )
    
    # Camera driver
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[config_file],
        remappings=[
            ('/camera/color/image_raw', '/image'),
            ('/camera/depth/image_raw', '/depth'),
        ]
    )
    
    # LIDAR driver
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='lidar',
        parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}]
    )
    
    # IMU driver
    imu_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[config_file]
    )
    
    # Perception
    object_detector = Node(
        package='perception_pkg',
        executable='yolo_detector',
        name='object_detector',
        parameters=[{'model_path': '/models/yolov8n.pt'}]
    )
    
    # Navigation
    nav_node = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='navigation'
    )
    
    # Motor controller
    motor_controller = Node(
        package='motor_control_pkg',
        executable='controller',
        name='motor_controller',
        parameters=[config_file]
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'robot.rviz')],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        
        # Launch nodes
        robot_state_publisher,
        joint_state_publisher,
        camera_node,
        lidar_node,
        imu_node,
        object_detector,
        nav_node,
        motor_controller,
        rviz_node,
    ])
```

Run:
```bash
ros2 launch humanoid_robot full_system.launch.py use_sim:=true
```

---

## Launch File Best Practices

### 1. Organize by Functionality

launch/
├── sensors.launch.py       # All sensors
├── perception.launch.py    # AI/vision
├── navigation.launch.py    # Nav2 stack
├── control.launch.py       # Motor control
└── full_system.launch.py   # Includes all above

### 2. Use Config Files

Don't hardcode parameters:
```python
# ❌ Bad
Node(..., parameters=[{'max_speed': 2.0, 'min_speed': 0.1}])

# ✅ Good
Node(..., parameters=[config_file])
```

### 3. Provide Launch Arguments
```python
DeclareLaunchArgument(
    'robot_name',
    default_value='atlas',
    description='Name of the robot'
)
```

### 4. Group Related Nodes
```python
from launch.actions import GroupAction

sensors = GroupAction([
    Node(...),  # Camera
    Node(...),  # LIDAR
    Node(...),  # IMU
])
```

---

## Debugging Launch Files

### Check Syntax
```bash
ros2 launch --show-args my_pkg robot.launch.py
```

### Print Launch Arguments
```python
print(f"Using simulation: {LaunchConfiguration('use_sim')}")
```

### Test Individual Nodes First
Before creating complex launch file, verify each node works:
```bash
ros2 run package_name node_name
```

### Use `--screen` for Output
```python
Node(..., output='screen')  # Shows logs in terminal
```

---

## Common Patterns

### Sequential Startup (Dependencies)
```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

# Start controller only after sensor is ready
controller_node = Node(...)
sensor_node = Node(...)

controller_start = RegisterEventHandler(
    OnProcessStart(
        target_action=sensor_node,
        on_start=[controller_node]
    )
)
```

### Lifecycle Management
```python
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition

lifecycle_node = LifecycleNode(
    package='my_pkg',
    executable='lifecycle_node',
    name='node'
)

# Auto-activate after configuring
activate = RegisterEventHandler(
    OnStateTransition(
        target_lifecycle_node=lifecycle_node,
        goal_state='inactive',
        entities=[
            LifecycleTransition(
                lifecycle_node_matcher=matches_action(lifecycle_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
            )
        ]
    )
)
```

---

## Practice Exercise

**Create a multi-robot system:**

1. **sensors.launch.py** - Camera, LIDAR, IMU
2. **robot1.launch.py** - Includes sensors with namespace `robot1`
3. **robot2.launch.py** - Includes sensors with namespace `robot2`
4. **multi_robot.launch.py** - Launches both robots

Test with:
```bash
ros2 launch my_pkg multi_robot.launch.py
ros2 topic list  # Should see /robot1/... and /robot2/...
```

*






