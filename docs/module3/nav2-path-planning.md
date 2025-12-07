---
id: nav2-path-planning
title: Nav2 - Navigation for Humanoids
sidebar_position: 4
---

# Chapter 4: Nav2 - Navigation for Humanoids

## What is Nav2?

ROS 2 navigation stack:
- Path planning (A*, DWB)
- Obstacle avoidance
- Localization (AMCL)
- Behavior trees

**Use for:** Mobile robots, humanoids, drones

---

## Architecture

Sensors → Costmap → Global Planner → Local Planner → cmd_vel
↑
Localization (AMCL/SLAM)

---

## Installation
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

---

## Configuration Files

### nav2_params.yaml
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

costmap:
  global_costmap:
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 1.0
    publish_frequency: 1.0
    static_map: true
    rolling_window: false
    resolution: 0.05
    
  local_costmap:
    global_frame: odom
    robot_base_frame: base_link
    update_frequency: 5.0
    publish_frequency: 2.0
    rolling_window: true
    width: 3
    height: 3
    resolution: 0.05
```

---

## Launch Nav2
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_yaml = os.path.join(get_package_share_directory('my_robot'), 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'controller_server', 'planner_server', 'bt_navigator']
            }]
        ),
    ])
```

---

## Sending Navigation Goals

### Command Line
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### Python Action Client
```python
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class NavClient(Node):
    def __init__(self):
        super().__init__('nav_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

# Usage
nav = NavClient()
nav.send_goal(5.0, 2.0, 0.0)  # x, y, yaw
```

---

## Humanoid Adaptations

### Foot Collision Checking
```yaml
footprint: [
  [0.15, 0.1],
  [0.15, -0.1],
  [-0.15, -0.1],
  [-0.15, 0.1]
]
```

### Slower, Careful Movement
```yaml
max_vel_x: 0.3  # Slower than wheeled robots
max_vel_theta: 0.5
```

### Balance Constraints
```python
# Custom controller considering ZMP
class HumanoidController:
    def compute_velocity(self, goal):
        cmd_vel = super().compute_velocity(goal)
        
        # Check if command maintains balance
        if not self.is_stable(cmd_vel):
            cmd_vel.linear.x *= 0.5  # Slow down
            
        return cmd_vel
```

---

## With Isaac Sim

### Complete System
```python
# Isaac Sim
from isaacsim.core.robots import Robot
robot = Robot("/World/Humanoid")

# Enable sensors
camera = Camera("/World/Humanoid/camera", frequency=30)
lidar = RotatingLidarPhysX("/World/Humanoid/lidar")

# Enable ROS 2
enable_extension("omni.isaac.ros2_bridge")
```
```bash
# Launch Nav2
ros2 launch my_robot nav2.launch.py

# Send goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0}}}}"
```

---

## Practice Exercises

**1. Basic Navigation:**
- Launch Nav2 in simulation
- Use RViz to set goal
- Watch robot navigate
- Test obstacle avoidance

**2. Waypoint Following:**
- Define 5 waypoints
- Navigate sequentially
- Measure time per segment

**3. Dynamic Obstacles:**
- Add moving objects in sim
- Test Nav2 avoidance
- Tune costmap parameters

**Next →** [Chapter 5: Synthetic Data](./synthetic-data-generation.md)