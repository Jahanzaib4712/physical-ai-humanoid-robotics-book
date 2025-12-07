---
id: sensor-simulation
title: Simulating Sensors (LiDAR, Cameras, IMU)
sidebar_position: 3
---

# Chapter 3: Simulating Sensors (LiDAR, Cameras, IMU)

## Why Simulate Sensors?

Real sensors are expensive and have limitations:
- **LIDAR:** $1,000-$10,000+
- **Depth Camera:** $300-$500
- **IMU:** $50-$500

**Simulation provides:**
- Perfect ground truth data
- Configurable noise models
- Rapid prototyping
- Unlimited test scenarios

---

## Camera Sensors

### Basic RGB Camera
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>RGB_INT8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  
  <update_rate>30</update_rate>  <!-- 30 FPS -->
  
  <plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

### Camera Noise Model

Real cameras have noise. Add realistic noise:
```xml
<camera>
  <image>
    <width>1920</width>
    <height>1080</height>
  </image>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.007</stddev>  <!-- Standard deviation -->
  </noise>
</camera>
```

### Depth Camera (RGB-D)

Like Intel RealSense D435i:
```xml
<sensor name="rgbd_camera" type="rgbd_camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>RGB_INT8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  
  <depth_camera>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </depth_camera>
  
  <update_rate>30</update_rate>
</sensor>
```

**Output topics:**
- `/camera/image` - RGB image
- `/camera/depth` - Depth map (distance in meters)
- `/camera/points` - 3D point cloud

### Wide-Angle Camera

For omnidirectional vision:
```xml
<camera>
  <horizontal_fov>3.14159</horizontal_fov>  <!-- 180 degrees -->
  <image>
    <width>1280</width>
    <height>720</height>
  </image>
  <distortion>
    <k1>-0.25</k1>  <!-- Barrel distortion -->
    <k2>0.12</k2>
    <k3>0.0</k3>
    <p1>0.0</p1>
    <p2>0.0</p2>
    <center>0.5 0.5</center>
  </distortion>
</camera>
```

---

## LIDAR Sensors

### 2D LIDAR (Laser Scanner)

Like RPLidar A1/A2:
```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.1 0 0 0</pose>
  <topic>lidar</topic>
  <update_rate>10</update_rate>
  
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>  <!-- 360 rays -->
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180° -->
        <max_angle>3.14159</max_angle>   <!-- +180° -->
      </horizontal>
    </scan>
    
    <range>
      <min>0.2</min>  <!-- 20cm minimum -->
      <max>12.0</max> <!-- 12m maximum -->
      <resolution>0.01</resolution>  <!-- 1cm resolution -->
    </range>
    
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm noise -->
    </noise>
  </lidar>
  
  <plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

### 3D LIDAR (Spinning)

Like Velodyne VLP-16:
```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <topic>lidar_3d</topic>
  <update_rate>10</update_rate>
  
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>  <!-- Points per rotation -->
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>  <!-- 16 vertical channels -->
        <resolution>1.0</resolution>
        <min_angle>-0.2618</min_angle>  <!-- -15° -->
        <max_angle>0.2618</max_angle>   <!-- +15° -->
      </vertical>
    </scan>
    
    <range>
      <min>0.5</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  
  <plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

### Visualizing LIDAR

Enable visualization in SDF:
```xml
<lidar>
  <visualize>true</visualize>  <!-- Shows rays in Gazebo -->
</lidar>
```

Or use RViz2:
```bash
# Bridge LIDAR topic
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# Launch RViz2
rviz2
# Add → LaserScan → Topic: /lidar
```

---

## IMU (Inertial Measurement Unit)

### Standard IMU Configuration
```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>  <!-- 100 Hz -->
  <topic>imu</topic>
  
  <imu>
    <!-- Accelerometer -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- m/s² -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
    
    <!-- Gyroscope -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00008</stddev>  <!-- rad/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00008</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00008</stddev>
        </noise>
      </z>
    </angular_velocity>
  </imu>
  
  <plugin
    filename="gz-sim-imu-system"
    name="gz::sim::systems::Imu">
  </plugin>
</sensor>
```

**IMU provides:**
- **Linear acceleration** (m/s²) in X, Y, Z
- **Angular velocity** (rad/s) around X, Y, Z
- **Orientation** (quaternion) - computed from integration

---

## GPS Sensor
```xml
<sensor name="gps" type="gps">
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <topic>gps</topic>
  
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0</stddev>  <!-- 2m horizontal error -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>3.0</stddev>  <!-- 3m vertical error -->
        </noise>
      </vertical>
    </position_sensing>
    
    <velocity_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>  <!-- 0.1 m/s velocity error -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
      </vertical>
    </velocity_sensing>
  </gps>
  
  <plugin
    filename="gz-sim-navsat-system"
    name="gz::sim::systems::NavSat">
  </plugin>
</sensor>
```

---

## Force/Torque Sensor

For measuring contact forces in manipulation:
```xml
<sensor name="force_torque" type="force_torque">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <topic>ft_sensor</topic>
  
  <force_torque>
    <frame>child</frame>  <!-- or "parent" or "sensor" -->
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
  
  <plugin
    filename="gz-sim-forcetorque-system"
    name="gz::sim::systems::ForceTorque">
  </plugin>
</sensor>
```

Attach between two links:
```xml
<joint name="wrist_ft" type="fixed">
  <parent>forearm</parent>
  <child>hand</child>
  <sensor name="wrist_force" type="force_torque">
    <!-- Configuration above -->
  </sensor>
</joint>
```

---

## Contact Sensor (Bumper)

Detect collisions:
```xml
<sensor name="bumper" type="contact">
  <contact>
    <collision>body_collision</collision>
  </contact>
  <update_rate>100</update_rate>
  <topic>bumper</topic>
  
  <plugin
    filename="gz-sim-contact-system"
    name="gz::sim::systems::Contact">
  </plugin>
</sensor>
```

**Output:** List of collision points with forces.

---

## Multi-Sensor Fusion Example

Humanoid robot with all sensors:
```xml
<model name="humanoid">
  <link name="head">
    <!-- RGB Camera -->
    <sensor name="camera_front" type="camera">
      <pose>0.1 0 0 0 0 0</pose>
      <!-- Configuration... -->
    </sensor>
    
    <!-- Depth Camera -->
    <sensor name="depth_camera" type="rgbd_camera">
      <pose>0.1 0 0.05 0 0 0</pose>
      <!-- Configuration... -->
    </sensor>
  </link>
  
  <link name="torso">
    <!-- IMU for balance -->
    <sensor name="imu_torso" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <!-- Configuration... -->
    </sensor>
    
    <!-- 2D LIDAR for navigation -->
    <sensor name="lidar_nav" type="gpu_lidar">
      <pose>0 0 0.3 0 0 0</pose>
      <!-- 2D configuration... -->
    </sensor>
  </link>
  
  <link name="left_foot">
    <!-- Force sensor for ground contact -->
    <sensor name="force_left_foot" type="force_torque">
      <!-- Configuration... -->
    </sensor>
  </link>
</model>
```

---

## Bridging Sensors to ROS 2

### Launch File with All Bridges
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridges = [
        # Camera image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen'
        ),
        
        # LIDAR
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        ),
        
        # IMU
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            output='screen'
        ),
        
        # GPS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
            output='screen'
        ),
    ]
    
    return LaunchDescription(bridges)
```

---

## Sensor Calibration in Simulation

### Camera Calibration Matrix
```xml
<camera>
  <image>
    <width>640</width>
    <height>480</height>
  </image>
  <intrinsics>
    <fx>554.254691</fx>  <!-- Focal length X -->
    <fy>554.254691</fy>  <!-- Focal length Y -->
    <cx>320.5</cx>       <!-- Principal point X -->
    <cy>240.5</cy>       <!-- Principal point Y -->
    <s>0.0</s>           <!-- Skew -->
  </intrinsics>
</camera>
```

### Sensor TF Frames

Each sensor should publish its transform:
```xml
<plugin
  filename="gz-sim-pose-publisher-system"
  name="gz::sim::systems::PosePublisher">
  <publish_link_pose>true</publish_link_pose>
  <publish_sensor_pose>true</publish_sensor_pose>
</plugin>
```

---

## Performance Optimization

### GPU vs CPU Rendering
```xml
<!-- GPU (faster, recommended) -->
<sensor name="camera" type="camera">
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>  <!-- GPU -->
  </plugin>
</sensor>

<!-- CPU (slower, fallback) -->
<sensor name="camera" type="camera">
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre</render_engine>  <!-- CPU -->
  </plugin>
</sensor>
```

### Reduce Update Rate
```xml
<!-- High rate (expensive) -->
<update_rate>100</update_rate>

<!-- Lower rate (cheaper, usually sufficient) -->
<update_rate>30</update_rate>
```

### Limit Sensor Range
```xml
<range>
  <max>10.0</max>  <!-- Instead of 100.0 -->
</range>
```

---

## Realistic Noise Models

### Why Add Noise?

Perfect sensors in simulation lead to algorithms that fail in reality. Add realistic noise!

### Types of Noise

**Gaussian (most common):**
```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

**Uniform:**
```xml
<noise type="uniform">
  <min>-0.01</min>
  <max>0.01</max>
</noise>
```

### Calibrating Noise from Real Sensors

1. Collect real sensor data (stationary robot)
2. Calculate standard deviation: `σ = √(Σ(x - μ)² / N)`
3. Use `σ` as `stddev` in simulation

---

## Practice Exercises

**1. Camera Setup:**
- Add RGB camera to robot
- Bridge to ROS 2
- View in `rqt_image_view`
- Test different FOV values (60°, 90°, 120°)

**2. LIDAR Obstacle Detection:**
- Add 2D LIDAR
- Create world with obstacles
- Subscribe to `/lidar` in ROS 2
- Find nearest obstacle in each direction

**3. IMU Orientation Tracking:**
- Add IMU to robot
- Rotate robot manually in Gazebo
- Plot orientation (roll, pitch, yaw) in Python
- Compare with ground truth from model pose

**4. Multi-Sensor Fusion:**
- Setup robot with camera + LIDAR + IMU
- Bridge all three to ROS 2
- Create visualization in RViz2
- Verify sensor synchronization

**Next →** [Chapter 4: Unity Integration](./unity-integration.md)