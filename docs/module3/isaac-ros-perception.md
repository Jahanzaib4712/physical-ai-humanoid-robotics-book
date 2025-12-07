---
id: isaac-ros-perception
title: Isaac ROS - VSLAM & Perception
sidebar_position: 3
---

# Chapter 3: Isaac ROS - VSLAM & Perception

## What is Isaac ROS?

GPU-accelerated ROS 2 packages for:
- **Visual SLAM** (cuVSLAM)
- **Stereo depth** (ESS)
- **Object detection** (DOPE, Pose CNN)
- **Image segmentation** (U-Net)

**Speed:** 10-100x faster than CPU versions

---

## Installation

### Prerequisites
```bash
# NVIDIA Container Runtime
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Isaac ROS Common
```bash
cd ~/workspaces
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

cd isaac_ros_common
./scripts/run_dev.sh
```

---

## Visual SLAM (cuVSLAM)

### What is VSLAM?

**Visual Simultaneous Localization and Mapping:**
- Uses camera to estimate robot pose
- Builds 3D map of environment
- No LIDAR needed

### Installation
```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

cd ~/workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_visual_slam
source install/setup.bash
```

### Launch with RealSense
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Topics

**Subscribed:**
- `/camera/infra1/image_rect_raw` - Left image
- `/camera/infra2/image_rect_raw` - Right image
- `/camera/infra1/camera_info` - Camera calibration

**Published:**
- `/visual_slam/tracking/odometry` - Pose estimate
- `/visual_slam/tracking/vo_pose` - Visual odometry
- `/visual_slam/tracking/slam_path` - Trajectory
- `/visual_slam/vis/landmarks_cloud` - 3D map points

### Visualize in RViz
```bash
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam)/share/isaac_ros_visual_slam/rviz/default.cfg.rviz
```

---

## Stereo Depth (ESS)

### ESS (Embedded Stereo System)

DNN-based stereo depth estimation:
- Input: Stereo image pair
- Output: Depth map
- Speed: 30+ FPS @ 1920x1080

### Installation
```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth.git

colcon build --packages-up-to isaac_ros_ess
source install/setup.bash
```

### Launch
```bash
ros2 launch isaac_ros_ess isaac_ros_ess.launch.py
```

### Using with Isaac Sim
```python
# In Isaac Sim
from isaacsim.sensors import Camera

left_camera = Camera(
    prim_path="/World/CameraLeft",
    position=np.array([2, -0.1, 1]),
    frequency=30
)

right_camera = Camera(
    prim_path="/World/CameraRight",
    position=np.array([2, 0.1, 1]),
    frequency=30
)
```

Bridge to ROS 2:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera_left/rgb@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_right/rgb@sensor_msgs/msg/Image[gz.msgs.Image
```

---

## Object Detection (DOPE)

### DOPE (Deep Object Pose Estimation)

Detects 6D pose of known objects:
- Position (X, Y, Z)
- Orientation (quaternion)

### Supported Objects

- YCB objects (mustard, cracker box, etc.)
- Custom trained models

### Launch
```bash
ros2 launch isaac_ros_dope isaac_ros_dope.launch.py model_file_path:=/path/to/model.pth object_name:=Ketchup
```

### Output
```bash
ros2 topic echo /dope/pose_array
# geometry_msgs/PoseArray with detected objects
```

---

## Image Segmentation (U-Net)

### Semantic Segmentation

Classify each pixel:
- Road, sidewalk, building, person, car, etc.

### Installation
```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation.git

colcon build --packages-up-to isaac_ros_unet
source install/setup.bash
```

### Launch
```bash
ros2 launch isaac_ros_unet isaac_ros_unet_tensor_rt.launch.py model_file_path:=/path/to/model.onnx engine_file_path:=/path/to/model.plan
```

---

## Integrating with Isaac Sim

### Complete Pipeline

**Isaac Sim → ROS 2 → Isaac ROS → Navigation**
```python
# Isaac Sim: Generate stereo images
left_cam = Camera("/World/StereoLeft", frequency=30)
right_cam = Camera("/World/StereoRight", frequency=30)

# Enable ROS bridge
enable_extension("omni.isaac.ros2_bridge")
```
```bash
# Launch Isaac ROS pipeline
ros2 launch my_robot perception.launch.py
```
```python
# perception.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Visual SLAM
        Node(package='isaac_ros_visual_slam', executable='visual_slam_node'),
        
        # Stereo Depth
        Node(package='isaac_ros_ess', executable='ess_node'),
        
        # Object Detection
        Node(package='isaac_ros_dope', executable='dope_node'),
        
        # Segmentation
        Node(package='isaac_ros_unet', executable='unet_node'),
    ])
```

---

## Nvblox (3D Reconstruction)

### What is Nvblox?

GPU-accelerated 3D reconstruction:
- Fuses depth images into voxel grid
- Creates 3D occupancy map
- Outputs 2D costmap for Nav2

### Launch
```bash
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

### Output Topics

- `/nvblox_node/mesh` - 3D mesh visualization
- `/nvblox_node/map_slice` - 2D costmap for navigation

---

## Performance Comparison

| Algorithm | CPU (FPS) | Jetson Orin (FPS) | Speedup |
|-----------|-----------|-------------------|---------|
| Visual SLAM | 5-10 | 30-60 | 6-12x |
| Stereo Depth | 1-2 | 30+ | 15-30x |
| Object Detection | 5-10 | 30+ | 3-6x |
| Segmentation | 2-5 | 30+ | 6-15x |

---

## Practice Exercises

**1. VSLAM Setup:**
- Launch Isaac Sim with stereo cameras
- Run cuVSLAM
- Drive robot (teleop)
- Visualize trajectory in RViz

**2. Depth Estimation:**
- Setup ESS node
- Subscribe to depth output
- Find closest obstacle
- Trigger stop if < 0.5m

**3. Object Detection:**
- Train DOPE on custom object
- Place object in Isaac Sim
- Detect 6D pose
- Command robot to grasp

**Next →** [Chapter 4: Nav2](./nav2-path-planning.md)