---
id: isaac-sim
title: Isaac Sim - Photorealistic Simulation
sidebar_position: 2
---

# Chapter 2: Isaac Sim - Photorealistic Simulation

## Creating Realistic Environments

### Importing Assets

**From Omniverse Nucleus:**
```python
from isaacsim.core.utils.stage import add_reference_to_stage

add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Environments/Grid/default_environment.usd",
    prim_path="/World/Environment"
)
```

**From Local USD:**
```python
add_reference_to_stage(
    usd_path="/home/user/my_warehouse.usd",
    prim_path="/World/Warehouse"
)
```

---

## Lighting Systems

### HDR Environment Maps
```python
from pxr import UsdLux

stage = omni.usd.get_context().get_stage()
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1000)
dome_light.CreateTextureFileAttr("omniverse://localhost/NVIDIA/Assets/Skies/Clear/qwantani_4k.hdr")
```

### Directional Sun
```python
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(1500)
sun.CreateColorAttr((1.0, 0.95, 0.9))  # Warm sunlight
sun.AddOrientOp().Set((45, 30, 0))  # Angle
```

---

## Materials and Textures

### PBR Materials
```python
from pxr import UsdShade, Sdf

material = UsdShade.Material.Define(stage, "/World/Materials/MetalMaterial")
shader = UsdShade.Shader.Define(stage, "/World/Materials/MetalMaterial/Shader")

shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.8, 0.8, 0.8))
shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0)
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.2)

material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# Apply to cube
cube_prim = stage.GetPrimAtPath("/World/Cube")
UsdShade.MaterialBindingAPI(cube_prim).Bind(material)
```

---

## Robot Import

### From URDF
```python
from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.isaac.urdf")

import omni.isaac.core.utils.extensions as extensions_utils
from omni.importer.urdf import _urdf

urdf_interface = _urdf.acquire_urdf_interface()

result, prim_path = urdf_interface.parse_urdf(
    urdf_path="/path/to/robot.urdf",
    import_config=_urdf.ImportConfig(),
    dest_path="/World/Robot"
)
```

### Articulation Configuration
```python
from isaacsim.core.robots import Robot

robot = Robot(
    prim_path="/World/Robot",
    name="my_robot"
)

# Get joint names
joint_names = robot.dof_names
print(f"Joints: {joint_names}")

# Set joint positions
robot.set_joint_positions(
    positions=np.array([0, 0.5, -1.0, 0, 0.5, -1.0]),  # Radians
    joint_indices=[0, 1, 2, 3, 4, 5]
)
```

---

## Camera Sensors

### RGB Camera
```python
from isaacsim.sensors.camera import Camera

camera = Camera(
    prim_path="/World/Camera",
    position=np.array([3, 0, 1.5]),
    orientation=np.array([1, 0, 0, 0]),  # Quaternion
    frequency=30,
    resolution=(1920, 1080)
)

camera.initialize()
camera.add_motion_vectors_to_frame()  # For optical flow

# Get image
rgb_data = camera.get_rgba()[:, :, :3]  # Drop alpha
```

### Depth Camera
```python
camera.add_distance_to_image_plane_to_frame()
depth = camera.get_distance_to_image_plane()  # Meters
```

### Semantic Segmentation
```python
camera.add_semantic_segmentation_to_frame()
segmentation = camera.get_semantic_segmentation()
# Returns class ID per pixel
```

---

## LIDAR Sensors

### Rotating 3D LIDAR
```python
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.sensors import RotatingLidarPhysX

enable_extension("omni.isaac.range_sensor")

lidar = RotatingLidarPhysX(
    prim_path="/World/Lidar",
    name="lidar",
    position=np.array([0, 0, 1.0]),
    config="Velodyne_VLP16"  # Predefined configs
)

lidar.initialize()

# Get point cloud
point_cloud = lidar.get_current_frame()
# Shape: (N, 3) - X, Y, Z in meters
```

---

## Domain Randomization

### Randomize Lighting
```python
import random

def randomize_lighting():
    intensity = random.uniform(500, 2000)
    sun.GetIntensityAttr().Set(intensity)
    
    color_temp = random.uniform(0.8, 1.2)
    sun.GetColorAttr().Set((color_temp, 1.0, 1.0 / color_temp))
```

### Randomize Objects
```python
def randomize_scene():
    for i in range(10):
        cube = DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            position=np.random.uniform(-5, 5, 3),
            size=np.random.uniform(0.1, 0.5),
            color=np.random.rand(3)
        )
        world.scene.add(cube)
```

---

## Recording Data

### Save Images
```python
from PIL import Image

for i in range(100):
    world.step(render=True)
    
    rgb = camera.get_rgba()[:, :, :3]
    depth = camera.get_distance_to_image_plane()
    seg = camera.get_semantic_segmentation()
    
    Image.fromarray(rgb).save(f"rgb_{i:04d}.png")
    np.save(f"depth_{i:04d}.npy", depth)
    np.save(f"seg_{i:04d}.npy", seg)
```

---

## Replicator for Synthetic Data

### Basic Randomization
```python
import omni.replicator.core as rep

# Create randomizer
with rep.new_layer():
    # Randomize light
    light = rep.create.light(
        light_type="Distant",
        intensity=rep.distribution.uniform(500, 2000),
        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
    )
    
    # Randomize cubes
    cubes = rep.create.cube(
        count=10,
        position=rep.distribution.uniform((-5, 0, 0), (5, 0, 3)),
        scale=rep.distribution.uniform(0.1, 0.5)
    )
    
    # Attach camera
    camera = rep.create.camera(position=(10, 10, 10), look_at=(0, 0, 0))
    rp = rep.create.render_product(camera, (1024, 1024))
    
    # Write output
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir="output", rgb=True, bounding_box_2d_tight=True)
    writer.attach([rp])
    
    # Run
    rep.orchestrator.run()
```

---

## Performance Optimization

### Reduce Physics Steps
```python
from isaacsim.core import World

world = World(physics_dt=1.0/60.0, rendering_dt=1.0/30.0)
# Physics: 60 Hz, Rendering: 30 Hz
```

### Level of Detail (LOD)
```python
# Use simplified meshes for distant objects
prim.GetAttribute("lod:purpose").Set("proxy")
```

### Disable Expensive Features
```python
# Disable ray tracing
import carb

carb.settings.get_settings().set("/rtx/raytracing/enabled", False)
carb.settings.get_settings().set("/rtx/pathtracing/enabled", False)
```

---

## Practice Exercises

**1. Warehouse Scene:**
- Import warehouse USD
- Add 20 random boxes
- 3 cameras (front, left, right)
- Record 100 frames

**2. Robot Manipulation:**
- Import UR10 robot
- Position cube in front
- Move robot joints to grasp
- Record camera view

**3. Synthetic Dataset:**
- 100 scenes with random objects
- 5 camera angles per scene
- Save RGB + depth + segmentation
- Total: 500 images

**Next →** [Chapter 3: Isaac ROS](./isaac-ros-perception.md)