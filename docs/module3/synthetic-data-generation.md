---
id: synthetic-data-generation
title: Synthetic Data Generation
sidebar_position: 5
---

# Chapter 5: Synthetic Data Generation

## Why Synthetic Data?

**Problem:** Training AI needs millions of labeled images  
**Solution:** Generate perfect labels in simulation

### Benefits
- Infinite data (no collection cost)
- Perfect labels (no human error)
- Control all variables (lighting, poses)
- Rare scenarios (edge cases)

---

## Isaac Sim Replicator

### Basic Workflow
```python
import omni.replicator.core as rep

# Setup scene
with rep.new_layer():
    # Create objects
    cube = rep.create.cube(scale=0.5)
    
    # Randomize position
    with cube:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )
    
    # Create camera
    camera = rep.create.camera(position=(5, 5, 5), look_at=cube)
    rp = rep.create.render_product(camera, resolution=(1024, 1024))
    
    # Annotators (labels)
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="~/synthetic_data",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        distance_to_camera=True
    )
    writer.attach([rp])
    
    # Generate 1000 images
    with rep.trigger.on_frame(num_frames=1000):
        rep.randomizer.scatter_2d(cube, surface_prims=["/World/Ground"])

rep.orchestrator.run()
```

---

## Domain Randomization

### Lighting Randomization
```python
with rep.new_layer():
    lights = rep.create.light(
        light_type=rep.distribution.choice(["Sphere", "Disk", "Rect"]),
        temperature=rep.distribution.uniform(3000, 7000),
        intensity=rep.distribution.log_uniform(100, 10000),
        position=rep.distribution.uniform((-5, -5, 2), (5, 5, 10))
    )
```

### Texture Randomization
```python
with cube:
    rep.randomizer.color(
        colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1))
    )
    
    rep.randomizer.texture(
        texture_list=[
            "texture1.png",
            "texture2.png",
            "texture3.png"
        ]
    )
```

### Camera Randomization
```python
camera = rep.create.camera(
    position=rep.distribution.uniform((3, 3, 1), (7, 7, 3)),
    look_at=cube,
    focus_distance=rep.distribution.uniform(0.5, 5.0),
    f_stop=rep.distribution.uniform(1.8, 5.6)
)
```

---

## YCB Dataset Generation

### Download YCB Objects
```bash
# Get from Nucleus
omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Props/YCB
```

### Randomized Scenes
```python
import omni.replicator.core as rep

ycb_objects = [
    "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
    "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/004_sugar_box.usd",
    "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd",
    "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
]

with rep.new_layer():
    # Randomly select and place objects
    for obj_path in ycb_objects:
        obj = rep.create.from_usd(obj_path, semantics=[('class', obj_path.split('/')[-1].split('.')[0])])
        with obj:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.5, -0.5, 0.5), (0.5, 0.5, 1.0)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
    
    # Camera orbiting
    camera = rep.create.camera(focus_distance=1.0, f_stop=2.0)
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((1, 1, 0.5), (2, 2, 1.5)),
            look_at=(0, 0, 0.5)
        )
    
    rp = rep.create.render_product(camera, (640, 480))
    
    # Write annotations
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="~/ycb_dataset",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        instance_id_segmentation=True
    )
    writer.attach([rp])
    
    with rep.trigger.on_frame(num_frames=5000):
        rep.randomizer.scatter_2d(prims=obj, surface_prims="/World/Ground")

rep.orchestrator.run()
```

---

## COCO Format Export

### Convert to COCO JSON
```python
import json
import os
from pathlib import Path

def create_coco_annotations(data_dir):
    coco = {
        "images": [],
        "annotations": [],
        "categories": []
    }
    
    # Read replicator output
    for img_file in Path(data_dir).glob("rgb_*.png"):
        img_id = int(img_file.stem.split('_')[1])
        
        coco["images"].append({
            "id": img_id,
            "file_name": img_file.name,
            "height": 480,
            "width": 640
        })
        
        # Read bounding boxes from JSON
        bbox_file = img_file.with_name(f"bounding_box_2d_tight_{img_id:04d}.json")
        with open(bbox_file) as f:
            bboxes = json.load(f)
        
        for bbox in bboxes['data']:
            coco["annotations"].append({
                "id": len(coco["annotations"]),
                "image_id": img_id,
                "category_id": bbox['semanticId'],
                "bbox": [bbox['x_min'], bbox['y_min'], 
                         bbox['x_max'] - bbox['x_min'], 
                         bbox['y_max'] - bbox['y_min']],
                "area": (bbox['x_max'] - bbox['x_min']) * (bbox['y_max'] - bbox['y_min']),
                "iscrowd": 0
            })
    
    # Save
    with open(os.path.join(data_dir, "annotations.json"), 'w') as f:
        json.dump(coco, f)

create_coco_annotations("~/ycb_dataset")
```

---

## Training Pipeline

### 1. Generate Data (Isaac Sim)
```python
# Run replicator script
rep.orchestrator.run()  # Generates 10k images
```

### 2. Train Model (PyTorch)
```python
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg

cfg = get_cfg()
cfg.merge_from_file("configs/COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml")
cfg.DATASETS.TRAIN = ("ycb_train",)
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 10  # YCB objects

trainer = DefaultTrainer(cfg)
trainer.train()
```

### 3. Test in Isaac Sim
```python
from detectron2.engine import DefaultPredictor

predictor = DefaultPredictor(cfg)

# Get image from camera
rgb = camera.get_rgba()[:, :, :3]

# Inference
outputs = predictor(rgb)
boxes = outputs["instances"].pred_boxes
classes = outputs["instances"].pred_classes
scores = outputs["instances"].scores
```

---

## Best Practices

### 1. Realistic Ranges
```python
# ❌ Bad: Unrealistic
position=rep.distribution.uniform((-100, -100, 0), (100, 100, 50))

# ✅ Good: Robot's workspace
position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 1.5))
```

### 2. Balanced Dataset
```python
# Equal samples per class
objects_per_class = 1000
for obj_class in object_classes:
    with rep.trigger.on_frame(num_frames=objects_per_class):
        rep.randomizer.instantiate(obj_class)
```

### 3. Progressive Difficulty
```python
# Start simple, add complexity
stage1 = single_object + simple_background  # 1k images
stage2 = multiple_objects + cluttered_background  # 2k images
stage3 = occlusions + varied_lighting  # 3k images
```

---

## Practice Exercises

**1. Basic Generation:**
- Create scene with 10 cubes
- Randomize colors and positions
- Generate 100 RGB images
- Verify variety

**2. Object Detection Dataset:**
- Use 5 YCB objects
- 1000 images per object (5k total)
- Export as COCO format
- Train YOLOv8

**3. Segmentation Dataset:**
- Create warehouse scene
- Multiple objects on shelves
- Generate semantic segmentation
- Train U-Net for pixel classification

**4. Sim-to-Real:**
- Generate synthetic grasping dataset
- Train policy in simulation
- Test on real robot (if available)
- Measure success rate

---

## Conclusion: Module 3 Complete!

You've mastered:
✅ Isaac Sim photorealistic rendering  
✅ Isaac ROS GPU-accelerated perception  
✅ Nav2 navigation stack  
✅ Synthetic data generation at scale  

**Next →** [Module 4: Vision-Language-Action](../module4/vla-introduction.md)