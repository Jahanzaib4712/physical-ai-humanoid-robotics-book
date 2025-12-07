id: python-ros2-integration
title: Python Integration with ROS 2
sidebar_position: 5
---

# Chapter 5: Python Integration with ROS 2

## Bridging AI and Robotics

### Why Python for Robotics?

Python is the language of modern AI/ML, and ROS 2's Python support (`rclpy`) enables seamless integration:

✅ **NumPy, PyTorch, TensorFlow** - AI frameworks  
✅ **OpenCV, PIL** - Computer vision  
✅ **SciPy** - Scientific computing  
✅ **Faster prototyping** than C++  

### rclpy vs rclcpp

| Feature        | Python (rclpy)       | C++ (rclcpp)       |
| -------------- | -------------------- | ------------------ |
| Speed          | Slower               | Faster             |
| Development    | Rapid                | Verbose            |
| AI Integration | Excellent            | Manual             |
| Use Case       | Perception, Planning | Control, Real-time |

**Best Practice:** Python for high-level logic, C++ for low-level control.

---

## Integrating Computer Vision

### Example: Object Detection with YOLO
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import torch

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        self.get_logger().info('YOLO Detector initialized')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]  # Pandas DataFrame
        
        # Create detection message
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        for _, detection in detections.iterrows():
            det_msg = Detection2D()
            det_msg.bbox.center.x = (detection['xmin'] + detection['xmax']) / 2
            det_msg.bbox.center.y = (detection['ymin'] + detection['ymax']) / 2
            det_msg.bbox.size_x = detection['xmax'] - detection['xmin']
            det_msg.bbox.size_y = detection['ymax'] - detection['ymin']
            
            det_msg.results[0].id = str(int(detection['class']))
            det_msg.results[0].score = detection['confidence']
            
            detection_array.detections.append(det_msg)
        
        self.detection_pub.publish(detection_array)
        
        # Visualize (optional)
        annotated = results.render()[0]
        cv2.imshow('Detections', annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Working with Images

### CvBridge: ROS ↔ OpenCV
```python
from cv_bridge import CvBridge

bridge = CvBridge()

# ROS Image → OpenCV
cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

# OpenCV → ROS Image
ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
```

### Common Encodings

- `bgr8` - Standard OpenCV format
- `rgb8` - RGB (need conversion from OpenCV)
- `mono8` - Grayscale
- `32FC1` - Depth images (float32)

---

## NumPy Integration

### Point Cloud Processing
```python
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pc_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pc_callback,
            10
        )
    
    def pc_callback(self, msg):
        # Convert to NumPy array
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_array = np.array(list(points))
        
        # Process: Remove ground plane
        non_ground = points_array[points_array[:, 2] > 0.1]  # Z > 0.1m
        
        # Cluster points
        from sklearn.cluster import DBSCAN
        clustering = DBSCAN(eps=0.3, min_samples=10).fit(non_ground)
        
        self.get_logger().info(f'Found {len(set(clustering.labels_))} clusters')
```

---

## PyTorch Integration

### Running Neural Network Inference
```python
import torch
import torchvision.transforms as transforms
from PIL import Image as PILImage

class NeuralNetNode(Node):
    def __init__(self):
        super().__init__('neural_net')
        
        # Load model
        self.model = torch.hub.load('pytorch/vision', 'resnet50', pretrained=True)
        self.model.eval()
        
        # Preprocessing
        self.transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        
        # GPU if available
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        
        self.subscription = self.create_subscription(Image, '/camera/image', self.callback, 10)
    
    def callback(self, msg):
        # ROS → PIL → Tensor
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        input_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)
        
        # Inference
        with torch.no_grad():
            output = self.model(input_tensor)
        
        # Get prediction
        probabilities = torch.nn.functional.softmax(output[0], dim=0)
        top_prob, top_class = torch.max(probabilities, dim=0)
        
        self.get_logger().info(f'Predicted class: {top_class.item()}, confidence: {top_prob.item():.2f}')
```

---

## Async Operations

### Using Python's asyncio
```python
import asyncio
from rclpy.executors import MultiThreadedExecutor

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')
        self.timer = self.create_timer(1.0, self.async_timer_callback)
    
    async def fetch_data(self):
        # Simulate async I/O
        await asyncio.sleep(0.5)
        return "Data from async operation"
    
    def async_timer_callback(self):
        # Run async code
        loop = asyncio.get_event_loop()
        data = loop.run_until_complete(self.fetch_data())
        self.get_logger().info(f'Received: {data}')

def main():
    rclpy.init()
    node = AsyncNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
```

---

## External Libraries

### Using OpenAI API in ROS 2
```python
import openai
from std_msgs.msg import String

class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot')
        
        # Set API key
        openai.api_key = self.declare_parameter('openai_key', '').value
        
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )
        
        self.response_pub = self.create_publisher(String, '/chatbot_response', 10)
    
    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        
        # Call GPT-4
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful robot assistant."},
                {"role": "user", "content": msg.data}
            ]
        )
        
        answer = response.choices[0].message['content']
        
        # Publish response
        response_msg = String()
        response_msg.data = answer
        self.response_pub.publish(response_msg)
        
        self.get_logger().info(f'Response: {answer}')
```

---

## Threading and Concurrency

### Using Threading for Blocking Operations
```python
import threading

class ThreadedNode(Node):
    def __init__(self):
        super().__init__('threaded_node')
        
        # Start background thread
        self.thread = threading.Thread(target=self.background_task)
        self.thread.daemon = True
        self.thread.start()
    
    def background_task(self):
        while rclpy.ok():
            # Long-running task
            result = self.expensive_computation()
            self.get_logger().info(f'Computed: {result}')
            time.sleep(5)
    
    def expensive_computation(self):
        # Simulate heavy computation
        import time
        time.sleep(2)
        return 42
```

---

## Package Structure Best Practices
````
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   ├── perception_node.py
│   │   ├── control_node.py
│   │   └── planning_node.py
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── image_processing.py
│   │   └── transformations.py
│   └── models/
│       └── yolov5s.pt
├── launch/
│   └── robot.launch.py
├── config/
│   └── params.yaml
├── setup.py
├── package.xml
└── README.md

setup.py Configuration
pythonfrom setuptools import setup
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot package with Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = my_robot_pkg.nodes.perception_node:main',
            'control = my_robot_pkg.nodes.control_node:main',
        ],
    },
)

Error Handling
pythonclass RobustNode(Node):
    def callback(self, msg):
        try:
            # Process message
            result = self.process(msg)
        except ValueError as e:
            self.get_logger().error(f'Invalid data: {e}')
            return
        except Exception as e:
            self.get_logger().fatal(f'Unexpected error: {e}')
            raise  # Critical error, shutdown
        
        # Publish result
        self.publisher.publish(result)

Practice Exercises
1. Face Detection Node:

Subscribe to camera
Use OpenCV Haar Cascades
Publish bounding boxes

2. Voice Command Processor:

Use speech_recognition library
Convert audio → text
Publish as String message

3. Multi-Model Inference:

Load 2 PyTorch models (detection + segmentation)
Run inference on same image
Compare timing

