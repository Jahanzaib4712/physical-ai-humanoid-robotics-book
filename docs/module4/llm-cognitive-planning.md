---

## FILE: docs/module4/llm-cognitive-planning.md
```markdown
---
id: llm-cognitive-planning
title: LLMs for Cognitive Planning
sidebar_position: 3
---

# Chapter 3: LLMs for Cognitive Planning

## The AI Brain for Robots

**Traditional Robotics:** Hardcoded rules and state machines  
**Modern Approach:** LLMs as cognitive reasoning engines
```
"Clean the kitchen"
       ↓
   [LLM Planner]
       ↓
Step 1: Identify dirty dishes
Step 2: Pick up each dish
Step 3: Place in dishwasher
Step 4: Wipe counters
Step 5: Take out trash

Why LLMs for Planning?
Advantages
✅ Common sense reasoning: Understands implicit knowledge
✅ Flexible planning: Adapts to new situations
✅ Natural language interface: No programming needed
✅ Context awareness: Considers environment state
✅ Error recovery: Can replan when failures occur
Example: Traditional vs LLM
Traditional (Hardcoded):
pythonif task == "set_table":
    pick_plate()
    place_at_position([0.5, 0, 0.8])
    pick_fork()
    place_at_position([0.3, 0, 0.8])
    # Must code every step explicitly
LLM-based:
pythonplan = llm.generate_plan("Set the table for dinner")
# LLM outputs:
# 1. Pick up plate from cabinet
# 2. Place plate at center of table
# 3. Get fork and knife from drawer
# 4. Arrange utensils on left and right of plate
# 5. Add napkin folded on left
for step in plan:
    execute(step)

LLM Selection for Robotics
Model Comparison
ModelParametersSpeedReasoningCostGPT-4~1.7TSlowExcellentHighClaude Sonnet 4~500BFastExcellentMediumGPT-3.5175BVery fastGoodLowLLaMA 3 70B70BFastGoodFree (self-hosted)Mistral 7B7BVery fastDecentFree
For robotics:

Development: GPT-4 or Claude (best reasoning)
Deployment: LLaMA 3 70B (self-hosted, fast)


Basic Task Planning
Using OpenAI API
pythonimport openai
from openai import OpenAI

client = OpenAI(api_key="your-api-key")

class LLMPlanner:
    def __init__(self):
        self.system_prompt = """
You are a robotic task planner. Given a high-level task, break it down 
into atomic robot actions. Available actions:
- navigate(location)
- pick(object)
- place(object, location)
- open(container)
- close(container)

Output as JSON list of steps.
"""
    
    def generate_plan(self, task, scene_description):
        response = client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"Task: {task}\nScene: {scene_description}"}
            ],
            temperature=0.7
        )
        
        plan_text = response.choices[0].message.content
        return self.parse_plan(plan_text)
    
    def parse_plan(self, plan_text):
        """Convert LLM output to executable actions"""
        import json
        try:
            plan = json.loads(plan_text)
            return plan
        except:
            # Fallback: parse natural language
            return self.parse_natural_language(plan_text)

# Usage
planner = LLMPlanner()
scene = "Kitchen with dirty dishes on counter, dishwasher is open"
task = "Load the dishwasher"

plan = planner.generate_plan(task, scene)
print(plan)
# Output:
# [
#   {"action": "navigate", "params": {"location": "counter"}},
#   {"action": "pick", "params": {"object": "plate"}},
#   {"action": "navigate", "params": {"location": "dishwasher"}},
#   {"action": "place", "params": {"object": "plate", "location": "rack"}},
#   ...
# ]

Few-Shot Prompting
Improving Plan Quality
pythonclass ImprovedLLMPlanner:
    def __init__(self):
        self.system_prompt = """
You are a robotic task planner. Generate safe, efficient plans.

Examples:

Task: "Make coffee"
Scene: "Coffee machine on counter, mug in cabinet, coffee beans in pantry"
Plan:
1. navigate(cabinet)
2. open(cabinet_door)
3. pick(mug)
4. close(cabinet_door)
5. navigate(pantry)
6. open(pantry_door)
7. pick(coffee_beans)
8. close(pantry_door)
9. navigate(coffee_machine)
10. place(coffee_beans, machine_hopper)
11. place(mug, machine_platform)
12. press(brew_button)
13. wait(30)

Task: "Tidy living room"
Scene: "Books on couch, remote on floor, pillows scattered"
Plan:
1. navigate(couch)
2. pick(book_1)
3. navigate(bookshelf)
4. place(book_1, shelf)
5. navigate(couch)
6. pick(book_2)
7. navigate(bookshelf)
8. place(book_2, shelf)
9. navigate(floor)
10. pick(remote)
11. navigate(coffee_table)
12. place(remote, table)
13. navigate(couch)
14. pick(pillow)
15. place(pillow, couch_corner)

Now generate a plan for the given task.
"""

ReAct Pattern (Reasoning + Acting)
Interleaved Reasoning and Actions
pythonclass ReActPlanner:
    def plan_with_reasoning(self, task, observations):
        messages = [
            {"role": "system", "content": "You are a robot. Think step by step."},
            {"role": "user", "content": f"Task: {task}\nObservations: {observations}"}
        ]
        
        plan = []
        max_steps = 10
        
        for step in range(max_steps):
            response = client.chat.completions.create(
                model="gpt-4",
                messages=messages
            )
            
            text = response.choices[0].message.content
            messages.append({"role": "assistant", "content": text})
            
            # Parse thought and action
            if "Thought:" in text and "Action:" in text:
                thought = text.split("Action:")[0].replace("Thought:", "").strip()
                action = text.split("Action:")[1].strip()
                
                plan.append({"thought": thought, "action": action})
                
                # Execute action and get observation
                observation = self.execute_action(action)
                messages.append({"role": "user", "content": f"Observation: {observation}"})
                
                if "DONE" in observation:
                    break
        
        return plan

# Example output:
# Step 1:
#   Thought: I need to find the object first
#   Action: navigate(table)
#   Observation: At table, see red cup and blue bowl
# 
# Step 2:
#   Thought: Task asks for red cup, I see it on table
#   Action: pick(red_cup)
#   Observation: Grasped red cup successfully
# 
# Step 3:
#   Thought: Now need to deliver to person
#   Action: navigate(person)
#   Observation: Reached person
#
# Step 4:
#   Thought: Hand over the cup
#   Action: handover(red_cup)
#   Observation: DONE - Cup handed to person

Chain-of-Thought (CoT) Reasoning
Explicit Reasoning Steps
pythoncot_prompt = """
Task: Pick up the apple from the table and place it in the bowl

Let's think step by step:
1. Where is the apple? The apple is on the table.
2. Where is the bowl? The bowl is on the counter.
3. What do I need to do first? Navigate to the table.
4. Then what? Pick up the apple.
5. Then? Navigate to the counter.
6. Finally? Place the apple in the bowl.

Plan:
1. navigate(table)
2. pick(apple)
3. navigate(counter)
4. place(apple, bowl)
"""

response = client.chat.completions.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "Generate plans with reasoning"},
        {"role": "user", "content": "Task: Put the book on the shelf"}
    ]
)

Error Recovery with LLMs
Replanning on Failure
pythonclass RobustPlanner:
    def execute_with_recovery(self, plan):
        for i, step in enumerate(plan):
            success = self.robot.execute(step)
            
            if not success:
                # Get failure reason
                error = self.robot.get_last_error()
                
                # Ask LLM to replan
                remaining_task = f"Continue from step {i+1}: {plan[i:]}"
                context = f"Previous step failed: {error}"
                
                new_plan = self.llm.generate_plan(remaining_task, context)
                
                # Continue with new plan
                return self.execute_with_recovery(new_plan)
        
        return True  # Success

# Example:
# Original plan: [navigate(table), pick(cup), ...]
# Step 2 fails: "Cup is too slippery, grasp failed"
# LLM replans: [adjust_gripper_force(), pick(cup), ...]

Grounding LLM Actions
Mapping Language to Robot Primitives
pythonclass ActionGrounder:
    def __init__(self):
        self.action_map = {
            "pick": self.pick_action,
            "place": self.place_action,
            "navigate": self.navigate_action,
            "open": self.open_action,
        }
    
    def ground_action(self, llm_action):
        """Convert LLM action string to robot command"""
        # Parse: "pick(red_cup)"
        action_name = llm_action.split("(")[0]
        params = llm_action.split("(")[1].rstrip(")").split(",")
        
        # Map to robot primitive
        if action_name in self.action_map:
            return self.action_map[action_name](params)
        else:
            raise ValueError(f"Unknown action: {action_name}")
    
    def pick_action(self, params):
        """Convert 'pick(object)' to robot grasp command"""
        object_name = params[0].strip()
        
        # Get object pose from perception
        object_pose = self.perception.get_object_pose(object_name)
        
        # Generate grasp
        grasp_pose = self.compute_grasp(object_pose)
        
        return {
            "primitive": "grasp",
            "target_pose": grasp_pose,
            "gripper_width": 0.08
        }

Integrating with ROS 2
Complete LLM Planning Node
pythonimport rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import openai

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        
        # Subscribe to task requests
        self.task_sub = self.create_subscription(
            String, 'task_request', self.task_callback, 10
        )
        
        # Publish action commands
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        
        # LLM client
        self.client = OpenAI(api_key=self.declare_parameter('openai_key', '').value)
        
        self.get_logger().info('LLM Planner ready')
    
    def task_callback(self, msg):
        task = msg.data
        self.get_logger().info(f'Received task: {task}')
        
        # Generate plan
        plan = self.generate_plan(task)
        
        # Execute plan
        for action in plan:
            action_msg = String()
            action_msg.data = action
            self.action_pub.publish(action_msg)
            
            # Wait for completion
            rclpy.spin_once(self, timeout_sec=5.0)
    
    def generate_plan(self, task):
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a robot task planner."},
                {"role": "user", "content": f"Task: {task}\nGenerate step-by-step plan."}
            ]
        )
        
        plan_text = response.choices[0].message.content
        return self.parse_plan(plan_text)
    
    def parse_plan(self, plan_text):
        # Simple parsing: split by newlines
        lines = plan_text.strip().split('\n')
        actions = [line.strip() for line in lines if line.strip()]
        return actions

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

Advanced: Code as Policies
LLM Generates Python Code
pythondef code_as_policy(task, scene):
    prompt = f"""
Generate Python code to accomplish the task.
Available functions: robot.navigate(x, y), robot.pick(obj), robot.place(obj, loc)

Task: {task}
Scene: {scene}

Generate executable Python code:
"""
    
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )
    
    code = response.choices[0].message.content
    code = code.replace("```python", "").replace("```", "").strip()
    
    # Execute generated code (⚠️ Security risk in production!)
    exec(code, {"robot": robot})

# Example output:
# def execute_task():
#     robot.navigate(0.5, 0.3)  # Move to table
#     robot.pick("red_block")
#     robot.navigate(1.0, 0.5)  # Move to box
#     robot.place("red_block", "box")
# 
# execute_task()

Multi-Agent Collaboration
Multiple Robots Coordinated by LLM
pythonclass MultiRobotPlanner:
    def plan_for_team(self, task, num_robots):
        prompt = f"""
Task: {task}
Number of robots: {num_robots}

Assign subtasks to each robot efficiently. Output as JSON:
{{
  "robot_1": ["subtask_1", "subtask_2"],
  "robot_2": ["subtask_3", "subtask_4"]
}}
"""
        
        response = client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )
        
        import json
        assignments = json.loads(response.choices[0].message.content)
        return assignments

# Example:
task = "Move all boxes from room A to room B"
assignments = planner.plan_for_team(task, num_robots=3)
# Output:
# {
#   "robot_1": ["navigate(room_A)", "pick(box_1)", "navigate(room_B)", "place(box_1)"],
#   "robot_2": ["navigate(room_A)", "pick(box_2)", "navigate(room_B)", "place(box_2)"],
#   "robot_3": ["navigate(room_A)", "pick(box_3)", "navigate(room_B)", "place(box_3)"]
# }

Optimization Tips
1. Caching Plans
pythonimport hashlib
import json

class CachedPlanner:
    def __init__(self):
        self.cache = {}
    
    def get_plan(self, task, scene):
        # Create cache key
        key = hashlib.md5(f"{task}_{scene}".encode()).hexdigest()
        
        if key in self.cache:
            return self.cache[key]
        
        # Generate new plan
        plan = self.llm.generate_plan(task, scene)
        self.cache[key] = plan
        return plan
2. Batch Processing
python# Instead of calling LLM for each step
for step in steps:
    reasoning = llm.reason(step)  # Slow!

# Batch all steps
all_steps = "\n".join(steps)
all_reasoning = llm.reason_batch(all_steps)  # Faster!
3. Local LLM for Speed
bash# Run LLaMA locally with Ollama
ollama run llama3:70b
pythonimport requests

def local_llm_plan(task):
    response = requests.post('http://localhost:11434/api/generate',
        json={
            "model": "llama3:70b",
            "prompt": f"Task: {task}\nGenerate plan:",
            "stream": False
        }
    )
    return response.json()['response']

Practice Exercises
1. Basic Planning:

Setup OpenAI API
Generate plan for "Set the table"
Parse into action list
Print each step

2. Grounded Execution:

Create action grounder
Map LLM actions to robot primitives
Test in simulation

3. Error Recovery:

Implement replanning on failure
Simulate grasp failure
Verify LLM generates alternative plan

4. Multi-Step Task:

Task: "Prepare breakfast"
Generate full plan (10+ steps)
Execute in Isaac Sim
Measure success rate

Next → Chapter 4: Computer Vision Integration

---

## FILE: docs/module4/computer-vision-integration.md
```markdown
---
id: computer-vision-integration
title: Computer Vision for Object Detection
sidebar_position: 4
---

# Chapter 4: Computer Vision for Object Detection

## Closing the Perception Loop

**Problem:** LLM says "pick up the red cup" but robot doesn't know *where* it is.

**Solution:** Computer vision detects objects and provides locations.
```
Voice: "Pick up the red cup"
  ↓
LLM: "Plan: 1. Locate red cup, 2. Navigate, 3. Grasp"
  ↓
Vision: "Red cup at position (0.5, 0.3, 0.8)"
  ↓
Robot: Executes grasp at (0.5, 0.3, 0.8)

Object Detection Models
YOLO (You Only Look Once)
Best for: Real-time detection (30+ FPS)
bashpip install ultralytics
pythonfrom ultralytics import YOLO
import cv2

# Load model
model = YOLO('yolov8n.pt')  # nano (fastest), s, m, l, x (most accurate)

# Inference
image = cv2.imread('scene.jpg')
results = model(image)

# Parse detections
for result in results:
    boxes = result.boxes
    for box in boxes:
        # Get bbox coordinates
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        
        # Get class and confidence
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        label = model.names[cls]
        
        print(f"Detected {label} at ({x1}, {y1}, {x2}, {y2}) with confidence {conf:.2f}")
        
        # Draw bbox
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(image, f"{label} {conf:.2f}", (int(x1), int(y1)-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

cv2.imshow('Detections', image)
cv2.waitKey(0)

ROS 2 Vision Node
Real-Time Detection
pythonimport rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10
        )
        
        # Publish annotated image
        self.viz_pub = self.create_publisher(
            Image, '/detections/image', 10
        )
        
        self.get_logger().info('YOLO Detector ready')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO
        results = self.model(cv_image, verbose=False)
        
        # Create detection message
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                detection = Detection2D()
                
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                # Class and confidence
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(cls)
                hypothesis.hypothesis.score = conf
                
                detection.results.append(hypothesis)
                detection_array.detections.append(detection)
                
                # Draw on image
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{self.model.names[cls]} {conf:.2f}"
                cv2.putText(cv_image, label, (int(x1), int(y1)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Publish detections
        self.detection_pub.publish(detection_array)
        
        # Publish visualization
        viz_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.viz_pub.publish(viz_msg)

def main():
    rclpy.init()
    node = YOLODetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

3D Object Localization
From 2D Detection to 3D Position
pythonclass ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer')
        
        # Subscribe to detections + depth
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10
        )
        
        # Publish 3D positions
        self.pose_pub = self.create_publisher(PoseArray, '/object_poses', 10)
        
        self.depth_image = None
        self.camera_info = None  # Get from /camera/camera_info
    
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    
    def detection_callback(self, msg):
        if self.depth_image is None:
            return
        
        pose_array = PoseArray()
        pose_array.header = msg.header
        
        for detection in msg.detections:
            # Get bbox center
            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)
            
            # Get depth at center
            depth = self.depth_image[cy, cx]
            
            # Convert pixel to 3D point (using camera intrinsics)
            x_3d, y_3d, z_3d = self.pixel_to_3d(cx, cy, depth)
            
            # Create pose
            pose = Pose()
            pose.position.x = x_3d
            pose.position.y = y_3d
            pose.position.z = z_3d
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
        
        self.pose_pub.publish(pose_array)
    
    def pixel_to_3d(self, u, v, depth):
        """Convert pixel coordinates to 3D using camera intrinsics"""
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return x, y, z

Semantic Segmentation
Pixel-Level Classification
bashpip install segmentation-models-pytorch
pythonimport segmentation_models_pytorch as smp
import torch

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        # Load DeepLabV3+
        self.model = smp.DeepLabV3Plus(
            encoder_name="resnet50",
            encoder_weights="imagenet",
            classes=21,  # PASCAL VOC classes
            activation=None
        )
        self.model.eval()
        
        # Load pretrained weights
        checkpoint = torch.load('deeplabv3plus.pth')
        self.model.load_state_dict(checkpoint)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.callback, 10
        )
        
        self.seg_pub = self.create_publisher(
            Image, '/segmentation', 10
        )
    
    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Preprocess
        image_tensor = self.preprocess(cv_image)
        
        # Inference
        with torch.no_grad():
            output = self.model(image_tensor)
            segmentation = torch.argmax(output, dim=1)[0].cpu().numpy()
        
        # Colorize
        colored_seg = self.colorize_segmentation(segmentation)
        
        # Publish
        seg_msg = self.bridge.cv2_to_imgmsg(colored_seg, 'bgr8')
        self.seg_pub.publish(seg_msg)
    
    def colorize_segmentation(self, seg_map):
        """Map class IDs to colors"""
        colors = {
            0: [0, 0, 0],      # Background
            1: [128, 0, 0],    # Person
            15: [192, 128, 128],  # Table
            20: [64, 64, 128],    # Cup
            # ... more classes
        }
        
        h, w = seg_map.shape
        colored = np.zeros((h, w, 3), dtype=np.uint8)
        
        for class_id, color in colors.items():
            mask = seg_map == class_id
            colored[mask] = color
        
        return colored

Tracking Objects
DeepSORT for Multi-Object Tracking
bashpip install deep-sort-realtime
pythonfrom deep_sort_realtime.deepsort_tracker import DeepSort

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        # Initialize tracker
        self.tracker = DeepSort(max_age=30, n_init=3)
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.callback, 10
        )
        
        self.track_pub = self.create_publisher(
            MarkerArray, '/tracks', 10
        )
    
    def callback(self, msg):
        # Convert detections to DeepSORT format
        detections = []
        for det in msg.detections:
            bbox = det.bbox
            x1 = bbox.center.position.x - bbox.size_x / 2
            y1 = bbox.center.position.y - bbox.size_y / 2
            x2 = bbox.center.position.x + bbox.size_x / 2
            y2 = bbox.center.position.y + bbox.size_y / 2
            
            conf = det.results[0].hypothesis.score
            cls = det.results[0].hypothesis.class_id
            
            detections.append(([x1, y1, x2, y2], conf, cls))
        
        # Update tracker
        tracks = self.tracker.update_tracks(detections, frame=None)
        
        # Publish tracks
        marker_array = MarkerArray()
        for track in tracks:
            if not track.is_confirmed():
                continue
            
            track_id = track.track_id
            bbox = track.to_ltrb()
            
            marker = self.create_marker(track_id, bbox)
            marker_array.markers.append(marker)
        
        self.track_pub.publish(marker_array# 