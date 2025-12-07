id: ros2-advanced
title: Advanced ROS 2 Concepts
sidebar_position: 4
---

# Chapter 4: Advanced ROS 2 Concepts

## Mastering Quality of Service, Actions, and Lifecycle

### Quality of Service (QoS)

QoS determines **how messages are delivered** between publishers and subscribers.

#### Why QoS Matters

Different use cases need different guarantees:
- **Camera images:** Can drop frames, need low latency
- **Motor commands:** Must arrive, order matters
- **Diagnostics:** Reliable, can tolerate slight delay

#### QoS Profiles

**1. Sensor Data** (Default for sensors)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Can drop messages
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # Keep last 10 messages
)

self.subscription = self.create_subscription(
    Image, '/camera/image', self.callback, sensor_qos
)
```

**2. System Default** (Reliable)
```python
system_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**3. Services** (Reliable + Volatile)
```python
service_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE  # No persistence
)
```

#### QoS Parameters

**Reliability:**
- `BEST_EFFORT` - Fast, can drop messages (sensors)
- `RELIABLE` - Guaranteed delivery (commands)

**Durability:**
- `VOLATILE` - Only for active subscribers
- `TRANSIENT_LOCAL` - Late joiners get last message

**History:**
- `KEEP_LAST` - Keep N recent messages
- `KEEP_ALL` - Never drop (until memory full)

**Deadline:** Max time between messages
```python
deadline_qos = QoSProfile(
    deadline=Duration(seconds=0, nanoseconds=100000000)  # 100ms
)
```

---

## Actions in Depth

Actions are for **long-running tasks** with feedback.

### Action Definition

Create `action/Navigate.action`:
````
# Goal
float32 target_x
float32 target_y
---
# Result
bool success
float32 final_x
float32 final_y
float32 distance_traveled
---
# Feedback
float32 current_x
float32 current_y
float32 distance_remaining
float32 percent_complete


from rclpy.action import ActionServer
from my_interfaces.action import Navigate
import time

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Navigating to ({goal_handle.request.target_x}, {goal_handle.request.target_y})')
        
        feedback_msg = Navigate.Feedback()
        total_distance = 10.0  # Simulated
        
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Navigate.Result()
            
            # Simulate progress
            feedback_msg.percent_complete = (i + 1) * 10.0
            feedback_msg.distance_remaining = total_distance * (1 - (i + 1) / 10)
            
            self.get_logger().info(f'Feedback: {feedback_msg.percent_complete}%')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        
        result = Navigate.Result()
        result.success = True
        result.distance_traveled = total_distance
        return result




from rclpy.action import ActionClient

class NavigationActionClient(Node):
    def __init__(self):
        super().__init__('nav_action_client')
        self._action_client = ActionClient(self, Navigate, 'navigate')
    
    def send_goal(self, x, y):
        goal_msg = Navigate.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.percent_complete}%')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Success={result.success}, Distance={result.distance_traveled}')
````

---

## Lifecycle Nodes

Lifecycle nodes have **managed states** for controlled startup/shutdown.

### States
````
Unconfigured → Configuring → Inactive → Activating → Active
                    ↓                         ↓          ↓
                Cleaning Up ← Deactivating ← ─┘        ShuttingDown        



from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
    
    def on_configure(self, state: LifecycleState):
        self.get_logger().info('Configuring...')
        # Load parameters, allocate resources
        self.publisher = self.create_lifecycle_publisher(String, 'output', 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState):
        self.get_logger().info('Activating...')
        # Start processing
        self.timer = self.create_timer(1.0, self.timer_callback)
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info('Deactivating...')
        # Stop processing
        self.timer.cancel()
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('Cleaning up...')
        # Release resources
        self.destroy_publisher(self.publisher)
        return TransitionCallbackReturn.SUCCESS
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from lifecycle node'
        self.publisher.publish(msg)


Component Composition
Components allow multiple nodes in single process (lower overhead).
Creating Component


import rclpy
from rclpy.node import Node

class TalkerComponent(Node):
    def __init__(self, options):
        super().__init__('talker', options=options)
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from component'
        self.pub.publish(msg)



Composition Launch File
pythonfrom launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_pkg',
                plugin='my_pkg::TalkerComponent',
                name='talker'),
            ComposableNode(
                package='my_pkg',
                plugin='my_pkg::ListenerComponent',
                name='listener'),
        ],
        output='screen',
    )
    
    return LaunchDescription([container])        



Benefits:

Reduced CPU/memory overhead
Intra-process communication (zero-copy)
Faster startup


Parameter Events
Monitor parameter changes across system:    

from rcl_interfaces.msg import ParameterEvent

class ParameterMonitor(Node):
    def __init__(self):
        super().__init__('param_monitor')
        self.subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.callback,
            10
        )
    
    def callback(self, msg):
        for changed_param in msg.changed_parameters:
            self.get_logger().info(
                f'Node {msg.node} changed {changed_param.name} to {changed_param.value}'
            )


Security (DDS Security)
Enable encrypted communication:
bash# Generate security keys
ros2 security create_keystore demo_keys
ros2 security create_key demo_keys /talker
ros2 security create_key demo_keys /listener

# Set environment
export ROS_SECURITY_KEYSTORE=~/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Run nodes (now encrypted)
ros2 run demo_nodes_cpp talker

Best Practices
1. Use Appropriate QoS
python# Sensors: BEST_EFFORT
# Commands: RELIABLE
# Status: RELIABLE + TRANSIENT_LOCAL
2. Handle Failures Gracefully
pythontry:
    transform = self.tf_buffer.lookup_transform(...)
except TransformException as ex:
    self.get_logger().warn(f'Could not transform: {ex}')
    return
3. Monitor System Health
pythonfrom diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# Publish diagnostics
diag_msg = DiagnosticArray()
status = DiagnosticStatus()
status.name = 'Robot Controller'
status.level = DiagnosticStatus.OK
status.message = 'Operating normally'
diag_msg.status.append(status)
self.diag_pub.publish(diag_msg)
4. Use Namespaces for Multi-Robot
python# robot1/cmd_vel, robot2/cmd_vel
Node(..., namespace='robot1')

Performance Optimization
1. Intra-Process Communication
python# Enable zero-copy within same process
rclpy.init(args=args)
executor = rclpy.executors.SingleThreadedExecutor()
# Components in same container automatically use intra-process
2. Callback Groups
pythonfrom rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# Allow parallel execution
self.callback_group = Reentrant CallbackGroup()
self.timer = self.create_timer(1.0, self.callback, callback_group=self.callback_group)
3. Executors
python# Multi-threaded for parallel callbacks
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
````

---

## Practice Exercises

**1. QoS Experiment:**
- Create camera publisher with BEST_EFFORT
- Create subscriber with RELIABLE
- Observe incompatibility warning

**2. Build Action Server:**
- Create "Charge Battery" action
- Goal: Target charge level (80%, 100%)
- Feedback: Current level every second
- Result: Time taken, final level

**3. Lifecycle Node:**
- Create sensor driver as lifecycle node
- Configure: Open device
- Activate: Start publishing
- Deactivate: Stop publishing
- Cleanup: Close device            