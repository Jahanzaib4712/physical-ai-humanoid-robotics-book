---
id: unity-integration
title: Unity for High-Fidelity Rendering
sidebar_position: 4
---

# Chapter 4: Unity for High-Fidelity Rendering

## Why Unity for Robotics?

### Gazebo vs Unity

| Feature | Gazebo | Unity |
|---------|--------|-------|
| Physics Accuracy | Excellent | Good |
| Visual Quality | Good | Excellent |
| Rendering Speed | Fast | Very Fast |
| Asset Library | Limited | Massive (Asset Store) |
| Learning Curve | Steep | Moderate |
| VR/AR Support | Limited | Native |

**Use Unity for:**
- Photorealistic simulation
- Human-robot interaction studies
- VR/AR telepresence
- Marketing demos
- Synthetic data generation

---

## Unity Robotics Hub

### Installation

**Step 1: Install Unity Hub**
```bash
# Download from unity.com
# Install Unity Editor 2022.3 LTS (recommended)
```

**Step 2: Install ROS TCP Connector**

In Unity Package Manager:
1. Window → Package Manager
2. Click `+` → Add package from git URL
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

**Step 3: Install URDF Importer**

https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer

---

## Importing URDF into Unity

### Step 1: Prepare URDF

Ensure your URDF has:
- Valid mesh files (.dae, .obj, .stl)
- Correct file paths
- Proper joint definitions

### Step 2: Import in Unity

1. **Assets → Import Robot from URDF**
2. Select your `.urdf` file
3. Configure import settings:
   - Mesh scaling: Usually 1.0
   - Axis settings: Y-up
   - Collision meshes: Import or generate
4. Click **Import**

Unity creates:
- GameObject hierarchy matching URDF links
- Articulation Body components (Unity physics)
- Visual meshes
- Collision meshes

### Step 3: Configure Physics

Unity uses **Articulation Bodies** (not Rigidbody) for robots:
```csharp
// Accessed via Inspector or script
ArticulationBody joint = GetComponent<ArticulationBody>();

// Joint limits
joint.xDrive = new ArticulationDrive
{
    lowerLimit = 0f,
    upperLimit = 150f,
    stiffness = 10000f,
    damping = 100f,
    forceLimit = 1000f
};
```

---

## ROS 2 - Unity Communication

### Architecture

ROS 2 Node ←→ ROS TCP Endpoint (Python) ←→ Unity (C#)

### Step 1: Install ROS TCP Endpoint
```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2: Configure Unity

In Unity:
1. **Robotics → ROS Settings**
2. Set ROS IP: `127.0.0.1` (or your ROS machine IP)
3. Set ROS Port: `10000`
4. Protocol: `ROS2`

### Step 3: Start Endpoint
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Step 4: Test Connection

Unity script:
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RosPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_test";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        StringMsg msg = new StringMsg("Hello from Unity!");
        ros.Publish(topicName, msg);
    }
}
```

In ROS 2:
```bash
ros2 topic echo /unity_test
```

---

## Controlling Robot from ROS 2

### Publishing Joint Commands

**Unity script** (C#):
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointController : MonoBehaviour
{
    ROSConnection ros;
    ArticulationBody joint;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("joint_commands", UpdateJoint);
        joint = GetComponent<ArticulationBody>();
    }
    
    void UpdateJoint(JointStateMsg msg)
    {
        // Set target position
        var drive = joint.xDrive;
        drive.target = msg.position[0] * Mathf.Rad2Deg;  // Convert rad to deg
        joint.xDrive = drive;
    }
}
```

**ROS 2 publisher** (Python):
```python
from sensor_msgs.msg import JointState

pub = self.create_publisher(JointState, 'joint_commands', 10)

msg = JointState()
msg.name = ['elbow_joint']
msg.position = [1.57]  # 90 degrees in radians
pub.publish(msg)
```

---

## Camera Simulation in Unity

### Adding Camera

1. Create Empty GameObject
2. Add Component → Camera
3. Configure:
   - Field of View: 60
   - Clipping Planes: Near 0.1, Far 100
   - Render Texture: Create new (640x480)

### Publishing to ROS 2
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    public Camera imageCamera;
    RenderTexture renderTexture;
    Texture2D texture2D;
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("camera/image");
        
        renderTexture = new RenderTexture(640, 480, 24);
        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
        imageCamera.targetTexture = renderTexture;
    }
    
    void Update()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();
        
        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                }
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            step = 640 * 3,
            data = texture2D.GetRawTextureData()
        };
        
        ros.Publish("camera/image", msg);
    }
}
```

---

## High-Quality Rendering

### HDRP (High Definition Render Pipeline)

**Step 1: Create HDRP Project**
- Start with HDRP template in Unity Hub

**Step 2: Lighting Setup**
```csharp
// Add directional light
GameObject light = new GameObject("Main Light");
Light lightComp = light.AddComponent<Light>();
lightComp.type = LightType.Directional;
lightComp.intensity = 1.0f;
lightComp.shadows = LightShadows.Soft;
```

**Step 3: Post-Processing**
1. Create Volume: GameObject → Volume → Global Volume
2. Add Overrides:
   - Bloom (glow effect)
   - Ambient Occlusion (shadows in crevices)
   - Motion Blur
   - Depth of Field

### Realistic Materials
```csharp
// Metallic surface
Material metal = new Material(Shader.Find("HDRP/Lit"));
metal.SetFloat("_Metallic", 1.0f);
metal.SetFloat("_Smoothness", 0.8f);

// Matte surface
Material matte = new Material(Shader.Find("HDRP/Lit"));
matte.SetFloat("_Metallic", 0.0f);
matte.SetFloat("_Smoothness", 0.2f);
```

---

## Synthetic Data Generation

### Semantic Segmentation

**Use Perception Package:**
1. Install: `com.unity.perception`
2. Add component to camera: **Perception Camera**
3. Configure:
   - Semantic Segmentation
   - Instance Segmentation
   - Bounding Box 2D/3D

**Output:** Perfect labels for each object class.

### Domain Randomization

Vary environment to improve AI generalization:
```csharp
using UnityEngine.Perception.Randomization.Scenarios;

public class RobotScenario : FixedLengthScenario
{
    public void RandomizeLighting()
    {
        Light sun = GameObject.Find("Sun").GetComponent<Light>();
        sun.intensity = Random.Range(0.5f, 1.5f);
        sun.color = Random.ColorHSV();
    }
    
    public void RandomizeObjectPositions()
    {
        GameObject[] objects = GameObject.FindGameObjectsWithTag("Movable");
        foreach (var obj in objects)
        {
            obj.transform.position = new Vector3(
                Random.Range(-5f, 5f),
                0f,
                Random.Range(-5f, 5f)
            );
        }
    }
}
```

---

## VR/AR Integration

### VR Camera Setup
```csharp
using UnityEngine.XR;

public class VRCamera : MonoBehaviour
{
    void Start()
    {
        XRSettings.enabled = true;
        XRSettings.LoadDeviceByName("Oculus");
    }
}
```

### Teleoperation in VR

Control robot remotely through VR:
1. User wears VR headset
2. Tracks head position → Camera on robot
3. Hand controllers → Robot arms
4. Haptic feedback for collisions

---

## Performance Optimization

### Reduce Draw Calls
```csharp
// Static batching for non-moving objects
StaticBatchingUtility.Combine(gameObject);

// GPU instancing for repeated objects
material.enableInstancing = true;
```

### Level of Detail (LOD)
```csharp
LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
LOD[] lods = new LOD[3];

lods[0] = new LOD(0.6f, highDetailRenderers);  // Close
lods[1] = new LOD(0.3f, mediumDetailRenderers);  // Medium
lods[2] = new LOD(0.1f, lowDetailRenderers);  // Far

lodGroup.SetLODs(lods);
```

### Occlusion Culling

1. Window → Rendering → Occlusion Culling
2. Bake occlusion data
3. Camera only renders visible objects

---

## Unity ML-Agents Integration

Train RL policies directly in Unity:
```bash
pip install mlagents
```

**Agent script:**
```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class RobotAgent : Agent
{
    public override void OnEpisodeBegin()
    {
        // Reset environment
        transform.localPosition = Vector3.zero;
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        // Add observations (position, velocity, etc.)
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(GetComponent<Rigidbody>().velocity);
    }
    
    public override void OnActionReceived(float[] actions)
    {
        // Apply actions (motor commands)
        float torque = actions[0];
        GetComponent<ArticulationBody>().SetDriveTorque(torque);
    }
}
```

Train:
```bash
mlagents-learn config.yaml --run-id=robot_training
```

---

## Practice Exercises

**1. URDF Import:**
- Export humanoid URDF from Gazebo
- Import into Unity
- Verify joint movements
- Compare physics with Gazebo

**2. ROS 2 Integration:**
- Publish camera image from Unity
- Subscribe in ROS 2
- Display in `rqt_image_view`
- Measure latency

**3. Synthetic Dataset:**
- Create scene with 100 randomized cubes
- Enable semantic segmentation
- Generate 1000 labeled images
- Train simple object detector

**4. VR Teleoperation:**
- Setup VR camera on robot
- Map controller input to joint commands
- Test remote manipulation
- Add haptic feedback

**Next →** [Chapter 5: ROS 2 - Gazebo Bridge](./ros2-gazebo-bridge.md)
