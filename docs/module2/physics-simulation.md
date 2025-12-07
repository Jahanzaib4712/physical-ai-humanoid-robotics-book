---
id: physics-simulation
title: Physics, Gravity & Collisions
sidebar_position: 2
---

# Chapter 2: Physics, Gravity & Collisions

## Understanding Physics Engines

### What is a Physics Engine?

A physics engine calculates:
- **Rigid body dynamics** - How objects move and rotate
- **Collisions** - When objects touch/intersect
- **Constraints** - Joints, springs, damping
- **Forces** - Gravity, friction, applied forces

Gazebo supports multiple physics engines:
- **DART** (default) - Fast, accurate
- **Bullet** - Gaming industry standard
- **ODE** - Legacy, less accurate
- **TPE** - Trivial Physics Engine (simple, fast)

### Configuring Physics
```xml
<physics name="default_physics" type="dart">
  <!-- Time step (smaller = more accurate but slower) -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor (1.0 = realtime, 2.0 = 2x speed) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Solver iterations (more = more accurate) -->
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

### Physics Step Size

**max_step_size** determines simulation accuracy:

| Step Size | Accuracy | Speed | Use Case |
|-----------|----------|-------|----------|
| 0.0001s   | Very High | Slow | Precise manipulation |
| 0.001s    | High | Good | General robotics (recommended) |
| 0.004s    | Medium | Fast | Simple navigation |
| 0.01s     | Low | Very Fast | Visualization only |

**Rule of thumb:** Smaller objects/faster motion need smaller steps.

---

## Gravity Configuration

### Standard Earth Gravity
```xml
<world name="my_world">
  <gravity>0 0 -9.81</gravity>  <!-- m/s² in Z direction -->
</world>
```

### Custom Gravity Scenarios

**Moon (1/6 Earth):**
```xml
<gravity>0 0 -1.62</gravity>
```

**Mars (3/8 Earth):**
```xml
<gravity>0 0 -3.71</gravity>
```

**Zero gravity (space):**
```xml
<gravity>0 0 0</gravity>
```

**Sideways gravity (testing):**
```xml
<gravity>-9.81 0 0</gravity>  <!-- Pulls objects in -X direction -->
```

---

## Collision Detection

### Collision Geometry

Every link needs collision geometry for physics:
```xml
<link name="body">
  <!-- Visual: What you see -->
  <visual name="visual">
    <geometry>
      <mesh>
        <uri>model://my_robot/meshes/body.dae</uri>  <!-- Complex mesh -->
      </mesh>
    </geometry>
  </visual>
  
  <!-- Collision: Simplified for physics -->
  <collision name="collision">
    <geometry>
      <box><size>0.5 0.3 0.2</size></box>  <!-- Simple box -->
    </geometry>
  </collision>
</link>
```

**Why simplify collision?**
- Mesh collision is 10-100x slower
- Simple shapes (box, sphere, cylinder) are fast
- Slight inaccuracy is usually acceptable

### Collision Bitmasks

Control which objects collide:
```xml
<collision name="robot_body">
  <geometry>
    <box><size>1 1 1</size></box>
  </geometry>
  <surface>
    <contact>
      <collide_bitmask>0x01</collide_bitmask>  <!-- Group 1 -->
    </contact>
  </surface>
</collision>

<collision name="sensor">
  <geometry>
    <sphere><radius>0.1</radius></sphere>
  </geometry>
  <surface>
    <contact>
      <collide_bitmask>0x02</collide_bitmask>  <!-- Group 2 -->
    </contact>
  </surface>
</collision>
```

Bitmask `0x01` only collides with `0x01`, not `0x02`.

---

## Surface Properties

### Friction
```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>    <!-- Friction coefficient (0=ice, 1=rubber) -->
        <mu2>1.0</mu2>  <!-- Perpendicular friction -->
      </ode>
    </friction>
  </surface>
</collision>
```

**Common friction values:**
- **0.0** - Ice/Teflon (frictionless)
- **0.1** - Smooth metal on metal
- **0.5** - Wood on wood
- **1.0** - Rubber on concrete
- **1.5+** - Sticky surfaces

### Bounce (Restitution)
```xml
<surface>
  <bounce>
    <restitution_coefficient>0.8</restitution_coefficient>  <!-- 0=no bounce, 1=perfect bounce -->
    <threshold>0.001</threshold>  <!-- Min velocity for bounce -->
  </bounce>
</surface>
```

**Values:**
- **0.0** - Clay (no bounce)
- **0.5** - Basketball
- **0.8** - Rubber ball
- **0.95** - Super ball

### Contact Properties
```xml
<surface>
  <contact>
    <ode>
      <kp>1000000.0</kp>  <!-- Contact stiffness (higher = harder) -->
      <kd>1.0</kd>        <!-- Contact damping (higher = less bounce) -->
      <max_vel>0.01</max_vel>  <!-- Max penetration velocity -->
      <min_depth>0.001</min_depth>  <!-- Min penetration depth -->
    </ode>
  </contact>
</surface>
```

---

## Inertial Properties

### Why Inertia Matters

Incorrect inertia causes:
- Unrealistic motion (spinning too fast/slow)
- Simulation instabilities (explosions)
- Poor control performance

### Calculating Inertia

**For simple shapes, use formulas:**

**Box:**

mass = 10 kg
width = 0.5 m, depth = 0.3 m, height = 0.2 m
Ixx = (1/12) * mass * (depth² + height²) = 0.108
Iyy = (1/12) * mass * (width² + height²) = 0.242
Izz = (1/12) * mass * (width² + depth²) = 0.283

**In SDF:**
```xml
<inertial>
  <mass>10.0</mass>
  <inertia>
    <ixx>0.108</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>0.242</iyy>
    <iyz>0</iyz>
    <izz>0.283</izz>
  </inertia>
</inertial>
```

**Cylinder:**

mass = 5 kg
radius = 0.1 m, length = 0.5 m
Ixx = (1/12) * mass * (3radius² + length²) = 0.113
Iyy = (1/12) * mass * (3radius² + length²) = 0.113
Izz = (1/2) * mass * radius² = 0.025

**Sphere:**
mass = 2 kg
radius = 0.15 m
I = (2/5) * mass * radius² = 0.018 (all axes)

### Inertia Calculator Tool

Online tool: **inertialtensor.com**

Or use MeshLab to compute from 3D meshes.

---

## Joint Physics

### Joint Types Review

**Revolute (hinge):**
```xml
<joint name="elbow" type="revolute">
  <parent>upper_arm</parent>
  <child>forearm</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>0</lower>
      <upper>2.618</upper>  <!-- 150 degrees -->
      <effort>100</effort>    <!-- Max torque (Nm) -->
      <velocity>2.0</velocity>  <!-- Max angular vel (rad/s) -->
    </limit>
    <dynamics>
      <damping>0.5</damping>      <!-- Joint friction -->
      <friction>0.1</friction>    <!-- Static friction -->
      <spring_reference>0</spring_reference>  <!-- Rest position -->
      <spring_stiffness>0</spring_stiffness>  <!-- Spring constant -->
    </dynamics>
  </axis>
</joint>
```

### Joint Damping

**Damping** resists motion (like friction in oil):
```xml
<dynamics>
  <damping>1.0</damping>  <!-- Nm/(rad/s) -->
</dynamics>
```

**Low damping (0.1):** Joint moves freely, may oscillate  
**Medium damping (1.0):** Realistic robot joint  
**High damping (10.0):** Joint moves slowly, heavily damped

### Joint Springs

Simulate passive compliance:
```xml
<dynamics>
  <spring_reference>1.57</spring_reference>  <!-- 90 degrees rest position -->
  <spring_stiffness>50.0</spring_stiffness>  <!-- Nm/rad -->
</dynamics>
```

Joint will try to return to 90° position.

---

## Advanced: Contact Sensors

### Bumper Sensor

Detect collisions:
```xml
<sensor name="bumper" type="contact">
  <contact>
    <collision>body_collision</collision>
  </contact>
  <plugin
    filename="gz-sim-contact-system"
    name="gz::sim::systems::Contact">
  </plugin>
</sensor>
```

Subscribe to contacts:
```bash
gz topic -e -t /model/robot/sensor/bumper/contact
```

---

## Real-World Scenarios

### Humanoid Walking

**Challenge:** Bipedal walking requires:
- Precise foot placement
- Dynamic balance
- Compliant joints
- Fast physics updates

**Configuration:**
```xml
<physics name="fast_physics" type="dart">
  <max_step_size>0.0005</max_step_size>  <!-- 2000 Hz -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**Joint properties:**
```xml
<dynamics>
  <damping>2.0</damping>    <!-- High damping for stability -->
  <friction>0.5</friction>
</dynamics>
```

### Object Manipulation

**Challenge:** Grasping requires:
- High-friction fingers
- Soft contact
- Stable grasps

**Gripper finger collision:**
```xml
<surface>
  <friction>
    <ode>
      <mu>1.5</mu>  <!-- High friction -->
      <mu2>1.5</mu2>
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>100000.0</kp>  <!-- Soft contact -->
      <kd>10.0</kd>
    </ode>
  </contact>
</surface>
```

---

## Debugging Physics Issues

### Unstable Simulation

**Symptoms:** Models explode, jitter, fly away

**Solutions:**
1. Reduce `max_step_size` (e.g., 0.001 → 0.0005)
2. Increase contact stiffness (`kp`)
3. Check mass/inertia values (not zero!)
4. Verify collision geometries don't overlap

### Slow Simulation

**Symptoms:** Real-time factor << 1.0

**Solutions:**
1. Increase `max_step_size` (0.001 → 0.002)
2. Simplify collision geometries
3. Reduce number of contacts
4. Use faster physics engine (DART vs ODE)

### Objects Fall Through Ground

**Cause:** Missing collision geometry

**Solution:**
```xml
<model name="ground">
  <static>true</static>  <!-- Important! -->
  <link name="link">
    <collision name="collision">  <!-- Must have collision! -->
      <geometry>
        <plane><normal>0 0 1</normal></plane>
      </geometry>
    </collision>
  </link>
</model>
```

---

## Practice Exercises

**1. Bouncing Ball:**
- Create sphere with mass=0.5kg
- Set restitution=0.9
- Drop from 2m height
- Measure bounce heights

**2. Friction Ramp:**
- Create inclined plane (30°)
- Place box on ramp
- Test with friction: 0.1, 0.5, 1.0
- Observe sliding behavior

**3. Pendulum:**
- Create rod with sphere at end
- Attach with revolute joint
- Add damping=0.1
- Release from 45° angle
- Measure oscillation period

**Next →** [Chapter 3: Sensor Simulation](./sensor-simulation.md)