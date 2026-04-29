# Gazebo Simulation

This page describes the simulation world, the Gazebo plugins that run in it, and how the robot is spawned.

---

## Simulation World

**File:** `src/ros_gz_example_gazebo/worlds/test_xacro.sdf.xacro`

The world is written as an **SDF/xacro hybrid** — it uses xacro macros to generate repetitive geometry (walls, obstacles, thermal targets) and is processed at launch time by `subprocess.check_output(['xacro', world_xacro_file])` in `test_xacro.launch.py` (lines 23–26).

### Arena Layout

- A 10 × 10 m enclosed arena (`arena_size = 10.0`)
- 0.2 m thick, 1.0 m tall outer walls (North, South, East, West)
- Two internal vertical walls forming rooms with door openings:
  - `room_v_left_top/bot` at x = −1.5 m
  - `room_v_right_top/bot` at x = +1.8 m
- Two **thermal target objects** to demonstrate the thermal camera:
  - `hot_target_1`: 360 K (~87 °C), red colour, at (3.0, 1.0)
  - `cold_target_1`: 285 K (~12 °C), blue colour, at (−2.2, −1.4)

### Physics

| Parameter | Value |
|-----------|-------|
| Physics engine | DART (default) |
| Max step size | 0.001 s (1 ms) |
| Real-time factor | 1.0 |

---

## World-level Gazebo Plugins

These are loaded by Gazebo Harmonic for every simulation:

| Plugin filename | System class | Purpose |
|-----------------|-------------|---------|
| `gz-sim-physics-system` | `gz::sim::systems::Physics` | Runs the physics simulation step |
| `gz-sim-user-commands-system` | `gz::sim::systems::UserCommands` | Enables model spawning from the GUI/CLI |
| `gz-sim-scene-broadcaster-system` | `gz::sim::systems::SceneBroadcaster` | Publishes scene state for GUI and RViz |
| `gz-sim-sensors-system` | `gz::sim::systems::Sensors` | Activates all sensor plugins (LiDAR, camera, IMU) |

> The `gz-sim-sensors-system` plugin is required for any sensor to produce data. Without it, all sensors are silent.

---

## Robot-level Plugins

These are attached to the robot model and defined in the URDF/xacro (see [Robot Model](Robot-Model)):

| Plugin | Purpose |
|--------|---------|
| `gz-sim-mecanum-drive-system` | Drives the four mecanum wheels and publishes odometry + TF |
| `gz-sim-joint-state-publisher-system` | Publishes wheel and LiDAR joint states |
| `gz-sim-joint-position-controller-system` | Controls the LiDAR tilt joint via PID |

---

## Custom Gazebo System Plugins

The package `ros_gz_gazebo` (`src/ros_gz_example_gazebo/`) compiles two custom plugins:

| Plugin | Source file | Loaded in |
|--------|-------------|-----------|
| `BasicSystem` (`ros_gz_gazebo::BasicSystem`) | `src/BasicSystem.cc` | `diff_drive.sdf` world only |
| `FullSystem` (`ros_gz_gazebo::FullSystem`) | `src/FullSystem.cc` | `diff_drive.sdf` world only |

These are **not used in the main `test_xacro` simulation**. They are demonstration plugins from the ros_gz template project.

---

## Robot Spawning Flow

The `test_xacro.launch.py` performs the following steps at launch time:

```
1. xacro test_xacro.sdf.xacro  →  temp_world.sdf
2. xacro test.urdf.xacro        →  temp_robot.urdf
3. gz sdf -p temp_robot.urdf    →  model.sdf  (written to models/xacro_test/)
4. Gazebo loads temp_world.sdf
5. World SDF includes:  <include><uri>model://xacro_test</uri></include>
   which picks up the generated model.sdf via GZ_SIM_RESOURCE_PATH
```

The `GZ_SIM_RESOURCE_PATH` is set before Gazebo starts:

```python
# test_xacro.launch.py, lines 45–49
models_path = os.path.join(pkg_project_description, 'models')
gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=[models_path, os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
)
```

---

## Launching the Simulation

```bash
# Default (RViz + SLAM enabled)
ros2 launch ros_gz_bringup test_xacro.launch.py

# Without RViz
ros2 launch ros_gz_bringup test_xacro.launch.py rviz:=false

# Without SLAM
ros2 launch ros_gz_bringup test_xacro.launch.py slam:=false

# Custom SLAM params
ros2 launch ros_gz_bringup test_xacro.launch.py \
  slam_params_file:=/path/to/your/params.yaml
```

---

## Sending Commands Manually

```bash
# Drive the robot forward
ros2 topic pub /test_robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Strafe right (mecanum)
ros2 topic pub /test_robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Tilt the LiDAR to 30°
ros2 topic pub /lidar_cmd_pos std_msgs/msg/Float64 "{data: 0.5}"
```
