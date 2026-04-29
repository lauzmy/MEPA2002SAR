# Repository Structure

This page describes every package in the repository, its purpose, and what files it contains.

---

## Package Overview

```
src/
├── ros_gz_bringup/
├── ros_gz_description/
├── ros_gz_example_gazebo/
├── ros_gz_application/
└── sensorer/ldlidar/
```

---

## `ros_gz_bringup`

**Role:** Orchestration — contains all launch files and configuration YAML files.

### Launch files (`launch/`)

| File | Purpose |
|------|---------|
| `test_xacro.launch.py` | **Main simulation launch** — starts Gazebo, RSP, bridge, SLAM, EKF, RViz |
| `gregor.launch.py` | Physical robot launch — starts real sensors, SLAM, EKF, RViz |

### Configuration files (`config/`)

| File | Purpose |
|------|---------|
| `ros_gz_bridge.yaml` | Defines all bridged topics between ROS 2 and Gazebo |
| `ekf.yaml` | `robot_localization` EKF filter parameters |
| `mapper_params_online_async.yaml` | `slam_toolbox` online async mapping parameters |
| `nav2_params_protomota.yaml` | Full Nav2 stack parameters for the mecanum robot |
| `test_robot.rviz` | RViz2 configuration for the simulation/real robot |

### Key `package.xml` dependencies

```xml
<depend>ros_gz</depend>
<depend>ros_gz_description</depend>
<depend>ros_gz_gazebo</depend>
<depend>slam_toolbox</depend>
<depend>nav2_bringup</depend>
<depend>robot_localization</depend>
<depend>sdformat_urdf</depend>
<depend>tf2_ros</depend>
<depend>rviz2</depend>
<depend>joint_state_publisher</depend>
```

---

## `ros_gz_description`

**Role:** Robot model definition — URDF/xacro files and SDF models used both by Gazebo and `robot_state_publisher`.

### Models (`models/`)

| Model | Description |
|-------|-------------|
| `xacro_test/test.urdf.xacro` | **Primary robot model** — mecanum drive + tilting LiDAR + thermal camera + IMU |

### Hooks (`hooks/`)

Sets up the `GZ_SIM_RESOURCE_PATH` environment variable so Gazebo can find model files after `colcon build`.

---

## `ros_gz_example_gazebo`

**Role:** Custom Gazebo system plugins + simulation worlds.

### Worlds (`worlds/`)

| File | Description |
|------|-------------|
| `test_xacro.sdf.xacro` | Main simulation arena — walled enclosure with thermal target objects |

### Custom plugins (`src/`)

| Plugin | Description |
|--------|-------------|
| `BasicSystem.cc` | Minimal example Gazebo system plugin |
| `FullSystem.cc` | Extended example Gazebo system plugin |

These plugins are compiled to shared libraries (`BasicSystem.so`, `FullSystem.so`) and placed on `GZ_SIM_PLUGIN_PATH`.

### `CMakeLists.txt` build notes

- Detects `GZ_VERSION` env variable to select `gz-sim8` (Harmonic) or `gz-sim7` (Garden)
- Links against `gz-plugin2`, `gz-common5`, and `gz-sim`

---

## `ros_gz_application`

**Role:** Custom ROS 2 application nodes written in Python.

### Nodes (`noder/`)

| Script | Installed as | Description |
|--------|-------------|-------------|
| `lidar_sweeper.py` | `lidar_sweeper` | Publishes a sinusoidal `Float64` to `/lidar_cmd_pos` to tilt the LiDAR up and down |
| `laser_assembler.py` | `laser_assembler` | Collects 2D laser scans over a full tilt sweep, transforms them to a fixed frame, and publishes a 3D `PointCloud2` on `/assembled_cloud` |

### Launch scripts (`launch/`)

| Script | Description |
|--------|-------------|
| `cmd_vel_publisher.py` | Simple node that publishes constant `Twist` to `/diff_drive/cmd_vel` (used in the diff_drive demo only) |

### Python runtime dependencies (not in `package.xml`)

These must be installed separately:

```
laser_geometry      → ros-jazzy-laser-geometry
tf2_ros             → ros-jazzy-tf2-ros
tf2_sensor_msgs     → ros-jazzy-tf2-sensor-msgs
sensor_msgs_py      → ros-jazzy-sensor-msgs-py
```

---

## `sensorer/ldlidar`

**Role:** Placeholder for the physical LiDAR driver package.

> **Status:** Currently empty. The `ldlidar` driver package must be cloned/installed separately. See [Getting Started](Getting-Started) for details.
