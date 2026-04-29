# MEPA2002SAR – Project Wiki

Welcome to the wiki for **MEPA2002SAR** — a ROS 2 / Gazebo Harmonic project featuring a mecanum-drive robot with a tilting LiDAR, RGB-thermal camera, and IMU sensor. The project supports both Gazebo simulation and real-robot deployment.

---

## Pages

| Page | Description |
|------|-------------|
| [Repository Structure](Repository-Structure) | Overview of all packages and their roles |
| [Robot Model](Robot-Model) | URDF/xacro description: links, joints, sensors, Gazebo plugins |
| [Gazebo Simulation](Gazebo-Simulation) | Simulation world, world plugins, and how to spawn the robot |
| [ROS Nodes and Topics](ROS-Nodes-and-Topics) | All running nodes, published/subscribed topics, and the TF frame tree |
| [Configuration Files](Configuration-Files) | EKF, SLAM Toolbox, Nav2, and ros_gz_bridge configuration |
| [Getting Started](Getting-Started) | Step-by-step guide to install dependencies and run the simulation from scratch |

---

## Quick Overview

```
src/
├── ros_gz_bringup/        Launch files + YAML configuration
├── ros_gz_description/    URDF/xacro robot model + SDF models
├── ros_gz_example_gazebo/ Custom Gazebo plugins + SDF worlds
├── ros_gz_application/    Custom ROS 2 Python nodes
└── sensorer/ldlidar/      (Reserved for physical LiDAR driver)
```

The primary simulation launch file is:

```bash
ros2 launch ros_gz_bringup test_xacro.launch.py
```

For the physical robot:

```bash
ros2 launch ros_gz_bringup gregor.launch.py
```
