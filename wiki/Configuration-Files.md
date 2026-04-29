# Configuration Files

This page documents all YAML configuration files in `src/ros_gz_bringup/config/`.

---

## `ros_gz_bridge.yaml`

Defines the topic bridges between ROS 2 and Gazebo Harmonic via `ros_gz_bridge`'s `parameter_bridge` node.

**Full path:** `src/ros_gz_bringup/config/ros_gz_bridge.yaml`

Each entry specifies:
- `ros_topic_name` — topic name visible in ROS 2
- `gz_topic_name` — topic name inside Gazebo
- `ros_type_name` / `gz_type_name` — message types on each side
- `direction` — `ROS_TO_GZ` or `GZ_TO_ROS`

### Bridge table

| ROS Topic | ROS Type | GZ Topic | Direction |
|-----------|----------|---------|-----------|
| `/test_robot/cmd_vel` | `geometry_msgs/msg/Twist` | `/model/test_robot/cmd_vel` | ROS→GZ |
| `/clock` | `rosgraph_msgs/msg/Clock` | `/clock` | GZ→ROS |
| `/test_robot/odometry` | `nav_msgs/msg/Odometry` | `/model/test_robot/odometry` | GZ→ROS |
| `/test_robot/scan` | `sensor_msgs/msg/LaserScan` | `/scan` | GZ→ROS |
| `/test_robot/points` | `sensor_msgs/msg/PointCloud2` | `/scan/points` | GZ→ROS |
| `/joint_states` | `sensor_msgs/msg/JointState` | `/world/test_world/model/test_robot/joint_state` | GZ→ROS |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `/model/test_robot/pose_static` | GZ→ROS |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `/camera/image_raw` | GZ→ROS |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `/camera/camera_info` | GZ→ROS |
| `/imu` | `sensor_msgs/msg/Imu` | `/imu` | GZ→ROS |
| `/lidar_cmd_pos` | `std_msgs/msg/Float64` | `/model/test_robot/joint/lidar_joint/0/cmd_pos` | ROS→GZ |

> The `/tf_static` bridge is configured with `qos_overrides./tf_static.publisher.durability: transient_local` to ensure late-joining nodes receive the static transforms.

---

## `ekf.yaml`

Configures the `robot_localization` Extended Kalman Filter node (`ekf_filter_node`).

**Full path:** `src/ros_gz_bringup/config/ekf.yaml`

### Frame configuration

| Parameter | Value |
|-----------|-------|
| `map_frame` | `map` |
| `odom_frame` | `odom` |
| `base_link_frame` | `base_footprint` |
| `world_frame` | `odom` |
| `two_d_mode` | `true` |
| `publish_tf` | `true` |
| `frequency` | 30 Hz |

### Active sensor inputs

| Input | Topic | Fused fields |
|-------|-------|-------------|
| `odom0` (wheel odometry) | `/test_robot/odometry` | vx, vy, vyaw |

The odom configuration vector `[false, false, false, false, false, false, true, true, false, false, false, true, false, false, false]` selects velocity fields only (x-vel, y-vel, yaw-rate).

> **IMU is currently disabled** in the config (lines are commented out). To enable IMU fusion, uncomment the `imu0` block and set `/imu` as the source topic.

---

## `mapper_params_online_async.yaml`

Configures `slam_toolbox` running in **online async** mode (live mapping).

**Full path:** `src/ros_gz_bringup/config/mapper_params_online_async.yaml`

| Parameter | Value | Notes |
|-----------|-------|-------|
| `mode` | `mapping` | Live map building (not localization) |
| `scan_topic` | `/test_robot/scan` | Must match the bridged LiDAR topic |
| `base_frame` | `base_footprint` | Robot's base frame |
| `odom_frame` | `odom` | Odometry frame |
| `map_frame` | `map` | Output map frame |
| `resolution` | 0.05 m | 5 cm per cell |
| `max_laser_range` | 12.0 m | Matches LiDAR sensor max range |
| `map_update_interval` | 2.0 s | How often the map is updated |
| `transform_timeout` | 0.2 s | TF lookup timeout |

---

## `nav2_params_protomota.yaml`

Full Nav2 parameter file tuned for the **mecanum-drive robot** (omnidirectional movement).

**Full path:** `src/ros_gz_bringup/config/nav2_params_protomota.yaml`

> **Note:** Nav2 is **not started automatically** by any launch file in this repository. This config file is provided for use with `nav2_bringup`. See [Getting Started](Getting-Started) for how to launch Nav2.

### Key settings

| Component | Key parameter | Value |
|-----------|--------------|-------|
| `bt_navigator` | `odom_topic` | `/test_robot/odometry` |
| `controller_server` | `controller_plugins` | `FollowPath` (DWB) |
| `FollowPath` (DWB) | `min_vel_y` / `max_vel_y` | ±0.5 m/s (strafing enabled) |
| `local_costmap` | `observation_sources` | `/test_robot/scan` |
| `global_costmap` | `planner_plugins` | `SmacPlanner2D` |
| `collision_monitor` | `cmd_vel_out_topic` | `/test_robot/cmd_vel` |
| `velocity_smoother` | `odom_topic` | `/test_robot/odometry` |
| Robot radius | all costmaps | 0.15 m |

### Starting Nav2 manually

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=$(ros2 pkg prefix ros_gz_bringup)/share/ros_gz_bringup/config/nav2_params_protomota.yaml
```

---

## `test_robot.rviz`

Pre-configured RViz2 layout for the simulation. Loaded automatically by `test_xacro.launch.py` and `gregor.launch.py` when `rviz:=true` (default).

Displays:
- Robot model (from `/robot_description` + `/tf`)
- Laser scan (`/test_robot/scan`)
- Point cloud (`/assembled_cloud`)
- Occupancy map (`/map` from slam_toolbox)
- Camera image (`/camera/image_raw`)
- TF axes
