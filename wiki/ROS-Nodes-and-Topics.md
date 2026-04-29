# ROS Nodes and Topics

This page lists all running nodes, the topics they use, and the expected TF frame tree when running the main simulation (`test_xacro.launch.py`).

---

## Running Nodes

| Node | Package | Launch file | Description |
|------|---------|-------------|-------------|
| `gz_sim` (Gazebo) | `ros_gz_sim` | `test_xacro.launch.py` | Runs the physics simulation |
| `robot_state_publisher` | `robot_state_publisher` | `test_xacro.launch.py` | Reads URDF + `/joint_states` → publishes `/tf` |
| `parameter_bridge` | `ros_gz_bridge` | `test_xacro.launch.py` | Bridges ROS ↔ Gazebo topics |
| `lidar_sweeper` | `ros_gz_application` | `test_xacro.launch.py` | Publishes tilt commands to `/lidar_cmd_pos` |
| `tilt_laser_assembler` | `ros_gz_application` | `test_xacro.launch.py` | Assembles tilt sweeps into a 3D point cloud |
| `ekf_filter_node` | `robot_localization` | `test_xacro.launch.py` | Fuses odometry → `/odometry/filtered` + `odom→base_footprint` TF |
| `slam_toolbox` (async) | `slam_toolbox` | `test_xacro.launch.py` | Builds a 2D occupancy map from `/test_robot/scan` |
| `rviz2` | `rviz2` | `test_xacro.launch.py` | Visualization (optional, `rviz:=true` by default) |

---

## Topics

### Topics received by the robot (ROS → Gazebo)

| Topic | Type | Published by | Consumed by |
|-------|------|-------------|------------|
| `/test_robot/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop / Nav2 / user | `ros_gz_bridge` → Gazebo MecanumDrive |
| `/lidar_cmd_pos` | `std_msgs/msg/Float64` | `lidar_sweeper` | `ros_gz_bridge` → Gazebo JointPositionController |

### Topics published by the simulation (Gazebo → ROS)

| Topic | Type | Gazebo source | Consumed by |
|-------|------|--------------|------------|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo clock | All nodes with `use_sim_time: true` |
| `/test_robot/odometry` | `nav_msgs/msg/Odometry` | MecanumDrive plugin | `ekf_filter_node` |
| `/test_robot/scan` | `sensor_msgs/msg/LaserScan` | gpu_lidar sensor | `tilt_laser_assembler`, `slam_toolbox`, Nav2 costmaps |
| `/test_robot/points` | `sensor_msgs/msg/PointCloud2` | gpu_lidar scan/points | User / RViz |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Thermal camera sensor | User / RViz |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Thermal camera sensor | User / RViz |
| `/imu` | `sensor_msgs/msg/Imu` | IMU sensor | (Currently not fused — see EKF config) |
| `/joint_states` | `sensor_msgs/msg/JointState` | JointStatePublisher plugin | `robot_state_publisher` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Gazebo pose_static | `/tf` tree |

### Topics published by application nodes

| Topic | Type | Published by | Consumed by |
|-------|------|-------------|------------|
| `/assembled_cloud` | `sensor_msgs/msg/PointCloud2` | `tilt_laser_assembler` | RViz / user |

### Topics published by SLAM and localization

| Topic | Type | Published by |
|-------|------|-------------|
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `robot_localization` EKF |

---

## TF Frame Tree

```
map
└── odom                        ← published by robot_localization EKF
    └── base_footprint          ← published by MecanumDrive plugin (via bridge) or EKF
        └── body_link           ← published by robot_state_publisher
            ├── front_left_wheel_link
            ├── front_right_wheel_link
            ├── rear_left_wheel_link
            ├── rear_right_wheel_link
            ├── lidar_hinge_link    ← tilt joint (driven by lidar_sweeper)
            │   └── lidar_link      ← LiDAR sensor origin
            ├── camera_link         ← thermal camera
            └── imu_link            ← IMU sensor
```

### Frame summary

| Frame | Source | Used by |
|-------|--------|---------|
| `map` | `slam_toolbox` | Nav2 global planner, RViz |
| `odom` | `robot_localization` EKF | All local navigation |
| `base_footprint` | MecanumDrive TF + EKF | Robot center at ground level |
| `body_link` | `robot_state_publisher` | Collision / visual root |
| `lidar_link` | `robot_state_publisher` | Scan frame_id for all laser scans |
| `camera_link` | `robot_state_publisher` | Camera frame_id |
| `imu_link` | `robot_state_publisher` | IMU frame_id |

---

## `lidar_sweeper` Node Details

**Script:** `src/ros_gz_application/noder/lidar_sweeper.py`

Publishes a sinusoidal position command to the LiDAR tilt joint:

```
position = amplitude × sin(speed × t)
```

| Parameter | Default |
|-----------|---------|
| Amplitude | 0.5 rad (~28°) |
| Speed | 2.0 rad/s |
| Publish rate | 20 Hz |
| Output topic | `/lidar_cmd_pos` (`std_msgs/Float64`) |

---

## `tilt_laser_assembler` (laser_assembler) Node Details

**Script:** `src/ros_gz_application/noder/laser_assembler.py`

Remapped to subscribe to `/test_robot/scan` (via launch file remapping, line 98–100).

| Parameter | Value |
|-----------|-------|
| Fixed frame | `odom` |
| Robot frame | `base_footprint` |
| Sweep duration | 2.0 s |
| Input topic | `/scan` (remapped to `/test_robot/scan`) |
| Output topic | `/assembled_cloud` (`sensor_msgs/PointCloud2`) |

**Processing pipeline:**

1. Receives `LaserScan` on `/test_robot/scan`
2. Projects each scan to `PointCloud2` in the laser frame using `laser_geometry`
3. Transforms the cloud to the `odom` fixed frame using `tf2_sensor_msgs`
4. Accumulates points for 2.0 s (one full LiDAR sweep)
5. Transforms the accumulated cloud back to `base_footprint`
6. Publishes as `/assembled_cloud`
