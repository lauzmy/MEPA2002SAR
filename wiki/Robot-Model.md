# Robot Model

The robot is defined in `src/ros_gz_description/models/xacro_test/test.urdf.xacro`.

It is a **four-wheeled mecanum-drive robot** with a **tilting 2D LiDAR**, a **thermal camera**, and an **IMU**.

---

## Physical Parameters

| Parameter | Value |
|-----------|-------|
| Body dimensions (L × W × H) | 220 mm × 100 mm × 100 mm |
| Wheel radius | 37.5 mm |
| Wheel width | 43 mm |
| Wheelbase (front–back) | 150 mm |
| Wheel separation (left–right) | 143 mm |
| LiDAR tilt range | ±45° (±0.785 rad) |

---

## Link and Joint Tree

```
base_footprint  (root)
└── body_link  [fixed]
    ├── front_left_wheel_link   [continuous, axis Y]
    ├── front_right_wheel_link  [continuous, axis Y]
    ├── rear_left_wheel_link    [continuous, axis Y]
    ├── rear_right_wheel_link   [continuous, axis Y]
    ├── lidar_hinge_link        [revolute, axis Y, ±45°]  ← tilt joint
    │   └── lidar_link          [fixed, +32 mm Z]         ← LiDAR sensor
    ├── camera_link             [fixed, front of body]    ← thermal camera
    └── imu_link                [fixed, top of body]      ← IMU sensor
```

---

## Sensors

### LiDAR (`lidar_link`)

Defined at xacro line 317–346.

| Property | Value |
|----------|-------|
| Gazebo sensor type | `gpu_lidar` |
| Horizontal samples | 720 |
| Horizontal FOV | 360° (−π to +π) |
| Range | 0.12 m – 12.0 m |
| Update rate | 10 Hz |
| Gazebo topic | `/scan` |
| Bridged ROS topic | `/test_robot/scan` (`sensor_msgs/LaserScan`) |
| Also bridged as | `/test_robot/points` (`sensor_msgs/PointCloud2`) |

The `lidar_sweeper` node drives the `lidar_joint` sinusoidally via `/lidar_cmd_pos`, creating a 3D sweep that `laser_assembler` accumulates into `/assembled_cloud`.

### Thermal Camera (`camera_link`)

Defined at xacro line 376–400.

| Property | Value |
|----------|-------|
| Gazebo sensor type | `camera` (thermal) |
| Resolution | 640 × 480 px |
| Format | `L16` (16-bit greyscale = temperature) |
| Horizontal FOV | 80° (1.396 rad) |
| Temperature range | 20 °C – 100 °C |
| Update rate | 20 Hz |
| Bridged ROS topic (image) | `/camera/image_raw` (`sensor_msgs/Image`) |
| Bridged ROS topic (info) | `/camera/camera_info` (`sensor_msgs/CameraInfo`) |

### IMU (`imu_link`)

Defined at xacro line 429–448.

| Property | Value |
|----------|-------|
| Gazebo sensor type | `imu` |
| Update rate | 100 Hz |
| Angular velocity noise | σ = 0.0002 rad/s |
| Linear acceleration noise | σ = 0.017 m/s² |
| Bridged ROS topic | `/imu` (`sensor_msgs/Imu`) |

> **Note:** IMU is currently commented out in `ekf.yaml`. It is bridged but not fused by the EKF filter.

---

## Gazebo Plugins (in URDF)

All plugins are declared inside the `<gazebo>` tag at the bottom of `test.urdf.xacro`.

| Plugin filename | Class | Purpose |
|-----------------|-------|---------|
| `gz-sim-mecanum-drive-system` | `gz::sim::systems::MecanumDrive` | Drives all four wheels; publishes odometry and TF |
| `gz-sim-joint-state-publisher-system` | `gz::sim::systems::JointStatePublisher` | Publishes `/world/.../joint_state` → bridged to `/joint_states` |
| `gz-sim-joint-position-controller-system` | `gz::sim::systems::JointPositionController` | PID controller for `lidar_joint` — receives commands on `/model/test_robot/joint/lidar_joint/0/cmd_pos` |

### MecanumDrive topic mapping (xacro lines 470–476)

| Parameter | Value |
|-----------|-------|
| Command topic (GZ) | `/model/test_robot/cmd_vel` |
| Odometry topic (GZ) | `/model/test_robot/odometry` |
| TF topic (GZ) | `/model/test_robot/tf` |
| `frame_id` | `odom` |
| `child_frame_id` | `base_footprint` |

The `ros_gz_bridge` maps these Gazebo topics to the ROS topics `/test_robot/cmd_vel` and `/test_robot/odometry`.

---

## Wheel Friction Configuration

Each wheel has custom ODE friction parameters set via `<gazebo reference>` tags (xacro lines 234–264):

- `mu` (longitudinal friction) = 1.5  
- `mu2` (lateral friction) = 0.0 (allows mecanum lateral slip)  
- `fdir1` is expressed in `base_footprint` to correctly orient friction axes per wheel
