# MEPA2002SAR — Gregor

ROS 2 workspace + Arduino firmware + KiCad electronics for **Gregor**, a mecanum-drive search-and-rescue robot that builds 3D maps with a tilted LD06 LiDAR and MOLA-LO.

Deep-dive design docs live in the [project wiki](https://github.com/lauzmy/MEPA2002SAR/wiki).

## Repository layout

```
.
├── src/                       # ROS 2 colcon workspace
│   ├── gregor_application/        # Python nodes: lidar3d, allocator, IMU, YOLO, thermal, collision, ...
│   ├── gregor_bringup/            # Launch files + YAML configs
│   ├── gregor_description/        # Real-robot URDF + meshes (CAD-exported)
│   ├── gregor_sim_description/    # Gazebo worlds (gregor_description holds the robot URDF)
│   ├── m-explore-ros2/            # vendored: frontier exploration (explore_lite + map_merge)
│   └── sensors/ldlidar/           # vendored: LD06 LiDAR driver
├── ESP32_Motor_Controller/    # Arduino firmware for the mecanum motor controller (UART ↔ allocator.py)
├── ESP32_pot_rot_encoder/     # Arduino firmware for the tilt-pot encoder (UART ↔ lidar3d.py)
├── Electronics/               # KiCad PCB project
├── docs/                      # Style guide + cleanup/restructure specs
└── Dockerfile, Makefile, compose.yml  # Containerised dev environment
```

## Quickstart

**Prereqs:** Docker Engine with Compose plugin and `make`.

```bash
make build       # build the dev image
make up          # start the container
make shell       # exec in
```

Inside the container:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.sh
```

### Run in simulation (Gazebo)

Edit [`src/gregor_bringup/config/IRL/3d_mapping.yaml`](src/gregor_bringup/config/IRL/3d_mapping.yaml) and set `sim: true`, then:

```bash
ros2 launch gregor_bringup 3dMapping.launch.py
```

Spawns Gregor in Gazebo, starts the lidar3d + EKF + MOLA-LO stack, opens RViz, and records a rosbag for offline reprocessing.

### Run on the real robot

```bash
ros2 launch gregor_bringup gregor_real.launch.py
```

Or the unified launch with `sim: false` in the YAML:

```bash
ros2 launch gregor_bringup 3dMapping.launch.py
```

### Optional GUI from inside the container

```bash
make build DESKTOP=1
make x11
make recreate
```

## Firmware

The two ESP32 firmware projects under `ESP32_Motor_Controller/` and `ESP32_pot_rot_encoder/` talk to `gregor_application` over UART. Flash with the Arduino IDE / arduino-cli. Wire protocols are documented at the top of each `.ino` file and on the Python side in [`allocator.py`](src/gregor_application/nodes/allocator.py) and [`lidar3d.py`](src/gregor_application/nodes/lidar3d.py).

## Hardware design

`Electronics/Electronics.kicad_pro` — open with KiCad 8+.

## Development

- Coding standards: [`docs/style-guide.md`](docs/style-guide.md). Every cleaned source file follows it.
- Recent restructuring: [`docs/cleanup-spec.md`](docs/cleanup-spec.md) (style pass) and [`docs/restructure-spec.md`](docs/restructure-spec.md) (package renames).
- Long-form design rationale lives in the [wiki](https://github.com/lauzmy/MEPA2002SAR/wiki) — comments in source contain `# See wiki: <Page>` breadcrumbs where deep context exists.
