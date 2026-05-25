# Gregor Wiki Migration Plan — 2026-05-25

Produced by `deep-wiki:wiki-architect`. Plan-of-record for rewriting
`MEPA2002SAR.wiki/` to reflect the post-restructure source tree and
fulfil the `# See wiki: <Page>` breadcrumbs left by the cleanup pass.

Source repo: `MEPA2002SAR/` (this repo).
Wiki repo: `../MEPA2002SAR.wiki/` (sibling).

---

## Part 1 — Audit of existing pages

| Page | Verdict | Reason |
|---|---|---|
| `Home.md` | **REWRITE** | References `ros_gz_bringup test_xacro.launch.py`, `gregor.launch.py` (deleted), and old package names from before the restructure. Quick-start snippet is wrong. |
| `Architecture.md` | **REWRITE** | The Mermaid diagram shows the old `ros_gz_bringup` / `ros_gz_application` graph with `lidar_sweeper` + `tilt_laser_assembler`, none of which exist under those names anymore. MOLA-LO, `lidar3d`, `lidar_leveler`, and the EKF/RSP TF chain are absent. |
| `Bringup.md` | **REWRITE** | Documents `test_xacro.launch.py` (renamed/restructured), `gregor.launch.py` (deleted), and the old bridge YAML. None of the MOLA-LO timing, sim spawn, or frame breadcrumbs are covered. |
| `Configuration-Files.md` | **REWRITE** | Documents the old `ekf.yaml`/`ekf_imu.yaml` path. The IRL vs sim config split under `config/IRL/` vs `config/sim/` does not exist in this page. MOLA pipeline YAMLs are absent. |
| `Development-Guide.md` | **REWRITE** | References old package names, instructs adding plugins to the deleted `ros_gz_example_gazebo` package, and has no mention of the Docker-first workflow or `docs/style-guide.md`. |
| `Packages.md` | **DELETE** | Entirely describes the old four-package layout (`ros_gz_*`). Content would confuse rather than orient. Replaced by the new `Architecture/Packages` page. |
| `Robot-Model.md` | **REWRITE** | References `src/ros_gz_description/models/xacro_test/test.urdf.xacro` (wrong path post-restructure). The `gregor_sim_description` world is a separate package now. Physical-robot TF chain (`laser → lidar_mount_link → upper_level_link → base_link`) is undocumented. |
| `ROS-Nodes-and-Topics.md` | **REWRITE** | Node inventory is entirely pre-restructure: `lidar_sweeper`, `laser_assembler`, and old topic names. `lidar3d`, `lidar_leveler`, `allocator`, `IMU`, `thermal_*`, `YOLO*`, `collision_avoidance`, `exploration_monitor` are all missing or wrong. |
| `Setup.md` | **REWRITE** | The quickstart installs bare ROS 2 + deps manually; the actual workflow is Docker-first (`make build / make up / make shell`). References `ros_gz_bringup test_xacro.launch.py`. MOLA-LO and `mola-map.sh` prerequisites are absent. |
| `Simulation-Assets.md` | **REWRITE** | Documents `ros_gz_example_gazebo` (deleted package), `BasicSystem.cc` / `FullSystem.cc` (dropped in restructure), and the old xacro world path. The `gregor_sim_description` merge and the Z=0.05 spawn constraint are unaddressed. |
| `Workspace-Layout.md` | **REWRITE** | Lists the pre-restructure package tree verbatim. The new `src/` layout from `docs/restructure-spec.md` and `README.md` is completely different. |
| `_Sidebar.md` | **REWRITE** | Section names and page links match none of the target structure. |
| `README.md` | **DELETE** | GitHub wiki repos auto-render `Home.md` as the root. A `README.md` in a wiki repo is dead content; the new `Home.md` replaces it. |

---

## Part 2 — Target wiki structure

```
Home

Getting Started
  Setup                         ← Docker quickstart; real-robot vs sim
  Workspace-Layout              ← new src/ tree; hardware/; docs/

Architecture
  System-Overview               ← node graph, TF chain, data flow
  Packages                      ← per-package reference (new)

Subsystems
  Lidar3D/
    Overview
    SimClock
    MonotonicStamps
    PerBeamTiming
    FlushTiming
    SweepWindow
    ZCull
    Projection
    CloudStamping
  LidarLeveler/
    Design
    PointCloud-Layout
  Allocator/
    Kinematics
    Protocol
    EKF-Tuning
  IMU/
    Gravity-Correction
  YOLO/
    CvBridgeWorkaround
  Thermal (no sub-pages needed yet)

MOLA-LO
  Offline-Mapping
  Smoother-vs-Simple
  SparseLD06-Tuning
  IMU-Config
  Frames
  Pipelines
  StateEstimator-Simple
  StateEstimator-Smoother

Bringup
  Launch-Files                  ← replaces old Bringup.md
  MolaLo-Timing
  MolaLo-Frames
  SimSpawn
  Configuration-Files           ← IRL vs sim YAML split

Hardware
  Robot-Model                   ← real URDF, meshes, TF chain
  Simulation-Assets             ← gregor_sim_description world
  Firmware                      ← motor_controller + encoder protocols

Development
  Development-Guide             ← style guide pointer, Docker workflow
```

**Sidebar (`_Sidebar.md`) mirrors this tree exactly.** Max depth shown in sidebar: two levels (section + page name, no sub-sub-pages expanded).

---

## Part 3 — Per-page briefs

### Getting Started / Setup
Replaces `Setup.md`. Docker-first workflow: `make build`, `make up`, `make shell`, `colcon build`, `source install/setup.sh`. One section for sim (`3dMapping.launch.py` with `sim: true`) and one for real-robot. Prerequisites include MOLA-LO apt packages. Anchored at `README.md` Quickstart section and `Makefile`.

### Getting Started / Workspace-Layout
Replaces `Workspace-Layout.md`. Renders the tree from `README.md` and `docs/restructure-spec.md`, explaining what lives in each package and why `hardware/` and `docs/` are at repo root. Cross-references the restructure rationale.

### Architecture / System-Overview
Replaces `Architecture.md`. Mermaid diagram of the real-robot node graph: `ldlidar → lidar3d → lidar_leveler → MOLA-LO`; `allocator → EKF → RSP`; thermal and YOLO branches. The TF chain `laser → lidar_mount_link → upper_level_link → base_link` annotated. Anchored at `src/gregor_bringup/launch/3dMapping.launch.py` and `src/gregor_bringup/launch/mola_lo.launch.py`.

### Architecture / Packages
Replaces `Packages.md` with accurate content. Table of all six packages (`gregor_application`, `gregor_bringup`, `gregor_description`, `gregor_sim_description`, `m-explore-ros2`, `sensors/ldlidar`) with role, install path, and key files. Anchored at each `package.xml`.

---

### Subsystems / Lidar3D / Overview
Satisfies `# See wiki: Lidar3D/Overview`. Describes the tilt-servo loop, per-beam 3D projection into `base_link`, and why `lidar3d` replaces `lidar_sweeper` + `laser_assembler`. Anchored at `src/gregor_application/nodes/lidar3d.py`.

### Subsystems / Lidar3D / SimClock
Satisfies `# See wiki: Lidar3D/SimClock`. Explains Gazebo `/clock` backward-jump edge cases and the guard logic that detects and drops negative time deltas. Anchored at `lidar3d.py`.

### Subsystems / Lidar3D / MonotonicStamps
Satisfies `# See wiki: Lidar3D/MonotonicStamps`. Explains why ties in `sweep_start_s` are broken by bumping 1 ms forward (MOLA-LO requires strictly increasing cloud stamps) rather than dropping the cloud. Anchored at `lidar3d.py`.

### Subsystems / Lidar3D / PerBeamTiming
Satisfies `# See wiki: Lidar3D/PerBeamTiming`. Documents the three LD06 driver quality tiers for `time_increment` / `scan_time` and how each is handled to produce correct per-beam `time` offsets in the PointCloud2. Anchored at `lidar3d.py`.

### Subsystems / Lidar3D / FlushTiming
Satisfies `# See wiki: Lidar3D/FlushTiming`. Explains why cloud flushing is triggered from `_on_scan` rather than a separate timer: a timer would re-stamp with a stale `latest_t` after a scan gap. Anchored at `lidar3d.py`.

### Subsystems / Lidar3D / SweepWindow
Satisfies `# See wiki: Lidar3D/SweepWindow`. Explains why the sweep eviction cutoff uses `<=` instead of `<`: Gazebo's `gpu_lidar` stamps every beam in a scan identically, so `<` would retain the entire previous sweep. Anchored at `lidar3d.py`.

### Subsystems / Lidar3D / ZCull
Satisfies `# See wiki: Lidar3D/ZCull`. Tilt-latency artefacts (beams projected at the wrong tilt angle near sweep boundaries) and the `min_z_m` / `max_z_m` ROS parameters used to cull them. Anchored at `lidar3d.py` and `src/gregor_bringup/config/IRL/3d_mapping.yaml`.

### Subsystems / Lidar3D / Projection
Satisfies `# See wiki: Lidar3D/Projection`. The URDF kinematic chain `laser → lidar_mount_link → upper_level_link → base_link` baked into the per-beam projection transform, and why it is baked (no live TF lookup per beam for latency reasons). Anchored at `lidar3d.py` and `src/gregor_description/urdf/gregor.urdf`.

### Subsystems / Lidar3D / CloudStamping
Satisfies `# See wiki: Lidar3D/CloudStamping`. Why the cloud header stamp is `sweep_start_s` (not the latest beam), and why individual `time` fields are non-negative offsets from that stamp — both required by MOLA-LO's timestamp contract. Anchored at `lidar3d.py`.

---

### Subsystems / LidarLeveler / Design
Satisfies `# See wiki: LidarLeveler/Design`. Explains what `lidar_leveler` does (rotates clouds to gravity-level using IMU orientation), why two variants (`_imu` — live gravity correction; `_zero` — identity rotation for debugging), and why MOLA-LO needs a leveled cloud. Anchored at `src/gregor_application/nodes/lidar_leveler.py`.

### Subsystems / LidarLeveler / PointCloud-Layout
Satisfies `# See wiki: LidarLeveler/PointCloud-Layout`. Documents how the PointCloud2 byte buffer is reinterpreted as a structured numpy array (`x, y, z, time` fields) and rotated in-place without copying. Anchored at `lidar_leveler.py`.

---

### Subsystems / Allocator / Kinematics
Satisfies `# See wiki: Allocator/Kinematics`. Mecanum IK/FK matrices, motor-to-corner mapping (M4=FL, M3=FR, M2=RL, M1=RR as wired), and the wheel-geometry constants. Anchored at `src/gregor_application/nodes/allocator.py`.

### Subsystems / Allocator / Protocol
Satisfies `# See wiki: Allocator/Protocol`. Full wire protocol for the ESP32 UART link: 8-byte packet layout, CRC-8/MAXIM polynomial `0x31`, status codes. Cross-references `hardware/firmware/motor_controller/motor_controller.ino` where the matching decode lives. Anchored at `allocator.py` and `motor_controller.ino`.

### Subsystems / Allocator / EKF-Tuning
Satisfies `# See wiki: Allocator/EKF-Tuning`. Documents why mecanum slip on `vy` and yaw rate is high, the resulting odom covariance values published, and why the EKF is configured to trust IMU over wheel odometry for heading. Anchored at `allocator.py` and `src/gregor_bringup/config/IRL/ekf_imu.yaml`.

---

### Subsystems / IMU / Gravity-Correction
Satisfies `# See wiki: IMU/Gravity-Correction`. Explains why `IMU.py` publishes raw accelerometer output (gravity included) rather than linear acceleration, and the downstream effect on `MOLA_IMU_GRAVITY_CORRECTION` in the MOLA-LO config. Anchored at `src/gregor_application/nodes/IMU.py`.

---

### Subsystems / YOLO / CvBridgeWorkaround
Satisfies `# See wiki: YOLO/CvBridgeWorkaround`. Explains the cv_bridge / numpy ABI crash on Jazzy and the workaround in `YOLO_gstreamer.py` that builds `sensor_msgs/Image` manually from a raw numpy buffer. Anchored at `src/gregor_application/nodes/YOLO_gstreamer.py`.

---

### MOLA-LO / Offline-Mapping
Satisfies `# See wiki: MolaLo/Offline-Mapping`. What `mola-map.sh` does (replay a recorded bag through MOLA-LO offline), when to pass `--noviz` vs `--latest`, and expected outputs. Anchored at `src/gregor_bringup/scripts/mola-map.sh`.

### MOLA-LO / Smoother-vs-Simple
Satisfies `# See wiki: MolaLo/Smoother-vs-Simple`. Why `state-estimation-smoother.yaml` segfaults on Jazzy (upstream issue) and why `state-estimation-simple.yaml` is the default. Anchored at `src/gregor_bringup/scripts/state-estimation-simple.yaml` and `state-estimation-smoother.yaml`.

### MOLA-LO / SparseLD06-Tuning
Satisfies `# See wiki: MolaLo/SparseLD06-Tuning`. Rationale for `MOLA_SIMPLEMAP_MIN_XYZ`, `MOLA_MINIMUM_ICP_QUALITY`, `MOLA_NAVSTATE_SIGMA_REL_POSE_ANG=0.03`, voxel size — all chosen for the LD06's sparse point density. Anchored at `src/gregor_bringup/scripts/mola_system.yaml` and `mola_system_noviz.yaml`.

### MOLA-LO / IMU-Config
Satisfies `# See wiki: MolaLo/IMU-Config`. Explains why the IMU is intentionally not registered as a MOLA sensor input: the EKF pre-fuses IMU with wheel odometry and feeds the result to MOLA-LO rather than letting MOLA-LO handle the raw IMU. Anchored at `mola_system.yaml` and `src/gregor_bringup/config/IRL/ekf_imu.yaml`.

### MOLA-LO / Frames
Satisfies `# See wiki: MolaLo/Frames`. Why `MOLA_TF_BASE_LINK=base_footprint` avoids the ~25 mm Z bias from the physical IMU mount height, and the env var `MOLA_TF_FOOTPRINT_LINK=''`. Anchored at `src/gregor_bringup/launch/mola_lo.launch.py`.

### MOLA-LO / Pipelines
Satisfies `# See wiki: MolaLo/Pipelines`. Decision record: `lidar3d-default.yaml` vs `lidar3d-gicp-optimize-twist.yaml` — when each is appropriate, what the GICP-twist variant trades off. Anchored at `mola_lo.launch.py` and the `filter-pipeline.yaml` scripts.

### MOLA-LO / StateEstimator-Simple
Satisfies `# See wiki: MolaLo/StateEstimator-Simple`. Documents tightened sigma values and the reasoning for `0.03` rel-pose angular sigma in the simple state estimator. Anchored at `state-estimation-simple.yaml`.

### MOLA-LO / StateEstimator-Smoother
Satisfies `# See wiki: MolaLo/StateEstimator-Smoother`. Documents the `kinematic_model` field the upstream MOLA-LO example omits, and what happens without it. Anchored at `state-estimation-smoother.yaml`.

---

### Bringup / Launch-Files
Replaces `Bringup.md`. Documents `3dMapping.launch.py` (primary, sim+IRL), `gregor_real.launch.py`, and `mola_lo.launch.py`. Launch arguments table, sequencing diagram, and `sim:` flag explained. Anchored at `src/gregor_bringup/launch/`.

### Bringup / MolaLo-Timing
Satisfies `# See wiki: Bringup/MolaLo-Timing`. Why an 8-second `TimerAction` delays MOLA-LO startup: the first lidar3d clouds arrive before the EKF/RSP TF chain is settled, causing MOLA to reject them. Anchored at `3dMapping.launch.py`.

### Bringup / MolaLo-Frames
Satisfies `# See wiki: Bringup/MolaLo-Frames`. The `base_link_frame=base_link` parameter, the EKF/RSP TF chain required by MOLA, and the `MOLA_TF_FOOTPRINT_LINK=''` env var. Anchored at `mola_lo.launch.py`.

### Bringup / SimSpawn
Satisfies `# See wiki: Bringup/SimSpawn`. Why Gregor spawns at Z=0.05 m (avoids floor-contact physics stutter), and the "first cloud captures mid-tilt" failure mode if the spawn pose changes. Anchored at `3dMapping.launch.py`.

### Bringup / Configuration-Files
Replaces `Configuration-Files.md`. Documents the `config/IRL/` vs `config/sim/` split for all YAML configs, with a table of every file, its owner node, and its key parameters. Anchored at `src/gregor_bringup/config/`.

---

### Hardware / Robot-Model
Replaces `Robot-Model.md`. Real-robot URDF `gregor.urdf` (exported from Fusion), link/joint tree for physical robot, URDF chain `laser → lidar_mount_link → upper_level_link → base_link`, mesh sources in `gregor_description/meshes/`. Anchored at `src/gregor_description/urdf/gregor.urdf`.

### Hardware / Simulation-Assets
Replaces `Simulation-Assets.md`. `gregor_sim_description` package layout (merged from `ros_gz_description` + `ros_gz_example_gazebo`), SDF world file, environment hook, Z=0.05 spawn note (pointer to `Bringup/SimSpawn`). Anchored at `src/gregor_sim_description/worlds/test_xacro.sdf.xacro`.

### Hardware / Firmware
New page. Motor controller and encoder ESP32 firmware roles, UART wiring, flash instructions (Arduino IDE / arduino-cli). Cross-references `Allocator/Protocol` for the packet spec. Anchored at `hardware/firmware/motor_controller/motor_controller.ino` and `hardware/firmware/encoder/encoder.ino`.

---

### Development / Development-Guide
Replaces `Development-Guide.md`. Docker workflow (`make` targets), `docs/style-guide.md` as the coding standard, how to add a node (placement in `gregor_application/nodes/`, CMakeLists install, launch wiring), `colcon build` + `source install/setup.sh`, and the `# See wiki:` breadcrumb convention. Anchored at `docs/style-guide.md`, `Makefile`, and `src/gregor_application/CMakeLists.txt`.

---

## Breadcrumb coverage check

| Source breadcrumb | Resolved to |
|---|---|
| `IMU/Gravity-Correction` | Subsystems/IMU/Gravity-Correction (with cross-ref from MOLA-LO/IMU-Config) |
| `Allocator/EKF-Tuning` | Subsystems/Allocator/EKF-Tuning |
| `Allocator/Kinematics` | Subsystems/Allocator/Kinematics |
| `Allocator/Protocol` | Subsystems/Allocator/Protocol |
| `LidarLeveler/Design` | Subsystems/LidarLeveler/Design |
| `LidarLeveler/PointCloud-Layout` | Subsystems/LidarLeveler/PointCloud-Layout |
| `Lidar3D/Overview` through `Lidar3D/CloudStamping` (8 pages) | Subsystems/Lidar3D/* (8 pages, 1:1) |
| `Bringup/MolaLo-Timing` | Bringup/MolaLo-Timing |
| `Bringup/MolaLo-Frames` | Bringup/MolaLo-Frames |
| `Bringup/SimSpawn` | Bringup/SimSpawn |
| `YOLO/CvBridgeWorkaround` | Subsystems/YOLO/CvBridgeWorkaround |
| `MolaLo/Offline-Mapping` through `MolaLo/StateEstimator-Smoother` (8 pages) | MOLA-LO/* (8 pages, 1:1) |

All 29 breadcrumbs in the debt list map to exactly one page in the target structure.
