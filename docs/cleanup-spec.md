# Code Cleanup Spec — 2026-05-23

## Goal

Make the MEPA2002SAR codebase consistent, readable, and trustworthy. Trustworthy means: if you read it, what you see is what it does — no stale or hallucinated comments, no dead code, no Norwegian/English mixing.

This is a **code-first** pass. The wiki revamp comes later and will lean on the source code being trustworthy first.

## Non-goals

- Refactoring node behavior or splitting files into modules.
- Touching vendored code (`m-explore-ros2`, `sensors/ldlidar`).
- Documentation/wiki rewrites.
- New features, performance work, test infrastructure.

## Scope (in priority order)

1. **Python nodes** — `MEPA2002SAR/src/ros_gz_application/nodes/*.py` (~2100 lines, 9 files).
2. **Launch files** — `MEPA2002SAR/src/ros_gz_bringup/launch/*.py` and `ros_gz_application/launch/*.py`. Cleaned alongside the node they launch.
3. **Robot description** — `GregorURDF/` and `ros_gz_description/` (URDF/xacro, RViz/Gazebo configs).
4. **ESP32 firmware** — `hardware/firmware/motor_controller/`, `hardware/firmware/encoder/` (C++).

## The standard

See [style-guide.md](style-guide.md). In short:

- English everywhere.
- Descriptive names carry the meaning that comments used to.
- Only `why` comments survive. Section dividers (`# --- Name ---`) allowed.
- One-line module docstring required; function docstrings only when non-obvious.
- Standard file template: imports → constants → class (params → pubs/subs → state → timers → callbacks → helpers) → `main()`.

## Workflow per file

1. **Read and understand** — read end-to-end; flag anything unclear to the user before guessing intent.
2. **Rename** — apply naming rules. This is most of the work.
3. **Restructure** — reorder to match the template, add section dividers.
4. **Comments** — strip `what` comments and AI restate-the-code noise; translate any remaining Norwegian; keep only `why` comments.
5. **Validate** — `colcon build --packages-select <pkg>`. For sensor/processing nodes, replay a representative rosbag from `/rosbags/` and confirm published topics still look right. For nodes that need real hardware (e.g. `allocator.py` ↔ ESP32 over UART), build-only and flag a manual test.
6. **Commit** — one commit per file. Message format: `cleanup(<filename>): apply style guide, rename for clarity`.

## Node order

Small and isolated → large and coupled:

| # | File | Lines | Notes |
|---|------|-------|-------|
| 1 | `IMU.py` | 189 | **Reference node** — first to be cleaned; canonical example. |
| 2 | `collision_avoidance.py` | 80 | |
| 3 | `thermal_reading.py` | 81 | |
| 4 | `thermal_processing.py` | 87 | |
| 5 | `YOLO.py` | 127 | |
| 6 | `YOLO_gstreamer.py` | 131 | |
| 7 | `allocator.py` | 286 | Hardware-coupled; build-only validation. |
| 8 | `lidar_leveler.py` | 315 | |
| 9 | `lidar3d.py` | 823 | Last — benefits most from a settled template. |

Launch files are touched as their owning node is touched. URDF and ESP32 firmware each get a final batch pass.

## Validation

- **Build gate:** `colcon build --packages-select ros_gz_application` must succeed before commit.
- **Behavioral gate:** for nodes that consume rosbag-replayable topics (IMU, lidar3d, lidar_leveler, thermal, YOLO, collision_avoidance), replay a relevant bag from `/rosbags/` and spot-check published output.
- **Hardware-only nodes:** flagged in the commit message as needing manual test.

## Out of scope but worth noting

- A future wiki revamp should reference [`style-guide.md`](style-guide.md) and the cleaned `IMU.py` as the trusted entry points.
- If a node's intent is genuinely unclear during the read step, that's a signal to stop and ask, not refactor.
