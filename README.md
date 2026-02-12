# MEPA2002SAR

A ROS 2 + Gazebo Sim workspace that launches simulation, bridges ROS↔Gazebo topics, and runs a small application node that publishes `cmd_vel` for a diff‑drive robot.

## Quickstart
**Prereqs:** Docker Engine with Compose plugin and `make`.

1. Build and start the container:
   - `make build`
   - `make up`
   - `make shell`
2. Build the workspace in the container:
   - `colcon build --cmake-args -DBUILD_TESTING=ON`
   - `source install/setup.sh`
3. Launch the diff‑drive demo:
   - `ros2 launch ros_gz_bringup diff_drive.launch.py`

**Optional GUI tools (RViz/RQt):**
- `make build DESKTOP=1`
- `make recreate`
- `make x11`

## Documentation
- [Architecture](Architecture.md)
- [Setup & Build](Setup.md)
- [Bringup & Launch](Bringup.md)
- [Workspace Layout](Workspace-Layout.md)
- [Packages](Packages.md)
- [Simulation Assets](Simulation-Assets.md)
- [Development Guide](Development-Guide.md)
