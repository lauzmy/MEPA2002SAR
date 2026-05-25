# Restructure Spec — 2026-05-25

Follows the [code cleanup pass](cleanup-spec.md). Drivers:

- **Onboarding pain**: too many "wait, which package is this" moments. Goal: one canonical layout someone can grasp by reading the tree.
- **Cruft drift**: deprecated launches, backup URDFs, misplaced nodes accumulating. Goal: delete what isn't used.
- **Generic template names**: `ros_gz_application`, `ros_gz_bringup`, `ros_gz_description`, `ros_gz_example_gazebo` came from the upstream `ros_gz_project` example. Goal: rename to reflect Gregor ownership.

## Non-goals

- Splitting firmware / electronics / wiki into separate repos.
- Converting vendored deps (`m-explore-ros2`, `sensors/ldlidar`) to submodules — defer.
- Re-architecting node responsibilities — that was the cleanup pass.

## Changes

### Deletions (one commit)

- `src/GregorURDF/urdf/gregor.urdf.bak`, `gregor.urdf.bak2` — backup files in version control.
- `src/GregorURDF/urdf/Gergor_rotert.urdf` — experimental rotated variant (typo'd filename).
- `src/ros_gz_bringup/launch/gregor.launch.py` — deprecated; superseded by `3dMapping.launch.py`'s sim branch.
- `src/ros_gz_bringup/launch/test_xacro.launch.py` — deprecated.
- `src/ros_gz_bringup/launch/lidar3d_test.launch.py` — deprecated.
- `src/ros_gz_application/launch/cmd_vel_publisher.py` — misplaced (a node, not a launch) and unused. Launch dir now empty and removed.
- Kept: `mola_lo.launch.py`. Marked not-actively-maintained but `3dMapping.launch.py` imports it for live MOLA-LO.

### Renames (one commit per package)

| Before | After | Notes |
|---|---|---|
| `src/ros_gz_application/` | `src/gregor_application/` | Plus inner Python module dir; all cross-references swept (launches, imports, package.xml). |
| `src/ros_gz_bringup/` | `src/gregor_bringup/` | CMakeLists, package.xml, all `get_package_share_directory()` lookups updated. |
| `src/GregorURDF/` | `src/gregor_description/` | Directory rename only — the ROS package inside was already named `gregor_description`. |

### Merges (one commit)

`src/ros_gz_description/` + `src/ros_gz_example_gazebo/` → `src/gregor_sim_description/`

- `models/xacro_test/` (from `ros_gz_description`) and `worlds/` (from `ros_gz_example_gazebo`) collapsed into one package.
- One environment hook (`gregor_sim_description.{sh,dsv}.in`) exports both `models/` and `worlds/` on `GZ_SIM_RESOURCE_PATH`.
- The C++ plugin scaffolding (`BasicSystem.cc`, `FullSystem.cc` and headers) from `ros_gz_example_gazebo` is dropped — they were unmodified Open Robotics template code, and the test world doesn't load them.

### Docs (one commit)

- `README.md` rewritten — was still describing the upstream `ros_gz_project` template (diff-drive demo, orphaned `.md` doc links). Now describes Gregor with a real workspace map and per-mode quickstart.
- `docs/restructure-spec.md` (this file) added as the plan-of-record.

## Final layout

```
src/
├── gregor_application/        # Python nodes
├── gregor_bringup/            # Launch + configs
├── gregor_description/        # Real-robot URDF + meshes
├── gregor_sim_description/    # Sim worlds + sim model assets
├── m-explore-ros2/            # vendored
└── sensors/ldlidar/           # vendored
```

## Validation

- `python3 -c "import ast; ast.parse(...)"` passes on every touched Python file.
- `git grep` for old package names returns 0 hits in `src/`.
- `colcon build` not run from the cleanup environment; must be verified externally.

## Out of scope but flagged

- **Vendored deps** (`m-explore-ros2`, `sensors/ldlidar`): consider git submodules or `.repos` (vcstool) later. Today they're plain directories.
- **`scripts/`, `Electronics/`, `ESP32_*` at repo root**: each works, none is a ROS package. Could become their own subpackages or get their own `docs/` entries; the README points at them as-is.
- **Wiki as a sibling repo** (`MEPA2002SAR.wiki/`): cross-links from source use `# See wiki: <Page>` breadcrumbs. Many wiki pages haven't been written yet — see the cleanup spec for the full debt list.
