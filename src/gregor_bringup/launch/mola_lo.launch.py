"""Live MOLA-LO for MEPA2002SAR (sim or real).

Wraps the upstream ``mola_lidar_odometry`` online launch
(``ros2-lidar-odometry.launch.py``) with the env vars / YAMLs we use
for offline replay (``scripts/mola-map.sh``), so the same pipeline,
state estimator and tuning apply both modes.

Usage
-----
Terminal A — bring up sim (or real robot):
    ros2 launch gregor_bringup 3dMapping.launch.py

Terminal B — run live SLAM:
    ros2 launch gregor_bringup mola_lo.launch.py

Override topics if needed:
    ros2 launch gregor_bringup mola_lo.launch.py \\
        lidar_topic:=/lidar3d/points odom_topic:=/odometry/filtered

Notes
-----
* MOLA's BridgeROS2 publishes ``map -> odom`` itself (REP-105 indirect
  mode), so robot_localization keeps owning ``odom -> base_footprint``
  and there is no conflict.
* For sim use ``use_sim_time:=true`` (the default here). For the real
  robot pass ``use_sim_time:=false``.
* The smoother estimator gives slightly better fusion of wheel-odom +
  lidar but is more brittle on first-scan timing; default is the
  Simple estimator, matching the offline script.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_mola_lo = get_package_share_directory('mola_lidar_odometry')

    # ---- Launch arguments --------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Sim mode — should be true when running with Gazebo.'),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/lidar3d/points',
            description='PointCloud2 topic from the lidar3d node.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odometry/filtered',
            description='Wheel/EKF odometry feeding the state estimator. '
                        'Use /wheel/odometry to bypass the EKF.'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu',
            description='IMU topic (set to "" to disable).'),
        DeclareLaunchArgument(
            'base_link_frame', default_value='base_footprint',
            description='REP-105 base frame anchored by the EKF.'),
        # See wiki: MolaLo/Smoother-vs-Simple.
        DeclareLaunchArgument(
            'use_smoother', default_value='False',
            description='If true, use the Smoother state estimator '
                        '(sliding-window factor graph). False = Simple.'),
        DeclareLaunchArgument(
            'with_gui', default_value='True',
            description='Enable the MolaViz GUI.'),
    ]

    use_sim_time   = LaunchConfiguration('use_sim_time')
    lidar_topic    = LaunchConfiguration('lidar_topic')
    odom_topic     = LaunchConfiguration('odom_topic')
    imu_topic      = LaunchConfiguration('imu_topic')
    base_link      = LaunchConfiguration('base_link_frame')
    use_smoother   = LaunchConfiguration('use_smoother')
    with_gui       = LaunchConfiguration('with_gui')

    # ---- Path resolution ---------------------------------------------------
    # Reuse our local state-estimator YAMLs (same defaults as the offline
    # mola-map.sh script). The upstream system YAML expects an env var.
    bringup_scripts = os.path.join(get_package_share_directory('gregor_bringup'), 'scripts')
    simple_yaml = os.path.join(bringup_scripts, 'state-estimation-simple.yaml')
    smoother_yaml = os.path.join(bringup_scripts, 'state-estimation-smoother.yaml')

    # See wiki: MolaLo/Pipelines for why lidar3d-default over lidar3d-gicp-optimize-twist.
    pipeline_yaml = os.path.join(
        pkg_mola_lo, 'pipelines', 'lidar3d-default.yaml')

    # ---- Env vars (only those upstream has NO launch arg for) -------------
    # Topic / frame / GUI / use_sim_time are forwarded via launch_arguments
    # below (upstream sets the corresponding env vars itself). Setting them
    # here too would be harmless but would mask the upstream values, so
    # we only set what upstream doesn't expose.
    env_vars = [
        # Pipeline + estimator YAMLs (upstream picks the right one based
        # on use_state_estimator; we override its auto-resolved path).
        SetEnvironmentVariable('MOLA_ODOMETRY_PIPELINE_YAML', pipeline_yaml),

        # Pipeline / GUI tweaks identical to mola-map.sh
        SetEnvironmentVariable('MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD', 'z'),
        SetEnvironmentVariable('MOLA_GUI_LAST_CLOUDS_COLOR_FIELD',  'z'),
        SetEnvironmentVariable('MOLA_ABS_MIN_SENSOR_RANGE', '0.5'),
        SetEnvironmentVariable('MOLA_OPTIMIZE_TWIST', 'false'),
        SetEnvironmentVariable('MOLA_NAVSTATE_KINEMATIC_MODEL',
                               'KinematicModel::ConstantVelocity'),

        # Tight gravity-derived roll/pitch prior in the ICP solver.
        # Default is 2.0 deg; we drop it to 0.5 deg because:
        #   - the BNO085 (real) and Gazebo IMU (sim) both report
        #     gravity to <<1° on a static base, and
        #   - we now look up the correct base_link → imu_link TF
        #     (`ignore_imu_pose_from_tf:=false`, see launch_arguments
        #     below + the <gz_frame_id>imu_link</gz_frame_id> SDF tag
        #     in gregor.urdf), so the gravity vector is rotated into
        #     base_link with no offset error.
        # This is what stops the local map from tilting during turns:
        # the only constraint on roll/pitch from a single tilted 2D
        # ring is degenerate, so without a strong gravity prior the ICP
        # GN solver explains pure-yaw motion as a roll/pitch+yaw screw.
        # MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION still pins the published
        # trajectory to z=0/roll=pitch=0, but it acts AFTER ICP — by
        # then the local map has already accumulated tilted scans. The
        # gravity prior fixes it at the source.
        SetEnvironmentVariable('MOLA_IMU_GRAVITY_SIGMA_DEG', '0.5'),
        # NOTE: do NOT set MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION here. The
        # upstream online launch already passes `enforce_planar_motion`
        # via its launch arg (we set it below), which configures the
        # state estimator through the supported path. Setting the env var
        # in addition lands in a different parameter slot in the auto-
        # generated online system YAML and was observed to destabilise
        # the smoother (re-introduced "timestamps went backwards" warns
        # and made map-frame Z drift worse). The offline `mola-map.sh`
        # script does set the env var, but only because the offline
        # `mola_system.yaml` reads it explicitly.

        # Live: stamp MOLA's outputs with sim/wall clock to match
        # use_sim_time on the included launch.
        SetEnvironmentVariable('MOLA_ROS2_PUBLISH_IN_SIM_TIME',
                               use_sim_time),

        # Disable MOLA's built-in `base_footprint -> base_link` static
        # /tf broadcast. By default BridgeROS2 publishes one at startup
        # using `base_footprint_frame -> base_link_frame =
        # base_footprint_to_base_link_tf` (default identity). That
        # conflicts with the URDF static joint already broadcast by
        # robot_state_publisher (xyz=(0,0,0.025), yaw=90°), giving
        # base_link two parents and tf2 reports a "tree contains a
        # loop". Setting MOLA_TF_FOOTPRINT_LINK to an empty string
        # makes the YAML's `base_footprint_frame` empty, which the
        # BridgeROS2 source treats as "don't broadcast". The
        # publish_localization_following_rep105 lookup still works:
        # tf2 resolves `odom -> base_link` through the chain
        # `odom -> base_footprint` (EKF) + `base_footprint -> base_link`
        # (RSP) automatically.
        SetEnvironmentVariable('MOLA_TF_FOOTPRINT_LINK', ''),
    ]

    # Pick the right state-estimator YAML based on use_smoother.
    se_yaml = PythonExpression([
        "'", smoother_yaml, "' if '", use_smoother,
        "' .lower() in ('1','true','yes','on') else '", simple_yaml, "'"
    ])

    # ---- Include upstream launch ------------------------------------------
    upstream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_mola_lo, 'ros2-launchs', 'ros2-lidar-odometry.launch.py')),
        launch_arguments={
            'use_sim_time':                use_sim_time,
            'lidar_topic_name':            lidar_topic,
            'lidar_topic_type':            'PointCloud2',
            'imu_topic_name':              imu_topic,
            'odom_topic_name':             odom_topic,
            'mola_tf_base_link':           base_link,
            'use_state_estimator':         use_smoother,
            'state_estimator_config_yaml': se_yaml,
            'use_mola_gui':                with_gui,
            # Suppress the upstream RViz instance — 3dMapping.launch.py
            # already starts our own RViz with the project layout.
            'use_rviz':                    'False',
            # Forward the EKF's `odom -> base_footprint` /tf into MOLA's
            # state estimator as a motion prior. Without this MOLA's
            # navstate has only lidar + IMU; whenever an ICP step is
            # weak (fast rotation, low-overlap cloud, sparse 1 s sweep),
            # the pose simply freezes until the next good ICP snaps it
            # forward — and any motion accumulated in between is lost
            # ("robot keeps moving, map stays still, then catches up
            # without picking up the extra movement"). With this on,
            # MOLA integrates wheel/IMU-fused odometry between ICPs and
            # the symptom disappears. The forwarded odom is treated as
            # a *relative* prior (sigmas come from
            # `state-estimation-simple.yaml`'s `sigma_relative_pose_*`
            # entries), so it never overrides ICP when ICP is healthy.
            'forward_ros_tf_odom_to_mola': 'True',
            'enforce_planar_motion':       'True',
            # IMU accelerometer-based verticality correction. Constrains
            # ICP roll/pitch using the gravity vector averaged over the
            # last few IMU samples (see `imu_gravity_correction` block in
            # mola_lidar_odometry/pipelines/lidar3d-default.yaml).
            # Without it, the only constraint on roll/pitch is the
            # geometry inside one 1 s sweep cloud, which is too sparse
            # for stable verticality on a mecanum platform — over time
            # the local map tilts and "walls move" / "rotation drifts
            # randomly" as MOLA re-levels each ICP cycle. Upstream
            # default is `true`; we explicitly enable it here. The
            # `ignore_imu_pose_from_tf:=true` flag above keeps MOLA from
            # looking up a non-existent IMU sensor TF; gravity
            # correction itself only needs the raw accel readings.
            'imu_gravity_correction':      'True',
            # IMU sensor pose lookup. The Gazebo SDF now sets
            # `<gz_frame_id>imu_link</gz_frame_id>` on the IMU sensor,
            # and the real BNO085 driver (gregor_application/IMU.py)
            # publishes /imu/data with frame_id="imu_link" too. Both map
            # to the URDF static TF base_link → imu_link broadcast by
            # robot_state_publisher, so MOLA can correctly rotate the
            # measured gravity vector into base_link before applying the
            # roll/pitch prior. This fixes the "map tilts during turns"
            # symptom that came from MOLA assuming identity IMU pose
            # while the real IMU axes were yawed -90° relative to
            # base_link (imu_link is a child of upper_level_link).
            'ignore_imu_pose_from_tf':     'false',
            # The 3D point cloud `/lidar3d/points` is projected into
            # `base_link` by lidar3d.py (see `output_frame` param).
            # MOLA looks up `base_link -> mola_tf_base_link` from /tf,
            # which is the identity URDF static transform broadcast by
            # robot_state_publisher (RSP). With MOLA_TF_FOOTPRINT_LINK
            # disabled (see env_vars above) there is no longer a
            # competing publisher, so leave this on its default `false`
            # — MOLA reads the real pose from /tf and we get a
            # geometrically correct map frame.
        }.items(),
    )

    return LaunchDescription(args + env_vars + [upstream_launch])
