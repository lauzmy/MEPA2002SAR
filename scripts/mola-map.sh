#!/usr/bin/env bash
# Run MOLA-LO offline against a recorded bag from this project.
#
# Unlike `mola-lo-gui-rosbag2` (which uses an upstream system YAML that may
# omit the wheel-odometry sensor entry), this wrapper invokes `mola-cli`
# directly against our own MEPA2002SAR-tailored system YAML, ensuring:
#   - LiDAR topic           : /lidar3d/points (3-D cloud built by lidar3d)
#   - Wheel odometry topic  : /wheel/odometry (nav_msgs/Odometry, mecanum)
#   - State estimator       : Smoother (only one that fuses wheel odometry)
#   - Planar motion         : enforced (flat floor, pins z/roll/pitch to 0)
#
# Usage:
#   ./mola-map.sh <bag_path>          [extra mola-cli args...]
#   ./mola-map.sh -l | --latest       [extra mola-cli args...]
#
# Examples:
#   ./mola-map.sh ~/ros2_ws/rosbags/3d_mapping_2026-05-10_03-01-13
#   ./mola-map.sh --latest
#
# Override env vars before invoking if you need to tweak something:
#   MOLA_LIDAR_TOPIC=/foo ./mola-map.sh -l
#   ROSBAGS_DIR=/some/other/dir ./mola-map.sh -l
#   MOLA_STATE_ESTIMATOR=mola::state_estimation_simple::StateEstimationSimple ./mola-map.sh -l

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
ROSBAGS_DIR="${ROSBAGS_DIR:-$HOME/ros2_ws/rosbags}"
SYSTEM_YAML="${MOLA_SYSTEM_YAML:-$SCRIPT_DIR/mola_system.yaml}"
SYSTEM_YAML_NOVIZ="${MOLA_SYSTEM_YAML_NOVIZ:-$SCRIPT_DIR/mola_system_noviz.yaml}"

usage() {
    echo "Usage: $0 [--noviz] <bag_path> [extra mola-cli args...]" >&2
    echo "       $0 [--noviz] -l | --latest    [extra mola-cli args...]" >&2
}

# ---- argument parsing -------------------------------------------------------
NOVIZ=0
if [ "$#" -ge 1 ] && { [ "$1" = "--noviz" ] || [ "$1" = "-n" ]; }; then
    NOVIZ=1
    shift 1
fi

if [ "$#" -lt 1 ]; then
    usage
    exit 1
fi

case "$1" in
    -l|--latest)
        shift 1
        if [ ! -d "$ROSBAGS_DIR" ]; then
            echo "Error: rosbags directory does not exist: $ROSBAGS_DIR" >&2
            exit 1
        fi
        BAG="$(ls -1dt "$ROSBAGS_DIR"/*/ "$ROSBAGS_DIR"/*.mcap 2>/dev/null | head -n 1 || true)"
        BAG="${BAG%/}"
        if [ -z "$BAG" ]; then
            echo "Error: no bags found in $ROSBAGS_DIR" >&2
            exit 1
        fi
        echo "[mola-map] --latest resolved to: $BAG"
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        BAG="$1"
        shift 1
        ;;
esac

if [ ! -e "$BAG" ]; then
    echo "Error: bag path does not exist: $BAG" >&2
    exit 1
fi
if [ "$NOVIZ" = "1" ]; then
    SYSTEM_YAML="$SYSTEM_YAML_NOVIZ"
    echo "[mola-map] --noviz: using headless system YAML"
fi
if [ ! -f "$SYSTEM_YAML" ]; then
    echo "Error: MOLA system YAML not found: $SYSTEM_YAML" >&2
    exit 1
fi

# ---- resolve absolute paths to bundled YAMLs --------------------------------
# Our system YAML defaults reference relative paths like
# "../pipelines/lidar3d-default.yaml" and
# "../state-estimator-params/state-estimation-smoother.yaml". Those are
# relative to mola-cli-launchs/, which doesn't exist next to our YAML, so we
# resolve them to absolute install paths instead.

MOLA_LO_PREFIX="$(ros2 pkg prefix mola_lidar_odometry 2>/dev/null || true)"
if [ -z "$MOLA_LO_PREFIX" ]; then
    echo "Error: cannot locate mola_lidar_odometry package (ros2 pkg prefix failed)." >&2
    echo "       Have you sourced your ROS 2 environment?" >&2
    exit 1
fi
MOLA_LO_SHARE="$MOLA_LO_PREFIX/share/mola_lidar_odometry"

if [ -z "${MOLA_ODOMETRY_PIPELINE_YAML:-}" ]; then
    # NOTE: We tried lidar3d-gicp-optimize-twist.yaml. With our slow indoor
    # motion, sparse single-LD06 cloud, and no IMU, the twist solver had no
    # observability and diverged into long radial streaks through origin
    # (see git history). The default pipeline + a tight motion-model prior
    # from the wheel-odom-fed Simple estimator gives much better deskew.
    export MOLA_ODOMETRY_PIPELINE_YAML="$MOLA_LO_SHARE/pipelines/lidar3d-default.yaml"
fi
export MOLA_OPTIMIZE_TWIST="${MOLA_OPTIMIZE_TWIST:-false}"

if [ -z "${MOLA_STATE_ESTIMATOR_YAML:-}" ] && \
   [[ "${MOLA_STATE_ESTIMATOR:-}" == *"StateEstimationSmoother"* ]]; then
    # Prefer our local override (which adds the `kinematic_model` key the
    # installed smoother requires but the bundled YAML omits). Fall back to
    # bundled YAMLs if our copy is absent.
    if [ -f "$SCRIPT_DIR/state-estimation-smoother.yaml" ]; then
        SMOOTHER_YAML="$SCRIPT_DIR/state-estimation-smoother.yaml"
    else
        SMOOTHER_YAML="$MOLA_LO_SHARE/state-estimator-params/state-estimation-smoother.yaml"
        if [ ! -f "$SMOOTHER_YAML" ]; then
            SMOOTHER_PREFIX="$(ros2 pkg prefix mola_state_estimation_smoother 2>/dev/null || true)"
            if [ -n "$SMOOTHER_PREFIX" ] && \
               [ -f "$SMOOTHER_PREFIX/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml" ]; then
                SMOOTHER_YAML="$SMOOTHER_PREFIX/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml"
            fi
        fi
    fi
    if [ -f "$SMOOTHER_YAML" ]; then
        export MOLA_STATE_ESTIMATOR_YAML="$SMOOTHER_YAML"
    else
        echo "Warning: state-estimation-smoother.yaml not found; smoother will use C++ defaults." >&2
    fi
fi

# Default Simple-estimator YAML path (when smoother not selected). Search the
# usual install locations.
if [ -z "${MOLA_STATE_ESTIMATOR_YAML:-}" ]; then
    SIMPLE_CANDIDATES=(
        "$SCRIPT_DIR/state-estimation-simple.yaml"
        "$MOLA_LO_SHARE/state-estimator-params/state-estimation-simple.yaml"
        "$(ros2 pkg prefix mola_state_estimation_simple 2>/dev/null || true)/share/mola_state_estimation_simple/params/state-estimation-simple.yaml"
    )
    for c in "${SIMPLE_CANDIDATES[@]}"; do
        if [ -n "$c" ] && [ -f "$c" ]; then
            export MOLA_STATE_ESTIMATOR_YAML="$c"
            break
        fi
    done
    if [ -z "${MOLA_STATE_ESTIMATOR_YAML:-}" ]; then
        echo "Warning: state-estimation-simple.yaml not found; using C++ defaults." >&2
    fi
fi

# ---- topic / behaviour env vars (all overridable) ---------------------------
export MOLA_INPUT_ROSBAG2="$BAG"
export MOLA_LIDAR_TOPIC="${MOLA_LIDAR_TOPIC:-/lidar3d/points}"
# Prefer the EKF-fused odometry over the raw wheel odom: in sim, EKF
# combines MecanumDrive twist with the gyro yaw rate, smoothing any
# kinematics-only quirks. Override with MOLA_ODOMETRY_TOPIC=/wheel/odometry
# to compare against the raw signal.
export MOLA_ODOMETRY_TOPIC="${MOLA_ODOMETRY_TOPIC:-/odometry/filtered}"
# REP-105: odom → base_footprint → base_link. The MecanumDrive plugin
# publishes odom→base_footprint, so anchor MOLA on base_footprint to avoid
# a constant ~25 mm Z bias from chaining through base_link.
export MOLA_TF_BASE_LINK="${MOLA_TF_BASE_LINK:-base_footprint}"
# Default to Simple, NOT Smoother. The Smoother shipped in jazzy
# (libmola_state_estimation_smoother) segfaults inside
# `process_pending_gtsam_updates()` on the first lidar scan when fed our bag,
# triggered by an invalid (UINT64_MAX) timestamp on a TF observation it stores
# as kf_idx=0. Simple ALSO fuses our explicit CObservationOdometry entry,
# which is what we actually need. Override with:
#   MOLA_STATE_ESTIMATOR=mola::state_estimation_smoother::StateEstimationSmoother
export MOLA_STATE_ESTIMATOR="${MOLA_STATE_ESTIMATOR:-mola::state_estimation_simple::StateEstimationSimple}"
export MOLA_NAVSTATE_KINEMATIC_MODEL="${MOLA_NAVSTATE_KINEMATIC_MODEL:-KinematicModel::ConstantVelocity}"
export MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION="${MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION:-True}"

# ---- pipeline tweaks for our (no-IMU, no-intensity) LD06 build --------------
# Our /lidar3d/points cloud has only x,y,z (no intensity field). The bundled
# lidar3d-default.yaml tries to colour the GUI cloud by `intensity`, which
# segfaults on the first viz update. Use `z` instead.
export MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD="${MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD:-z}"
export MOLA_GUI_LAST_CLOUDS_COLOR_FIELD="${MOLA_GUI_LAST_CLOUDS_COLOR_FIELD:-z}"
# We have no IMU, so disable gravity correction (would otherwise log warnings
# every frame and may interfere with smoother priors).
export MOLA_IMU_GRAVITY_CORRECTION="${MOLA_IMU_GRAVITY_CORRECTION:-false}"
# LD06 indoor range is ~0.1–12 m; the upstream default of 5 m would gut our
# cloud. Drop it to 0.5 m so close-range features survive.
export MOLA_ABS_MIN_SENSOR_RANGE="${MOLA_ABS_MIN_SENSOR_RANGE:-0.5}"

cat <<EOF
[mola-map] Bag                      : $BAG
[mola-map] System YAML              : $SYSTEM_YAML
[mola-map] LO pipeline YAML         : $MOLA_ODOMETRY_PIPELINE_YAML
[mola-map] State estimator          : $MOLA_STATE_ESTIMATOR
[mola-map] State estimator YAML     : ${MOLA_STATE_ESTIMATOR_YAML:-<defaults>}
[mola-map] LIDAR topic              : $MOLA_LIDAR_TOPIC
[mola-map] Odometry topic           : $MOLA_ODOMETRY_TOPIC
[mola-map] base_link tf frame       : $MOLA_TF_BASE_LINK
[mola-map] Kinematic model          : $MOLA_NAVSTATE_KINEMATIC_MODEL
[mola-map] Enforce planar motion    : $MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION

EOF

exec mola-cli "$SYSTEM_YAML" "$@"
