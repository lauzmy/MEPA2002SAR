#!/usr/bin/env bash
# Run MOLA-LO offline against a recorded bag. See wiki: MolaLo/Offline-Mapping.
#
# Usage:
#   ./mola-map.sh [--noviz] <bag_path>     [extra mola-cli args...]
#   ./mola-map.sh [--noviz] -l | --latest  [extra mola-cli args...]
#
# Override behaviour via env vars (see the wiki page for the full set):
#   ROSBAGS_DIR              — where --latest looks (default: ~/ros2_ws/rosbags)
#   MOLA_LIDAR_TOPIC         — default /lidar3d/points
#   MOLA_ODOMETRY_TOPIC      — default /odometry/filtered (EKF-fused)
#   MOLA_STATE_ESTIMATOR     — see wiki: MolaLo/Smoother-vs-Simple
#   MOLA_SIMPLEMAP_OUTPUT    — defaults to "<bag>.simplemap"; set '' to disable

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
ROSBAGS_DIR="${ROSBAGS_DIR:-$HOME/ros2_ws/rosbags}"
SYSTEM_YAML="${MOLA_SYSTEM_YAML:-$SCRIPT_DIR/mola_system.yaml}"
SYSTEM_YAML_NOVIZ="${MOLA_SYSTEM_YAML_NOVIZ:-$SCRIPT_DIR/mola_system_noviz.yaml}"

usage() {
    echo "Usage: $0 [--noviz] <bag_path> [extra mola-cli args...]" >&2
    echo "       $0 [--noviz] -l | --latest    [extra mola-cli args...]" >&2
}

# --- Argument parsing ---
NOVIZ=0
while [ "$#" -ge 1 ]; do
    case "$1" in
        --noviz|-n) NOVIZ=1; shift 1 ;;
        *) break ;;
    esac
done

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

# --- Resolve install paths for MOLA pipeline + estimator YAMLs ---
# Our system YAML references "../pipelines/..." relative to mola-cli-launchs/
# which doesn't exist next to it; resolve to absolute install paths instead.
MOLA_LO_PREFIX="$(ros2 pkg prefix mola_lidar_odometry 2>/dev/null || true)"
if [ -z "$MOLA_LO_PREFIX" ]; then
    echo "Error: cannot locate mola_lidar_odometry — source your ROS 2 environment first." >&2
    exit 1
fi
MOLA_LO_SHARE="$MOLA_LO_PREFIX/share/mola_lidar_odometry"

# LO pipeline. See wiki: MolaLo/Pipelines for why lidar3d-default beats
# lidar3d-gicp-optimize-twist on our slow indoor + sparse LD06 cloud.
if [ -z "${MOLA_ODOMETRY_PIPELINE_YAML:-}" ]; then
    export MOLA_ODOMETRY_PIPELINE_YAML="$MOLA_LO_SHARE/pipelines/lidar3d-default.yaml"
fi
export MOLA_OPTIMIZE_TWIST="${MOLA_OPTIMIZE_TWIST:-false}"

# Smoother-estimator YAML. Our local override adds `kinematic_model` (the
# installed smoother YAML omits it). Fall back to bundled if our copy is gone.
if [ -z "${MOLA_STATE_ESTIMATOR_YAML:-}" ] && \
   [[ "${MOLA_STATE_ESTIMATOR:-}" == *"StateEstimationSmoother"* ]]; then
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

# Simple-estimator YAML (when smoother not selected). Search common install paths.
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

# Auto-derive simplemap output from bag name. MOLA_SIMPLEMAP_OUTPUT='' disables auto-save.
if [ -z "${MOLA_SIMPLEMAP_OUTPUT+x}" ]; then
    BAG_NAME="$(basename -- "$BAG")"
    export MOLA_SIMPLEMAP_OUTPUT="$(dirname -- "$BAG")/${BAG_NAME%.mcap}.simplemap"
fi

# Expected bag duration (best-effort, for the [mola-map] info banner only).
BAG_DURATION_SEC=""
META_YAML=""
if [ -d "$BAG" ]; then
    META_YAML="$BAG/metadata.yaml"
elif [ -f "${BAG%.mcap}/../metadata.yaml" ]; then
    META_YAML="$(dirname "$BAG")/metadata.yaml"
fi
if [ -n "$META_YAML" ] && [ -f "$META_YAML" ]; then
    BAG_NS=$(grep -A1 '^ *duration:' "$META_YAML" | grep 'nanoseconds:' | head -1 | awk '{print $2}' || true)
    if [ -n "$BAG_NS" ] && [ "$BAG_NS" -gt 0 ] 2>/dev/null; then
        TIME_WARP="${MOLA_TIME_WARP:-1.0}"
        BAG_DURATION_SEC=$(awk -v ns="$BAG_NS" -v tw="$TIME_WARP" 'BEGIN{printf "%.1f", ns/1e9/tw}')
    fi
fi

# --- Topics, frames, estimator (all overridable via env) ---
export MOLA_INPUT_ROSBAG2="$BAG"
export MOLA_GENERATE_SIMPLEMAP="${MOLA_GENERATE_SIMPLEMAP:-true}"
export MOLA_LIDAR_TOPIC="${MOLA_LIDAR_TOPIC:-/lidar3d/points}"
# EKF-fused odometry preferred; raw /wheel/odometry is a useful comparison override.
export MOLA_ODOMETRY_TOPIC="${MOLA_ODOMETRY_TOPIC:-/odometry/filtered}"
# Anchor on base_footprint to avoid the ~25 mm Z bias from going via base_link. See wiki: MolaLo/Frames.
export MOLA_TF_BASE_LINK="${MOLA_TF_BASE_LINK:-base_footprint}"
# Default Simple, not Smoother — Smoother segfaults on jazzy. See wiki: MolaLo/Smoother-vs-Simple.
export MOLA_STATE_ESTIMATOR="${MOLA_STATE_ESTIMATOR:-mola::state_estimation_simple::StateEstimationSimple}"
export MOLA_NAVSTATE_KINEMATIC_MODEL="${MOLA_NAVSTATE_KINEMATIC_MODEL:-KinematicModel::ConstantVelocity}"
# Lowercase `true`/`false` matters — yaml-cpp treats `True`/`False` as plain strings.
export MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION="${MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION:-true}"
# Isotropic relative-pose angular sigma. See wiki: MolaLo/SparseLD06-Tuning for why 0.03 rad.
export MOLA_NAVSTATE_SIGMA_REL_POSE_ANG="${MOLA_NAVSTATE_SIGMA_REL_POSE_ANG:-0.03}"

# --- Pipeline tweaks for our (no-intensity) LD06 build ---
# Our cloud has only x,y,z; the default GUI cloud colouring by `intensity` segfaults.
export MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD="${MOLA_GUI_CURRENT_CLOUD_COLOR_FIELD:-z}"
export MOLA_GUI_LAST_CLOUDS_COLOR_FIELD="${MOLA_GUI_LAST_CLOUDS_COLOR_FIELD:-z}"

# --- IMU intentionally not registered as a MOLA sensor. See wiki: MolaLo/IMU-Config. ---
export MOLA_NAVSTATE_IMU_SENSOR_NAME="${MOLA_NAVSTATE_IMU_SENSOR_NAME:-__none__}"
export MOLA_IMU_GRAVITY_CORRECTION="${MOLA_IMU_GRAVITY_CORRECTION:-false}"
# LD06 indoor range ~0.1-12 m; upstream default 5 m would gut our cloud.
export MOLA_ABS_MIN_SENSOR_RANGE="${MOLA_ABS_MIN_SENSOR_RANGE:-0.2}"

# --- Sparse single-LD06 indoor mapping tuning. See wiki: MolaLo/SparseLD06-Tuning. ---
export MOLA_SIMPLEMAP_MIN_XYZ="${MOLA_SIMPLEMAP_MIN_XYZ:-0.3}"          # keyframe per ~30 cm
export MOLA_SIMPLEMAP_MIN_ROT="${MOLA_SIMPLEMAP_MIN_ROT:-1}"             # keyframe per ~1°
export MOLA_MINIMUM_ICP_QUALITY="${MOLA_MINIMUM_ICP_QUALITY:-0.40}"
export MOLA_CLOUD_DECIMATION_VOXEL_SIZE="${MOLA_CLOUD_DECIMATION_VOXEL_SIZE:-0.08}"
export MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER="${MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER:-0.45}"

cat <<EOF
[mola-map] Bag                      : $BAG
[mola-map] System YAML              : $SYSTEM_YAML
[mola-map] LO pipeline YAML         : $MOLA_ODOMETRY_PIPELINE_YAML
[mola-map] State estimator          : $MOLA_STATE_ESTIMATOR
[mola-map] State estimator YAML     : ${MOLA_STATE_ESTIMATOR_YAML:-<defaults>}
[mola-map] LIDAR topic              : $MOLA_LIDAR_TOPIC
[mola-map] Odometry topic           : $MOLA_ODOMETRY_TOPIC
[mola-map] IMU                       : not registered (fuse=$MOLA_NAVSTATE_IMU_SENSOR_NAME, gravity=$MOLA_IMU_GRAVITY_CORRECTION)
[mola-map] base_link tf frame       : $MOLA_TF_BASE_LINK
[mola-map] Kinematic model          : $MOLA_NAVSTATE_KINEMATIC_MODEL
[mola-map] Enforce planar motion    : $MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION
[mola-map] Generate simplemap       : $MOLA_GENERATE_SIMPLEMAP
[mola-map] SimpleMap output         : ${MOLA_SIMPLEMAP_OUTPUT:-<auto-save disabled>}
[mola-map] Bag replay duration      : ${BAG_DURATION_SEC:+~${BAG_DURATION_SEC}s (at ${MOLA_TIME_WARP:-1.0}x)}${BAG_DURATION_SEC:-unknown}
[mola-map] --- tuning ---
[mola-map] SM keyframe min XYZ      : $MOLA_SIMPLEMAP_MIN_XYZ m
[mola-map] SM keyframe min rot      : $MOLA_SIMPLEMAP_MIN_ROT deg
[mola-map] ICP min quality          : $MOLA_MINIMUM_ICP_QUALITY
[mola-map] Voxel size               : $MOLA_CLOUD_DECIMATION_VOXEL_SIZE m
[mola-map] ICP debug log below      : $MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER

EOF

if [ "${MOLA_GENERATE_SIMPLEMAP:-true}" != "true" ]; then
    echo "[mola-map] WARNING: MOLA_GENERATE_SIMPLEMAP is not true — simplemap will NOT be built." >&2
elif [ -z "${MOLA_SIMPLEMAP_OUTPUT:-}" ]; then
    echo "[mola-map] WARNING: MOLA_SIMPLEMAP_OUTPUT is empty — auto-save is DISABLED." >&2
    echo "[mola-map]          You must press 'Save Simple Map' in the GUI AFTER the bag" >&2
    echo "[mola-map]          finishes replaying (wait for the progress to stop)." >&2
fi

exec mola-cli "$SYSTEM_YAML" "$@"
