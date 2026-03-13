#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# Install any workspace dependencies that are not yet present.
# Runs on every container start so changes to src/ are always satisfied.
if [ -d "${ROS_WS}/src" ]; then
    rosdep install --from-paths "${ROS_WS}/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO" 2>&1
fi

if [ -n "$USERNAME" ] && id "$USERNAME" >/dev/null 2>&1; then
    chown "$USERNAME:$USERNAME" "/home/$USERNAME"
    export HOME="/home/$USERNAME"
    cd "$HOME"
    exec gosu "$USERNAME" "$@"
fi

exec "$@"
