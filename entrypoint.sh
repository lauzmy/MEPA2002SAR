#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# Grant access to hardware PWM sysfs (runs as root before gosu drop)
if [ -d /sys/class/pwm/pwmchip0 ]; then
    chmod a+rw /sys/class/pwm/pwmchip0/export /sys/class/pwm/pwmchip0/unexport 2>/dev/null || true
    for dir in /sys/class/pwm/pwmchip0/pwm*; do
        [ -d "$dir" ] && chmod -R a+rw "$dir" 2>/dev/null || true
    done
fi

# Install any workspace dependencies that are not yet present.
# Runs on every container start so changes to src/ are always satisfied.
if [ -d "${ROS_WS}/MEPA2002SAR/src" ]; then
    rosdep update -n --rosdistro "$ROS_DISTRO" || true
    apt-get update || true
    rosdep install --from-paths "${ROS_WS}/MEPA2002SAR/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO" 2>&1 || true
fi

if [ -n "$USERNAME" ] && id "$USERNAME" >/dev/null 2>&1; then
    chown "$USERNAME:$USERNAME" "/home/$USERNAME"
    export HOME="/home/$USERNAME"
    cd "$HOME"
    exec gosu "$USERNAME" "$@"
fi

exec "$@"
