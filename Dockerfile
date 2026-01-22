FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Build-time switch:
#   INSTALL_DESKTOP=1  -> install ROS 2 desktop meta-package (GUI tools like rqt/rviz)
#   INSTALL_DESKTOP=0  -> headless image (smaller, faster to build)
ARG INSTALL_DESKTOP=1

# Install a minimal set of packages first so we can create the user early.
# This makes UID/GID collisions fail fast instead of after large package installs.
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales sudo ca-certificates bash-completion \
 && rm -rf /var/lib/apt/lists/*

# Disable the sudo lecture message (noise in interactive shells).
RUN echo 'Defaults lecture = never' > /etc/sudoers.d/00-lecture \
 && chmod 0440 /etc/sudoers.d/00-lecture

# Configure UTF-8 locale to avoid issues with Python/ROS tools and terminal output.
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create a non-root user for development inside the container.
# The base image may already contain UID/GID 1000, so the logic is defensive.
# Some groups may not exist on all variants, so we only add groups that exist.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN set -eux; \
    getent group "${USER_GID}" >/dev/null || groupadd -g "${USER_GID}" "${USERNAME}"; \
    if getent passwd "${USER_UID}" >/dev/null; then \
      u="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
      if [ "$u" != "${USERNAME}" ]; then usermod -l "${USERNAME}" "$u"; fi; \
      usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
      usermod -g "${USER_GID}" "${USERNAME}"; \
    else \
      useradd -m -u "${USER_UID}" -g "${USER_GID}" "${USERNAME}"; \
    fi; \
    for g in sudo dialout video render plugdev; do getent group "$g" >/dev/null && usermod -aG "$g" "${USERNAME}" || true; done; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >"/etc/sudoers.d/${USERNAME}"; chmod 0440 "/etc/sudoers.d/${USERNAME}"

# Install development tools and ROS-related build tooling.
# Desktop install is conditional to support a smaller headless variant.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git nano vim tmux htop less curl wget \
    build-essential cmake pkg-config \
    python3-pip python3-venv python3-colcon-common-extensions python3-rosdep python3-vcstool \
 && if [ "${INSTALL_DESKTOP}" = "1" ]; then \
      apt-get install -y --no-install-recommends ros-jazzy-desktop; \
    fi \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (best-effort / idempotent).
RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init \
 && rosdep update

# Default ROS workspace location (mounted from host in normal use).
ENV ROS_WS=/home/${USERNAME}/ros2_ws
RUN mkdir -p ${ROS_WS}/src && chown -R ${USERNAME} /home/${USERNAME}

# Create baseline snapshots at build time.
# Pip baseline tracks ONLY user-installed packages (pip install --user), which should be empty initially.
RUN apt-mark showmanual | sort > /opt/baseline_apt_manual.txt \
 && runuser -u ${USERNAME} -- python3 -m pip list --user --format=freeze | sort > /opt/baseline_pip_user.txt \
 && touch /opt/baseline_rosdep_keys.txt

# Copy helper scripts (deps-status, deps-export-*, rosdep-install wrapper, etc.)
# Keep this late so edits to scripts do not invalidate earlier build cache.
COPY scripts/ /usr/local/bin/
RUN chmod +x /usr/local/bin/deps-* /usr/local/bin/rosdep-install

# Ensure ROS environment is sourced for all interactive shells.
# Also run deps-status on shell start (non-fatal).
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
 && echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> /etc/bash.bashrc \
 && echo "deps-status || true" >> /etc/bash.bashrc

# Enable colored prompt and tag this shell as (ros2)
RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc \
 && echo '' >> /home/${USERNAME}/.bashrc \
 && echo '# Custom prompt: user@host(ros2):path$' >> /home/${USERNAME}/.bashrc \
 && echo 'if [[ $- == *i* ]]; then' >> /home/${USERNAME}/.bashrc \
 && echo '  PS1="\\[\\e[1;32m\\]\\u@\\h\\[\\e[0m\\]\\[\\e[1;33m\\](ros2)\\[\\e[0m\\]:\\[\\e[1;34m\\]\\w\\[\\e[0m\\]\\$ "' >> /home/${USERNAME}/.bashrc \
 && echo 'fi' >> /home/${USERNAME}/.bashrc \
 && chown ${USERNAME} /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}
USER ${USERNAME}
CMD ["bash"]
