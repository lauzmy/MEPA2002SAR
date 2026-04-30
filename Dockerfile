FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic
ENV ROS_DOMAIN_ID=73
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

# Configure UTF-8 locale.
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales ca-certificates bash-completion \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install dev tools, colcon, rosdep, and the full ROS 2 desktop.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git nano vim tmux htop less curl wget \
    build-essential cmake pkg-config \
    python3-pip python3-venv python3-colcon-common-extensions python3-rosdep python3-vcstool gosu \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-cmake-vendor \
    ros-jazzy-gz-common-vendor \
    ros-jazzy-gz-plugin-vendor \
    ros-jazzy-gz-sim-vendor \
 && rm -rf /var/lib/apt/lists/*

# Create a development user that matches the host UID/GID.
RUN set -eux; \
    if getent group "${USERNAME}" >/dev/null; then \
        groupmod --gid "${USER_GID}" "${USERNAME}"; \
    elif getent group "${USER_GID}" >/dev/null; then \
        existing_group="$(getent group "${USER_GID}" | cut -d: -f1)"; \
        groupmod --new-name "${USERNAME}" "${existing_group}"; \
    else \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    if id -u "${USERNAME}" >/dev/null 2>&1; then \
        usermod --uid "${USER_UID}" --gid "${USER_GID}" --home "/home/${USERNAME}" --move-home --shell /bin/bash "${USERNAME}"; \
    elif getent passwd "${USER_UID}" >/dev/null; then \
        existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
        usermod --login "${USERNAME}" --gid "${USER_GID}" --home "/home/${USERNAME}" --move-home --shell /bin/bash "${existing_user}"; \
    else \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi

# Initialize rosdep (idempotent).
RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init \
 && rosdep update

# Workspace path — src/ is bind-mounted from the host at runtime.
ENV USERNAME=${USERNAME}
ENV ROS_WS=/home/${USERNAME}/ros2_ws
RUN mkdir -p ${ROS_WS}/src \
 && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

# Source ROS and the workspace overlay (if built) for every interactive shell.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
 && echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> /etc/bash.bashrc

# Colored (ros2) prompt.
RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc \
 && echo '' >> /home/${USERNAME}/.bashrc \
 && echo 'if [[ $- == *i* ]]; then' >> /home/${USERNAME}/.bashrc \
 && echo '  PS1="\[\e[1;32m\]\u@\h\[\e[0m\]\[\e[1;33m\](ros2)\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ "' >> /home/${USERNAME}/.bashrc \
 && echo 'fi' >> /home/${USERNAME}/.bashrc \
 && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
