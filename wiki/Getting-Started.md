# Getting Started

This guide covers everything you need to run the simulation from scratch on **Ubuntu 24.04** with **ROS 2 Jazzy** and **Gazebo Harmonic**.

---

## Prerequisites

- Ubuntu 24.04 LTS (Noble Numbat)
- A working internet connection
- `sudo` access

---

## Step 1: Install ROS 2 Jazzy

Follow the official instructions, or use the commands below:

```bash
# Set up locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop
```

Source ROS 2 in every terminal (add to `~/.bashrc` to make permanent):

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 2: Install Gazebo Harmonic and ros_gz

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-cmake-vendor \
  ros-jazzy-gz-common-vendor \
  ros-jazzy-gz-plugin-vendor \
  ros-jazzy-gz-sim-vendor
```

Set the Gazebo version environment variable (required for this project):

```bash
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 3: Install Project Dependencies

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-nav2-bringup \
  ros-jazzy-sdformat-urdf \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-laser-geometry \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## Step 4: Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/lauzmy/MEPA2002SAR.git
```

---

## Step 5: Install rosdep dependencies

```bash
cd ~/ros2_ws
sudo rosdep init   # Skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 6: Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

After building, source the workspace overlay:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 7: Launch the Simulation

```bash
ros2 launch ros_gz_bringup test_xacro.launch.py
```

This will:
1. Convert the xacro world and robot model to SDF
2. Start Gazebo Harmonic with the 10×10 m arena
3. Spawn the mecanum robot with all sensors active
4. Start `robot_state_publisher`, `ros_gz_bridge`, `slam_toolbox`, `robot_localization` EKF
5. Open RViz2 with a pre-configured layout

### Optional launch arguments

```bash
# Disable RViz (headless simulation)
ros2 launch ros_gz_bringup test_xacro.launch.py rviz:=false

# Disable SLAM (saves CPU)
ros2 launch ros_gz_bringup test_xacro.launch.py slam:=false

# Both disabled
ros2 launch ros_gz_bringup test_xacro.launch.py rviz:=false slam:=false
```

---

## Step 8: Drive the Robot

Open a new terminal and send velocity commands:

```bash
source ~/ros2_ws/install/setup.bash

# Drive forward
ros2 topic pub --once /test_robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Stop
ros2 topic pub --once /test_robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Or use keyboard teleoperation:

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# Run teleop and remap to the correct topic
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/test_robot/cmd_vel
```

---

## Optional: Run with Docker

If you prefer to use Docker instead of a local install, a `Dockerfile` is provided at the root of the repository.

```bash
# Build the image
docker build -t mepa2002sar .

# Run with display forwarding (for Gazebo + RViz)
xhost +local:docker
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/src:/home/ubuntu/ros2_ws/src \
  mepa2002sar
```

Inside the container:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch ros_gz_bringup test_xacro.launch.py
```

---

## Troubleshooting

### Gazebo doesn't find the robot model

```bash
# Check that GZ_SIM_RESOURCE_PATH includes the models directory
echo $GZ_SIM_RESOURCE_PATH
# Should contain: ~/ros2_ws/install/ros_gz_description/share/ros_gz_description/models
```

### `xacro` command not found

```bash
sudo apt install -y ros-jazzy-xacro
```

### `gz` command not found

```bash
sudo apt install -y ros-jazzy-gz-sim-vendor
# Then check:
which gz
```

### TF tree has gaps (no `odom → base_footprint`)

Make sure the EKF node and `ros_gz_bridge` are both running. Check:

```bash
ros2 run tf2_tools view_frames
```

### SLAM map is not building

Verify that `/test_robot/scan` is publishing:

```bash
ros2 topic echo /test_robot/scan --once
```

If there is no output, the `ros_gz_bridge` may not be running or the Gazebo sensors system plugin is missing from the world SDF.
