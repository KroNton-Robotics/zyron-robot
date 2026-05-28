#!/bin/bash
# Install ROS 2 Jazzy packages and system libraries required by zyron-robot.
# Tested on Ubuntu 24.04.
set -e

ROS_DISTRO=jazzy

echo "==> Updating apt cache..."
sudo apt-get update

# ── Build toolchain ────────────────────────────────────────────────────────────
echo "==> Installing build tools..."
sudo apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ament-cmake

# ── Core ROS 2 runtime ────────────────────────────────────────────────────────
echo "==> Installing core ROS 2 packages..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-lifecycle \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-pluginlib

# ── Robot description (zyron_description) ────────────────────────────────────
echo "==> Installing robot description packages..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro

# ── ros2_control stack (zyron_control) ────────────────────────────────────────
# hardware_interface, controller_manager, ros2_control_node, spawner
echo "==> Installing ros2_control packages..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-controller-manager

# ── Localization (robot_localization EKF) ─────────────────────────────────────
echo "==> Installing robot_localization..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-robot-localization

# ── Nav2 lifecycle manager (zyron_firmware launch) ───────────────────────────
echo "==> Installing nav2_lifecycle_manager..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager


# ── System library: LibSerial (zyron_firmware serial driver) ─────────────────
echo "==> Installing libserial-dev..."
sudo apt-get install -y \
    libserial-dev

# ── Serial port access: add user to dialout group ────────────────────────────
echo "==> Adding ${USER} to dialout group (serial port access for /dev/ttyUSB0)..."
sudo usermod -aG dialout "$USER"
