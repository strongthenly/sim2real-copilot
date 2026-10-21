#!/bin/bash
# 环境自检：ROS2 + Gazebo 是否装好、磁盘/内存/GUI 基线。
# 用法: wsl -d <distro> -- bash /mnt/c/Users/<user>/check_ros2_gazebo.sh
set -u
echo "===== ROS2 ====="
echo "ROS_DISTRO (source 前): ${ROS_DISTRO:-<空>}"
ls /opt/ros/*/setup.bash 2>/dev/null
echo "/opt/ros/*/bin (前10):"; ls /opt/ros/*/bin 2>/dev/null | head -10
echo "ros2 pkg list 数量:"; bash -c 'source /opt/ros/*/setup.bash 2>/dev/null; ros2 pkg list 2>/dev/null | wc -l'
echo "ros-jazzy-desktop dpkg:"; dpkg -l 2>/dev/null | grep -E "^ii  ros-jazzy-desktop" || echo "  (未装)"
echo ""
echo "===== Gazebo ====="
which gz 2>&1
gz --version 2>&1 | head -2
echo "gz-harmonic/gazebo dpkg:"; dpkg -l 2>/dev/null | grep -iE "gz-harmonic|gz-sim" | head -5
echo ""
echo "===== 基线 ====="
echo "用户: $(whoami)"
echo "磁盘: $(df -h / 2>/dev/null | tail -1)"
echo "内存: $(free -h 2>/dev/null | grep Mem)"
