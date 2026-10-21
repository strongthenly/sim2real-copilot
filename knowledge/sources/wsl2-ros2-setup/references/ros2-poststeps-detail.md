# ROS2 装好源后的后续搭建（Gazebo / 工作空间 / 感知桥 / ament 包）

本文件是 `wsl2-ros2-setup` 的"装好源之后"的补充细节，均为实测。

## 1. Gazebo Harmonic 安装（OSRF 源，绕开 502 的 packages.ros.org）

```bash
# GPG key 在根目录（不要用子目录！）
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg
# 源（dists 用 Ubuntu 代号 noble）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | sudo tee /etc/apt/sources.list.d/gazebo.list
sudo apt update
sudo apt install -y gz-harmonic -o Acquire::Retries=10
# 验证
gz --version     # 打印 Gazebo Tools CLI; gz sim 是 8.x (Harmonic)
```

验证服务端能启动（headless，批量实验用）：
```bash
timeout 35 gz sim -s -r empty.sdf   # exit=124 (timeout kill) = 进程常驻成功启动；否则才是有错
```

## 2. ros_gz_bridge（感知桥）
```bash
sudo apt install -y ros-jazzy-ros-gz -o Acquire::Retries=8
# 之后 /opt/ros/jazzy/lib/ros_gz_bridge/ 有 parameter_bridge / static_bridge / bridge_node
```
- 用法：launch 里 `Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': bridge_yaml}])`。
- `parameter_bridge` 从 gz 话题收到 ROS 话题，方向 `GZ_TO_ROS`；反向 `ROS_TO_GZ`。

## 3. colcon 工作空间 + ament_python 包骨架
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws && git init -b main
# 包放 ~/ros2_ws/src/<pkg>/ 下，含:
#   package.xml（<build_type>ament_python</build_type>）
#   setup.py（data_files 装 package.xml + resource + launch/* + worlds/*）
#   setup.cfg
#   resource/<pkg>         （空文件）
#   <pkg>/__init__.py
#   <pkg>/<node>.py        （rclpy 节点，entry_points console_scripts）
#   launch/、worlds/       （aumtp）
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash && ros2 pkg list | grep <pkg>
```
- **`ros2 --version` 报 unrecognized 是正常的**，用 `echo $ROS_DISTRO`（应显示 jazzy）+ `ros2 pkg list | wc -l`（几百个==装好）确认。
- 新终端要 `source /opt/ros/jazzy/setup.bash`，建议写进 `~/.bashrc`。

## 4. 一个最小四旋翼 + 3D LiDAR world（SDF）要点
- world 里加载系统：`gz-sim-physics-system`、`gz-sim-user-commands-system`、`gz-sim-sensors-system`、`gz-sim-scene-broadcaster-system`。
- 四旋翼：`<model>` 一个 `base_link`（几何 + inertial），先 `<static>true</static>` 悬停以稳定验证感知；飞控/运动（MulticopterControl）后续再加。
- 3D LiDAR：`<sensor type="gpu_lidar">` + `<lidar>` 配 `<horizontal>` + `<vertical>`（vertical samples>1 即 3D 多线），`<topic>lidar</topic>`，配 `gz-sim-sensors-system`。
- 用一个 `static` 的 `gpu_lidar` + 简单障碍墙 world 即可先跑通 `/scan` → ROS2，避免一上来就卡在飞行动力学。

## 5. VSCode 连 WSL —— "连错发行版"坑
- VSCode WSL 扩展默认连 **默认发行版**。ROS2 若装在非默认 distro，会打开空环境、像"连不上"。
- 修复：`wsl --set-default <distro>`（PowerShell）；或 `F1 → WSL: Connect to WSL` 选目标；或在目标 distro 里 `code .`。
- 判连对没：VSCode 左下角应显示 `WSL: <distro>`，新终端是 bash 且能 `source /opt/ros/jazzy/setup.bash`。

## 6. GitHub 推送私有仓库
- 私有仓库推送用 **HTTPS + PAT**；`Password:` 处填 PAT（`ghp_...`），**不是登录密码**；GitHub 报 `Password authentication is not supported` 即填错了。
- 免输：`git config --global credential.helper store`，之后 push 一次输入，缓存到 `~/.git-credentials`。
- 远程非空（建库勾了自动 README）：报 `fetch first`，确认远程只是占位后 `git push -u origin main --force`。
- Git 身份未配置时（WSL 新的），`git config --global user.name` / `user.email`。
