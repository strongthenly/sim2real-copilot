---
name: wsl2-ros2-sim-platform
description: "在 Windows 上用 WSL2 搭建 ROS2(Jazzy)+Gazebo 机器人/无人机仿真平台；覆盖中国网络下的代理/镜像处理、WSL2 网络坑、ROS2 安装与验证。"
version: 1.0.0
author: Hermes Agent
license: MIT
platforms: [windows]
metadata:
  hermes:
    tags: [wsl2, ros2, gazebo, simulation, proxy, china, ubuntu24.04]
---

# WSL2 ROS2 Simulation Platform

## 何时使用
- 在 Windows 上通过 WSL2（Ubuntu 22.04/24.04）搭建 ROS2 + Gazebo 仿真环境，用于机器人/无人机自主探索、导航、SLAM 研究与论文实验。
- 安装 ROS2 (Jazzy) / Gazebo (Harmonic)，且官方仓库被墙/很慢（中国大陆网络）。
- 调试：WSL2 连不上 Windows 宿主上的代理；VSCode remote-WSL 连到错误的发行版；`ros2: command not found`。

## 环境基线（WSL2 + Windows）
- 查版本/发行版：`wsl -l -v`（默认发行版带 `*`）。WSL2 版本 >= 2.7 且含 WSLg 最佳。
- 内存/CPU：改 `~/.wslconfig` 的 `[wsl2]` 段（`memory=`、`processors=`、`swap=`）。**改时先备份**（`Copy-Item ... .bak`），并**保留已有的 `kernel=` 行**（如 NVIDIA JetPack 自定义内核）——在作者行后追加，别覆盖。
- GPU 直通：WSL 里 `nvidia-smi` 能看到显卡即 CUDA 已暴露（Gazebo 3D 渲染 / CUDA 可用）。
- WSLg 已含 GUI 支持（RViz、gz sim 窗口能渲染）。

## 核心坑：WSL2 网络 vs Windows 代理（Clash / VPN）
- 默认 **NAT 模式**：WSL 在独立 `172.25.x/16` vEthernet 网段，**够不到监听 `127.0.0.1` 的 Windows 代理**。症状：`502 Bad Gateway [IP: 127.0.0.1 <port>]`；或 WSL 能上 github.com 但连不上需要走代理的国外源。
- **修复**：在 `~/.wslconfig` 的 `[wsl2]` 下加 `networkingMode=mirrored`，然后 `wsl --shutdown` + 重启（镜像模式即 WSL 共享 Windows 网络栈，`127.0.0.1:<proxy>` 在 WSL 内可达）。改完必须彻底 `wsl --shutdown` 才生效。
- 让 apt 走代理：`/etc/apt/apt.conf.d/proxy` 写 `Acquire::http::Proxy "http://127.0.0.1:7890";` + `Acquire::https::Proxy ...`。下载过程中 Clash 必须保持运行。
- **`502 Bad Gateway` 从代理返回 = 上游节点问题，不是网络/apt**。判断：下载速度应到几 MB/s；若只有 ~17 kB/s 且伴随成片 502，节点是垃圾 → **切换更快更稳的节点**（看实际下载速度，不是看 ping），并加大重试 `-o Acquire::Retries=10`。小元数据能下、大包 502 = 节点对大量下载不稳。

## ROS2 安装（Ubuntu 24.04，中国网络）
- **国内没有可用的 ROS2 apt 镜像**（清华/阿里/USTC/华为/腾讯的 `ros2/ubuntu` 全 404）。官方 `packages.ros.org`（OSL/OSU 机房）被墙/极慢 → 必须走代理，或换好节点。
- **`dists/` 子目录用的是 Ubuntu 代号，不是 ROS 发行为名**：Ubuntu 24.04 的正确路径是 `.../dists/noble/...`（**不是** `jazzy`）。用错会全程 404。
- 加 key + 源（key 在 GitHub raw，WSL 直连常不通，带 `-x <proxy>`）：
  ```
  sudo curl -x http://127.0.0.1:7890 -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
  ```
- 安装：`sudo apt install -y ros-jazzy-desktop -o Acquire::Retries=10`。**让它跑完，别 Ctrl+C**（一断会有半装状态）。

## Gazebo 安装
- **OSRF 官方源**（`packages.osrfoundation.org`）经代理比 packages.ros.org 稳（200、快、少 502）。
- GPG key 是**根路径**：`https://packages.osrfoundation.org/gazebo.gpg`（不是子路径）。源：`deb [arch=... signed-by=...] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main`，装 `gz-harmonic`。

## PX4 真飞控接入：走 MAVROS（MAVLink），不要走 uXRCE/Fast-RTPS
- **现代 PX4（v1.15+ / main）只走 uXRCE-DDS**（`rtps` 命令已移除，`px4_ros_com`/Fast-RTPS 弃用）。
- **在 ROS2 Jazzy 上接 uXRCE 是死结**：micro-ROS-Agent(main) 要 Fast DDS v3，但 Jazzy 只有 fastrtps2，两者**不可共存**（`eProsima_atomic`/`fastrtps` target 冲突，`fastrtps-shared-targets.cmake:42 Some (but not all) targets already defined`）。混装 ros-rolling-fastdds + jazzy 也不行。退到 micro-ROS-Agent v2(fastrtps2) 又协议旧、连不上 PX4 main。
- **治本 = 用 MAVROS（MAVLink）**：MAVLink 不走 fastdds，主流、干净、发论文通行（Gazebo+PX4+MAVROS 是标准姿势）。
- 装 MAVROS：`sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-mavros-extras`（走 packages.ros.org，502 切节点）。
- **MAVROS 还要 GeographicLib geoid 数据**，否则 mavros_node 崩溃：`UAS: GeographicLib exception: File not readable .../geoids/egm96-5.pgm`。修复：`sudo apt install -y geographiclib-tools && sudo geographiclib-get-geoids egm96-5`。
- MAVROS 连 PX4 SITL：launch 文件名是 **`px4.launch`（没有 `.py`）**。`ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`。**PX4 的 mavlink 端口因版本/机型而异**（14540 / 14557 / 14580），连不上（`/mavros/state` 一直 `connected: false`）就看 PX4 启动日志那行 `INFO [mavlink] ... on udp port X remote port Y` 精确配 `fcu_url`。
- 验证接通：`ros2 topic echo /mavros/state --once` → `connected: true`。
- **PX4 SITL 启动坑**：`make px4_sitl gz_x500` 时若已有 gz world 在跑（日志 `gazebo already running world: xxx`），PX4 **复用该 gz 不弹新窗口**、还把模型塞进去 → 先 `pkill -f "gz sim"` 清干净，再启动，PX4 才会弹自己的 Gazebo 窗口显示 X500。

## WSL 发行版迁移到 E 盘（省 C 盘空间）
- 磁盘在 `C:\Users\<user>\AppData\Local\wsl\{GUID}\ext4.vhdx`；`wsl -l -v` 看发行版。
- 迁移三步（**先导出确认完整，再注销，import 后验证**）：
  1. `wsl --shutdown`（务必先关所有发行版）
  2. `wsl --export <distro> E:\wsl\<name>.tar`（大文件，用 background+notify 跑，别前台长阻塞）
  3. 确认 tar 大小合理（非 0）→ `wsl --unregister <distro>`（释放 C 盘 vhdx）→ `wsl --import <distro> E:\wsl\<name> E:\wsl\<name>.tar`
- import 后验证 `wsl -d <distro> -- ls /home/<user>` 数据在 + 默认用户通常保留。
- `bsdtar: ... cannot archive sockets` 是**无害警告**（PX4 临时 socket，运行时重建），不影响导出完整性。
- tar 导出是**回退备份**：导入 + 验证 OK 前别删；确认稳定后删 tar 释放空间。

## 验证与常见坑
- `ros2: command not found` = 没 source。`source /opt/ros/jazzy/setup.bash`（可加入 `~/.bashrc` 自动生效）。
- `ros2 --version` 在某些 build 报 `unrecognized arguments: --version` —— 不代表坏了，改用 `echo $ROS_DISTRO` + `ros2 pkg list | wc -l` 确认。
- 装没装完整：`dpkg -l | grep ros-jazzy-desktop` 应 `ii`；`/opt/ros/jazzy/bin/` 里应有 `ros2`。若 `bin/` 为空或 dpkg 为空 = 之前被中断，没装完。
- VSCode remote-WSL 连的是**默认发行版**。若你有 26.04 默认 + 24.04 目标（装 ROS2 的那个），要么 `wsl --set-default <target>`，要么在 WSL 里 `code .` 或 F1 明确选目标发行版。

## PowerShell ↔ WSL 引号坑
- PowerShell 里 `curl` 是 `Invoke-WebRequest` 的别名 → 用 `curl.exe` 或 `Invoke-WebRequest`。
- 多行/带管道和 `$` 的 bash 命令经 `wsl -d <distro> -- bash -lc '...'` 传会因引号/`$` 展开报错。**把脚本写到 WSL 能读的路径**（`C:\Users\<user>\x.sh` = `/mnt/c/Users/<user>/x.sh`），再 `wsl -d <distro> -- bash /mnt/c/.../x.sh`。

## 支持文件
- `references/china-proxy-ros2-troubleshooting.md` — 中国网络下诊断 502/镜像/dists 代号的详细判读与决策流。
- `scripts/check_ros2_gazebo.sh` — 可复跑的环境自检脚本（ROS2/Gazebo/磁盘/内存/GUI）。
