---
name: ros2-gazebo-sim-setup
description: "在 Windows+WSL2 上搭建 ROS2+Gazebo 无人机仿真环境（自主探索/导航算法研究）。覆盖系统选型、WSL2 镜像模式、国内网络经 Clash 代理、ROS2/Gazebo apt 源坑、VSCode 连接发行版、source 环境这些实操细节。"
version: 1.0.0
author: Hermes Agent
license: MIT
platforms: [windows, linux, macos]
metadata:
  shebang: "source /opt/ros/jazzy/setup.bash"
  tags: [ros2, gazebo, wsl, drone, simulation, autonomous-exploration, china-network]
  related_skills: [px4-flight-controller-tuning, uav-ros-mapping-lifecycle]
---

# ROS2 + Gazebo 无人机仿真环境搭建

## 触发条件
- 用户要在 Windows (WSL2) / Linux 上装 ROS2 + Gazebo 做无人机或机器人仿真（自主探索、导航、建图算法研究）
- 目标发论文（自主探索类），用最前沿 ROS2 开源库做 baseline
- 大陆网络环境（访问 `packages.ros.org` 等国外源受阻），需要走 Clash 类本地代理

## 系统/版本选型（先问，别默认）
做**自主探索论文、纯仿真为主** → 用最新栈：**Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic**（Jazzy 官方配 24.04，Harmonic 也支持 24.04）。
- 若算法必须下放到**现有实机**（如 Jetson + ROS1 Noetic）→ 保持同栈（ROS1），否则两套栈迁移工作量巨大。**这一点必须先跟用户确认**，不要一股脑推 ROS2。

## 安装步骤（顺序）
1. **WSL2 发行版**：`wsl --install -d Ubuntu-24.04 --name <name>`。别用默认 26.04（太新，ROS/Gazebo 生态适配不稳）。
2. **WSL2 镜像网络模式**（关键，见下）：`~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored`，然后 `wsl --shutdown`。
3. **apt 走 Clash 代理**：写 `/etc/apt/apt.conf.d/proxy`（`Acquire::http/::https::Proxy "http://127.0.0.1:7890"`）。
4. **加 ROS2 apt 源** + **Gazebo 源**（见 Pitfalls 里的关键坑）。
5. `sudo apt update` 验证两源都通。
6. 装 `ros-jazzy-desktop`（量大，走代理需耐心，`-o Acquire::Retries=8` 容错全通）。
7. **VSCode 连接**：装 WSL 扩展，`wsl --set-default <你的ROS发行版>` 保证连对，或 `WSL: Connect to WSL` 选对。

## 关键坑（每条都踩过，务必绕开）

### WSL2 连不上 Windows 上的 Clash 代理
**症状**：WSL 里 `curl` 到 `packages.ros.org` 超时/000，而 Windows 宿主能连。
**根因**：WSL2 默认 **NAT 模式**，流量走独立虚拟网卡 + NAT，**不经过 Windows 上仅监听 `127.0.0.1` 的 Clash 代理**（Clash 默认只 bind 127.0.0.1，WSL 的网关 IP 连不到它）。
**修复**：`~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored`，`wsl --shutdown` 重启。镜像模式让 WSL 共享 Windows 网络栈，WSL 内 `127.0.0.1:7890` 即 Windows 的 Clash 端口。
**验证**：WSL 内 `bash -lc 'bash /dev/tcp... '` 或 `curl -x http://127.0.0.1:7890` 测 `packages.ros.org` 返回非 000。
**验证特征**：`ip route` 不再是独立 `172.25.x` 网段（镜像后网关变 `172.27.x` / 共享 Windows 地址）。

### ROS2 apt 源：`dists/` 用 Ubuntu 代号，不是 ROS 发行版名
**症状**：`http://packages.ros.org/ros2/ubuntu/dists/jazzy/Release` 返回 **404**（明明路径该对）。
**根因**：ROS2 仓库的 `dists/` 子目录是 **Ubuntu codename**（24.04=`noble`），**不是** ROS 发行版名（`jazzy`）。正确：`dists/noble/Release`。
**正确源行**：`deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main`（`noble` 替换你的 Ubuntu 代号）。

### 大陆无 ROS2 apt 镜像 + 官方源不稳
- **国内镜像（清华/阿里/USTC/华为/腾讯）的 `ros2/ubuntu` 全部 404** —— 大陆基本没有 ROS2 二进制镜像。
- `packages.ros.org` 托管在 OSL 机房（`ftp.osuosl.org`），大陆直连不通；走 Clash 时**小文件（元数据）能下，大包（deb）会 502 Bad Gateway** —— 这是 **Clash 节点到 OSL 机房拉不动大文件**，不是网络/ROS 问题。
- **绕开方案**：Gazebo 本体走 **OSRF 源 `packages.osrfoundation.org/gazebo/ubuntu-stable`**（这个走代理 200，且与 packages.ros.org 无关）；ROS2 主包用 `Acquire::Retries` 容错反复拉。
- `502 Bad Gateway [IP: 127.0.0.1 7890]` = Clash 节点问题。**重启 Clash（切健康节点）通常能救**；屡试仍 502 则果断换路（如 RoboStack/conda 走清华镜像）。

### Gazebo 源 & key
- 源：`https://packages.osrfoundation.org/gazebo/ubuntu-stable`（`dists/noble/Release` 走代理 200）。
- key：`https://packages.osrfoundation.org/gazebo.gpg`（**根目录**，不是 `gazebo/repos/gazebo.gpg` 等子路径，那些 000/404）。
- 装：`sudo apt install -y gz-harmonic`。ROS2-Gazebo 桥 `ros-jazzy-ros-gz` 在 packages.ros.org（可能 502，装 Gazebo 本体后再按需处理）。

### `ros2: command not found`
**两个原因，先分清**：
1. **没 source 环境** —— 每个新终端都要 `source /opt/ros/jazzy/setup.bash`；一劳永逸写进 `~/.bashrc`：`echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc`。
2. **desktop 根本没装完**（此前只装上 `ament-cmake` 测试小子集）—— source 后仍 `not found` 时查 `ls /opt/ros/jazzy/bin` 有没有 `ros2`、`dpkg -l | grep ros-jazzy-desktop` 是否 `ii`。没有 = 重跑 `sudo apt install -y ros-jazzy-desktop -o Acquire::Retries=8`，**别中断**（之前 Ctrl+C 会只装一半，`/opt/ros/jazzy` 只有 `setup.bash`、无 `bin/`）。

### VSCode 连接错了发行版
多个 WSL 发行版时，VSCode WSL 扩展默认连**默认发行版**。若用户目标 ROS 环境在非默认发行版，会打开空环境。修复：`wsl --set-default <目标发行版>`，或 `WSL: Connect to WSL` 手动选，或在该发行版终端里 `code .`。
**诊断**：`code --list-extensions` 看有无 `remote-wsl`；`update.code.visualstudio.com`（vscode-server 源）网络可达性。

## 验证
⚠️ **`ros2 --version` 会报 `unrecognized arguments: --version`**（这版 CLI 不认该写法），**别用它判版本**；改用下面方式。
```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO && ros2 pkg list | wc -l    # 出 jazzy + 几百个包 = ROS2 装好
apt-cache search ros-jazzy | head -3         # 出 ros-jazzy-* 包 = ROS 源通
dpkg -l | grep ros-jazzy-desktop              # 出 ii 才是真的装了 desktop
```

### Gazebo 验证
```bash
which gz && dpkg -l | grep gz-harmonic       # 出 gz-harmonic 1.0.0 = 已装
timeout 35 gz sim -s -r empty.sdf 2>&1 | tail   # exit=124(=timeout到点杀) = 服务端常驻启动成功；立刻非124+报错 = 失败
```
`gz sim -s`（headless 服务端，只起 `-s` 无 `-r` 也可加 `-r` 自动跑）是常驻进程不自动退出；被 `timeout` 杀(124)恰恰证明"活着没崩"= 启动成功。这也是以后批跑探索实验要用的 headless 模式。

## 详细诊断记录
见 `references/network-proxy-and-sources.md`（本会话完整排障路径：NAT→镜像模式、DNS 未污染、OSL 被限、dists=noble、502=Clash 节点、国内无镜像等，含具体 curl 探测命令）。

### 搭建收尾（git 工作站推送 + Windows 侧验证 + 包骨架）
见 `references/git-push-and-validation.md`——私有仓库 **HTTPS+PAT 认证**（GitHub 弃用登录密码、`credential.helper store` 记住、远程非空 `--force`/pull-rebase）；Windows 无 ROS 环境用 **`py_compile`+XML 做静态验证**、真 `colcon build` 需在 WSL + `sudo apt install python3-colcon-common-extensions`；**ament_python 最小功能包骨架**要点（package.xml/setup.py/setup.cfg/resource、entry_points console_scripts、空 `src/` 用 `.gitkeep`）。

## 常见排查命令
- WSL 内探测代理：`curl -x http://127.0.0.1:7890 -sIL -o /dev/null -w "%{http_code} %{time_total}s" --max-time 15 <url>`
- 看网络模式：`ip route`（独立 `172.25.x` = NAT；变 `172.27.x`/共享 = 镜像）
- 看默认发行版：`wsl -l -v`（带 `*` 的是默认）
