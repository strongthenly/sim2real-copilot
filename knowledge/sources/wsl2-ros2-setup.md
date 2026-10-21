---
name: wsl2-ros2-setup
description: "在 Windows WSL2 上搭建 ROS2/Gazebo 无人机仿真环境——尤其国内网络 + Clash 代理场景。核心坑：WSL2 默认 NAT 模式够不到 Windows 上只监听 127.0.0.1 的 Clash 端口（用 networkingMode=mirrored 修复）；ROS2 apt 仓库 dists/ 子目录用 Ubuntu 代号(24.04=noble)而非 ROS 版本名(jazzy)；国内无 ROS2 apt 镜像，官方 packages.ros.org 直连不通(OSU 机房受限)须经代理。也涵盖在 Linux/Ubuntu 装 Hermes。"
version: 1.0.0
author: Hermes Agent
license: MIT
platforms: [linux, macos, windows]
metadata:
  hermes:
    tags: [wsl2, ros2, gazebo, sim, proxy, china, networking, uv, hermes-install]
    related_skills: [windows-ssh-automation, powershell-automation, px4-ulog-analysis]
---

# WSL2 + ROS2 无人机仿真环境搭建（含国内网络/Clash 代理）

## 触发条件
- 在 Windows WSL2 里搭 ROS2/Gazebo 无人机仿真平台，且**国内网络**（连外网慢/被限，需 Clash 等本地代理）
- 遇到 WSL2 里访问 Windows 宿主上的代理/OAuth/服务连不上
- 装 ROS2 时 `packages.ros.org` 连不上、或 `dists/jazzy/Release` 返回 404 无从下手
- 想在 Linux（含 Ubuntu 20.04）装 Hermes Agent

## 三大关键知识点（本技能的灵魂，实测验证过）

### 1. WSL2 够不到 Windows 宿主上的 Clash 代理 —— 用 mirror 模式
- **症状**：WSL 里 `curl -x http://<网关>:7890 https://...` 超时(000)，但 `127.0.0.1:7890` 在 Windows 侧是 OPEN。
- **根因**：WSL 默认 **NAT 模式**，流量走独立虚拟网卡(如 `172.25.x.x / 172.27.x.x`)，WSL 里的 `127.0.0.1` 是**它自己**的回环，不是 Windows 的。而 Clash 默认只监听 Windows 本机 `127.0.0.1:7890`，WSL 用网关 IP 够不到。
- ⚠️ **先探测你的 Clash 实际端口，别写死 7890**（本会话教训：写 7890 一直连不上，实际是 **7899**）。不同客户端默认不同：Clash for Windows=7890、Clash Verge/Mihomo=7897、也可能是 7891/7899 等自定义。Windows 侧用 `Test-NetConnection 127.0.0.1 -Port <p>` 逐个探测 7890/7891/7892/7897/7899/9090，找 `True` 的那个；**记下该端口**（本用户是 7899），后续所有 `curl --proxy`、`git -c http.proxy`、apt 代理都用它，别假设 7890。
- **修复**：在 Windows 的 `~/.wslconfig` 的 `[wsl2]` 段加一行，然后**必须 `wsl --shutdown` 重启**才生效：
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  镜像模式让 WSL **共享 Windows 网络栈**，此时 WSL 里的 `127.0.0.1` 就是 Windows 的 → Clash `127.0.0.1:7890` 可达。
- **验证镜像生效**：`ip route` 的 default 网关不再是固定的 `172.25.x`（可能变 eth2 / 不同网段），且 `127.0.0.1:7890` 在 WSL 里 TCP OPEN。
- 若 `.wslconfig` 已存在（如用户为 NVIDIA JetPack 交叉编译设了 `kernel=` 自定义内核），**必须保留那行**，只追加 `networkingMode` 和内存项，并先备份 `.wslconfig.bak`。
- ⚠️ 不要用单反斜杠 Windows 路径写 `.wslconfig`（会触发"转义字符无效"警告）；用正斜杠 `C:/ProgramData/...` 或双反斜杠。

### 2. ROS2 apt 仓库 dists/ 用 Ubuntu 代号，不是 ROS 版本名
- **坑**：`http://packages.ros.org/ros2/ubuntu/dists/jazzy/Release` 返回 404 —— 因为 `dists/` 下面的子目录是 **Ubuntu 代号**，不是 ROS2 发行版名。
- **正确**：Ubuntu 24.04 → `.../dists/noble/Release`；22.04 → `jammy`；20.04 → `focal`。
- apt 源行 `$(. /etc/os-release && echo $UBUNTU_CODENAME)` 就是取这个代号。

### 3. 国内直连 ROS2 官方源不通，也没有国内 ROS2 apt 镜像
- **实测**：清华 / 阿里云 / 中科大 / 华为云 / 腾讯云 的 `/ros2/ubuntu/dists/noble/Release` **全部 404** —— 国内主流镜像**没有** ROS2 Debian 仓库。
- **官方源** `packages.ros.org` 解析到 `ftp.osuosl.org`（美国俄勒冈州立大学 OSL 机房）。DNS 没被污染（系统解析与阿里 DoH 结果一致），但**国内直连这些 IP 的路由被限/超时**。
- **解法**：走 Clash 代理。镜像模式后，apt 配代理指向 `127.0.0.1:7890`。
- **判节点质量（装大包的关键判据）**：看 `apt` 装包时的 `Fetched X kB in Ys (Z kB/s)` 那行。**Z ≥ ~500 kB/s = 节点健康**，800MB 的 `ros-jazzy-desktop` 能顺利下完；**Z 掉到十几 kB/s（如 17 kB/s）或反复 `502 Bad Gateway [IP: 127.0.0.1 7890]` = 节点到 OSL 机房拉不动大文件**。此时**重启 Clash / 换一个下载速度快、不抖的节点**（选"专用/中转"类，别只看 ping 延迟），别死磕 apt。换好后**先装小包试水**（`ros-jazzy-ament-cmake` 通了再上大的），全程 `-o Acquire::Retries=8` 容错零星 502。

## 连通性诊断探针（先测再装，别盲装）

在 WSL 里跑（PowerShell 宿主侧用 `wsl.exe -d <distro> -- bash /mnt/c/~/probe.sh`，**避免多层引号被 PowerShell 破坏**——多行复杂命令建议写成脚本文件，用 `wsl -d <distro> -- bash /mnt/c/路径/脚本.sh` 执行，不要用 `bash -lc "多行"`）：

1. **确认代理链路是否正常**：经 Clash 测 google/bing/example 应 **200**；若全 200 说明代理健康，问题在目标域名规则。
2. **看目标域名 404 是不是路径问题**：`curl -sL` 读 **响应体**。若 body 是 `<address>Apache Server at packages.ros.org Port 80</address>` → **真实服务器到了，只是 URL 错**（多半是 dists 用了 jazzy 而非 noble），不是网络问题。
3. **区分 HEAD 与 GET**：`-sIL`（HEAD）某些服务器行为不同，用 `-sL -w "%{http_code} %{size_download}B"` 的 GET 更准。
4. **测真实包索引进而不只是 Release**：`.../dists/noble/main/binary-amd64/Packages.gz` 若 200 且几 MB，说明仓库真可用。

## apt 走代理配置（镜像模式后）

`/etc/apt/apt.conf.d/proxy`：
```bash
sudo tee /etc/apt/apt.conf.d/proxy > /dev/null <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
EOF
```
Ubuntu 系统源顺手换清华镜像提速（`archive.ubuntu.com`/`security.ubuntu.com` → `mirrors.tuna.tsinghua.edu.cn/ubuntu`，注意 22.10+ 用 deb822 的 `/etc/apt/sources.list.d/ubuntu.sources`，要 sed `URIs:` 那行，两种格式都处理，先备份）。
ROS 源：
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
```
（`ros.key` 在 GitHub raw 上，WSL 直连一般可达；若不行经 `-x` 代理下。）

## 在 Linux/Ubuntu 装 Hermes Agent
- **Ubuntu 20.04 可装**：`curl -fsSL https://hermes-agent.nousresearch.com/install.sh | bash`。安装脚本明确分支 `ubuntu|debian`。
- Hermes 要求 **Python ≥3.11,<3.14**；20.04 默认 3.8，但脚本用 **uv 自动装 Python 3.11** 绕开。Node 需要 22.22+/24.11+/26+，脚本自动装 Hermes-managed Node。
- 前置：联网 + `sudo`（仅 apt 装构建工具 `build-essential python3-dev libffi-dev`），Hermes 本身不占 root。
- ⚠️ Hermes **桌面版(Electron)** 对 glibc 有要求（20.04 是 2.31 较老）；**CLI/TUI 版绝对没问题**，优先 CLI。

## 装好 ROS2 源后的后续搭建（Gazebo 本体 / 工作空间 / 感知桥 / IDE / 推送）

前几节解决"源 + 网络"，这里是**装好源之后**的标准步骤（本技能实测通过）。

### 4. Gazebo Harmonic 走 OSRF 源（绕开 502 的 packages.ros.org）
- `gz-harmonic` 的 deb 在 `packages.osrfoundation.org/gazebo/ubuntu-stable`（dists=**noble**），**不是** packages.ros.org → 即使 packages.ros.org 反复 502，Gazebo 也能正常装。
- GPG key 在**根目录** `https://packages.osrfoundation.org/gazebo.gpg`（别用子目录如 `/gazebo/gazebo.gpg`，会 404）。
- 完整源配置脚本见 `references/ros2-poststeps-detail.md`。

### 5. colcon 工作空间 + ros_gz_bridge（感知桥）
- ROS2 用 **colcon** 不是 catkin：`~/ros2_ws/src` + `colcon build --symlink-install`。
- ⚠️ **`ros2 --version` 会报 unrecognized**——别当错误；用 `echo $ROS_DISTRO` + `ros2 pkg list | wc -l` 确认装好。
- **每个新终端必须 `source /opt/ros/jazzy/setup.bash`**（写进 `~/.bashrc` 一劳永逸；否则 `ros2: command not found`）。
- `sudo apt install ros-jazzy-ros-gz` 装感知桥，之后 `/opt/ros/jazzy/lib/ros_gz_bridge/` 有 `parameter_bridge / static_bridge / bridge_node`。
- 主题映射模板（gz `/lidar` → ROS `/scan`）见 `templates/bridge.yaml`；ament_python 包骨架要点见 `references/ros2-poststeps-detail.md`。

### 5.5 感知桥调试 + gpu_lidar 全 `.inf` 的几何坑、libEGL 警告（实测）
**先分清"gz 传感器没发"还是"bridge 没接上"**：
- 看 **gz 侧**：`gz topic -l | grep lidar`（出现 `/lidar`、`/lidar/points` = 传感器在工作，问题在 bridge）。查类型用 `gz topic -i -t /lidar`（`-l/-i/-e` 是子命令，`-t` 只给名字，别单独 `-t`）。
- 看 **ROS 侧**：`ros2 topic list` 里有没有 `/scan`（有时因时序 `grep scan` 一时见不到、隔几秒就出现，别慌）。
- 若 gz 有、launch 日志也打 `Creating GZ->ROS Bridge: [/lidar -> /scan]`，但 `/scan` 无数据 → 多半 `bridge.yaml` 的 `gz_type_name` 没对上。关键字段：`gz_topic_name`、`gz_type_name`（如 `gz.msgs.LaserScan`）、`ros_type_name`（`sensor_msgs/msg/LaserScan`）、`direction: GZ_TO_ROS`。3D 点云桥 `/lidar/points`（`gz.msgs.PointCloudPacked` → `sensor_msgs/msg/PointCloud2`）。

**gpu_lidar ranges 全 `.inf`**：`.inf` = 光线射出没碰到东西；`0` = 传感器未就绪——**别混淆**。`frame_id/angle_min/max/range_max/beam数` 都正常但 ranges 全 `.inf` → **纯几何问题，不是 GPU/桥接**。
- 判据：LiDAR 在高度 `z`、垂直半角 `vmax`，水平距离 `d` 处垂直覆盖 `[z-d·tan(vmax), z+d·tan(vmax)]`；障碍物（墙高 `h`、中心 `zc`，即 z∈[zc-h/2,zc+h/2]）落在此区间外 → 一根都扫不到 → 全 `.inf`。
- 实例：四旋翼 z=2m、vmax=±0.3rad、墙 1m 高（z 0~1）→ 覆盖 `[1.07,2.93]` 与墙不相交 → 全 `.inf`。**修法：加高墙（4m）/ 增大垂直视野 / 降飞行高度**。
- 改 `.sdf` 后**必须 `colcon build`**（`--symlink-install` 不自动同步 data_files 的 worlds/launch，需重新 copy 进 install）再重启 launch。

**libEGL `/dev/dri Permission denied` 在 WSL 是非致命警告，别当根因**：Gazebo 在 WSL 走 WSLg(Mesa/D3D12)，试图原生开 `/dev/dri/renderD128|card0` 被拒是**正常**的 `libEGL warning`；**不影响传感器出点云**，只让 GUI 退到软渲染（`gz sim gui` 吃高 CPU）。用户已在 `video`/`render` 组 + `nvidia-smi` 可见即 GPU 直通 OK——**别看到这串 warning 就去折腾 GPU 权限，先确认 `/scan` 有没有数据**。

### 6. VSCode 连 WSL 的"连错环境"坑
- VSCode 的 WSL 扩展**默认连"默认发行版"**。若 ROS2 装在非默认 distro（如默认是 26.04、ROS2 在 24.04 的 `lyuav`），会打开一个空环境，看起来"连不上/连错"。
- 修复：PowerShell 里 `wsl --set-default <distro>`，或 VSCode `F1 → WSL: Connect to WSL` 选目标，或在目标 distro 终端里 `code .`。

### 7. GitHub 推送私有仓库
- 私有仓库 HTTPS push：`Password:` 处填 **PAT**（不是登录密码；GitHub 已废弃密码认证，报 `Password authentication is not supported` 就是填错了）。
- 免输：`git config --global credential.helper store`。
- 远程非空（GitHub 建库时自动初始化 README 等）→ push 报 `fetch first` 被拒：确认远程只是占位后 `git push --force`。

### 8. PX4 SITL 接入（B 路线：真实飞控 + ROS2-Gazebo）
这是把仿真从"感知层"推到"可飞的真飞控"的一步，**最大的新坑是 Gazebo 源冲突**：

- **PX4 的 `Tools/setup/ubuntu.sh` 会自己装 Gazebo 并写 `/etc/apt/sources.list.d/gazebo-stable.list`**（signed-by=`/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg`）。若你**已手动加过** Gazebo 源（`gazebo.list`，signed-by=`gazebo-archive-keyring.gpg`），就会出现**两个 source 指向同一仓库但 Signed-By 不同**：
  ```
  E: Conflicting values set for option Signed-By regarding source http://packages.osrfoundation.org/gazebo/ubuntu-stable/ noble:
     /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg != /usr/share/keyrings/gazebo-archive-keyring.gpg
  E: The list of sources could not be read.
  ```
- **修复**（两份 keyring 内容相同，都是同一个 Gazebo key，删一份即可）：`sudo rm /etc/apt/sources.list.d/gazebo.list` 保留 PX4 的 `gazebo-stable.list`，`sudo apt update`，然后**重跑一次 `bash Tools/setup/ubuntu.sh`**（它上次是被这个冲突打断的）。
- **编译 + 启动 SITL**：`cd ~/PX4-Autopilot && make px4_sitl gz_x500`。首次编译 20~40 分钟；编译完第二段自动启 `gz` X500 并**保持前台**。
- **成功标志**：出现 `pxh>` 提示符、`INFO [mavlink] ... udp port 14580 remote port 14540`、`logger started`、gz 窗口出现 X500。
- **SITL 常见噪音（别当故障）**：`WARN Preflight Fail: No connection to the GCS`（没连 QGC，正常）、`NodeShared::Publish() Error: Interrupted system call`、`ERROR [vehicle_imu] ... timestamp error`（SITL 时序/时间戳抖动）。只要 `pxh>` 在、X500 在、MAVLink 端口正常即健康。
- **ROS2 接 PX4（注意：仓库名是坑，且现代 PX4 走 uXRCE 不是 Fast-RTPS）**：
  - ❌ `PX4/PX4-ROS2-bridge` **不存在**（实测 `Repository not found`）。正确是 **`PX4/px4_ros_com`**（Fast-RTPS bridge）+ **`PX4/px4_msgs`**（消息），两个都要放进**同一个 workspace 的 `src/`**，`colcon build` 才会先建 px4_msgs 再建 bridge（否则报 `Could not find px4_msgsConfig.cmake`）。
  - ⚠️ **现代 PX4（v1.15+/main）只用 `uXRCE-DDS`**，`pxh>` 里 `rtps status` 报 `Invalid command`（Fast-RTPS 已被移除）→ `px4_ros_com`(Fast-RTPS) 也接不上。
  - ⚠️ **uXRCE 的 agent 要 Fast DDS v3，而 Jazzy 只有 Fast RTPS v2（`fastrtps 2.11`）**：`ros-rolling-fastdds` 装进来会跟 jazzy 的 fastrtps 撞 **eProsima target 名冲突**（`Some targets already defined ... eProsima_atomic`），无法共存。退到 `micro-ROS-Agent` v2 用 fastrtps2 则协议旧、连不上 PX4 main，且 v2.0.4 tag 是 **tag 不是 branch**（`git clone -b v2.0.4` 报 not found，要 `git checkout`；clone 后要先 `git fetch --tags`）。
  - **结论（实测）**：**在 Jazzy 上 PX4-SITL+ROS2(uXRCE) 是环境级死结**（fastdds3/fastrtps2 不能共存 + agent 版本匹配不上 PX4 main），不是配置能绕过的。
  - ✅ **换用 MAVROS(MAVLink) 就干净绕过 uXRCE 死结**（本会话实测 `/mavros/state connected: true` 打通）：`sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras` → **先 `sudo geographiclib-get-geoids egm96-5`**（否则 mavros_node 启动即崩 `GeographicLib exception`）→ `ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`。**注意 launch 文件名是 `px4.launch`（无 `.py`）**，用 `.py` 会报 `file 'px4.launch.py' was not found`。MAVROS 是 PX4↔ROS2 的标准 MAVLink 桥，可连 SITL 或真机，是优先路线。

### 9. 无人机自主探索的主流做法：**不用 PX4，用简化四旋翼 + 直接发指令**（关键决策）
调研 GitHub 高引用探索库确认：**顶会级 UAV 探索框架全部用「Gazebo + 简化四旋翼动力学 + 直接发指令」，不接 PX4**，探索算法完全自由（这正是"随意介入算法"的诉求正常解）：
- **HKUST/FUEL**（1471⭐，RA-L/ICRA）—— Gazebo + `uav_simulator` 简化四旋翼 + 深度相机/点云 + frontier，RViz **2D Nav Goal** 触发。
- **STAR-Lab/RACER**（789⭐，TRO 2023 最佳论文）、**STAR-Lab/EPIC**（236⭐）—— 同理，简化四旋翼 + FAST-LIO + 勘探算法，不用 PX4。
- 少数用 PX4 的（如 `princepatel51204-dot/GarudaNEX`）反而最重，且卡在 PX4-ROS2。
- **要点**：它们是 ROS1（Melodic/Noetic）+ `catkin`；但**思想一样适用于 ROS2**——用 `gz-sim` 的**简化四旋翼速度控制**（响应 `/cmd_vel`），bridge `/cmd_vel`(ROS Twist→gz) + `/scan`(gz lidar→ROS)，探索节点直接发 `/cmd_vel` 驱动，`ros_gz_bridge` 已装即可。
- ⚠️ **不要用泛化的 `gz-sim` `MulticopterControl` 插件**（实测：只悬停、**不响应 `/cmd_vel` 水平移动**）。**官方可靠做法是 `gz-sim` 的 `multicopter_velocity_control.sdf`**：**`MulticopterMotorModel`(每个旋翼一个) + `MulticopterVelocityControl`**，速度指令话题是 **`<robotNamespace>/gazebo/command/twist`**（带模型名前缀，非 `/cmd_vel`）。完整参数/xml 见 `references/px4-ros2-and-mainstream.md`。

⚠️ **`MulticopterVelocityControl` 必须先发 `enable` 才接管，否则四旋翼"不动"**（源码确认它订阅 `<robotNamespace>/gazebo/command/enable` 的 `gz.msgs.Boolean`；没 enable 前四旋翼悬停/坠落、不响应 `/cmd_vel`，现象就是"发了指令却不动"，容易误判为模型/物理问题）。**自动化**：在 ROS2 launch 里用 `TimerAction(period≈4.0)` 延迟发一次 enable（等 gz 先订阅）：
```python
from launch.actions import ExecuteProcess, TimerAction
enable = ExecuteProcess(cmd=['gz','topic','-t','/<robotNamespace>/gazebo/command/enable','-m','gz.msgs.Boolean','-p','data: true'])
# LaunchDescription 里加: TimerAction(period=4.0, actions=[enable])
```
> 相关话题都带模型名前缀：指令 `<ns>/gazebo/command/twist`、odom `/model/<name>/odometry`、pose `/model/<name>/pose`。bridge 的 `gz_topic_name` 必须写全 `<ns>/gazebo/command/twist`（不是裸 `/cmd_vel`）。

**结论**：用户要"无人机+随意介入算法" → 走**简化四旋翼（`MulticopterVelocityControl`+`MulticopterMotorModel`）+ 3D LiDAR + frontier 探索节点（可随意改）**，**不要**在 PX4-ROS2 上烧时间（环境级死结见上节 8）。

## 参考脚本
- `references/session-scripts.md` —— 连通性诊断探针脚本 + 源配置脚本，可直接复制。
- `templates/bridge.yaml` —— ros_gz_bridge 主题映射模板（gz `/lidar` → ROS `/scan`）。
- `references/ros2-poststeps-detail.md` —— 装好源后：Gazebo 本体安装、ament_python 包骨架、launch/bridge 细节。
- `references/px4-ros2-and-mainstream.md` —— PX4-ROS2 接入的仓库名坑、Jazzy 上 fastdds3/fastrtps2 冲突证据、主流 UAV 探索做法（FUEL/RACER 简化四旋翼）、ROS2 简化四旋翼 MulticopterControl SDF 结构。
