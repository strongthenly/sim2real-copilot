# 国内装 ROS2/Gazebo/PX4（WSL2 + Clash 代理）— 实测命令

## 网络层：WSL2 ↔ Windows Clash
- WSL2 默认 NAT，Windows 上仅监听 127.0.0.1 的 Clash(7890) 从 WSL 够不到。
- 修：`~/.wslconfig`：
  ```ini
  [wsl2]
  kernel=C:/ProgramData/NVIDIA Corporation/SDKs/JetPack_6.2.2_Linux/vmlinux   # 用户自定义内核,用正斜杠
  memory=12GB
  processors=10
  swap=4GB
  networkingMode=mirrored        # 关键:WSL 共享 Windows 网络栈
  ```
  改后 `wsl --shutdown` 重启。镜像模式下 WSL 的 `127.0.0.1:7890` == Windows 的 Clash。
- 探测：`CreatePSCLSite` 用 `Test-NetConnection 127.0.0.1 -Port 7890`；WSL 内 `curl -x http://127.0.0.1:7890 <url>`。

## Ubuntu apt 源
- 换清华镜像（传统 `sources.list` 和 deb822 `ubuntu.sources` 都要 sed）：
  `https://mirrors.tuna.tsinghua.edu.cn/ubuntu`（对应 archive/security）。
- apt 走 Clash：
  ```bash
  sudo tee /etc/apt/apt.conf.d/proxy >/dev/null <<'EOF'
  Acquire::http::Proxy "http://127.0.0.1:7890";
  Acquire::https::Proxy "http://127.0.0.1:7890";
  EOF
  ```

## ROS2 Jazzy（Ubuntu 24.04=noble）
- apt-cache 里那个 `ros-jazzy-fastdds` **不存在**；Jazzy 只有 `ros-jazzy-fastrtps`(Fast RTPS 2)。
- 加 ROS 源用 **noble**（不是 jazzy）：`deb [arch=amd64 signed-by=...] http://packages.ros.org/ros2/ubuntu noble main`。
- key：`curl -x http://127.0.0.1:7890 -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`（raw.githubusercontent 直连 WSL 不通，走代理）。
- 装：`sudo apt install -y ros-jazzy-desktop`（下载量大，偶发 502 用 `-o Acquire::Retries=10`）。
- 每次新终端：`source /opt/ros/jazzy/setup.bash`（写入 `~/.bashrc` 一劳永逸）。

## Gazebo Harmonic + ros_gz_bridge
- Gazebo 官方源 key 在**根**：`https://packages.osrfoundation.org/gazebo.gpg`；源 `deb [arch=... signed-by=...] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main`。
- 装 `gz-harmonic`（OSRF 源，不是 packages.ros.org，国内经代理也较稳）。
- **ros_gz_bridge**：`sudo apt install -y ros-jazzy-ros-gz -o Acquire::Retries=8`（走 packages.ros.org）。
- 若 `ros-jazzy-ros-gz` 混入导致 `/etc/apt/sources.list.d/` 有两个同源不同 keyring → `Conflicting values set for Signed-By`，删掉一个（保留一个 source 一个 keyring）。

## PX4 SITL
- clone：`git clone -c http.proxy=... --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot`（几 GB）。
- 依赖：`bash Tools/setup/ubuntu.sh`（会与已加 Gazebo 源冲突，先删重复 source）。
- 编译+启动：`make px4_sitl gz_x500`（首次编译 20-40min；前台常驻，Ctrl+C 停）。
- Mavlink 端口在启动日志（`INFO [mavlink] ... on udp port XX remote port YY`），配 MAVROS 时用。

## 常见误判
- `libEGL ... /dev/dri Permission denied`：WSL 下 gz GUI 常见，只影响 GUI 渲染，不影响 server/传感器。用 headless `gz sim -s` 或数据判断。
- `gz topic -t /lidar` 显示 usage：要 `-i` 才是查类型。
