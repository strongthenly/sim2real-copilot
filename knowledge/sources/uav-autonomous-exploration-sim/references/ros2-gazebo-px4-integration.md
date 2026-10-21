# ROS2 + Gazebo + PX4 集成实测(国内网络 / Clash / WSL2)

这些是在 Windows WSL2 里搭 UAV 自主探索仿真平台时,被反复验证/踩过坑的**确定性**做法。涉及网络、安装、传感集成、PX4 接入四类。前提:WSL2 已建好 Ubuntu 24.04 发行版、内存已调大(见 `wsl-simulation-setup.md`)。

## 1. WSL2 + Windows 上的 Clash —— 关键解锁(默认 NAT 模式连不上代理)

**症状**:Windows 挂了 Clash(127.0.0.1:7890),但 WSL 里 curl/apt 连国外源超时或 000。

**根因**:WSL2 默认是 **NAT 模式**,走独立虚拟网卡(网关如 `172.25.x.1`),**流量不经过 Windows 的 VPN/代理**;而且 Clash 通常只监听 `127.0.0.1`,WSL 从 NAT 网关(172.25.x.1)够不到它 → 连接挂起/被拒。

**修法(不改 Clash 任何设置)**:在 `~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored`,然后 `wsl --shutdown` 重启:

```ini
[wsl2]
memory=12GB
processors=10
swap=4GB
networkingMode=mirrored     # 让 WSL 共享 Windows 网络栈 → 127.0.0.1 与主机互通
```

镜像模式后,WSL 里 `127.0.0.1:7890` 就是 Windows 的 Clash 端口,curl/apt 加 `-x http://127.0.0.1:7890` / 配 apt 代理即可走通。**验证镜像是否生效**:`ip route` 不再有 `172.25.x` 独立网段、`127.0.0.1:7890` 能 TCP 连上(而不是连到 WSL 自己的回环)。

> 注意:改 `.wslconfig` 前先备份(`.wslconfig.bak`),保留原有 `kernel=` 等行,只增不删。

## 2. ROS2 Jazzy 安装 —— 国内无镜像、必须走代理,且 dists 用 Ubuntu 代号

**关键事实**:
- **国内没有任何 ROS2 apt 镜像**(清华/阿里/USTC/华为/腾讯的 `ros2/ubuntu` 全 404)。
- 官方源 `packages.ros.org` 在美国 OSL 机房,**国内直连不通**(DNS 正常、解析到 osuosl.org,但连不上)。
- **`dists/` 子目录用 Ubuntu 代号(noble / jammy),不是 ROS 版本名(jazzy)!** 正确 URL 是 `http://packages.ros.org/ros2/ubuntu/dists/noble/Release`。用 `jazzy` 会 404。

**配置**(在 WSL 里):
```bash
# apt 走代理
sudo tee /etc/apt/apt.conf.d/proxy >/dev/null <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
EOF
# 加 ROS2 源 + key(key 下载也要走代理!)
sudo curl -x http://127.0.0.1:7890 -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop -o Acquire::Retries=8
```

**坑**:
- 下载 `ros.key`(raw.githubusercontent.com)在 WSL 里**直连不通**,必须 `curl -x http://127.0.0.1:7890`。
- Ubuntu 官方源也慢 → 换清华/阿里镜像提速(Ubuntu 源走 Clash 会自动直连国内镜像)。
- **Ubuntu 24.04 源可能是 deb822 格式(`/etc/apt/sources.list.d/ubuntu.sources`),替换 URL 时两种格式都要 sed。**

## 3. apt 报 `502 Bad Gateway [IP: 127.0.0.1 7890]` = Clash 节点质量差,不是网络/ROS 问题

**判读**:`apt update` 拉小元数据可能成功(200),但下载实际 deb(几 MB→几百 MB)时**成片 502** + `Fetched ... (17.6 kB/s)` 这种极慢速度 → 是**当前 Clash 节点对目标机房下载不稳/限速**,不是网络坏、不是 ROS 源坏。

**修法(结合)**:
1. **切 Clash 节点**到延迟低且**下载速度快**的(别只看 ping,要看下载 MB/s;选"专用/中转",避开公共节点)。
2. 提高 apt 容错:`-o Acquire::Retries=8` / 全局写 `Acquire::Retries "8";` 到 apt.conf.d/proxy,应对偶发 502。
3. 重启 Clash 可缓解节点半死连接。

**底线**:若换一圈节点仍成片 502/速度 <100kB/s,说明节点群到该机房都差,换条路(见下)。

## 4. apt source `Signed-By` 冲突(`Conflicting values set for option Signed-By`)

**症状**:安装某些工具(PX4 的 `ubuntu.sh` 等)会自己加 Gazebo 源,若之前已手动加过同名源但用了**不同的 keyring 文件**,apt 报 `Conflicting values set for option Signed-By ... != ...`、`list of sources could not be read`。

**修法**:`/etc/apt/sources.list.d/` 里同一个仓库只有一份 entry,重复的删掉。两个 keyring 往往是同一把 key(文件大小相同),保留任一即可。

## 5. ros_gz_bridge(Gazebo ↔ ROS2 桥)

装包:`sudo apt install -y ros-jazzy-ros-gz`。
可用工具:`ros_gz_bridge` 提供 `parameter_bridge` / `static_bridge` / `bridge_node`。
启动方式:launch 里 `Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': <yaml>}])`。

`bridge.yaml`(话题映射)格式:
```yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

**坑**:
- SDF/launch/worlds 是 `data_files`,改源码后**必须重新 `colcon build`** 才会同步到 install(launch 从 install/share 读)。
- 3D LiDAR 在 gz 侧话题是 `/lidar`(LaserScan)和 `/lidar/points`(PointCloudPacked),想直接取 3D 点云就桥 `/lidar/points -> sensor_msgs/msg/PointCloud2`。

## 6. Gazebo LiDAR 全 `.inf` 的几何坑(不是传感器坏)

**症状**:`ros2 topic echo /scan` 里 `ranges` 全 `.inf`,但传感器、桥、类型都正常。

**根因**:LiDAR 悬停高度 + 垂直 FOV(±0.3rad 这种窄视野)在目标距离处**没覆盖到障碍物**。示例:四旋翼 static 在 z=2m,墙高仅 1m(z 0~1)且距 3m,垂直视锥只覆盖 [1.07, 2.93] → 墙全在下方扫不到 → 全 `.inf`。

**修法**:要么**把墙加高**(如 4m)让它进入视锥,要么**加大 LiDAR 垂直 FOV**,要么**降低悬停高度**。改 SDF 后 rebuild + 重启 launch。
**验证**:`top`/几何算一下:在水平距离 d 处,垂直覆盖范围 ≈ `[z - d*tan(vmax), z + d*tan(vmax)]`,确认障碍物 z 区间与之相交。

## 7. PX4-ROS2 接入 —— Jazzy 用 Fast-RTPS(px4_ros_com),别强上 uXRCE/micro_ros_agent

**关键不匹配**:
- **ROS2 Jazzy 自带的是 Fast DDS 2.x(`ros-jazzy-fastrtps`)**,**没有** `ros-jazzy-fastdds`(Fast DDS 3.x 只在 `kilted/rolling` 源有)。
- **`micro_ros_agent`(uXRCE-DDS 方案)强制要求 Fast DDS v3**(CMake `find_package(fastdds ... version 3)`)→ 在 Jazzy 上编译**必然报错找不到 fastdds v3**。
- 所以 **Jazzy 上 PX4-ROS2 走 Fast-RTPS 方案(px4_ros_com)才是通的**——它用 Fast DDS 2.x,Jazzy 自带,能直接 compile 过。

**px4_ros_com 用法**:
```bash
mkdir -p ~/px4_ros2_ws/src && cd ~/px4_ros2_ws/src
git clone -c http.proxy=http://127.0.0.1:7890 -c https.proxy=http://127.0.0.1:7890 --recursive https://github.com/PX4/px4_ros_com.git
git clone -c http.proxy=http://127.0.0.1:7890 -c https.proxy=http://127.0.0.1:7890 --recursive https://github.com/PX4/px4_msgs.git
cd ~/px4_ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build
```
> px4_ros_com 依赖 px4_msgs,必须把**两个**都放进同一个 workspace 的 `src/`,否则 `find_package(px4_msgs)` 报"Could not find px4_msgsConfig.cmake"。

**PX4 SITL 启动**:`cd ~/PX4-Autopilot && make px4_sitl gz_x500`(首次编译 20~40 分钟)。SITL 侧启用 RTPS:在 `pxh>` 里 `rtps status` 看状态、必要时 `rtps start`;ROS2 侧 `ros2 launch px4_ros_com px4_ros_com.launch.py`。出现 `/fmu/out/vehicle_status` 等 = PX4↔ROS2 通。

**仓库名注意**:PX4 官方 ROS2 接口仓库叫 **`PX4/px4_ros_com`**、消息包叫 **`PX4/px4_msgs`**;`PX4/PX4-ROS2-bridge` 不存在。uXRCE 的 agent 源码仓库是 **`micro-ROS/micro-ROS-Agent`**(连字符大写 -A,不是下划线 `micro_ros_agent`,后者 clone 404)。

## 8. GitHub 私有仓库推送(credential 免密)

- 私有仓库 `ls-remote` 会报 `could not read Username`(需认证)。
- **GitHub 已不接受登录密码**,`Password` 栏必须填 **PAT**(在 https://github.com/settings/tokens/new 生成,勾 `repo` scope)。
- 免每次输入:全局 `git config --global credential.helper store`,之后 push 输一次 PAT 即存到 `~/.git-credentials`。
- 若远程是新建时 GitHub 自动初始化的(有 README/.gitignore),本地 push 会报 `! [rejected] ... (fetch first)`,本地是**整套要覆盖**的用 `git push -u origin main --force`,否则 `git pull --rebase` 合并。
