# PX4 → ROS2 接入：MAVROS(MAVLink) 是唯一干净路

本文件记录 2026-09 一次完整的 PX4 SITL + ROS2(Jazzy) 接入排查结论。
在多条路上逐个触发死结后，确认 **MAVROS(MAVLink, udp)** 是主流标准桥；uXRCE / Fast-RTPS 在当前 Jazzy 环境是死结。

## 三条通路的诊断结论(别再踩)
| 路线 | 结果 | 根因 |
|---|---|---|
| `px4_ros_com`(Fast-RTPS) | ❌ | 现代 PX4(main) 已移除 `rtps` 命令(`pxh> rtps status` → `Invalid command`)，Fast-RTPS 被废弃 |
| `micro_ros_agent`(uXRCE) | ❌ | 强制 `fastdds` v3；Jazzy 只有 `ros-jazzy-fastrtps`(Fast DDS 2.x)。装 `ros-rolling-fastdds` 后 CMake 报 `eProsima_atomic` target 冲突——两套 eProsima 库无法共存 |
| **MAVROS(MAVLink)** | ✅ 走通 | 用 MAVLink over UDP，不依赖 fastdds/fastrtps 版本 |

> 所以别再让 uXRCE / Fast-RTPS 那套(px4_ros2_ws、px4_ros_com、micro_ros_agent)浪费时间——直接删掉，走 MAVROS。

## MAVROS 接入步骤
```bash
# 1) 安装(包在 packages.ros.org，经代理；成片 502 就换 Clash 节点)
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-mavros-extras

# 2) PX4 SITL(提供 MAVLink)——已编译
cd ~/PX4-Autopilot && make px4_sitl gz_x500   # 记日志里 "on udp port X remote port Y" 那行

# 3) 起 MAVROS。注意 launch 文件名是 px4.launch(无 .py)！
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

# 4) 验证
ros2 topic echo /mavros/state --once    # connected: true + heartbeats 在涨
ros2 topic echo /mavros/local_position/pose --once
```
- `fcu_url` 端口要跟 PX4 SITL 日志的 mavlink 端口对齐；`connected:false` 就是端口不对。
- 探索算法后续用 MAVROS offboard(/mavros/setpoint_raw/local 或 /mavros/offboard)发位置/速度，或直接发 `/mavros/setpoint_position/local` 组合 setpoint。

## gz-sim 简化四旋翼(不接 PX4 时的替代，用户要"无人机跑算法"时)
主流探索框架(FUEL/RACER/EPIC)用 Gazebo + 简化四旋翼 + 直接发指令，不接 PX4。gz-sim8 标准做法：
- 用 **`MulticopterMotorModel`(×4, 每旋翼一个)** + **`MulticopterVelocityControl`**，都抄官方 `examples/worlds/multicopter_velocity_control.sdf` 的参数(`motorConstant 8.54858e-06`、`momentConstant 0.016`、`velocityGain 2.7 2.7 2.7`、`attitudeGain 2 3 0.15`、`angularRateGain 0.4 0.52 0.18`)。
- **指令话题 = `/<model>/gazebo/command/twist`**(gz.msgs.Twist)，不是 `/cmd_vel`。bridge 时 ROS `/cmd_vel`(geometry_msgs/Twist) → gz `/<model>/gazebo/command/twist`。
- **必须发 `enable`(gz.msgs.Boolean)才接管**：`/<model>/gazebo/command/enable` → `data: true`。launch 里用 `TimerAction(period=4)` 自动发。
- 官方模型是 fuel 远程 `<include>`(如 `X3 UAV`)，**无法往其 link 加 LiDAR** → 自写 X 型四旋翼(机身 box + 4 臂 visual + 4 电机 cylinder + 4 薄桨 cylinder，base_link 挂 `gpu_lidar`)，rotor link 用 revolute joint + MulticopterMotorModel，控制器参数抄官方。
- **简化四旋翼不接 PX4 时，探索算法直接发 `/cmd_vel`/setpoint 驱动**，感知经 `/scan`(bridge 自 gz `/lidar`)。用户最在意"随意介入算法"时，这个最顺(不受 PX4 框架套牢)。

## 杂项踩坑
- `git clone -b v2.0.4` 报 branch not found → 那是 tag，用 `git fetch --tags && git checkout v2.0.4`。
- 官方 source(GitHub raw)在 WSL 直连慢/不通，`curl -x http://127.0.0.1:7890` 走代理；GitHub API search 可用来找正确仓库名(eg `micro-ROS/micro-ROS-Agent` 是连字符大写，不是 `micro_ros_agent`；PX4 org 无 `PX4-ROS2-bridge`，只有 `px4_ros_com`/`px4_msgs`)。
- 用户若中途要"真实无人机形状"，视觉上给 X 型(4 臂+桨)，但**飞行物理用简化动力学即可**——真实飞控(PX4)反而是"随意介入算法"的负担。

## MAVROS 依赖坑：缺 GeographicLib geoid 数据直接崩
`ros2 launch mavros px4.launch ...` 启动后 `mavros_node` 秒崩：
```
terminate called after throwing an instance of 'std::invalid_argument'
what():  UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm
```
这不是连接问题，是 **MAVROS 缺地球重力位(geoid)数据集**。修：
```bash
sudo apt install -y geographiclib-tools
sudo geographiclib-get-geoids egm96-5     # 下载 egm96-5 数据集(可能需走 Clash)
```
然后重跑 `ros2 launch mavros px4.launch ...` 即可。**这是 MAVROS 在全新 Ubuntu 上的常踩坑，首次接入必查。**

## PX4 的 gz_x500 自带模型 = 无 LiDAR（探索感知要先补）
`make px4_sitl gz_x500` 起的 `x500_0` **默认不带 3D LiDAR**（只有相机/测距）。`ros2 topic list | grep -iE "lidar|scan|pointcloud"` 为空、只有 `/mavros/local_position/*`(位姿)——所以探索算法的"眼"(点云建图)要先自己加。
- 模型结构：`Tools/simulation/gz/models/x500/model.sdf` 是 `<include merge='true'><uri>model://x500_base</uri></include>` + 电机插件。**真正的机体(base_link + 传感器)在 `x500_base` 模型**。
- PX4 gz 自带 `models/lidar_2d_v2`(2D LiDAR 模型)，但 x500 默认不挂。
- **做法**：改 `Tools/simulation/gz/models/x500_base/model.sdf` 的 `base_link`，加一个 `gpu_lidar` sensor(topic=`/lidar`，3D：水平 360° + 垂直多线，量程 30m)，再 bridge `/lidar → /scan` 进 ROS2，探索算法订阅它 + `/mavros/local_position/odom`。
- 另一条更省事的路：不接 PX4，用上面的"gz-sim 简化四旋翼"(自写 X 型 + base_link 自带 `gpu_lidar`)，探索直接发 `/cmd_vel` 驱动——**多数探索论文(FUEL/RACER)就是这条**，感知/算法完全自由、不受 x500_base 无 LiDAR 和 PX4 框架限制。真实飞控(PX4)只在需要"飞行动力学/真部署"时才值得。

## 杂：PX4 SITL 会复用已在跑的 Gazebo(不开新窗)
`make px4_sitl gz_x500` 日志若出现 `gazebo already running world: <world>`，说明你的 WSL 里**已有另一个 gz sim 在跑**(比如先前 `ros2 launch exploration ...` 起的)，PX4 把 `x500_0` 塞进那个 world、**不再弹新窗口**，还可能跟你的自定义 world 打架。处理：`pkill -f "gz sim"` 清干净再让 PX4 独立起它自己的窗口。若 rv2/老 gz 进程僵着 `pkill -9 -f "gz sim"`。
