# 探索/规划算法接入 AirSim+PX4（含 FUEL 现实）

## 一、算法驱动无人机的三条控制通路
```
ROS2 规划器/探索节点(WSL) → MAVROS(offboard) → PX4 v1.11.3(WSL) → AirSim 无人机(Windows)
```
- **首选：MAVROS（ROS2）+ PX4 offboard** —— 贴合 ROS2 探索栈、无老依赖坑。发 `PoseStamped` 到 `/mavros/setpoint_position/local`（20Hz 持续流）→ 切 `OFFBOARD` → arm。见 `scripts/offboard_planner_test.py`（最小验证 + 控制器起点模板）。
- **次选：AirSim Python API（airsim 库）** —— 见 SKILL.md 的 Python 3.9 venv 说明（v1.8.2 老 msgpackrpc 与 3.12 不兼容）。它直连 AirSim RPC(41451)，**绕过 PX4**（不算"PX4 操控"）。
- QGC `.plan` Mission 是**人工**下发航线，不是"算法"驱动。

**验证顺序**：① QGC 里 Arm/Takeoff 确认 PX4 真控制 AirSim（最直观）→ ② MAVROS 连 PX4 → ③ 跑 offboard 节点让无人机按算法飞（方形航线）→ 即证明**环境可被算法驱动**。

## 二、感知/里程计回路（探索算法还要"看"）
探索算法需要：**点云/深度（建图）+ 里程计（定位）**。从 AirSim 取：
- **里程计**：MAVROS 的 `/mavros/local_position/odom`、`/pose`、`/velocity_*`（PX4 EKF 输出，最省事）。
- **点云/图像**：AirSim 传感器 → ROS。`gz_x500` 默认不带 LiDAR；需在 AirSim 侧配 LiDAR/Depth 相机，再桥到 ROS（话题名对齐实机/算法输入）。
- 用实机时是 MID360 + D435 → FAST-LIVO2；仿真里则用 AirSim 传感器 + 同样的话题喂同一算法。

## 三、FUEL 接入的现实（★ 2026-09-12 更新：别再走 Docker / ros1_bridge）

> **本节旧结论已作废**：原先写的是「用户环境是 ROS2 Jazzy / Ubuntu 24.04 → FUEL 直接装不了，
> 只能用 Docker 跑 ROS1 Noetic + `ros1_bridge`」。那是**当时只有 `lyuav`** 的判断。
> 现在有了 **`ros1-twin`（Ubuntu 20.04 + ROS1 Noetic + gcc9）** → FUEL 在上面
> **一次编译通过（25 包 / 0 error）**、自带 demo 能自主探索 → **直接装在 ros1-twin 上即可，
> 零 bridge、零 Docker**。
> 完整实战流程（依赖 / 编译 / demo / I/O 契约 / 输入桥 / 剥离自带仿真 / 坑）见
> **`references/fuel-planner-integration.md`**。

FUEL（HKUST, Fast UAV Exploration）**不是** ROS2 项目：
- **运行环境 ROS1（Melodic/Noetic, Ubuntu 20.04）** + 自带 `uav_simulator`（简化仿真；**接自研平台时必须剥离**）。
- 输入：**点云/深度 + 里程计 + 传感器位姿**（三条，见 integration 参考）；输出：`PositionCommand`（驱动无人机）。
- **选发行版的原则**：已有 ROS1 发行版（如 `ros1-twin`）就**直接装在它上面**；
  只有 ROS2 环境时才退而求其次用 Docker + `ros1_bridge`（下策，能避则避）。

**接入三段**：① 感知：FAST-LIVO2/AirSim 点云 → FUEL 输入；② 里程计 + 传感器位姿 → `/odom_world` + `/map_ros/pose`；③ 控制：FUEL 的 `/position_cmd` → MAVROS → PX4 → AirSim。
（若目标是 ROS2 自研探索栈而非 FUEL，仍可选：RACER ROS2 分支 / FUEL 社区移植 / 自研 frontier，`ros2_ws/src/exploration` 已有骨架 —— 纯 ROS2，不经 bridge。）

## 四、论文方向（用户目标：期刊二区，算法/平台皆可）
- 用户**独有资源** = AirSim 高保真 + 真 PX4 + 实机复刻 + 有实机可验证 → 差异化在 **Sim-to-Real**。
- **路线 P（平台型，二区稳收）**：高保真复刻平台（实机在 AirSim + PX4 在环 + 传感器话题级一致）。目标刊：**SoftwareX / Drones / Sensors / IEEE Access**。
- **路线 A（算法型，分量更高）**：高保真仿真驱动的 LiDAR 探索 + 实机验证，FUEL/RACER 作 baseline。目标刊：**Sensors / Drones / RAS**，够强冲 RA-L。
- **推荐**：平台为骨架 + 一个算法对比实验为亮点（避免"只平台"）。二区友好刊：Drones / Sensors / IEEE Access / Journal of Intelligent & Robotic Systems / Aerospace / SoftwareX。
- 核心图表素材 = **仿真-实机一致性验证**（同话题同格式对照）+ 探索效率对比。
