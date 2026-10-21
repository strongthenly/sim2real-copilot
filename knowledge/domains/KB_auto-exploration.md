# 自主探索与仿真平台（ROS1 双胞胎 / WSL ROS2 / PX4 SITL）

> 领域 `auto-exploration` ｜ 由 5 个原始技能合并而成 ｜ 生成于 2026-10-21

**这一层解决什么**：探索算法是论文的主战场；这一层记录平台选型结论、国内网络下的安装坑，以及"双胞胎栈"怎么和实机对齐。

**建议读法**：A 平台选型与关键判断 → B WSL/代理/apt 源 → C ROS1 双胞胎栈 → D 批量实验与验证

**包含的原始技能**：`uav-autonomous-exploration-sim`、`uav-ros2-sim-setup`、`wsl2-ros2-setup`、`ros2-gazebo-sim-setup`、`wsl2-ros2-sim-platform`


---

### 来源技能 `uav-autonomous-exploration-sim`

> 搭建"UAV 自主探索(autonomous exploration)算法研究"仿真平台——面向发论文的算法对比实验。覆盖技术栈选型(RUbuntu/ROS2/Gazebo 版本兼容矩阵)、批量实验模式(headless gzserver)、WSL 上的搭建流程、以及"飞控动力学对探索论文是次要"的关键判断。适用于用户要做 PX4/ROS 无人机的自主探索、SLAM、导航规划研究并需要可复现仿真实验的场景。

#### 关键判断(先讲清,别直接埋头装)

**自主探索论文的飞控动力学是次要的。** 大量顶会/RA-L 探索工作(RACER、fast-explorer 那类)直接在 Gazebo 里用**简化四旋翼动力学**跑，不需要真 PX4 SITL。真正花精力的是三件：
1. **高保真 LiDAR 点云建图**(探索算法的感知基础)——能真实仿真出点云，才能对接 FAST-LIVO2 建图。
2. **可编程/随机环境**——才能批量跑实验，对比覆盖率、探索完成时间、路径长度、算力占用(这就是论文指标)。
3. **与主流开源探索库兼容**——才能做 baseline 对比(审稿人看"你改进了哪个已有算法")。

**发论文几乎都走"基于开源库二开 + 做对比"路线**：站在已有算法(RACER / fast-explorer / FALCON)边上做增量创新，比完全自研好发得多。

**主流论文平台的实证结论(2026 调研，用户问"他们出论文用什么"时用)**：高引用 UAV 探索框架 **FUEL(1471⭐)、RACER(789⭐, TRO 2023)、EPIC** 全部用 **Gazebo + 简化四旋翼动力学 + 直接发指令**，并**不用 PX4**(各自带 uav_simulator / FAST-LIO + frontier)。**AirSim(Unreal)只在"高保真视觉/相机/RGBD / 实机复刻"类论文**用，是少数派，且必须 Unreal Engine(几十 GB 下载 + 好显卡，WSL 还跑不了 UE)。给用户选型时：探索论文优先 **Gazebo(+PX4/MAVROS 或简化四旋翼)**，**不要默认 AirSim**。**但若用户明确要"高保真复现实机 + 真实视觉"**，AirSim 就是正解，且可用**混合架构：Windows 跑 Unreal/AirSim(仿真)+ WSL 算 ROS/PX4/探索算法(大脑)**——靠 `networkingMode=mirrored` 让 WSL 的 `127.0.0.1` 与 Windows 互通即可(Clash / AirSim RPC 都靠这个)。

#### 推荐技术栈(2026 UAV 自主探索论文主线)

| 层 | 推荐 | 说明 |
|---|---|---|
| 系统 | **Ubuntu 24.04 LTS**(强烈建议) | 前沿探索库几乎都是 ROS2，且新版 Gazebo 要 22.04+；24.04 是 Jazzy / Gazebo Harmonic 原生搭档 |
| ROS | **ROS 2 Jazzy**(或 Humble) | 社区最适合发论文、探索库/工具最全 |
| 仿真 | **Gazebo Harmonic** + 真实点云 LiDAR 模型 | 保真建图基础 |
| 建图 | **FAST-LIVO2**(ROS2 移植)或 FAST-LIO | 与实机感知一致 |
| 探索策略 | frontier / 信息增益(RACER、fast-explorer 等做 baseline) | 或自研 |

#### 版本兼容矩阵(最容易踩的坑)

- **Ubuntu 20.04**：只剩 ROS Noetic + Gazebo Classic 11(旧)。跟主流开源探索库(ROS2)不兼容，须大量自改移植；审稿人看到旧栈观感打折。**只有当要真部署到 Jetson(20.04+Noetic)实机、且坚持仿真↔实机同栈时才选。**
- **Ubuntu 22.04**：ROS2 Humble + Gazebo Harmonic，可行。
- **Ubuntu 24.04**：ROS2 Jazzy + Gazebo Harmonic，官方原生，最稳(首选)。
- **默认 WSL 发行版常是新 LTS(如 26.04)→ 太新**，ROS2/Gazebo 官方包适配不成熟，应**新建一个 24.04 发行版**专门做仿真，别用默认那个。

#### 批量实验模式(关键技巧)

- **批量实验用 headless `gzserver`**(不渲染，纯 CPU 就能跑物理+传感器仿真)——不依赖 GPU，RTX3050 之类不用操心。
- **可视化用 WSLg**(Windows WSL2 默认自带，不用额外装 X server)。Gazebo 3D 渲染、Rviz 全走它。
- 物理内存偏紧时(如 WSL 只分到 7.6GB)用 `.wslconfig` 调大——见 `references/wsl-simulation-setup.md`。

#### 搭建流程

1. (若在 Windows)确认/新建 Ubuntu 24.04 WSL 发行版、调内存 —— 见 `references/wsl-simulation-setup.md`。
2. **搞定国内网络走 Clash(关键前提)** · 装 ROS 2 Jazzy + Gazebo Harmonic + ros_gz_bridge —— 见 `references/ros2-gazebo-px4-integration.md`。
3. 建一个带 3D LiDAR 的 Gazebo world,桥 `/scan`(`/lidar`) 进 ROS2,`ros2 topic echo /scan` 确认有真实反射值(不是全 `.inf`)。
4. (若走真飞控)编译 PX4 SITL(`make px4_sitl gz_x500`),用 **MAVROS(MAVLink)** 接入 ROS2。现代 PX4 已移除 `rtps` 命令,Fast-RTPS(px4_ros_com)是废弃姿势;uXRCE(micro_ros_agent)在 Jazzy 必因 Fast DDS v3 编译失败;**唯一干净路 = MAVROS**。让 `/mavros/state.connected` 变 true、`/mavros/local_position/pose` 有数据。详见 `references/px4-mavros-route.md`。
5. 拉 FAST-LIVO2(ROS2 版)验证建图;拉探索 baseline(RACER / fast-explorer / FALCON)并编译。
6. headless 起仿真 → 记录覆盖率/探索时间/路径长度 → 对比。

#### 关键坑(实测,详见 references/ros2-gazebo-px4-integration.md)

- **WSL2 默认 NAT 模式连不上 Windows 的 Clash**:必须在 `~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored` + `wsl --shutdown`,WSL 才能用 `127.0.0.1:7890`(默认 NAT 下走独立 172.25.x 网卡、够不到只监听 127.0.0.1 的 Clash)。
- **国内无 ROS2 apt 镜像**;官方源 `packages.ros.org` 直连不通(OSL 机房)。必须给 apt 配代理(`/etc/apt/apt.conf.d/proxy`),且 **`dists/` 用 Ubuntu 代号(noble)不是 ROS 版本名(jazzy)**。下载 `ros.key`(raw.github)必须 `curl -x` 走代理。
- **apt 成片 502 = Clash 节点质量差**(不是网络/ROS 坏):换节点(看重下载 MB/s 而非 ping)+ `-o Acquire::Retries=8`。
- **Gazebo LiDAR 全 `.inf` 是几何坑**:墙高/LiDAR 高度/垂直 FOV 没对上。z 处悬停、垂直半角 v,覆盖 ≈ `[z-d*tan(v), z+d*tan(v)]`,障碍物 z 区间要与之相交。
- **PX4→ROS2 唯一干净路 = MAVROS(MAVLink)，别走 uXRCE / Fast-RTPS**：现代 PX4(main)只支持 uXRCE-DDS，`pxh>` 里 `rtps` 命令已移除；`px4_ros_com`(Fast-RTPS)是废弃姿势；`micro_ros_agent`(uXRCE)强制 Fast DDS v3，而 Jazzy 只有 Fast DDS 2.x(`ros-jazzy-fastrtps`)，两者是同名 eProsima target(`eProsima_atomic`)冲突 → 编译死结。**MAVROS 走 MAVLink(udp)，不需要 fastdds**，是主流标准桥。详细步骤见 `references/px4-mavros-route.md`。
- **mavros 的 launch 文件名是 `px4.launch`（没有 `.py`）**：`ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`；连上后 `ros2 topic echo /mavros/state` 应 `connected: true`(heartbeats 在涨)。若 connected:false 是端口不对——看 PX4 SITL 日志 `INFO [mavlink] ... on udp port X remote port Y` 精确配 fcu_url。
- **gz-sim 简化四旋翼：`MulticopterVelocityControl` 必须发 `enable`(gz.msgs.Boolean)才接管**——否则四旋翼悬停/原地不动。launch 里用 `TimerAction`(period≈4s) 自动 `gz topic -t /<model>/gazebo/command/enable -m gz.msgs.Boolean -p "data: true"`。官方已验证的配置参考 gz-sim8 的 `examples/worlds/multicopter_velocity_control.sdf`(MulticopterMotorModel×4 + MulticopterVelocityControl，指令 topic `/<model>/gazebo/command/twist`，`comLinkName`/`velocityGain`/`attitudeGain`/`angularRateGain`/`motorConstant` 等抄官方值)。
- **PX4 gz_x500 自带模型无 LiDAR**：`x500/model.sdf` 只是 `<include model://x500_base>`，真正的 base_link/传感器在 `x500_base`；探索的"眼"(点云建图)要往 `x500_base` 的 base_link 加一个 `gpu_lidar`(**topic=/lidar**)再 bridge → 更省事是直接用上面简化四旋翼(自写 X 型+base_link 自带 LiDAR)。PX4 无 LiDAR 时 `ros2 topic list | grep -iE "lidar|scan|pointcloud"` 为空、只有 `/mavros/local_position/*`。
- **PX4 SITL 会复用已在跑的 Gazebo**：日志出现 `gazebo already running world: ...` 说明另有 gz sim 在跑，PX4 把模型塞进去、不开新窗，还打架——先 `pkill -f "gz sim"` 清干净再让 PX4 独立起窗。
- **mavros 首次接入必装 GeographicLib geoid**：`ros2 launch mavros px4.launch` 若秒崩并报 `GeographicLib exception: File not readable .../egm96-5.pgm`，是缺地球重力位数据——`sudo apt install -y geographiclib-tools && sudo geographiclib-get-geoids egm96-5` 再重跑。详见 `references/px4-mavros-route.md`。
- **官方模型是 fuel 远程下载的 `<include>`(如 `X3 UAV`)，无法往其 link 塞 LiDAR sensor** → 要自写四旋翼模型(机身+4臂+4桨=X 型，视觉真实) + 抄官方 MulticopterMotorModel/MulticopterVelocityControl 参数 + 在 base_link 挂 LiDAR。
- **`git clone -b <tag>` 报 "branch not found" 是因为那是 tag 不是 branch** → 用 `git fetch --tags && git checkout <tag>`。
- **Windows 侧 `wsl.exe -- bash -lc '...'` 引号极易坏**(中文 echo、`$`、`()`、内嵌双引号被 PowerShell 转义)→ **把命令写成 .sh 脚本复制到 WSL 执行**(write_file 到 `C:/Users/.../x.sh`，再 `wsl -d <distro> -- bash /mnt/c/.../x.sh`)，别内联长命令。这也是所有 wsl 交互式命令(装包/诊断)的稳定姿势。
- **接 FUEL(及多数经典探索库) 到 ROS2/AirSim 栈的障碍(实测)**：FUEL 原始是 **ROS1(Noetic/Ubuntu 20.04)** + 自带 uav_simulator；在 ROS2(22.04/24.04) 栈上用只能 **Docker 跑 Noetic 版 FUEL + `ros1_bridge`(ROS1↔ROS2)**，或改用 **ROS2 版探索框架**(RACER 有 ROS2 版/社区移植)。接进 AirSim+PX4 平台要打通**三段**：①感知(AirSim LiDAR/深度点云 → ROS → FUEL 建图输入) ②里程计(AirSim 位姿 / FAST-LIO → FUEL 定位) ③控制(FUEL 的 `PositionCommand` → AirSim/PX4)。**AirSim 原生 LiDAR 是机械重复扫描，MID360 是非重复花瓣式**——高保真复刻 MID360 要近似或写 custom sensor(否则点云分布就不是 MID360 的样子)。
- **修正：若有现成 ROS1(Noetic) 环境(如 WSL `ros1-twin`)，FUEL 是「直接编译就跑」(实测 2026-09)**：建**独立工作空间 `~/fuel_ws`**（别污染既有 `twin_ws`）→ `find src -name package.xml` 得 **25 个包** → `catkin_make -j4` **rc=0、零 error**（只有 unused-variable 警告）→ demo 跑通自主探索。**不必 Docker、不必 `ros1_bridge`**。依赖只有两个坑：**`nlopt` 要源码装 v2.7.1**（`libarmadillo-dev` 自带即可）；**LKH 不用单独装**——`utils/lkh_tsp_solver/CMakeLists.txt` 用 `file(GLOB SRCS src/*.c)` 把 ~120 个 LKH 源码**编进库了**（曾以为要下 LKH 二进制，白找一轮）。
- **FUEL 的确切接口(读源码核实，接入照这个接)**：输入 `/odom_world`(nav_msgs/Odometry) + **`/map_ros/pose`(传感器位姿 PoseStamped，地图 raycast 用)** + `/map_ros/cloud`(PointCloud2) 或 `/map_ros/depth`（**depth 与 cloud 只订一个**）；触发 `/move_base_simple/goal`（可**编程 `rostopic pub` 发，不必手点 RViz**）；输出 **`/position_cmd`**(`quadrotor_msgs/PositionCommand`，带 position/velocity/acceleration/yaw **前馈**)；frame 默认 `world`（FAST-LIVO2 是 `camera_init`，必须对齐）。**→ 接 MAVROS 的取舍见下方「桥已落地」一节（默认用 `setpoint_position/local`，别自己手算 ENU↔NED）。**
- **跑 FUEL demo 别用主 master**：给它**独立 master(换端口如 11321)** + 独立 `ROS_MASTER_URI`，与在跑的链路零干扰。demo 自带仿真栈(`pcl_render_node` 渲深度 + `map_generator` 发 office.pcd + `so3_control`/`quadrotor_simulator_so3`)，`roslaunch exploration_manager exploration.launch`。**实测性能(本机 i5-12500H)：Total 14.4ms**（Cost mat 3.5 / TSP(LKH) 1.9 / Local refine 0.36 / Kino 0.2 / Traj 5.8），比论文 i7-8550U 的 24-36ms 还快——**这个数字可直接进论文性能对比表**。
> **FUEL 完整接入方案(5 阶段路线 + 两个桥的设计 + 算法突破切入点 + 找/下载 baseline 论文的 API 姿势)见 `references/fuel-integration.md`。**

#### FUEL 接真实平台：桥已落地（阶段④⑤ 实测通过）

- **做法：新建独立小包 `fuel_bridge`，不改 FUEL 源码**——`scripts/fuel_input_bridge.py`(FAST-LIVO2→FUEL) + `scripts/fuel_ctrl_bridge.py`(FUEL→MAVROS) + `launch/fuel_real.launch`（**只含算法，不含自带仿真**）+ `scripts/start_fuel_explore.sh`(一键启动)。实测 FUEL 用 FAST-LIVO2 真实点云建出 **`±4.5m`（=MID360 量程）** 的场景占栅格、frontier 簇 20 个、TSP 矩阵正常。**接口清单/代码骨架/验证配方 + 「撞墙/飞错方向」的坐标系分层诊断（§12，含**对齐落地实现**）+ **防撞六参数调优表（§13）** 见 `references/fuel-bridge-implementation.md`；只读对齐诊断脚本 `scripts/check-frame-alignment.sh`（**含连续采样残差法：对齐正确时应有 `p_px4 ≈ Rz(psi)·p_fuel`，稳定样本平均残差 <0.3m 判对** —— 单次采样在无人机旋转时会给出乱跳的 psi，别据此下结论）**。⚠️ **阶段⑤ 头号误判：FUEL 启动后停在 `WAIT_TRIGGER`，不显式触发（pub `/move_base_simple/goal`，z≥-0.1）就永不规划、无人机原地悬停**；日志里的 `to visit:`/`cost mat` 出自 0.5s 的 `frontierCallback`、**不代表探索在跑**——判据看 `Triggered!` 与每秒一行的 `[FSM]: state:`（详见该文件 §9/§10）。
- **★ `init_z` 必须等于 `seed_z`（"起飞卡住"最隐蔽的真因，实测 2026-09-12）**：`traj_server` 启动时**无条件先发一条 `(init_x, init_y, init_z)` 占位 `PositionCommand`**（源码 `traj_server.cpp`：`nh("~")` 读参 → 直接赋给 `cmd.position` 后发出）。控制桥一见 `last_cmd != None` 就**永远走 FUEL 分支、再也不走 seed 悬停分支** ⇒ 无人机老实飞到 `init_z`（如 1.0m）**锁死不动**，而启动脚本死等 `seed_z`（1.5m）永不满足。**修法：`<arg name="init_z" default="$(arg seed_z)"/>`（单一来源）**，并给等待逻辑加"**高度连续不变即算稳定悬停**"的兜底，别硬等具体数值。参数真实路径是 **`/traj_server/traj_server/init_z`**（node 名 + param 名叠加，因 traj_server 用 `nh("~")`）——查 `/traj_server/init_z` 会"查不到"而误判。
- **★ 无人机「疯狂撞墙 / 飞错方向」先分层，别乱查（实测 2026-09-12）**：判据是**算法日志是否在正常推进** —— `Next view:` 持续更新 + `[FSM]: from EXEC_TRAJ to PLAN_TRAJ` 循环 ⇒ **算法层是好的、问题在执行层**（发的目标 ≠ 实际飞的方向）；`Next view` 不动才是没触发/仿真冻结。执行层**头号原因是两个坐标系的 yaw 没对齐**：算法侧 `world` = SLAM 的 `camera_init`（朝向 = SLAM 初始化那刻 LiDAR 的朝向），PX4 侧 = EKF home / MAVROS ENU（朝向来自磁力计）——两者初始朝向大概率不同 ⇒ 算法说"往左"、PX4 飞"往前" ⇒ 直冲墙壁。**一键只读诊断 `scripts/check-frame-alignment.sh`**（对比两个位姿的 yaw/位置偏差；>10° 就是它，修法=输出桥里做 yaw 对齐）。次要原因：**FUEL 只避让「已知」障碍**（`max_ray_length` 默认 4.5 太小，未知区域当可通行，没扫到的墙会被直接穿过）——**但别只改这一个**：FUEL 整套防撞余量都是按它自家小场景 demo 配的（`obstacles_inflation` 默认 **0.199 ≈ 20cm**，对 ~0.5m 机身**等于没有余量**；`max_vel` 1.5 配 20cm 余量 ⇒ 容错仅 ~0.13s）⇒ **六个参数要一起调**（含 `clearance_threshold` / `min_candidate_clearance` / `max_acc`；实测表 + 改法 + 备份约定见 `references/fuel-bridge-implementation.md` §13）。**顺序很重要：坐标对齐没修好之前调这些参数是白费功夫。**
- **一键生命周期脚本（实测可用）**：`~/fuel_all.sh [seed_z] [mode] [goal_x] [pcd]`（AirSim 自检 → twin 链路 `--full` → 开 PX4 专线 → 起 MAVROS → 起 FUEL+控制桥 → 自动起飞 → 自动发 Nav Goal 触发）与 `~/fuel_stop.sh [--all|--no-save|--keep-px4]`（默认**保留 ROS master 常驻**）。**关闭 FAST-LIVO2 必须用 SIGINT**（`pkill -INT -f fastlivo_mapping`）——保存 PCD 是 `main()` 在 `ros::spin()` 返回后调 `savePCD()`，**`kill -9` 直接丢图；且 roscpp 也不响应 SIGTERM，所以「超时就 kill -TERM」是无效兜底（见 `uav-ros-mapping-lifecycle`），只能是重发 SIGINT 或明确改成 SIGKILL 并告知不保存**。
- **PCD 保存开关**：参数 `pcd_save/pcd_save_en`(bool) + `pcd_save/interval`(默认 -1)，落盘到 `~/twin_ws/src/FAST-LIVO2/Log/pcd/{all_raw_points,all_downsampled_points}.pcd`。⚠️ 参数**在节点构造时读一次** ⇒ FAST-LIVO2 已在跑时改它**无效**（须先停再起），脚本要主动提示。
- **"点云时有时无"不是等不够久，是仿真冻住了**——判据只需一条命令：**`rostopic hz /clock`**。链路是 `PX4 死/tmux server 死 → TCP4560 断 → LockStep 下 AirSim 不步进 → /clock 停 → 所有传感器话题全无数据`，此时等 90s 还是 900s 都没用。配套判据：`ss -tn | grep ':4560'`（无=没连）、`pgrep -f bin/px4`、**`tmux ls`（无 `px4` 会话 = tmux server 死了，PX4 被 SIGHUP 杀）**（这组判据已封装成 `scripts/check-sim-health.sh`，直接跑）。健壮启动脚本应按序自检 `4560 → /clock 步进 → /twin → /cloud_registered`，最后一项超时就**自动重启 FAST-LIVO2（最多 3 次）**。
- **FUEL 的 `exploration.launch` 混着自带仿真，接真实平台必须拆**：它 = `algorithm.xml` + `traj_server` + `waypoint_generator` + **`simulator.xml`(pcl_render_node/map_generator/so3_control/quadrotor_simulator_so3)** ⇒ 去掉 `simulator.xml` 那段，只留算法三件，输入交给自己的桥。
- **桥只改 `frame_id` 字符串即可**：FAST-LIVO2 的点云本身已是配准后的世界系坐标，把它当 FUEL 的 `world` 用；不做坐标变换（只把 `frame_id` 从 `camera_init` 改写为 `world`），位姿同时转发给 `/odom_world` 与 `/map_ros/pose`。
- **`PositionCommand` → MAVROS 的两条路（默认选①）**：① **`setpoint_position/local`(PoseStamped，ROS 侧 ENU，MAVROS 自动转 PX4 的 NED)** ← **默认**：位置 + yaw 转四元数即可，**不要自己做 ENU↔NED**（手算易错）；② `setpoint_raw/local`(PositionTarget，NED) 只在确实要速度/加速度前馈时用，做成可选参数。
- **OFFBOARD 前必须预发 setpoint**：PX4 硬要求先连续收到 setpoint（约 3s）才允许切 OFFBOARD ⇒ 桥的节奏是"**先发 N 秒 setpoint → `set_mode(OFFBOARD)` → `arming`**"，做成 `offboard_wait` 参数。
- **别在 `rospy.Timer` 回调里做阻塞式服务等待**（`wait_for_service` 会卡住整个 50Hz 循环，桥看起来"没输出"）⇒ 惰性 `ServiceProxy` + 调用时 try/except。
- **状态变更类服务调用必须节流，否则日志刷屏**：`set_mode(OFFBOARD)` / `arming` 写在 50Hz 的 tick 里、又不加门控 ⇒ 用户 tail 到的是连续几十行 `[fuel_ctrl] → Arm`。加 `_next_action = now + Duration(2.0)`，每 2s 最多动作一次。⚠️ 顺带一个**误读陷阱**：`armed` 在 MAVROS 侧可能滞后/误报（无人机明明在飞却一直读到 `armed: False`）⇒ **判"飞没飞"看 `/mavros/local_position/pose` 有没有在动，别看日志还在不在打 Arm**。
- **FAST-LIVO2 冷启动要 30~60s，期间刷 `[ LIO ]: No point!!!` 是正常的**（IMU 初始化 3.3%→100% + 首帧）——**别误判成"点云格式不对/坏了"**；等出现 `effective feature num: 9xxx` / `Update Voxel Map` 才算就绪（实测手动跑满 45s 后 residual 0.0035 完全正常；`No point` 出自 `LIVMapper.cpp` 的 `feats_undistort->empty()`）。
- **XML 注释里禁止连续双连字符 `--`**：`<!-- ---------- 标题 ---------- -->` 会让 launch **文件非法**、roslaunch 报 XML 解析失败 ⇒ 写 `<!-- 标题 -->`。
- **验证 ROS 转发必须"先订阅、后发布"**：publisher 默认不 latch，`rostopic pub` **之后**才起的 `rostopic echo` 会错过消息（症状像"桥没转发"，其实是测试方法错）。正确：先后台 `rostopic echo > file` → 再 pub → 再 grep。⚠️ 测试消息**别省字段**（`PositionCommand` 缺 `velocity/acceleration` 会让 pub 无效、桥根本没收到）。
- **改完必须确认 Windows 备份 ↔ WSL 部署一致**：`patch` 改的是 Windows 侧文件、跑的是 WSL 那份 ⇒ `cp` 过去并在验证里加 **md5 对比**（本轮就因漏同步出过"静态检查 FAIL"的假警报）。
- **验证脚本自身也算改动，会形成"创建→验证→再验证"循环**：把脚本**经 base64 传入、在 WSL 内 `/tmp` 临时落盘执行后删除**（Windows 侧零新增文件），顺带绕开 PowerShell 引号地狱：`[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($s))` 然后 `wsl ... bash -c "echo <b64> | base64 -d > /tmp/x.sh; bash /tmp/x.sh; rm -f /tmp/x.sh"`。
- **`grep` 检查代码要匹配"真实调用"而不是词**：`grep wait_for_service` 会命中**注释里**的同名文字 ⇒ 用 `grep "rospy\.wait_for_service("`。
- **`pkill -f` 自匹配的另一变体**：内联 `wsl ... bash -c "... pkill -f roslaunch ..."` 时**命令行本身含该模式** ⇒ 杀掉自己（症状：命令无输出、退出码 15）⇒ 清理逻辑一律写进 `.sh` 文件再执行。
- **用算法驱动 AirSim 无人机做环境验证，首选 `算法(ROS2) → MAVROS(offboard) → PX4 → AirSim`**（不要指望 AirSim 的 `airsim` Python 包，其依赖与新版 Python 冲突）。⚠️ **但 AirSim 会占用 PX4 的 onboard MAVLink(14540/14580)，与 MAVROS 默认 `fcu_url` 撞车**（症状 `VER: broadcast request timeout`/`connected:false`）→ 在 PX4 `pxh>` 另开专线 `mavlink start -u 14600 -o 14601 -r 2000000 -t 127.0.0.1 -m onboard`，MAVROS 用 `fcu_url:=udp://:14601@127.0.0.1:14600`。

- **AirSim LockStep：必须先有 PX4 连上 TCP4560，仿真才跑**（AirSim 日志停在 `Waiting for TCP connection on port 4560` 期间**传感器零数据**，桥自然也没数据）→ 一键脚本要**轮询等 `ss -tn | grep ':4560'` 建链**再启传感器桥；桥从启动到出数据约 **30s**，别 12s 就判失败；**别每次启动都无条件重启 PX4**（重启要多等几十秒）；启动前 **Blocks 必须在 Play**。
- **roslaunch 报 `run_id on parameter server does not match declared run_id: xxx vs yyy`** = 有**残留 roslaunch** 占着 master 的 `/run_id`（例如之前 `roslaunch mavros` 没退干净）→ 启动前清理**所有** roslaunch（不只目标那一个）再起干净 roscore。
- **"相机看得到障碍、LiDAR 点云却没有" 多半是 FOV 几何盲区，不是 bug**：MID360 垂直 FOV `-7°~52°`（向下仅 7°）⇒ 飞高 h 时**盲区半径 ≈ `h/tan(7°)`**（3m 高 → 24m，实测点云最近点 23.09m 完全吻合；落地 0.4m → 最近点 1.84m）。**移动无人机（前进 2m/40m）不改变盲区**。出路：飞低 / 放宽 `settings.json` 的 `VerticalFOVLower`（如 -7→-25，偏离实机）/ 给 LiDAR 加 `Pitch` 倾角；改 settings.json 需**重启 Blocks**。
- **ROS1 + AirSim 栈的 PX4 接 MAVROS 必过两关**：① 补 GeographicLib geoid（缺则 `[FATAL] GeographicLib exception: File not readable .../egm96-5.pgm` 启动即退，可从已装好的另一发行版直接拷 3 个文件）；② 另开专用 mavlink（默认 14540 被占 → `FCU: DeviceError:udp:bind: Address already in use`）：`pxh> mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard`，MAVROS `fcu_url:=udp://:14611@127.0.0.1:14610`。**offboard 脚本一停发 setpoint，PX4 超时即执行 AUTO.LAND**（脚本结束就降落，是安全行为非故障）。
- **FAST-LIVO2 的三个配置坑（实测）**：① **`img_en:0` 也必须 load 相机配置**（`rosparam load camera.yaml /laserMapping`），否则秒崩 `terminate ... Camera model not correctly specified`；② **XYZ-only 点云喂不进去**——`lidar_type:4`(l515 分支) 要 `pcl::PointXYZRGB`、velodyne/oust64/robosense 要 `intensity` ⇒ 改桥让点云带 `rgb` 字段（改完**必须重启桥**）；③ 实机 bag 的 `livox_ros_driver2/CustomMsg` 与 v1 **md5 相同**（`e4d6829b...`）⇒ 建一个"只含 msg"的最小 `livox_ros_driver2` 包即可直接回放 bag。详见 `references/ros1-twin-digital-twin-ops.md` §8。
- **别把 `roscore` 慢当成卡死**：这类机器上 roscore 可能要 **2 分钟以上**才就绪（与 PX4 的 `make` 抢 CPU；DNS 已排除：`getfqdn` 0.028s）⇒ **长等待循环必须打进度输出**，且**能复用 master 就别重启**（清理阶段只杀 `roslaunch` + 节点，保留 roscore；run_id 冲突的成因是残留 roslaunch）。
- **"RViz 打不开窗口" 不等于 rviz 坏了**：WSLg 把**所有** Linux 窗口渲染在单个 **`msrdc.exe`** 进程里 ⇒ `Get-Process | MainWindowTitle` **找不到 rviz 窗口**。用 `scripts/wslg-window-dump.ps1`（Win32 `EnumWindows`）枚举 msrdc 的窗口，常能看到窗口**其实全屏开着**只是被别的窗口盖住；用户说的 "warning" 往往另有一个 **ROS 1 End-of-Life 弹框**（Noetic 停服 2025-05-31 后 roscore 会弹）。`[WARN:COPY MODE]` 前缀 = WSLg 退化成软件合成，常见成因是 `~/.wslconfig` 指定了**自定义内核**（如 JetPack 的 vmlinux）⇒ 注释掉 `kernel=` 后 `wsl --shutdown` 重启。⚠️ 给 PowerShell 5.1 的 `.ps1` **只写英文**（UTF-8 无 BOM 的中文会被按 ANSI 读，报 `The string is missing the terminator`）。

> 网络/安装/PX4 接入的逐条命令与判读,见 `references/ros2-gazebo-px4-integration.md`。
> **PX4→ROS2 用 MAVROS(MAVLink) 的接入步骤 + gz-sim 简化四旋翼(MulticopterVelocityControl)模式,见 `references/px4-mavros-route.md`。**
> **AirSim(Windows Unreal) + WSL 算 ROS 的混合架构、UE4.27+VS2022(17) 硬要求、WSL 迁移到非 C 盘,见 `references/airsim-wsl-setup.md`。**
> **AirSim 三个高发坑(实测)**：① UE4Editor.exe 首次打开 Blocks 常闪退/无窗口——查 `Blocks\Saved\Logs\Blocks.log` 若反复 `Incompatible or missing module: Blocks/AirSim`，是模块没编，**手动跑 UBT `Build.bat ... BlocksEditor Win64 Development -Project=...\Blocks.uproject`** 编编辑器模块(生成 `UE4Editor-AirSim.dll`);② UBT/Build.bat 报 `Could not find NetFxSDK` 是缺 .NET Framework SDK → VS 勾 **".NET 桌面开发"**;③ `--no-full-poly-car` 让 Blocks 报一堆 `加载失败 /AirSim/VehicleAdv/SUV/...M_Caliper/M_Leather`,需 `curl` 下 `car_assets.zip` 解压到 `Unreal\Plugins\AirSim\Content\VehicleAdv` 再重启 UE。
> **AirSim spawn 无人机的头号坑(实测)——`VehicleType` 值**：自定义 `Vehicles` 时 `VehicleType` 必须是 AirSim 认识的**车辆类型**常量(`"SimpleFlight"` / `"PX4"`，源码 `kVehicleTypeSimpleFlight="simpleflight"`)，**不是机架**「QuadX」——用 `QuadX` 会报 **`There were no compatible vehicles created for current SimMode!`** + 连带 **`SimHUD getCamera() 空指针 EXCEPTION_ACCESS_VIOLATION @0xb8`** 崩溃。**`"no compatible vehicles"` + getCamera 崩溃不是两个问题，是同一个**：settings 写 `Vehicles` 会 `vehicles.clear()` 后走 `pawn_paths.at(getVehiclePawnPathName(vehicle))`，VehicleType 不对 → `.at("")` 抛异常 → 无 pawn → HUD 拿相机才崩。**最小配置 `{"SimMode":"Multirotor"}`(不写 Vehicles) 自动创建默认无人机**，验证平台通不通最省事；spawn 后**别再加 `Cameras` 或错误的 VehicleType**(会重新触发)。
> **AirSim 接 PX4 SITL 的完整配方(实测)**：PX4 必须 **v1.11.x**(主分支太新不兼容)；启动用 **`make px4_sitl_default none_iris`**(posix，等 TCP 4560，`make px4_sitl` 不带模型只编译不启动)；AirSim settings 用 `PX4Multirotor + UseTcp:true + TcpPort:4560 + ControlPort 14540/14580 + Barometer + Parameters(NAV_RCL_ACT:0/COM_OBL_ACT:1/LPE_LAT,LON)`；跨系统靠镜像网络 + 防火墙开 TCP 4560/UDP 14540。**PX4 v1.11.3 必须用 GCC 10**(GCC 13 拒 `math::max(long,int)`)，`CC=gcc-10 CXX=g++-10` 须先 `rm -rf build/px4_sitl_default`(否则 CMake 缓存记住 /usr/bin/c++=gcc-13)，且要改 `WorkQueueManager.cpp:257` 的 `math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(...))` 为 `math::max(static_cast<size_t>(..), static_cast<size_t>(..))`(别用 `std::max`，缺 `<algorithm>`)。**完整步骤+逐条命令见 `references/airsim-px4-integration.md`。**
> **ROS1(ROS Noetic, WSL `ros1-twin`) 侧数字孪生链路**：AirSim+PX4v1.11.3+`twin_bridge`+FAST-LIVO2+MAVROS+RViz 的**一键启动时序**(LockStep 等 4560 / 等 /twin 出数据 / roslaunch run_id 冲突)、**MAVROS geoid+专用 mavlink 14610/14611**、**offboard 位置+yaw setpoint 控制脚本骨架与判读**（含 offboard 超时→AUTO.LAND）、**LiDAR FOV 盲区几何**、交付物清单、**FAST-LIVO2 两套数据源配置坑（§8：cam_model / rgb 字段 / Livox md5 借壳）**、**WSLg 窗口排查（§9）**，见 `references/ros1-twin-digital-twin-ops.md`；WSLg 窗口枚举脚本 `scripts/wslg-window-dump.ps1`；**链路体检探针 `scripts/check-sim-health.sh`（用户报"点云时有时无/没数据"时第一件事就跑它，一条 `/clock` 判出仿真是否冻结）**。

> **要做/要学"无人机集群控制 / 多智能体协同控制"(swarm / multi-agent cooperative control)时**：集群控制 4 大主题知识地图、集群仿真 4 级台阶(本项目已有 AirSim 多机基础,是这里最低摩擦的入口)、以及教材《集群系统智能协同控制》(董希旺/化永朝/任章, 清华社, ISBN 9787302723103)的 7 模块结构 + 理论↔实践配平表,见 `references/swarm-control-learning-path.md`。

#### 工程化与验证（ROS 工作空间，实测）

- **★ 在项目根建 `Makefile` 的 `test`/`lint` 入口 —— 这是"每轮都被要求验证"的正解**：Hermes 的验证机制靠**发现 canonical 命令**（Makefile / package.json 这类）判定"已验证"。ROS 工作空间 / 纯 shell 项目默认**没有**这类入口 ⇒ 每轮都提示 unverified；而在 `Temp` 下自建 `hermes-verify-*.sh` 反而**让该脚本本身成为新的 changed path**，形成"创建→验证→再验证"死循环（保留或删除都一样，因为建/删都算改动）。**修法：项目根放 `Makefile`**（`test:` 跑冒烟测试、`lint:` 跑语法检查）→ harness 立刻识别并返回 `canonical_command: make lint` 判 PASS。
  - `lint` 用 `python3 -c "import ast,glob;[ast.parse(open(f).read()) for f in glob.glob('scripts/*.py')]"`，**别用 `python3 -m py_compile`**（后者每次留 `__pycache__`，跑完工作区就脏，还得再清一次）。
  - 兜底手段（项目不适合建 Makefile 时）：验证脚本**经 base64 传入、在 WSL 内 `/tmp` 临时落盘执行后删除**，Windows 侧零新增文件，顺带绕开 PowerShell 引号地狱：`[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($s))` → `wsl ... bash -c "echo <b64> | base64 -d > /tmp/x.sh; bash /tmp/x.sh; rm -f /tmp/x.sh"`。
- **`catkin_install_python` 在 `devel/` 放的是 relay wrapper，不是脚本副本**：`devel/lib/<pkg>/<script>.py` 只有约 20 行 `exec(compile(open('<src 绝对路径>').read()))` ⇒ **grep 业务标识（如新加的函数名/参数名）会得 0 命中**，极易误判成"devel 里是旧版、新逻辑没生效"（本轮就据此走错一轮）。**正确判据**：`head` 看它是不是 wrapper（含 `generated from catkin/cmake/template/script.py.in`）+ 看 `src` 那份的 mtime；**别用 grep 计数判版本**。
- **`<arg>` 必须"先定义后引用"**：把 `<arg name="init_z" default="$(arg seed_z)"/>` 写在 `seed_z` 定义**之前**，roslaunch 会直接报错 ⇒ 把被引用的 arg **上移到文件顶部**。改完用 `roslaunch <pkg> <file> --dry-run` 真解析一次 —— 它比 XML 语法检查更能抓这类错（XML 合法但 arg 前向引用非法）。
- **各节点日志在 `~/.ros/log/<uuid>/<node>-N.log`，不在 roslaunch 的 stdout 汇总里**：让用户 `grep /tmp/xxx.log` 找某个节点的打印会**查不到而误判"没生效"**（本轮就因此白查一轮）⇒ 诊断提示一律给 `~/.ros/log/latest/<node>*.log`。
- **`grep` 检查内容时，以 `--` 开头的关键字必须用 `-e`**：`grep -q "--no-save" file` 会被当成选项、报 `unrecognized option` ⇒ 写 `grep -q -e "--no-save"`（同理 `--all` / `--keep-px4`）。
- **写桥的冒烟测试有"单实例测不全"的硬约束**：控制桥一旦收到任何 FUEL 指令就把 `last_cmd` 永久置位、**再也回不到 seed 悬停分支** ⇒「seed 悬停」「未对齐时拒绝转发」「对齐后按朝向差旋转」**三者不能共用一个桥实例**。做法：**起两个实例**（`~align_samples:=1000` 让对齐永不完成 → 测 seed + 拒绝；`~align_samples:=5` → 测对齐 + 旋转），各自独立 master/端口。断言前还要**等异步状态生效**（先发几条指令 + `sleep 1` 再清计数），否则测到的是上一阶段的残留。
- **冒烟测试要防 flaky，否则会把"环境残留"误读成"代码坏了"**：测试**开头先清残留**（`pkill -f <各节点>` + `pkill -f "rosmaster --core -p <各测试端口>"` + `sleep 1`）—— 本轮就出现过 `make test` 一次 FAILED、紧接着重跑 ALL PASS，真因是上一轮的进程还活着。**加固后连跑 3 次全 PASS 才算稳**；遇到偶发 FAIL 先怀疑残留/时序，再怀疑代码。
- **测试里判定浮点不要用字符串相等**：`pos_cmd.z` 实测 `1.51`（期望 `1.5`）⇒ 用 `awk` 容差比较；抽数字用 `tr -dc '0-9.-'`。
- **`~xxx` 私有参数与 launch 的对应关系**：`<node name="traj_server">` 内写 `<param name="traj_server/init_z">` ⇒ 完整路径是 **`/traj_server/traj_server/init_z`**（node 名 + param 名叠加）；而节点用 `nh("~")` 读同名相对名正好命中。查 `<node>/<param>` 会"查不到"而误判。
- 现成可抄：`templates/ros-bridge-smoke-test.sh`（双实例冒烟测试骨架）、`references/ros-verification-and-testing.md`（Makefile / 验证阶梯 / 各坑细节）。

#### 交付风格（本用户，实测偏好）

- **操作步骤/启停说明要直接贴在对话框里**（原话："…直接在对话框告诉我"）——**别只给 `.md` 文件路径让用户自己开**。写文档照旧，但正文里必须把可照抄的命令与预期输出贴全。
- 启动/停止一律**收敛成单脚本 + 开关**（`fuel_all.sh [参数]` / `fuel_stop.sh [--all|--no-save|--keep-px4]`），别堆一堆脚本；启停要**秒级**（常驻服务如 ROS master 宁可留着，也别每次重启等分钟级）。
- **改动要透明**：改代码逐处 `patch` 并贴 diff，Windows 侧改完**必须同步到 WSL 并 md5 比对**。
- **交付参数化脚本时，参数含义要当场列成表**：用户实测会直接问「`fuel_all.sh 1.5 position 3.0 0` 这些参数是什么意思」—— 别只把说明写在脚本头部注释里，正文给「一行完整示例 + 每参数含义 + 默认值 + 只给部分参数时其余取默认」，并说明改动某参数的**实际影响**（例：改 `seed_z` 会同时决定起飞高度与 FUEL 初始轨迹高度）。
- **长排查不要卡在"等"**：任何等待循环都要**打进度 + 有兜底 + 失败时给具体诊断**（例：等起飞同时打印 `高度/armed/mode`，超时列出 3 类常见原因），不干等、不无限排查。
- **给用户照抄的诊断命令要自带 `source`**：只 `source /opt/ros/noetic/setup.bash` 会让 `rostopic echo /planning/pos_cmd` 报 **`ERROR: Cannot load message class for [quadrotor_msgs/PositionCommand]. Are your messages built?`**（自定义 msg 在另一个工作空间）—— 命令里把 `source ~/twin_ws/devel/setup.bash`、`~/fuel_ws/devel/setup.bash` 一并写上，别让用户自己猜（用户会以为是坏的）。


---

### 来源技能 `uav-ros2-sim-setup`

> 在 WSL(Linux) 从零搭建 ROS2+Gazebo+PX4 无人机探索仿真环境（含国内网络/代理、MAVROS、LiDAR world、论文主流选型）。用户常用。

在 WSL (Ubuntu) 搭建 ROS2 Jazzy + Gazebo Harmonic + PX4 SITL + MAVROS 的无人机探索仿真平台，供自主探索算法（frontier 等）开发 / 发论文。

#### 环境底座（本会话实测栈）

- WSL2 Ubuntu 24.04 (`lyuav`)；ROS2 Jazzy；Gazebo Harmonic (gz-sim8)；PX4-Autopilot (SITL `gz_x500`)；MAVROS
- 探索算法放 `~/ros2_ws/src/<pkg>`（colcon）；用户 GitHub 仓库 `strongthenly/-lyuav`
- 关键连通：`/mavros/state.connected: true` = PX4↔ROS2 打通

#### 关键 Pitfalls（最易踩，务必记住）

1. **WSL2 默认 NAT 连不到 Windows 上仅监听 127.0.0.1 的 Clash(7890)**。解决：`~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored`，`wsl --shutdown` 重启。之后 WSL 共享 Windows 网络栈，`127.0.0.1:7890` 可达。
2. **国内无 ROS2 apt 镜像**（清华/阿里/USTC/华为/腾讯全 404）。官方 `packages.ros.org` 直连不通（OSU 机房被限）→ 必须经 Clash 代理。
3. **ROS2 源 `dists/` 子目录用 Ubuntu 代号（24.04=noble）而非 ROS2 发行版名（jazzy）**。正确 URL `http://packages.ros.org/ros2/ubuntu/dists/noble/Release`。"代理通但 404"的经典坑就是 dists 名用错。
4. **apt 走代理**：`/etc/apt/apt.conf.d/proxy` 写两行 `Acquire::http::Proxy` / `https::Proxy` = `http://127.0.0.1:7890`。
5. **`.wslconfig` 的 `kernel=` 路径用正斜杠**（`C:/ProgramData/...`），避免反斜杠转义报错；改前备份 `.bak`。
6. **MAVROS 启动崩溃 → 缺 GeographicLib geoid**：`sudo apt install -y geographiclib-tools` + `sudo geographiclib-get-geoids egm96-5`。
7. **PX4 ↔ ROS2 通信：用 MAVROS(MAVLink) 而非 uXRCE**。Jazzy 只有 Fast RTPS 2，uXRCE 的 micro_ros_agent 强制 Fast DDS 3 → `ros-rolling-fastdds` 与 fastrtps2 的 `eProsima_atomic` target 冲突（死结）。MAVROS 走 MAVLink 天然兼容。验证用 `/mavros/state.connected=true`。
8. **gz-sim `MulticopterVelocityControl` 必须先发 `enable`**（`gz topic -t .../command/enable -m gz.msgs.Boolean -p "data: true"`）才接管，否则四旋翼不动/不稳。launch 里可用 `ExecuteProcess`+`TimerAction` 自动 enable。
9. **Gazebo LiDAR 几何**：LiDAR 悬停高度+垂直视野 vs 障碍高度不匹配 → `ranges` 全 `.inf`。四旋翼 z=2m、垂直 ±0.3rad 时，墙须 >1.07m 才进视锥。加高墙/调视野/降高度。
10. **WSL 里 `libEGL ... /dev/dri Permission denied`** 常见：不影响 gz server/传感器数据，只影响 GUI 渲染（可用 headless `gz sim -s` 跑数据）。不是 bug，别当 bug 修。
11. **PX4 的 gz_x500 不带 3D LiDAR**（只有相机/rangefinder）。要 LiDAR 须改 PX4 `Tools/simulation/gz/models/x500_base/model.sdf`（加 gpu_lidar sensor）。改 PX4 文件前备份+贴 diff。

12. **MAVROS 启动命令**：`ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`——**launch 文件是 `px4.launch` 不是 `px4.launch.py`**（mavros 包只有 `px4.launch`，用 .py 会报 "file not found in the share directory of package 'mavros'"）。连不上时看 PX4 日志 `INFO [mavlink] ... on udp port X remote port Y` 精确配 fcu_url。
13. **PX4 SITL "gazebo already running world"**：若已有一个 gz sim 在跑（比如先启动了 exploration launch / 之前残留的 gz），`make px4_sitl gz_x500` 会**复用**那个 world 并把 x500_0 塞进去，**不开自己的窗口**（"gazebo没弹出来"）。要独立窗口先 `pkill -f "gz sim"`（彻底的再 `pkill -9 -f "gz sim"`）再重跑。
14. **`ros2 run <pkg> <node>` 报 "Package not found"**：source 错了 workspace 或在别的 ws 下。`exploration` 在 `~/ros2_ws`，先 `cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash` 再 run；别在 `px4_ros2_ws` 等别的 ws 下找它。改过 world/launch 后要 `colcon build --symlink-install`（symlink 对 data_files 不生效，仍要 rebuild 才更新 install/share）。

#### AirSim 与 WSL 共存架构（用户明确想"WSL 操作 + Windows 端 AirSim"）

- AirSim 必须 Unreal Engine → **只能 Windows 本机跑，WSL 跑不了 Unreal GUI**。常见架构 = WSL 算 ROS/算法 + Windows 跑 Unreal/AirSim 高保真仿真。
- 靠 `networkingMode=mirrored`（WSL 与 Windows 网络栈共享），**WSL 里 `127.0.0.1:<AirSim RPC 41451>` 可直接连 Windows 的 AirSim**——这是协同命门，恰好前面已配镜像模式支持。
- AirSim 官方 ROS bridge（`airsim_ros_pkgs` 或 ROS2 版）跑在 WSL，通过 RPC 连 Windows AirSim，把 AirSim 的 LiDAR/相机/位姿发布成 ROS 话题、也收 ROS 命令控制车辆。PX4 留 WSL（MAVLink 指挥 AirSim 无人机）。

#### AirSim 实际安装（Windows 侧）要点

- **UE4.27 只支持 VS 2019/2022（16/17 工具集），不支持 VS 18（2026）**。误装 VS18 会导致 AirSim/UE4.27 编译不兼容。装 VS 时选 **"使用 C++ 的桌面开发" + Windows SDK**（18 版留着不影响，但 build 必须用 VS2022 17）。定位 VS 用 `vswhere -all -products * -property installationPath`（别 `&` 调用，会被工具误判为后台——写成 .cmd 用 `cmd /c` 跑，且 .cmd 里**只用英文**避免 GBK 乱码）。
- **AirSim clone 用默认分支**：`git clone https://github.com/microsoft/AirSim.git E:\AirSim`（`--branch master` 会报 "Remote branch master not found"）。
- **AirSim `build.cmd` 构建的是 C++ 库（AirLib/rpclib/MavLinkCom），不需要 UE 路径**（UE 只用于后续打开 Unreal 项目）。但**必须从 "x64 Native Tools Command Prompt for VS 2022" 运行**（需要 `%VisualStudioVersion%`），否则报 "run from x64 Native Tools Command Prompt for VS 2022"。可用脚本 `call vcvarsall.bat x64 && cd /d E:\AirSim && build.cmd` 后台跑。
- **build.cmd 用 iwr 下载 rpclib/eigen（直连 GitHub/GitLab），国内直连 GitHub 时通时断**（git clone 成功但 iwr 卡死）→ 先 `curl.exe -L` 手动预放依赖到 `E:\AirSim\external\rpclib\rpclib-2.3.0\` 与 `E:\AirSim\AirLib\deps\eigen3\`，再 `build.cmd --no-full-poly-car`（跳过 ~37MB SUV 资产下载）。完整配方见 `references/airsim-install-windows.md`。
- **UE 打开 Blocks 的硬坑**（详见 reference）：① UE4.27 编辑器 exe 是 `UE4Editor.exe`（不是 UE5 的 `UnrealEditor.exe`）；② 首次打开若 UE 闪退（无窗口/无进程），模块会缺失（`Incompatible or missing module: Blocks / AirSim`）→ **手动跑 `Build.bat BlocksEditor Win64 Development -Project="...Blocks.uproject" -WaitMutex`** 编译；③ UBT 报 `Could not find NetFxSDK` → **VS 加 ".NET 桌面开发"工作负载**；④ `dinput8.lib/dxguid.lib ... not resolvable` 是警告非致命；⑤ UE 在 E 盘时 `update_from_git.bat` 报注册表错误（rungenproj）非致命。

#### 用户偏好 / 环境事实（本类任务）

- **存储放 E 盘，不占 C 盘**：大文件 / Unreal / AirSim 装 E 盘；WSL 发行版若占满 C 盘，可 `wsl --export` + `wsl --unregister` + `wsl --import` 迁到 E 盘（迁移配方见 `references/wsl-migration-and-airsim.md`）。

#### 论文主流平台（选型依据，用户发论文参考）

- 主流无人机探索（FUEL 1471★ / RACER 789★ / EPIC 236★）用 **Gazebo + 简化四旋翼或 PX4 + MAVROS + FAST-LIO + frontier**，**不用 AirSim**。
- AirSim(Unreal) 只在要高保真视觉时用（需 Unreal：Windows、几十 GB、好 GPU），**非探索主流**。
- 用户环境 → `Gazebo+PX4+MAVROS` 是论文主流姿势。

#### 用户偏好（本类任务）

- 用户**坚持无人机**（拒用车/小车替代），要**真实外观机型**（非光杆 box），要**能随意介入算法**（探索算法做成独立可改节点）。
- 指挥官模式：默认远程命令用户亲手跑，agent 给命令+判读；说"你来跑/帮我做"才 agent 执行。每次 1-3 条 + 预期输出。
- 改文件先讲方案+备份+贴 diff+可回滚；讲原理用大白话+类比。
- 反复失败/高强度协作后用户会说"今天先这样"收工——停止推进，简洁总结进度并落记忆。


---

### 来源技能 `wsl2-ros2-setup`

> 在 Windows WSL2 上搭建 ROS2/Gazebo 无人机仿真环境——尤其国内网络 + Clash 代理场景。核心坑：WSL2 默认 NAT 模式够不到 Windows 上只监听 127.0.0.1 的 Clash 端口（用 networkingMode=mirrored 修复）；ROS2 apt 仓库 dists/ 子目录用 Ubuntu 代号(24.04=noble)而非 ROS 版本名(jazzy)；国内无 ROS2 apt 镜像，官方 packages.ros.org 直连不通(OSU 机房受限)须经代理。也涵盖在 Linux/Ubuntu 装 Hermes。

**三大关键知识点（本技能的灵魂，实测验证过）**

#### 1. WSL2 够不到 Windows 宿主上的 Clash 代理 —— 用 mirror 模式

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

#### 2. ROS2 apt 仓库 dists/ 用 Ubuntu 代号，不是 ROS 版本名

- **坑**：`http://packages.ros.org/ros2/ubuntu/dists/jazzy/Release` 返回 404 —— 因为 `dists/` 下面的子目录是 **Ubuntu 代号**，不是 ROS2 发行版名。
- **正确**：Ubuntu 24.04 → `.../dists/noble/Release`；22.04 → `jammy`；20.04 → `focal`。
- apt 源行 `$(. /etc/os-release && echo $UBUNTU_CODENAME)` 就是取这个代号。

#### 3. 国内直连 ROS2 官方源不通，也没有国内 ROS2 apt 镜像

- **实测**：清华 / 阿里云 / 中科大 / 华为云 / 腾讯云 的 `/ros2/ubuntu/dists/noble/Release` **全部 404** —— 国内主流镜像**没有** ROS2 Debian 仓库。
- **官方源** `packages.ros.org` 解析到 `ftp.osuosl.org`（美国俄勒冈州立大学 OSL 机房）。DNS 没被污染（系统解析与阿里 DoH 结果一致），但**国内直连这些 IP 的路由被限/超时**。
- **解法**：走 Clash 代理。镜像模式后，apt 配代理指向 `127.0.0.1:7890`。
- **判节点质量（装大包的关键判据）**：看 `apt` 装包时的 `Fetched X kB in Ys (Z kB/s)` 那行。**Z ≥ ~500 kB/s = 节点健康**，800MB 的 `ros-jazzy-desktop` 能顺利下完；**Z 掉到十几 kB/s（如 17 kB/s）或反复 `502 Bad Gateway [IP: 127.0.0.1 7890]` = 节点到 OSL 机房拉不动大文件**。此时**重启 Clash / 换一个下载速度快、不抖的节点**（选"专用/中转"类，别只看 ping 延迟），别死磕 apt。换好后**先装小包试水**（`ros-jazzy-ament-cmake` 通了再上大的），全程 `-o Acquire::Retries=8` 容错零星 502。

#### 连通性诊断探针（先测再装，别盲装）

在 WSL 里跑（PowerShell 宿主侧用 `wsl.exe -d <distro> -- bash /mnt/c/~/probe.sh`，**避免多层引号被 PowerShell 破坏**——多行复杂命令建议写成脚本文件，用 `wsl -d <distro> -- bash /mnt/c/路径/脚本.sh` 执行，不要用 `bash -lc "多行"`）：

1. **确认代理链路是否正常**：经 Clash 测 google/bing/example 应 **200**；若全 200 说明代理健康，问题在目标域名规则。
2. **看目标域名 404 是不是路径问题**：`curl -sL` 读 **响应体**。若 body 是 `<address>Apache Server at packages.ros.org Port 80</address>` → **真实服务器到了，只是 URL 错**（多半是 dists 用了 jazzy 而非 noble），不是网络问题。
3. **区分 HEAD 与 GET**：`-sIL`（HEAD）某些服务器行为不同，用 `-sL -w "%{http_code} %{size_download}B"` 的 GET 更准。
4. **测真实包索引进而不只是 Release**：`.../dists/noble/main/binary-amd64/Packages.gz` 若 200 且几 MB，说明仓库真可用。

#### apt 走代理配置（镜像模式后）

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

#### 在 Linux/Ubuntu 装 Hermes Agent

- **Ubuntu 20.04 可装**：`curl -fsSL https://hermes-agent.nousresearch.com/install.sh | bash`。安装脚本明确分支 `ubuntu|debian`。
- Hermes 要求 **Python ≥3.11,<3.14**；20.04 默认 3.8，但脚本用 **uv 自动装 Python 3.11** 绕开。Node 需要 22.22+/24.11+/26+，脚本自动装 Hermes-managed Node。
- 前置：联网 + `sudo`（仅 apt 装构建工具 `build-essential python3-dev libffi-dev`），Hermes 本身不占 root。
- ⚠️ Hermes **桌面版(Electron)** 对 glibc 有要求（20.04 是 2.31 较老）；**CLI/TUI 版绝对没问题**，优先 CLI。

#### 装好 ROS2 源后的后续搭建（Gazebo 本体 / 工作空间 / 感知桥 / IDE / 推送）

前几节解决"源 + 网络"，这里是**装好源之后**的标准步骤（本技能实测通过）。

#### 4. Gazebo Harmonic 走 OSRF 源（绕开 502 的 packages.ros.org）

- `gz-harmonic` 的 deb 在 `packages.osrfoundation.org/gazebo/ubuntu-stable`（dists=**noble**），**不是** packages.ros.org → 即使 packages.ros.org 反复 502，Gazebo 也能正常装。
- GPG key 在**根目录** `https://packages.osrfoundation.org/gazebo.gpg`（别用子目录如 `/gazebo/gazebo.gpg`，会 404）。
- 完整源配置脚本见 `references/ros2-poststeps-detail.md`。

#### 5. colcon 工作空间 + ros_gz_bridge（感知桥）

- ROS2 用 **colcon** 不是 catkin：`~/ros2_ws/src` + `colcon build --symlink-install`。
- ⚠️ **`ros2 --version` 会报 unrecognized**——别当错误；用 `echo $ROS_DISTRO` + `ros2 pkg list | wc -l` 确认装好。
- **每个新终端必须 `source /opt/ros/jazzy/setup.bash`**（写进 `~/.bashrc` 一劳永逸；否则 `ros2: command not found`）。
- `sudo apt install ros-jazzy-ros-gz` 装感知桥，之后 `/opt/ros/jazzy/lib/ros_gz_bridge/` 有 `parameter_bridge / static_bridge / bridge_node`。
- 主题映射模板（gz `/lidar` → ROS `/scan`）见 `templates/bridge.yaml`；ament_python 包骨架要点见 `references/ros2-poststeps-detail.md`。

#### 5.5 感知桥调试 + gpu_lidar 全 `.inf` 的几何坑、libEGL 警告（实测）

**先分清"gz 传感器没发"还是"bridge 没接上"**：
- 看 **gz 侧**：`gz topic -l | grep lidar`（出现 `/lidar`、`/lidar/points` = 传感器在工作，问题在 bridge）。查类型用 `gz topic -i -t /lidar`（`-l/-i/-e` 是子命令，`-t` 只给名字，别单独 `-t`）。
- 看 **ROS 侧**：`ros2 topic list` 里有没有 `/scan`（有时因时序 `grep scan` 一时见不到、隔几秒就出现，别慌）。
- 若 gz 有、launch 日志也打 `Creating GZ->ROS Bridge: [/lidar -> /scan]`，但 `/scan` 无数据 → 多半 `bridge.yaml` 的 `gz_type_name` 没对上。关键字段：`gz_topic_name`、`gz_type_name`（如 `gz.msgs.LaserScan`）、`ros_type_name`（`sensor_msgs/msg/LaserScan`）、`direction: GZ_TO_ROS`。3D 点云桥 `/lidar/points`（`gz.msgs.PointCloudPacked` → `sensor_msgs/msg/PointCloud2`）。

**gpu_lidar ranges 全 `.inf`**：`.inf` = 光线射出没碰到东西；`0` = 传感器未就绪——**别混淆**。`frame_id/angle_min/max/range_max/beam数` 都正常但 ranges 全 `.inf` → **纯几何问题，不是 GPU/桥接**。
- 判据：LiDAR 在高度 `z`、垂直半角 `vmax`，水平距离 `d` 处垂直覆盖 `[z-d·tan(vmax), z+d·tan(vmax)]`；障碍物（墙高 `h`、中心 `zc`，即 z∈[zc-h/2,zc+h/2]）落在此区间外 → 一根都扫不到 → 全 `.inf`。
- 实例：四旋翼 z=2m、vmax=±0.3rad、墙 1m 高（z 0~1）→ 覆盖 `[1.07,2.93]` 与墙不相交 → 全 `.inf`。**修法：加高墙（4m）/ 增大垂直视野 / 降飞行高度**。
- 改 `.sdf` 后**必须 `colcon build`**（`--symlink-install` 不自动同步 data_files 的 worlds/launch，需重新 copy 进 install）再重启 launch。

**libEGL `/dev/dri Permission denied` 在 WSL 是非致命警告，别当根因**：Gazebo 在 WSL 走 WSLg(Mesa/D3D12)，试图原生开 `/dev/dri/renderD128|card0` 被拒是**正常**的 `libEGL warning`；**不影响传感器出点云**，只让 GUI 退到软渲染（`gz sim gui` 吃高 CPU）。用户已在 `video`/`render` 组 + `nvidia-smi` 可见即 GPU 直通 OK——**别看到这串 warning 就去折腾 GPU 权限，先确认 `/scan` 有没有数据**。

#### 6. VSCode 连 WSL 的"连错环境"坑

- VSCode 的 WSL 扩展**默认连"默认发行版"**。若 ROS2 装在非默认 distro（如默认是 26.04、ROS2 在 24.04 的 `lyuav`），会打开一个空环境，看起来"连不上/连错"。
- 修复：PowerShell 里 `wsl --set-default <distro>`，或 VSCode `F1 → WSL: Connect to WSL` 选目标，或在目标 distro 终端里 `code .`。

#### 7. GitHub 推送私有仓库

- 私有仓库 HTTPS push：`Password:` 处填 **PAT**（不是登录密码；GitHub 已废弃密码认证，报 `Password authentication is not supported` 就是填错了）。
- 免输：`git config --global credential.helper store`。
- 远程非空（GitHub 建库时自动初始化 README 等）→ push 报 `fetch first` 被拒：确认远程只是占位后 `git push --force`。

#### 8. PX4 SITL 接入（B 路线：真实飞控 + ROS2-Gazebo）

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

#### 9. 无人机自主探索的主流做法：**不用 PX4，用简化四旋翼 + 直接发指令**（关键决策）

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

```
> 相关话题都带模型名前缀：指令 `<ns>/gazebo/command/twist`、odom `/model/<name>/odometry`、pose `/model/<name>/pose`。bridge 的 `gz_topic_name` 必须写全 `<ns>/gazebo/command/twist`（不是裸 `/cmd_vel`）。

**结论**：用户要"无人机+随意介入算法" → 走**简化四旋翼（`MulticopterVelocityControl`+`MulticopterMotorModel`）+ 3D LiDAR + frontier 探索节点（可随意改）**，**不要**在 PX4-ROS2 上烧时间（环境级死结见上节 8）。

#### 参考脚本

- `references/session-scripts.md` —— 连通性诊断探针脚本 + 源配置脚本，可直接复制。
- `templates/bridge.yaml` —— ros_gz_bridge 主题映射模板（gz `/lidar` → ROS `/scan`）。
- `references/ros2-poststeps-detail.md` —— 装好源后：Gazebo 本体安装、ament_python 包骨架、launch/bridge 细节。
- `references/px4-ros2-and-mainstream.md` —— PX4-ROS2 接入的仓库名坑、Jazzy 上 fastdds3/fastrtps2 冲突证据、主流 UAV 探索做法（FUEL/RACER 简化四旋翼）、ROS2 简化四旋翼 MulticopterControl SDF 结构。


---

### 来源技能 `ros2-gazebo-sim-setup`

> 在 Windows+WSL2 上搭建 ROS2+Gazebo 无人机仿真环境（自主探索/导航算法研究）。覆盖系统选型、WSL2 镜像模式、国内网络经 Clash 代理、ROS2/Gazebo apt 源坑、VSCode 连接发行版、source 环境这些实操细节。

#### 系统/版本选型（先问，别默认）

做**自主探索论文、纯仿真为主** → 用最新栈：**Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic**（Jazzy 官方配 24.04，Harmonic 也支持 24.04）。
- 若算法必须下放到**现有实机**（如 Jetson + ROS1 Noetic）→ 保持同栈（ROS1），否则两套栈迁移工作量巨大。**这一点必须先跟用户确认**，不要一股脑推 ROS2。

#### 安装步骤（顺序）

1. **WSL2 发行版**：`wsl --install -d Ubuntu-24.04 --name <name>`。别用默认 26.04（太新，ROS/Gazebo 生态适配不稳）。
2. **WSL2 镜像网络模式**（关键，见下）：`~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored`，然后 `wsl --shutdown`。
3. **apt 走 Clash 代理**：写 `/etc/apt/apt.conf.d/proxy`（`Acquire::http/::https::Proxy "http://127.0.0.1:7890"`）。
4. **加 ROS2 apt 源** + **Gazebo 源**（见 Pitfalls 里的关键坑）。
5. `sudo apt update` 验证两源都通。
6. 装 `ros-jazzy-desktop`（量大，走代理需耐心，`-o Acquire::Retries=8` 容错全通）。
7. **VSCode 连接**：装 WSL 扩展，`wsl --set-default <你的ROS发行版>` 保证连对，或 `WSL: Connect to WSL` 选对。

**关键坑（每条都踩过，务必绕开）**

#### WSL2 连不上 Windows 上的 Clash 代理

**症状**：WSL 里 `curl` 到 `packages.ros.org` 超时/000，而 Windows 宿主能连。
**根因**：WSL2 默认 **NAT 模式**，流量走独立虚拟网卡 + NAT，**不经过 Windows 上仅监听 `127.0.0.1` 的 Clash 代理**（Clash 默认只 bind 127.0.0.1，WSL 的网关 IP 连不到它）。
**修复**：`~/.wslconfig` 的 `[wsl2]` 加 `networkingMode=mirrored`，`wsl --shutdown` 重启。镜像模式让 WSL 共享 Windows 网络栈，WSL 内 `127.0.0.1:7890` 即 Windows 的 Clash 端口。
**验证**：WSL 内 `bash -lc 'bash /dev/tcp... '` 或 `curl -x http://127.0.0.1:7890` 测 `packages.ros.org` 返回非 000。
**验证特征**：`ip route` 不再是独立 `172.25.x` 网段（镜像后网关变 `172.27.x` / 共享 Windows 地址）。

#### ROS2 apt 源：`dists/` 用 Ubuntu 代号，不是 ROS 发行版名

**症状**：`http://packages.ros.org/ros2/ubuntu/dists/jazzy/Release` 返回 **404**（明明路径该对）。
**根因**：ROS2 仓库的 `dists/` 子目录是 **Ubuntu codename**（24.04=`noble`），**不是** ROS 发行版名（`jazzy`）。正确：`dists/noble/Release`。
**正确源行**：`deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main`（`noble` 替换你的 Ubuntu 代号）。

#### 大陆无 ROS2 apt 镜像 + 官方源不稳

- **国内镜像（清华/阿里/USTC/华为/腾讯）的 `ros2/ubuntu` 全部 404** —— 大陆基本没有 ROS2 二进制镜像。
- `packages.ros.org` 托管在 OSL 机房（`ftp.osuosl.org`），大陆直连不通；走 Clash 时**小文件（元数据）能下，大包（deb）会 502 Bad Gateway** —— 这是 **Clash 节点到 OSL 机房拉不动大文件**，不是网络/ROS 问题。
- **绕开方案**：Gazebo 本体走 **OSRF 源 `packages.osrfoundation.org/gazebo/ubuntu-stable`**（这个走代理 200，且与 packages.ros.org 无关）；ROS2 主包用 `Acquire::Retries` 容错反复拉。
- `502 Bad Gateway [IP: 127.0.0.1 7890]` = Clash 节点问题。**重启 Clash（切健康节点）通常能救**；屡试仍 502 则果断换路（如 RoboStack/conda 走清华镜像）。

#### Gazebo 源 & key

- 源：`https://packages.osrfoundation.org/gazebo/ubuntu-stable`（`dists/noble/Release` 走代理 200）。
- key：`https://packages.osrfoundation.org/gazebo.gpg`（**根目录**，不是 `gazebo/repos/gazebo.gpg` 等子路径，那些 000/404）。
- 装：`sudo apt install -y gz-harmonic`。ROS2-Gazebo 桥 `ros-jazzy-ros-gz` 在 packages.ros.org（可能 502，装 Gazebo 本体后再按需处理）。

#### `ros2: command not found`

**两个原因，先分清**：
1. **没 source 环境** —— 每个新终端都要 `source /opt/ros/jazzy/setup.bash`；一劳永逸写进 `~/.bashrc`：`echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc`。
2. **desktop 根本没装完**（此前只装上 `ament-cmake` 测试小子集）—— source 后仍 `not found` 时查 `ls /opt/ros/jazzy/bin` 有没有 `ros2`、`dpkg -l | grep ros-jazzy-desktop` 是否 `ii`。没有 = 重跑 `sudo apt install -y ros-jazzy-desktop -o Acquire::Retries=8`，**别中断**（之前 Ctrl+C 会只装一半，`/opt/ros/jazzy` 只有 `setup.bash`、无 `bin/`）。

#### VSCode 连接错了发行版

多个 WSL 发行版时，VSCode WSL 扩展默认连**默认发行版**。若用户目标 ROS 环境在非默认发行版，会打开空环境。修复：`wsl --set-default <目标发行版>`，或 `WSL: Connect to WSL` 手动选，或在该发行版终端里 `code .`。
**诊断**：`code --list-extensions` 看有无 `remote-wsl`；`update.code.visualstudio.com`（vscode-server 源）网络可达性。

#### 验证

⚠️ **`ros2 --version` 会报 `unrecognized arguments: --version`**（这版 CLI 不认该写法），**别用它判版本**；改用下面方式。
```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO && ros2 pkg list | wc -l    # 出 jazzy + 几百个包 = ROS2 装好
apt-cache search ros-jazzy | head -3         # 出 ros-jazzy-* 包 = ROS 源通
dpkg -l | grep ros-jazzy-desktop              # 出 ii 才是真的装了 desktop
```

#### Gazebo 验证

```bash
which gz && dpkg -l | grep gz-harmonic       # 出 gz-harmonic 1.0.0 = 已装
timeout 35 gz sim -s -r empty.sdf 2>&1 | tail   # exit=124(=timeout到点杀) = 服务端常驻启动成功；立刻非124+报错 = 失败
```
`gz sim -s`（headless 服务端，只起 `-s` 无 `-r` 也可加 `-r` 自动跑）是常驻进程不自动退出；被 `timeout` 杀(124)恰恰证明"活着没崩"= 启动成功。这也是以后批跑探索实验要用的 headless 模式。

#### 详细诊断记录

见 `references/network-proxy-and-sources.md`（本会话完整排障路径：NAT→镜像模式、DNS 未污染、OSL 被限、dists=noble、502=Clash 节点、国内无镜像等，含具体 curl 探测命令）。

#### 搭建收尾（git 工作站推送 + Windows 侧验证 + 包骨架）

见 `references/git-push-and-validation.md`——私有仓库 **HTTPS+PAT 认证**（GitHub 弃用登录密码、`credential.helper store` 记住、远程非空 `--force`/pull-rebase）；Windows 无 ROS 环境用 **`py_compile`+XML 做静态验证**、真 `colcon build` 需在 WSL + `sudo apt install python3-colcon-common-extensions`；**ament_python 最小功能包骨架**要点（package.xml/setup.py/setup.cfg/resource、entry_points console_scripts、空 `src/` 用 `.gitkeep`）。

#### 常见排查命令

- WSL 内探测代理：`curl -x http://127.0.0.1:7890 -sIL -o /dev/null -w "%{http_code} %{time_total}s" --max-time 15 <url>`
- 看网络模式：`ip route`（独立 `172.25.x` = NAT；变 `172.27.x`/共享 = 镜像）
- 看默认发行版：`wsl -l -v`（带 `*` 的是默认）


---

### 来源技能 `wsl2-ros2-sim-platform`

> 在 Windows 上用 WSL2 搭建 ROS2(Jazzy)+Gazebo 机器人/无人机仿真平台；覆盖中国网络下的代理/镜像处理、WSL2 网络坑、ROS2 安装与验证。

#### 环境基线（WSL2 + Windows）

- 查版本/发行版：`wsl -l -v`（默认发行版带 `*`）。WSL2 版本 >= 2.7 且含 WSLg 最佳。
- 内存/CPU：改 `~/.wslconfig` 的 `[wsl2]` 段（`memory=`、`processors=`、`swap=`）。**改时先备份**（`Copy-Item ... .bak`），并**保留已有的 `kernel=` 行**（如 NVIDIA JetPack 自定义内核）——在作者行后追加，别覆盖。
- GPU 直通：WSL 里 `nvidia-smi` 能看到显卡即 CUDA 已暴露（Gazebo 3D 渲染 / CUDA 可用）。
- WSLg 已含 GUI 支持（RViz、gz sim 窗口能渲染）。

#### 核心坑：WSL2 网络 vs Windows 代理（Clash / VPN）

- 默认 **NAT 模式**：WSL 在独立 `172.25.x/16` vEthernet 网段，**够不到监听 `127.0.0.1` 的 Windows 代理**。症状：`502 Bad Gateway [IP: 127.0.0.1 <port>]`；或 WSL 能上 github.com 但连不上需要走代理的国外源。
- **修复**：在 `~/.wslconfig` 的 `[wsl2]` 下加 `networkingMode=mirrored`，然后 `wsl --shutdown` + 重启（镜像模式即 WSL 共享 Windows 网络栈，`127.0.0.1:<proxy>` 在 WSL 内可达）。改完必须彻底 `wsl --shutdown` 才生效。
- 让 apt 走代理：`/etc/apt/apt.conf.d/proxy` 写 `Acquire::http::Proxy "http://127.0.0.1:7890";` + `Acquire::https::Proxy ...`。下载过程中 Clash 必须保持运行。
- **`502 Bad Gateway` 从代理返回 = 上游节点问题，不是网络/apt**。判断：下载速度应到几 MB/s；若只有 ~17 kB/s 且伴随成片 502，节点是垃圾 → **切换更快更稳的节点**（看实际下载速度，不是看 ping），并加大重试 `-o Acquire::Retries=10`。小元数据能下、大包 502 = 节点对大量下载不稳。

#### ROS2 安装（Ubuntu 24.04，中国网络）

- **国内没有可用的 ROS2 apt 镜像**（清华/阿里/USTC/华为/腾讯的 `ros2/ubuntu` 全 404）。官方 `packages.ros.org`（OSL/OSU 机房）被墙/极慢 → 必须走代理，或换好节点。
- **`dists/` 子目录用的是 Ubuntu 代号，不是 ROS 发行为名**：Ubuntu 24.04 的正确路径是 `.../dists/noble/...`（**不是** `jazzy`）。用错会全程 404。
- 加 key + 源（key 在 GitHub raw，WSL 直连常不通，带 `-x <proxy>`）：
  ```
  sudo curl -x http://127.0.0.1:7890 -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
  ```
- 安装：`sudo apt install -y ros-jazzy-desktop -o Acquire::Retries=10`。**让它跑完，别 Ctrl+C**（一断会有半装状态）。

#### Gazebo 安装

- **OSRF 官方源**（`packages.osrfoundation.org`）经代理比 packages.ros.org 稳（200、快、少 502）。
- GPG key 是**根路径**：`https://packages.osrfoundation.org/gazebo.gpg`（不是子路径）。源：`deb [arch=... signed-by=...] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main`，装 `gz-harmonic`。

#### PX4 真飞控接入：走 MAVROS（MAVLink），不要走 uXRCE/Fast-RTPS

- **现代 PX4（v1.15+ / main）只走 uXRCE-DDS**（`rtps` 命令已移除，`px4_ros_com`/Fast-RTPS 弃用）。
- **在 ROS2 Jazzy 上接 uXRCE 是死结**：micro-ROS-Agent(main) 要 Fast DDS v3，但 Jazzy 只有 fastrtps2，两者**不可共存**（`eProsima_atomic`/`fastrtps` target 冲突，`fastrtps-shared-targets.cmake:42 Some (but not all) targets already defined`）。混装 ros-rolling-fastdds + jazzy 也不行。退到 micro-ROS-Agent v2(fastrtps2) 又协议旧、连不上 PX4 main。
- **治本 = 用 MAVROS（MAVLink）**：MAVLink 不走 fastdds，主流、干净、发论文通行（Gazebo+PX4+MAVROS 是标准姿势）。
- 装 MAVROS：`sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-mavros-extras`（走 packages.ros.org，502 切节点）。
- **MAVROS 还要 GeographicLib geoid 数据**，否则 mavros_node 崩溃：`UAS: GeographicLib exception: File not readable .../geoids/egm96-5.pgm`。修复：`sudo apt install -y geographiclib-tools && sudo geographiclib-get-geoids egm96-5`。
- MAVROS 连 PX4 SITL：launch 文件名是 **`px4.launch`（没有 `.py`）**。`ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`。**PX4 的 mavlink 端口因版本/机型而异**（14540 / 14557 / 14580），连不上（`/mavros/state` 一直 `connected: false`）就看 PX4 启动日志那行 `INFO [mavlink] ... on udp port X remote port Y` 精确配 `fcu_url`。
- 验证接通：`ros2 topic echo /mavros/state --once` → `connected: true`。
- **PX4 SITL 启动坑**：`make px4_sitl gz_x500` 时若已有 gz world 在跑（日志 `gazebo already running world: xxx`），PX4 **复用该 gz 不弹新窗口**、还把模型塞进去 → 先 `pkill -f "gz sim"` 清干净，再启动，PX4 才会弹自己的 Gazebo 窗口显示 X500。

#### WSL 发行版迁移到 E 盘（省 C 盘空间）

- 磁盘在 `C:\Users\<user>\AppData\Local\wsl\{GUID}\ext4.vhdx`；`wsl -l -v` 看发行版。
- 迁移三步（**先导出确认完整，再注销，import 后验证**）：
  1. `wsl --shutdown`（务必先关所有发行版）
  2. `wsl --export <distro> E:\wsl\<name>.tar`（大文件，用 background+notify 跑，别前台长阻塞）
  3. 确认 tar 大小合理（非 0）→ `wsl --unregister <distro>`（释放 C 盘 vhdx）→ `wsl --import <distro> E:\wsl\<name> E:\wsl\<name>.tar`
- import 后验证 `wsl -d <distro> -- ls /home/<user>` 数据在 + 默认用户通常保留。
- `bsdtar: ... cannot archive sockets` 是**无害警告**（PX4 临时 socket，运行时重建），不影响导出完整性。
- tar 导出是**回退备份**：导入 + 验证 OK 前别删；确认稳定后删 tar 释放空间。

#### 验证与常见坑

- `ros2: command not found` = 没 source。`source /opt/ros/jazzy/setup.bash`（可加入 `~/.bashrc` 自动生效）。
- `ros2 --version` 在某些 build 报 `unrecognized arguments: --version` —— 不代表坏了，改用 `echo $ROS_DISTRO` + `ros2 pkg list | wc -l` 确认。
- 装没装完整：`dpkg -l | grep ros-jazzy-desktop` 应 `ii`；`/opt/ros/jazzy/bin/` 里应有 `ros2`。若 `bin/` 为空或 dpkg 为空 = 之前被中断，没装完。
- VSCode remote-WSL 连的是**默认发行版**。若你有 26.04 默认 + 24.04 目标（装 ROS2 的那个），要么 `wsl --set-default <target>`，要么在 WSL 里 `code .` 或 F1 明确选目标发行版。

#### PowerShell ↔ WSL 引号坑

- PowerShell 里 `curl` 是 `Invoke-WebRequest` 的别名 → 用 `curl.exe` 或 `Invoke-WebRequest`。
- 多行/带管道和 `$` 的 bash 命令经 `wsl -d <distro> -- bash -lc '...'` 传会因引号/`$` 展开报错。**把脚本写到 WSL 能读的路径**（`C:\Users\<user>\x.sh` = `/mnt/c/Users/<user>/x.sh`），再 `wsl -d <distro> -- bash /mnt/c/.../x.sh`。


---
