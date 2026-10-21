---
name: airsim-drone-exploration
description: 搭建 AirSim(Unreal UE4.27) + PX4 + ROS2 无人机自主探索仿真平台。覆盖 UE4.27 编辑器名、VS2022 非 VS18、.NET 桌面开发、依赖下载走 Clash 代理、SuvCarPawn/settings.json 等高频坑；PX4 须 v1.11.3(20.04/gcc9 免 patch；24.04+GCC13 才需 gcc-10+static_cast)、QGC(14550)/航线 .plan、MAVROS offboard 算法驱动验证(scripts/offboard_planner_test.py)、定点模式三层验证(QGC 手动/OFFBOARD 测漂移 scripts/hover_verify_ros1.py/接 FAST-LIVO2 vision_pose)、实机外形(FBX→pawn＋PawnPath)、探索算法/FUEL(接入实战见 references/fuel-planner-integration.md)/论文方向；ROS1 Noetic 数字孪生：twin_bridge→FAST-LIVO2 LIO 预览(lidar_type4 须补零值 rgb、cam_model 必填、RViz Fixed Frame=camera_init、livox_ros_driver2↔v1 md5 相同造最小 msg 包)、MAVROS 秒退两坑/桥无数据/roslaunch run_id/roscore 慢/WSL 无 systemd 等启动链路排障见 references/startup-chain-troubleshooting.md、一键启停模板 v5(pkill -x 防自杀)、PX4 SITL 须前台/tmux、Windows↔WSL 坑(引号须落 .sh 脚本、`wsl -u root` 免密)、LiDAR 垂直 FOV 盲区(半径=高度/tan7°，scripts/cloud_geometry_analyze.py)、offboard 位移/多段机动/持续飞行(scripts/offboard_move_forward_ros1.py + offboard_maneuver_ros1.py)、脚本一停即自动降落(offboard 超时+COM_OBL_ACT=1 非故障)、cloud_registered 不随无人机动属正常。
---

# AirSim + PX4 + ROS2 无人机探索仿真平台

## 架构（关键）
- **Windows 端**：Unreal Engine 4.27 + AirSim（高保真环境 + 传感器）。UI 在 Windows。
- **WSL(lyuav) 端**：ROS2 Jazzy + PX4 SITL + MAVROS(MAVLink) + 探索算法。
- 互通：WSL `networkingMode=mirrored` 让 WSL↔Windows 的 `127.0.0.1` 互通 → WSL 能连 Windows 上的 AirSim RPC (默认 41451)。
- **PX4↔ROS2 用 MAVROS(MAVLink)**（`ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`），**不要用 uXRCE**（fastdds3/fastrtps2 在 Jazzy 上死结）。

## 关键死磕点（每次必踩）
1. **UE4.27 编辑器可执行文件叫 `UE4Editor.exe`**（不是 UE5 的 UnrealEditor.exe）。路径 `E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe`。
2. **必须 VS 2022 (17.x)**，不是 VS 18(2026)。UE4.27 用 `cmake -G "Visual Studio 17 2022"` + MSVC；VS18 编译会不兼容。
3. **必须装 ".NET 桌面开发"**（Visual Studio Installer → 修改 VS2022 → 勾 .NET desktop development）——否则 UnrealBuildTool 报 `ERROR: Could not find NetFxSDK install dir`。
4. **依赖下载走 Clash 代理**（用户端口 7899，非 7890）。国内直连 GitHub 时通时断。

## 完整步骤
1. 装 **Unreal Engine 4.27**（Epic Launcher，装 E 盘；库里有 UE4.27.2）。
2. 装 **VS 2022 Community (17.x)**：勾 "使用 C++ 的桌面开发" + Windows 10 SDK + **".NET 桌面开发"**。
3. clone AirSim（默认分支）：`git clone https://github.com/microsoft/AirSim.git E:\AirSim`（直连或 `-c http.proxy=http://127.0.0.1:7899`）。
4. 从 **"x64 Native Tools Command Prompt for VS 2022"** 运行 `build.cmd --no-full-poly-car`（--no-full-poly-car 跳过 37MB SUV 高清车资产）。构建 AirLib/rpclib/MavLinkCom + copy 插件到 `Unreal\Plugins\AirSim\`。
   - 若 build.cmd 下载 rpclib/eigen 失败（iwr 直连不通）：手动 `curl -L --proxy http://127.0.0.1:7899` 下载 rpclib v2.3.0 zip 解压到 `external\rpclib\rpclib-2.3.0`，eigen-3.3.7 解压到 `AirLib\deps\eigen3\Eigen`，再重跑 build.cmd。
5. 编译 Blocks 编辑器模块（缺 .NET 会报 NetFxSDK，装好 .NET 后）：
   `E:\Program Files\Epic Games\UE_4.27\Engine\Build\BatchFiles\Build.bat BlocksEditor Win64 Development -Project="E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject" -WaitMutex`
6. **必须补下 SUV 资产**（--no-full-poly-car 跳过了，但 Blocks 关卡引用 SUV 材质/SuvCarPawn）：
   `curl -L --proxy http://127.0.0.1:7899 https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip` 解压到 `E:\AirSim\Unreal\Plugins\AirSim\Content\VehicleAdv`（产出 Materials/SUV/Textures/Vehicle 等）。否则 UE 打开报 `Failed to load asset class SuvCarPawn_C` 或 `AutomotiveMaterials/*` 加载失败。
7. **settings.json**。路径 `C:\Users\<USER>\Documents\AirSim\settings.json`（改前先备份 `settings.json.bak`）。两种配置都能出无人机：
   - **最小配置**（自动建默认多旋翼，最简单）：
     ```json
     { "SettingsVersion": 1.2, "SimMode": "Multirotor" }
     ```
   - **自定义 vehicle**（要指定类型/接 PX4 时用）——**`VehicleType` 必须是 AirSim 的「车类型」常量：`"SimpleFlight"` 或 `"PX4"`**（`kVehicleTypeSimpleFlight="simpleflight"`、`kVehicleTypePX4="px4multirotor"`，大小写不敏感）。**千万不要写 `"QuadX"`**——那是**机架 frame 类型**，不是车类型，会报 `There were no compatible vehicles`：
     ```json
     { "SimMode": "Multirotor", "Vehicles": { "Drone_1": { "VehicleType": "SimpleFlight" } } }
     ```
   ⚠️ 写 `Vehicles` 时只有 `VehicleType` 用**错值**（如 `QuadX`）才报"无车辆"；用 `SimpleFlight`/`PX4` 就能正常 spawn（实测通过）。`FirmwareName`/`Cameras` 可不写，写对 `VehicleType` 即可。
8. 用 `UE4Editor.exe` 打开 `E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject` → 点 **Play** → 出现 QuadX 四旋翼无人机。之后可 **F1 看键位**（**PageUp/PageDown 升降，W/S 前后俯仰，A/D 偏航**；R=录制，方向键是车用的）。脚本控制用 **airsim Python API**（`pip install airsim` → `airsim.MultirotorClient()`，`takeoffAsync()`/`moveByVelocityAsync()`），WSL 经镜像网络连 `127.0.0.1:41451`。⚠️ **v1.8.2 的 `pip install airsim` 在 Python 3.12 上装不上**（`PythonClient/airsim` 依赖老 `msgpackrpc`，钉死 `tornado<5`，而 tornado 4.x 只支持 Python ≤3.9）：用 **Python 3.9 venv** 跑，或**优先走 MAVROS(ROS2) 控 PX4**（更贴合 ROS2 探索栈，见 references/qgc-ground-station.md §5）。

## 接 PX4 SITL（真飞控固件）—— 完整要点见 references/px4-sitl-integration.md
- **PX4 必须 v1.11.x**（AirSim 只测过 v1.11.x；v1.18 的 MAVLink 与 AirSim 不兼容）→ 单独 clone 一个 `v1.11.3` 并重编，别动现有的。
- **启动命令 `make px4_sitl_default none_iris`**（不是 `make px4_sitl`，也不是 gz_x500）；`make px4_sitl` 不带模型**只编译不启动**（无 pxh>）。
- **⚠️ 编译 v1.11.3 在新 Ubuntu(24.04/GCC13) 必踩**：`platforms/common/px4_work_queue/WorkQueueManager.cpp:257` 的 `math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(...))` 报 `no matching function for 'max(long int,int)'`（新 glibc 让 `PTHREAD_STACK_MIN` 变 `long`、`PX4_STACK_ADJUSTED` 返回 `size_t`，类型不匹配，任何编译器都拒）。**修法**：改成 `math::max(static_cast<size_t>(PTHREAD_STACK_MIN), static_cast<size_t>(PX4_STACK_ADJUSTED(wq->stacksize)))`。**别用 `std::max`**（该文件没 include `<algorithm>` → 报 `'max' is not a member of 'std'`）。**再用 GCC-10 编译**：`CC=gcc-10 CXX=g++-10 make px4_sitl_default none_iris`，**且必须先 `rm -rf build/px4_sitl_default`**（否则 CMake 已缓存 `/usr/bin/c++`=gcc-13、编译仍走 gcc-13，CC 环境变量对已缓存 build 无效）。gcc-10 安装：`sudo apt install gcc-10 g++-10`（在 universe 组件，需 apt 源含 universe、且 apt 代理端口指向实际 Clash 端口，否则 `Ign:` 拉不到——本项目是 7899）。完整复现见 references/px4-v1.11.3-gcc13-build.md。
- **编译环境选择（重要）**：PX4 v1.11.3 的**原生组合是 Ubuntu 20.04 + GCC 9**——在这种系统上**既不用 gcc-10、也不用 WorkQueueManager patch**，直接 `make px4_sitl_default none_iris` 就能编过（本会话在 `ros1-twin`(Ubuntu20.04/gcc9.4) 已验证）。上面那套 `static_cast` patch + gcc-10 + `rm -rf build` **只针对新系统（Ubuntu 24.04 / GCC 13）**。想省事就把 PX4 装在 Ubuntu 20.04 的发行版里。
- **构建依赖检查清单（装 PX4 前先查）**：`git / gcc / g++ / cmake(≥3.13) / python3 / make / curl` + **`ninja`（Ubuntu 20.04 常缺，必装！）**。缺 ninja 用**免密 root** 装：`wsl -d <distro> -u root -- apt-get install -y ninja-build`（验证 `ninja --version`）。
- **⚠️ PX4 SITL 不能"后台无 tty"起**：`make px4_sitl_default none_iris` 在后台（无终端）跑时，`pxh>` shell 读到 stdin EOF 会**立刻 `pxh> Exiting NOW.` 退出**（哪怕前面已打印 `Simulator connected on TCP 4560`）。→ 必须**前台终端**跑，或 **tmux**（`tmux new -s px4` → 运行 → `Ctrl+B` 再 `D` 脱离；`tmux attach -t px4` 回去），**或直接把命令交给用户跑**。RViz(GUI/WSLg) 同理——这类"占终端/要看窗口"的长驻服务，用户偏好**给命令让他自己执行**（"你告诉我启动链路，我来启动"/"我自己来跑"），不要 agent 一跑就阻塞。
- settings `VehicleType` 用 **`"PX4Multirotor"`** + `UseTcp:true` + `TcpPort:4560` + `ControlPortLocal/Remote` + Barometer + `NAV_RCL_ACT:0` 等（见参考文件里的完整 JSON）。
- PX4 在 **TCP 4560** 等 AirSim 连；Windows 防火墙放行 **TCP 4560 + UDP 14540**；跨系统经镜像网络。
- 验证：PX4 日志 `Simulator connected on UDP port 14560` + `EKF GPS checks passed / commencing GPS fusion`。
- MAVROS 连它：`ros2 launch mavros px4.launch`（**注意是 `.launch` 不是 `.launch.py`**）；先 `sudo geographiclib-get-geoids egm96-5`（否则 mavros 崩 `GeographicLib exception`）。
- **⚠️ 关键：MAVROS 连不上 PX4（`VER: broadcast request timeout` → `connected:false`）= MAVLink 端口被 AirSim 占用**。AirSim 的 PX4 车辆用 `ControlPortLocal:14540 / ControlPortRemote:14580`，**正好是 MAVROS 的标准 onboard 端口**，两者抢同一条 MAVLink 链路（AirSim 先占，MAVROS 发出去没人应）。
  - **修复**：在 PX4 的 `pxh>` 里**另开一条 MAVLink 专线给 MAVROS**：`mavlink start -u 14600 -o 14601 -r 2000000 -t 127.0.0.1 -m onboard`，然后 `ros2 launch mavros px4.launch fcu_url:=udp://:14601@127.0.0.1:14600`。
    - 若报 `ERROR [mavlink] port 14600 already occupied`：说明该专线**已存在**（之前跑过）→ **换个端口重开**（如 `-u 14610 -o 14611`），或直接让 MAVROS 连已存在的 `14600`。别硬刚同端口。
    - `ros2 topic echo /mavros/state --once` 报 `topic does not appear to be published yet` = **MAVROS 节点没在跑**（不是连接失败）——MAVROS 必须先启动并保持运行，echo 才有输出。
  - **不要**用 `fcu_url:=udp://:14540@127.0.0.1:14580`（会撞 AirSim）。备选：关掉 QGC 后走它的通道 `fcu_url:=udp://:14550@127.0.0.1:18570`（PX4 日志 `mode: Normal ... on udp port 18570 remote port 14550`）。
- `ERROR [mavlink] Unknown param name: COM_OBL_ACT` 是**非致命警告**（PX4 v1.11.3 没有这个参数名，AirSim 设了它、PX4 直接忽略；只影响 offboard 结束后的悬停/降落行为，**不影响 takeoff/Mission/QGC**）。想消掉就从 settings.json 的 `Parameters` 删掉 `COM_OBL_ACT`。
- `WARN [commander] command 520 unsupported` + `INFO [tone_alarm] notify negative` **反复刷屏（~1Hz）= 某组件在周期性重发一条 PX4 v1.11.3 不支持的指令**（多为 QGC 在重试被 preflight 挡下的动作，或 AirSim↔PX4 链路重试）；**非致命、不挡 MAVROS/offboard**。定位法：临时关掉 QGC 看刷屏是否停。
- **QGC 验证/控制（最直观，推荐先做）**：装 QGroundControl(Windows)→ 连 **UDP 14550** → 左上出现 **`Ready`/`Hold`** + 飞行器仪表 → **Arm/Takeoff** → **AirSim 里无人机由 PX4 起飞**（这一步亲眼验证"PX4 真在控制 AirSim"）。下发航线用 **`.plan`** 文件：模板 `templates/airsim_px4_mission.plan`（起飞 + 4 环绕航点 + RTL），QGC Plan 视图 File→Load 导入 → Upload → Arm → Start Mission。完整见 `references/qgc-ground-station.md`。

## 验证"环境能被算法驱动"（MAVROS offboard）
- 脚本 `scripts/offboard_planner_test.py`：20Hz 持续发 `/mavros/setpoint_position/local` → 流 2s 后切 `OFFBOARD` → 定期 arm → 每 15s 换航点飞**方形航线**。跑法（MAVROS/PX4/AirSim 都开着）：`source /opt/ros/jazzy/setup.bash && python3 scripts/offboard_planner_test.py`。
- 这是"规划器/探索算法"的最小替代（算法发 setpoint → MAVROS → PX4 → AirSim 无人机飞）。**无人机按算法飞起来 = 仿真环境可被算法驱动**。把它当控制器起点模板，把方形航点换成 frontier/规划器输出即可。
- 探索算法（认识回路、FUEL 接入现实、论文选题方向）见 `references/exploration-controller-integration.md`。
- **算法层突破 / 论文陪读**：用户要"从算法层突破、先学别人的算法（FUEL 等）、做论文陪读"时，按 **`references/fuel-algorithm-teardown.md`** 的"论文陪读方法论"走（拉论文+源码→提取PDF文本→读核心→写拆解笔记→给下一步选项），内含 **FUEL 算法核心拆解**（FIS + 层次化 TSP + EGO）、源码↔文件对照表、**对接 ros1-twin 的数据流**（点云→栅格、位姿→odom）、**创新点切入点**（位姿不确定性/语义 frontier 等）。可复用于 EGO-Planner/RACER 等后续陪读。
- **FUEL 真正接进平台（2026-09-12 已跑通「编译 + demo 自主探索 + 输入桥」）**：按 **`references/fuel-planner-integration.md`** 走。核心事实：① 走 **`ros1-twin`(Ubuntu20.04/ROS1 Noetic/gcc9)**，**一次编译通过 25 包 0 error**，不需要 ros1_bridge/Docker（`exploration-controller-integration.md` 里"FUEL 装不了"的旧结论已作废）；② 独立工作空间 **`~/fuel_ws`**，绝不动 `~/twin_ws`；③ 依赖只有 `libarmadillo-dev`(apt) + **nlopt v2.7.1(源码装)**，**LKH(TSP) 不用装** —— 它的完整源码就在 `utils/lkh_tsp_solver/src/*.c`，随 catkin 编进库（别去官网下二进制）；④ FUEL 要 **3 个输入**：`/odom_world`(Odometry) + **`/map_ros/pose`(传感器位姿, raycast 必需，最易漏)** + `/map_ros/cloud`(PointCloud2)，输出 `/position_cmd`(`quadrotor_msgs/PositionCommand`)；⑤ 跑自带 demo 用**独立 master 端口**(如 11321) 避免撞 twin 的 11311，触发探索**可编程发** `/move_base_simple/goal`（不必手点 RViz）；⑥ **接自研平台必须剥离 `simulator.xml`**（`exploration.launch` = algorithm.xml + traj_server + waypoint_generator + **自带仿真**）→ 用 `templates/fuel_real.launch` + `scripts/fuel_input_bridge.py`。实测规划周期 **Total 14.4ms**（论文 i7-8550U 为 24~36ms，可直接进论文性能表）。输出桥 `/position_cmd → MAVROS setpoint_raw/local` 已于 2026-09-12 **端到端跑通输入侧**（FUEL 用 FAST-LIVO2 的真实数据建出 SDF 地图：`/sdf_map/occupancy_all` 907 体素、范围 ±4.4m 与 LiDAR 4.5m 量程吻合、frontier 20 簇 + `cost mat (20,20)` 在维护；实测证据/启动顺序见 `references/fuel-planner-integration.md` §9）。输入侧已端到端跑通（实测证据见 §9）；**输出桥已实现并通过 ad-hoc 验证**（`scripts/fuel_ctrl_bridge.py`：走 `setpoint_position/local`(ENU) 让 **MAVROS 代做 ENU→NED**，避免手工换算；带 **`seed` 悬停**，解决"FUEL 未触发→没 setpoint→PX4 进不了 OFFBOARD→无人机飞不起来"的死结，见 §14）；**一键脚本 `scripts/fuel_all.sh`**（= twin 链路 + MAVROS 专线 + 起飞 1.5m + FUEL + **自动触发**，唯一手动项是 AirSim 点 ▶ Play）+ **配对关闭脚本 `scripts/fuel_stop.sh`**（默认停 FUEL/桥/MAVROS/FAST-LIVO2/PX4/RViz 但**保留 ROS master**；`--all` 全停、`--no-save`、`--keep-px4`）。
- **★★★ 撞墙的头号根因 = 坐标系朝向（yaw）不一致**（2026-09-12 实测：FAST-LIVO2 `camera_init` yaw=**0.00°** vs PX4 EKF yaw=**82.24°**，而原点只差 5mm）→ 外部规划器的目标点被 PX4 朝**错误方向**执行 ⇒ 全速撞墙。**"原点对齐" ≠ "坐标系对齐"** —— 任何「LiDAR-inertial 里程计 + PX4/MAVROS」的搭配都要查 yaw（FUEL/SUPER/EGO 通用）。解法：控制桥自动估计 `psi_offset = yaw_px4 − yaw_fuel` 并旋转目标（**对齐完成前拒绝转发**做防撞保险）、`~psi_offset_deg` 可手动覆盖、只读诊断脚本 `scripts/fuel_frame_check.sh`（用户报"撞墙"时**先跑它别再飞**）。完整系统讲解见 `references/fuel-planner-integration.md` **§17**，同节还含：`init_z ≠ seed_z` 的 traj_server **占位指令**锁死高度陷阱、`<arg value=>` 不可被命令行覆盖（须用 `default=`）、roslaunch `$(arg)` 前向引用报错、参数真实路径叠加（`/traj_server/traj_server/init_z`，查错路径会假报"参数不存在"）。
- **项目内 canonical 测试入口**：`make test`（=`fuel_bridge/test/smoke_test.sh`，**25 项**：静态语法/XML/防撞参数断言/arm 节流 + 输入桥转发 / seed 悬停 / 未对齐防撞 / 朝向对齐旋转+yaw）/ `make lint`（用 **`ast.parse`，别用 `py_compile`**——后者会留 `__pycache__` 把工作区弄脏）。**要做任何验证先想起它**；它也顺手终结了"临时验证脚本本身被算作 changed path"的无限循环（见 `references/fuel-planner-integration.md` §15 末尾）。
- **撞墙排查三把（只读，先跑再飞）**：`scripts/fuel_frame_check.sh`（一次性位姿对比 → `psi_offset`）→ `scripts/fuel_align_check.sh`（**持续残差**验证 `p_px4 ≈ Rz(psi)·p_fuel`）→ `scripts/fuel_map_check.sh`（FUEL 建图质量：occupancy/esdf 点数与范围 + 目标附近 3m 障碍数）。**读法见 §19**：残差 **0.4~1.1m 属正常**（1s 采样 × 0.8m/s 滞后就贡献 ~0.4m，不是 0 才对）；无人机**快速旋转时 psi 会乱跳成 16°/149° 这种垃圾值并让残差虚高到 8~9m —— 那是采样不同步，不是对齐错**（本会话为此白折腾一轮）；三次运行 psi 都是 **82.3x°** ⇒ 确定性差异。
- **对齐修好后仍撞墙 = FUEL 默认防撞参数对真机过松**（用户明确要求**别动 `box_min_z`/`box_max_z`**）：`algorithm.xml` 里 `obstacles_inflation 0.199→0.45`（20cm 余量对 ~0.5m 机身等于没有）、`max_ray_length 4.5→8.0`、`clearance_threshold 0.2→0.45`、`min_candidate_clearance 0.21→0.45`；`fuel_real.launch` 的 `max_vel 1.5→0.8`、`max_acc 1.5→1.0`。改动处加 `FM:` 注释 + 备份（`algorithm.xml.bak_fm` + 桌面 `algorithm_FM.xml`）。**前 4 个是写在 `algorithm.xml` 里的 `<param>` 而非 arg，launch 覆盖不了，必须改文件**。概念上记牢：**FUEL 把未观测区域当可通行**，参数只能缓解、不能消除（§18.1）。
- **★ 两个"别再走"的误判**：① `devel/lib/<pkg>/*.py` 是 catkin 生成的 **relay/wrapper**（内容只有一句 `exec(compile(open('<src路径>').read(),…))`）⇒ **grep 不到自己的新代码是正常的**，**不等于"跑的是旧版脚本"**（本会话据此误报过一轮）；判版本要看 src 的 mtime/内容。② `roslaunch` 的**每节点日志在 `~/.ros/log/<uuid>/<node>.log`**，`~/.ros/log/latest/` 指向最新一次 —— 让用户查日志时**务必给对路径**（本会话曾让用户 grep `/tmp/fuel_real.log`，而对齐日志其实一直在 per-node 日志里）。
- **PCD 保存开关**：`bash ~/fuel_all.sh 1.5 position 3.0 1`（第 4 个参数，1 存 / 0 不存）→ 导出 `PCD_SAVE`
  → `twin_livo_fast.sh` 在**启动 FAST-LIVO2 之前**设 `rosparam set /pcd_save/pcd_save_en true`（+`interval -1`+建 `Log/pcd`）。
  产物 `~/twin_ws/src/FAST-LIVO2/Log/pcd/{all_raw_points,all_downsampled_points}.pcd`。
  **★ 必须用 SIGINT 关闭才有 PCD**（`savePCD()` 在 `main()` 的 `spin()` 之后）—— `fuel_stop.sh` 已正确用 `pkill -INT` 并等它退出；`kill -9` 不存。
  **⚠️ 对"已在跑"的 FAST-LIVO2 改该参数无效**（参数在构造时读一次）→ 脚本会提示先 `fuel_stop.sh` 再重跑。详见 `references/fuel-planner-integration.md` §16.4。
★ **头号误判：FUEL 启动后停在 `WAIT_TRIGGER`，不显式触发就永不规划、无人机原地悬停** —— 而 `to visit: N` / `cost mat size final: (N,N)` 这些日志**会骗你**（它们出自 0.5s 的 `frontierCallback`，与"探索有没有启动"**无关**）；判据只看 **`Triggered!`** 和 **`[FSM]: state:`**（详见 §11）。**真机端到端飞行（无人机被 FUEL 驱动）仍未实测** —— 需用户自己启动。
- **两个通用坑（写任何 ROS 桥 / launch 必踩）**：① **launch/XML 注释里不能出现连续 `--`** —— `<!-- ---------- 输入桥 ---------- -->` 违反 XML 规范，文件**直接非法、roslaunch 拒绝**（本会话 `fuel_real.launch` 真踩，靠 `ET.parse()` 自检才早发现）→ 分隔线一律用 `====`/`——`，别用 `----`；② **验证"桥转发没转发"必须「先订阅、再发布」** —— ROS publisher **默认不 latch**，先 `rostopic pub` 再 `rostopic echo` 会在订阅生效前把消息发完，**误判桥坏了**（本会话真误判过一轮，差点去改一个完全正常的桥）。正确：后台 `timeout 14 rostopic echo /out > /tmp/x.txt &` → sleep → 再 pub → sleep → grep 校验。

### 定点模式 / OFFBOARD 悬停验证（三层，ROS1 ros1-twin 侧，2026-09-10）
用户问“怎么确认定点模式能用”时，按“省事→严格”给三层，并说清**每层验证的到底是什么**（否则会被“飞起来没炸”误导）：
1. **QGC 手动（30 秒，先做）**——起飞 → 切 `Position` → 打杆移动 → **松手** → 看是否**自己停住**（QGC 速度归 0、位置不再变）。验证的是 **PX4 自身定点能力**（定位源 = AirSim 真值/内部估计）。
2. **MAVROS offboard 悬停测漂移（代码级，接规划器的前置）**——脚本 `scripts/hover_verify_ros1.py`（ROS1 版）：预发 setpoint 3s → 切 `OFFBOARD` → Arm → 飞到 `(0,0,5)` → 悬停 20s → 打印 x/y/z 范围 + **最大漂移** → `AUTO.LAND`。**⚠️ 漂移必须只统计「到达目标后的稳态段」**（`z > TARGET_z - 0.5` 的样本），否则会把**起飞爬升 0→5m** 算进去，误报 `最大漂移 5.000 m / ⚠️ 漂移偏大`——本会话第一次跑就踩了这个口径坑，实际稳态漂移只有 <5cm。**判据：稳态三维最大漂移 < 0.5 m = 定点正常；持续单向漂移 = 查定位源**。本会话实测通过：`mode=OFFBOARD`，升到 5m 后 x/y 抖 <1cm、z 钉在 5.00。
   - **⚠️ MAVROS(ROS1) 起不来的两个必踩坑（2026-09-10 ros1-twin 实测，都表现为「MAVROS 节点秒退」）**：
     - ① **`[FATAL] UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm`** → `REQUIRED process [mavros-1] has died!`。修：`sudo geographiclib-get-geoids egm96-5`（要联网），或**同机已有该数据的发行版直接拷**（本会话从 `lyuav` 拷 `/usr/share/GeographicLib/geoids/*` 经 C 盘中转到 `ros1-twin`：`egm96-5.pgm`(18.6MB)+`.aux.xml`+`.wld`）——**多发行版同机时拷文件比重新下快得多**。
     - ② **`[FATAL] FCU: DeviceError:udp:bind: Address already in use` + `udp0: Bind address: 0.0.0.0:14540`** → 14540 被占（多为上一个没退干净的 MAVROS），`fcu_url:=udp://:14540@127.0.0.1:14580` 起不来。**修：给 PX4 开专用 MAVLink 专线，再让 MAVROS 连它**——`tmux send-keys -t px4 "mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard" Enter`（PX4 控制台回显 `mode: Onboard, data rate: 2000000 B/s on udp port 14610 remote port 14611`），然后 `roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610` → 实测 `/mavros/state` 出 `connected: True / armed: False / guided: True`。**这与 lyuav/ROS2 那套「14540 撞 AirSim→改专用线」同源**（本质都是端口占用）；**ROS1 侧先 `ss -lunp | grep -E '14540|14580|18570'` 核实际占用**（ros1-twin 实测 PX4 只监听 14580+18570，AirSim 走 TCP 4560）再定 fcu_url，别照搬别处的结论。
     - 判读：MAVROS 起来后 `rosnode list | grep mavros` 有 `/mavros` + `rostopic echo -n 1 /mavros/state` 出 `connected: True`；`topic [/mavros/state] does not appear to be published yet` = **MAVROS 根本没启动**（不是"连不上"）——先确认它起了再查连接。
   - **顺序（接规划器前必须全绿）**：PX4(跑) → 专用 MAVLink(14610/14611) → MAVROS(connected:True) → `hover_verify_ros1.py` 稳态漂移 <0.5m → 才算"控制回路打通"。
3. **接 FAST-LIVO2 定位的定点（★ 论文要的就是这层）**——`FAST-LIVO2 → /mavros/vision_pose/pose → PX4 EKF2 → 定点用 LIVO 定位`。前两层用的是 AirSim 真值/内部估计；**只有这层把真值断掉仍能定住，才证明 LIVO 定位可用**。
- 判读别踩：无人机**静止**时坐标在原点附近 ±mm 级微抖是**正常噪声**（见本文件“判读”节的 mm 级说明），不是“定点坏了”；要看“跟踪”必须让它飞起来。
- 配套关闭脚本模板：`templates/twin_oneclick_stop.sh`（与一键启动配对；`pkill -x` 防自杀 + 残留自检，见 Windows↔WSL 节的 pkill 自杀坑）。**默认语义 = 停 PX4+桥+算法+RViz、保留 ROS master**；只有 `--all` 才连 master 一起停（用户 2026-09 明确要求的取舍）。**stop 后 `rosnode list` 仍会列出刚被杀的节点名（`/laserMapping` 等）——那是 ROS 的注册残留（硬杀进程不会自动注销），无害，master 下次连接时清掉**，别据此判断"没停干净"。

## 实机外形复刻（把 AirSim 无人机换成实机模型）
- 目标：AirSim 里无人机显示**实机外形**（X8），飞行仍由 **PX4** 控制（只换视觉 mesh，不改动力学）。
- **两种换法（优先第 1 种）**：
  1. **`PawnPath` + 自定义 pawn 蓝图**（不改 AirSim 插件 content，最干净、最推荐）：在**项目目录** `/Game/...` 里新建/复制一个继承 AirSim flying pawn 的蓝图（如 `BP_CustomDrone_GroundSafe`，可加**地面碰撞**防贴地穿模），再在 settings.json 给车辆指向它：
     ```json
     "Vehicles": { "PX4": { "...": "...", "PawnPath": "CustomDrone" } },
     "PawnPaths": { "CustomDrone": { "PawnBP": "Class'/Game/CustomDrone/BP_CustomDrone_GroundSafe.BP_CustomDrone_GroundSafe_C'" } }
     ```
     换模型只改该蓝图里的 mesh。**实测有效**（本会话即用此法完成 X8 外形替换 + 方向修正 + 地面碰撞）。
  2. 直接改**插件**里的 `BP_FlyingPawn`（见下）——能用，但改的是插件二进制资产（官方警告风险），且升级 AirSim 会丢。
- 素材首选 **`.fbx`**（Unreal 原生；CAD 需先导 FBX/OBJ）；真实尺寸、Y-up、单位 cm、面数适中（3050 4GB 别过高）。
- **FBX 导入对话框要点**（Blender 导出的 FBX 常为 cm / Y-up RH）：**导入统一缩放 = 1.0**（FBX cm = UE cm，明显过大/过小再调）、勾 **转换场景**（Y-up→Z-up）、材质选 **新建材质**、静态模型别勾骨骼网格体；**⚠️ 导入路径别落在 `/AirSim/Blueprints`（AirSim 插件目录）**——官方警告"改插件 content 有风险"，应在**项目目录 `/Game/...`**（如先建 `/Game/vehicles`）导入。找不到"切到项目 Content"入口时：Content Browser 左侧 **Sources 面板点 `Content`**，或顶部**路径栏输 `/Game` 回车**；实在切不动、直接导到插件目录**也能用**（只是不干净，AirSim 的 BP 仍能引用）。
- 步骤（UE 编辑器 GUI）：Esc 停 Play → Content Browser **Import** FBX（勾 `Combine Meshes`/`Import Materials`/`Convert Scene`）→ **⚠️ 先让 AirSim 插件内容显示出来，否则根本找不到 `BP_FlyingPawn`**（官方 `docs/working_with_plugin_contents.md`：**"Plugin contents are not shown in Unreal projects by default."**；**最快 = Content Browser 顶部搜索框直接输 `BP_FlyingPawn`**，勾 `Show Plugin Content` 后仍可能看不到文件夹但搜索框能搜到）→ 打开蓝图 **`BP_FlyingPawn`**（`Plugins/AirSim Content/Blueprints/`；实体 `E:\AirSim\Unreal\Plugins\AirSim\Content\Blueprints\BP_FlyingPawn.uasset`）→ 组件树找 mesh 组件 → Details 的 **Static Mesh** 换成实机模型 → 调 **Scale**（按实机轴距校准）+ 归中 → Compile/Save → Play。
- 传感器/数据流一致性（MID360 非重复扫描是最大障碍、D435、IMU 按 datasheet、话题级一致）+ 分层实施路线 + 需采集的实机信息，见 `references/real-vehicle-replica.md`。

## AirSim 数字孪生 + FAST-LIVO2（ROS1 Noetic 传感器桥）
当目标是用 AirSim 当**传感器数据源**、把**真实 FAST-LIVO2 (LIO)** 跑在 WSL 的 **ROS1 Noetic** 里（"数字孪生"；**与 ROS2 的 lyuav 分开**一个发行版 `ros1-twin` + 工作区 `~/twin_ws`）：
- 桥 `twin_bridge` 直连 AirSim RPC(41451) → 发 `/twin/*` 话题（imu/lidar/color/depth/clock）；一键脚本 `start-sensor-preview.ps1` / `start-fastlivo-preview.ps1` / `start-rviz-preview.ps1` / `check-airsim-lidar.ps1` / `verify-live-streams.ps1`（都在 Windows 侧调 wsl 跑）。
- FAST-LIVO2 预览模式：`lidar_type: 4`（走 L515 路径，绕开需要 Livox 逐点时间的路径）+ `img_en: 0` + 身份外参；位姿**隔离**到 `/twin/fast_livo/vision_pose_preview`，**不回灌 PX4**（未标定不许污染飞控）。
- **必踩清单（细节与实机标定值见 `references/airsim-fastlivo2-ros1-twin.md` §2）**：① 桥发的点云是**纯 XYZ**（`create_cloud_xyz32`，无 rgb），而 `lidar_type:4`(L515) 预处理**要 rgb → 必须给桥接点云补零值 rgb 字段**（`create_cloud` + 4 个 PointField `x/y/z/rgb`，**已实测跑通**）；其余 handler(velodyne/ouster/robosense) 要 intensity —— **纯 XYZ 哪个 handler 都喂不了**；② **`cam_model` 必须要，哪怕 `img_en:0`** —— 不 `rosparam load <camera.yaml> /laserMapping` 就崩 `Camera model not correctly specified`（刷屏 `Cannot find value for parameter: laserMapping/cam_model`）；③ **RViz 不出图 = Fixed Frame 没设** —— FAST-LIVO2 输出(frame `/cloud_registered`/`/aft_mapped_to_init`/`/path`) frame 都是 **`camera_init`**，自带 `.rviz` 里是空占位 `<Fixed Frame>`，要在 `Global Options → Fixed Frame` 填 `camera_init`；**③b「点云消失/看不见」的第二大根因 = `Views` 视角**（与 Fixed Frame 是两个独立原因，别只查一个）：自带配置里 `Views.Current` 是 **`Class: rviz/ThirdPersonFollower`**（跟随模式，视野会随某个 frame 乱跑）且 **`Distance: 124`**（相机离焦点 124 米），而点云只有 ±35m → **在 124m 外小到看不见**。修：把 `Views.Current` 改成 **`rviz/Orbit`** + **`Distance: 40`** + **`Focal Point: (0,0,5)`** + `Yaw: 0.6 / Pitch: 0.35`（这几个值在文件里是唯一的，直接文本替换即可；改完 `pkill -x rviz` 重启 RViz 才生效，改配置不影响运行中的 RViz）。**判据：`/cloud_registered` 有 ~10Hz 数据 + `scripts/cloud_geometry_analyze.py` 显示点多在原点附近 ±35m ⇒ 数据没问题，是「看」的问题**（本会话实测：6571 点、中位 (0.76,0.23,2.8)、frame=camera_init）。**「上方俯视」= Orbit + 大 Pitch**：用户要「从上方俯视 + 鼠标自由移动」时，把 `Pitch` 从 `0.35`(≈20°，接近平视) 提到 **`1.35` rad(≈77°)**、`Distance: 45`——**别用 `TopDownOrtho`**（正交固定视角，鼠标不能旋转）；`Orbit` 本身已支持左键旋转 / 中键平移 / 滚轮缩放。**「点云消失」首要排查顺序 = 先确认数据流还在不在**：`rostopic hz /cloud_registered` 出 `no new messages` = **算法链被停**（桥/FAST-LIVO2/PX4 被 `twin_livo_stop.sh` 停了、只剩 master+RViz）→ `bash ~/twin_livo_fast.sh` 补回即可；**有 ~10Hz 数据但 RViz 看不到，才查视角/display/Fixed Frame**。别数据断了先去改视角；④ bag 是 `livox_ros_driver2/CustomMsg` 而工作区只有 v1 时 `rosbag play` 加载不了 → 建**最小 msg-only 包** `livox_ros_driver2`（拷 v1 的 `CustomMsg.msg`/`CustomPoint.msg`，md5 相同；FAST-LIVO2 的 v1 订阅者按 md5 照样连；⚠️ package.xml maintainer email 必须合法，`x@local` 会 build 失败）；⑤ `ClockType:"SteppableClock"` + `LockStep:true` → 仿真时间靠 **PX4 步进**，AirSim 单独 Play 可能**时间冻结/传感器无新数据**（先确认 PX4 SITL 在跑）；⑥ **改 settings.json 后必须关掉整个 `UE4Editor.exe` 重开**——编辑器里停/重开 PIE **不会**重读 AirSim 设置（重启后跑 `check-airsim-lidar.ps1` 见 `mid360_preview_profile_loaded: true` 才算真加载）。
- **判读**：静止无人机的 `/aft_mapped_to_init` 位置在原点附近 ±**mm 级**微抖（z 抖 0.1mm 级）是**正常估计噪声**，不是 bug；**真异常 = 持续单向漂移**。要验证"跟踪"得让无人机飞。实机标定值/话题/`/usb_cam/image_raw` 等从实机 `~/catkin_ws/src/FAST-LIVO2/config/MID360*.yaml` + `camera_self.yaml` 抄（实机 bag 在 `~/Desktop/Bags/*.bag`，含 `/livox/lidar`+`/livox/imu_tf`+输出，可回放复现）。
- 完整细节（数据接口表、传感器 JSON、坑清单、观察话题）见 `references/airsim-fastlivo2-ros1-twin.md`。
- **可直接复制的模板**：算法配置 `templates/fastlivo2_twinsim.yaml`（指向 `/twin/*` + `lidar_type:4` + 实机外参/IMU噪声，含相机配置示例与"为何不能用实机 lidar_type:1"的说明）；一键启动 `templates/twin_oneclick_launch.sh`（PX4(tmux) + 桥 + FAST-LIVO2 + RViz，清理全用 `pkill -x`）。

### 「相机能看到障碍、点云里却没有」= LiDAR 垂直 FOV 盲区（2026-09-10 实测定位）
用户报"相机朝向前面明明有障碍，点云里没扫到"时，**先算几何、别先怀疑桥/算法**：
- AirSim 的 `TwinLidarPreview` 垂直 FOV = **`VerticalFOVLower:-7` ~ `VerticalFOVUpper:52`**（**这是在复刻 MID360 的真实 FOV**，天生朝上偏，向下只有 7°）。
- **盲区半径 = 无人机对地高度 ÷ tan(|下界|)**。3m 高 → **≈24.4m**。本会话实测：点云**最近点恒为 23.08m**（与理论吻合），且**前进 2m 后最近点仍是 23.08m**（纯平移不改变盲区）。
- **判据**：俯仰角分布全部 ≥ -7°、且**水平距离 min ≫ 0** → 是 FOV 盲区，**不是链路/算法坏了**（FAST-LIVO2 日志照常 `Update Voxel Map`）。
- **量化诊断脚本**：`scripts/cloud_geometry_analyze.py` —— 同时订阅桥原始点云与 FAST-LIVO2 输出，打印点数、xyz 范围、**俯仰角分布**、**水平距离 min/max**，并自动判读。
- **修法（改 `C:\\Users\\<USER>\\Documents\\AirSim\\settings.json`，改完必须关掉整个 `UE4Editor.exe` 重开，PIE 停/开不重读配置）**：
  1. 放大向下 FOV：`VerticalFOVLower: -7 → -25`（仿真探索友好，**代价：偏离实机 FOV**）
  2. 给 LiDAR 加安装倾角：`Pitch: 0 → -20`（模拟实机倾装）
  3. 飞低（1m 高 → 盲区降到 ≈8m；不改配置，但效果有限）
  4. 保持真实 FOV（论文讲"高保真"时用），测试时靠飞低
- **推荐双配置**：`settings.json`（实机复刻 `-7~52`，讲保真时用） + `settings_sim.json`（`-25`，跑探索/规划时用），按场景切换。

### 数字孪生里的判读（别把正常现象当故障）
- **★★ FAST-LIVO2 启动后头 ~45–60 秒的 `[LIO]: No point!!!` + `IMU Initializing: 3.3%` 是初始化瞬态，不是故障**（本会话为此白查了多轮）。源码根因：`src/LIVMapper.cpp:345` 的 `if (feats_undistort->empty()) { std::cout << "[ LIO ]: No point!!!"; return; }` —— `feats_undistort` 由 **IMU 去畸变**产生，IMU 没初始化完就为空，于是**每帧都打印 No point**。跑满 ~60s 自动恢复（`effective feature num: 9341, residual 0.0035` + `Update Voxel Map`）。**纪律：见到它就让它跑满 60 秒再判**，别先去查点云字段名/TwinSim.yaml/IMU 数值（本会话这些**全部正常**：字段 `x,y,z,rgb` ✓、`lidar_type:4` ✓、IMU `acc z=9.84` ✓、`use_sim_time=true` ✓ —— 答案只是"等够时间"）。同理 **`/twin/lidar` 有 10Hz ≠ `/cloud_registered` 有数据**；判"链路就绪"只看 **`/cloud_registered` 有没有 ~10Hz**（`twin_livo_fast.sh` 报的"耗时 1 秒"只是**发完启动命令**，不是链路就绪）。
- **`/cloud_registered` 是 "`camera_init` 世界坐标系下的地图"
- **`/mavros/local_position/pose`（PX4/EKF 估计）与 `/aft_mapped_to_init`（FAST-LIVO2 自己的里程计）是两条独立来源，诊断时别混**：本会话 PX4 报 z=2.99、LIVO 报 z=2.54（差 0.45m，正常），两者互印可判 LIVO 有没有跟上高度。
- **offboard 沿机头方向位移**（"让无人机前进 N 米"这类要求）：`scripts/offboard_move_forward_ros1.py` —— 读 pose 四元数算 yaw → 目标 = 当前 + forward×D（保持高度）→ 预发 setpoint 3s → 切 OFFBOARD → arm → `err<0.2m` 为止。实测：目标 2m → 实到 **1.855m**、z 稳住不动。
- **offboard 多段机动（前进→转向→再前进）**：`scripts/offboard_maneuver_ros1.py`（`SEGMENTS=[("fwd",10),("right",90),("fwd",20)]` 这种段列表，含 fwd/right/left/hold）。转向靠 **yaw setpoint**：`PoseStamped.pose.orientation` 填只绕 z 的四元数 `z=sin(y/2), w=cos(y/2)`；**转向段位置保持不变、只用 `TURN_SEC≈4s` 插值 yaw** → PX4 会原地转向。**"右转 90°" = yaw − 90°**（ENU 逆时针为正，写成 `("left",90)` 就是 +90°）。实测 `[fwd 10, right 90, fwd 20]`：x 走 +9.7m 再 y 走 −19.7m，**两段严格正交**、高度 2.98m 全程不变、每段误差 ~0.3m。
- **持续飞行（可设速度/距离）**：位置 setpoint **每 tick 递增** `d = min(d + SPEED*dt, dist)`（等于匀速直线，比速度环好调），**必须带安全上限**（`MAX_DIST`/`MAX_SEC`，到点即停再悬停）。实测 2 m/s 飞 40m：z 2.97→2.98 几乎零漂、指令与实际滞后仅 ~0.5m。用户说"一直往前飞"时**务必留上限**，别真的无限飞。
- **⚠️ 脚本一停，无人机会自动降落**（本会话实测 `mode: AUTO.LAND`、z 掉到 −0.43）——脚本退出就不再发 setpoint → PX4 的 **offboard 超时保护**退出 OFFBOARD，而 settings 里 `COM_OBL_ACT:1` 决定此刻是**降落**（不是悬停）。**这是安全行为、不是故障**；要让无人机留在空中，得**让脚本继续运行**，或在脚本末尾把 `set_mode` 切回 `AUTO.LOITER`/`POSCTL`。用户看到"它自己降下来了"先讲这一条。
- **飞行会让点云持续增长**（本会话 4726 → 5376 → 6179 点，随 40m 直线 + 多段机动逐段累积）→ 证明**建图是活的**（用户怀疑"点云没变"时用这个举证）；反过来，**无人机落地后 LiDAR 最近点骤降到 1.84m**（高度 0.4m → 盲区 ≈3m）——这是"盲区 = 高度 ÷ tan(7°)"最直观的反面证据。
- **ENU↔FLU 别误判成坐标错乱**：无人机沿 MAVROS/ENU 的 **+x（东）** 飞，而 FAST-LIVO2 的 `/cloud_registered` 地图沿 **−y（FLU 里 y=西）** 延伸 —— 标准 ENU↔FLU 旋转，**自洽无误**（桥发的 `twin_lidar_flu` 是 x=前/y=左/z=上的 FLU 系）。

## Windows↔WSL 调用与运维坑（高频）
- **在 Hermes 的 PowerShell 里调 WSL，别用 `wsl -d X -- bash -lc '...'` 传含中文/双引号/括号的命令**——PowerShell 会把它搞乱（本会话多次只回显 `===`，或报 `bash: -c: line 1: syntax error near unexpected token '('`）。**正确做法：把逻辑写成 `.sh` 脚本文件（如 `C:\Users\<USER>\foo.sh`），再 `wsl.exe -d X -- bash /mnt/c/Users/liyan/foo.sh`**（脚本内可随意用引号/中文）。无引号的简单命令可直接 `wsl.exe -d X -- <cmd>`。
  - **变体：PowerShell 会把命令里的 `2>/dev/null` 当成它自己的重定向** → 报 `out-file : 未能找到路径"C:\dev
ull"的一部分。`，命令根本没跑到 WSL（看着却像 WSL 侧出错）。**凡命令含 `2>` / 引号 / 中文 / 括号，一律 base64 传脚本**：
    ```powershell
    $b = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($s))
    wsl.exe -d ros1-twin -u <USER> -- bash -c "echo $b | base64 -d > /tmp/x.sh; bash /tmp/x.sh; rm -f /tmp/x.sh"
    ```
  - **`nohup ... &` 起的长流程会在 `wsl.exe` 返回时被 WSL 会话杀掉**（症状：临时目录都建好了，日志文件和进程却都没有 —— 像"启动了又没启动"）。→ **长流程直接前台跑**（Hermes 前台 timeout 可到 600s，够起整套后端）；确需后台就放 tmux 或 `setsid`。
  - **验证入口/一键脚本时，故意用干净 shell（不预先 source ROS）跑一次** —— "脚本没自己 source ROS"、"`set -u` 撞 ROS setup.bash"、`2>/dev/null` 这三类缺陷 `bash -n` **全都过**，只有真跑一遍才暴露（本会话 6 个缺陷里 4 个是这么抓到的）。
- **写 `.ps1` 辅助脚本（Windows 侧直接执行）时里面别写中文**：PowerShell 5.1 按 ANSI/GBK 读**无 BOM** 的 `.ps1`，中文变乱码并**破坏引号配对** → 报 `The string is missing the terminator: "` / `Missing closing '}'`（本会话实测：`write_file` 写的中文 `.ps1` 直接解析失败，报错行号还指到中文那段）。**`.ps1` 一律纯 ASCII**；中文说明放 `.sh`/Python 侧，或让 PowerShell 只干"枚举/调用"这类无中文字面量的活。（典型用途：`Add-Type` 调 Win32 `EnumWindows` 枚举 WSLg 合成器 `msrdc` 的窗口 —— 见 `scripts/enum_wslg_windows.ps1`。）
- **WSL 的 sudo 密码拿不到也不用问、不用猜**（是用户建发行版时自己设的）。需要 root 直接用 WSL 的**免密 root**：`wsl.exe -d <distro> -u root -- <cmd>`（apt 安装/改文件都行，无需密码）；用户忘了密码可重置 `wsl.exe -d <distro> -u root passwd <user>`。多数编译/配置工作根本不需要 sudo。
- **各 WSL 发行版各管各的**（本项目：`lyuav`=Ubuntu24.04/ROS2/PX4；`ros1-twin`=Ubuntu20.04/ROS1/FAST-LIVO2+twin_bridge）。查进程要分别进：`wsl -d <d> -- pgrep -a px4`（`windows` 的 Get-Process 查不到 WSL 内进程）。**PX4/桥在哪个发行版跑都不影响 AirSim**——PX4 走 TCP 4560、ROS 桥走 RPC 41451，两条独立链路，互不干扰。
- **用 VS Code 连 WSL（`code .` / Remote-WSL）—— 首次装 server 必须走代理**：装好扩展 **`ms-vscode-remote.remote-wsl`** 后，**在 WSL 里 `code .`** 就会拉起 Windows 侧 GUI、连上该发行版、并自动安装 `~/.vscode-server`。首次安装**要联网下 ~100MB（解压后约 650MB）**，国内直连极慢 → **先 `export https_proxy=http://127.0.0.1:7899 http_proxy=http://127.0.0.1:7899` 再跑 `code --version`**（这条命令本身就会触发 `Installing VS Code Server ... → Downloading`）。成功标志：`~/.vscode-server/bin/<commit>/` 存在 + `Compatibility check successful (0)`；连上后 **Windows 窗口标题 = `<文件夹> [WSL: <发行版>] - Visual Studio Code`**。⚠️ **别用 `code --list-extensions`/`code --version` 查东西**——它们会**拉起 GUI 然后挂住**（本会话直接卡到超时）；查扩展改**看目录** `Get-ChildItem "$env:USERPROFILE\.vscode\extensions"`。细节见 `references/vscode-wsl-connect.md`。
- **把成果推 GitHub（Windows 侧维护"含 Linux 脚本"的仓库）** —— 本会话实测的四个点：
  - **换行符**：Windows `core.autocrlf=true` 会把 `*.sh` 检出成 CRLF（在 WSL 里直接跑会挂），且让 `git status` 反复冒出**无内容差异的 "M"**（`git diff` 为空 = 假脏）。修：仓库加 **`.gitattributes`**（`* text=auto eol=lf`，另 `*.png binary` 等）→ `git add --renormalize .` → 提交推送；之后 status 稳定干净。
  - **推送必须走代理**：国内直连报 `Failed to connect to github.com:443 after 21104 ms`；用 `git -c http.proxy=http://127.0.0.1:7899 -c https.proxy=http://127.0.0.1:7899 push origin main`（Clash 端口 7899）。
  - **WSL 内没有 git 凭据**：在 WSL 里跑 `git ls-remote` / `push` 会**卡住等输入**（Windows 的 credential helper 在 WSL 不可用）→ 加 `export GIT_TERMINAL_PROMPT=0` 让它快速失败；**鉴权/推送放 Windows 侧做**。
  - **推完要核对**：`git rev-parse HEAD` 与 `git -c http.proxy=... ls-remote origin main` 的 SHA 一致才算真推上；`Everything up-to-date` = 已是最新（不是失败，别当报错重试）。
  - **写"推送后自检脚本"时当心 `git ls-files` 会转义非 ASCII 文件名**（中文名被输出成 `"docs/2026-09-10_\345\274\200\345\217\221..."`）→ `for f in $(git ls-files); [ -s "$f" ]` 会**误报"空文件"**（路径解不开）。改用 `while IFS= read -r -d '' f; do ...; done < <(git ls-files -z)`。本会话踩过：验证脚本因此假失败，差点去改一个内容完好的文件。
  - 收尾清理：**删 `.bak` 前先列出来**；早期会话留下的一堆临时诊断 `.sh` 若不确定用途，**移到 `桌面\_old_scripts_archive\` 归档**（可恢复）比直接 `rm` 稳；`find ~ -name '*.bak' -not -path '*/.git/*' -not -path '*/build/*' -not -path '*/devel/*' -delete` 可安全清备份。
- **⚠️ 一键/清理脚本里 `pkill -f <关键词>` 会误杀脚本自己——尤其脚本**文件名**含该关键词时**（本会话实测：一键脚本名 `twin_livo_rviz.sh`，内部有 `pkill -f rviz` → **把自己杀了**，刚打印 `[1/7] 清理旧节点 ...` 就 `Terminated` 退出，极难一眼看出）。**规避**：清进程优先用 **`pkill -x <进程名>`**（精确匹配进程名，不匹配整条命令行，`rviz`/`px4`/`rosmaster` 这种单个可执行名直接用它），或把模式写成不含连续目标串的正则（`pkill -f 'rv[i]z'`）。凡写"一键启动/清理"脚本，**自查：脚本名与内容里出现的每个词，是否会被自己的 `pkill -f` 命中**。（`ssh-remote-linux-ops` 记的是 plink `-m` 脚本自误杀的同源变体，可互参。）
  - **变体①（最阴，本会话连踩两次）：内联命令也会自杀。** 用 `wsl.exe -d X -- bash -c "... pkill -f roslaunch ..."` 时，**外层那个 bash 的 cmdline 本身就含 `roslaunch` 这个词** → 把自己杀了。**症状 = 输出全空 + exit 15（SIGTERM）**，完全看不出是自杀。**规避：清理逻辑一律写进 `.sh` 文件再 `bash /path/x.sh` 执行**（脚本进程的 cmdline 只是路径，不含这些词）。
  - **变体②：文件名撞词。** 辅助脚本取名 `ros1_start_mavros.sh` 而内部有 `pkill -f mavros` → 自杀。**文件名避开 `rviz`/`mavros`/`roslaunch`/`roscore` 这些词**，或直接改用 `pkill -x`。
- **一键启动脚本模板**：`templates/twin_oneclick_launch.sh`（PX4(tmux 保 tty) + 桥(幂等检测 `/twin` 有无数据) + FAST-LIVO2(`rosparam load` 含相机 `cam_model`) + RViz + **MAVROS 控制回路**（PX4 专线 `mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard` + geoid 自检 + `fcu_url:=udp://:14611@127.0.0.1:14610` + `/mavros/state` 自检）；清理全用 `pkill -x`；**配套关闭脚本 `templates/twin_oneclick_stop.sh`**）。改路径/话题/lidar_type 即可复用；用户偏好"给我一个 sh，我自己跑"。**⚠️ 若一键脚本不含 MAVROS，`hover_verify` 会永远卡在「等待 FCU 连接」**——那是 MAVROS 没起，不是脚本写错（本会话实测）。

## 一键启动链路排障（现象 → 根因，2026-09-12 实战）
启动脚本"看着卡住"或"桥起了没数据"时**按现象查**；完整证据链/原文日志/排查命令见 `references/startup-chain-troubleshooting.md`：
1. **桥进程在、`/twin` 一直无数据** → 先查 **PX4↔AirSim 的 TCP4560**。AirSim 配 `LockStep` 时**必须 PX4 连着才跑仿真**，断链就停在 `Waiting for TCP connection on port 4560`、**传感器零数据**。Windows 侧证据：`Blocks.log` 出现 `Exception sending messages to vehicle` + `TcpClientPort socket send failed with error: 10053` 后回落该等待行。判据：**`ss -tn | grep ':4560'` 有 established 才算连上**。**桥从启动到首帧约 30s**，别等 10s 判失败。AirSim 会**自动等重连**，修 PX4 那侧即可、**不用重启 UE**。
   - **★ 唯一判据 = `rostopic hz /clock`**（用户 2026-09-12 明确纠正：**"点云有时有时没有，不是没等 90s 的问题"**）。
     因果链：**tmux server 死**（`tmux ls` 报 `no server running on /tmp/tmux-1000/default`）→ **PX4 被 SIGHUP 杀** → 4560 断 → LockStep 不步进 → **`/clock` 停止** → 所有传感器话题全无数据。**等再久也没用**，别把"再等等"当解法。
     四个一眼判据：`rostopic hz /clock`（不走=冻结）/ `ss -tn | grep 4560`（无=未连）/ `pgrep -f bin/px4`（无=没起）/ `tmux ls`（无 px4 会话=server 挂了）。`rostopic echo` 报 **`Is /clock being published?`** 就是冻结的直白提示。
     已做进脚本：`twin_livo_fast.sh --full` 按 `4560 → /clock → /twin → /cloud_registered(无则自动重启 FAST-LIVO2，最多 3 次)` 逐环验证并报明确错误；`fuel_all.sh` 在 `/clock` 不走时直接 exit 并打印排查命令。详见 `references/fuel-planner-integration.md` §16。
2. **`RLException: run_id on parameter server does not match declared run_id`** → **残留 roslaunch 占着 `/run_id`**（多为以前跑过的 `roslaunch mavros ...`）。修：`pkill -f roslaunch`（**全部**，不是只清目标那一个）+ 等它真退出再起干净 roscore。
3. **`[3/7] 启动 roscore` 静默 1~3 分钟 ≠ 卡死，而且能根治：绕开 `roscore`** —— 逐个计时钉死元凶：**`roscore` 包装脚本单次 140s，但它内部真正干活的 `rosmaster`=2s、`rosout`=0s**。所以**别再等 roscore**，直接起这两者：`setsid rosmaster --core -p 11311 &` → 轮询 `rosnode list` 就绪 → `rosparam set /rosdistro|/rosversion` → `setsid rosrun rosout rosout __name:=rosout &` → **master 4s 就绪、`roslaunch` 完全正常（桥 1s 出数据）、全量启动 140s → 6s**。已排除：DNS（`getfqdn`/`gethostbyaddr` 几十 ms）、`ROS_IP=127.0.0.1`（设了无效）；CPU 被 `make px4_sitl` 抢只是**次因**。次优做法：**master 还在就复用**（别无脑重启 roscore）。等待循环**必须打印进度**（否则用户报"卡住"）；判"真卡没卡"：`pgrep -af <脚本名>` + **`ps --ppid <pid>` 看子进程走到哪**。⚠️ 别拿"`/tmp/roscore.log` 为空"当失败判据（roscore 输出本来就少），判据是 `rosnode list`。可直接复制的模板：`templates/twin_oneclick_fast.sh`（把这段内建进去了）。
4. **想让启动 ≤7 秒，靠的是"常驻"而不是"启动更快"** —— 配 **`twin_livo_fast.sh`（只补"缺失"的组件、不杀不重启；环境已就绪时 0 秒）** + **关闭脚本默认停 PX4+桥+算法+RViz、只保留 ROS master 常驻**（`--all` 才连 master 一起停）。**这是用户 2026-09 明确纠正过的取舍**：最初我把 master+PX4 都留着，用户回"**PX4肯定也要停吧，可以roscore常驻**"——**PX4 占资源、该停**；**master 只是轻量小服务、该留**（留它省掉那 140s）。设计一键脚本时别自己发挥成"都留"。**因为 master/PX4 重启一次就是分钟级**，所以用户抱怨"启动太慢"时**先查是不是 stop 脚本把 master 也杀了**。实测：清空后全量 6s，紧接着再跑 0s。
5. **WSL 里 `sudo shutdown now` 报 `System has not been booted with systemd as init system`** → WSL 无 systemd，**是正常限制不是故障**。关机在 **Windows PowerShell**：`wsl --terminate <发行版>`（单个）或 `wsl --shutdown`（全部）。收尾顺序 = 先 `twin_livo_stop.sh` 停 ROS 侧 → 再 terminate → 最后手动关 AirSim(Blocks)/QGC。
6. **RViz "打不开窗口" / Alt+Tab 切不过去** → 多半是 **WSLg 退化成 `[WARN:COPY MODE]`（CPU 软件合成）**，**窗口其实存在**（用 `scripts/enum_wslg_windows.ps1` 枚举 WSLg 合成器 `msrdc` 的窗口可证：会看到 `[WARN:COPY MODE] fast_livo2.rviz - RViz (...)`，visible/未最小化）。用户说的那个 "warning" 常是 **`ROS 1 End-of-Life is May 31st, 2025` 停服提示框**（与 RViz 无关）。修：`wsl --shutdown` 让 WSLg 重init；仍 COPY MODE 则注释掉 `.wslconfig` 里的自定义 `kernel=`（本机是 JetPack 内核）。详见 `references/startup-chain-troubleshooting.md` §⑥。**缩小/移动 WSLg 窗口（用户说「把 rviz 窗口弄小」）**：`scripts/enum_wslg_windows.ps1` 自带 resize 模式——`powershell -ExecutionPolicy Bypass -File enum_wslg_windows.ps1 rviz 120 80 960 640`（按标题子串定位 + `SetWindowPos` 改大小/位置，无需在 WSL 里装 wmctrl/xdotool）；WSLg 窗口就是普通 Windows 窗口，用户直接拖边缘也能缩，RViz 关闭时会记住窗口大小。⚠️ resize 前先确认窗口**已出现**（列表模式里能看到目标 title；若 `NOTFOUND` 多半是 rviz 还没起来，先 `bash ~/twin_livo_fast.sh` 起它再缩）。
> 一键启动脚本**最终形态 = 单个脚本 + `--full` 开关**（本会话把原来"全量 `twin_livo_rviz.sh`"与"快补 `twin_livo_fast.sh`"两个脚本**合并成一个**，用户问"twin_livo_rviz.sh还要保留吗？"时答案是**不留**——`fast` 已覆盖全量启动，把 rviz 版独有的"清残留 roslaunch / 等 TCP4560 / 等 `/twin` + 验证"降级成 `--full` 选项即可）：
> - **默认**（`bash ~/twin_livo_fast.sh`）：只补"缺失"的组件、不杀不重启 → 常驻时 **0-2 秒**
> - **`--full`**（`bash ~/twin_livo_fast.sh --full`）：先 `pkill -f roslaunch` 清残留（防 run_id 冲突）→ 启动 → **等 TCP4560 established + 等 `/twin` 出数据**并打印验证结果 → 异常时用（实测 4 秒）
> - 共性：**不杀在跑的 PX4**、**绕开 roscore 直接起 master**、全程进度输出。
> - 模板：`templates/twin_oneclick_fast.sh`（含以上两种模式）+ `templates/twin_oneclick_stop.sh`（配对关闭，默认保留 master）。

## 全自动 SITL 实验入口脚本（`run_auto.sh` 这类「一行出一组数据」的脚本）
用户要「全自动实验入口（检查/启动/起飞/飞行/降落/记录/绘图一条龙）」时，按 **`references/px4-sitl-experiment-automation.md`** 的契约写，别从头设计。硬要点：
- **环境自举**：脚本自己 `source /opt/ros/noetic/setup.bash`（+ twin_ws），**不依赖调用者的 shell**（否则 `rosmaster: command not found`）；且 **source 期间必须 `set +u`** —— ROS 的 `1.ros_distro.sh` 引用未定义变量，撞 `set -u` 直接报 `ROS_DISTRO: unbound variable` 秒退。
- **只清理自己启动的进程**：维护 `OWNED_PIDS` 账本**逆序** kill，**不用 `pkill -f`**；tmux 会话单独记标志。**未确认落地（试验 summary 里没有 `landed_disarmed` 事件）→ 保留 MAVROS/PX4/master 不杀**，并在终端明确告知"请人工确认飞行器状态"。
- **启动前拒绝「脏环境」**：已有 master / PX4 / MAVROS / 其他控制话题发布者 → **报错退出**（严禁双控制节点），**不替别人清理**。
- **「就绪」必须数据驱动**：`/clock` 在步进（LockStep 下唯一判据）+ `connected:True` + `armed:False` + `landed_state==1` + `use_sim_time=True`，再查 8 个 control setpoint 话题发布者为空。**别拿日志关键词当就绪判据**。
- **PX4 必须 `make px4_sitl_default none_iris`**（tmux 内）：直接调 `build/px4_sitl_default/bin/px4 -d <etc>` 会 `Error opening startup file, does not exist: etc/init.d/rcS`。
- **`Clock` 在 `rosgraph_msgs`**，`from sensor_msgs.msg import Clock` 会 ImportError。
- **MAVROS `setpoint_raw/local` 坐标语义（源码级已确认）**：`coordinate_frame` 只要不是 `BODY_*`，MAVROS **一律把输入当 ENU 并自己转 NED** ⇒ **ROS 侧填 ENU 值，绝不手工转换**（手工转＝双重转换）。掩码 **2503** = 忽略位置/加速度/yaw_rate、用「速度+yaw」、**`FORCE`(512) 不置位** ⇒ `acceleration_or_force` 是**加速度**语义 —— **别照抄 FUEL 控制桥 raw 分支把 512 当"忽略 force"**。
- **起飞前探空域**：`scripts/airsim_airspace_probe.py`（纯 socket+msgpack，**不需要 airsim pip 包**）—— 取 LiDAR 点云旋到世界 NED，按 ENU 方向给出「走廊内点数 / 最近障碍距离」。本机 Blocks **只有 `.uproject`、没有 `Blocks.exe`** ⇒ 无法自动进入 Play，**"用户先 Play"是唯一保留的人工前置**，脚本负责**检测**（RPC `ping` + `simIsPaused`）并提示，**别猜启动方式**。
- **跑出第一组数据之后（别停在"飞通了"）**：进入**批量重复 + 测量口径**阶段，见 **`references/control-experiment-metrics.md`**。四条最常用的：① **每轮试验前必须 AirSim `reset` 复位场景**，否则第二次起点 = 上次降落点 ⇒ **两组不可作严格配对**；② 交错顺序（`0 200 200 0 | ...`）抵消时间漂移，**失败运行也全部保留**（`*_FAILED_runN/` + `FAIL_REASON.txt`）；③ 报数必须**拆口径** —— `t_send/t_decel/t_low/t_stop` 四事件，其中 **`cmd_latency≈85 ms` 跨延迟跨速度恒定**（这才是"应用层发送 ≠ 物理制动"的量化值），而"低速→停稳 0.50 s"只是 **dwell 人为窗口**（物理制动完成看 `t_low`）；④ **名义延迟 ≠ 实测延迟**（50 Hz 量化 ⇒ 名义 200 ms 实测 214 ms），两个口径都报并做物理自洽核对；拟合自变量要用**实测速度**且取"请求前 0.25 s"均值。
- **★ 安全判据 / 可恢复命题的两条铁律**（用户明确纠正过，别再写错）：① **硬安全约束（碰撞、围栏）在定义上不允许越界**，可以"有界越界"的**只能是任务容差** —— 曾把研究定位写成"允许学习器有界越界"属概念错误；② 加"虚拟边界"时必须**真正参与在线触发**（用飞行中在线可测的量判定并立即发零），**事后判定不算**；比较判据要给全「越界率 / 越界量 / 边界余量 / 触发提前量」。承载判据比较**新写一个 trial 节点，别改已有实验节点**。
- **交付分四级说**：已编写 / 离线通过 / **后端联通（真起后端并读到就绪信号）** / 实际飞行 —— 没做的就写没做，别用"跑通了"糊过去。

## 判读
- **双击 `.uproject` 弹 `Installation Not Found / No Unreal Engine installations found. Would you like to locate one manually?`** → UE 装在非默认盘且**未注册到系统**（`.uproject` 靠注册表里的 `EngineAssociation`（如 `"4.27"`）找引擎，E 盘安装常不注册）。**修复**：点 `Yes` → 手动定位到 `E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe`（一次性注册，之后双击 `.uproject` 可开）；或**绕过注册直接用编辑器开**：`Start-Process "E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe" "E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject"`。**重启 UE** 也可用此命令（先 `Stop-Process -Name UE4Editor -Force` 再 Start-Process，注意 -Force 会丢未保存改动）。
- 闪退无窗口(进程消失) → 模块缺失：先跑 Build.bat(步骤5)编译模块。
- `NetFxSDK install dir` → 装 .NET 桌面开发(步骤2)。
- `SuvCarPawn_C` / `AutomotiveMaterials` 加载失败 → 补下 car_assets(步骤6)。
- `Failed to load asset class` 车辆 → settings.json 改 Multirotor/QuadX(步骤7)。
- **`There were no compatible vehicles created for current SimMode!` + `SimHUD::getCamera()` 空指针崩溃(`EXCEPTION_ACCESS_VIOLATION @0xb8`)** → `SimHUD.cpp:155` 里 `player_controller->GetPawn()` 返回空（无人机 pawn 没被 spawn）。**首要根因 = settings 的 `VehicleType` 值写错**：
  1. `kVehicleTypeSimpleFlight="simpleflight"`、`kVehicleTypePX4="px4multirotor"` 是**车类型**；**`"QuadX"` 只是机架 frame 类型，不是有效 VehicleType** → AirSim 认不出 → 无车辆（实测：把 `VehicleType` 从 `QuadX` 改成 `SimpleFlight` 后无人机立刻出现）。
  2. **修复**：`VehicleType` 用 `"SimpleFlight"` 或 `"PX4"`，自定义 vehicle 即正常 spawn。
  3. 源码机制（参考）：`AirSimSettings.hpp:906` 写 `Vehicles` 会 `vehicles.clear()`；`SimModeBase.cpp:622` `pawn_paths.at(getVehiclePawnPathName(vehicle))`，type 不对时该 key 不在 `pawn_paths` → 无车辆。`getCamera` 崩溃是**无 pawn 的下游症状，不是 VS 编译/Content 兼容问题**。
  - **对 PX4 的影响（重要更正）**：`VehicleType:"PX4"` 是**合法车类型** → **不用改 AirSim 源码就能接 PX4**（先`make px4_sitl`供给 MAVLink，AirSim 侧 `VehicleType:"PX4"` 连它）。之前误以为 PX4 被 `getVehiclePawnPathName` 挡住要改源码——实测 `VehicleType` 写对即可过，属误判。
  - 判根：日志 `no compatible vehicles` + `SimHUD.cpp:155 GetPawn()==null`（GetPawn 空是直接证据）。看 settings 的 `VehicleType` 是否写成了 `QuadX`/其它非车类型。
  - ⚠️ 误区：**这不是 UE4.27 Content/资产兼容问题，最小 settings 就能出无人机**；`--no-full-poly-car` 当年跳过的 SUV 资产要用 `--no-full-poly-car` 但需要**另下 car_assets.zip** 补 SUV 材质（见步骤 6），否则 SuvCarPawn/AutomotiveMaterials 才报错。
- UE 窗口加载慢/黑屏 → WSLg/GPU 权限问题（可参考 WSL 镜像网络 + /dev/dri）；Windows 本机 UE 一般无此问题。
