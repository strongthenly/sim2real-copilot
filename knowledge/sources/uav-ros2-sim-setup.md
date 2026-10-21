---
name: uav-ros2-sim-setup
description: 在 WSL(Linux) 从零搭建 ROS2+Gazebo+PX4 无人机探索仿真环境（含国内网络/代理、MAVROS、LiDAR world、论文主流选型）。用户常用。
version: 1.0.0
---

# UAV ROS2 仿真环境搭建

在 WSL (Ubuntu) 搭建 ROS2 Jazzy + Gazebo Harmonic + PX4 SITL + MAVROS 的无人机探索仿真平台，供自主探索算法（frontier 等）开发 / 发论文。

## 触发场景
- 用户要"无人机仿真环境"、"无人机跑算法"、"自主探索"、"发论文"
- 搭建/安装 ROS2、Gazebo、PX4、MAVROS
- 国内网络装 ROS2/PX4（代理、镜像）

## 环境底座（本会话实测栈）
- WSL2 Ubuntu 24.04 (`lyuav`)；ROS2 Jazzy；Gazebo Harmonic (gz-sim8)；PX4-Autopilot (SITL `gz_x500`)；MAVROS
- 探索算法放 `~/ros2_ws/src/<pkg>`（colcon）；用户 GitHub 仓库 `strongthenly/-lyuav`
- 关键连通：`/mavros/state.connected: true` = PX4↔ROS2 打通

## 关键 Pitfalls（最易踩，务必记住）
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

## AirSim 与 WSL 共存架构（用户明确想"WSL 操作 + Windows 端 AirSim"）
- AirSim 必须 Unreal Engine → **只能 Windows 本机跑，WSL 跑不了 Unreal GUI**。常见架构 = WSL 算 ROS/算法 + Windows 跑 Unreal/AirSim 高保真仿真。
- 靠 `networkingMode=mirrored`（WSL 与 Windows 网络栈共享），**WSL 里 `127.0.0.1:<AirSim RPC 41451>` 可直接连 Windows 的 AirSim**——这是协同命门，恰好前面已配镜像模式支持。
- AirSim 官方 ROS bridge（`airsim_ros_pkgs` 或 ROS2 版）跑在 WSL，通过 RPC 连 Windows AirSim，把 AirSim 的 LiDAR/相机/位姿发布成 ROS 话题、也收 ROS 命令控制车辆。PX4 留 WSL（MAVLink 指挥 AirSim 无人机）。

### AirSim 实际安装（Windows 侧）要点
- **UE4.27 只支持 VS 2019/2022（16/17 工具集），不支持 VS 18（2026）**。误装 VS18 会导致 AirSim/UE4.27 编译不兼容。装 VS 时选 **"使用 C++ 的桌面开发" + Windows SDK**（18 版留着不影响，但 build 必须用 VS2022 17）。定位 VS 用 `vswhere -all -products * -property installationPath`（别 `&` 调用，会被工具误判为后台——写成 .cmd 用 `cmd /c` 跑，且 .cmd 里**只用英文**避免 GBK 乱码）。
- **AirSim clone 用默认分支**：`git clone https://github.com/microsoft/AirSim.git E:\AirSim`（`--branch master` 会报 "Remote branch master not found"）。
- **AirSim `build.cmd` 构建的是 C++ 库（AirLib/rpclib/MavLinkCom），不需要 UE 路径**（UE 只用于后续打开 Unreal 项目）。但**必须从 "x64 Native Tools Command Prompt for VS 2022" 运行**（需要 `%VisualStudioVersion%`），否则报 "run from x64 Native Tools Command Prompt for VS 2022"。可用脚本 `call vcvarsall.bat x64 && cd /d E:\AirSim && build.cmd` 后台跑。
- **build.cmd 用 iwr 下载 rpclib/eigen（直连 GitHub/GitLab），国内直连 GitHub 时通时断**（git clone 成功但 iwr 卡死）→ 先 `curl.exe -L` 手动预放依赖到 `E:\AirSim\external\rpclib\rpclib-2.3.0\` 与 `E:\AirSim\AirLib\deps\eigen3\`，再 `build.cmd --no-full-poly-car`（跳过 ~37MB SUV 资产下载）。完整配方见 `references/airsim-install-windows.md`。
- **UE 打开 Blocks 的硬坑**（详见 reference）：① UE4.27 编辑器 exe 是 `UE4Editor.exe`（不是 UE5 的 `UnrealEditor.exe`）；② 首次打开若 UE 闪退（无窗口/无进程），模块会缺失（`Incompatible or missing module: Blocks / AirSim`）→ **手动跑 `Build.bat BlocksEditor Win64 Development -Project="...Blocks.uproject" -WaitMutex`** 编译；③ UBT 报 `Could not find NetFxSDK` → **VS 加 ".NET 桌面开发"工作负载**；④ `dinput8.lib/dxguid.lib ... not resolvable` 是警告非致命；⑤ UE 在 E 盘时 `update_from_git.bat` 报注册表错误（rungenproj）非致命。

## 用户偏好 / 环境事实（本类任务）
- **存储放 E 盘，不占 C 盘**：大文件 / Unreal / AirSim 装 E 盘；WSL 发行版若占满 C 盘，可 `wsl --export` + `wsl --unregister` + `wsl --import` 迁到 E 盘（迁移配方见 `references/wsl-migration-and-airsim.md`）。

## 论文主流平台（选型依据，用户发论文参考）
- 主流无人机探索（FUEL 1471★ / RACER 789★ / EPIC 236★）用 **Gazebo + 简化四旋翼或 PX4 + MAVROS + FAST-LIO + frontier**，**不用 AirSim**。
- AirSim(Unreal) 只在要高保真视觉时用（需 Unreal：Windows、几十 GB、好 GPU），**非探索主流**。
- 用户环境 → `Gazebo+PX4+MAVROS` 是论文主流姿势。

## 用户偏好（本类任务）
- 用户**坚持无人机**（拒用车/小车替代），要**真实外观机型**（非光杆 box），要**能随意介入算法**（探索算法做成独立可改节点）。
- 指挥官模式：默认远程命令用户亲手跑，agent 给命令+判读；说"你来跑/帮我做"才 agent 执行。每次 1-3 条 + 预期输出。
- 改文件先讲方案+备份+贴 diff+可回滚；讲原理用大白话+类比。
- 反复失败/高强度协作后用户会说"今天先这样"收工——停止推进，简洁总结进度并落记忆。

## 相关 references
- `references/domestic-ros2-install.md` — 国内装 ROS2/Gazebo/PX4 完整命令 + WSL2+Clash 网络详细
- `references/px4-mavros-vs-uxrce.md` — PX4-ROS2 通信选型（MAVROS vs uXRCE）+ geoid + offboard
- `references/gazebo-quad-lidar.md` — Gazebo 四旋翼+3D LiDAR world + MulticopterVelocityControl + LiDAR 几何
- `references/wsl-migration-and-airsim.md` — WSL 发行版迁到非 C 盘（export/unregister/import + socket 警告说明）+ WSL(ROS)/Windows(Unreal AirSim) 共存架构
