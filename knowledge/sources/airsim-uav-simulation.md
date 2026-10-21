---
name: airsim-uav-simulation
description: "在 Windows 上用 AirSim + Unreal Engine 搭建高保真无人机仿真环境（复现实机/视觉/发论文）。覆盖 UE 版本选择、VS 工具链、AirSim 构建、settings.json 配置、与 PX4/ROS(WSL) 协同架构，以及**把真实点云地图（LAS/PCD）做成可飞可撞的 UE4 场景**（OBJ 坐标约定、Python 自动化建关卡、PBR 材质、碰撞体、起降坪、场景后期就地更新）。"
version: 1.0.0
author: Hermes Agent
license: MIT
platforms: [windows]
metadata:
  hermes:
    tags: [airsim, unreal, uav, simulation, drone, high-fidelity, ue4]
---

# AirSim UAV Simulation (Windows + Unreal)

## 何时使用
- 要用 **AirSim（微软，基于 Unreal Engine）** 做**高保真**无人机/机器人仿真，用于复现实机、视觉/相机、或发论文的高保真对比。
- 需要把 AirSim 当"环境层"，外部（WSL）跑 ROS + 算法，Windows 跑 AirSim/Unreal 仿真。或接 PX4 真飞控。
- **写/改任何"启动 AirSim + PX4 + ROS 后端"的脚本、或做 SITL 实验自动化之前，先读本 skill**（尤其下面 §③ 启动 PX4 SITL 与文末坑表）—— 别自己猜二进制路径、端口、消息掩码或启停方式，这里都有实测答案；实验编排（一键脚本骨架、就绪判据、安全清理、配对公平性）见 `references/px4-sitl-experiment-automation.md`。
- **拿一份真实点云地图（`.las` / `.ply` / `.pcd`，例如 odin1 手持扫描的松树林）要在里面跑仿真/探索** → 先读
  `references/pointcloud-map-to-simulation-world.md`，并跑 `scripts/las_analyze.py` 拿到决策数字。
  **先记住这条硬约束：点云本身不能做碰撞体** —— 必须先决定「转成几何搬进 UE4（有物理、会撞）」还是
  「绕过 UE4 当传感器回放（无物理）」，否则方案一定跑偏。
  **决定走 UE4 路线后 → 读 `references/ue4-scene-build-and-edit.md`**（OBJ 坐标约定、自动化建关卡、
  PBR 材质、隐藏碰撞体、起降坪、场景后期"就地更新不新建关卡"）。
- 与 Gazebo 对比：Gazebo 轻/好复现；**AirSim 视觉保真高但要 Unreal**（重，几十 GB + VS 工具链）。

## 核心前提：AirSim 必须 Unreal，WSL 跑不了
- AirSim 基于 Unreal Engine（微软），**Unreal 是桌面 GUI 应用，WSL 里跑不了**。必须在 **Windows（或原生 Linux）本机**搭。
- **协同架构**（WSL 算 ROS + Windows 跑 AirSim）可行：
  ```
  【Windows】 Unreal + AirSim(高保真环境+传感器) + 可选QGC
        │ AirSim RPC(localhost:41451) / MAVLink
  【WSL】  AirSim ROS bridge ↔ ROS2 算法 ↔ PX4 SITL(MAVROS) -> 指挥 AirSim 里的无人机
  ```
  关键：WSL 用 `networkingMode=mirrored` 后，WSL 的 `127.0.0.1` 与 Windows 的 `127.0.0.1` 互通 → WSL 里能连 Windows 的 AirSim RPC（这是协同命门）。
- 数据量大：Unreal 引擎几十 GB + 仿真输出 → **装到非系统盘（E 盘）**，别占 C。

## 版本选型（关键决策）
- **弱显卡（如 RTX3050 4GB / 核显）用 Unreal Engine 4.27**；UE5.x 高保真非常吃力、易崩。
- **UE4.27 的编辑器可执行文件是 `UE4Editor.exe`**（不是 UE5 的 `UnrealEditor.exe`）！路径 `...\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe`。用错名字就"找不到"。
- **VS 用 2022（17.x）**，**不要用 VS 18 / VS 2026**：AirSim `build.cmd` 明确要 VS2022（cmake `-G "Visual Studio 17 2022"`），UE4.27 官方只支持 VS2019/2022（工具集 v142/v143）。装错版本判别：`vswhere.exe -all -products * -property installationPath`（VS2022 是 `...\\2022\\Community`,VS18 是 `...\\18\\Community`）。
  ⚠️ **VS18 能"编译成功"但"运行时崩"**：UE 的 UBT 自动挑"最高版本"的 VS → 用 VS18(MSVC 14.51) 编译 AirSim 插件，编译**不报错**、甚至 `exit 0` 通过，但产出的 DLL **运行时 ABI 不兼容** → `getCamera()` 空指针崩溃(EXCEPTION_ACCESS_VIOLATION @0xb8)。判别：Build.bat 日志出现 `toolchain (...\\18\\Community\\...\\MSVC\\14.51)` 或 `Detected compiler newer than Visual Studio 2019` → **卸载 VS18（保留 VS2022）**，重跑 `Build.bat`（日志应变为 `Visual Studio 2022 ... MSVC\\14.44`），再 Play。
- **AirSim 分支/版本**：clone 默认分支（AirSim 现为 **`main`**，已不是 `master`）。⚠️ **"切稳定 tag 修崩溃"是陷阱**——实测 `git tag` 里 **`v1.8.2` 与 `main` HEAD 指向同一个 commit**（`git log --oneline -1` 相同，clone 的就是 v1.8.2），切 tag 等于没切、白编译。遇到运行时崩溃，**先按下面 "settings 没 spawn 出 vehicle" 或 "VS 工具链" 排查，不要先归因版本/换 tag**。

## 构建关键点（AirSim，Windows）
- 依赖：**UE4.27(装E盘) + VS2022(含 C++ 桌面开发 + .NET 桌面开发) + Windows SDK**。
- `build.cmd` 必须从 **x64 Native Tools Command Prompt for VS 2022** 运行（否则报 `You need to run this command from x64 Native Tools Command Prompt for VS 2022`）。
- **缺 `.NET Framework SDK 4.6+`** 会报 `Could not find NetFxSDK install dir; this will prevent SwarmInterface` → VS 里勾 **".NET 桌面开发"** 工作负载。
- `build.cmd` 用 PowerShell `iwr` 下载依赖（rpclib/eigen）直连常失败 → 用 `curl -x <代理>` **预放**：
  - rpclib → `external\rpclib\rpclib-2.3.0\`（zip 来自 `github.com/rpclib/rpclib/archive/v2.3.0.zip`）
  - eigen → `AirLib\deps\eigen3\Eigen`（zip 来自 `gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip`）
  - `build.cmd --no-full-poly-car` 跳过 SUV 车资产(37MB，无人机不需要)，但这会导致 SUV 材质/pawn 缺 → 见下。
- `build.cmd` 只编 C++ 库（AirLib/rpclib）+ copy 插件源码；**AirSim 的 Unreal 插件 DLL 要 UE 打开/或 `Build.bat` 编译时才生成**。

## UE 打开 Blocks 项目 + 编译插件
- UE 打开 `.uproject` 报 `Incompatible or missing module: Blocks/AirSim` = 模块没编 → 别只靠打开时自动补，用：
  ```
  "<UE>\\Engine\\Build\\BatchFiles\\Build.bat" BlocksEditor Win64 Development -Project="<AirSim>\\Unreal\\Environments\\Blocks\\Blocks.uproject" -WaitMutex
  ```
  （target = `<Project>Editor`；成功生成 `UE4Editor-Blocks.dll` + `UE4Editor-AirSim.dll`。）
- UE 启动闪退无窗口 → 看 `Saved\Logs\Blocks.log` 尾部（崩溃栈/`Incompatible module`）。

## settings.json 配置（关键，避开坑）
路径：`%USERPROFILE%\Documents\AirSim\settings.json`（AirSim/PX4 都读这里）。
- **多旋翼（无人机）而非车辆**——默认会 spawn 一辆 SUV 车（`SuvCarPawn`/`VehicleAdv/SUV/...` 加载失败之源）。多旋翼配置关键：**`SimMode: Multirotor` + `VehicleType` 用「车类型」常量 `"SimpleFlight"` / `"PX4"`**（`kVehicleTypeSimpleFlight="simpleflight"`、`kVehicleTypePX4="px4multirotor"`，大小写不敏感）：
  ```json
  { "SettingsVersion": 1.2, "SimMode": "Multirotor",
    "Vehicles": { "Drone_1": { "VehicleType": "SimpleFlight" } } }
  ```
  ⚠️ **`VehicleType` 千万不要写 `"QuadX"`**——那是**机架 frame 类型**，不是有效车类型 → AirSim 认不出 → 报 **`There were no compatible vehicles created for current SimMode! Check your settings.json`**（实测：把 `QuadX` 改成 `SimpleFlight` 后无人机立刻出现）。`FirmwareName`（内置飞控，默认 SimpleFlight）**不必写**，写对 `VehicleType` 即可 spawn。
- ⚠️ **`getCamera()` 空指针崩溃通常只是"没 spawn 出车辆"的【下游症状】，不是它本身的原因**。诊断顺序：
  1. 看 `Blocks.log` 尾部有无 `no compatible vehicles ... Check your settings.json` → **有**，则根因是 settings 的 `VehicleType` 值写错（多为写成了 `QuadX` 机架类型）。**修法：`VehicleType` 改成 `"SimpleFlight"` 或 `"PX4"`（车类型常量）**，不是加 Cameras、更不是版本 bug。
  2. 无该句但 Build.bat 用了 VS18(MSVC 14.51) → 卸载 VS18 用 VS2022 重编。
  3. 都排除才考虑版本/兼容。
  （本会话实测：先加 Cameras 仍崩、先切 v1.8.2 tag 无效（v1.8.2==main 同 commit）、先归因缺 FirmwareName 也不对——真相是 **`VehicleType` 由 `QuadX` 改成 `SimpleFlight` 后无人机立刻出现**。）
- 无人机**可配 Cameras**（可选，用于视觉/相机仿真，`getCamera` 能拿到相机才显示）：`"Cameras":{ "0":{ "CaptureSettings":[{"ImageType":0,"Width":640,"Height":480,"FOV_Degrees":90}] } }`。
- 改了 settings.json → UE 里 **Esc 停止 + 重新 Play** 才会重读（或重启 UE）。

## 复刻实机外形（换成实机模型 / 自定义无人机 pawn）
- **推荐：新建自己的无人机 pawn 蓝图 + 用 settings 的 `PawnPath` 指向它**（别直接改 AirSim 自带的 `BP_FlyingPawn`——它在插件内容里，且官方提醒"改插件 content 是改二进制、要小心"）：
  ```json
  { "Vehicles": { "PX4": { "VehicleType": "PX4Multirotor", "PawnPath": "CustomDrone", "...": "..." } },
    "PawnPaths": { "CustomDrone": { "PawnBP": "Class'/Game/CustomDrone/BP_CustomDrone_GroundSafe.BP_CustomDrone_GroundSafe_C'" } } }
  ```
  流程：实机 FBX 导入 `/Game/...` → 在该 pawn 蓝图里设 mesh + 按实机尺寸缩放 + 归中 + 加地面碰撞。**外观只影响视觉，飞行仍由 PX4/SimpleFlight 决定**（正是要的：外形像实机、控制是真飞控）。
- **`BP_FlyingPawn` 在 Content Browser 里默认看不到**——AirSim 官方 `docs/working_with_plugin_contents.md` 明说 *"Plugin contents are not shown in Unreal projects by default"*。要显示：Content Browser **右下 `View Options` → 勾 `Show Plugin Content`**；或直接在**搜索框搜 `BP_FlyingPawn`**（最快）；或路径栏输 `/AirSim/Blueprints`。看不到插件内容 ≠ 资源不存在。
- 导入实机 FBX：Content Browser **Import**。Blender 导出的常见是 **cm、Y-UP(RH)** → `导入统一缩放` 保持 **1.0**（UE 内部单位也是 cm）+ 勾 `转换场景`(Y-up→Z-up) + `Import Materials`。**别导进 `/AirSim/...`（插件目录）**，导到项目 `/Game/...`。几十 MB 高面数模型会让弱显卡(3050)实时卡顿，必要时减面/LOD。

## PX4 / ROS 协同（接真实飞控）
- 接 PX4 用 **MAVROS（MAVLink）**，别用 uXRCE/Fast-RTPS（Jazzy 上 fastdds3/fastrtps2 不可共存的死结，见 `wsl2-ros2-sim-platform`）。
- AirSim 官方 ROS bridge（`airsim_ros_pkgs`/ROS2 版）在 ROS 侧经 RPC 连 Windows AirSim，把传感器/状态发布成 ROS 话题、也接收命令。

### AirSim ↔ PX4 SITL 对接（官方 px4_sitl.md，多步）
架构：`PX4 SITL(WSL) ←MAVLink→ AirSim(Windows,VehicleType=PX4) ←QGC/MAVROS操控`；跨系统靠 WSL **镜像网络**（Windows ↔ WSL 的 127.0.0.1 互通），AirSim 默认连 PX4 的 `127.0.0.1`。

**① PX4 版本硬要求**：AirSim 的 PX4 集成**只支持 PX4 `v1.11.x`**（`px4_sitl.md` 明确 `git checkout v1.11.3`）。**v1.18+/新 PX4 接口不兼容**（实测你 PX4 是 `v1.18.0-beta1` 接不上）。须**单独 clone v1.11.3**（`git clone --branch v1.11.3 --recursive`，别动现有新 PX4，新 PX4 留作他用）。

**② 编译 v1.11.3：Ubuntu 20.04(gcc9) 原生可编；Ubuntu 24.04(gcc13) 才必须用 gcc-10**：v1.11.3 时代的 `math::max(PTHREAD_STACK_MIN(长整), …(整))` 靠隐式类型转换，**GCC 13（`-Wfatal-errors`）直接拒绝** → `WorkQueueManager.cpp:257 error: no matching function for call to 'max(long int,int)'`。装 `gcc-10 g++-10` 后：
  ```bash
  sudo apt install -y gcc-10 g++-10
  cd ~/PX4-AirSim && rm -rf build/px4_sitl_default && CC=gcc-10 CXX=g++-10 make px4_sitl_default none_iris
  ```
  ⚠️ **`CC/CXX` 环境变量只在 CMake 首次 configure 时生效**——若 build 缓存已存在（UBT/ccache 用了 `/usr/bin/c++`=gcc-13），改环境变量无效，必须先 `rm -rf build/px4_sitl_default` 再 configure（日志应变 `The CXX compiler identification is GNU 10.5.0`）。
- **编译依赖**：先 `apt install -y ninja-build`（PX4 构建用 ninja）+ `python3 -m pip install -r Tools/setup/requirements.txt`（jinja2/empy/pyyaml/cerberus/pyros-genmsg…）；缺 ninja/empy 会直接 build 失败。装 apt 包**免密**：`wsl -d <distro> -u root -- apt-get install -y ninja-build`（不用知道 WSL 用户密码）。

**③ 启动 PX4 SITL**：`make px4_sitl`（不带模型）**只编译固件**（输出 Generating SBOM，不启动、无 `pxh>`）；要启动须 `make px4_sitl_default none_iris`（=`pxh>` + `INFO [simulator] Waiting for simulator to connect on TCP port 4560`）。
- ⚠️ **PX4 SITL 必须在【带 tty 的会话】里跑**：它是交互式 `pxh>` shell，**后台/无终端运行（stdin EOF）会立刻退出**（日志末尾 `pxh> Exiting NOW.` + `MAKE_EXIT=0`）。从 agent 用 `background=true` 跑会秒退（连上 AirSim 一瞬间又掉线）→ 用 **tmux**（`tmux new -s px4` 里跑、`Ctrl+B` 再 `D` 脱离、`tmux attach -t px4` 回看；装：`wsl -d <distro> -u root -- apt-get install -y tmux`），或**让用户在自己的终端前台跑**。判断"编译+启动一体"：`make px4_sitl_default none_iris` 会编译完**直接启动** PX4。
- ⚠️ **AirSim 与 PX4 是【双向等对方】**，谁是 server 看日志：AirSim(Blocks.log) 打 `Waiting for TCP connection on port 4560, local IP 127.0.0.1`（AirSim 在等/监听）；PX4 打 `Waiting for simulator to accept connection on TCP port 4560`。**两端都在等是正常的初始态**；只有 PX4 日志出现 **`INFO [simulator] Simulator connected on TCP port 4560.`** 才算真连上。

**④ AirSim settings（官方 PX4 SITL 格式）**——注意 `VehicleType` 是 **`"PX4"`**：
  ```json
  { "SimMode":"Multirotor", "ClockType":"SteppableClock",
    "Vehicles":{ "PX4":{ "VehicleType":"PX4Multirotor", "UseSerial":false, "LockStep":true,
      "UseTcp":true, "TcpPort":4560, "ControlPortLocal":14540, "ControlPortRemote":14580,
      "Sensors":{ "Barometer":{ "SensorType":1, "Enabled":true, "PressureFactorSigma":0.0001825 } },
      "Parameters":{ "NAV_RCL_ACT":0, "NAV_DLL_ACT":0, "COM_OBL_ACT":1,
                     "LPE_LAT":47.641468, "LPE_LON":-122.140165 } } } }
  ```
  （`NAV_RCL_ACT/NAV_DLL_ACT=0` 关 RC 失效保护、`COM_OBL_ACT=1` offboard 完悬停、`LPE_LAT/LON` 与 OriginGeopoint 一致 → 快速 GPS 定位。）
- 需在 Windows **防火墙放行 TCP 4560 + UDP 14540**；每停一次 Unreal app 要重启 px4 app。
- 验证成功：PX4 窗口出现 `INFO [simulator] Simulator connected on UDP port 14560` + `INFO [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)` + `EKF commencing GPS fusion`。

**⑤ `LockStep:true` + `PX4Multirotor` ⇒ AirSim 会卡在【等 PX4】（最常踩）**
只要 settings 是 PX4 车种 + LockStep，AirSim Play 后**不会自己跑**，而是阻塞在 Blocks.log：
`LogTemp: Waiting for TCP connection on port 4560, local IP 127.0.0.1`
→ **PX4 没起，AirSim 就不出无人机、不出传感器**（现象：ROS 桥能连上 41451，但拿到的传感器无数据/时间戳不动，因为 `SteppableClock` 要 PX4 步进）。别误判成桥坏了。
- 只想跑**感知/SLAM（LIO/探索预览）不需要 PX4**时：把车辆换成 AirSim 自带飞控即可自己跑——`"VehicleType": "SimpleFlight"`，并**不写** `LockStep`/`UseTcp`/`TcpPort`/`ControlPort*`/`ClockType:SteppableClock` 等 PX4 专用项（Sensors/Cameras/PawnPath 保留）。
- 要用真 PX4：**先起 PX4 SITL**，AirSim 日志出现 `Simulator connected on TCP port 4560` 才算初始化完成。

**⑥ MAVROS 连不上 PX4（`VER: broadcast/unicast request timeout`）＝ 端口被 AirSim 占了**
AirSim 的 PX4 车种用 `ControlPortLocal:14540 / ControlPortRemote:14580`（= MAVROS 标准 `fcu_url`），**二者撞车** → MAVROS 发出去没人应（`VER: ... request timeout` → `your FCU don't support AUTOPILOT_VERSION`）。
- **修法：在 PX4 `pxh>` 里另开一条 MAVLink 专线给 MAVROS**：
  ```
  mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard
  ros2 launch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610
  ```
  （`-u` 本地端口、`-o` 远端端口；报 `port 14600 already occupied` 就换号。验证 `ros2 topic echo /mavros/state --once` → `connected: true`。）
- 或不用 QGC 时改走 GCS 通道：`fcu_url:=udp://:14550@127.0.0.1:18570`。

**⑦ 用 MAVROS 发 offboard 让 AirSim 无人机按算法飞（ROS2 最小可用）**
`/mavros/setpoint_position/local` **20Hz 持续发 setpoint** → 流满 **~2s 后**再 `/mavros/set_mode` 切 `OFFBOARD` → `/mavros/cmd/arming`。要点：**setpoint 必须先于模式切换持续流**（PX4 硬要求），否则拒绝进 offboard；`/mavros/state` 显示 `mode=AUTO.LOITER, guided:true` 是正常起点。
- **AirSim 官方 Python 客户端（`PythonClient/airsim`）在 Python 3.12 上装不通**：它依赖老 `msgpackrpc + tornado 4.x`（只支持 Python ≤3.9），tornado6 又与 msgpackrpc 冲突 → **3.12 环境别走 airsim pip 包**，改走 **MAVROS(ROS2)** 控制，或单开 Python 3.9 环境。

## 支持文件
- `scripts/las_analyze.py` — **真实点云地图体检**（纯 numpy，无需 laspy/PDAL/open3d）：
  手解 LAS 头 + 分块读点数千万级点云 + 1 m 网格覆盖率 + 0.5 m 体素抽稀 +
  **逐格局部地面的 AGL 分带**（飞行高度带体素数 = UE4 碰撞几何规模）+ 可选出按高度上色的 PLY。
  `python3 scripts/las_analyze.py <file.las> --voxel 0.5 --ply`
- `references/pointcloud-map-to-simulation-world.md` — **点云地图 → 仿真世界**：
  「点云不能碰撞」这条硬约束、三条路线（UE4 体素几何 / 程序化种树 / 纯 ROS 回放）对比与推荐节奏、
  体检要看哪四个决策数字、本次 4.59 ha 松树林实测数字（7633 万点→29 万体素→飞行带 6.3 万方块）、
  **两个分析陷阱**（AGL 不能用全局分位当基准；裸局部极大值做单木提取会严重漏检）、
  UE4 侧环境勘探命令（`UE4Editor-Cmd.exe` 可全自动建关卡）
- `references/ue4-scene-build-and-edit.md` — **UE4 侧执行层**（方案定了之后怎么真做出来）：
  **OBJ 导入坐标约定实测表**（单位 1:1 当 cm、Y 镜像、★**绝不手动翻面绕序**否则法线全朝下地面隐形）、
  commandlet vs `-ExecutePythonScript` 分工、**关卡必须从"能跑 AirSim 的关卡"派生**（GameMode Override）、
  `-nosound`、**UE4.27 Python 不能建组件 → HISM 做不了**、PBR 材质（GL 法线翻绿通道 / 树叶 MASKED+双面叶 /
  **别调 `recompile_material` 会卡死**）、**隐藏树干网格做碰撞**、**场景后期"就地更新资产不新建关卡"**、
  起降坪做法（偶数尺寸中心落在格点）、**树密度两法交叉验证 + 不许编造超过实测上限**、
  人工元素选位（要林内空隙不是离树最远）与诚实标注、交付前数值自洽校验。
- `references/airsim-ue4-install.md` — 完整的 UE4.27 + VS2022 + AirSim 搭建命令序列与本次实测的判读点（版本探测、依赖预放、Build.bat、SUV 资产、settings 多旋翼）。
- `references/airsim-px4-twin-stack.md` — AirSim(Windows) + ROS1/Noetic(WSL) 数字孪生栈：twin_bridge（RPC 41451 → `/twin/*` 话题）、FAST-LIVO2 仿真预览（L515 / `lidar_type:4`）、传感器频率实测、启动器脚本，"预览话题独立命名、标定前不回灌 PX4"的设计；**并含正式链路**（`start_fastlivo.py` 门禁 + calibration/algorithm/camera 三个 yaml 的必填清单 + `/livox/*` 实机话题契约）。
- `references/px4-sitl-experiment-automation.md` — **搭好之后怎么自动化跑实验**：一键脚本骨架（前置检查→场景复位→启动后端→数据级就绪判据→跑实验→只清理自己的PID）、写这类脚本必踩的 **6 个坑**（脚本自 source ROS、`set -u` 与 ROS setup 冲突、必须用 `make px4_sitl_default none_iris` 而非直接调二进制、`Clock` 在 `rosgraph_msgs`、`--skip-trial` 类参数校验、`--help` 行号越界）、AirSim 侧要点（无法自动 Play→检测+提示、`simReset` 保证配对公平性、用 LiDAR RPC 自检飞行方向空旷性）、**MAVROS 坐标约定源码级核对法**、**分级诚实交付**规范（已编写/离线通过/后端联通/实际飞行通过）、以及**批量重复层**（交错序列 0→200→200→0、保留全部运行含失败、mean±std 与 CV<15% 稳定性门槛、请求时刻状态核对初始条件、取"请求那一刻"必须按时间就近而非按阶段名）、**多档扩展与口径校正**（名义延迟≠实测延迟：50 Hz 发布量化使名义 200 ms 实测约 214 ms（+7%），别归因"PX4 加减速动态"；用梯形积分把「请求后位移」拆成【延迟窗口】+【制动段】把账算平并做物理核对；扩档仍用交错块、每轮 full-restart + `simReset`；多档 `sweep_report.py` 三联图；**绘图必须用隔离 venv**，系统 numpy 1.17.4 会 ImportError）。

## 常见坑速查
| 症状 | 根因 / 修法 |
|---|---|
| UE 编辑器"找不到"，路径不对 | UE4.27 用 `UE4Editor.exe`，不是 `UnrealEditor.exe` |
| 双击 `.uproject` 弹 `Installation Not Found / No Unreal Engine installations found` | UE 装在非默认盘、**未在注册表注册**（`.uproject` 的 `EngineAssociation:"4.27"` 在注册表找不到）→ 点 **Yes 手动定位** `...\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe`（注册一次，以后双击可用）；或**绕过**：`UE4Editor.exe "<path>\Blocks.uproject"` 直接启动 |
| 改了 settings.json 但 Play 没生效 | 编辑器内停/重启 **PIE 不会重读 settings**；必须 **保存工程 + 关掉整个 `UE4Editor.exe` + 重开项目**（改传感器/车辆配置后尤其） |
| `build.cmd` 报 "run from x64 Native Tools" | 没从 VS2022 x64 Native Tools Prompt 跑 |
| `Could not find NetFxSDK` | VS 缺 .NET 桌面开发(→装 .NET Framework SDK 4.6+) |
| `build.cmd` 卡在 Downloading rpclib | iwr 直连不通，改用 curl 代理预放依赖 |
| SUV 材质/SuvCarPawn 加载失败 | 跳过了 car_assets(→下载解压) 或默认 spawn SUV(→settings 改多旋翼) |
| Play 报 SimHUD getCamera 空指针 | 日志末尾找 `no compatible vehicles ... Check your settings.json` → **有**则 settings 的 `VehicleType` 改成 `"SimpleFlight"`/`"PX4"`（勿写 `QuadX` 机架类型）；若 Build.bat 用 VS18(MSVC 14.51)→卸载 VS18 用 VS2022 重编；都不是才怀疑版本 |
| 用脚本启动 PX4 报 `Error opening startup file, does not exist: etc/init.d/rcS` | **别直接调 `build/.../bin/px4`**（会去猜 `-d <etc>`）→ 用 `make px4_sitl_default none_iris`，它会准备 rcS 并让 none 模拟器在 4560 等 AirSim |
| 脚本里 `source setup.bash` 报 `ROS_DISTRO: unbound variable` | `set -u` 与 ROS setup 脚本冲突 → `set +u; source …; set -u`（仅 source 期间关 nounset）；另脚本要**自己 source**，别依赖调用者 |
| 想让脚本自动进入 AirSim Play | **已可自动化**：`UE4Editor.exe <proj> /Game/<Path>/<Level> -game -windowed -nosound` 直接进关卡并起 SimMode（判据 15~30 s 内 41451 LISTENING + `Game class is 'AirSimGameMode'`）。**不带 `-nosound` 会因音频线程死循环卡成 ~0.4 fps、AirSim 起不来**。旧方案"必须让用户手工点 ▶ Play"已作废；仅当只想在编辑器里自己 Play 时才需人工。详见 `references/ue4-scene-build-and-edit.md` |
| 关卡能加载、网格能构建，但 **AirSim RPC 41451 永不监听** | 关卡 World Settings 的 **GameMode Override 不是 `AirSimGameMode`**（`new_level` 建的空关卡没有它）→ 日志 `Game class is 'BlocksGameMode'`。**修法：从项目里已知能跑 AirSim 的关卡派生**（`load_level` → 只删 StaticMeshActor → 另存），别 `new_level` |
| 地面**完全看不见**、只剩一堆悬空方块；按 ▶ Play 像"掉入虚空" | 地形网格**法线 100% 朝下**被背面剔除 —— 多为"为抵消 Y 镜像又手动翻了面绕序"（**导入器已翻过，翻两次=反向**）。**修法：OBJ 只写 `Y=-y`，不翻绕序**；材质加 `two_sided=True` 兜底；导出后用代码统计法线朝上/朝下占比，必须 100% 朝上 |
| 地形四周多出一大片"复制出来的"假地形 / 场景杂乱 | 把真实数据网格用 `np.clip` 复制边缘 **padding 成 2 的幂**（如 203×229 → 253×253）。**OBJ 静态网格不需要 2 的幂**（那是 Landscape 的要求）→ 用真实尺寸 |
| 建材质时脚本卡住不动（等几分钟无响应） | `MaterialEditingLibrary.recompile_material()` **每个材质触发一次着色器编译，第 2 个就卡死**。**修法：不调它**，建完统一 `save_asset`，引擎在关卡加载时自会编译（去掉后 3 材质+建关卡 15 秒完成） |
| 材质光照/凹凸看着不对（方向怪、凹凸反了） | 法线贴图是 **OpenGL(`nor_gl`)** 格式却直接用了 → **`flip_green_channel=True`** 且 `compression=TC_NORMALMAP`、`srgb=False`；`nor_dx` 才不用翻。1k 贴图铺 200+ m 还要给 `MaterialExpressionTextureCoordinate` 设 `u/v_tiling`（如 100），否则糊成一片 |
| 用 `Test-NetConnection` 探 4560 之后 PX4 再也连不上 | AirSim 的 TCP server **接受任何连接** → 探测被当成 SITL 连上，日志出现**假的** `Connected to SITL over TCP`，随后永远 `not receiving any messages from HIL`。**修法：绝不用 TCP 探 4560**（判据只用 41451；要看 4560 用 `netstat` 解析），探过之后**必须重启 UE4** |
| 想用 HISM 实例化大量植被 | **UE 4.27 的 Python 不能动态创建组件**（无 `add_component_by_class`、无 `HierarchicalInstancedStaticMeshActor` 类）→ N 个独立 StaticMeshActor 替代；要 HISM 得在编辑器里手工放。**别在文档里写"HISM instances"却实际用独立 actor** |
| 地形**看得见**，但按 ▶ Play 飞机仍"掉入虚空" | 先读 `%USERPROFILE%\Documents\AirSim\settings.json` 的 `Vehicles.<name>.X/Y/Z` —— **被改回 `(0,0,0)` 就生成在世界原点**，那个高度可能是地形内部或任意高处。用户会自己改这个文件 → **排查"掉入虚空"先看它，别只怀疑几何**。每次改前备份 `settings.json.bak_before_<用途>_<时间戳>` |
| 地形高程整体偏差 10~40 m（表现为树"浮空"/"埋地"、飞机莫名撞地） | 生成地形的 **ground 中间产物被哨兵值填充**（`np.isnan(ground).sum() == 0` 是**红灯**：说明用有限值代替了 NaN，实测 75% 的格都被填成同一个值）。→ 见 `pointcloud-forest-structure-extraction` 步骤 2 的（跨度判据 + 迭代下包络 + 拿点云当裁判）修法；**`grey_opening` 对成片高值区无效**（实测仅降 16%） |
| 改完场景想"就地更新"而不是新建关卡 | 关卡引用的是资产路径 → **同路径重导 OBJ 覆盖 `terrain_xx` 即可自动更新**，不必重建关卡；只需单独调整变化的那几个 actor（如起降坪的位置/缩放/标签）后 `save_current_level`。**别为每次小改新建 vN 关卡**（版本越堆越多） |
