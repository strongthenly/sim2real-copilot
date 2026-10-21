# AirSim 安装（Windows）完整配方 + 国内网络下载

AirSim（微软，开源免费 MIT）基于 **Unreal Engine**，**只能 Windows 本机跑，WSL 跑不了 Unreal GUI**。用户常见架构：
`Windows 端 Unreal+AirSim + WSL 端 ROS2/算法（经镜像网络互连）`。

## 授权 / 版本事实
- **Unreal 免费**：仅当"游戏产品营收超过 $1,000,000"才收 5% 版税；学习/研究/仿真/无人机开发完全免费。
- **UE4.27.2** 是 UE4 系列最终补丁版，Epic 库"添加引擎版本"里可选。空气 Sim 的 UE4.27 支持即测 4.27.x。
- **UE4.27 只支持 VS 2019/2022（16/17 工具集）**，**不支持 VS 18（2026）**。误装 VS18 → AirSim 编译不兼容。

## 第一步：装 Unreal + VS（Windows）
1. Epic Games Launcher（store.epicgames.com/download）→ 登录 → 虚幻引擎→库 → 添加 **Unreal Engine 4.27.2**，路径改到非 C 盘（如 `E:\Epic Games\UE_4.27`）。
2. Visual Studio **2022**（17 版，用官方 aka.ms 直链区分于最新 18 版）→ 勾 **"使用 C++ 的桌面开发" + Windows SDK**。`https://aka.ms/vs/17/release/vs_community.exe`
   - ⚠️ 官网默认给最新（VS 18/2026），别下错。用 aka.ms/vs/17 的直链才是 VS2022。

## 第二步：clone AirSim
```powershell
git clone https://github.com/microsoft/AirSim.git E:\AirSim   # 用默认分支(main)，不要 --branch master(不存在)
```
若走代理 `-c http.proxy=http://<port> -c https.proxy=http://<port>`。

## 第三步：build.cmd（构建 C++ 库 + 准备 Unreal 插件）
- **不需要 UE 路径**（build.cmd 只编译 AirLib/rpclib/MavLinkCom，`cmake -G "Visual Studio 17 2022"` + `msbuild AirSim.sln`）。UE 只用于后续打开 Unreal 项目。
- **必须从 "x64 Native Tools Command Prompt for VS 2022" 运行**，否则报 `You need to run this command from x64 Native Tools Command Prompt for VS 2022`。
- 自动化脚本 `E:\airsim_build.cmd`：
  ```cmd
  @echo off
  call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
  cd /d E:\AirSim
  build.cmd
  ```
  用 `terminal(background=true, notify_on_complete=true)` 跑（长任务）。
- **build.cmd 会自动下载 rpclib（github）+ eigen（gitlab）+ SUV 资产（~37MB）**，用 PowerShell `iwr`（走 codeload/gitlab 直连）。
- **国内直连 GitHub 时通时断**（git clone 刚通、iwr 下秒卡死），最稳是 Clash 代理：
  - 先探测实际端口：`Test-NetConnection 127.0.0.1 -Port 7890/7891/7897/9090`（Clash for Windows 默认 7890，**Clash Verge/Mihomo 默认 7897**，非固定）。所有常见端口都不通 = Clash 核心没起来（不是只开 GUI）。
  - 走代理 `curl.exe -L --proxy http://127.0.0.1:<port> ...`。
- **iwr 下载失败（最省事的绕法）**：手动 `curl.exe -L` 预放依赖，再重跑：
  - rpclib：`curl -L https://github.com/rpclib/rpclib/archive/v2.3.0.zip` → 解压出 `external\rpclib\rpclib-2.3.0\`
  - eigen：解压后把 `Eigen` 移到 `AirLib\deps\eigen3\Eigen`
  - 重跑 `build.cmd --no-full-poly-car`（跳过 SUV 37MB 下载，用默认 vehicle，足够跑探索）

## 其它坑
- **UE 装在 E 盘**：后续 `make px4_sitl` / Unreal 项目打开时，若脚本默认找 C 盘 Epic 路径，建符号链接 `cmd /c mklink /D "C:\Program Files\Epic Games\UE_4.27" "E:\Epic Games\UE_4.27"`（需管理员）。
- **定位 VS 的稳妥方法**：vswhere 写成 .cmd（**只用英文，避免 GBK 乱码**），`cmd /c` 跑 `vswhere -all -products * -property installationPath`。
- **PX4 留 WSL + AirSim 在 Windows**：MAVLink（MAVROS）指挥，AirSim 传感器经 AirSim ROS bridge（WSL）进 ROS2。

## 在 UE 里真正跑起来（打开 Blocks 项目）—— 实测关键
1. **UE4.27 的编辑器可执行文件叫 `UE4Editor.exe`，不是 UE5 的 `UnrealEditor.exe`**！UE4 用 `Engine\Binaries\Win64\UE4Editor.exe`（UE5 才改名叫 UnrealEditor.exe）。用错名字（UnrealEditor.exe）Test-Path 会一直 False，白排查半天。启动：
   ```powershell
   Start-Process "E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe" -ArgumentList '"E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject"'
   ```
2. **UE 打开 `.uproject` 但模块缺失（`Incompatible or missing module: Blocks / AirSim`）**：首次打开若 UE 启动就闪退（无窗口、无进程、LOCALAPPDATA 无日志），就不会自动触发"重建模块"。**手动用 UnrealBuildTool 编译**（可靠路径）：
   ```cmd
   "E:\Program Files\Epic Games\UE_4.27\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development -Project="E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject" -WaitMutex
   ```
   `Build.bat 用 UBT 编译 .uproject 的编辑器目标（含 AirSim 插件），是长任务（几分钟），用 `background=true + notify_on_complete=true` 跑。编译 AirSim 模块时 `dinput8.lib / dxguid.lib ... was not resolvable` 是**警告非致命**（UBT 会自己搜库路径，AirSim 常见），别当错误修。
3. **UE 需要 .NET Framework SDK（4.6+）**：`Build.bat / UBT` 报 `Could not find NetFxSDK install dir ... Install a version of .NET Framework SDK at 4.6.0 or higher` → 在 **Visual Studio Installer 给 VS2022 加 ".NET 桌面开发"(.NET desktop development)** 工作负载（含 .NET Framework SDK/targeting pack）。只勾 C++ 会缺它。装号 NET 才能编过。
4. **`update_from_git.bat` 报注册表错误**（`HKEY_CLASSES_ROOT\Unreal.ProjectFile\shell\rungenproj` 缺失 / `'/projectfiles' is not recognized`）通常**非致命**——`build.cmd` 主编译已成功（exit 0），只是最后"用注册表定位 UE 生成项目文件"那步因 UE 装在 E 盘、没注册到默认关联而失败。直接用 UE4Editor 打开 .uproject 即可，不用管它。
5. **UE 装在 E 盘：真实路径常见是 `E:\Program Files\Epic Games\UE_4.27`**（含空格）。别只猜 `E:\Epic Games\...`。
