# AirSim(Windows Unreal) + WSL 算 ROS 的混合架构搭建

用户若想"高保真复现实机 + 真实视觉"做探索/论文，AirSim 是正解。它是**混合架构**：
- **Windows 端**跑 Unreal Engine + AirSim(仿真环境 + 高保真传感器渲染)
- **WSL 端**跑 ROS2 + PX4 SITL(MAVROS) + 探索算法(大脑)，通过 MAVLink/RPC 指挥

## 硬性架构前提(先讲清，避免白装)
1. **AirSim 必须 Unreal Engine，Unreal 只能在 Windows 本机跑，WSL 跑不了**(桌面 GUI 应用)。所以整套在 Windows 侧。
2. **WSL ↔ Windows 互通靠 `networkingMode=mirrored`**(已在 `.wslconfig`)。这样 WSL 的 `127.0.0.1:<port>` 与 Windows 的 `127.0.0.1:<port>` 互通 → WSL 能连 Windows 上的 **Clash 代理(注意实际端口是本机 Clash 的，常见 7899/7897/7890，别硬编码)和 AirSim RPC(默认 41451)**。这是混合架构的命门。
3. AirSim 官方 ROS bridge(`airsim_ros_pkgs` / ROS2 版)在 WSL 侧运行，经 RPC 连 Windows 的 AirSim，把 LiDAR/相机/位姿发布成 ROS 话题、也收 ROS 命令。

## 版本硬要求(最容易翻车)
- **Unreal Engine 用 4.27(4.27.2)**——不是 5.x。UE5 对 4GB 显卡(如 RTX3050 mobile)压力大、常崩；UE4.27 对 AirSim 兼容最好。Epic 库里旧版本只给一个最终补丁版(4.27.2)，**没问题，直接装**。
- **Visual Studio 必须用 2022(17.x)** —— AirSim `build.cmd` 用 `msbuild` + `cmake -G "Visual Studio 17 2022"`，且**检查 `%VisualStudioVersion% >= 17.0`**。**UE4.27 官方只支持 VS2019/2022(16/17),不支持 VS 18(2026)**。
  - ⚠️ **epic 坑：用户从 MS 官网下载"最新"vs_community.exe 会装成 VS 18(2026)，不是 2022**。装 VS2022 要用 **17 版官方 aka.ms 链接**：`https://aka.ms/vs/17/release/vs_community.exe`。
  - 验证装的是不是 2022:`C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe -all -products * -property installationPath`(路径含 `2022` 才对；含 `18` 是 2026)。找 `vcvarsall.bat`:`Get-ChildItem "C:\Program Files\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvarsall.bat"`。
- 工作负载勾 **"使用 C++ 的桌面开发"** + 组件 **Windows 10 SDK**。装好后开始菜单应有 **"x64 Native Tools Command Prompt for VS 2022"**。

## 安装步骤
1. Epic Games Launcher → 登录 → 虚幻引擎→库→添加 **Unreal Engine 4.27** → 安装路径改 **E 盘**(如 `E:\Epic Games\UE_4.27`)，不占 C。
2. VS 2022 Community，勾 C++ 桌面开发 + Windows 10 SDK(工具集用默认即可，别纠结 v142)。
3. **clone AirSim**(默认分支——AirSim 默认分支**不是 master**，`git clone -b master` 会报 `Remote branch master not found`):
   ```
   git clone https://github.com/microsoft/AirSim.git E:\AirSim
   ```
   - 国内直连 GitHub 通常可通(拉 raw 文件都行)；代理端口**别硬编码 7890**——不同 Clash 客户端默认不同(Clash Verge/Mihomo=7897，Clash for Windows=7890，**本机实测 7899**)。先用 `Test-NetConnection 127.0.0.1 -Port <端口> -InformationLevel Quiet` 扫 `7890/7897/7899/9090` 找实际监听口；全不通=Clash 核心没真正启动(只开了 GUI/端口改了)，而非网络坏。**直连 GitHub**(拉 raw/curl)通常可通，但**时通时断**——下载依赖走代理更稳。
4. **build.cmd 构建 C++ 核(不需 UE 路径)**——它编译 AirLib/rpclib/MavLinkCom，用 `msbuild`。**必须从 "x64 Native Tools Command Prompt for VS 2022" 运行**(否则报 "You need to run this command from x64 Native Tools Command Prompt for VS 2022" 或 VS 版本 <17 报错):
   ```
   cd E:\AirSim
   build.cmd
   ```
   会依次下载 **rpclib**(GitHub zip)+ **eigen**(gitlab)+ **高精度 SUV 资产(~37MB)** 并编译 `AirSim.sln`(Debug+Release)。⚠ **这些下载在国内直连/`iwr` 下常失败**(build.cmd 用 PowerShell `iwr` 下，第一次跑常卡死/崩在 `Downloading rpclib`——报 `iwr: 无法连接到远程服务器`)。**事先用 `curl.exe -L --proxy http://127.0.0.1:<clash端口> --max-time 180 <url>` 手动下好并解压到 build.cmd 期望的位置**，让 build.cmd 跳过下载直接编译：`rpclib`→`external\rpclib\rpclib-2.3.0`、`eigen`→`AirLib\deps\eigen3\Eigen`；并加 **`--no-full-poly-car`** 跳过那个 37MB SUV(无人机用不到，也省得卡住)。，输出复制到 `Unreal\Plugins\AirSim`，再更新 `Unreal\Environments\*`。长任务(十几分钟)，跑完别 Ctrl+C。
5. 之后用 UE 打开 `Unreal\Environments\Blocks\Blocks.uproject`(UE 编辑器自己知道 UE 路径，不依赖 build.cmd)。⚠ **用 `Engine\Binaries\Win64\UE4Editor.exe` 打开**(UE4 的编辑器 exe 叫 **`UE4Editor.exe`**，UE5 才叫 `UnrealEditor.exe`；别找 UnrealEditor)——完整路径如 `E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe "<blocks.uproject>"`。**三个常判读者会误判的点**：① 首次打开弹"缺失模块/重建"→点 **Yes**；② UnrealBuildTool 此时才编译 AirSim 插件(几分钟)，所以 `build.cmd` 跑完后 `Plugins\AirSim\Binaries` 是空的**正常**(插件 DLL 是 UE 打开项目时编译的，不是 build.cmd)；③ build.cmd 最后一步 `update_from_git.bat` 报 `'/projectfiles' is not recognized` / 注册表 `Unreal.ProjectFile\rungenproj` 缺失——**这是良性的**(它靠注册表找 UE 路径失败，但**exit 0 即主编译成功**，UE 编辑器自己知道 UE 路径)。

## Unreal 编辑器模块编译(手动 Build.bat)——当 UE 不自动编译模块时
`build.cmd` 只编了 C++ 核(AirLib/rpclib)，**Blocks + AirSim 的 Unreal 编辑器模块 DLL 是打开项目/或 UBT 时编的**。但**首次用 UE4Editor.exe 打开 `Blocks.uproject` 时，UE 常常不自动编译、直接闪退/无窗口**(而非弹"重建"框)：日志 `E:\AirSim\Unreal\Environments\Blocks\Saved\Logs\Blocks.log` 里会重复出现 **`LogInit: Warning: Incompatible or missing module: Blocks / AirSim`**(`Still incompatible`)。**判读：出现这个 = 模块没编，需手动用 UBT Build.bat 编译编辑器目标**(若 UE 自动编译顺利反而没这次问题)。
```
"E:\Program Files\Epic Games\UE_4.27\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development -Project="E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject" -WaitMutex
```
✅ 成功标志：`[11/11] BlocksEditor.target` + `Total execution time` + 生成 `UE4Editor-Blocks.dll` 和 **`UE4Editor-AirSim.dll`**。之后 UE4Editor 重新打开 Blocks 就能加载插件(进程内存比首次高、稳定存活)。编完可能提示 `Detected compiler newer than VS 2019`——无害。

## 三个会卡死编译/打开的硬依赖
- **UnrealBuildTool 需要 .NET Framework SDK(4.6+)**:若只勾了"C++ 桌面开发"，`Build.bat`(和 UE 自动编译)会报 `ERROR: Could not find NetFxSDK install dir; ... Install a version of .NET Framework SDK at 4.6.0 or higher.`(SwarmInterface)。**修复：Visual Studio Installer → 修改 VS2022 → 勾 ".NET 桌面开发"(.NET desktop development，含 .NET Framework 4.8 SDK) → 修改**，然后重跑 Build.bat。这是 UE4.27 编译工具链的硬依赖。
- **`--no-full-poly-car` 会让 Blocks 加载失败(SUV 材质缺失)**:跳过那个 37MB SUV 车资产后，Blocks 环境一打开就报一堆 **`加载失败 /AirSim/VehicleAdv/SUV/AutomotiveMaterials/Materials/.../M_Leather / M_Caliper / M_Plastic_Matte ... 引用者 StaticMeshComponent0`**。**修复：走代理 `curl -L --proxy ... https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip` 下 38MB 包，`Expand-Archive` 到 `E:\AirSim\Unreal\Plugins\AirSim\Content\VehicleAdv`(落地出 `Materials / PhysicsMaterials / Sound / SUV / Textures / Vehicle`，且 `SUV\AutomotiveMaterials` 下有 `Materials`+`Textures`)→ 重启 UE。**若已解压但 UE 仍报同样错，多半是**旧窗口/缓存**——彻底 `Stop-Process UE4Editor*` 再全新启动(而不是只点关闭)。
- **UE4Editor.exe 是 0.4MB 的小启动器**，真引擎靠 Engine\Binaries\Win64 的 DLL 加载；若它闪退无窗口且无进程，先查 `Blocks.log` 的 `Incompatible or missing module` 再走上面的 Build.bat;别急着怪显卡。

## WSL 发行版迁移到非 C 盘(E 盘技术)
用户要"大文件放 E 盘不占 C"时，把 WSL 发行版整体迁走：
```
wsl --shutdown                                     # 先关所有 WSL(会停正在跑的 PX4/仿真)
wsl --export lyuav E:\wsl\lyuav.tar                # 导出(完整备份，占用=发行版大小)
wsl --unregister lyuav                             # 从 C 盘注销(删 C 盘 vhdx)
wsl --import lyuav E:\wsl\lyuav E:\wsl\lyuav.tar   # 导入到 E 盘
```
- **安全顺序：先 export 确认(退出码 0 + tar 大小合理)→ 再 unregister → import**。tar 是回退点。
- `bsdtar ... cannot archive sockets` 只是警告(PX4 运行时临时 socket，无数据)，不影响完整性。
- 验证：`wsl -d lyuav -- ls /home/<user>`(数据在)、`E:\wsl\lyuav\ext4.vhdx` 存在、默认用户保持。`.wslconfig`(镜像网络等)在 Windows 全局，import 不受影响。
- 完成后确认可用了再删 tar；否则留着当备份。

## 注意
- 探索算法(PX4/ROS2/MAVROS)和 AirSim 视觉是两条线，靠 AirSim ROS bridge 打通。**AirSim 接 PX4 真实飞控(MAVLink, TCP 4560 + PX4 v1.11.x + GCC10)的完整配方，见 `references/airsim-px4-integration.md`。**
- 显卡弱(3050 mobile / 4GB)只能中低画质 + 小场景；**批量实验仍可用 headless gzserver，不用 AirSim/GUI**。
