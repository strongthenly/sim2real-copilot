# AirSim + UE4.27 完整搭建（本次实测）

## 选型依据
- 显卡 RTX3050 4GB → 用 **Unreal Engine 4.27**（UE5 吃力）。
- UE4.27 编辑器 exe = `UE4Editor.exe`（不是 UE5 的 `UnrealEditor.exe`）。
- VS 要 **VS2022(17)**，不是 VS18/2026。判别：
  ```
  "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -all -products * -property installationPath
  "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -all -products * -property catalog_productDisplayVersion
  ```
  VS2022 = `...\2022\Community`；**错误**的 VS18 = `...\18\Community`（千万别用，AirSim/UE4 兼容问题）。

## 1. 装 UE4.27（E 盘）+ VS2022
- Epic Games Launcher → 虚幻引擎 → 库 → 添加 **4.27.2** → 安装路径改 E 盘。**只装 Launcher 不算装好引擎**，一定要在库里把 UE_4.27.2 本体装上（20-40GB，`E:\Program Files\Epic Games\UE_4.27`，验证 `UE4Editor.exe` 存在）。
- VS2022 Community：**C++ 桌面开发** + **.NET 桌面开发**（后者必须，否则 UBT 报 NetFxSDK）。

## 2. clone AirSim（直连 GitHub 时通时断 → 不行就走代理）
```
git clone https://github.com/microsoft/AirSim.git E:\AirSim   # 默认分支=master(主UE5)
```
- 直连不通（国内）时：`git -c http.proxy=http://127.0.0.1:7899 -c https.proxy=... clone ...`（实测 Clash 端口是 7899，非 7890）。

## 3. 依赖预放（绕过 build.cmd 的 iwr 直连下载失败）
`build.cmd` 用 PowerShell `iwr` 下载 rpclib/eigen，**直连经常失败**。用 curl 走代理预放到 build 期望的位置：
```
# rpclib -> external\rpclib\rpclib-2.3.0\
curl.exe -L --proxy http://127.0.0.1:7899 https://github.com/rpclib/rpclib/archive/v2.3.0.zip -o E:\AirSim\external\rpclib.zip
Expand-Archive E:\AirSim\external\rpclib.zip -DestinationPath E:\AirSim\external\rpclib -Force
# eigen -> AirLib\deps\eigen3\Eigen
curl.exe -L --proxy http://127.0.0.1:7899 "https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip" -o E:\AirSim\eigen.zip
Expand-Archive E:\AirSim\eigen.zip -DestinationPath E:\AirSim\AirLib\deps\eigen_src -Force
New-Item -ItemType Directory -Path E:\AirSim\AirLib\deps\eigen3 -Force
Move-Item E:\AirSim\AirLib\deps\eigen_src\eigen-3.3.7\Eigen E:\AirSim\AirLib\deps\eigen3\Eigen
```

## 4. 构建（必须从 VS2022 x64 Native Tools Prompt 跑 build.cmd）
`build.cmd` 会：编译 rpclib → SUV 资产 → eigen → `msbuild AirSim.sln`(x64 Debug+Release) → copy 到 `Unreal\Plugins\AirSim\Source\AirLib` → 更新 environments。
- 无人机不用 SUV 车：`build.cmd --no-full-poly-car`（但会让 SUV 素材缺失，见 step6）。
- 没 .NET 时加 **".NET 桌面开发"** 再跑。

## 5. UE 编译插件 DLL（build.cmd 不编 Unreal 插件 DLL）
UE 打开 `.uproject` 报 `Incompatible or missing module: Blocks/AirSim` → 手动编译：
```
"E:\Program Files\Epic Games\UE_4.27\Engine\Build\BatchFiles\Build.bat" BlocksEditor Win64 Development -Project="E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject" -WaitMutex
```
成功：`UE4Editor-Blocks.dll` + `UE4Editor-AirSim.dll` 生成（约 66 秒）。

## 6. SUV 资产（--no-full-poly-car 跳过了，加载会失败）
报错 `加载失败 /AirSim/VehicleAdv/SUV/AutomotiveMaterials/Materials/...` → 补 car_assets：
```
curl.exe -L --proxy http://127.0.0.1:7899 https://github.com/Microsoft/AirSim/releases/download/v1.2.0/car_assets.zip -o car_assets.zip
Expand-Archive car_assets.zip -DestinationPath E:\AirSim\Unreal\Plugins\AirSim\Content\VehicleAdv -Force
```
材质要落在 `VehicleAdv\SUV\AutomotiveMaterials\Materials\...`（含 AutoMotivesMaterials + Textures）。

## 7. settings.json 改多旋翼（避开默认 spawn SUV 的 SuvCarPawn 失败 + SimHUD 空指针）
`%USERPROFILE%\Documents\AirSim\settings.json`（改前备份 `.bak`）：
```json
{ "SettingsVersion":1.2, "SimMode":"Multirotor",
  "Vehicles": { "Drone_1": { "VehicleType":"QuadX",
    "Cameras": { "0": { "CaptureSettings": [ {"ImageType":0,"Width":640,"Height":480,"FOV_Degrees":90} ] } } } } }
```
- `Cameras` 必须配，否则 `PawnSimApi::getCamera()` 空指针崩溃（`SimHUD::initializeSubWindows`）。加了还崩 = **AirSim master 与 UE4.27 兼容 bug** → checkout UE4.27 稳定 tag（如 v1.8.x）重新编译，别在配置上耗。

## 8. 启动/重启
- UE 编辑器：`"<UE>\\Engine\\Binaries\\Win64\\UE4Editor.exe" "<AirSim>\\Unreal\\Environments\\Blocks\\Blocks.uproject"`
- 点 Play 运行 AirSim；改了 settings.json 必须 **Esc + 重新 Play**（或重启 UE）才重读。
- UE 闪退无窗口 → 看 `E:\AirSim\Unreal\Environments\Blocks\Saved\Logs\Blocks.log` 尾部。

## 关键鉴别：airsim 进程/资产
- `getCamera()` 空指针崩溃 = 配置(Cameras) 或 master/UE4.27 兼容，别当构建问题。
- `Incompatible or missing module` = 模块没编译（用 Build.bat）。
- 只有 Build.bat 生成的 `.dll` 在 `Plugins\AirSim\Binaries` 正常；build.cmd 后的 Binaries 为空白属正常（要 UE/Build.bat 才生成插件 dll）。
