# AirSim(Windows) + PX4 SITL(WSL) 真实飞控集成

目标：让 AirSim 里的无人机由**真实 PX4 飞控**控制（发论文主流：PX4 真飞控 + AirSim 高保真环境），
而不是 AirSim 内置的 SimpleFlight 或键盘。架构：
```
AirSim(Windows, VehicleType=PX4) ←TCP 4560→ PX4 SITL(WSL, posix) ←QGC/MAVROS→ 你操控
```
跨系统靠 `networkingMode=mirrored`（WSL 的 127.0.0.1 与 Windows 互通）。

## ⚠️ 头号大坑：AirSim 的 VehicleType 值（决定 spawn 成败）
**自定义 `Vehicles` 时，`VehicleType` 必须是 AirSim 认识的**车辆类型**常量，不是机架**：
- ✅ 正确：`"SimpleFlight"`、`"PX4"`（源码常量 `kVehicleTypeSimpleFlight="simpleflight"`、`kVehicleTypePX4="px4multirotor"`，大小写不敏感）
- ❌ 错误：`"QuadX"`（那是**机架类型**，不是车辆类型）→ AirSim 报 **`There were no compatible vehicles created for current SimMode!`** → 连带 `SimHUD getCamera()` 空指针 **`EXCEPTION_ACCESS_VIOLATION @0xb8`** 崩溃。

**`"no compatible vehicles"` + getCamera 崩溃的根因链**：
- settings 里写了 `Vehicles` → `AirSimSettings.hpp` 会 `vehicles.clear()`（清默认），改走
  `createVehiclePawn()` → `pawn_paths.at(getVehiclePawnPathName(vehicle))`；base 的
  `getVehiclePawnPathName` 返回 `""`，若 `VehicleType` 不对 → `.at("")` 抛异常 → 无车辆。
- HUD 拿不到 pawn → `getCamera()` 空指针崩溃。**所以先解决 spawn（用对 VehicleType），getCamera 崩溃自然消失**。
- **最小配置 `{"SimMode":"Multirotor"}`（不写 Vehicles）会自动创建默认无人机**——验证平台通不通最省事。
  即使成功 spawn 出无人机，**别加 `Cameras` 和错误的 `VehicleType`**（都会重新触发"无车辆"）。

## AirSim settings（官方 PX4 SITL 格式，`docs/px4_sitl.md`）
```json
{
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true, "TcpPort": 4560,
      "ControlPortLocal": 14540, "ControlPortRemote": 14580,
      "Sensors": { "Barometer": { "SensorType": 1, "Enabled": true, "PressureFactorSigma": 0.0001825 } },
      "Parameters": { "NAV_RCL_ACT": 0, "NAV_DLL_ACT": 0, "COM_OBL_ACT": 1,
                      "LPE_LAT": 47.641468, "LPE_LON": -122.140165 }
    }
  }
}
```
- `UseTcp: true` + `TcpPort: 4560`：AirSim 经 **TCP 4560** 连 PX4（PX4 的 `[simulator]` 用 TCP）。
- `ControlPortLocal/Remote` 是 GCS/onboard MAVLink 通道（14540/14580）。
- `Barometer` 设置让 PX4 更快拿 GPS lock；`COM_OBL_ACT:1` offboard 结束悬停不迫降；`NAV_RCL_ACT:0/NAV_DLL_ACT:0` 禁用 RC 失联 failsafe（无遥控纯 API 飞行必需）；`LPE_LAT/LON` 与 `OriginGeopoint` 一致。

## PX4 版本硬约束（最容易卡）
**AirSim 的 PX4 集成只支持 PX4 v1.11.x**（`docs/px4_sitl.md` 明确 `checkout v1.11.3`）。
主分支太新（实测 v1.18-beta）MAVLink/接口与 AirSim 不兼容。需**独立 clone 一套 v1.11.3**（别动现役的新版）：
```bash
git clone --recursive --branch v1.11.3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-AirSim   # 走代理
```

## PX4 SITL 启动命令
**`make px4_sitl`（不带模型）只编译固件（输出 SBOM），不会启动 SITL（无 `pxh>`）**——必须带仿真模型目标：
```bash
cd ~/PX4-AirSim && make px4_sitl_default none_iris   # posix，等 AirSim TCP 4560 连接
```
看到：`INFO [simulator] Waiting for simulator to accept connection on TCP port 4560` + `pxh>` 即就绪。

## PX4 v1.11.3 必须用 GCC 10（GCC 13 编译失败）
新系统 GCC 13 的 `-Wfatal-errors` 拒绝 v1.11.3 里的 `math::max(PTHREAD_STACK_MIN(long), PX4_STACK_ADJUSTED(size_t))`（隐式类型转换）。
- **装旧 GCC**：`sudo apt install -y gcc-10 g++-10`。若 apt 卡 0% / 大量 `Ign:`，是 apt 代理端口错（见下）。
- **CC/CXX 环境变量只在首次 configure 生效**：build 缓存已记住 `/usr/bin/c++`(gcc-13)，改环境变量没用。
  必须先 `rm -rf build/px4_sitl_default`，再 `CC=gcc-10 CXX=g++-10 make px4_sitl_default none_iris`。
- CMake 日志应显示 `The CXX compiler identification is GNU 10.5.0`（确认吃到 gcc-10）。

## v1.11.3 + 新 glibc 源码编译修复（实测 3 处）
用 gcc-10 编仍可能报同一类型错，**必须改源码**。`platforms/common/px4_work_queue/WorkQueueManager.cpp:257`：
```cpp
// 原（隐式转换，新 glibc 的 PTHREAD_STACK_MIN 是 long → 任何编译器都拒）
const size_t stacksize_adj = math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize));
// 改：显式转同一类型。注意用 math::max（PX4 的），别用 std::max（需 <algorithm>，此文件没 include）
const size_t stacksize_adj = math::max(static_cast<size_t>(PTHREAD_STACK_MIN), static_cast<size_t>(PX4_STACK_ADJUSTED(wq->stacksize)));
```
改完重编（`CC=gcc-10 CXX=g++-10 make px4_sitl_default none_iris`），可能还有其它同型错误，逐处同样的 cast 修。

## 对接验证
1. PX4 SITL 前台跑（保持 `pxh>`）；AirSim（UE4Editor 打开 Blocks）Play。
2. AirSim settings 已是上面 PX4 格式；UE 前端开着。
3. PX4 日志出现如下即接通：
   ```
   INFO [simulator] Simulator connected on TCP port 4560.
   INFO [mavlink] partner IP: 127.0.0.1
   INFO [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)
   INFO [ecl/EKF] EKF commencing GPS fusion
   INFO [commander] home: ...  home_set
   ```
4. QGC（可选）：PX4 SITL 模式下用 QGC 连接（别插真飞控，否则 QGC 选真板）；`MAV_BROADCAST=1` 才允许非本机。

## 其它坑
- **每次停 UE 都要重启 PX4**（`px4_sitl.md` 明示）。
- **防火墙开 TCP 4560 + UDP 14540**（Windows 侧，AitSim 要连跨系统的 PX4）。

## apt/网络两个实用修法
- apt 大量 `Ign:`（universe 组件）+ 卡 0% = **apt 代理端口指错**（配成 7890，实际 Clash 是 7899）：
  `sudo sed -i 's/7890/7899/g' /etc/apt/apt.conf.d/proxy && sudo apt update`
- gcc-10 在 Ubuntu **universe** 组件（components 里要有 `main universe restricted multiverse`）。
