# AirSim + PX4 SITL 集成（真飞控固件）

> AirSim 的 PX4 集成是**「软件在环 SITL」**，比 SimpleFlight 复杂（官方 docs/px4_setup.md 明确警告 set up PX4 is not a trivial task）。本页记录实测要点。

## 0. 两个大前提（必看，否则白忙）
1. **PX4 版本必须 v1.11.x**。AirSim 只测过 v1.11.x；你现有的 v1.18（如 `v1.18.0-beta1`）MAVLink 接口与 AirSim **不兼容**，接不上。需**单独 clone 一个 PX4 v1.11.3**（`git clone --recursive --branch v1.11.3 ...`，别动现有的），并重新编译。
2. **PX4 启动命令是 `make px4_sitl_default none_iris`**（不是 `make px4_sitl`，也不是 gz_x500）。
   - `make px4_sitl`（不带模型）**只编译固件**（输出 Generating SBOM…），**不启动**，无 `pxh>`。
   - `none_iris` 让 PX4 处于「等待 simulator 连接」模式（在 TCP 4560 等 AirSim），不自己弹 gazebo。
   - 若你的 PX4 是旧版本 v1.8.*，命令换成 `make posix_sitl_ekf2 none_iris`。

## 1. settings.json（PX4 SITL 官方格式）
路径 `C:\Users\<USER>\Documents\AirSim\settings.json`（改前先备份 `.bak`）。**VehicleType 用 `"PX4Multirotor"`**（匹配 `kVehicleTypePX4="px4multirotor"`，大小写不敏感）：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "Sensors": {
        "Barometer": { "SensorType": 1, "Enabled": true, "PressureFactorSigma": 0.0001825 }
      },
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_ACT": 1,
        "LPE_LAT": 47.641468,
        "LPE_LON": -122.140165
      }
    }
  }
}
```
- `UseTcp:true` + `TcpPort:4560`：PX4 的 `[simulator]` 用 **TCP 4560** 等 AirSim 连。
- `NAV_RCL_ACT:0`/`NAV_DLL_ACT:0`：无遥控时关闭 failsafe（纯 python 脚本飞）。**切勿在真机用**。
- `COM_OBL_ACT:1`：offboard 命令结束后 hover 而非 land。
- `LPE_LAT/LON` 要与 AirSim 的 `OriginGeopoint` 一致（设 home 位置）。

## 2. 启动顺序（每次重开 Unreal 后都要重启 PX4 app —— docs 明确）
1. WSL：`cd ~/PX4-AirSim && make px4_sitl_default none_iris` → 等 `INFO [simulator] Waiting for simulator to connect on TCP port 4560` + mavlink 端口日志。
2. **防火墙**：Windows 放行 **TCP 4560** + **UDP 14540**。
3. UE 里 Play（读上面的 settings）。AirSim 连上后 PX4 窗口出现：
   `INFO [simulator] Simulator connected on UDP port 14560` + `INFO [ecl/EKF] EKF GPS checks passed` + `EKF commencing GPS fusion`。
4. 跨系统：AirSim(Windows) ↔ PX4 SITL(WSL) 经 `networkingMode=mirrored` 的 `127.0.0.1` 走 TCP 4560。QGC 可选（SITL 也可纯 python API 控制）。

## 3. MAVROS 连这个 PX4（若在 WSL 用 ROS 控制）
- MAVROS 走 **MAVLink**（不是 uXRCE；uXRCE 的 fastdds3/fastrtps2 在 Jazzy 是死结）。
- **launch 文件名是 `px4.launch`，不是 `px4.launch.py`**（mavros 包里只有 `px4.launch`，用 `.py` 会报 `file 'px4.launch.py' was not found`）。
  ```bash
  source /opt/ros/jazzy/setup.bash
  ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
  ```
- **先装 GeographicLib geoid**，否则 mavros_node 启动即崩：
  ```bash
  sudo apt install -y geographiclib-tools
  sudo geographiclib-get-geoids egm96-5
  ```
  报错特征：`UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm`。
- 验证：`ros2 topic echo /mavros/state --once` → `connected: true`。
- PX4 SITL 的 fcu 端口以启动日志为准（`on udp port XXX remote port YYY`），`14540@...` 参考值。

## 4. 判读速查
- `make px4_sitl` 只出 `Generating SPDX SBOM` 无 `pxh>` → 没启动，改用 `make px4_sitl_default none_iris`。
- PX4 窗口没出现 `Waiting for simulator ... TCP port 4560` → 版本太新（v1.18）或 model 不对。
- mavros 崩溃 `GeographicLib exception` → 装 `geographiclib-get-geoids egm96-5`。
- AirSim `no compatible vehicles`（PX4 模式）→ 先确认 `VehicleType` 是 `"PX4Multirotor"`（不是 `QuadX`，见主 SKILL.md）。
