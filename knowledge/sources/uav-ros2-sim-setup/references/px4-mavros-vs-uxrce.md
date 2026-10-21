# PX4 ↔ ROS2 通信选型：MAVROS(MAVLink) 而非 uXRCE

## 为什么 uXRCE 在 Jazzy 是死结（实证）
- 现代 PX4(main/v1.15+) **只用 uXRCE-DDS**，`pxh>` 里已无 `rtps` 命令（Fast-RTPS 被移弃）。
- uXRCE 的标准 agent `micro-ROS-Agent`（main）**强制要 Fast DDS v3**（`find_package(fastdds 3)`）。
- Jazzy **只有 Fast RTPS 2**（`ros-jazzy-fastrtps`）。强行装 `ros-rolling-fastdds`：
  - 装到 `/opt/ros/rolling`，需 `export CMAKE_PREFIX_PATH=/opt/ros/rolling` 才被找到；
  - 且与 jazzy 的 fastrtps2 同含 `eProsima_atomic` target → CMake **target 冲突** `Some (but not all) targets ... already defined`。
- 退到 micro_ros_agent 旧版 v2.0.x（fastrtps 2）也不稳：tag `v2.0.4` 需 `git fetch --tags` 才有；协议比 PX4 main 旧，大概率连不上。

→ 结论：**在 Jazzy 上，PX4-ROS2 通信用 MAVROS(MAVLink)**，天然兼容，无需 fastdds3。

## MAVROS 正确接法（实测通过）
1. 装：`sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs ros-jazzy-mavros-extras`（走 packages.ros.org，502 切节点）。
2. 启动 launch 文件名是 **`px4.launch`（无 .py）**（`ls /opt/ros/jazzy/share/mavros/launch/` 确认）。
3. 启动：`ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557`（PX4 SITL 先跑着）。
4. **必须装 GeographicLib geoid 否则崩溃**：
   `sudo apt install -y geographiclib-tools` + `sudo geographiclib-get-geoids egm96-5`。
   崩溃报错：`UAS: GeographicLib exception: /usr/share/GeographicLib/geoids/egm96-5.pgm ... install_geographiclib_dataset.sh`。
5. 验证：`ros2 topic echo /mavros/state --once` → `connected: true` = 打通。

## 下一步（探索执行端）
- offboard 控制：`/mavros/setpoint_raw/local` 持续发 setpoint + `/mavros/set_mode` custom_mode="OFFBOARD" + arm（`/mavros/cmd/arming`）。
- PX4 SITL 默认 mode `AUTO.LOITER`（悬停），offboard 才是控制飞行。

## PX4 gz_x500 传感器
- **不带 3D LiDAR**（只有相机/rangefinder）。探索要 LiDAR 须改 `Tools/simulation/gz/models/x500_base/model.sdf` 加 `gpu_lidar` sensor（x500/model.sdf 是 `include model://x500_base` + 电机插件，机体/传感器在 x500_base）。改 PX4 文件前备份+贴 diff。
