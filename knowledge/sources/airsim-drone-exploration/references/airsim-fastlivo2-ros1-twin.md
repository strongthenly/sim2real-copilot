# AirSim 数字孪生 + FAST-LIVO2（ROS1 Noetic）

场景：在 Windows UE4.27/AirSim 之上，用 WSL 里的 ROS1 Noetic 跑真实 **FAST-LIVO2 (LIO)**。
不是"仿真里跑 SLAM"，而是用 AirSim 当**传感器数据源**、桥到 ROS1 话题、喂给真实算法。

## 架构
```
UE4/AirSim (Windows, RPC 41451)  --twin_bridge-->  /twin/* (ROS1, WSL ros1-twin)  -->  FAST-LIVO2
```
- 发行版 `ros1-twin`（**Ubuntu 20.04 + ROS Noetic**），工作区 `~/twin_ws`；**与 ROS2 的 lyuav 是两个独立 WSL 发行版**，别混。
- 桥 `twin_bridge`（ROS1 包，`bridge.py`）：直连 AirSim RPC（msgpack，默认 41451），用 `simGetImages` / `getLidarData` / `getImuData` 拉数据 → 发 ROS1 话题。
- 一键脚本（Windows PowerShell 调 `wsl -d ros1-twin -u <USER> -- bash <script.sh>`）：
  - `start-sensor-preview.ps1` → 起桥（默认 IMU 220Hz / RGB 30Hz / Depth 5Hz；自动探测 AirSim host，可 `-AirSimHost x.x.x.x`）
  - `start-rviz-preview.ps1` → RViz（固定系 `twin_lidar_flu`）
  - `start-fastlivo-preview.ps1` → 起 FAST-LIVO2 预览
  - `check-airsim-lidar.ps1` → 校验雷达配置是否真加载（`mid360_preview_profile_loaded: true`）
  - `verify-live-streams.ps1` → 临时 ROS master + 采 10s 验五路话题（`LIVE_STREAM_VERIFY_COMPLETE`）

## 数据接口（/twin/*，刻意与实机话题隔离命名）
| 话题 | 类型 | 来源/限制 |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | AirSim 传感器时间（禁混墙钟） |
| `/twin/imu/data_raw` | sensor_msgs/Imu | AirSim 机体 IMU，FRD→FLU，`orientation_covariance[0]=-1`（不泄露姿态真值） |
| `/twin/lidar/points_preview` | sensor_msgs/PointCloud2 | **默认 XYZ-only**（`create_cloud_xyz32`），局部 FLU；**不是 Livox CustomMsg**（见 §2.2 改法） |
| `/twin/camera/color/image_raw` | sensor_msgs/Image | AirSim RGB8；理想针孔 |
| `/twin/camera/color/camera_info` | CameraInfo | 来自模拟分辨率/FOV（非实机内参） |
| `/twin/camera/depth/image_raw` | Image | 理想平面深度，32FC1 米（非 D435 噪声模型） |

## AirSim 侧传感器配置（`Documents/AirSim/settings.json`）
```json
"Vehicles": { "PX4": {
  "VehicleType": "PX4Multirotor", "UseSerial": false, "UseTcp": true, "TcpPort": 4560,
  "ControlPortLocal": 14540, "ControlPortRemote": 14580, "LockStep": true,
  "Sensors": {
    "Barometer": { "SensorType": 1, "Enabled": true },
    "TwinImu":   { "SensorType": 2, "Enabled": true },
    "TwinLidarPreview": { "SensorType": 6, "Enabled": true,
      "NumberOfChannels": 64, "PointsPerSecond": 200000, "RotationsPerSecond": 10,
      "HorizontalFOVStart": -180, "HorizontalFOVEnd": 180,
      "VerticalFOVUpper": 52, "VerticalFOVLower": -7, "Range": 40,
      "X":0,"Y":0,"Z":0, "DataFrame": "SensorLocalFrame" }
  } } },
"CameraDefaults": { "CaptureSettings": [ {"ImageType":0,"Width":640,"Height":480,"FOV_Degrees":90},
                                         {"ImageType":1,"Width":640,"Height":480,"FOV_Degrees":90} ] }
```
- SensorType 编号：**1=Barometer、2=Imu、6=Lidar**。
- 实测频率（可见显示模式）：IMU ~190–200Hz、LiDAR ~9.8Hz、RGB/Depth ~3Hz；切 AirSim `NoDisplay` 后 RGB 可达 ~27–35Hz、Depth ~5–18Hz（算法跑时先 `enable-camera-performance.ps1` 再重启 Blocks；要恢复跟随视角跑 `enable-camera-display.ps1` 再重启）。
- 雷达预览按 MID-360 标称视场(360°/ -7°~52°)、10Hz、200000 点/秒、64 规则通道近似。

## FAST-LIVO2 预览模式（无实机标定时的 LIO 验证）
- 配置里：`lidar_type: 4`（**L515 瞬时点云处理路径**，绕开需要 Livox 逐点时间的路径）、`img_en: 0`（关视觉融合）、**身份外参**（仅仿真预览）、`imu_en: true`。
- `start_fastlivo_preview.sh` 启动前**先 gate 校验**：ROS master 可用 + `/twin/imu/data_raw` 与 `/twin/lidar/points_preview` 有数据 + `/laserMapping` 未在跑；不满足直接退出（防止错数据进算法）。
- 位姿输出**隔离**到 `/twin/fast_livo/vision_pose_preview`（把上游 `/mavros/vision_pose/pose` 重映射），**不回灌 PX4**。
- 观察：`rosnode info /laserMapping`、`rostopic hz /cloud_registered`、`rostopic hz /aft_mapped_to_init`、`rostopic hz /path`。

---

# 二、把 FAST-LIVO2 真跑起来（实机标定 / 仿真话题 / bag 回放）

## 2.1 三个必知（否则起不来 / RViz 没图）
1. **`cam_model` 必须要，哪怕 `img_en: 0`**：FAST-LIVO2 初始化仍读 `laserMapping/cam_model`，**不 load 相机配置就直接崩**：
   ```
   [ERROR] Cannot find value for parameter: laserMapping/cam_model, will try again.   (刷屏)
   terminate called after throwing an instance of 'std::runtime_error'
     what():  Camera model not correctly specified.
   ```
   **修法**：启动前 **`rosparam load <camera.yaml> /laserMapping`**（注意是 load 到 **节点命名空间 `/laserMapping`**，不是全局）。相机 yaml 至少含 `cam_model / cam_width / cam_height / scale / cam_fx / cam_fy / cam_cx / cam_cy / cam_d0..d3`。
2. **`lidar_type` 常量与字段要求**（源码 `include/common_lib.h`：`AVIA=1`、`L515=4`；另有 VELO16/OUST64/XT32/PANDAR128/ROBOSENSE）：

   | lidar_type | handler | 输入 ROS 类型 | **必需字段** |
   |---|---|---|---|
   | 1 (AVIA) | `avia_handler` | `livox_ros_driver/CustomMsg` | Livox 逐点时间/线号/反射率（`offset_time`/`line`/`tag`/`reflectivity`） |
   | 4 (L515) | `l515_handler` | `sensor_msgs/PointCloud2` | **`rgb`**（`pcl::fromROSMsg` 到 `pcl::PointXYZRGB`） |
   | Velodyne/Ouster/Robosense/XT32/Pandar128 | 各自 handler | `PointCloud2` | **`intensity`** |

   → **纯 XYZ 的 PointCloud2（`create_cloud_xyz32`）哪个 handler 都喂不了**（L515 要 rgb、其余要 intensity）→ 见 §2.2。
3. **RViz 不出图 = Fixed Frame 没设**：FAST-LIVO2 所有输出（`/cloud_registered`、`/aft_mapped_to_init`、`/path`）的 `header.frame_id` 都是 **`camera_init`**；自带 `rviz_cfg/fast_livo2.rviz` 里 Fixed Frame 是空占位 `<Fixed Frame>` → **RViz 打开但什么都不显示**。**在 RViz 左上 `Global Options → Fixed Frame` 填 `camera_init`** 即可（或改 .rviz 文件里对应字段）。

## 2.2 让仿真桥的 XYZ 点云被 FAST-LIVO2 吃（补 rgb，**已实测验证**）
改 `bridge.py` 的 `read_lidar`：把 `create_cloud_xyz32(...)` 换成带 rgb 字段的 `create_cloud(...)`：
```python
fields=[point_cloud2.PointField('x',0,point_cloud2.PointField.FLOAT32,1),
        point_cloud2.PointField('y',4,point_cloud2.PointField.FLOAT32,1),
        point_cloud2.PointField('z',8,point_cloud2.PointField.FLOAT32,1),
        point_cloud2.PointField('rgb',12,point_cloud2.PointField.FLOAT32,1)]
pts=[[float(q[0]),float(q[1]),float(q[2]),0.0] for q in p]
self.lidar.publish(point_cloud2.create_cloud(Header(stamp=s,frame_id='twin_lidar_flu'),fields,pts))
```
配置 `lidar_type: 4`（L515 路径，不需要 Livox 逐点时间）→ **跑通**（实测 `/twin/lidar` 10.6Hz / `/twin/imu` 197Hz → `/cloud_registered` 10.6Hz / `/aft_mapped_to_init` 9.3Hz，LIO 日志有 `[ Preprocess ] Input point number` / `[ LIO ] ... average residual ...`）。
- **改完必须重启桥**（旧进程还用旧代码；症状是 FAST-LIVO2 报 `Failed to find match for field 'rgb'`）。
- 仿真下外参可填实机值（影响 mm 级）或单位阵（AirSim 的 TwinImu/TwinLidar 同点 X=Y=Z=0）。

## 2.3 bag 的 msg 类型不匹配（`livox_ros_driver2` vs `livox_ros_driver`）
实机 bag 的 `/livox/lidar` 用 **`livox_ros_driver2/CustomMsg`**，而工作区常只装了 `livox_ros_driver`（v1）→ `rosbag play` 加载不了该消息类。
- **先比 md5**：`rosmsg md5 livox_ros_driver/CustomMsg` vs `rosbag info <bag>` 里的 md5 —— 本会话两者相同（`e4d6829bdfe657cb6c21a746c86b21a6`）= **消息定义一致**。
- **最小修法（msg-only 包）**：在 `src/` 建一个**只含消息**的 catkin 包 `livox_ros_driver2`：
  - `msg/CustomMsg.msg` + `msg/CustomPoint.msg` **直接拷 v1 的同名文件**（定义相同 ⇒ md5 相同）
  - `CMakeLists.txt`：`add_message_files(DIRECTORY msg FILES CustomMsg.msg CustomPoint.msg)` + `generate_messages(DEPENDENCIES std_msgs)` + `catkin_package(CATKIN_DEPENDS message_runtime std_msgs)`
  - `catkin_make --pkg livox_ros_driver2`
  - 之后 `rosbag play` 能加载该类型；**FAST-LIVO2 的 v1 订阅者靠 md5 相同照样能连**（ROS1 连接按 md5 匹配，不是类型名）。
- ⚠️ `package.xml` 的 `<maintainer email="...">` **必须是合法邮箱**：写 `xxx@local` 会让 `catkin_make` 报 `Invalid email "..." for person "..."`。
- 也可以用 `pscp -pw <pw> -hostkey SHA256:... user@host:/path/x.bag C:\...` 把实机 bag 拉回本地。

## 2.4 实机标定数据在哪（fouruav）
实机 `~/catkin_ws/src/FAST-LIVO2/config/`：
- `MID360.yaml`（基础）/ `MID360_Fly.yaml`（飞行，低算力）/ `MID360_Fly_Lidar.yaml`（建图，高质量）→ **外参 + IMU 噪声 + 预处理**
- `camera_self.yaml`（自研相机内参）；`launch/mapping_MID360.launch` 里 load 的是 `MID360_Fly_Lidar.yaml` + `camera_self.yaml`

实测值（本会话抄录）：
- 外参：`extrinsic_T=[-0.011,-0.02329,0.04412]`、`extrinsic_R=[1,0,0,0,1,0,0,0,1]`；相机-雷达 `Rcl=[-0.000601,-0.999899,0.014194, -0.006397,-0.014190,-0.999879, 0.999979,-0.000691,-0.006388]`、`Pcl=[0.011720,-0.078665,-0.048631]`
- 预处理：`lidar_type: 1`(Avia)、`scan_line: 4`（建图）/`6`（飞行）、`blind: 0.2~0.3`、`point_filter_num: 1~3`
- IMU 噪声：`acc_cov 0.5 / gyr_cov 0.3 / b_acc_cov 1e-4 / b_gyr_cov 1e-4`
- 相机：`800x600, fx893.68, fy892.146, cx382.965, cy301.193, d0..d3 = 0.072654,-0.091823,0.001453,-0.002248`
- 实机话题：`/livox/lidar`、`/livox/imu_tf`、`/usb_cam/image_raw`
- 实机 bag：`~/Desktop/Bags/*.bag`（含 `/livox/lidar`(CustomMsg) + `/livox/imu_tf` + FAST-LIVO2 输出 `/cloud_registered`+`/path`，可回放复现；`rosbag info` 看话题/时长）

## 2.5 判读：静止时坐标"跳"多半是正常噪声，不是 bug
- 无人机**静止** → `/aft_mapped_to_init/pose/pose/position` 应稳定在原点附近。本会话实测 x/y ≈ 2~7 **mm**、z ≈ ±0.1 **mm**，**抖动幅度仅 0.1mm 级 = 正常估计噪声**（LiDAR-惯性定位静止时就是 mm 级），**不是异常跳变**。
- **真正异常 = 持续朝一个方向漂移**（发散）。
- IMU 体检：`rostopic echo /twin/imu/data_raw/linear_acceleration` 的 z 应 ≈ ±9.8（重力对）、角速度 ≈ 0。
- 想让坐标"动"来验证**跟踪**：让无人机飞（QGC Arm/Takeoff 或 offboard）——坐标才会跟随真实轨迹。

## 2.6 长驻/交互式服务（PX4 SITL、RViz）交给用户跑或用 tmux
- **PX4 SITL 用后台方式（无 tty）启动 → `pxh>` shell 读到 stdin EOF 会立刻 `pxh> Exiting NOW.` 退出**（日志里能看到 `Simulator connected on TCP 4560` 之后马上 Exiting）。→ 必须**前台终端**跑，或 **tmux**（`tmux new -s px4` → 运行 → `Ctrl+B` 再 `D` 脱离；回去 `tmux attach -t px4`），或**直接把命令交给用户跑**。
- RViz 是 GUI（WSLg）同理——给用户命令让他在自己终端起，或 `setsid rviz -d <cfg> &` 后台起。
- 用户偏好：这类"要占终端/要看窗口"的服务，**给命令让他自己执行**（"你告诉我启动链路，我来启动"/"我自己来跑"），而不是 agent 一跑就阻塞。

---

## 关键坑（预览模式专属）
1. **`lidar_type: 4`(L515) 要求 PointCloud2 带 `rgb` 字段，而桥默认发纯 XYZ（`create_cloud_xyz32`）→ 预处理必失败**。**已验证修法见 §2.2**（桥补零值 rgb 字段）。
2. **AirSim 通用旋转雷达 ≠ MID360 非重复扫描**：没有逐点采样时间 / 真实线号 / 反射强度。**绝不能伪造这些字段**喂 FAST-LIVO2。正式 MID-360 LIVO 链路必须等：MID360 逐点时间 + 雷达-IMU / 雷达-相机 / 机体-传感器外参 + D435 内参畸变 + IMU 噪声 + 时间偏移 全部标定（**实机标定值见 §2.4**）。
3. **`ClockType: "SteppableClock"` + `LockStep: true`** → 仿真时间由 **PX4 步进**。AirSim 单独 Play 很可能**时间冻结、传感器无新数据**（桥报"仿真时间倒退/无消息"）。先确认 PX4 SITL（`~/PX4-AirSim`，见 `px4-sitl-integration.md`）在跑；否则改 `ClockType`。
4. **改 settings.json 后，编辑器里停止/重开 PIE 不会重读 AirSim 设置**——必须**保存工程 + 关闭整个 `UE4Editor.exe` + 重开工程**。重启后跑 `check-airsim-lidar.ps1`，只有 `mid360_preview_profile_loaded: true` 才算服务端真加载了 64 通道/200000 点配置。
5. **两个 `UE4Editor.exe` 会抢 41451**：确认只有一个在跑，否则桥连到旧实例/拿不到数据。
6. 桥默认是**只读预览**（不向飞控写控制）。算法闭环（MAVROS 端口、里程计回灌）需在确认 PX4 当前 MAVLink 拓扑后再配，防与已有控制链路争用。

## 排查起手式
```bash
# 1) 有没有残留节点/进程
rosnode list                      # master 没起会报 "Unable to communicate with master!"（= 干净，无残留）
pgrep -af "px4|mavros|roscore|fastlivo_mapping|bridge.py"
# 2) 数据在不在
source /opt/ros/noetic/setup.bash && source ~/twin_ws/devel/setup.bash
rosrun twin_bridge check_streams.py _duration:=10
# 3) AirSim 是否在跑（Windows）
#    TCP 127.0.0.1:41451 通 = Blocks 在 Play；不通 = 没 Play/没跑
# 4) FAST-LIVO2 起没起 / 输出在不在
rosnode list | grep laserMapping
rostopic hz /cloud_registered      # 地图
rostopic hz /aft_mapped_to_init    # 定位（注意拼写：init 有 t，别写成 ini）
```
