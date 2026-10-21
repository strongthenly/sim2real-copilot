# AirSim ↔ ROS1/Noetic(WSL) 数字孪生栈（twin_ws + twin_bridge）

本会话实测搭成的一套"实机数字孪生"参考架构：**Windows 跑 AirSim（实机外观）**，**一个独立 WSL(ROS1 Noetic) 跑 twin_bridge + FAST-LIVO2**。可作为"AirSim 传感器 → 与实机同名话题 → 算法零改动"的模板。

## 两个 WSL 的分工（不要混）
| 发行版 | 系统 | 装什么 |
|---|---|---|
| `lyuav` | Ubuntu 24.04 | PX4-AirSim(v1.11.3) + ROS2（探索/MAVROS 用） |
| `ros1-twin` | Ubuntu 20.04 | `~/twin_ws`：twin_bridge + FAST-LIVO2（ROS1 Noetic） |

> PX4 **属于哪个 WSL 跑都行**——它只通过 **TCP 4560** 连 Windows AirSim；ROS 桥走 **RPC 41451**，两条链路独立。别以为 PX4 必须和 FAST-LIVO2 同一个发行版。
> 在 Ubuntu 20.04(gcc9) 编译 PX4 v1.11.3 是原生组合，**不需要 gcc-10**（gcc-10 只是 24.04/gcc13 的规避）。

## twin_bridge（核心桥，Python + rospy）
- 用 **msgpack RPC** 直连 AirSim `127.0.0.1:41451`（WSL 镜像网络下可达 Windows 的 localhost）。
- 每类传感器一个线程 + 独立轮询频率：IMU / LiDAR / RGB / Depth。
- **只读预览、不发控制**。时间戳取自 AirSim 传感器时间戳（`/clock`），并用"时间戳不可倒退"做护栏（倒退即报错要求重启）。
- IMU：AirSim 机体 FRD→FLU，`orientation_covariance[0] = -1`（**不泄露姿态真值**给估计器）。
- LiDAR：`getLidarData` → **`create_cloud_xyz32`（XYZ-only，无 rgb 字段）**。
- 相机：`simGetImages`（RGB8 / 32FC1 深度）。

话题契约（预览话题**独立命名**，不覆盖实机话题）：

| 话题 | 类型 | 来源/限制 |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | AirSim 传感器时间，勿混墙钟 |
| `/twin/imu/data_raw` | sensor_msgs/Imu | 机体 FRD→FLU，无姿态真值 |
| `/twin/lidar/points_preview` | sensor_msgs/PointCloud2 | 通用雷达 XYZ 局部 FLU，**非 Livox CustomMsg**（无逐点时间/反射率） |
| `/twin/camera/color/image_raw` | sensor_msgs/Image | AirSim RGB8，理想针孔 |
| `/twin/camera/color/camera_info` | sensor_msgs/CameraInfo | 来自模拟分辨率/FOV，非实机内参 |
| `/twin/camera/depth/image_raw` | sensor_msgs/Image | 理想平面深度 32FC1，非 D435 噪声模型 |

## AirSim 侧 settings 要点（本次实测）
- `Vehicles.<name>.Sensors` 加 `TwinImu`(SensorType 2) + `TwinLidarPreview`(SensorType 6；64 线 / 200000 pts·s⁻¹ / 10 rps / 水平 360° 垂直 -7°~52° / `DataFrame: SensorLocalFrame`)，用通用旋转雷达**近似** MID-360（非重复扫描无法复现）。
- `PawnPath` 指向自定义 pawn（`/Game/CustomDrone/BP_CustomDrone_GroundSafe`）= 实机外形。
- `CameraDefaults.CaptureSettings` 出 640×480 RGB + 深度，FOV 90。
- ⚠️ **改 settings 后，编辑器的"停止/重新 Play"不会重读**——必须**保存工程 → 关掉整个 `UE4Editor.exe` → 重开工程**。重启后再跑校验脚本确认 64 通道/200000 预览档真的加载。

## FAST-LIVO2 仿真预览（无实机标定时）
- 用官方 FAST-LIVO2（`~/twin_ws`，编出 `devel/lib/fast_livo/fastlivo_mapping`）。
- 预览配置：`lidar_type: 4`（走 **L515 瞬时点云**路径）、`img_en: 0`（关视觉）、外参用**单位矩阵**（仅预览）、`pcd_save: false`、`pose_output_en: false`。
- 启动脚本会**硬门禁**：ROS master 不在 → 退出；`/twin/imu` 或 `/twin/lidar` 无数据 → 退出；`/laserMapping` 已在跑 → 退出。**必须先有桥的数据流才启动**。
- **已确认 + 已修（2026-09-10 读源码 + 实测）**：桥的点云是 **XYZ-only（无 rgb）**，而 **FAST-LIVO2 没有任何一个 PointCloud2 handler 吃纯 xyz**——`l515_handler`（`lidar_type:4`）用 `pcl::PointCloud<pcl::PointXYZRGB>` + `pcl::fromROSMsg` ⇒ **要 `rgb` 字段**；`velodyne/oust64/xt32/robosense/pandar128` 要 **`intensity`**。纯 xyz 会报 `Failed to find match for field 'rgb'`。（`lidar_type` 常量：`AVIA=1`、`L515=4`。）
  **修法**：改桥 `bridge.py` 的 `read_lidar`，把 `create_cloud_xyz32(...)` 换成显式 `create_cloud` 带 `x,y,z,rgb`（rgb=0.0）；FAST-LIVO2 配置配 `lidar_type: 4`。**改完必须重启桥**（旧进程仍是 xyz-only）。
- **用仿真桥 + 实机标定跑 FAST-LIVO2（本次搭好，待用户执行）**：
  - `~/twin_ws/src/FAST-LIVO2/config/TwinSim.yaml`：`common` 话题指 `/twin/lidar/points_preview` + `/twin/imu/data_raw` + `/twin/camera/color/image_raw`，`img_en:0`、`lidar_en:1`、`preprocess.lidar_type: 4`、`scan_line: 4`、`blind: 0.3`；外参 `extrinsic_T/R` + `Rcl/Pcl` + IMU 噪声抄实机标定值（见 `fast-livo2-mapping → references/twin-bag-replay.md`）。
  - `TwinSim_camera.yaml`：抄实机 `camera_self.yaml`（fx 893.68 / fy 892.146 / cx 382.965 / cy 301.193 + 畸变）。
  - 启动：`rosparam load TwinSim.yaml` + **`rosparam load TwinSim_camera.yaml /laserMapping`（即使 `img_en:0` 也必须，否则崩 `Camera model not correctly specified`）** + `rosrun fast_livo fastlivo_mapping __name:=laserMapping`；RViz 用 `rviz_cfg/fast_livo2.rviz`。
  - 前置：桥必须先跑（`start-sensor-preview.ps1`；而桥需要 **Blocks 在 Play + PX4 连着**，见主 SKILL 第⑤条）。收尾：`rostopic hz /aft_mapped_to_init`（≈5Hz）+ `/cloud_registered`（≈10Hz）。
- 输出：`/cloud_registered`、`/aft_mapped_to_init`、`/path`。

## 启动顺序（实测）
1. 起 **PX4 SITL**（要 data 就必须——见主 SKILL 第⑤条；无 PX4 则先把车辆换 `SimpleFlight`）
2. **UE Blocks 进 Play**（AirSim 运行、41451 监听）
3. **传感器桥**（PowerShell 调 `wsl -d ros1-twin -- bash start_sensor_preview.sh …`；等出现 `Connecting to AirSim ... (IMU=… RGB=… Depth=…)`）
4. `rosrun twin_bridge check_streams.py _duration:=10` 验频率/时间戳
5. **FAST-LIVO2 预览** → 再开 **RViz**（fixed frame `twin_lidar_flu`）

## 实测频率（可见显示 vs NoDisplay）
- 可见跟随视角：IMU ~190–200 Hz、雷达 ~9.8 Hz、RGB/Depth ~3 Hz，时间戳无倒退。
- 切 AirSim `NoDisplay`（离屏）：RGB ~27–35 Hz、Depth ~5–19 Hz，仿真实时倍率 ~0.8–0.99。

## 安全/设计红线（务必保留）
- **标定与逐点时间未验证前，绝不把预览位姿回灌 PX4**：预览位姿输出被隔离到 `/twin/fast_livo/vision_pose_preview`（不是 `/mavros/vision_pose/pose`）。
- 通用 XYZ 点云**不得**冒充 Livox 数据冒充标定结果；预览只为验证"算法进程 + 建图输出 + 接口"。

## 正式链路（有实机标定时）：`start_fastlivo.py` 门禁 + 必填清单
拿到实机标定后走正式入口（`rosrun twin_bridge start_fastlivo.py --calibration <c> --algorithm-config <a> --camera-config <cam>`）。它**逐项校验、缺一项 `p.error` 直接拒启**——先把这三个 yaml 的 `status/twin_status` 都改成 `verified`，再填必填项：

- **`calibration.pending.yaml`**：`status: verified` + `lidar_per_point_timing_verified: true` + 必填非空：`algorithm_imu_source`、`lidar_imu_transform`、`lidar_camera_transform`、`body_lidar_transform`、`camera_intrinsics`、`sensor_time_offsets`、`imu_noise`。
- **algorithm config（`fast_livo.pending.yaml`）**：`twin_status: verified`；必填 `common.img_topic|lid_topic|imu_topic`、`extrin_calib.extrinsic_T/extrinsic_R/Rcl/Pcl`、`time_offset.imu_time_offset/img_time_offset`、`preprocess.lidar_type/scan_line/blind`、`imu.acc_cov/gyr_cov/b_acc_cov/b_gyr_cov`。**话题必须精确等于实机名**：`/livox/lidar`、`/livox/imu`、`/left_camera/image` —— **≠ 预览的 `/twin/*`**（对不上会报 wrong topics）。
- **camera config（`camera_d435.pending.yaml`）**：`twin_status: verified` + `cam_model/cam_width/cam_height/scale/cam_fx/fy/cx/cy/cam_d0..d3`。

要点：
- 标定未验证时，入口把位姿输出**隔离**到 `/twin/fast_livo/vision_pose_preview`；只有 **control config**（`status:verified` + `failsafe_behavior_verified` + `px4_version/fcu_url/gcs_url/mavlink ids`）也验证了，才解除隔离回灌真 `vision_pose`。
- **预览 `/twin/*` → 实机标准 `/livox/*` 的转换**由 `validated_sim_inputs.launch` 做（严格校验后才转发；或改用实机 Livox 驱动）。
- FAST-LIVO2 仓库自带 `config/camera_pinhole.yaml`（**针孔，D435 适用**）+ `avia / HILTI22 / MARS_LVIG / NTU_VIRAL` 等；`~/twin_ws` 里**已 clone + 编译好（`devel/lib/fast_livo/fastlivo_mapping`），不需要再 clone**。
- 要看"**定位数据**"就是 `/aft_mapped_to_init`（里程计/位姿）+ `/cloud_registered`（配准点云/地图）+ `/path`（轨迹）。
- RViz 启动器有**前置门禁**：master 不在→exit2、桥没发 `/twin/lidar/points_preview`→exit3、话题无消息→exit4。**没数据流时启动 RViz 必然失败**，别当成 RViz 坏。

## 拿到实机 bag 之后：回放复现定位（2026-09-10 跑通）
实机（fouruav）录的 bag 可在 ros1-twin 里用**实机标定 + 实机 bag** 跑**正式** FAST-LIVO2 复现定位。两个必踩的坑：
1. bag 里 `/livox/lidar` 的类型是 `livox_ros_driver2/CustomMsg`，而工作区装的是 `livox_ros_driver`（v1）→ **先 `rosmsg md5` 比对**：两者 **md5 相同**（ROS1 按 md5 判定兼容），但 `rosbag play` 仍需能加载 bag 的类型名 → 造一个**最小 msg-only `livox_ros_driver2` 包**（复制 v1 的 `.msg`）+ `catkin_make --pkg livox_ros_driver2`。
2. FAST-LIVO2 **即使 `img_en: 0` 也必须把相机配置 load 到节点命名空间**：`rosparam load <camera>.yaml /laserMapping`，否则崩 `terminate ... Camera model not correctly specified`。

完整配方（实机标定实测值抄录、启动/回放命令、成功验证频率）见 **`fast-livo2-mapping` 技能 → `references/twin-bag-replay.md`**。另：实机 `~/Desktop/Bags/` 里名字叫 `odin_data_*.bag` 的**可能装的是 MID360 数据**，用 `rosbag info` 看话题才准。
