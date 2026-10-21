# 实机 bag 在 ROS1 工作区回放复现 FAST-LIVO2 定位（ros1-twin / twin_ws）

场景：本机 WSL `ros1-twin`（Ubuntu 20.04 + ROS Noetic）的 `~/twin_ws` 里已有编译好的
FAST-LIVO2 + livox_ros_driver + twin_bridge；数据用实机（fouruav）录的 bag；
目标 = **用实机标定 + 实机 bag 跑正式 FAST-LIVO2，看定位输出**（不是仿真桥的预览模式）。

> 实测日期 2026-09-10，全链路跑通。

## 1. 实机标定数据在哪（fouruav）
`/home/fouruav/catkin_ws/src/FAST-LIVO2/config/`：
- `MID360.yaml`（基准）、`MID360_Fly.yaml`（飞行）、`MID360_Fly_Lidar.yaml`（建图）
- **`MID360_Fly_Lidar.yaml` 才是 launch 实际加载的**（`launch/mapping_MID360.launch` 里 `rosparam load` 的就是它；`MID360.yaml` 那行被注释掉了）
- `camera_self.yaml`（自家相机内参；launch 里 load 进 node 命名空间）
- launch 还带 `pcd_save` 参数开关：true=录 pcd / false=仅建图

实机实测值（直接抄，勿编）：
```
extrin_calib:
  extrinsic_T: [-0.011, -0.02329, 0.04412]          # 雷达-IMU 平移
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
  Rcl: [-0.000601, -0.999899,  0.014194,
        -0.006397, -0.014190, -0.999879,
         0.999979, -0.000691, -0.006388]            # 相机-雷达 旋转
  Pcl: [0.011720, -0.078665, -0.048631]             # 相机-雷达 平移
imu: acc_cov 0.5, gyr_cov 0.3, b_acc_cov 0.0001, b_gyr_cov 0.0001
preprocess: lidar_type 1 (Livox Avia), scan_line 4(建图)/6(飞行), blind 0.2~0.3, point_filter_num 1~3
time_offset: imu_time_offset 0.0, img_time_offset 0.1, exposure_time_init 0.0
话题: img /usb_cam/image_raw, lid /livox/lidar, imu /livox/imu_tf
```
`camera_self.yaml`：`Pinhole, 800x600, fx 893.68, fy 892.146, cx 382.965, cy 301.193,
d0..d3 = 0.072654, -0.091823, 0.001453, -0.002248`
`livox_ros_driver2/config/MID360s_config.json`：`lidar_type 8`，雷达 IP `<NET_LIDAR>.12`，host `<NET_LIDAR>.5`

## 2. 坑1：bag 的消息类型是 `livox_ros_driver2`，工作区装的是 `livox_ros_driver`（v1）
实机 bag 里 `/livox/lidar` = `livox_ros_driver2/CustomMsg`；twin_ws 只有 v1。

**先 `rosmsg md5` 对比**：v1 与 v2 的 CustomMsg **md5 完全相同**
（`e4d6829bdfe657cb6c21a746c86b21a6`）→ 定义一致。ROS1 连接按 **md5** 判定兼容，
所以 v2 发布者能被 v1 订阅者（FAST-LIVO2）收到。

**但 `rosbag play` 仍要能在本环境加载 bag 里的类型名** → 在工作区造一个**最小 msg-only 包** `livox_ros_driver2`：
```bash
mkdir -p ~/twin_ws/src/livox_ros_driver2/msg
cp ~/twin_ws/src/livox_ros_driver/msg/*.msg ~/twin_ws/src/livox_ros_driver2/msg/   # CustomMsg.msg + CustomPoint.msg
# package.xml: name=livox_ros_driver2 / build_depend message_generation,std_msgs / exec_depend message_runtime,std_msgs
# CMakeLists.txt:
#   find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
#   add_message_files(DIRECTORY msg FILES CustomMsg.msg CustomPoint.msg)
#   generate_messages(DEPENDENCIES std_msgs)
#   catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
cd ~/twin_ws && source /opt/ros/noetic/setup.bash && catkin_make --pkg livox_ros_driver2
rosmsg md5 livox_ros_driver2/CustomMsg      # 必须仍为 e4d6829bdfe657cb6c21a746c86b21a6
```
- ⚠️ `<maintainer email="...">` 必须是合法邮箱（写 `twin@local` 会被 catkin_pkg 拒：
  `InvalidPackage: Invalid email "twin@local" for person`）→ 用 `xxx@example.com`
- 只需消息定义，**不需要**编译完整 Livox 驱动（那要 Livox-SDK）

## 3. 坑2：FAST-LIVO2 即使 `img_en: 0` 也必须加载相机模型参数
只 `rosparam load` 算法配置就启动会崩：
```
[ERROR] Cannot find value for parameter: laserMapping/cam_model, will try again.
terminate called after throwing an instance of 'std::runtime_error'
  what():  Camera model not correctly specified.
```
→ 必须把相机配置 **load 到节点命名空间**：`rosparam load <camera>.yaml /laserMapping`
（相机内参是构造函数里就读的，跟 img_en 无关）

## 4. 启动 + 回放配方（ros1-twin；roscore/node 用 setsid 常驻）
```bash
source /opt/ros/noetic/setup.bash && source ~/twin_ws/devel/setup.bash
pkill -f fastlivo_mapping; pkill -f rosmaster; sleep 3
setsid roscore > /tmp/roscore.log 2>&1 < /dev/null &          # sleep 6
rosparam load ~/twin_ws/src/FAST-LIVO2/config/TwinReal.yaml    # 实机标定版算法配置（话题 /livox/*）
rosparam load ~/twin_ws/src/FAST-LIVO2/config/TwinReal_camera.yaml /laserMapping
setsid rosrun fast_livo fastlivo_mapping __name:=laserMapping > /tmp/fastlivo.log 2>&1 < /dev/null &   # sleep 8
rosbag play --rate 0.5 ~/bags/<实机bag>.bag                     # 慢放便于中途检查
```
`TwinReal.yaml` = 上表实机值 + `common.lid_topic: /livox/lidar`、`imu_topic: /livox/imu_tf`、
`img_en: 0`、`lidar_type: 1`、`scan_line: 4`；`TwinReal_camera.yaml` = `camera_self.yaml` 的值。

**验证（13.4s bag 实测通过）**：
- `/aft_mapped_to_init` ≈ **5 Hz**（定位里程计）✅  ← 这就是"定位数据"
- `/cloud_registered` ≈ **10 Hz**（注册点云/地图）✅
- `/path` 可能 `no new messages`（发得稀疏，正常）
- 日志正常：`[Preprocess] Input point number: 19968 → Output point number: 14709`、
  `[LIO] Raw feature num: 14709, effective feature num: 6628, average residual: ~0.01`、`Update Voxel Map`

## 5. 关键概念：`lidar_type: 1` 要逐点时间 → 仿真桥的 XYZ-only 点云喂不了
实机用 `lidar_type: 1`（Livox Avia），**要求 Livox CustomMsg 的逐点时间**。
AirSim 仿真桥发的点云是 **XYZ-only**（`create_cloud_xyz32`，无逐点时间）→ 喂不了类型 1。
- **正式链路只能用实机 bag 或实机在线数据**（带 CustomMsg）
- 仿真数据只能走 `lidar_type: 4`（瞬时点云路径）做 LIO 预览
- ⚠️ **但 `lidar_type: 4` 也不是无条件的**（2026-09-10 读源码确认）：`common_lib.h` 里 `AVIA=1`、`L515=4`；`preprocess.cpp` 的 `process(PointCloud2)` 按 `lidar_type` 分派 handler。`l515_handler` 用 `pcl::PointCloud<pcl::PointXYZRGB>` + `pcl::fromROSMsg` ⇒ **要 `rgb` 字段**；`velodyne/oust64/xt32/robosense/pandar128` 要 **`intensity`**——**没有任何 handler 吃纯 XYZ-only**。纯 xyz 喂进去报 `Failed to find match for field 'rgb'`。
  **修法**：改桥 `bridge.py` 的 `read_lidar`，把 `create_cloud_xyz32(...)` 换成 `create_cloud(..., [x,y,z,rgb(FLOAT32@12)], [[x,y,z,0.0],...])`（rgb 全 0），再**重启桥**；FAST-LIVO2 侧配 `lidar_type: 4`。改用仿真桥跑时的算法/相机配置见 **airsim-drone-exploration** 技能里的 `references/airsim-fastlivo2-ros1-twin.md` §2.2（`TwinSim.yaml` / `TwinSim_camera.yaml`，话题指向 `/twin/*`）。
- 这也解释了为什么 Codex 的 `start_fastlivo.py` 正式入口要卡
  `p.error('Calibration/timing not verified; missing: ...')`
  （它要求 `status: verified` + `lidar_per_point_timing_verified: true` + 实机话题 `/livox/*`）

## 6. 实机 bag 参考（fouruav `~/Desktop/Bags/`）
- 名字都叫 `odin_data_<时间戳>.bag`，但内容**可能是 MID360 数据**——`rosbag info` 看话题才知道
  （实测 `odin_data_20260910_210320.bag` 内含 `/livox/lidar` CustomMsg + `/livox/imu_tf` + 输出 `/cloud_registered` + `/path`）
- 好的验证 bag 应同时含**输入**（/livox/lidar、/livox/imu_tf）与**输出**（/cloud_registered、/path），
  这样自己跑的结果能跟 bag 里实机那次跑的轨迹对比

## 7. 看结果：RViz Fixed Frame + 坐标判读（2026-09-10 实测）

### 7.1 RViz 打开却"没有图像流" = Fixed Frame 没设
FAST-LIVO2 的**所有输出**（`/cloud_registered`、`/aft_mapped_to_init`、`/path`）`header.frame_id` 都是 **`camera_init`**。
自带 `rviz_cfg/fast_livo2.rviz` 里 Global Options 的 Fixed Frame 是**空占位 `<Fixed Frame>`** → RViz 能开但**点云/轨迹都不显示**。
**改法**：RViz 左上 `Global Options → Fixed Frame` 填 **`camera_init`** → 回车即出图。
再加 display：`Odometry`（Topic `/aft_mapped_to_init`）、`Path`（Topic `/path`）、`PointCloud2`（`/cloud_registered`）。

### 7.2 查坐标的命令
```bash
source /opt/ros/noetic/setup.bash && source ~/twin_ws/devel/setup.bash
rostopic echo /aft_mapped_to_init/pose/pose/position          # 位置 x y z（实时流）
rostopic echo -n 1 /aft_mapped_to_init/pose/pose/position     # 只取一帧
rostopic echo /aft_mapped_to_init/pose/pose                   # 位置 + 姿态四元数
```
⚠️ 话题名是 **`/aft_mapped_to_init`**（`init` 有 `t`）；写成 `/aft_mapped_to_ini` 会得到 `does not appear to be published yet` + rospy `ValueError: topic name is not a non-empty string`（后一个报错是 rostopic 自己的崩溃，容易误判成节点没起）。

### 7.3 "z 坐标在跳变" 正常吗？—— 静止时 mm 级抖动是正常的
无人机**静止**时实测连续 12 帧：
```
x: 0.0014 ~ 0.0037 m   (1~4 mm)
y: 0.0045 ~ 0.0071 m   (4~7 mm)
z: -0.00004 ~ 0.00014 m (±0.1 mm)
```
- **位置在原点附近 ±mm 级微抖 = 正常估计噪声**（LiDAR-惯性定位静止时就是这个量级），**不是异常**。
- `y` 有 ~5mm 小偏置 = 初始对齐的正常小误差，可忽略。
- **真正的异常 = 持续朝一个方向漂移（发散）**。
- IMU 体检：`rostopic echo /twin/imu/data_raw/linear_acceleration` 的 **z ≈ ±9.8**（重力对）、`angular_velocity` ≈ 0 → IMU 正常。
- 静止只能验证"不漂"；**要验证"跟踪"，必须让无人机飞**（QGC Arm/Takeoff，或 offboard），坐标才会跟随真实轨迹。
