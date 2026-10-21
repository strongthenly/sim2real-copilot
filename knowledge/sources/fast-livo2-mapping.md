---
name: fast-livo2-mapping
description: 运维 FAST-LIVO2 建图（Jetson NX + MID360）：PCD 保存/关闭/启动流程、异步清理线程防误杀、二次启动提速、PCD 点数骤减排查、零残留保证。Use when 建图 PCD 异常（缺失/变小/损坏）、关闭后进程残留、启动/关闭慢或卡死、需要改 start_fastlivo2.sh / nx_command_server.py 的关闭逻辑；或在 ROS1 工作区（ros1-twin/twin_ws）用实机 bag 回放复现 FAST-LIVO2 定位（livox_ros_driver2↔v1 消息 md5 一致、cam_model 必填、实机标定抄录、RViz Fixed Frame=camera_init、静止时 mm 级坐标抖动属正常、roscore 绕开提速、run_id 冲突、AirSim LockStep 时序、RViz 窗口缩小）。
---

# FAST-LIVO2 建图 PCD 保存/关闭/启动 运维

## 架构地图（fouruav NX）
- 启动入口: `scripts/NX_code/start_fastlivo2.sh true|false` — true=建图用 `MID360_Fly_Lidar.yaml`，false=飞行用 `MID360_Fly.yaml`（按 `$WITH_CLOUD` 选 CONFIG_NAME，`$2` 可覆盖 lidar/fly）；**脚本全路径 `~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_fastlivo2.sh`，不在 home 目录**
- 启动链路（2026-08-10 确认）：rs_d435_camera.launch → msg_MID360s.launch（Livox MID-360S 驱动）→ mid360_imu_tf.launch → rviz → mapping_MID360.launch → px4_and_others.launch（px4.launch=ttyTHS0:921600 + static TF map→odom/map→camera_init + realman）
- **FAST-LIVO2 发布话题（LIVMapper.cpp）**：`/cloud_registered`（PointCloud2 注册点云 ~10Hz）、`/aft_mapped_to_init`（Odometry 里程计高频）、`/path`、`/Laser_map`、`/dyn_obj*`、`/LIVO2/imu_propagate`——下游消费者（SUPER 规划器等）订阅这俩，不是 /livox/lidar
- launch: `task_run_scripts/launch/mapping_MID360.launch` 加载 `config/$(config_name).yaml` + `camera_self.yaml`（rosparam 会覆盖 yaml 里的同名参数！）
- 指令服务: `nx_command_server2.py`（317B 包装）→ `nx_command_server.py`（~98KB 实际逻辑）；UDP 6003 收 QGC 指令（Odin-MindCloud=启动建图 / KillLiDAR=关闭），HTTP 6004
- 服务启停: `start_qgc_servers.sh`（roscore + nx_touch_server2 + nx_command_server2 + rviz_full_stream2 四服务，SIGTERM 触发 cleanup）

## PCD 保存机制（LIVMapper.cpp）
- `savePCD()` **只在 `run()` 主循环 `while(ros::ok())` 退出后调用一次**（LIVMapper.cpp:553）
- `ros::ok()` 只被 **SIGINT（Ctrl+C）** 置 false；**SIGTERM 默认杀死进程 → savePCD 不执行 → PCD 丢**。这是"PCD 丢失/损坏"的根源
- `pcd_save/interval`: `-1` = 全部帧攒内存、结束时一次性写一个文件（savePCD 路径）；`>0` = 每 N 帧分片写（日志出现 `current scan saved`，会 swap 清空缓冲）
- 文件名: 上游默认 `all_raw_points.pcd`（互相覆盖）；已改为 `save_session_id_` 时间戳（LIVMapper.h 成员 + 构造函数 strftime 生成 `%Y%m%d_%H%M%S`），重编译后每次建图独立文件
- **`dense_map_en`: true=`feats_undistort` 全量点；false=`feats_down_body` 降采样点 → PCD 点数骤减 5~10x！**
- `point_filter_num`: 抽稀倍数（3 = 每 3 点取 1）
- PCD 目录: `~/catkin_ws/src/FAST-LIVO2/Log/pcd/`；轨迹 `lidar_poses.txt`（每帧一行，10Hz 时 行数≈秒数×10）

## 关闭流程（nx_command_server.py，已优化版）
```
KillLiDAR → 异步线程 bg_kill_lidar（界面立即响应）
  → kill_lidar_processes() [互斥锁 _kill_lidar_lock 保护]
    → _kill_tracked_radar_groups(): SIGINT 进程组 → 【只等 fastlivo_mapping 退出(最长60s)=PCD写完标志】 → SIGKILL 进程组
    → pkill -SIGINT（全模式兜底）→ 动态等待(只匹配 fastlivo_mapping) → pkill -9 → rosnode kill → zombie cleanup
```
- 启动路径: 先 **join bg_kill_lidar**（防误杀）→ 若等过则快速 pgrep 验证跳过完整清理 → `_launch_and_track_radar`
- 启动（bg_start_radar）/关闭（bg_kill_lidar）**都异步化**——阻塞 UDP 主循环 = 界面"卡死"

## 关键坑（全部实战踩过）
1. **异步清理线程无"代际"概念**：上一轮 bg_kill_lidar 还在跑时启动新进程，它的 pkill 会**误杀新进程**（实测新进程 13 秒被上一轮清理杀掉，PCD 只有 8 秒数据）→ 启动前必须 join 等上一轮彻底结束
2. **启动路径同步 kill 阻塞 UDP recvfrom 循环** → 所有指令无响应 = 假死 → 必须异步化
3. **二次启动慢的真瓶颈不是 PCD 写盘**（NVMe 上 341MB 写盘 mtime=指令同一秒，几乎瞬间），是关闭线程**串行等待链**（进程组轮询 90s + 动态等待 15s + rosnode 清理）→ 优化：只等 fastlivo_mapping 退出即可，pkill 动态等待也只匹配 fastlivo_mapping
4. **CONFIG_NAME bug**：start_fastlivo2.sh `true` 分支曾误用 `MID360_Fly.yaml` → dense_map_en=false + 抽稀 → PCD 点数骤减（92 秒只有 60MB）。修回 `MID360_Fly_Lidar.yaml`
5. **进程组 vs 独立组**：laserMapping / livox_ros_driver2_node 是 roslaunch 子节点、**有自己的 PGID**（setsid），killpg 只杀 roslaunch 组 → 靠 pkill -9 全模式兜底独立组
6. **pkill -f 自杀**：脚本命令行含 pattern 时 pkill -f 会匹配自身（bash -c '...' 内容含关键字）；**更隐蔽的一种：脚本文件名本身含关键词**（实测一键脚本名 `twin_livo_rviz.sh` 里写 `pkill -f rviz` → -f 匹配整条命令行 → 命中自己的名字 → 刚打印 `[1/7] 清理旧节点 ...` 就 `Terminated` 退出，极难一眼看出）→ 清理进程优先用 **`pkill -x <进程名>`**（精确匹配进程名，rviz/px4/rosmaster/roscore 这类单个可执行名直接用它）；必须 -f 时写成不含连续目标串的正则（`pkill -f 'rv[i]z'`）；**写完一键脚本自查：文件名+内容里出现的每个词，会不会被自己的 pkill 命中**（详见 `airsim-drone-exploration` 的 Windows↔WSL 节）
7. **rviz_full_stream2.py 是常驻推流服务**（不能杀，杀了遥控器没画面）；判断残留时 pgrep 要排除它
8. launch 的 rosparam **会覆盖 yaml 同名参数**——运行时参数以日志 PARAMETERS 段为准，别只看 yaml

## 诊断流程（PCD 异常）
1. `ls -la Log/pcd/` 看大小/时间戳
2. 日志 grep `Raw point cloud data saved` 看 point count
3. 判断"数据没采够 vs 没保存好"：对比 ①日志 feature 行数（约 1500 行≈2.5min）②`lidar_poses.txt` 行数 ③进程存活（ps lstart vs 日志 mtime）
4. 每帧点数：`grep 'Raw feature' | sed 's/.*Raw feature num: //;s/,.*//'` 统计均值（正常 1 万+，飞行模式 2~4 千）
5. **运行时参数 vs yaml**：日志 PARAMETERS 段 grep `/publish/dense_map_en` `/preprocess/point_filter_num` —— 运行时为准
6. 完整性：`scripts/check_pcd.py`（头部 POINTS × 32B + 头长 == 文件大小，差 0 B = 完整）

## 视觉融合配置（img_en / img_topic，2026-08 实战）
- **FAST-LIVO2 的视觉里程计（VIO）由 yaml 控制**：`img_en`（0=纯雷达 1=视觉+雷达融合）、`img_topic`（图像订阅话题）、`cam_model/cam_fx/cam_fy/cam_cx/cam_cy`（相机内参，启动日志 "Found parameter" 可见）
- **飞行配置 `MID360_Fly.yaml` 当前 img_en: 0 + img_topic: /usb_cam/image_raw**——但 D435 实际发布 `/camera/color/image_raw`（~30Hz）→ 话题不匹配 + 视觉关闭 → **VIO 不工作，`/rgb_img` 无数据** → vision_pose 只剩雷达推算。注释写着"视觉+雷达融合 img_en=1"但实际值是 0（提交 7b83f8c 引入，所有分支一致）
- **排查视觉断线**：`rostopic hz /rgb_img`（FAST-LIVO2 重发布的图像，无数据=VIO 断）→ 查 `img_en`/`img_topic` 与相机实际话题
- FAST-LIVO2 发布的 `/mavros/vision_pose/pose`（frame_id=camera_init，LIVMapper.cpp:1399-1405 写死）是给 PX4 的视觉定位源；MAVROS px4_config.yaml `vision_pose.tf.listen: false` 时插件从 topic 订阅
- `MID360_Fly.yaml` vs `MID360_Fly_Lidar.yaml` 的实质差异：img_en（1 vs 0）、max_iterations（3 vs 5）、voxel_size、filter 参数——**飞行模式靠视觉融合，建图模式纯雷达**

## 实机 bag 在 ROS1 工作区回放复现定位（ros1-twin / twin_ws，2026-09-10 跑通）
完整配方 + 实机标定实测值 + 命令：**`references/twin-bag-replay.md`**。要点速记：
- 实机标定在 fouruav 的 `~/catkin_ws/src/FAST-LIVO2/config/`；**launch 实际加载的是 `MID360_Fly_Lidar.yaml`**（`MID360.yaml` 那行被注释掉了），相机是 `camera_self.yaml`
- bag 的 `/livox/lidar` 类型是 `livox_ros_driver2/CustomMsg`，工作区装的是 `livox_ros_driver`（v1）→ **先 `rosmsg md5` 对比**：两者 md5 相同（`e4d6829bdfe657cb6c21a746c86b21a6`），ROS1 按 md5 判定兼容；但 `rosbag play` 仍要能加载 bag 的类型名 → 造一个**最小 msg-only `livox_ros_driver2` 包**（复制 v1 的 .msg）+ `catkin_make --pkg livox_ros_driver2`
- FAST-LIVO2 **即使 `img_en: 0` 也必须把相机配置 load 到节点命名空间**：`rosparam load <camera>.yaml /laserMapping`，否则崩 `Camera model not correctly specified`
- `lidar_type: 1`（Livox Avia）**要求逐点时间** → AirSim 仿真桥的 XYZ-only 点云（`create_cloud_xyz32`）喂不了；**正式链路只能用实机 bag / 实机在线数据**，仿真只能用 `lidar_type: 4` 做预览
- ⚠️ **`lidar_type` 常量 + 各 handler 的字段要求（2026-09-10 读源码确认）**：`include/common_lib.h` 里 `AVIA=1`、`L515=4`（另有 VELO16/OUST64/XT32/PANDAR128/ROBOSENSE）；`preprocess.cpp` 的 `process(sensor_msgs::PointCloud2)` 按 `lidar_type` 分派 handler。**关键：没有任何一个 PointCloud2 handler 能直接吃纯 XYZ-only 点云**——`l515_handler` 用 `pcl::PointCloud<pcl::PointXYZRGB>` + `pcl::fromROSMsg` ⇒ **必须有 `rgb` 字段**；`velodyne/oust64/xt32/robosense/pandar128` 都读 **`intensity`**。纯 xyz 喂进去会报 `Failed to find match for field 'rgb'`（或缺 intensity）。
- **⇒ 用仿真桥（XYZ-only）跑 FAST-LIVO2 必须给点云补字段（已实测修法）**：改 `twin_bridge/scripts/bridge.py` 的 `read_lidar`，把
  `create_cloud_xyz32(Header(...), p)`
  换成显式字段（rgb 全 0）：
  ```python
  fields=[PointField('x',0,PointField.FLOAT32,1), PointField('y',4,PointField.FLOAT32,1),
          PointField('z',8,PointField.FLOAT32,1), PointField('rgb',12,PointField.FLOAT32,1)]
  pts=[[float(q[0]),float(q[1]),float(q[2]),0.0] for q in p]
  self.lidar.publish(point_cloud2.create_cloud(Header(stamp=s,frame_id='twin_lidar_flu'),fields,pts))
  ```
  再把 FAST-LIVO2 配置的 `lidar_type` 设 **4**（走 L515 路径）。**改完必须重启桥**才生效（旧进程仍是 xyz-only）。
- 成功标志：`/aft_mapped_to_init` ≈5Hz、`/cloud_registered` ≈10Hz、日志 `effective feature num` + `average residual ~0.01` + `Update Voxel Map`
- **RViz 看定位/建图：Fixed Frame 必须设 `camera_init`**（FAST-LIVO2 所有输出 `/cloud_registered`/`/aft_mapped_to_init`/`/path` 的 `frame_id` 都是它）。自带 `rviz_cfg/fast_livo2.rviz` 里 Fixed Frame 是**空占位 `<Fixed Frame>`** → **RViz 打开但什么都不显示**（本会话实测症状）。改法：RViz 左上 `Global Options → Fixed Frame` 填 `camera_init`；再加 `Odometry`(topic `/aft_mapped_to_init`) / `Path`(topic `/path`) display。**第二个独立根因（"点云看不到/消失"）＝ `Views` 相机视角，与 Fixed Frame 无关**：自带 `.rviz` 的 `Views.Current` 若是 `Class: rviz/ThirdPersonFollower` + `Distance: 124`，±35m 点云在 124m 外小到看不见；改 `rviz/Orbit` + `Distance:40` + `Focal Point:(0,0,5)`（"上方俯视+鼠标自由动" = Orbit + `Pitch:1.35`≈77°，别用 TopDownOrtho）。**排查顺序先数据后视角**：`rostopic hz /cloud_registered` 出 `no new messages` = 算法链被停（桥/FAST-LIVO2/PX4 被 stop 脚本停了），先 `twin_livo_fast.sh` 补回；有 ~10Hz 数据才去查 Fixed Frame/Views/display。细节见 `airsim-drone-exploration` 技能 ③b。
- **判读：无人机"静止"时坐标在跳 ≈ 正常噪声，不是 bug**。`/aft_mapped_to_init/pose/pose/position` 在原点附近 **±mm 级**微抖（本会话实测 x/y 2~7mm、z ≈ ±0.1mm）是正常估计噪声；**真异常 = 持续朝一个方向漂移（发散）**。IMU 体检：`rostopic echo /twin/imu/data_raw/linear_acceleration` 的 z ≈ ±9.8（重力对）、角速度 ≈ 0。想看坐标"动"来验证跟踪，得让无人机飞（QGC Arm/Takeoff 或 offboard）。
- ⚠️ 起始日志 `START: ...` / `参数` 之后若出现 `Failed to find match for field 'rgb'` = 桥还是旧代码（补 rgb 那步没重启桥）。

## ROS1 工作区启动提速 + 常见坑（twin_ws，2026-09-12 实战）

### roscore 绕开提速（本机 roscore 要 140s！）
- 实测：本机 `roscore` **包装脚本**启动要 **140 秒**，但它内部真正干活的 `rosmaster`（**2s**）和 `rosout`（**0s**）本体都极快 → **绕开 roscore，直接起 rosmaster + rosout**，全量启动 140s → **~4s**：
  ```bash
  setsid rosmaster --core -p 11311 > /tmp/rosmaster.log 2>&1 < /dev/null &
  for i in $(seq 1 30); do timeout 2 rosnode list >/dev/null 2>&1 && break; sleep 1; done
  rosparam set /rosdistro noetic; rosparam set /rosversion 1.17.4
  setsid rosrun rosout rosout __name:=rosout > /tmp/rosout.log 2>&1 < /dev/null &
  ```
- 排查时先证伪：加 `ROS_IP=127.0.0.1` / `ROS_MASTER_URI` 没用（不是 DNS/主机名问题）；`rosmaster` 单独 2s、`rosout` 单独 0s 也快——慢的就是 roscore 这个包装层
- 日常策略：**master 常驻**（关闭脚本默认别杀它），`twin_livo_fast.sh` 只补缺 → 0-2s

### FAST-LIVO2 冷启动要 30~60s，期间刷 `[LIO]: No point!!!` 是正常的（2026-09-12 实测）
- 启动后头几十秒会**持续刷** `[ LIO ]: No point!!!` / `[ VIO ] No point!!!`，同时 `IMU Initializing: 3.3 %` 刚起步——**这是 IMU 初始化 + 首帧阶段，不是点云格式坏、也不是桥没接好**（该行出自 `LIVMapper.cpp` 的 `if (feats_undistort->empty() || feats_undistort == nullptr)` 判断，`handleLIO()` 里的 `return`）。
- **判据（等这个才对）**：日志出现 `[ LIO ] Raw feature num: 9xxx, downsampled ... effective feature num: 9xxx, average residual: 0.003` + `Update Voxel Map`，`/cloud_registered` 才有 ~9-10Hz。实测**手动跑满 45s 即完全正常**（residual 0.0035、`/cloud_registered` 9.43Hz、`/aft_mapped_to_init` 9.98Hz、IMU `linear_acceleration.z ≈ 9.84`）。
- ⚠️ **别用"启动 10~20s 还没数据"判故障**，更别据此去改 `lidar_type` / 给点云补 `rgb`（那是另一个真问题，见上文 §lidar_type 段）。**先等够 60s 再下结论**——本会话就为此多花了一轮排查。
- `IMU Initializing` 停在很低百分比 = 还在收 IMU（正常推进），不是卡死；若 IMU 话题本身没数据（`rostopic hz /twin/imu/data_raw` 无输出）才是真故障。

### roslaunch 报 run_id 不匹配
`RLException: run_id on parameter server does not match declared run_id: <A> vs <B>` = **有残留 roslaunch 占着 master 的 `/run_id`**（之前跑过的 mavros / twin_bridge 等 roslaunch 没退干净）。修法：启动前 `pkill -f roslaunch` 清干净再起 master（放脚本里时注意 pkill 自匹配，见坑 6）。

### AirSim LockStep 时序（twin 链路"桥起了但没数据"）
AirSim 配 `ClockType:SteppableClock + LockStep:true` 时，**必须 PX4 先连上 TCP 4560，仿真才开跑** → 没连上就**一个传感器数据都不出** → 桥"起了但没数据"。判据：`ss -tn | grep ':4560'` 出现 ESTAB；AirSim 日志 `Connected to SITL over TCP`（反之 `Waiting for TCP connection on port 4560` = 还没连上/断开了）。启动顺序：起 PX4 → 等 4560 ESTAB → 再启桥。AirSim 断连后会自动回到 `Waiting` 状态等 PX4 重连。

### RViz 窗口缩小/移动（WSLg）
RViz 窗口是 WSLg 用 `msrdc.exe`（RDP 合成器）渲染的**普通 Windows 窗口，默认全屏铺开**。想缩小/移动：PowerShell + Win32 `SetWindowPos`，`EnumWindows` 按标题找 msrdc 窗口（标题形如 `fast_livo2.rviz* - RViz (ros1-twin)`，**`*` = 有未保存修改**）：
```powershell
SetWindowPos(hwnd, IntPtr.Zero, x, y, w, h, 0x0004)  # 0x0004=SWP_NOZORDER
```
- `.ps1` 必须**纯英文**：中文会在 PowerShell 5.1 读无 BOM UTF-8 时坏字符串（报 `The string is missing the terminator`）
- 窗口标题带 `[WARN:COPY MODE]` = WSLg 退化成软件合成（自定义内核如 JetPack `vmlinux` 缺 GPU 合成支持所致），`wsl --shutdown` 重启后通常恢复、前缀消失

## 回退保障（必须留后路）
- `~/rollback_snapshots/rollback.sh planB|original|current`（快照含 .py/.sh/.cpp/.h，回退后需重编译 C++ + 重启服务）
- 每次改前 `cp 文件 文件.bak.时间戳`

## 验证
- 零残留: `pgrep -af 'fastlivo|livox|laserMapping|roslaunch|imu_tf'`（排除 rviz_full_stream2，输出空白=干净）；`rosnode list` 只剩 full_screen_streamer/nx_qgc_bridge_daemon/rosout
- 二次启动时间: server 日志 `收到 QGC 指令` 时间戳间隔
- 服务重启: 精确 PID `kill -TERM` start_qgc_servers 主进程 → 补杀残留 → `setsid bash start_qgc_servers.sh` 重启（勿用 pkill -f 全模式）
