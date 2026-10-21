---
name: odin-ros-driver-diagnostics
description: 诊断留形科技(Manifold Tech) odin1 手持设备（dToF LiDAR+鱼眼相机+IMU）ROS1 驱动 odin_ros_driver 的数据异常——"算法定位数据一直没有"、/odin1/* 话题 0 数据、节点启动即退出。含节点架构、数据流、防误判事实（配置关闭导致的"正常退出"、odin_to_mavros 缓存位姿死发假 vision_pose、SDK 日志目录空=设备未连接）与 SSH 排查流程。
---

# odin1 ROS 驱动诊断（odin_ros_driver）

## 触发条件
- odin1 手持采集设备（Manifold Tech/留形科技，dToF LiDAR + 鱼眼相机 + IMU）驱动数据异常
- 症状典型表述："算法定位数据一直没有"、话题无数据、节点启动半秒即"cleanly"退出
- 雷达已启动但 RViz 无点云/无图像（fouruav 机载场景）

## 环境速查
- 采集电脑：`<DESKTOP_USER>@<NET_LAB>.196`（Ubuntu 20.04 桌面机，用户 <DESKTOP_USER>，密码 <DESKTOP_PASSWORD>）
- SSH 连接：`plink.exe -ssh -batch -hostkey 'SHA256:<DESKTOP_HOSTKEY_FP>' -pw '<DESKTOP_PASSWORD>' <DESKTOP_USER>@<NET_LAB>.196 '<cmd>'`（plink 引号坑见 windows-ssh 技能）
- 工作区：`/home/<DESKTOP_USER>/jkhk_work/odin_ws`（git 仓库）
- 启动：`roslaunch odin_ros_driver odin1_ros1.launch`；`config/control_command.yaml` 改后必重启驱动
- **fouruav 机载（<NET_WLAN_B>.31）**：工作区 `~/catkin_ws`（odin_ros_driver 在 src/ 下），启动经 NX 投影按钮"启动雷达"（见下"按钮/Profile 机制"）

## 节点架构（odin1_ros1.launch，5 节点）
| 节点 | output | 职责 |
|---|---|---|
| host_sdk_sample | screen | SDK 主节点，发布全部 /odin1/* 原始话题 |
| pcd2depth_node | log | 点云→深度图（演示节点） |
| cloud_reprojection_node | screen | 点云重投影（演示节点） |
| image_overlay_node | screen | 图像叠加 |
| odin_to_mavros | screen | /odin1/odometry_highfreq → /mavros/vision_pose/pose（喂飞控） |

数据流：`设备 → host_sdk_sample → /odin1/odometry_highfreq → odin_to_mavros → /mavros/vision_pose/pose`

## 防误判事实（本类问题最关键的三个知识点）
1. **pcd2depth_node / cloud_reprojection_node 启动即 cleanly 退出 = 正常，不是故障**：
   - `src/pcd2depth_ros.cpp` main：读参数 `register_keys/senddepth`，==0 → 打印 "Depth image will not be published." 并 `return 0`
   - `src/cloud_reprojection_ros.cpp` main：同构，读 `register_keys/sendreprojection`
   - 默认 `control_command.yaml` 两者都是 0。roslaunch 日志里 "process has finished cleanly" 就是它。**别在这俩节点上浪费时间排查**。
2. **odin_to_mavros 发布的是缓存死值**：`src/odin_to_mavros.cpp` 用全局变量 `vins_odom` + `rate(30)` 循环无条件 publish，**从不检查数据新鲜度**。上游断流后 vision_pose 仍 30Hz 恒定（delta 0.033~0.034s、std ~0.0001，timer 级稳定 = 死值特征），内容是最有一条位姿甚至全 0。**绝不能拿 /mavros/vision_pose/pose 的 hz 判断定位是否正常**；飞控 EKF 吃死值会漂移。
3. **SDK 日志目录为空 = 设备没连上**：`log/Driver_<启动时间戳>/` 里 devstatuslog:1 时正常连接会周期写 `dev_status.csv`；空目录 = 连接流程没走到。lsusb 看 Bus 002（USB3.0 总线）只有 root hub 无设备 = 物理层未识别（换 USB3 直插口/换线/查供电）。

## 诊断流程（SSH 直连，一次查完）
1. 节点日志：`ls ~/.ros/log/<run_id>/`——output=log 的节点有 `*-stdout.log`（如 pcd2depth 的 "will not be published" 就在这里）；output=screen 的打印在 roslaunch 终端（用户贴的 roslaunch 主日志里没有 = 没贴全，不是没输出）
2. `rosnode list; rostopic list`：确认节点存活、话题存在（话题在≠有数据）
3. 断流判定：`timeout 12 rostopic hz /odin1/odometry /odin1/odometry_highfreq /odin1/cloud_raw /odin1/imu /odin1/wiwc /odin1/camera_pose_visual`——全 "no new messages" = SDK 断流（注意 odin_to_mavros 订的是 odometry_highfreq 不是 odometry）
4. 设备连接：`ls -la log/Driver_*/` 最新目录 + `lsusb`
5. 分层结论：节点退出→查配置键（非 bug）；/odin1/* 全 0→设备物理连接问题；vision_pose 有 hz→查 odin_to_mavros 源码确认是否死值

## 修复方向
- odin_to_mavros 加新鲜度检查：`vins_odom.header.stamp` 与 ros::Time::now() 差 >0.5s 则不发布并打印警告（改代码前先备份+贴 diff，遵守用户透明改码偏好）
- 设备连接：换 USB3.0 直插口、换线、确认供电；`dmesg | tail -20`（需 sudo）看内核识别

## fouruav 机载场景专属坑（2026-08-13 实测）

### 1. 多个 host_sdk_sample 抢 USB → LIBUSB_ERROR_BUSY → 无点云
- **症状**：话题都在（/odin1/cloud_raw 等）但全部 "no new messages"；RViz 无点云/无图像
- **根因**：按"启动雷达"按钮 + 手动 roslaunch 各起一个 host_sdk_sample，多个进程抢同一 USB 设备 → 后起的 `Failed to claim interface 0: LIBUSB_ERROR_BUSY`
- **判读**：`ps aux | grep host_sdk_sample | grep -v grep | wc -l` >1 = 冲突；日志 strings 提取见 `LIBUSB_ERROR_BUSY`
- **修复**：杀光全部 host_sdk_sample（kill -9 PID），**只启动一次**（按钮或手动二选一，别混用）

### 2. 启动雷达按钮 = Radar_Launcher.yaml 的 active_profile 机制
- **按钮逻辑**：nx_command_server.py（新版，catkin_ws 里）读 `scripts/NX_code/Radar_Launcher.yaml` 的 `active_profile` 决定启动哪种雷达（odin / mid360）——改 `active_profile: "odin"` 即切 odin 模式
- **重启 server 才生效**：改 yaml 后必须 `pkill nx_command_server2.py && nohup python3 -u ...nx_command_server2.py &`，日志应出现 `📡 雷达配置已加载: profile='odin'`
- **注意**：`nx_command_server2.py` 只是包装（`from nx_command_server import start_command_server`），真正逻辑在 `nx_command_server.py`；老版本 `/home/fouruav/NX_code/` 下那份是旧逻辑（Odin/Odin-MindCloud 硬编码 cmd），实际跑的是 catkin_ws 里新版

### 3. RViz 同名节点互相顶掉（odin_ros.rviz 被关）
- **症状**：按按钮3 后 odin_ros.rviz 关闭、只剩 top_down.rviz
- **根因**：odin1_ros1.launch 的 RViz 节点 `name="rviz"` 与 real_flight.launch 的 RViz 节点 `name="rviz"` 同名 → ROS 同名节点后注册顶掉先注册（日志 `Reason given for shutdown: new node registered with same name`）
- **修复**：real_flight.launch 的 RViz 节点改名 `name="rviz_topdown"`（已改），两界面共存

### 4. 驱动日志含 \0 二进制垃圾
- odin_launch.log 可能混入大量 \0 字节（`grep` 报 "匹配到二进制文件"）——用 `strings <log> | grep -iE "claim|busy|error"` 提取文本

### 5. READY→NOT READY（视觉链路多进程打架）——2026-08-13 实测
- **症状**：启动雷达后先是 READY，过一会变 NOT READY；或一直 NOT READY（EKF pos_horiz_rel/velocity_horiz 全 False）
- **根因**：多个 odom_to_mavros 实例同时发 /mavros/vision_pose/pose → 频率飙升（正常 ~10Hz，打架时数千 Hz）→ PX4 EKF 判定视觉数据不可信 → 融合丢失 → NOT READY
- **常见来源**：手动起过 odom_to_mavros + 按钮又自动起；或重复按"启动雷达"
- **判读**：`rostopic hz /mavros/vision_pose/pose` 频率异常（数千 Hz）+ `ps aux | grep -c odom_to_mavros` >1
- **修复**：杀光全部 odom_to_mavros + host_sdk_sample（`kill -9` PID 精确杀，勿用 `pkill -f`——会匹配到 plink 自身命令行导致 SSH 会话被杀 exit 128），**只按一次"启动雷达"**
- **已根治**：启动雷达按钮命令已集成 odom_to_mavros（`sleep 12 && python3 .../odom_to_mavros.py`，等 odin 起来再桥）——**勿再手动起**；Radar_Launcher.yaml odin cleanup 的 pkill_patterns 已加 `host_sdk_sample`/`odom_to_mavros`/`cloud_reprojection_node`/`image_overlay_node`/`pcd2depth_node`，"关闭雷达"能杀干净

### 5b. vision_pose 单实例超频（odin 400Hz 全转发 → 限速 20Hz）— 2026-08-13 实测
- **症状**：odom_to_mavros 只有 1 个（单实例锁已生效）、EKF 一度融合 True，但启动后仍 NOT READY / 反复跳
- **根因**：odin 的 `/odin1/odometry_highfreq` 是 **~400Hz 高频话题**，odom_to_mavros.py 的 `odom_callback` 是**回调直发**（`rospy.Subscriber(...highfreq, self.odom_callback)` + 回调里 publish）→ vision_pose 被全量转发成 **500-1000Hz**（`rostopic hz` 显示 0.000~0.006s）→ **PX4 EKF 认为视觉数据异常 → 拒绝融合 → NOT READY**
- **区分**：多进程打架也是频率飙升，但**单实例 + 超高频 = 超频问题**（不是重复进程）
- **修复（odom_to_mavros.py 回调限速，2026-08-13 已实施+验证）**：`__init__` 加 `self._last_pub = rospy.Time(0)`，回调 publish 前：
  ```python
  now = rospy.Time.now()
  if (now - self._last_pub).to_sec() >= 0.05:   # 20Hz 上限
      self._last_pub = now
      self.vision_pub.publish(pose_map)
  ```
- **验证**：`rostopic hz /mavros/vision_pose/pose` 应 ~10-20Hz（0.049~0.096s），EKF pos_horiz_rel/velocity_horiz 稳定 True。改完重启 odom_to_mavros（kill 旧实例 + `rm /tmp/odom_to_mavros.lock` + 重新 nohup 启动）

### 5c. vision_pose 频率上限与时间戳（2026-08-14 实测问答）
- **PX4 EKF2 处理 vision_pose 上限 ~100Hz**，建议上限 50Hz——超过收益递减 + 噪声相关性风险（同一时刻多观测被当独立观测 → 过度信任 → 漂移）
- **20Hz 对 1m/s 飞行足够**：EKF = IMU 预测(250-500Hz 连续内插) + 视觉修正(低频校准)；20Hz = 每 0.05s 修正一次，1m/s 移动 0.05m，IMU 内插完全覆盖。类比：IMU=秒表(2ms 跳)，视觉=对表(50ms 校一次)——对表频率不影响秒表精度
- **真正影响 1m/s 跟踪的是延迟不是频率**：odom_to_mavros 现在 `pose_map.header.stamp = rospy.Time.now()`（转发时刻当时间戳）→ 引入几十 ms 延迟 → 1m/s 下 50ms≈5cm 位置偏差
- **改原始时间戳（msg.header.stamp）的隐患——先查 `control_command.yaml` 的 `use_host_ros_time`**：
  - `=1`（主机时间）→ 用原始时间戳安全（治延迟）
  - `=0`（设备时钟）→ **保留 now()**（设备时间≠主机时间 → PX4 收到错位时间戳 → EKF 拒绝/错误补偿，比 now() 更差）
  - 其他隐患：驱动启动时时钟校准 → 时间戳前后跳；某些驱动 stamp=0/固定值 → PX4 拒收。查不了就**只改频率，时间戳改动等确认同步后再说**
- 频率调整：限速行 `>= 0.05`(20Hz) → `0.04`(25Hz) / `0.02`(50Hz)

### 6. PX4 ready 判定与参数
- EKF2_EV_CTRL=11（视觉位置+速度融合，PX4 1.17 已废弃 EKF2_AID_MASK）
- **POSCTL/Offboard 就绪条件**：pos_horiz_rel=True + velocity_horiz=True 即可（视觉给的是相对位置；pos_horiz_abs 需 GPS/外部绝对源，**不一定需要**）
- 查参数：`rosservice call /mavros/param/get "param_id: 'EKF2_EV_CTRL'"`
- NOT READY 时看 `/mavros/estimator_status` 哪项 False 定位缺哪个源

### 7. SUPER 导航接 odin（换传感器适配）
- SUPER 只依赖 `cloud_topic` + `odom_topic` 两个输入——换传感器只改传感器层，导航核心（方案C/F 代码）不动
- `task_run_scripts/config/real_flight.yaml` 的 `rog_map/cloud_topic` 从 `/cloud_registered`（FAST-LIVO2）改 `/odin1/cloud_slam`（odin SLAM 配准点云，frame_id="odom"，10Hz）——改完重启 real_flight 生效；odin 的 odometry 自带定位，**不需要 FAST-LIVO2**

## 驱动版本管理（固件适配）

odin 驱动与设备固件**必须版本匹配**，不匹配启动报 `soc version too low` / `unknown cmd code`。仓库 `github.com/manifoldsdk/odin_ros_driver` **无正式 Release，只有 12 个 git tag**。匹配只看源码里的 `required_firmware_version_{major,minor,patch}` 三常量，**不看 tag 名/`ros_driver_version` 字符串**。

**版本匹配表（驱动 tag → 最低固件要求）**：

| 驱动 tag | 最低固件 | 说明 |
|---|---|---|
| v0.13.1 / v0.13.0 | ≥0.13.0 | 配 V0.13.x 固件最优 |
| 0.12.0 | ≥0.12.0 | 配 0.12.x 固件；对 0.13.3 固件「能跑非最优」 |
| v0.10.5_20260525 / v0.10.4 | ≥0.11.0 | — |
| 0.10.3 / v0.10.2 / v0.10.0 | ≥0.10.0 | 配 0.10.x 固件 |

**易踩坑**：
- **tag 名与内部 `ros_driver_version` 字符串可能不同步**——`0.12.0` tag 源码里字符串写的是 `"0.11.0"`（厂商不同步）。启动日志 `ros_driver_version:0.11.0, recommended_min_firmware_version:0.12.0` 是正常的，别被吓到。
- 查设备固件版本：`grep -iE 'soc_version|slam_version|mcu_version' ~/.ros/log/latest/rosout.log`（握手成功后打印 `soc_version: V0.13.3` 等）。注意 `latest` 符号链接可能指向旧目录——用确切 UUID 目录或 `ls -td ~/.ros/log/*/ | head -1` 定位当前实例。

**降级/升级标准流程（单包增量编译，勿用 build_ros.sh）**：
```bash
cd ~/catkin_ws/src/odin_ros_driver
git checkout -- package.xml          # 丢弃上次 cp 的残留
git checkout <tag>                   # 切版本
cp package_ros1.xml package.xml      # 覆盖成 ROS1 包描述
source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && catkin_make --pkg odin_ros_driver -j$(nproc)
```
- **⚠️ 绝不要跑 `./script/build_ros.sh`**：它 `rm -rf` 整个 workspace 的 build/devel/install/log，会把 FAST-LIVO2/livox 等全部包清掉全量重编（Orin NX 上 1 小时+）。`catkin_make --pkg odin_ros_driver` 只重编 odin 一个包（约 56 秒，3 节点 rebuilt）。
- checkout 后父仓库显示 `M src/odin_ros_driver`（gitlink 漂移）= 预期，**别 commit、别跑 `git submodule update`**（会把驱动拉回旧版）。

**数据丢失补救线索（.olx/.bin 私有格式）**：`control_command.yaml` 里 `recorddata: 1` 表示 SDK 实时把 rgb+odometry+slam 点云录成 **olx 私有格式**，直出到 `~/odin_ws/src/odin_ros_driver/recorddata/<开始时间>/`（SDK 注释强调 "please copy the full folder"）。这就是用户丢失 `.olx/.bin` 的来源——下次采集务必整个文件夹拷走；`.olx/.bin` 无公开格式文档，转回需 SDK 导出函数/残留样本逆向/厂商格式文档。

## 话题↔开关映射（含两个命名坑）

`control_command.yaml` 的 `register_keys` 段决定发布哪些话题。**改后必重启驱动**（SDK 启动时一次性读配置）。

| 开关 | 话题名 | 内容 |
|---|---|---|
| sendrgbcompressed=1 | /odin1/image/compressed | JPEG 彩图 |
| sendrgb=1 | /odin1/image | bgr8 解码彩图（pcd2depth 依赖它） |
| senddtof=1 | /odin1/cloud_raw | 雷达点云（49152 点/帧） |
| sendcloudslam=1 | /odin1/cloud_slam | SLAM 配准地图（odom 系 + rgb） |
| sendodom=1 | /odin1/odometry | 位姿 |
| sendimu=1 | /odin1/imu | IMU |
| **senddepth=1** | **/odin1/depth_img_competetion** | 深度图（**注意拼写是 competetion，不是 competition**） |
| senddepth=1 | /odin1/depth_img_competetion_cloud | 深度彩色点云 |
| sendreprojection=1 | /odin1/reprojected_image | 点云重投影到图像 |
| sendoverlay=1 | /odin1/overlay_image | 叠加融合图 |
| sendrgbundistort=1 | /odin1/image/undistorted | 去畸变图（README 写 `image_undistort` 是错的，实际带斜杠） |

- senddepth/sendreprojection/sendoverlay 默认 0；`senddepth` 吃算力（SDK 注释 "high computing resource usage"，Jetson NX 慎开，可只开 sendreprojection）。
- **话题注册 ≠ 有数据**：`initialize_publishers()` 无条件创建所有 publisher，真正决定发帧的是 `m_undistort_map_init_success`（去畸变）等内部成功标志——`no new messages` 时查 `loadCameraParams` 是否静默失败（错误走 std::cerr 不进 rosout）。

## 关联
- vision_pose 喂 PX4 EKF（EKF2_EV_CTRL 相关），与 uav-ros-diagnostics 场景联动；PX4 切 HOLD 事故背景见记忆
- 源码关键片段见 references/odin-source-key-snippets.md
- **odin bag 振荡/数据质量分析**（模仿 odin_bag_report 模板：IMU 质量/振荡检测/频谱/高频振动/odometry/结论 → md+图 → PDF）：完整配方、提取要点、全部坑清单见 references/odin-bag-oscillation-analysis.md。PDF 走 Edge headless（Start-Process 调用，PowerShell `&` 会被终端误判），无需 pandoc
- **双报告对比（交控方 vs 留形方）"同等条件我方无问题"论证模板**：报告 8 节结构、四层科学论证（同一检测器两种定性→量化差异→对方数据自带线索→方法学自洽）、证据链红绿矩阵、黑暗环境专项论点（dToF 主动光源不受光照影响）、措辞基调，见 references/odin-bag-report-comparison.md
- ★ **判读铁律（08-19 用户纠正）**：1s RMS 滑窗只测"运动活跃度"，区分不了手动机动 vs 故障振荡——曾把全程打杆悬停机动误判成"振荡 100%"。**悬停前后左右摇摆+摇杆活跃 = 正常机动**；须用摇杆 manual_control 过零/幅度、位置轨迹往返、姿态-加速度一致性、vision_pose 稳定性验证，报告写"运动活跃段"而非"振荡段"
