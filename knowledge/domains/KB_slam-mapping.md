# 定位与建图（FAST-LIVO2 / odin / PCD 生命周期）

> 领域 `slam-mapping` ｜ 由 3 个原始技能合并而成 ｜ 生成于 2026-10-21

**这一层解决什么**：FAST-LIVO2 是唯一定位源：它一断，飞机立刻失去定点能力。PCD 保存机制与"飞行配置 vs 建图配置"直接绑定飞行安全。

**建议读法**：A PCD 保存与关闭机制（SIGINT 语义）→ B 两种配置模式的选择 → C 关闭/启动流程 → D odin 驱动专项

**包含的原始技能**：`uav-ros-mapping-lifecycle`、`fast-livo2-mapping`、`odin-ros-driver-diagnostics`


---

### 来源技能 `uav-ros-mapping-lifecycle`

> 运维 Jetson NX 机载电脑上的 FAST-LIVO2 建图生命周期——启动/关闭流程、PCD 保存机制（SIGINT 语义、异步关闭、防误杀）、建图/飞行配置模式选择（树林等重负载场景防漂移迫降）、PCD 完整性验证与着色。Use when FAST-LIVO2 的 PCD 丢失/损坏/点数少、关闭或二次启动慢/卡、进程残留、飞行中定位漂移。

#### 系统架构（fouruav NX，Ubuntu 20.04 + ROS Noetic）

- **部署形态（2026-08-21 实测）**：机载 NX_code 已是 **Cython 安全部署版**——核心 `nx_command_server.py` 源码被编译成 `nx_command_server.cpython-38-aarch64-linux-gnu.so`，部署机**无源码、无 .git**（`git status` 报"不是 git 仓库"）。`nx_command_server2.py` 只是 5 行 import .so 的包装器（`*_launcher.py` 同义）。**改逻辑必须回开发机源码仓库**（README：CI `push master` → `build_secure.sh` 编译 → 发布 deploy 分支 → 拷贝回 NX → 重启服务）。部署机上能改的明文面：`Radar_Launcher.yaml`、`User_define_commands.yaml`、`config/*.yaml`、`launch/*.launch`、`start_fastlivo2.sh`。详见 references/secure-deploy-radar-chain.md。
- **雷达启动链路（遥控器/平板 QGC 按钮）**：QGC 按钮 → UDP 6003 JSON → .so 读 `scripts/NX_code/Radar_Launcher.yaml`（`active_profile: odin|mid360`，环境变量 `LIYUMEN_RADAR_PROFILE` 可覆盖）→ 取 `launch_with_cloud`（true=录数据建图）/`launch_without_cloud`（false=仅飞行）的 `command` → 执行 `bash start_fastlivo2.sh true|false` → 按序拉起 6 组节点（rs_d435 相机 → msg_MID360s 雷达驱动 → mid360_imu_tf → rviz 提前弹窗 → `mapping_MID360.launch`（`pcd_save`+`config_name`）→ `px4_and_others.launch`（px4+realman+TF））。关闭按钮走同 yaml 的 `cleanup` 规则（`pkill_patterns`+`rosnode_patterns`）。.so 内有 `global_radar_start_message/result/time/starting` 异步状态机。
- **用户自定义按钮**：`scripts/NX_code/User_define_commands.yaml` 的 UserCmd1/2/3——平板显示 `name`，机载执行 `command`（当前：UserCmd1=SUPER航点任务、UserCmd2=停止SUPER先切POSCTL、UserCmd3=测试占位）。
- **SSH 手动路径**：`task_run_scripts/scripts/NX_code/start_fastlivo2.sh`（带 cleanup trap）。
- **部署版侦察技巧（无源码时）**：`strings nx_command_server.*.so | grep -iE 'Radar_Launcher|UserCmd|6003|LIYUMEN|global_radar|launch_with'` 可挖出配置面/环境变量/端口/状态机字段。
- **配置**：`task_run_scripts/config/MID360_Fly.yaml`（飞行）/ `MID360_Fly_Lidar.yaml`（建图）。launch 用 `config_name` 参数加载。
- **PCD 输出**：`FAST-LIVO2/Log/pcd/`，路径由 CMakeLists 的 `ROOT_DIR` 编译期写死。
- **服务重启（⚠️ 旧写法已废弃）**：~~`pkill -f start_qgc_servers.sh` + 重新 `bash start_qgc_servers.sh`~~ ——2026-08-21 实测不可靠：脚本进程可能早已死（服务是 nohup 孤儿挂 systemd 名下），且 GNOME autostart 会自动复活服务。正确流程见下"服务进程排障"节（先 mv autostart 为 .off 再按精确进程名杀）。

#### 核心机制：PCD 保存的 SIGINT 语义

- **roscpp 只响应 SIGINT（Ctrl+C），不响应 SIGTERM**——`kill -TERM` 会让 fastlivo_mapping 直接暴毙，`savePCD()` 根本没机会执行。
- `savePCD()` 只在 `while(ros::ok())` 主循环退出后执行一次（LIVMapper.cpp 的 run() 末尾），且条件是 `pcd_save_en && interval < 0`。
- **正确关闭信号链**：SIGINT → ros::ok() 变 false → 主循环退出 → savePCD 写盘 → 进程退出。
- **PCD 写盘很快**（NVMe 上 341MB 几乎瞬间），"关闭慢"的瓶颈通常在**清理等待链**（等整个进程组消失、等 livox 优雅退出），不是写盘本身——诊断时先看日志 mtime vs 指令时刻，别误判成写盘慢。

#### 关闭/启动流程设计（nx_command_server.py，2026-08 优化后的状态）

1. **KillLiDAR 异步化**：丢进 `bg_kill_lidar` daemon 线程，UDP 主循环立即返回，界面不卡。
2. **启动也异步化**：`bg_start_radar` 线程，主循环永不阻塞（同步 join 会卡死 UDP 循环 → 界面假死）。
3. **防误杀**：启动前 `join(bg_kill_lidar)` 等上一轮关闭线程彻底结束——否则上一轮清理的 `pkill` 会匹配到刚启动的新进程，把它 13 秒内杀掉（PCD 只有 8 秒数据）。
4. **互斥锁**：`_kill_lidar_lock` 保证同一时间只有一个清理流程（装饰器包装 `_kill_lidar_processes_impl`）。
5. **只等关键进程**：关闭时只轮询 `pgrep -f fastlivo_mapping` 退出（= PCD 写完标志，最长 60s），**不等整个进程组**（rviz/livox 不响应 SIGINT 会拖到 90s）→ 二次启动从 20~30s 降到 3~8s。
6. **双保险**：killpg SIGINT 后再 `pkill -INT -f fastlivo_mapping`（防 roslaunch 没转发信号 → PCD 必保存）。
7. **零残留三重保险**：SIGKILL 进程组 + pkill -9 全模式兜底 + rosnode cleanup / 启动前 pgrep 验证。

#### 配置模式（飞行安全关键！）

| 参数 | MID360_Fly.yaml（飞行） | MID360_Fly_Lidar.yaml（建图） |
|---|---|---|
| dense_map_en | false（降采样点） | true（全量点云） |
| point_filter_num | 3（抽稀 3 倍） | 1（不抽稀） |
| lio.max_iterations | 3 | 5 |
| lio.voxel_size | 1.0 | 0.5 |
| 算力 | 低（保实时） | ≈2 倍（精度优先） |

- **树林等点云密集场景用 Fly_Lidar 会算力吃满 → 建图延迟高 → PX4 定点模式漂移 → 迫降**。飞行必须用 Fly.yaml；完整地图单独跑建图任务（不飞时慢扫）。
- `start_fastlivo2.sh` 的 `true` 分支决定默认配置；`$2` 参数（lidar/fly）可覆盖。**历史坑**：git 里曾把 `true` 分支误写成 `MID360_Fly.yaml`（注释写"高质量建图"但值错）→ PCD 点数骤减（2 分钟只有 60MB，dense_map_en=false + 抽稀 3 倍）。改配置后对照运行时 `/publish/dense_map_en` 与配置文件是否一致。

#### PCD 完整性验证

```bash
python3 task_run_scripts/scripts/check_pcd.py
```
原理：PCD 头部 `POINTS=N` × 每点字节数 + header 长度 = 文件实际大小，差 0 B = 完整。字段 `x y z intensity normal_x normal_y normal_z curvature` = 32B/点；`x y z rgb` = 16B/点。

#### PCD 着色（CloudCompare 打开全是蓝色）

- 原因：FAST-LIVO2 输出的 PCD 字段只有 `x y z intensity normal_* curvature`，**没有 RGB** → CloudCompare 默认蓝色。
- 解法：`python3 task_run_scripts/scripts/colorize_pcd.py <in.pcd>` → 生成 `<in>_color.pcd`（XYZI→XYZRGB，按高度 Z jet 色图）。转成 XYZI→XYZRGB 结构体：`struct.pack('<fffI', x, y, z, rgb)`，rgb = (r<<16)|(g<<8)|b。
- 或 CloudCompare 内即时着色：`Scalar field → intensity/z → Set color from scalar field`（不生成新文件）。

#### 回退保障

`~/rollback_snapshots/rollback.sh planB|original|current` — 一键恢复到方案B / 最原始版 / 当前版。快照含 nx_command_server.py、start_fastlivo2.sh、LIVMapper.cpp/h。回退到 planB/original 需重编译（`catkin_make -DCATKIN_WHITELIST_PACKAGES=fast_livo -j2`）+ 重启服务。

#### 诊断要点（PCD 数据量异常的区分）

- **PCD 时间短**（如 2 分钟飞行只有 8 秒数据）→ 进程被提前杀（异步清理误杀 / 手动强杀）→ 检查进程实际存活时长（日志 mtime vs 启动时刻）。
- **PCD 时间长但文件小**（92 秒只有 60MB）→ 配置错用飞行模式（每帧点数 2-4 千 vs 正常 1 万+）→ 检查 dense_map_en / point_filter_num 运行时值。
- **文件损坏**（头声明点数对不上）→ 写入被 SIGKILL 打断 → 检查关闭是否用了 SIGINT + 等待。

#### 服务进程排障（pkill 不起效 ≠ pkill 坏了）

- **症状**：`pkill -f start_qgc_servers.sh` 无任何效果，服务（roscore/三微服务）还在跑。
- **根因（2026-08-21 实测）**：脚本进程**早已不存在**（可能被前一次 pkill/SIGKILL 干掉），而 roscore/微服务是 nohup 子进程，死掉后被 reparent 到 `systemd --user`（`ps -o pid,ppid,cmd` 看 PPID 是 systemd 而非脚本 bash）→ 变成孤儿继续跑。此时 pkill 脚本名无目标，返回码 1，天然"不起效"。
- **判据三连**：① `pgrep -af start_qgc_servers` 只匹配到执行命令的 shell 自己（pkill -f 按完整命令行匹配，**会匹配到正在执行它的 bash -c**——ssh 里跑会自杀掉线）；② 微服务 PPID 是 systemd --user；③ `log/latest/start_qgc_servers.log` 无 "stopping all modules..."（trap 从未执行）。
- **正确停服务**：按**精确进程名**杀微服务，别杀脚本名：
  ```bash
  pkill -f nx_command_server2.py; pkill -f nx_touch_server2.py; pkill -f rviz_full_stream2.py; pkill -f roscore
  ```
  然后 `bash ~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_qgc_servers.sh` 重启（预期：新 `log/<时间戳>/` 出现，start_qgc_servers.log 打印 4 个 pid）。
- **教训**：`pkill -f` 的匹配目标是"当前存在的进程"，先 `pgrep -af` 确认目标存在再杀；pkill 模式用精确进程名（如 `nx_command_server2.py`），不用脚本名。
- **GNOME autostart 自动复活机制（治本关键，2026-08-21 实测）**：`~/.config/autostart/qgc_servers.desktop` 里的应用被 GNOME 会话监控，**进程退出后会自动重新拉起**——表现：杀了服务几秒后新进程 + 新日志目录又出现（实测 16:11:39 自动拉起第二套），遥控器画面不断，误以为 pkill 没作用。**要彻底停服务必须先移走自启入口再杀**：
  ```bash
  mv ~/.config/autostart/qgc_servers.desktop ~/.config/autostart/qgc_servers.desktop.off
  pkill -9 -f 'rviz_full_stream[2]\.py'; pkill -9 -f 'nx_command_server[2]\.py'; pkill -9 -f 'nx_touch_server[2]\.py'; pkill -9 -f 'roscor[e]'
  ```
  恢复：`mv ~/.config/autostart/qgc_servers.desktop.off ~/.config/autostart/qgc_servers.desktop` + `bash .../start_qgc_servers.sh`。
- **pkill -f 自杀坑增强版（远程 plink/bash -c 里必踩）**：`pkill -f xxx` 按完整命令行匹配，**执行命令的 shell（bash -c 整串）里任何位置出现目标子串都会自杀**——包括 grep -E 显示模式、echo 文本、其他命令参数。方括号技巧 `'roscor[e]'` 只能防"pkill 模式自身"，**防不住同命令串里的 grep 模式**（实测：`pkill -f 'roscor[e]'` 因同串 `grep -E '...roscore...'` 含 roscore 而自杀断连，exit 128）。安全姿势：① 命令整串（含 grep 显示模式）都避开目标串；② 或把 pkill 写进 /tmp 脚本再 `bash /tmp/kill.sh` 执行（脚本进程命令行是脚本路径，不匹配任何模式）。


---

### 来源技能 `fast-livo2-mapping`

> 运维 FAST-LIVO2 建图（Jetson NX + MID360）：PCD 保存/关闭/启动流程、异步清理线程防误杀、二次启动提速、PCD 点数骤减排查、零残留保证。Use when 建图 PCD 异常（缺失/变小/损坏）、关闭后进程残留、启动/关闭慢或卡死、需要改 start_fastlivo2.sh / nx_command_server.py 的关闭逻辑；或在 ROS1 工作区（ros1-twin/twin_ws）用实机 bag 回放复现 FAST-LIVO2 定位（livox_ros_driver2↔v1 消息 md5 一致、cam_model 必填、实机标定抄录、RViz Fixed Frame=camera_init、静止时 mm 级坐标抖动属正常、roscore 绕开提速、run_id 冲突、AirSim LockStep 时序、RViz 窗口缩小）。

#### 架构地图（fouruav NX）

- 启动入口: `scripts/NX_code/start_fastlivo2.sh true|false` — true=建图用 `MID360_Fly_Lidar.yaml`，false=飞行用 `MID360_Fly.yaml`（按 `$WITH_CLOUD` 选 CONFIG_NAME，`$2` 可覆盖 lidar/fly）；**脚本全路径 `~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_fastlivo2.sh`，不在 home 目录**
- 启动链路（2026-08-10 确认）：rs_d435_camera.launch → msg_MID360s.launch（Livox MID-360S 驱动）→ mid360_imu_tf.launch → rviz → mapping_MID360.launch → px4_and_others.launch（px4.launch=ttyTHS0:921600 + static TF map→odom/map→camera_init + realman）
- **FAST-LIVO2 发布话题（LIVMapper.cpp）**：`/cloud_registered`（PointCloud2 注册点云 ~10Hz）、`/aft_mapped_to_init`（Odometry 里程计高频）、`/path`、`/Laser_map`、`/dyn_obj*`、`/LIVO2/imu_propagate`——下游消费者（SUPER 规划器等）订阅这俩，不是 /livox/lidar
- launch: `task_run_scripts/launch/mapping_MID360.launch` 加载 `config/$(config_name).yaml` + `camera_self.yaml`（rosparam 会覆盖 yaml 里的同名参数！）
- 指令服务: `nx_command_server2.py`（317B 包装）→ `nx_command_server.py`（~98KB 实际逻辑）；UDP 6003 收 QGC 指令（Odin-MindCloud=启动建图 / KillLiDAR=关闭），HTTP 6004
- 服务启停: `start_qgc_servers.sh`（roscore + nx_touch_server2 + nx_command_server2 + rviz_full_stream2 四服务，SIGTERM 触发 cleanup）

#### PCD 保存机制（LIVMapper.cpp）

- `savePCD()` **只在 `run()` 主循环 `while(ros::ok())` 退出后调用一次**（LIVMapper.cpp:553）
- `ros::ok()` 只被 **SIGINT（Ctrl+C）** 置 false；**SIGTERM 默认杀死进程 → savePCD 不执行 → PCD 丢**。这是"PCD 丢失/损坏"的根源
- `pcd_save/interval`: `-1` = 全部帧攒内存、结束时一次性写一个文件（savePCD 路径）；`>0` = 每 N 帧分片写（日志出现 `current scan saved`，会 swap 清空缓冲）
- 文件名: 上游默认 `all_raw_points.pcd`（互相覆盖）；已改为 `save_session_id_` 时间戳（LIVMapper.h 成员 + 构造函数 strftime 生成 `%Y%m%d_%H%M%S`），重编译后每次建图独立文件
- **`dense_map_en`: true=`feats_undistort` 全量点；false=`feats_down_body` 降采样点 → PCD 点数骤减 5~10x！**
- `point_filter_num`: 抽稀倍数（3 = 每 3 点取 1）
- PCD 目录: `~/catkin_ws/src/FAST-LIVO2/Log/pcd/`；轨迹 `lidar_poses.txt`（每帧一行，10Hz 时 行数≈秒数×10）

#### 关闭流程（nx_command_server.py，已优化版）

```
KillLiDAR → 异步线程 bg_kill_lidar（界面立即响应）
  → kill_lidar_processes() [互斥锁 _kill_lidar_lock 保护]
    → _kill_tracked_radar_groups(): SIGINT 进程组 → 【只等 fastlivo_mapping 退出(最长60s)=PCD写完标志】 → SIGKILL 进程组
    → pkill -SIGINT（全模式兜底）→ 动态等待(只匹配 fastlivo_mapping) → pkill -9 → rosnode kill → zombie cleanup
```
- 启动路径: 先 **join bg_kill_lidar**（防误杀）→ 若等过则快速 pgrep 验证跳过完整清理 → `_launch_and_track_radar`
- 启动（bg_start_radar）/关闭（bg_kill_lidar）**都异步化**——阻塞 UDP 主循环 = 界面"卡死"

#### 关键坑（全部实战踩过）

1. **异步清理线程无"代际"概念**：上一轮 bg_kill_lidar 还在跑时启动新进程，它的 pkill 会**误杀新进程**（实测新进程 13 秒被上一轮清理杀掉，PCD 只有 8 秒数据）→ 启动前必须 join 等上一轮彻底结束
2. **启动路径同步 kill 阻塞 UDP recvfrom 循环** → 所有指令无响应 = 假死 → 必须异步化
3. **二次启动慢的真瓶颈不是 PCD 写盘**（NVMe 上 341MB 写盘 mtime=指令同一秒，几乎瞬间），是关闭线程**串行等待链**（进程组轮询 90s + 动态等待 15s + rosnode 清理）→ 优化：只等 fastlivo_mapping 退出即可，pkill 动态等待也只匹配 fastlivo_mapping
4. **CONFIG_NAME bug**：start_fastlivo2.sh `true` 分支曾误用 `MID360_Fly.yaml` → dense_map_en=false + 抽稀 → PCD 点数骤减（92 秒只有 60MB）。修回 `MID360_Fly_Lidar.yaml`
5. **进程组 vs 独立组**：laserMapping / livox_ros_driver2_node 是 roslaunch 子节点、**有自己的 PGID**（setsid），killpg 只杀 roslaunch 组 → 靠 pkill -9 全模式兜底独立组
6. **pkill -f 自杀**：脚本命令行含 pattern 时 pkill -f 会匹配自身（bash -c '...' 内容含关键字）；**更隐蔽的一种：脚本文件名本身含关键词**（实测一键脚本名 `twin_livo_rviz.sh` 里写 `pkill -f rviz` → -f 匹配整条命令行 → 命中自己的名字 → 刚打印 `[1/7] 清理旧节点 ...` 就 `Terminated` 退出，极难一眼看出）→ 清理进程优先用 **`pkill -x <进程名>`**（精确匹配进程名，rviz/px4/rosmaster/roscore 这类单个可执行名直接用它）；必须 -f 时写成不含连续目标串的正则（`pkill -f 'rv[i]z'`）；**写完一键脚本自查：文件名+内容里出现的每个词，会不会被自己的 pkill 命中**（详见 `airsim-drone-exploration` 的 Windows↔WSL 节）
7. **rviz_full_stream2.py 是常驻推流服务**（不能杀，杀了遥控器没画面）；判断残留时 pgrep 要排除它
8. launch 的 rosparam **会覆盖 yaml 同名参数**——运行时参数以日志 PARAMETERS 段为准，别只看 yaml

#### 诊断流程（PCD 异常）

1. `ls -la Log/pcd/` 看大小/时间戳
2. 日志 grep `Raw point cloud data saved` 看 point count
3. 判断"数据没采够 vs 没保存好"：对比 ①日志 feature 行数（约 1500 行≈2.5min）②`lidar_poses.txt` 行数 ③进程存活（ps lstart vs 日志 mtime）
4. 每帧点数：`grep 'Raw feature' | sed 's/.*Raw feature num: //;s/,.*//'` 统计均值（正常 1 万+，飞行模式 2~4 千）
5. **运行时参数 vs yaml**：日志 PARAMETERS 段 grep `/publish/dense_map_en` `/preprocess/point_filter_num` —— 运行时为准
6. 完整性：`scripts/check_pcd.py`（头部 POINTS × 32B + 头长 == 文件大小，差 0 B = 完整）

#### 视觉融合配置（img_en / img_topic，2026-08 实战）

- **FAST-LIVO2 的视觉里程计（VIO）由 yaml 控制**：`img_en`（0=纯雷达 1=视觉+雷达融合）、`img_topic`（图像订阅话题）、`cam_model/cam_fx/cam_fy/cam_cx/cam_cy`（相机内参，启动日志 "Found parameter" 可见）
- **飞行配置 `MID360_Fly.yaml` 当前 img_en: 0 + img_topic: /usb_cam/image_raw**——但 D435 实际发布 `/camera/color/image_raw`（~30Hz）→ 话题不匹配 + 视觉关闭 → **VIO 不工作，`/rgb_img` 无数据** → vision_pose 只剩雷达推算。注释写着"视觉+雷达融合 img_en=1"但实际值是 0（提交 7b83f8c 引入，所有分支一致）
- **排查视觉断线**：`rostopic hz /rgb_img`（FAST-LIVO2 重发布的图像，无数据=VIO 断）→ 查 `img_en`/`img_topic` 与相机实际话题
- FAST-LIVO2 发布的 `/mavros/vision_pose/pose`（frame_id=camera_init，LIVMapper.cpp:1399-1405 写死）是给 PX4 的视觉定位源；MAVROS px4_config.yaml `vision_pose.tf.listen: false` 时插件从 topic 订阅
- `MID360_Fly.yaml` vs `MID360_Fly_Lidar.yaml` 的实质差异：img_en（1 vs 0）、max_iterations（3 vs 5）、voxel_size、filter 参数——**飞行模式靠视觉融合，建图模式纯雷达**

#### 实机 bag 在 ROS1 工作区回放复现定位（ros1-twin / twin_ws，2026-09-10 跑通）

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

**ROS1 工作区启动提速 + 常见坑（twin_ws，2026-09-12 实战）**

#### roscore 绕开提速（本机 roscore 要 140s！）

- 实测：本机 `roscore` **包装脚本**启动要 **140 秒**，但它内部真正干活的 `rosmaster`（**2s**）和 `rosout`（**0s**）本体都极快 → **绕开 roscore，直接起 rosmaster + rosout**，全量启动 140s → **~4s**：
  ```bash
  setsid rosmaster --core -p 11311 > /tmp/rosmaster.log 2>&1 < /dev/null &
  for i in $(seq 1 30); do timeout 2 rosnode list >/dev/null 2>&1 && break; sleep 1; done
  rosparam set /rosdistro noetic; rosparam set /rosversion 1.17.4
  setsid rosrun rosout rosout __name:=rosout > /tmp/rosout.log 2>&1 < /dev/null &
  ```
- 排查时先证伪：加 `ROS_IP=127.0.0.1` / `ROS_MASTER_URI` 没用（不是 DNS/主机名问题）；`rosmaster` 单独 2s、`rosout` 单独 0s 也快——慢的就是 roscore 这个包装层
- 日常策略：**master 常驻**（关闭脚本默认别杀它），`twin_livo_fast.sh` 只补缺 → 0-2s

#### FAST-LIVO2 冷启动要 30~60s，期间刷 `[LIO]: No point!!!` 是正常的（2026-09-12 实测）

- 启动后头几十秒会**持续刷** `[ LIO ]: No point!!!` / `[ VIO ] No point!!!`，同时 `IMU Initializing: 3.3 %` 刚起步——**这是 IMU 初始化 + 首帧阶段，不是点云格式坏、也不是桥没接好**（该行出自 `LIVMapper.cpp` 的 `if (feats_undistort->empty() || feats_undistort == nullptr)` 判断，`handleLIO()` 里的 `return`）。
- **判据（等这个才对）**：日志出现 `[ LIO ] Raw feature num: 9xxx, downsampled ... effective feature num: 9xxx, average residual: 0.003` + `Update Voxel Map`，`/cloud_registered` 才有 ~9-10Hz。实测**手动跑满 45s 即完全正常**（residual 0.0035、`/cloud_registered` 9.43Hz、`/aft_mapped_to_init` 9.98Hz、IMU `linear_acceleration.z ≈ 9.84`）。
- ⚠️ **别用"启动 10~20s 还没数据"判故障**，更别据此去改 `lidar_type` / 给点云补 `rgb`（那是另一个真问题，见上文 §lidar_type 段）。**先等够 60s 再下结论**——本会话就为此多花了一轮排查。
- `IMU Initializing` 停在很低百分比 = 还在收 IMU（正常推进），不是卡死；若 IMU 话题本身没数据（`rostopic hz /twin/imu/data_raw` 无输出）才是真故障。

#### roslaunch 报 run_id 不匹配

`RLException: run_id on parameter server does not match declared run_id: <A> vs <B>` = **有残留 roslaunch 占着 master 的 `/run_id`**（之前跑过的 mavros / twin_bridge 等 roslaunch 没退干净）。修法：启动前 `pkill -f roslaunch` 清干净再起 master（放脚本里时注意 pkill 自匹配，见坑 6）。

#### AirSim LockStep 时序（twin 链路"桥起了但没数据"）

AirSim 配 `ClockType:SteppableClock + LockStep:true` 时，**必须 PX4 先连上 TCP 4560，仿真才开跑** → 没连上就**一个传感器数据都不出** → 桥"起了但没数据"。判据：`ss -tn | grep ':4560'` 出现 ESTAB；AirSim 日志 `Connected to SITL over TCP`（反之 `Waiting for TCP connection on port 4560` = 还没连上/断开了）。启动顺序：起 PX4 → 等 4560 ESTAB → 再启桥。AirSim 断连后会自动回到 `Waiting` 状态等 PX4 重连。

#### RViz 窗口缩小/移动（WSLg）

RViz 窗口是 WSLg 用 `msrdc.exe`（RDP 合成器）渲染的**普通 Windows 窗口，默认全屏铺开**。想缩小/移动：PowerShell + Win32 `SetWindowPos`，`EnumWindows` 按标题找 msrdc 窗口（标题形如 `fast_livo2.rviz* - RViz (ros1-twin)`，**`*` = 有未保存修改**）：
```powershell
SetWindowPos(hwnd, IntPtr.Zero, x, y, w, h, 0x0004)  # 0x0004=SWP_NOZORDER
```
- `.ps1` 必须**纯英文**：中文会在 PowerShell 5.1 读无 BOM UTF-8 时坏字符串（报 `The string is missing the terminator`）
- 窗口标题带 `[WARN:COPY MODE]` = WSLg 退化成软件合成（自定义内核如 JetPack `vmlinux` 缺 GPU 合成支持所致），`wsl --shutdown` 重启后通常恢复、前缀消失

#### 回退保障（必须留后路）

- `~/rollback_snapshots/rollback.sh planB|original|current`（快照含 .py/.sh/.cpp/.h，回退后需重编译 C++ + 重启服务）
- 每次改前 `cp 文件 文件.bak.时间戳`

#### 验证

- 零残留: `pgrep -af 'fastlivo|livox|laserMapping|roslaunch|imu_tf'`（排除 rviz_full_stream2，输出空白=干净）；`rosnode list` 只剩 full_screen_streamer/nx_qgc_bridge_daemon/rosout
- 二次启动时间: server 日志 `收到 QGC 指令` 时间戳间隔
- 服务重启: 精确 PID `kill -TERM` start_qgc_servers 主进程 → 补杀残留 → `setsid bash start_qgc_servers.sh` 重启（勿用 pkill -f 全模式）


---

### 来源技能 `odin-ros-driver-diagnostics`

> 诊断留形科技(Manifold Tech) odin1 手持设备（dToF LiDAR+鱼眼相机+IMU）ROS1 驱动 odin_ros_driver 的数据异常——"算法定位数据一直没有"、/odin1/* 话题 0 数据、节点启动即退出。含节点架构、数据流、防误判事实（配置关闭导致的"正常退出"、odin_to_mavros 缓存位姿死发假 vision_pose、SDK 日志目录空=设备未连接）与 SSH 排查流程。

#### 环境速查

- 采集电脑：`<DESKTOP_USER>@<NET_LAB>.196`（Ubuntu 20.04 桌面机，用户 <DESKTOP_USER>，密码 <DESKTOP_PASSWORD>）
- SSH 连接：`plink.exe -ssh -batch -hostkey 'SHA256:<DESKTOP_HOSTKEY_FP>' -pw '<DESKTOP_PASSWORD>' <DESKTOP_USER>@<NET_LAB>.196 '<cmd>'`（plink 引号坑见 windows-ssh 技能）
- 工作区：`/home/<DESKTOP_USER>/jkhk_work/odin_ws`（git 仓库）
- 启动：`roslaunch odin_ros_driver odin1_ros1.launch`；`config/control_command.yaml` 改后必重启驱动
- **fouruav 机载（<NET_WLAN_B>.31）**：工作区 `~/catkin_ws`（odin_ros_driver 在 src/ 下），启动经 NX 投影按钮"启动雷达"（见下"按钮/Profile 机制"）

#### 节点架构（odin1_ros1.launch，5 节点）

| 节点 | output | 职责 |
|---|---|---|
| host_sdk_sample | screen | SDK 主节点，发布全部 /odin1/* 原始话题 |
| pcd2depth_node | log | 点云→深度图（演示节点） |
| cloud_reprojection_node | screen | 点云重投影（演示节点） |
| image_overlay_node | screen | 图像叠加 |
| odin_to_mavros | screen | /odin1/odometry_highfreq → /mavros/vision_pose/pose（喂飞控） |

数据流：`设备 → host_sdk_sample → /odin1/odometry_highfreq → odin_to_mavros → /mavros/vision_pose/pose`

#### 防误判事实（本类问题最关键的三个知识点）

1. **pcd2depth_node / cloud_reprojection_node 启动即 cleanly 退出 = 正常，不是故障**：
   - `src/pcd2depth_ros.cpp` main：读参数 `register_keys/senddepth`，==0 → 打印 "Depth image will not be published." 并 `return 0`
   - `src/cloud_reprojection_ros.cpp` main：同构，读 `register_keys/sendreprojection`
   - 默认 `control_command.yaml` 两者都是 0。roslaunch 日志里 "process has finished cleanly" 就是它。**别在这俩节点上浪费时间排查**。
2. **odin_to_mavros 发布的是缓存死值**：`src/odin_to_mavros.cpp` 用全局变量 `vins_odom` + `rate(30)` 循环无条件 publish，**从不检查数据新鲜度**。上游断流后 vision_pose 仍 30Hz 恒定（delta 0.033~0.034s、std ~0.0001，timer 级稳定 = 死值特征），内容是最有一条位姿甚至全 0。**绝不能拿 /mavros/vision_pose/pose 的 hz 判断定位是否正常**；飞控 EKF 吃死值会漂移。
3. **SDK 日志目录为空 = 设备没连上**：`log/Driver_<启动时间戳>/` 里 devstatuslog:1 时正常连接会周期写 `dev_status.csv`；空目录 = 连接流程没走到。lsusb 看 Bus 002（USB3.0 总线）只有 root hub 无设备 = 物理层未识别（换 USB3 直插口/换线/查供电）。

#### 诊断流程（SSH 直连，一次查完）

1. 节点日志：`ls ~/.ros/log/<run_id>/`——output=log 的节点有 `*-stdout.log`（如 pcd2depth 的 "will not be published" 就在这里）；output=screen 的打印在 roslaunch 终端（用户贴的 roslaunch 主日志里没有 = 没贴全，不是没输出）
2. `rosnode list; rostopic list`：确认节点存活、话题存在（话题在≠有数据）
3. 断流判定：`timeout 12 rostopic hz /odin1/odometry /odin1/odometry_highfreq /odin1/cloud_raw /odin1/imu /odin1/wiwc /odin1/camera_pose_visual`——全 "no new messages" = SDK 断流（注意 odin_to_mavros 订的是 odometry_highfreq 不是 odometry）
4. 设备连接：`ls -la log/Driver_*/` 最新目录 + `lsusb`
5. 分层结论：节点退出→查配置键（非 bug）；/odin1/* 全 0→设备物理连接问题；vision_pose 有 hz→查 odin_to_mavros 源码确认是否死值

#### 修复方向

- odin_to_mavros 加新鲜度检查：`vins_odom.header.stamp` 与 ros::Time::now() 差 >0.5s 则不发布并打印警告（改代码前先备份+贴 diff，遵守用户透明改码偏好）
- 设备连接：换 USB3.0 直插口、换线、确认供电；`dmesg | tail -20`（需 sudo）看内核识别

**fouruav 机载场景专属坑（2026-08-13 实测）**

#### 1. 多个 host_sdk_sample 抢 USB → LIBUSB_ERROR_BUSY → 无点云

- **症状**：话题都在（/odin1/cloud_raw 等）但全部 "no new messages"；RViz 无点云/无图像
- **根因**：按"启动雷达"按钮 + 手动 roslaunch 各起一个 host_sdk_sample，多个进程抢同一 USB 设备 → 后起的 `Failed to claim interface 0: LIBUSB_ERROR_BUSY`
- **判读**：`ps aux | grep host_sdk_sample | grep -v grep | wc -l` >1 = 冲突；日志 strings 提取见 `LIBUSB_ERROR_BUSY`
- **修复**：杀光全部 host_sdk_sample（kill -9 PID），**只启动一次**（按钮或手动二选一，别混用）

#### 2. 启动雷达按钮 = Radar_Launcher.yaml 的 active_profile 机制

- **按钮逻辑**：nx_command_server.py（新版，catkin_ws 里）读 `scripts/NX_code/Radar_Launcher.yaml` 的 `active_profile` 决定启动哪种雷达（odin / mid360）——改 `active_profile: "odin"` 即切 odin 模式
- **重启 server 才生效**：改 yaml 后必须 `pkill nx_command_server2.py && nohup python3 -u ...nx_command_server2.py &`，日志应出现 `📡 雷达配置已加载: profile='odin'`
- **注意**：`nx_command_server2.py` 只是包装（`from nx_command_server import start_command_server`），真正逻辑在 `nx_command_server.py`；老版本 `/home/fouruav/NX_code/` 下那份是旧逻辑（Odin/Odin-MindCloud 硬编码 cmd），实际跑的是 catkin_ws 里新版

#### 3. RViz 同名节点互相顶掉（odin_ros.rviz 被关）

- **症状**：按按钮3 后 odin_ros.rviz 关闭、只剩 top_down.rviz
- **根因**：odin1_ros1.launch 的 RViz 节点 `name="rviz"` 与 real_flight.launch 的 RViz 节点 `name="rviz"` 同名 → ROS 同名节点后注册顶掉先注册（日志 `Reason given for shutdown: new node registered with same name`）
- **修复**：real_flight.launch 的 RViz 节点改名 `name="rviz_topdown"`（已改），两界面共存

#### 4. 驱动日志含 \0 二进制垃圾

- odin_launch.log 可能混入大量 \0 字节（`grep` 报 "匹配到二进制文件"）——用 `strings <log> | grep -iE "claim|busy|error"` 提取文本

#### 5. READY→NOT READY（视觉链路多进程打架）——2026-08-13 实测

- **症状**：启动雷达后先是 READY，过一会变 NOT READY；或一直 NOT READY（EKF pos_horiz_rel/velocity_horiz 全 False）
- **根因**：多个 odom_to_mavros 实例同时发 /mavros/vision_pose/pose → 频率飙升（正常 ~10Hz，打架时数千 Hz）→ PX4 EKF 判定视觉数据不可信 → 融合丢失 → NOT READY
- **常见来源**：手动起过 odom_to_mavros + 按钮又自动起；或重复按"启动雷达"
- **判读**：`rostopic hz /mavros/vision_pose/pose` 频率异常（数千 Hz）+ `ps aux | grep -c odom_to_mavros` >1
- **修复**：杀光全部 odom_to_mavros + host_sdk_sample（`kill -9` PID 精确杀，勿用 `pkill -f`——会匹配到 plink 自身命令行导致 SSH 会话被杀 exit 128），**只按一次"启动雷达"**
- **已根治**：启动雷达按钮命令已集成 odom_to_mavros（`sleep 12 && python3 .../odom_to_mavros.py`，等 odin 起来再桥）——**勿再手动起**；Radar_Launcher.yaml odin cleanup 的 pkill_patterns 已加 `host_sdk_sample`/`odom_to_mavros`/`cloud_reprojection_node`/`image_overlay_node`/`pcd2depth_node`，"关闭雷达"能杀干净

#### 5b. vision_pose 单实例超频（odin 400Hz 全转发 → 限速 20Hz）— 2026-08-13 实测

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

#### 5c. vision_pose 频率上限与时间戳（2026-08-14 实测问答）

- **PX4 EKF2 处理 vision_pose 上限 ~100Hz**，建议上限 50Hz——超过收益递减 + 噪声相关性风险（同一时刻多观测被当独立观测 → 过度信任 → 漂移）
- **20Hz 对 1m/s 飞行足够**：EKF = IMU 预测(250-500Hz 连续内插) + 视觉修正(低频校准)；20Hz = 每 0.05s 修正一次，1m/s 移动 0.05m，IMU 内插完全覆盖。类比：IMU=秒表(2ms 跳)，视觉=对表(50ms 校一次)——对表频率不影响秒表精度
- **真正影响 1m/s 跟踪的是延迟不是频率**：odom_to_mavros 现在 `pose_map.header.stamp = rospy.Time.now()`（转发时刻当时间戳）→ 引入几十 ms 延迟 → 1m/s 下 50ms≈5cm 位置偏差
- **改原始时间戳（msg.header.stamp）的隐患——先查 `control_command.yaml` 的 `use_host_ros_time`**：
  - `=1`（主机时间）→ 用原始时间戳安全（治延迟）
  - `=0`（设备时钟）→ **保留 now()**（设备时间≠主机时间 → PX4 收到错位时间戳 → EKF 拒绝/错误补偿，比 now() 更差）
  - 其他隐患：驱动启动时时钟校准 → 时间戳前后跳；某些驱动 stamp=0/固定值 → PX4 拒收。查不了就**只改频率，时间戳改动等确认同步后再说**
- 频率调整：限速行 `>= 0.05`(20Hz) → `0.04`(25Hz) / `0.02`(50Hz)

#### 6. PX4 ready 判定与参数

- EKF2_EV_CTRL=11（视觉位置+速度融合，PX4 1.17 已废弃 EKF2_AID_MASK）
- **POSCTL/Offboard 就绪条件**：pos_horiz_rel=True + velocity_horiz=True 即可（视觉给的是相对位置；pos_horiz_abs 需 GPS/外部绝对源，**不一定需要**）
- 查参数：`rosservice call /mavros/param/get "param_id: 'EKF2_EV_CTRL'"`
- NOT READY 时看 `/mavros/estimator_status` 哪项 False 定位缺哪个源

#### 7. SUPER 导航接 odin（换传感器适配）

- SUPER 只依赖 `cloud_topic` + `odom_topic` 两个输入——换传感器只改传感器层，导航核心（方案C/F 代码）不动
- `task_run_scripts/config/real_flight.yaml` 的 `rog_map/cloud_topic` 从 `/cloud_registered`（FAST-LIVO2）改 `/odin1/cloud_slam`（odin SLAM 配准点云，frame_id="odom"，10Hz）——改完重启 real_flight 生效；odin 的 odometry 自带定位，**不需要 FAST-LIVO2**

#### 驱动版本管理（固件适配）

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

#### 话题↔开关映射（含两个命名坑）

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

#### 关联

- vision_pose 喂 PX4 EKF（EKF2_EV_CTRL 相关），与 uav-ros-diagnostics 场景联动；PX4 切 HOLD 事故背景见记忆
- 源码关键片段见 references/odin-source-key-snippets.md
- **odin bag 振荡/数据质量分析**（模仿 odin_bag_report 模板：IMU 质量/振荡检测/频谱/高频振动/odometry/结论 → md+图 → PDF）：完整配方、提取要点、全部坑清单见 references/odin-bag-oscillation-analysis.md。PDF 走 Edge headless（Start-Process 调用，PowerShell `&` 会被终端误判），无需 pandoc
- **双报告对比（交控方 vs 留形方）"同等条件我方无问题"论证模板**：报告 8 节结构、四层科学论证（同一检测器两种定性→量化差异→对方数据自带线索→方法学自洽）、证据链红绿矩阵、黑暗环境专项论点（dToF 主动光源不受光照影响）、措辞基调，见 references/odin-bag-report-comparison.md
- ★ **判读铁律（08-19 用户纠正）**：1s RMS 滑窗只测"运动活跃度"，区分不了手动机动 vs 故障振荡——曾把全程打杆悬停机动误判成"振荡 100%"。**悬停前后左右摇摆+摇杆活跃 = 正常机动**；须用摇杆 manual_control 过零/幅度、位置轨迹往返、姿态-加速度一致性、vision_pose 稳定性验证，报告写"运动活跃段"而非"振荡段"


---
