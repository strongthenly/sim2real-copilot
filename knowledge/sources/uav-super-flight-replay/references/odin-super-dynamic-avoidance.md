# odin 传感器适配 + SUPER 动态避障调试（2026-08-14 全记录）

## 背景
fouruav 传感器从 MID360（FAST-LIVO2）换成 odin1（留形科技，dToF LiDAR+鱼眼+IMU，自带 SLAM）。SUPER 导航核心只依赖 3 输入（点云/里程计/位姿），只需改传感器层。本文件记录 odin 适配 + 动态避障（残留/卡死）调试的完整结论。

## odin 数据链路（当前定稿）

```
odin_ros_driver:
  /odin1/cloud_raw      frame=lidar  （当前帧原始点云 49152 点, 10Hz, 含 intensity+confidence 字段）
  /odin1/cloud_slam     frame=odom   （SLAM 配准点云 32864 点, 累积地图）
  /odin1/odometry_highfreq            （400Hz 高频里程计）
  TF: odom→imu（send_odom_baselink_tf: 1）, 还有 odom→lidar

位姿桥: odom_to_mavros.py → /mavros/vision_pose/pose → PX4 EKF（EKF2_EV_CTRL=11）
地图:  cloud_topic → FSM rog_map
```

## 关键坑

### 1. odom_to_mavros 双坑（NOT READY 根因）
- **多实例**：一次按钮点击由 bash 命令链拉起 4 个实例 → vision_pose 6570Hz 打架 → EKF 判视觉不可信 → NOT READY。**修复：pid 文件锁**（脚本开头查 /tmp/odom_to_mavros.lock，存在且进程活则退出）。不能用 ROS 节点名查重（anonymous=True 节点名带 PID 后缀 + 启动时序查不到）
- **超频**：odin odometry 400Hz 全转发 → vision_pose 500-1000Hz → EKF 拒收。**修复：回调限速 20Hz**（`if (now-last_pub).to_sec() < 0.05: return`）
- 判读：`ps aux | grep 'python3.*odom_to_mavros' | grep -v grep | wc -l` 应=1（grep 误计 bash 包装：命令行含字符串但非 python3 进程）；`rostopic hz /mavros/vision_pose/pose` 应 ~20Hz；`rostopic echo /mavros/estimator_status` 看 pos_horiz_rel=True
- **时间戳坑**：odin 消息 header.stamp 是设备时钟（与主机差几百秒），任何查 TF 的桥接节点必须 `rospy.Time(0)` 取最新 TF，用 msg.header.stamp 会 `Lookup would require extrapolation` 静默失败

### 2. cloud_raw 桥接节点（cloud_raw_lidar_to_odom.py）
cloud_raw frame=lidar，rog_map 的 cloudCallback **不做 TF 变换**（`pcl::fromROSMsg` 直接用）→ 点云错位 → 必须桥接：
```
订阅 /odin1/cloud_raw → tf2 lookup_transform('odom','lidar', Time(0)) → do_transform_cloud
→ 地面过滤 z>=0.1 → numpy 体素降采样 5cm（np.floor(arr/0.05).unique）
→ 发布 /odin1/cloud_raw_odom（frame=odom）
```
- 降采样：49152 → ~13000 点（否则 FSM CPU 116% 过载 → Replan over time）
- 地面过滤 z<0.1：地面点占 19%，不过滤则地面全是障碍（"被障碍包围"视觉）
- 代码坑：`from pcl_ros import point_cloud` 不存在（ImportError），用 `sensor_msgs.point_cloud2` + numpy 即可；`make_voxel_grid_filter()` 静默不生效，numpy 实现可靠

### 3. virtual_ground 量化 bug（无地图根因）
- prob_map.cpp 初始化 `posToGlobalIndex(virtual_ground_height)` 后回写 `id * resolution`
- **配置 -0.9 → 量化后 +0.15**（地面 z≈0 < 0.15 → below ground → 地图拒绝更新 → 无地图）
- 配置 -3.0 → -1.95（正常）。**修复：virtual_ground_height: -3.0**
- ceil 同理：virtual_ceil_height: 3 被地图尺寸钳制（map_size z=6 → 1.95），z>1.95 的点全判"越界=OCCUPIED"→ 高处全屏障碍。map_size z 加到 8 + ceil 4 可解
- **判读：`grep -aE '\[ProbMap\] virtual' real_flight.log` 看量化后实际值，不是 yaml 值**

### 4. 全屏障碍真根因 = fix_map_origin [0,0,1.5]→[0,0,0]
- 坐标系抬高 1.5m → 点云 z 相对地图原点 = z-1.5 → 地面点变"空中障碍"散布全屏
- 地面过滤 z<0.1 在地图系里错位失效
- **铁律：一次只改一个参数**。排查用 .bak 备份二分：`ls -la config/real_flight.yaml.bak*` → 对比关键参数 → cp 回"全屏障碍前"的备份（本会话恢复 .bak_odintopic）
- 注意备份的 cloud_topic 可能是旧值（/cloud_registered MID360 遗留），odin 需改回 /odin1/cloud_slam
- **改完配置必须确认 FSM 重启加载**（进程启动时间 vs 文件 mtime；日志 `Load param` 行是铁证）

### 5. 障碍残留 = 雷达贴地几何遮挡
- 地面测试雷达离地 0.1-0.3m → 激光低处扫 → 障碍背后遮挡无光线 → raycasting 清不到 → 残留
- 飞机拿起（雷达 1m+）→ 俯视全 → 残留消失（用户实测）
- **残留严重度与雷达高度强相关，飞行作业时残留本来就少；地面测试垫高雷达更接近飞行**

### 6. 全向光线 omni（主动清除残留）
- ROG-Map raycasting 是"被动清除"（只清激光扫过的方向）→ 空旷方向旧障碍残留
- 在 prob_map.cpp raycastProcess 点云光线循环后加全向稀疏探测光线：
  - 水平 step_h×垂直 step_v×fov_v ±60°×range 8m
  - 遇"当前帧 hit_cnt>0 的格子"停止（不清真实障碍），其余 insertUpdateCandidate(false) 清除
- yaml 配置：`omni_en: true, omni_range: 8.0, omni_step_h: 5.0, omni_step_v: 7.5, omni_fov_v: 60.0`
- **step_h 10° 时 8m 处光线间距 = 8×tan(10°) ≈ 1.4m 会漏细障碍；加密到 5° 残留明显改善，CPU 43%→45%**
- 代码坑：hit_cnt 是 `std::vector<uint16_t>`，用 `hash_id < size() && hit_cnt[hash_id] > 0` 索引访问
- 曾误判"omni 误清地面→被包围"回退（实为 fix_map_origin 改坏），恢复基线后重启用效果良好

### 7. 停车避让状态机 AVOID_WAIT（动态障碍不卡死）
- 原本：GENERATE_TRAJ 失败→留在原状态高频空转重试→bridge 0.5s 超时才被动悬停→视觉"卡死"
- 改 3 文件：
  1. `include/fsm/fsm.h`：枚举加 `AVOID_WAIT` + MACHINE_STATE_STR 加 "AVOID_WAIT" + 成员 `double avoid_wait_t_{0.0}` + `virtual void publishHoldPose() = 0;`
  2. `src/super_core/fsm.cpp`：GENERATE_TRAJ 失败分支 `publishHoldPose(); avoid_wait_t_=0.0; ChangeState(AVOID_WAIT)`；AVOID_WAIT case 每循环 `publishHoldPose()` + `avoid_wait_t_ += 0.1`，≥0.3s 重试 PlanFromRest，成功恢复 FOLLOW_TRAJ
  3. `include/ros_interface/ros1/fsm_ros1.hpp`：publishHoldPose 实现（PositionCommand：position=robot_state_.p、velocity=0、acceleration=0、yaw=robot_state_.yaw、trajectory_flag=0，发 `cmd_pub` 无下划线！cmd_pub 在 334 行 advertise 到 cfg_.cmd_topic=/planning/pos_cmd）
- 实测：人挡住→停车→人走开→0.06s 内恢复飞行；日志 `AVOID_WAIT: path recovered, resume flight.`
- 障碍持续存在时"恢复→又挡→又停"循环 = 物理限制（人等走才能继续），不是 bug

### 8. 多次穿行后卡死 = 终点判定 0.1m 嫌疑（待验证）
- 现象：多次穿行后日志 `FOLLOW_TRAJ + Using FULL GLOBAL PATH ×258`（轨迹一直在生成）但飞机不动
- 强嫌疑：super_planner.cpp:337 `(robot_state_.p - gi_.goal_p).norm() < 0.1 → traj_finish=true → 悬停`
- 验证：注释 0.1m 判定重编译测试，或每次 traj_finish 打日志
- `Using FULL GLOBAL PATH idx=0` 不是 bug（起点时最近点=0；global_last_idx_ 单向推进，fallback 不清除）

## 避障链路判读要点
- 日志关键词：`Global path blocked`（检测）→ `A* fallback path corridor OK`（绕行路径走廊成功）→ `GenerateExpTrajectory SUCCESS`（主轨迹）→ `generateBackupTrajectory return [OPT_FAILED]`（backup 失败不影响主轨迹）→ `AVOID_WAIT`（停车避让）
- 主轨迹 SUCCESS 是绕行执行者；backup 只是第二保险（失败无碍）
- A* 绕行路径贴障碍（min dis 0.04-0.15m）→ 走廊 GeneratePolytopeFromLine failed → inflation_step 3（0.6m 膨胀）让路径离障碍远

### 9. 绕行回正俯冲修复（bridge 加速度低通 + jerk 平滑）
- 现象（1m/s 实机）：绕行→回正切换时 z 0.68→1.02→0.84（冲高回落=俯冲），y -1.29→-0.43 回正
- 根因：MINCO 轨迹在切换点加速度突变 → bridge 原样透传 `setpoint_raw.acceleration_or_force`（加速度前馈）→ PX4 高度骤变
- 修复①控制层：`super_to_px4_bridge.cpp` 加速度前馈一阶低通 `acc_ff = α×新 + (1-α)×旧`，α=acc_ff_smooth（0.3 太钝→0.5 平衡；0.3~0.7 区间可调）
- 修复②轨迹层：real_flight.yaml `penna_jerk` 主轨迹 -5e5→**-7.5e5**（-1e6 太平滑拖慢，折中）、backup -1e5→-5e5
- 原理：俯冲=加速度突变——治本走轨迹 jerk 权重（越大越平滑），治标走控制低通（越快响应越易振荡），双保险最优

### 10. FAPP 算法调研（arXiv 2312.08743）+ 动态避障刷新频率
- **FAPP = Fast and Adaptive Perception and Planning for UAVs in Dynamic Cluttered Environments**（港科大 Lu Peng 团队 2023-12）
- 4 模块：①点云分割静态/动态（**最大价值**）②自适应运动估计（协方差适应）③高效轨迹优化 ④自适应重规划（=已实现的 AVOID_WAIT）
- **落地路线（不碰 C++）**：Python 桥接 cloud_raw_lidar_to_odom.py 里做"当前帧 vs 静态累积图差分"→ 动态物点剔除不进静态地图 → 残留/误判/卡死全解（阶段 1）；阶段 2 可选卡尔曼预测
- **ESDF 不建议开**：从占用图派生（残留同源，对残留清除无帮助）；3D 全图更新开销大（FSM CPU 45%→80%+ 风险，Orin NX 8GB 边缘）；当前膨胀层（inflation）已提供安全距离
- **刷新频率评估**（1m/s）：cloud_raw_odom 6.5Hz（桥接降采样耗时 0.15s/帧）+ replan 10Hz → 检测延迟 0.2-0.3s = 1m/s 下 0.3m 提前量；慢走的人（相对 2m/s）边界可用，快跑的人不够。提升路径：优化 numpy 降采样或增大体素（0.05→0.08）→ 目标 10Hz
- 注意 odin cloud_raw 置信度：99% 点 conf>35（噪声不是残留主因，别在置信度上浪费时间）

### 11. FAPP 阶段1 实际实施（cloud_dynamic_filter.py，2026-08-14 晚）
**目标**：动态点分割——人(动态)不进静态地图 → 残留/误判/卡死全解。Python 桥接实现，不碰 C++。

**① 前置数据验证（必做，决定方案A/B）**：人走进视野时对比 cloud_raw(当前帧) vs cloud_slam(累积) 体素差分（VOX=0.2m）：
```
基线: cloud_slam 1352 体素
人走进: raw 1744 vs slam 1348 → 差分 462 体素（人=动态, 能被识别!）
人走后: slam 新增仅 20 体素（1.5%）→ cloud_slam 基本纯静态 → 方案A（当前帧 vs 静态背景差分）可行
```
验证脚本要点：分 3 步（人离开基线 → 人走进站立 → 人走开），每步取点云体素集合求差集；**系统没启动(host_sdk=0)时全 0 输出是假象，先确认 odin 在跑**。

**② cloud_dynamic_filter.py 实现**（scripts/ 下，已提交 feat/4uav-guide）：
```
订阅 /odin1/cloud_raw → TF(Time(0)) 变换到 odom → 地面过滤 z>=0.1
→ 体素化 VOX=0.2 → 逐点查静态背景(cloud_slam 体素集合) → 有=static / 无=dynamic
→ 发布 /odin1/cloud_static(进地图) + /odin1/cloud_dynamic(RViz 可视化)
背景刷新: 每 2s 取最新 cloud_slam 重建体素集合(threading 后台, 防阻塞)
```
实测：static 15750 点 4Hz、dynamic 7087 点 3.8Hz，脚本本身开销小。

**③ ⚠️ 关键发现：naive 差分过度分割**（实施后立即暴露）：
```
dynamic 7537 点中仅 14% 在"人区域"(前方1-4m)，86% 是散布点(误分!)
根因: cloud_slam 只含"被扫过多次/配准稳定"的静态点；
      单次扫到的静态边角/远处点不在 slam 里 → 全被当动态剔除
→ 地图会缺静态结构(边角/远处墙), 不是可用状态
```
**修复方向（已定未实施）**：①**背景累积**：静态背景 = cloud_slam ∪ 历史 static 体素（每帧把确认静态的体素累加进背景，边角静态点第 2 帧就入背景，误分大幅下降）；②**动态判定加空间连续性**：dynamic 输出前做邻域密度检查（孤立散点剔除，成块点保留——周围 0.3m 内 ≥3 点才发），人=成块、噪声=散点。

**④ 接入方式**：real_flight.yaml `cloud_topic: /odin1/cloud_static`；验证：人走进 → RViz 看 cloud_dynamic 里人成块移动、cloud_static 无残留；人走 → dynamic 消失；静态墙/箱子一直在 static。

**⑤ v2 背景累积(A1) 失败——鸡生蛋缺陷**：只把"被判 static 的点"并入背景，但边角静态点永远被判 dynamic → 永远不累积 → 误分依旧（84%）。背景更新必须让**所有当前帧点**参与（带遗忘），不能只累积 static 结果。A2 密度聚类（scipy cKDTree 0.35m/3点）单独用也不够。

**⑥ ⭐ v3 时间一致性滑动窗口（尝试的正解，但最终因 odin 点云特性放弃）**：维护体素计数表 `coord → (count, last_time)`，每帧所有点 count++（上限 20），FORGET_T 未命中即遗忘；`count >= CONFIRM_N → static，< → dynamic`；动态候选再过密度聚类。基线正常：static 23295 / dynamic 8。
- **CONFIRM_N 敏感性（核心调优）**：5(0.5s)→人站 0.5s 转 static（残留回归）；20(2s)→仍短；60(6s)+FORGET_T 8s→穿行场景合理（穿行停留<6s 全程 dynamic，站 6s+ 的静止物转 static=等同静态障碍）。⚠️ FORGET_T 必须 > CONFIRM 时长，否则 count 中途被遗忘永远到不了阈值
- 判读坑：分割结果的"人形判断"要用**变换后 odom 系坐标**（飞机不在原点时 lidar 系范围误判）；人确实在 cloud_raw 但 static/dynamic 都没有 → 检查 CONFIRM 过短已转 static / 坐标系判断错

**⑦ ⭐ 最终结论：纯时间一致性分割对 odin dToF 点云不可靠 → FAPP 阶段1 暂停**（2026-08-14 晚实测）：dToF 点云每帧约 2 万点**间歇性闪烁/抖动**（同一物理点帧间在相邻体素间跳动）→ CONFIRM_N=60 时静态物（墙）体素 count 也到不了阈值 → 大量静态点误判 dynamic（实测 static 150 / dynamic 23318 反转）；VOX 0.2→0.4 部分改善（static 20540）但闪烁点仍 ~2 万误分。**根本矛盾：CONFIRM_N 小→人站 0.5s 转 static（残留回归）；CONFIRM_N 大→静态物也到不了（闪烁）**。要区分"站住的人"和"静止的箱子"必须做**运动检测**（体素位置序列漂移/速度一致性，FAPP 论文方法），纯计数不行。**处置：暂停动态分割，地图继续 cloud_slam + 全向光线（已实测残留改善 90%），优先排查"多次穿行卡死"根因（坑8/坑52→实为 fsm_node 崩溃）**。如重启 FAPP：①先做运动检测（每体素记录最近位置序列，位置漂移=动态）②或依赖 odin SDK 自带 SLAM 动态过滤能力

### 12. ⭐ p_miss 语义铁律（曾误调 0.6 → 全屏障碍）
- **p_miss 是"光线穿过格子的目标概率"，不是"清除强度"**。命中点→概率向 p_hit(0.70) 更新；光线穿过→概率向 **p_miss** 更新。判定：p_free=0.4999（<则自由）、p_occ=0.85（>则障碍）
- **p_miss=0.49(<0.4999) → 光线多次穿过概率降到自由 ✅；p_miss=0.6(>0.4999) → 概率停在 0.6 永远不自由 → 之前清掉的格子全变回"非自由" → 全屏障碍块**（2026-08-14 实锤，用户改完立即全屏）
- 想加快清除：调**小** p_miss（0.3-0.4，会误清障碍边缘慎调）或**加密全向光线**（omni_step_h 10→5），不是调大
- 铁律：**改概率参数前先确认它落在判定阈值（p_free/p_occ）哪一侧**；怀疑清除问题时先 `grep -A12 'raycasting:'` 看 enable + p_miss/p_free 相对关系

### 13. ⭐ "多次穿行卡死"真凶 = fsm_node 崩溃（corridor_generator.cpp:113 未捕获异常）——终点判定只是嫌疑已排除
- **现场抓取（关键方法论）**：卡死类问题事后查日志=状态丢失白折腾。让用户"卡死时保持系统运行别关"，立即查：①`ps aux | grep fsm_node`（进程没了=崩溃）②`rostopic hz /planning/pos_cmd`（no new messages=没人发指令）③`tail -60 real_flight.log`——**日志尾部出现 `Source ".../xxx.cpp", line N` + `#N` 堆栈帧 = 崩溃堆栈铁证**（进程已死）
- **崩溃链**：fsm.cpp:77 callReplanOnce → super_planner.cpp:191 ReplanOnce → generateExpTraj → :845 SearchPolytopeOnPath → **corridor_generator.cpp:113 `throw std::runtime_error("seed line too long")`**（A* 绕行路径相邻两点距离 > seed_line_max_length_×1.5=1.8m，异常未 try-catch → terminate → 进程死）→ 无 pos_cmd → bridge 超时悬停 = 视觉"卡死"
- **修复（fsm_node 16:16 编译）**：①**长段插值**——SearchPolytopeOnPath 生成 seed_lines 时相邻点距离 > seed_line_max_length_ 则 `n_split=ceil(len/max)` 中间插点（每段 ≤max，走廊正常生成）；②移除 throw
- **"防崩溃只是不报错"教训**：throw→return false 只保命（失败→AVOID_WAIT 等待恢复），要真正绕行必须插值治本（否则 A* 长段 → 走廊永远失败 → 反复等待）
- 终点判定 0.1m（super_planner.cpp:337）注释后实测仍卡死 → **已排除**（注释版 fsm_node 16:03）
- 验证：fsm_node 16:16 后日志 `seed line too long: 0`、AVOID_WAIT 11 次全部 recovered ✅
