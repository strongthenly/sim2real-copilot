# 方案A / odin 链路 / 1m/s 振荡排障（2026-08-13，<NET_WLAN_B>.31）

## 方案A：绕行时大盒子（corridor_small_ 标志）
- **问题**：F2 小立方体(0.2m)在 fallback 绕行时也生效 → A* 绕行路径贴障碍(CIRI min dis 0.04) → 走廊凸分解失败 → MINCO 全失败 → bridge 0.5s 超时 → position hold（现象="绕行趋势但暂停原地"）
- **改动**：
  - `exp_traj_optimizer_s4.h`：`bool corridor_small_{true}` + `setCorridorSmall()`/`corridorSmall()`——**setter 必须放 public 区**（类成员默认 private，编译报错 `is private within this context`）
  - `exp_traj_optimizer_s4.cpp` processCorridorWithGuideTraj：`corridor_small_ ? 小立方体(8顶点,半宽0.1m) : 大盒子(curIOB)`
  - `super_planner.cpp`：fallback 分支(约853行) `setCorridorSmall(false)`；正常循迹 "Using FULL GLOBAL PATH" 后(约706行) `setCorridorSmall(true)`
- 编译：`catkin_make -j2 -DROS_EDITION=ROS1`，确认 fsm_node 时间戳更新

## 终点判定（防终点来回摆）
- `super_planner.cpp` getOneCommandFromTraj：`(robot_state_.p - gi_.goal_p).norm() < 0.1 → traj_finish = true`（根因：idx 只剩2点时走廊生成失败→无新轨迹→终点附近摆）

## backup 优化（max_omg / max_acc_thr）
- backup 失败日志 `Opt failed, Omg or thr or Pos violation` → `traj_opt/boundary/max_omg` 2.5→6.0、`max_acc_thr` 20→25
- backup(BackupTrajOpt) 惩罚系数用独立段 `traj_opt/backup_traj`（penna_*），但物理约束 max_omg/max_vel/max_acc 共用 `traj_opt/boundary`（config.hpp 102-106 行加载）
- **backup 走独立链路**（generateBackupTrajectory → GeneratePolytopeFromLine → BackupTrajOpt），不受方案A 的 exp_traj vPolytopes 改动影响

## odin 链路（<NET_WLAN_B>.31，fouruav-3）
- 启动雷达 = `Radar_Launcher.yaml` 的 active_profile(odin)，命令链：odin1_ros1.launch → sleep 9 → px4_and_others.launch → sleep 12 → python3 odom_to_mavros.py
- **px4_and_others.launch 里的 odom_to_mavros_node 是注释掉的**——唯一启动源是 Radar_Launcher 的 python3
- **ready→not ready 根因链**：①多启动源/多实例 → 多个 odom_to_mavros 抢发 vision_pose(6570Hz 打架)；②单实例 400Hz odometry 全转发 → vision_pose 500-1000Hz → PX4 EKF 拒收超频 → 融合失败 NOT READY
- **修复**：odom_to_mavros.py 加 pid 文件锁(`/tmp/odom_to_mavros.lock`，`os.kill(pid,0)` 探活) + 发布限速 20Hz(50ms 节流)
- 验证进程数：`ps aux | grep 'python3.*odom_to_mavros' | grep -v grep | wc -l`（**bash 包装进程命令行也含该字符串，必须加 python3 前缀精确匹配**）
- 地图 cloud_topic：`/odin1/cloud_slam`（累积配准点云）。**cloud_raw（当前帧）对动态障碍人走即消更好，但曾导致无地图→已回退**——需先验证再切换

## 1m/s 振荡（俯冲）— 已实施修复（2026-08-14）
- 现象：1m/s 直行遇障碍绕行，恢复航迹时 z 冲高(0.68→1.02)再回落(0.84)=俯冲，y 绕行偏离后回正
- 日志判读：POS+VEL 相邻帧 z 突变 >0.15m；用 python 解析 POS+VEL 找 z/v 突变点（16MB 日志 grep 慢，python 快）。位置跳变 1m/帧 = 正常 1m/s（不是异常），z 突变帧才是振荡
- 根因：bridge(`super_to_px4_bridge.cpp`, **70Hz** Rate) 发 position + velocity×velocity_ff_gain(0.7) + acceleration 三重 setpoint——绕行→主航迹切换时 MINCO 加速度突变 → PX4 收到突变加速度 → 俯冲
- **修复①（bridge 加速度低通，已编译 10:41→10:50, 0 error）**：
  - `super_to_px4_bridge.cpp` 成员区加 `Eigen::Vector3d prev_acc_ff{Zero}` + `double acc_ff_smooth{0.5}`（低通系数，0.5=50%新+50%旧——**用户折中：0.3 太肉响应慢，0.5 平滑与响应平衡**）
  - use_accel_ff 分支：`acc_sm = acc_ff_smooth*acc_raw + (1-acc_ff_smooth)*prev_acc_ff; prev_acc_ff = acc_sm;` 再赋值 setpoint_raw.acceleration_or_force；else 分支 `prev_acc_ff.setZero()`
- **修复②（MINCO jerk 平滑，纯 yaml）**：`real_flight.yaml` 主轨迹 `penna_jerk -5e5→-7.5e5`（负值=最小化jerk；**用户折中：-1e6 太平滑拖慢优化，-7.5e5 平滑与响应折中**），backup_traj 段 `-1e5→-5e5`。注意 jerk 过大可能致 MINCO 优化失败（Opt failed），若出现回调到 -5e5
- 验证方法：重启后看绕行回正段 z 是否渐变（不再 0.68→1.02→0.84 冲高回落）

## 航点文件 CSV 格式坑（global_path_publisher）
- `global_path_publisher.py` 用 `csv.reader` 解析（默认**逗号分隔**，需表头 `x,y,z,qx,qy,qz,qw`）
- **空格分隔的航点文件会报 "[global_path] 文件为空" 直接退出**（float(row[0]) 解析整行失败）——生成航迹脚本必须写 CSV 逗号格式，不能 f"{x} {y} {z}" 空格格式
- 验证：前台 `timeout N rosrun task_run_scripts global_path_publisher.py _file:=...` 看 "发布 N 个航点" 日志

## rviz 视角坑
- **Fixed Frame 改成 imu（机载跟随视角）后地图不显示**——地图/轨迹是 world 系，imu 帧不存在或地图相对滑动。俯视用 `Fixed Frame: world`（top_down.rviz 572 行）
- rviz 配置文件是压缩 YAML（flow style），改字符串直接替换即可；rviz 进程需重启才加载新配置

## 通用技术坑（SSH 运维）
- **pkill -f 会匹配 plink 命令行自身**（命令行包含搜索串）→ SSH 会话被杀 exit 128。改用 `ps aux | grep X | grep -v grep | awk '{print $2}'` 拿 PID 再 `kill -9`（批量杀时 for 循环）
- **nohup python3 后台启动 ROS 节点可能卡住**（日志 0B、话题注册但 hz 0、CPU 2-3% select 等待）——前台 `timeout N rosrun ...` 验证正常后，用 `setsid bash -c 'source ... && source devel/setup.bash && exec python3 ... > log 2>&1' &` 启动
- **latch 话题 hz 统计不到**（只在新订阅时重发）→ 用 `rostopic echo /topic -n 1` 验证数据在发，不要误判"没发布"
- **同名 ROS 节点互相顶掉**（odin1_ros1.launch 的 rviz name="rviz" 与 real_flight.launch 的 rviz 冲突，日志 "new node registered with same name"）→ 改名 rviz_topdown
- **virtual_ground_height 网格量化 bug（2026-08-14 实锤）**：配置 -0.9 → 量化后实际 **+0.15**（ProbMap 初始化日志 `virtual_ground_height: 0.15`）→ **地面测试飞机 z≈0 < +0.15 → 每帧刷 "Odom below virtual ground" → 地图拒绝更新 = 没地图**。飞行测试 z=0.6 > 0.15 正常——**这就是"地面没地图、飞起来正常"的根因**（量化取整 posToGlobalIndex = int(1/res × pos + SIGN×0.5)，见 sliding_map.cpp 177-191 行 ORIGIN_AT_CENTER 模式）。**注意：virtual_ceil 3 → 日志 1.95 不是量化 bug**——量化验证 3.0/0.15 → id=20 → 3.00 正确；真因是 map_size z 边界钳制（见下方"被障碍包围"段）
- **修复**：`virtual_ground_height` 改 **-3.0**（绕过量化，量化后仍 <0）→ 地面 z≈0 在地图内 → 地图正常
- 判读以 **ProbMap 初始化日志的量化后值**为准（不是配置值）；`fix_map_origin [0,0,1.5]` 是地图滑动参考点，与 below ground 无直接关系

## "飞机被障碍包围"根因：map_size z 边界钳制（2026-08-14 最终）
- **现象**：omni 调参（fov_v ±60°）后 RViz 里飞机及扫到区域**全是障碍块**。先疑 omni 误清地面（关掉 omni_en=false 仍被包围）→ 排除 omni，是地图本身 z 范围错
- **数据**：`rog_map/occ` 7515 点，z 范围 -0.38~1.88，飞机周围 ±2m 占 32.5%；点云本身正常（x 0~4.4, y ±4, z -0.3~3.5，飞机周围仅 4.1%）
- **根因链**：`map_size: [30,30,6]` → 地图 z 半高仅 3m → `virtual_ceil_height: 3` 正好在地图边界 → **被钳制到 1.95**（不是量化 bug！量化 3.0→3.00 正确）→ `getGridType` 判 `pos.z() >= virtual_ceil → OCCUPIED` → **点云 z>1.95 的高处点全部被当障碍** → 视野内高处全标障碍 = "被包围"
- **修复（纯 yaml）**：`map_size z 6→8`（地图高度 -4~+4）+ `virtual_ceil_height 3→4`（量化后 3.9）→ 点云 z 0~3.5 全在地图内 → 高处不再是障碍
- **教训**：地图高度参数（map_size z / virtual_ceil / virtual_ground）三者必须匹配，且以 ProbMap 日志量化后值为准；**点云 z 范围要落在 [virtual_ground, virtual_ceil] 内**，否则越界点全被 getGridType 当 OCCUPIED（getGridType: `pos.z() <= virtual_ground || >= virtual_ceil → OCCUPIED`，prob_map.cpp 369-373 行）
- **通用检查**：`rog_map/occ` 话题读点云统计 z 范围 vs 地图 virtual 范围，快速定位"全障碍/被包围"是地图高度配置问题还是点云问题

### "被包围"第三根因：地面点被扫成障碍（地面过滤，2026-08-14 最终）
- **现象**：map_size z 6→8 + ceil 4 修复后**仍**"飞机本身和面前都是障碍块"（omni 已关、地图高度已对，问题依旧）
- **数据定位**（读 `/fsm_node/rog_map/occ` 量化）：总占用 7634 格，z 范围 -0.38~1.88，**z<0.3 低处占 14%**，前方 x>0.5 占 **96%**（障碍几乎全在雷达扫描方向）；对照点云 `/odin1/cloud_raw_odom`：**z<0.1 的地面点占 17-19%**（z 中位数 1.31，说明大量地面点）
- **根因**：odin 雷达向下/前扫描 → **地面点(z≈0)被点云 hit → 地面层全 occupied** → 飞机"站在障碍地上"+面前全是地面障碍块。MID360+FAST-LIVO2 时代点云有地面但处理不同
- **修复**：`cloud_raw_lidar_to_odom.py` 桥接加**地面过滤**——`GROUND_Z = 0.10`，`arr = arr[arr[:,2] >= GROUND_Z]`（numpy mask 剔除地面点，在体素降采样之前）。验证：点数 16440→13350，z 范围 0.10~3.48，z<0.1 占 0%
- **注意**：地面过滤只影响新点云——**地图里已累积的地面障碍需重启按钮3 重建地图**才清掉
- **飞机附近障碍量化检查（通用方法）**：python 读 `/fsm_node/rog_map/occ`（PointCloud2 → pc2.read_points）统计：总格数、x/y/z 范围、飞机为中心 0.5/1/1.5/2/3m 半径内格数占比、z<0.3 低处占比、前方 x>0.5 占比——快速区分"被包围"是地图高度配置 / 地面点 / 真实物体 / 噪声
- **遗留**：地面过滤后飞机 0.5m 内障碍 2%（正常），但 3m 内 64%、前方 96%——前方密集障碍可能是真实物体或 cloud_raw 低置信噪声点（confidence threshold 35 可能太低），下一步可在桥接加 confidence/intensity 过滤

## 地图不更新排障顺序
1. fsm_node 在跑吗（ps）→ 不在 = real_flight 没起/退了（先启动雷达再按按钮3）
2. cloud_topic 加载确认（日志 "Load param rog_map/ros_callback/cloud_topic success"）+ FSM 订阅了（`rostopic info /odin1/cloud_slam` Subscribers 含 /fsm_node）
3. ProbMap 日志：Init successfully / Odom below virtual ground / Map is empty
4. cloud_slam hz > 0（odin 驱动在跑）

## 动态障碍卡点与解法（2026-08-14，实测数据定位）
**卡点链路**（不是检测，是绕行轨迹生成）：
```
障碍出现 → 检测✅(blocked 632次) → A*绕行路径✅(但贴障碍0.04~0.2m)
→ 走廊生成❌(GeneratePolytopeFromLine failed, 要求离障碍≥安全距离)
→ MINCO绕行轨迹❌(Opt failed 74458次) → 无轨迹 → bridge 超时 → 悬停
```
**判读纠正**：`GenerateExpTrajectory SUCCESS 18619次` = 无障碍时的正常循迹（沿航迹飞），**不是**绕行成功；绕行时刻主轨迹失败→backup 也失败(18676次)→悬停。backup 与主轨迹是两条独立链路，backup 失败不影响主轨迹飞行，但主轨迹失败瞬间无兜底。

**解法（组合，不要只改一个）**：
- ① `rog_map/inflation_step 2→3`（膨胀0.4→0.6m）：A* 搜索时障碍膨胀更大 → 绕行路径离障碍≥0.6m → 走廊能扩出 → MINCO 有空间 → **绕行轨迹成功**（解决"绕行可行性"）
- ② `cloud_topic /odin1/cloud_slam → /odin1/cloud_raw`（当前帧）：人走 → 当前帧无人的点 → 光线清除 → 假障碍消失（解决"障碍残留"，632次blocked 里很多是已走的人）。**注意曾因 fsm 未启动+virtual_ground 误判导致"cloud_raw 无地图"，不是 cloud_raw 本身问题——验证时先确认 fsm 起来**
- ③（备选，未实施）backup 直发 A* 路径（不做 MINCO 优化，折线也能飞）：backup 永远成功，终极兜底
- 两改动互补：①让绕行可行，②让假障碍消失——只做①会因膨胀残留加重假障碍，只做②绕行仍失败

## cloud_raw frame 变换桥接（动态障碍残留根治，2026-08-14 实锤）
**关键事实：odin 点云 frame 不同**：`cloud_slam` = odom 系（累积配准点云，地图直接用），`cloud_raw` = **lidar 系**（当前帧原始点云）。
**rog_map 不做 frame 变换**：`rog_map_ros1.hpp` cloudCallback(97-115行) 直接 `pcl::fromROSMsg` 原样给地图 → **cloud_raw 直接喂地图 = 点云错位放置 → 飞机附近全是假障碍 → 走廊扩不出 → 动态障碍残留严重**。这就是"cloud_raw 方案之前失败"的另一半真相（另一半是 virtual_ground bug）。
**解法：写桥接节点** `scripts/cloud_raw_lidar_to_odom.py`（订阅 /odin1/cloud_raw → TF odom→lidar 变换 → 发布 /odin1/cloud_raw_odom），`cloud_topic` 改 `/odin1/cloud_raw_odom`。
**TF 时间戳坑（必踩）**：桥接查 TF 若用 `msg.header.stamp` 会报 `Lookup would require extrapolation ... into the past`（设备时间戳与主机时间差数百秒）→ **必须用 `rospy.Time(0)` 取最新 TF**（与 odom_to_mavros 同理），输出时间戳用 `rospy.Time.now()`。
验证：`rostopic hz /odin1/cloud_raw_odom`（~6.5Hz 正常，10Hz 输入丢一部分没关系）+ frame_id="odom" + FSM Subscribers 含 /fsm_node。
**动态障碍残留的完整解决链 = cloud_raw(当前帧) + TF变换到odom(坐标正确) + raycasting清除(p_miss)**——三缺一都残留。

### FSM 过载与体素降采样（2026-08-14，cloud_raw 接入后必踩）
- **症状**：cloud_raw 49152 点 × 6.5Hz 直喂 FSM → fsm_node CPU 116-117%（双核吃满）→ 日志刷 `Replan over time(0.17s)!!!! Return FAILED` → 规划失败循环 → 最终 fsm 正常退出（日志 "killing on exit"，**不是崩溃**，是过载后手动重启/被关）
- **根因**：cloud_raw 原始点云点数远多于 cloud_slam（累积配准点云）→ raycasting/地图处理过载 → 规划超时 → 卡死
- **修复**：桥接节点内**体素降采样**（5cm 栅格去重，49152→~16500 点）：
  - ❌ `out.make_voxel_grid_filter()`（pcl_ros 方法）**静默失败**（width 不变，日志无报错）——不可用
  - ❌ `from pcl_ros import point_cloud as pc` → `ImportError: cannot import name 'point_cloud'`——pcl_ros 无此模块
  - ✅ **numpy 手写降采样**：`pc2.read_points(out, field_names=("x","y","z"), skip_nans=True)` → `np.array(list(gen), dtype=np.float32)` → `key = np.floor(arr/0.05).astype(np.int32)` → `np.unique(key, axis=0, return_index=True)` 取 idx → `pc2.create_cloud_xyz32(out.header, arr[idx].tolist())`
  - 降采样后验证：`rostopic echo /odin1/cloud_raw_odom -n 1 | grep width`（应 ~16000 而非 49152）
- **地图"没更新"误判**：inf_occ 是 2Hz latch 话题，`rostopic hz` 短采样会报 no new messages（采样时机错过）——用 `rostopic echo -n 1` 看消息时间戳或用 8s 长采样确认，别误判地图停了

### raycasting 残留机制（2026-08-14 最终根因：只清"有点的方向"）
- **现象**：cloud_raw_odom 接入 + 降采样后仍残留，且**残留基本集中在地图上半部分（空旷方向）**
- **机制**（`prob_map.cpp` raycastProcess 672-800 行）：光线 = 从飞机(cur_odom) → **到每个点云点 p**（`raycaster.setInput(raycast_start, p)`，沿途格子 `insertUpdateCandidate(false)` 标记 free）
  - **只有"点云里有点的方向"才有光线** → 那些方向被清除
  - **空旷方向（当前帧无点云点）无光线 → 之前加入的旧障碍概率永不下降 → 永久残留**
  - 上半部分空旷 → 人站过留下障碍 → 人走 → 该方向仍无点 → 无光线穿过 → 残留
- **结论**：动态障碍清除依赖"当前帧点云覆盖该方向"。点云稀疏/方向缺失的区域 raycasting 无能为力——这是"cloud_raw 当前帧"方案的最后一块短板（前两块=frame 变换+FSM 过载已修）
- **解决（已实施 2026-08-14，fsm_node 12:08 编译 0 error）**：raycastProcess 加**全向探测光线**（除"点到点"光线外，按角度网格发探测线覆盖所有方向 → 空旷方向也有光线 → 旧障碍被清）。改动在 rog_map（SUPER 仓库）：
  - `config.hpp`：加载段(ray_range 后) + 成员区加 5 个参数——`omni_en`(bool,默认false)、`omni_range`(5.0m)、`omni_step_h`(10°)、`omni_step_v`(15°)、`omni_fov_v`(±45°)；yaml 的 raycasting 段加同名字段（omni_en: true）
  - `prob_map.cpp` raycastProcess 点云光线循环后加全向循环：水平 `for th_h 0..360 step_h` × 垂直 `for th_v -fov..fov step_v`，`dir=(cos(pit)cos(yaw), cos(pit)sin(yaw), sin(pit))`，`end=cur_odom+dir*range`，`start=cur_odom+dir*raycast_range_min`，逐格 `insertUpdateCandidate(id,false)` 标 free
  - **停止条件（关键，保护真实障碍）**：逐格前进时若**当前帧该格被点云命中**（hit_cnt>0）→ break 不清障碍本身
  - **编译坑**：`raycast_data_.hit_cnt` 是 `std::vector<uint16_t>`（prob_map.h 115行），**不是 map，没有 .find()**——`hit_cnt.find(hash_id)` 编译报错 `has no member named 'find'`。改用索引访问：`hash_id < hit_cnt.size() && hit_cnt[hash_id] > 0`（hash_id 范围与 insertUpdateCandidate 用的索引一致，安全）
  - 效果验证（**最终结论 2026-08-14 晚：omni 是成功的方案，勿回退**）：
    - 中途"飞机被包围在障碍块里"**不是 omni 的锅**——当时误判为 fov_v=±60° 扫地面，回退 omni_en=false 仍被包围 → 二分后查明**真凶是 fix_map_origin [0,0,1.5]→[0,0,0] 误改**（坐标系抬高 1.5m → 点云 z 偏移 → 地面点变"空中障碍"散布全屏）。omni_en=false 时它根本不执行，与全屏障碍无关
    - 恢复 `fix_map_origin [0,0,1.5]` + 恢复 .bak_odintopic 后地图正常 → 重新启用 `omni_en: true`（omni_range 8.0 / step_h 10 / step_v 10 / fov_v ±60）→ **残留明显改善**（occ 格数 7649→2136）
    - **残留本质补充：雷达贴地几何遮挡**——地面测试（雷达离地 0.1-0.3m）激光从低处扫，障碍"背后"区域无光线 → 残留多；**飞机拿起（雷达离地 1m+）残留消失**（用户实测）。飞行作业（0.6m+）时残留问题本身不严重
  - **加密光线（2026-08-14 晚，残留进一步改善）**：`omni_step_h 10→5°` + `omni_step_v 10→7.5°`（光线 ~468→~1224 条），**CPU 仅 43%→45%**（fsm_node，余量充足），残留"改善明显"（用户确认）。8m 处相邻光线间距 1.4m→0.7m（水平）
  - **FSM CPU 判读**：`ps aux | grep fsm_node` 看 CPU%——host_sdk(odin 驱动) 70%、python3 桥接 58%、rviz×2 35%+23% 都正常；加密光线几乎零额外开销
  - 残留"清了一部分"的另一半原因：**膨胀层残留**（inflation_step=3 → 0.6m 膨胀由障碍派生，原始栅格清了膨胀层可能滞后）+ p_miss 衰减慢——调 p_miss 0.6 有效但需配合膨胀刷新
- **1m/s 动态避障刷新频率评估**：cloud_raw_odom 降采样后 ~6.5Hz + 地图更新 6.5Hz(batch=1 每帧) + replan ~10Hz → 检测延迟 ~0.2-0.3s → 1m/s 下提前量 ~0.3m——**对慢走的人(0.5-1m/s)勉强够，对快跑(2-3m/s)不够**。提升方向：桥接降采样优化到 10Hz（numpy floor 去重已是最快路径，可加大体素 0.05→0.08）
- 备选：cloud_raw 当前帧 vs cloud_slam 静态图对比剔除消失障碍——复杂，优先级低

### vision_pose 频率与时间戳知识（PX4 EKF 约束）
- PX4 EKF2 处理 vision_pose 上限 ~100Hz，**建议 ≤50Hz**；**20Hz 对 1m/s 完全够**（EKF 靠 IMU 250Hz 内插，视觉只是低频校准：20Hz = 每 0.05s 修正一次，1m/s 下位移仅 5cm）
- **真正影响 1m/s 跟踪的是延迟不是频率**：odom_to_mavros 用 `rospy.Time.now()` 当时间戳（数据早于时间戳 → 延迟）vs 用 `msg.header.stamp`（原始时间戳 → EKF 补偿延迟）
- **改用原始时间戳的隐患**：若 odin 设备时钟与主机不同步（use_host_ros_time=0）→ 原始时间戳错位数百秒 → EKF 拒收/错误补偿 → **比 now() 更差**。先确认 `use_host_ros_time` 配置再决定；不确定就保留 now()

## 尾部未定论调试（2026-08-14 晚，全屏障碍持续 → 最终定论）

**最终真相（恢复 .bak_odintopic 后一切正常）**：
- **全屏障碍真凶 = fix_map_origin [0,0,1.5]→[0,0,0] 误改**（为修"点云 z vs 地图 z 差 1.5m"而改，实际那 1.5m 差就是 fix_map_origin 本身的坐标系设计，不是 bug）。改回 1.5 + 恢复 .bak_odintopic → 地图正常出、无全屏障碍
- **最终正常配置**：`cloud_topic=/odin1/cloud_slam` + `fix_map_origin [0,0,1.5]` + `map_size [30,30,6]` + `virtual_ceil 3` + `virtual_ground -3.0`（唯一保留的修复：绕过量化 bug）+ `inflation_step 2` + `omni_en true (step_h 5°)` + `p_miss 0.49` + `max_vel 0.3`（1.0 待恢复）
- **恢复方法（必踩教训）**：连锁改动后无法定位时，用 `.bak_odintopic`（全屏障碍前 15:06 正常版）整文件恢复——备份对比法（`ls -la real_flight.yaml.bak*` + 逐个 grep 关键参数）比逐参数试错快得多。**注意 .bak_odintopic 的 cloud_topic 是 /cloud_registered（MID360 遗留）**，恢复后需改回 /odin1/cloud_slam
- **方法论沉淀**：①一次只改一个参数，改完必须重启按钮3 + `grep 'Load param' 日志`核对加载值 ②FSM 可能没重启（日志仍是旧配置）——先确认 `ps aux | grep fsm_node` 启动时间 vs yaml 修改时间 ③以 ProbMap 初始化日志量化后值为准，配置值仅供参考 ④全屏障碍排查顺序：读 occ 量化(z vs 点云对照) → ProbMap 日志 → fix_map_origin 加载值 → 二分 cloud_slam vs cloud_raw

**fix_map_origin 坐标系抬高怀疑（已改未解决）**：
- 观察到点云(`/odin1/cloud_raw_odom`) z 0.10~3.47 vs 地图占用(`/fsm_node/rog_map/occ`) z -0.38~1.88——**差 ~1.5m，正好等于 fix_map_origin 原值 z=1.5** → 怀疑地图坐标系被抬高 1.5m（地面点变"空中障碍"散布全屏）→ 改 `fix_map_origin [0,0,1.5] → [0,0,0]`
- **改后仍全屏障碍**，且日志确认 fix_map_origin 已加载 [0 0 0]，但 ProbMap 仍显示 `virtual_ground_height: -1.95`、`virtual_ceil_height: 2.85`（配置 -3.0/4.0）→ **代码存在独立的地图 z 范围钳制**（约 -1.95~2.85，对称），不随配置完全一致——**以 ProbMap 初始化日志量化后值为准，配置值仅供参考**
- **二分测试法（推荐，一行 yaml 定位问题归属）**：`cloud_topic` 切回 `/odin1/cloud_slam`（此前正常出地图）重启按钮3——若 cloud_slam 地图正常 = 问题在 cloud_raw 桥接链路（TF/坐标系/过滤）；若 cloud_slam 也全屏 = 问题在地图参数/代码（z 钳制）。比逐参数试错快得多
- **置信度数据（排除噪声假说）**：cloud_raw 49152 点中 **99% confidence>35**（odin `cloud_raw_confidence_threshold: 35`）→ 低置信噪声不是全屏障碍主因，不必先加 confidence 过滤
- **排查顺序沉淀**：全屏障碍 → ①读 occ 量化（z 范围 vs 点云 z 对照）②查 ProbMap 日志量化后 virtual 值 ③查 fix_map_origin 加载值 ④二分 cloud_slam vs cloud_raw ⑤（若桥接链路）逐项查 TF/地面过滤/降采样

**验证流程（地面测试）**：启动雷达 → 按钮3 → 确认 inf_occ 2Hz（cloud_raw 建图✅）→ 人走近 2-3m → 日志 `Global path blocked` + `A* fallback path corridor OK`（走廊扩出✅）→ 人走开 → blocked 停止增加（清除生效✅）

**测试航迹**：动态障碍测试用小航迹（L形 前进1.2m+右移2m，R=0.5圆角，90点）比 12.5m 直行更适合（人可站前进段/右移段中间）；地面测试不装桨，人站雷达前 2-3m 走动即可验证检测+绕行规划。

## 用户文档偏好（SUPER.md 写作规范，2026-08-14 明确要求）
- **只写当前方案，不堆历史方案**（"不要之前的方案了，只写目前的方案"）——历史演进一句话带过或放 references，主文档保持"当前怎么工作"
- 章节结构：系统架构（数据流）→ 本次更改清单（按仓库分文件表）→ 待办事项 → **参数表（每个参数：当前值+作用）** → 传感器切换注意事项 → 已知坑
- **传感器切换章节必须有**（odin ↔ MID360S）：双向切换步骤（Radar_Launcher active_profile、cloud_topic、确认进程/TF）+ 通用坑（启动顺序、坐标系、单实例、ready→not ready）
- 参数解释讲清数值含义：penna_jerk 这类权重系数用大白话类比（-5e5=普通司机、-1e6=老司机、负数=最小化该代价），用户会追问"这两个数值代表什么"
- 文档更新后主动问是否推送 GitHub（Drone-Testing-Documentation，SUPER/SUPER.md）；网络不通时说明已存桌面待推

## FSM 状态机 + "停车避让"方案（2026-08-14 晚，已实施，fsm_node 14:52 编译 0 error）
**状态机**（`super_planner/src/super_core/fsm.cpp`，248 行）：`INIT → WAIT_GOAL → GENERATE_TRAJ → FOLLOW_TRAJ`（+ EMER_STOP）
**现状（为什么"卡死"）**：GENERATE_TRAJ 失败 → **留在 GENERATE_TRAJ 下轮重试**（无 sleep 注释在 `// ros::Duration(0.1).sleep()`）→ 但失败时**不发悬停指令** → bridge 0.5s 收不到新 pos_cmd 才被动切 position hold → 悬停期间 FSM 持续高频重试 PlanFromRest（A*/MINCO 耗时拖慢主循环）→ 障碍一直在 → 一直失败 → 视觉"永久卡死"。**理论上障碍消失重试能恢复，但无明确等待语义**
**PlanFromRest 失败路径**（super_planner.cpp:70-160）：①`getNearestCellNot(OCCUPIED, 起点, 3.0)` 失败 = 起点被障碍包围 → SUPER_NO_START_POINT ②generateExpTraj 失败 ③generateBackupTrajectory 失败（此时主轨迹已生成只缺 backup）——①是最常见的"人被围"卡死点
**已实施改动（3 文件）**：
- `include/fsm/fsm.h`：枚举 + `MACHINE_STATE_STR` 各加 `AVOID_WAIT`（注意字符串数组顺序必须与枚举一致，否则状态显示错位）；成员 `double avoid_wait_t_{0.0}`（避让计时）；虚函数声明 `virtual void publishHoldPose() = 0;`（放在 publishPolyTraj 声明旁）
- `src/super_core/fsm.cpp`：GENERATE_TRAJ 失败分支（原 `// ros::Duration(0.1).sleep()` 处）改为 `publishHoldPose(); avoid_wait_t_ = 0.0; ChangeState("MainFsmCallback", AVOID_WAIT);`；新增 `case AVOID_WAIT`：`avoid_wait_t_ += 0.1;`（FSM 主循环 ~10Hz）+ 每帧 `publishHoldPose()`（持续悬停）+ `if (avoid_wait_t_ >= 0.3)` 时重置计时并重试 `PlanFromRest`，成功 → `publishPolyTraj()` + `ChangeState(FOLLOW_TRAJ)` 恢复飞行（打日志 "path recovered, resume flight"），失败 → 继续等待
- `include/ros_interface/ros1/fsm_ros1.hpp`：`publishHoldPose()` 实现——构造 PositionCommand：`position = robot_state_.p`（当前位置）、velocity/acceleration 全 0、`yaw = robot_state_.yaw`、`trajectory_flag = 0`，`cmd_pub.publish(hold_cmd)`
- **必踩编译坑**：fsm_ros1.hpp 里发布器成员叫 **`cmd_pub`（无下划线）**（46 行声明、334 行 `cmd_pub = nh_.advertise(cfg_.cmd_topic, 10)`），不是 `cmd_pub_`！publishHoldPose 里写 `cmd_pub_.publish` 编译能过但运行崩溃（未 advertise 的 Publisher 调用 publish 是 UB/异常）——**必须与 334 行初始化的名字完全一致**。`cfg_.cmd_topic` = `/planning/pos_cmd`（yaml 17 行），正好是 bridge 订阅的话题
- **悬停指令原理**：bridge 订阅 `/planning/pos_cmd`（PositionCommand），0.5s 收不到新消息才切 hold_position；FSM 持续发"当前位置+零速度"pos_cmd → bridge 不超时且一直执行原地不动 = 主动悬停（比等超时响应快、不漂移）
**人=动态障碍卡死的物理本质**：人（1-2m 高大障碍）贴飞机近(<2m) + 移动 → 绕行规划空间不足 → A* 无解/走廊扩不出 → 无解悬停。这不是 bug 是规划空间限制——"近处障碍 → 减速/停车等待"比强行绕行安全
**轨迹穿地面/穿 updating range 外（分析，未实施）**：
- 穿地面：MINCO 只受 vPolytopes 走廊约束，**z 方向无下限**（走廊 z 无障碍就任意）→ 需加 z >= 地面高度(0.1m) 硬约束
- 穿 range 外：走廊生成 box 外的区域"无障碍信息"被当自由空间 → MINCO 穿未探索区 → 需"未知 = 障碍"（当前 `unk_inflation_en: false`）或走廊范围限制

## ESDF 要不要开（2026-08-14 判断：不要开）
- 配置：`esdf/enable: false`（注释明确"计算开销大，通常关闭"），resolution 0.1，local_update_box [5,5,3]
- **ESDF 对障碍残留无帮助**（ESDF 从占用图派生，占用图残留 ESDF 也残留——同源）
- **CPU 风险大**：3D ESDF 每帧全图更新，fsm_node 45% → 可能 80%+；Orin NX 8GB 边缘机器风险高（FSM 116% 过载 → Replan over time 教训）
- 当前避障已用膨胀层（inflation）提供安全距离概念，ESDF 只是"膨胀的精确化"——对当前架构增益有限
- 若只为"绕行轨迹更精确"，等动态避障链路稳定后可试小局部 ESDF，但那是锦上添花不是当前瓶颈
