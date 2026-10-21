# SUPER guide_path 注入改造（全局路径引导 + 局部规划）

> 2026-08-10 会话末产出。用户方向：**全局路径 = 飞一遍录制的轨迹**（人工飞行=人工全局规划），注入 SUPER 的 guide_path，让 SUPER 沿轨迹做走廊+MINCO 局部规划+动态避障。设计完成 + 完整实现步骤。**✅ 2026-08-11 已完整实施并编译通过**（见文末"最终实施版"节）。用户强调：**飞行中必须避动态障碍物**——避障靠 SUPER 每次 replan（10Hz）重建走廊 + 走廊失败回退 A* 双保险，见"动态避障设计"节。

## 方案 B 澄清（用户确认，2026-08-10 会话末）

- 用户明确：**方案 B（外部节点/航点流折中）当初是"写在 SUPER 的 fsm.cpp 中读取航点文件来分段发送"**——即改 FSM 内部读航点文件、分段发 goal
- 但**当前代码无痕迹**：SUPER 当前分支 `dev/4uav`，fsm.cpp 仅 230 行，无 ifstream/航点文件读取逻辑；task_run_scripts 全仓库搜也无（只有 slam_to_mavros 写 drone_pose.txt、realman_monitor 写日志等无关命中）。**可能已回退 / 在别的副本试过**
- **教训**：用户试过"FSM 读文件分段发送"（方案 B）后觉得不行，才转向"注入 guide_path"（方案 A）——**下次继续此话题不必再问方案 B 细节，直接按方案 A 推进**；若用户再提方案 B，先确认现象（如悬停后不恢复前进 → A 需加"障碍消失后自动继续沿轨迹"恢复逻辑）

## 为什么不需要外部全局 A* 节点

- 用户明确：全局路径来源 = 飞一遍飞机记录轨迹（人工飞行 = 人工全局规划）
- 不需要写 global_planner_node 复用 Astar——直接把录制轨迹注入 SUPER 内部即可

## SUPER 内部流程（源码定位，super_planner.cpp）

```
PlanFromRest(goal_p, goal_yaw, new_goal)          # 69 行
  ├─ getNearestCellNot(机器人位置→空闲)             # 95 行
  ├─ generateExpTraj(...)                          # 108 行
  │    ├─ vec_Vec3f guide_path;                    # 386 行 ← 局部变量，注入点
  │    ├─ ...A* 生成 guide_path (1108-1171 行)：
  │    │    astar_ptr_->escapePathSearch(...)      # 1132 行（起点被困时脱困）
  │    │    astar_ptr_->pointToPointPathSearch(...) # 1157 行（主搜索）
  │    └─ cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, ...)  # 710 行 ← 沿路径生成凸走廊
  │         └─ OptimizationExpTrajInPolytopes(...)  # 754 行 ← MINCO 在走廊内优化
  └─ generateBackupTraj(...)                       # 备份轨迹（planFromRest 失败兜底）
```

**注入原理**：在 A* 调用之前，若 `global_path_`（外部注入的录制轨迹）非空，直接用 `global_path_` 作为 guide_path（跳过 A*）→ SearchPolytopeOnPath 沿它生成走廊 → MINCO 优化。动态避障保留：走廊每次 replan 重建，地图实时更新。

## 改动点（4 处，全部在 SUPER 源码，实施时逐处备份+diff）

| 文件 | 改动 |
|---|---|
| `include/ros_interface/ros1/fsm_ros1.hpp` | 加 `ros::Subscriber global_path_sub_;`（订阅 `/global_path`, nav_msgs/Path）+ 回调存成员 |
| `include/super_core/super_planner.hpp` | 加成员 `vec_Vec3f global_path_;` + `setGlobalPath()` setter（带 mutex 保护） |
| `src/super_core/super_planner.cpp` generateExpTraj | A* 前检查：`if (!global_path_.empty()) { guide_path = global_path_; } else { ...A* 原逻辑... }` |
| `fsm_ros1.hpp` init() | 注册订阅；把全局路径转成 vec_Vec3f 传给 planner |

## 待用户确认的 3 个设计点（2026-08-10 会话末状态）

1. **障碍处策略**：
   - A：局部绕行后回归轨迹（推荐——SUPER 自带 A* fallback + backup traj 机制，绕过后回到轨迹）
   - B：只在轨迹附近小幅调整，不绕行，靠近障碍减速/悬停（**用户确认当初在 fsm.cpp 里试过读文件分段发送=方案 B，且已放弃**——见上文"方案 B 澄清"）
   - **已定方向：按 A 设计**，且用户强调动态避障必须保留
2. **滚动跟随范围**：轨迹几百米，SUPER 是滚动规划——每次只看前方 5~8m（参数 planning_horizon / safe_corridor_line_max_length=5.0），飞过的丢弃。实现：`findNearestIndex(robot_state_.p, global_path_)` 找当前位置在轨迹上的最近点，取前方 `roll_dist` 米子段作 guide_path
3. **轨迹点密度**：40Hz 录制 900+ 点太密，注入前按路径长度降到 ~0.3m 间隔（100-200 点）——工程处理不是改轨迹

## 完整实现步骤（2026-08-10 会话末产出，含动态避障设计）

### 改动文件清单（5 处，全备份+diff）

| # | 文件 | 改动 |
|---|---|---|
| 1 | `fsm_ros1.hpp` | 加 `global_path_sub_`（订阅 `/global_path`，nav_msgs/Path）+ 回调存路径 |
| 2 | `fsm.cpp` 或回调 | 收到路径 → `planner_ptr_->setGlobalPath(...)` |
| 3 | `super_planner.hpp` | 加成员 `vec_Vec3f global_path_` + `setGlobalPath()`（mutex 保护） |
| 4 | `super_planner.cpp` generateExpTraj | **核心注入**：A* 前 `if (!global_path_.empty())` 用全局路径作 guide_path（滚动取当前位置前方段），否则走原 A* 逻辑 |
| 5 | `real_flight.yaml` | 加参数：`global_path/roll_dist`（滚动取段长度）、`global_path/min_dist`（路径点间距） |

### 注入逻辑伪代码（super_planner.cpp generateExpTraj 开头）

```cpp
if (!global_path_.empty()) {
    // ① 滚动: 从当前位置找 global_path_ 上最近点 index
    int idx = findNearestIndex(robot_state_.p, global_path_);
    // ② 取前方 roll_dist 米的子路径作 guide_path
    guide_path = sliceGlobalPath(global_path_, idx, cfg_.global_roll_dist);
    guide_stamp = generateStampBySpeed(guide_path, cfg_.max_vel);
    using_global_path_ = true;
} else {
    ...原逻辑（热启动 or A* PathSearch）...
}
```

### 🛡️ 动态避障设计（用户强调重点）

| 场景 | 机制 |
|---|---|
| 动态障碍（人走进轨迹） | **天然生效**：每次 replan（10Hz）重建走廊（SearchPolytopeOnPath 基于当前地图），走廊自动避开新障碍 |
| 障碍完全挡住全局路径 | **必须加 fallback**：走廊生成失败 → 本次丢弃全局路径 → 走原 A* 绕行 → 下一轮 replan 重新用全局路径（不清 global_path_，只临时跳过） |

```cpp
if (!global_path_.empty()) {
    guide_path = 全局路径滚动段;
    if (SearchPolytopeOnPath(guide_path) 失败) {
        ROS_WARN("Global path blocked, fallback to A*");
        global_path_.clear();          // 本次不用全局路径
        ret = PathSearch(start, goal, horizon, guide_path);  // 原 A*
    }
}
```

效果：正常沿轨迹飞（走廊+MINCO）→ 遇动态障碍走廊重建自动避 → 避不开 A* fallback 绕行 → 绕过下一轮 replan 自动回到全局路径。

### 外部数据源：录制轨迹 → /global_path

新写小节点 `global_path_publisher.py`（~50 行，放 task_run_scripts/scripts/）：读 `guidepoints_new.txt` → 0.3m 抽稀（工程处理）→ 发布 nav_msgs/Path 到 `/global_path`（可循环或触发发布）。

### 实现顺序（4 步，每步可验证）

```
步骤 1：global_path_publisher.py（外部，无风险，写完 RViz 可见轨迹=验证数据源）
步骤 2：super_planner.hpp/cpp 加成员 + setGlobalPath
步骤 3：fsm_ros1.hpp 订阅 + 回调转发
步骤 4：generateExpTraj 注入 + fallback + 滚动取段
编译: catkin_make -j4 -DROS_EDITION=ROS1 (Jetson ~20min)
```

### 地面验证（不装桨）

1. 启动 FAST-LIVO2 + SUPER + global_path_publisher
2. RViz 确认 `/global_path` 显示轨迹（蓝色粗线）
3. 拨杆触发 → 看 frontend_path 沿轨迹走
4. **动态避障测试**：纸箱放轨迹前方 → 看走廊/轨迹自动绕开（日志 SearchPolytopeOnPath 或 A* fallback）
5. 移开纸箱 → 轨迹恢复沿原路径

## 避障分支兼容性（用户问"路径A会不会损坏避障分支"，2026-08-10 验证 ✅）

- **两个改动互不触碰**：避障分支 `feat/obstacle-to-px4` 改的是 **task_run_scripts 仓库**（obstacle_to_px4.cpp、mid360.yaml、launch/obstacle_to_px4.launch、CMakeLists 等）；路径 A 改的是 **SUPER 仓库**（fsm_ros1.hpp、super_planner.hpp/cpp）——不同 git 仓库，零重叠
- **验证方法（可复用）**：改 SUPER 源码前先跑 `git diff --name-only main..feat/obstacle-to-px4 | grep -iE "super|fsm|planner|astar"`——输出空 = 避障分支没碰 SUPER，放心改
- 避障链（PX4 层 OBSTACLE_DISTANCE）与路径 A（规划层 guide_path）数据流独立：一个管飞控响应，一个管规划路线，可同时工作
- 唯一注意：global_path_publisher.py 放 task_run_scripts/scripts/ 是**新增文件**，不改任何现有文件，不影响避障分支

## ROG-Map 概率地图刷新频率（2026-08-10 实机核对，动态避障性能依据）

- **更新定时器 1000Hz**（`createTimer(ros::Duration(0.001), updateCallback)`）——只是轮询，不是每 1ms 都重算
- **实际地图更新频率 = 点云频率**（updateCallback 逻辑：`if (unfinished_frame_cnt == 0) return;` 有新帧才 updateProbMap）→ 实测 /cloud_registered 9.8Hz → **地图刷新 ≈ 9.8Hz（每 0.1s 一帧）**
- **RViz 可视化是单独定时器**（`createTimer(1.0/viz_time_rate, vizCallback)`）——看到的"消失延迟"一部分是可视化节流，主要是概率衰减
- **人走后障碍块消失时间估算**：占用概率从 >0.80 衰减到 <0.30，p_miss=0.70，每帧衰减一步 → **约 5~15 帧 = 0.5~1.5 秒**才显示消失
- **对动态避障的影响**：新障碍（人）0.1s 内进入地图 → SUPER replan(10Hz) 感知；障碍离开后 0.5~1.5s 残留 → 短暂保守绕行（安全方向）。**结论：感知快、残留短，动态避障天然成立，无需为它改代码**。别乱调 p_free/p_hit（调太快会误清静态障碍）

## 坐标系事实（2026-08-10 实机核对）

- `map → camera_init` 静态变换 = 零偏移（0,0,0）→ 飞控 map 系与 SLAM camera_init 系同原点 ✅
- `/aft_mapped_to_init`：frame_id="camera_init" → child_frame_id="aft_mapped"（动态 TF）
- `/cloud_registered` 点云 frame = "camera_init"
- **livox_frame 在 TF 树中不存在**（FAST-LIVO2 不发布它）——别在 launch 里做 livox_frame→aft_mapped 静态变换，会失效
- 坐标系健康检查三连（起飞前跑）：`rosrun tf tf_echo map camera_init`（≈0）＋ 对比 `/aft_mapped_to_init` 与 `/mavros/local_position/pose` 的 position（应重合）＋ `rostopic echo /obstacle_to_px4/status`（min_dist 应 >1m，<0.3m 说明有残留障碍块或机架遮挡）

## 相关文件（机上）

- 使用手册：`task_run_scripts/guide_Readme.md`
- 排障手册：`task_run_scripts/troubleshooting_Readme.md`（含编译 ROS_EDITION 坑、避障链、MINCO NaN、拨杆、git 操作）
- 当前分支：feat/obstacle-to-px4（本地修改 stash，快照 ~/task_run_scripts_backup_20260810.tar.gz）

## ✅ 最终实施版（2026-08-11 实机，全部编译通过 0 error）

### 关键决策：用户要求"除非必须绕行，否则全程沿全局路径走"

**第一次注入用了"前 5m 滚动取段"（roll_dist=5），用户否了**——因为 5m 之外会走 A* 续搜、可能偏离轨迹。改为**全程取段**：

```
注入块逻辑（generateExpTraj 的 Second Part 之前）:
  if (global_path_valid_ && !global_path_.empty() && !global_skip_this_round_) {
      nearest_idx = 找当前位置在 global_path_ 上的最近点（线性扫 min_dis）
      sub_path = global_path_[nearest_idx .. 末尾]      ← 全程，不是前 5m
      if ((sub_path.back() - gi_.goal_p).norm() > res*5) sub_path.push_back(goal)
      guide_path = sub_path
      guide_stamp = 按 max_vel 均匀时间戳
      guide_path_end_vel = 0.0
  }
  global_skip_this_round_ = false;   // 每轮 replan 重置（注入块尾部）
```

**为什么全程取段就能"一直沿轨迹"**：Second Part 原有逻辑（635-660 行）检查 `(guide_path.back() - goal).norm() < res*5` → 真则直接补 goal 结束（NO NEED 分支），**不再 A* 续搜**。全程取段后 guide_path 末尾就是轨迹终点≈goal → 走 NO NEED → 走廊沿整条轨迹生成。若只取 5m，back() 离 goal 远 → 走 else 分支 A* 续搜 → 偏离轨迹。

### fallback（障碍绕行，用户强调动态避障必须保留）

走廊生成 `SearchPolytopeOnPath` 失败处（原 756-758 行 `return FAILED`）注入：

```
if (!bool_ret_code) {
    if (global_used_prev_) {   // 本轮用了全局路径且被障碍挡住
        global_skip_this_round_ = true;   // ← 临时跳过（不能 global_path_valid_=false，否则永久禁用）
        guide_path.clear();
        if (PathSearch(robot_state_.p, gi_.goal_p, planning_horizon, new_path) && new_path.size()>=2) {
            guide_path = new_path; 重建 stamp;
            SearchPolytopeOnPath(guide_path, sfc, ...)  // 重试走廊
            if 成功 continue; else return FAILED;
        } else return FAILED;
    } else return FAILED;
}
```

- **核心成员**：`global_skip_this_round_{false}`（super_planner.h）——每轮 replan 结束重置，所以障碍绕行只影响"这一轮"，下一轮自动恢复沿全局路径
- **日志判读**：`Global path blocked by obstacle, fallback to A* replan (temporarily)` → `A* fallback path corridor OK`（绕行成功）；`A* fallback corridor also failed`（罕见，绕行也失败）

### 编译验证（2026-08-11）

- `catkin_make -j4 -DROS_EDITION=ROS1` → `grep -c "error:"` = 0，`[100%] Built target fsm_node` ✅
- **注意查日志文件**：两次编译用不同日志名（globalpath.log vs globalpath2.log），查错文件会看到旧错误——确认时间戳
- 括号平衡：改前原文件就差 1（固有），比较"改前改后差值"而非绝对平衡

### 配套改动（同天完成）

- **按钮3**（NX_code feat/4uav-guide 提交 cffadd1 + **d7d573b**）：command 先启动 global_path_publisher 再 sleep 1 再启动 real_flight.launch。**⚠️ `& &&` 坑（cffadd1 实测踩坑，d7d573b 修复）**：`nohup cmd & && sleep 1 && nohup cmd2 &` 会让 nx_command_server 报 `❌ 指令执行出错: timed out`、什么都不启动——**修复为子括号包裹：`(nohup cmd &) && sleep 1 && (nohup cmd2 &)`**（子括号让 `&` 只作用于子 shell，外层立即返回，`&&` 链正常）。改完提交后必须重启 nx_command_server 才生效
- **RViz**（feat/4uav-guide 提交 b8c2b3a + **174085f 修复 Color 格式**）：top_down.rviz 加 GlobalPath 显示（rviz/Path 类，Topic /global_path，绿色粗线）——插入锚点：Goal 显示项块（Name: Goal ... Value: true）之后、Points 块之前
  - **⚠️ RViz Path 显示项 Color 格式坑（174085f 修复，2026-08-11 实机）**：`Color: 0, 255, 0`（逗号分隔字符串）**不是 RViz 标准格式，Path 显示项会解析失败→不订阅话题→绿线不显示**。正确格式是分号 `Color: 0; 255; 0`。**症状**：`rostopic info /global_path` 的 Subscribers 列表里只有 /fsm_node、**没有 rviz**（话题有数据、FSM 收到 705 点、但画面无绿线）
  - **⚠️⚠️ 最终根因（比 Color 更深；Color 修了仍不显示才暴露，2026-08-11 实机）**：**top_down.rviz 整个文件是"压缩单行式"非标准格式**——每行 `- Alpha: 0.5      Cell Size: 1      Class: rviz/Grid` 所有字段用多空格挤一行（老版本 RViz/别处工具生成）。**python yaml.safe_load 解析它时 `Displays` 数量 = 0**（键层级/结构不标准）→ 用 python 标准 YAML 多行块**混插**进这种文件 = RViz 解析错乱，显示项全部静默不生效（不只是 GlobalPath 失效）。**诊断命令（必跑）**：`python3 -c "import yaml; d=yaml.safe_load(open('rviz/top_down.rviz')); print(len(d['Visualization Manager']['Displays']))"`——输出 0 或 KeyError = 文件格式非标准，**任何 python 局部编辑都救不了，必须整体重建**
  - **✅ 根治**：用 python `yaml.dump` **整体重建标准格式 top_down.rviz**（9 个显示项：Grid/GlobalCloud(cloud_registered)/LocalCloud/Occ(rog_map/occ)/Waypoints(mkr)/GlobalPathMkr(/global_path_marker)/Goal(fsm goal)/FrontendPath(frontend_path)/ExpTraj(exp_traj)，Fixed Frame=map，TopDownOrtho 视图），旧文件备份 `.bak_oldformat`。重建后重启 SUPER（按钮2→按钮3）让 RViz 加载新文件
  - **✅ MarkerArray 兜底（最终生效的显示方案）**：global_path_publisher.py 额外发布 `/global_path_marker`（visualization_msgs/MarkerArray，LINE_STRIP 绿色线宽 0.08，ns=global_path，latch 常发），RViz 用 `rviz/MarkerArray` 显示项订阅（Waypoints 同款，已验证稳定）。**经验：这台 RViz 环境 MarkerArray 显示项远比 Path 可靠，新轨迹线一律用 MarkerArray 发**
  - **诊断法（可复用）**：RViz 显示项没生效时，先 `rostopic info <topic>` 看 rviz 在不在订阅者列表——不在 = 显示项解析失败（查 Color/字段格式），在但无画面 = 话题/坐标问题。改完 top_down.rviz 后需**重启 SUPER（按钮2→按钮3）让 RViz 重新加载配置**
- **global_path_publisher.py**（task_run_scripts/scripts/）：读 guidepoints_new.txt → nav_msgs/Path frame=map，**min_pt_dist 默认 0.0（不抽稀，全发）**，latch + 1Hz 周期重发（航迹更新自动生效）。**⚠️ 全局路径不抽稀是用户 2026-08-11 明确纠正的**（"为什么还是会抽稀，我传输的是全局路径"）：点越密轨迹保真度越高，SUPER 内部滚动取段+走廊生成+MINCO 自己处理点密度；抽稀是 waypoint_mission 航点任务的事，不是全局路径的事。第一次实机 705 点被 0.3m 抽稀成 7 点（RViz 只剩一小段），改 0.0 后 743 点全发正常

### 测试实机发现（2026-08-11 按钮3 首次测试）

- **按钮3 正常预期输出清单**（按按钮3 后逐项核对）：① `log/global_path_publisher.log` 出现（`发布 N 个航点`）② `log/real_flight.log` 更新（`GLOBAL PATH ENABLE` → `GLOBAL PATH received: N pts, roll_dist=5 m` → `Wait for goal`）③ 4 进程在跑（fsm_node/super_to_px4_bridge/waypoint_mission/global_path_publisher）④ RViz 绿线。**卡在 `Wait for goal` 是正常**（等拨杆触发）；`GLOBAL PATH ENABLE` 有但无 received = 发布器没起/崩了
- **同名节点顶替杀新实例**：残留旧发布器 + 新按钮3 → 日志 `new node registered with same name`，新实例随后 FileNotFoundError 退出。**重复启动前先 pgrep 清残留**
- **分支切换带走 data_guidepoints/**：切回避障分支后该目录消失 → 发布器 FileNotFoundError（`data_guidepoints/guidepoints_new.txt`）。恢复：`git checkout feat/4uav-guide -- data_guidepoints/guidepoints_new.txt`（只取文件）。**测路径 A 前先 ls 确认航迹在**
- **MAVROS 挂 → 飞控 HOLD 是安全机制**：MAVROS 进程没了 → PX4 检测到 OFFBOARD 指令流丢失 → 自动回退 HOLD。判 MAVROS 挂：`/mavros/state` 无输出 + 所有 /mavros/* 话题空。修法：重启 start_fastlivo2.sh。**地面待机模式常见 AUTO.LOITER**（RC 模式开关位置），用户界面翻译成"Hold/悬停"易误会——连续采样 `/mavros/state -n 5` 看是否真跳变
- **切 POSCTL 后自动回 HOLD 的定位问题（2026-08-11 后半段，详见 SKILL.md 坑 17/20）**：
  - 症状：`AUTO.LOITER` 地面待机正常，切到 position（POSCTL）后过一段时间自动切回 Hold；MAVROS 报 `navigation error: No valid global position estimate`；QGC 无 EKF2_AID_MASK 参数（微空 nxtpx4v2 定制固件，非标准 PX4 参数体系）
  - **PX4 1.17 参数体系换代**：`EKF2_NOAID_MASK` 已不存在（`PR: Unknown parameter to get`）；视觉融合开关是 `EKF2_EV_CTRL`（实测 = 11 已开），旧 `EKF2_AID_MASK` 读 0 只是废弃默认值——**别盯 AID_MASK**
  - **global vs local position 区分**：报错说的是缺 GLOBAL（经纬度，必须 GPS），不是缺 local（视觉/IMU 能给）；`POSCTL` 只需 local，`AUTO.*` 系列才要 GPS
  - **FAST-LIVO2 视觉从未工作（VIO 断）**：`MID360_Fly.yaml` 的 `img_en: 0` + `img_topic: /usb_cam/image_raw`（D435 实际发 /camera/color/image_raw）→ `/rgb_img` 空、定位纯 LiDAR+IMU。这可能是"定点保持不住"的根因之一（纯 LiDAR 定位运动后漂移）
  - **两天日志对比法**：`log/fastlivo2_20260811_120800_true/` vs `..._162056_false/` 的 fastlivo.log 里 img/vision 行数均为 0 → 两天视觉状态一致，排除"今天视觉断线"假设

### ⭐ 方案 C 最终收敛（2026-08-11 实飞后，保留 CH10 门禁 + 纯全局路径）

**背景**：方案 A（`use_waypoint:=false` 禁用 waypoint_mission）实测发现**丢失 CH10 触发门禁**——global_path_publisher 若直接发布 `/waypoint_goal`，FSM 的 waypointCallback 收到就 setGoalPosiAndYaw（**CH10 检查只存在于 waypoint_mission 内部**），导致"没按 CH10 无人机自己开始循迹"。**方案 C = 保留 waypoint_mission（门禁）+ 只发终点 + 全程全局路径**，三件事：

1. **生成 `data_guidepoints/endpoint.txt`**：从 guidepoints_new.txt 取**最后一点**（python 读 CSV 取 pts[-1]），空格格式 `x y z`（waypoint_mission 支持空格格式，与 real_mission.txt 同）
2. **按钮3 改回 `use_waypoint:=true waypoint_data_path:=.../endpoint.txt`**：waypoint_mission 读终点文件 → CH10 下降沿触发 → 发终点 goal（提交 8f816a3）
3. **global_path_publisher.py 移除 /waypoint_goal 发布**：只保留 /global_path + /global_path_marker，让 waypoint_mission 独占 goal 通道（避免绕过 CH10）

**最终链路**：`global_path_publisher→/global_path(完整轨迹)` ＋ `waypoint_mission(endpoint.txt)→CH10 触发→/waypoint_goal(终点)` ＋ `FSM: goal=终点, guide_path=完整轨迹→沿轨迹飞`。

**验证**：未拨杆时日志无 `Mavros triggered`（门禁生效）；拨杆后 `Pub goal` 只出现 1 次（终点）；`ps aux | grep waypoint_mission` 在跑。

**判读"门禁被绕过"的铁证**：日志有 `Get goal at` 但**没有 `-- [MISSION] Mavros triggered.`**；且 goal 是固定值（如 0.075,-0.075,0.825）而非航迹终点——即发布器发的 goal 绕过了触发检查。

### 分支状态（2026-08-11 结束）

- `feat/4uav-guide`（主仓库 4 提交 + NX_code 3 提交）：用户修改 + 路径 A 全套
- `feat/obstacle-to-px4`：避障分支原样保留（改前验证 git diff 零重叠）
- SUPER `dev/4uav`：路径 A 源码改动（4 文件 + .bak_globalpath 备份）
- 测试注意：主仓库切到 feat/4uav-guide 才有 GlobalPath rviz 显示 + 正确 waypoint.yaml；NX_code 子模块已指向 cffadd1（按钮3 新版生效）

### ⭐⭐ 会话末实机三连坑（2026-08-11 深夜，全部已修+编译通过）

**① 全局路径点密度崩溃走廊（飞直线根因）**：全局路径 930 点/4.31m = 平均间距 0.46cm → SearchPolytopeOnPath 沿 929 个微段生成走廊 → `Cannot find continuous corridor on path, overlap only -326`（盒子断裂）+ `Box search failed, box size is zero` → 走廊失败 → fallback A* → 无障碍空间 A* 直线到终点 → **飞机飞直线不走航迹转弯**。**判读**：`Using FULL GLOBAL PATH` 生效 + idx 正常推进 + RViz 轨迹直线 → 先 `python` 算航迹文件点间距（总路程/点数），<0.01m 即此坑。**修复（已实施）**：注入块取段后按 `global_path_downsample`（0.1m）降采样——`super_planner.h` 加 `global_path_downsample_{0.1}` 成员 + `setGlobalPath(path, roll_dist, downsample=0.1)` 第三参；`super_planner.cpp` 步骤 3)补终点后插入 3.5) 降采样块（`ds_path` 首点必留、末尾若被吃补回 `sub_path.back()` 保 NO NEED 分支）；`fsm_ros1.hpp` 读 `~global_path/downsample`（默认 0.1）传入；`real_flight.yaml` global_path 段加 `downsample: 0.1`。验证：触发后 `grep "Downsampled" log` 应见 `Downsampled global path: N pts (ds=0.1m)`。

**② 最近点取段跳段（飞直线的另一根因，与 ① 可叠加）**：注入块 `findNearestIdx` 全段线性扫找最近点——航迹起点在原点密集绕圈时，飞机起飞点 (0.08,-0.08) 离**后半段点**更近 → idx 从 131 跳到 775 → 取段 [775→终点] = 后半段直线 → 前半段转弯被跳过。**修复（已实施）**：加 `int global_last_idx_{-1}` 成员；找最近点**单向推进**（`search_from = global_last_idx_ > 0 ? global_last_idx_ : 0`，循环 `i < search_to`）；**首次（last_idx<0）搜索限制前 20% 段**（`search_to = max(size/5, 2)`）→ 触发时从轨迹起点附近开始，之后严格沿轨迹顺序推进。验证：`grep "Using FULL GLOBAL" | head -5` 看 idx 从 ~0 开始而非 775。

**③ 起飞点被障碍包围 / 地面触发无效（走廊失败的最终根因）**：
- **start point in obstacle 源码定位**：super_planner.cpp:1216-1221，PathSearch 开头 `if (start_type == OCCUPIED || OUT_OF_MAP) { warn(...); return false; }`——起点在地图判占用/越界就拒绝。
- **occ 探针实机验证**：`/fsm_node/rog_map/occ` 解析（16B/点 xyz float32），起点 (0,0,0.6) 1m 内 190 占用点、最近 0.44m 垂直障碍面 = **操作员站在飞机前方 0.4m**（人=障碍）。人退到 3m 外再解析 = 0 占用 → 占用源是人不是机架。
- **地面触发无效**：飞机未起飞（pose z≈0.0001）时拨杆 → 起点 z=0 低于地图下界（occ box_min z=0.1）→ 走廊+fallback A* 全部因起点越界失败 → 死循环 `PlanFromRest failed, try replan`。**必须先 OFFBOARD 起飞到 0.6m 悬停稳定再拨杆**。
- 判读：走廊失败 + `Global path blocked` + `The start point in obstacle` + `A* fallback path search failed` = 起点环境问题（人/机架/越界），先 occ_probe 查占用（5 分钟），别动代码。

