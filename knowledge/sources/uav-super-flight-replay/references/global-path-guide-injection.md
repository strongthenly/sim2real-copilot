# 路径 A：SUPER 外部全局路径引导（实现记录，2026-08）

## 目标与选型

用户需求：给 SUPER 一条全局路径（**飞一遍录制的轨迹**），SUPER 沿路径做局部规划 + 动态避障。
- ❌ 抽稀/拐点提取：对死轨迹删点，用户明确否定
- ❌ 外部全局 A* 节点：地图是局部 30×30m + robocentric 滑动，远目标需滚动重规划，复杂度高
- ✅ **改 SUPER 源码注入 guide_path**：走廊沿外部路径生成，动态避障天然保留

## SUPER 内部流程（注入点依据）

```
PlanFromRest → generateExpTraj():
  guide_path (局部变量, super_planner.cpp ~386行)
    ← 热启动(上一段轨迹) 或 PathSearch(A* 全图搜索, ~641行)
  → SearchPolytopeOnPath(guide_path, sfc, ...)  走廊(基于实时地图)
  → MINCO 在走廊内优化
  → ReplanOnce 每 0.1s 重规划 → 地图实时 → 动态避障
```

**注入策略（V2 最终版，2026-08-11）**：generateExpTraj 的 "Second Part"（几何段，`double guide_path_length = ...` 之前）插入全局路径引导块——若 `global_path_valid_`，从当前位置找最近索引、**取全程**（最近点→终点，不截断）、替换 guide_path、清空 guide_stamp 按 max_vel 重新生成时间戳。**关键：若最后一点离目标 > `cfg_.resolution * 5`，补 push 真实终点** → guide_path.back() 贴近目标 → Second Part 走 `(guide_path.back()-gi_.goal_p).norm() < res*5` 的 **NO NEED 分支，不再 A* 续搜** → 走廊沿整条轨迹生成。

**fallback（走廊失败时绕行，用户要求"除非必须切换 A* 否则一直沿全局路径"）**：在 `SearchPolytopeOnPath(guide_path, ...)` 返回 false 处（super_planner.cpp ~756 行 `if (!bool_ret_code)` 分支）注入：
```cpp
if (global_used_prev_) {   // 之前用了全局路径
    global_skip_this_round_ = true;   // 本轮跳过全局路径（不永久禁用）
    global_used_prev_ = false;
    guide_path.clear(); guide_stamp.clear();
    if (PathSearch(robot_state_.p, gi_.goal_p, cfg_.planning_horizon, new_path) && new_path.size()>=2) {
        guide_path = new_path;   // A* 绕行路径
        // 重新生成时间戳 + SearchPolytopeOnPath，成功则继续，失败则 return FAILED
    } else return FAILED;
} else return FAILED;
```
注入块条件加 `&& !global_skip_this_round_`，注入块尾部（else 分支后）**每轮 replan 重置 `global_skip_this_round_ = false`** → 障碍消失后下一轮自动恢复沿全局路径。`super_planner.h` 加成员 `global_skip_this_round_{false}`。

## 改动的 5 个文件

| 文件 | 改动 |
|---|---|
| `SUPER/super_planner/include/ros_interface/ros1/fsm_ros1.hpp` | 加 `global_path_sub_`（订阅 `/global_path`）+ `globalPathCallback`（nav_msgs/Path→rog_map::vec_Vec3f→`planner_ptr_->setGlobalPath()`；读 `nh_.param("global_path/roll_dist")`） |
| `SUPER/super_planner/include/super_core/super_planner.h` | 加成员 `global_path_` / `global_path_mtx_` / `global_path_valid_{false}` / `global_roll_dist_{5.0}` / `global_used_prev_{false}` + `setGlobalPath(path, roll_dist)` / `clearGlobalPath()` / `globalPathValid()` |
| `SUPER/super_planner/src/super_core/super_planner.cpp` | generateExpTraj "Second Part" 前注入引导块（滚动取段 + 时间戳 + 日志节流 global_used_prev_） |
| `SUPER/super_planner/config/real_flight.yaml` | 追加 `global_path: { roll_dist: 5.0 }` |
| `task_run_scripts/scripts/global_path_publisher.py`（新增） | 读 guidepoints_new.txt → 发布 `/global_path`（nav_msgs/Path, frame_id="map", latch + 1Hz 重发）。**⚠️ 默认 `min_pt_dist=0.0` 不抽稀**（2026-08-11 用户纠正：全局路径必须全量保真，抽稀是 waypoint_mission 航点任务的逻辑，套到全局路径会把 705 点抽成 7 点、RViz 无绿线、FSM 收到 `GLOBAL PATH received: 7 pts`；SUPER 自己会滚动取段+走廊生成处理点密度） |

## 动态避障保证（V2 已落盘）

- 走廊 `SearchPolytopeOnPath` 基于**实时地图**：人/障碍进入路径 → 走廊生成失败 → **fallback 块**（见上）用 `PathSearch` A* 绕行（本轮 `global_skip_this_round_` 跳过全局路径）→ 绕行走廊生成成功则继续；每轮 replan 末尾重置 skip → 障碍消失后**自动恢复沿全局路径**
- 日志判读：正常 `-- [SUPER] Using FULL GLOBAL PATH, idx=N total=M pts`；障碍 `Global path blocked by obstacle, fallback to A* replan` + `A* fallback path corridor OK`；罕见 `A* fallback corridor also failed`（force return）
- 地图更新 ~9.8Hz（=点云频率，update_timer 1ms 轮询 + 有新帧才 updateProbMap）；人走后障碍块 0.5~1.5s 消失（p_hit/p_miss 概率衰减）

## 编译注意

- `catkin_make -j4 -DROS_EDITION=ROS1`（livox 双版本，见 SKILL.md 坑#14）
- 注入块**只引用函数早期声明的变量**（guide_path/guide_stamp/robot_state_/cfg_/gi_）——引用后段局部变量（如 last_exp_traj_time_pos）直接编译失败（坑#19）
- 括号平衡差 1 先对比 `.bak_globalpath` 备份，原文件本就如此

## 回滚

每个文件改前已 `cp xxx xxx.bak_globalpath`（4 个备份）。回滚：`cp xxx.bak_globalpath xxx` 后重编。

## 分支状态（2026-08-11 晚，路径 A 已落地编译通过）

- `task_run_scripts`：main / feat/obstacle-to-px4（避障，**cloud_topic 修复已 commit f950ff8 并 push origin**，2026-08-11）/ feat/4uav-guide（用户复飞定制：配置 3 提交 + 文档/航迹 1 提交 + GlobalPath RViz 1 提交 + 按钮3 修复 d7d573b；**未 push**）
- `scripts/NX_code`：feat/4uav-guide（录制归档+按钮3 基础 1 提交 + 按钮3 加 global_path_publisher 1 提交 = cffadd1）
- `SUPER`：dev/4uav + 路径 A 未提交修改（4 文件 + 4 `.bak_globalpath` 备份）；编译 `catkin_make -DROS_EDITION=ROS1` 0 错误，`Built target fsm_node` ✅
- 备份：`~/task_run_scripts_backup_20260810.tar.gz`（拉取前）+ `~/obstacle_branch_20260810.tar.gz`（避障分支）

## 落地配套（2026-08-11 完成）

- **按钮3**（NX_code/User_define_commands.yaml，feat/4uav-guide 分支）：command = source ROS → **`(nohup rosrun task_run_scripts global_path_publisher.py _file:=.../guidepoints_new.txt > .../global_path_publisher.log 2>&1 &)`** → sleep 1 → **`(nohup roslaunch ... real_flight.launch waypoint_data_path:=.../guidepoints_new.txt 2>&1 &)`**。**⚠️ 后台命令必须子括号 `( ... &)` 包裹**——裸 `& &&` 会让 nx_command_server 执行超时（`❌ 指令执行出错: timed out`）什么都不启动（坑#22）。改后**必须重启 nx_command_server**（kill 6004 PID → nohup bash -c 'source ... && exec python3 -u nx_command_server2.py'）
- **RViz**（task_run_scripts/rviz/top_down.rviz，feat/4uav-guide 分支）：**⚠️ 初版用 `Class: rviz/Path, Topic: /global_path`（绿色粗线）实测不工作**（`rostopic info /global_path` 的 Subscribers 里只有 fsm_node 没有 RViz，即使 Enabled+Topic 正确+重启 RViz 也无效；Color 逗号/分号都试过）——**Path 显示项在此环境静默不订阅，最终方案是发布器额外发 `/global_path_marker`（MarkerArray LINE_STRIP，ns=global_path, scale.x=0.08, 绿色）**，rviz 加 `Class: rviz/MarkerArray, Topic: /global_path_marker, Namespaces: {global_path: true}` 显示项（格式抄 Waypoints 项，插入在 Goal 显示块后；rviz 配置 yaml 里 Goal 的 Name 和 Topic 分行写，锚点用 `Name: Goal` + 后续 `Value: true` 整块）。详见 SKILL.md 坑#25
- **测试确认点**：按按钮3 后 `grep "GLOBAL PATH" log/real_flight.log` 应见 `-- [Fsm] GLOBAL PATH received: N pts, roll_dist=5 m`
- **触发不变**：`start_trigger_type: 1` CH10 下降沿（路径 A 不改变触发方式）
