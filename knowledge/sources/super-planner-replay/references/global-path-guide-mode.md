# 路径A：SUPER 全局路径引导（2026-08-11 落地）

## 背景
用户需求：无人机沿录制航迹飞行（类似"飞一遍→复飞"），同时保留避障能力。
不要"抽稀成航点逐点发送"（曾试过方案B，FSM 读航点文件分段发送，被弃），
改为**整条轨迹作为全局路径注入 SUPER 的 guide_path**，走廊沿轨迹生成 + MINCO 优化。

## 数据链（组件）
```
guidepoints_new.txt (CSV 7列: x,y,z,qx,qy,qz,qw)
  → global_path_publisher.py (task_run_scripts/scripts/)
      → /global_path          (nav_msgs/Path, 全量点, 默认不抽稀 min_pt_dist=0)
      → /global_path_marker   (visualization_msgs/MarkerArray, LINE_STRIP 绿线, RViz 显示)
      → /waypoint_goal        (geometry_msgs/PoseStamped, 终点, 纯全局路径模式的 goal)
  → fsm_node (SUPER, dev/4uav 分支)
      → /planning/pos_cmd → super_to_px4_bridge → /mavros/setpoint_raw/local → PX4
```

## SUPER 源码改动（dev/4uav 分支，3 文件 + 1 配置）
1. `include/ros_interface/ros1/fsm_ros1.hpp`
   - 成员 `ros::Subscriber global_path_sub_`
   - init 里 `global_path_sub_ = nh_.subscribe("/global_path", 1, &FsmRos1::globalPathCallback, this)`
   - `globalPathCallback`: nav_msgs/Path → rog_map::vec_Vec3f → `planner_ptr_->setGlobalPath(pts, roll_dist)`；空路径 → clearGlobalPath
2. `include/super_core/super_planner.h`
   - 成员：`rog_map::vec_Vec3f global_path_; std::mutex global_path_mtx_; bool global_path_valid_{false}; double global_roll_dist_{5.0}; bool global_used_prev_{false}; bool global_skip_this_round_{false};`
   - `setGlobalPath(path, roll_dist)` / `clearGlobalPath()` / `globalPathValid()`
3. `src/super_core/super_planner.cpp` — generateExpTraj 注入块（"Second Part of Guide Path" 注释之前）：
   ```
   if (global_path_valid_ && !global_path_.empty() && !global_skip_this_round_) {
       找 global_path_ 上离 robot_state_.p 最近索引 nearest_idx
       guide_path = global_path_[nearest_idx .. 末尾]   // 全程，不截断
       if (guide_path.back() 离 gi_.goal_p 远) guide_path.push_back(gi_.goal_p)
       生成 guide_stamp（按 max_vel 均匀时间戳）
       guide_path_end_vel = 0
   }
   ```
   走廊失败 fallback（SearchPolytopeOnPath 失败处）：
   ```
   if (global_used_prev_) {
       global_skip_this_round_ = true;   // 本轮跳过全局路径（不永久禁用）
       guide_path.clear();
       PathSearch(robot_state_.p, gi_.goal_p, cfg_.planning_horizon, new_path)  // A* 绕行
       重生成走廊；也失败则 return FAILED
   }
   ```
   注入块末尾 `global_skip_this_round_ = false`（每轮重置 → 障碍消失后自动恢复沿全局路径）
4. `config/real_flight.yaml`（task_run_scripts 那份！不是 SUPER 仓库的）追加：
   ```yaml
   global_path:
     roll_dist: 5.0
   ```

## 纯全局路径模式（2026-08-11 晚，修复"乱转"）
**现象**：同时跑 waypoint_mission（逐点发 goal）+ 全局路径（guide_path 全程）→ FSM 在"追逐点 goal"和"沿轨迹"间切换 → 飞机乱转，最终 `close to goal` + `OptimizationExpTrajInPolytopes failed` + `SUPER command timeout` → POS_HOLD。
**修复**：
- 按钮3 command 加 `use_waypoint:=false`（real_flight.launch 已有 arg `<arg name="use_waypoint" default="true"/>`）
- global_path_publisher.py 增加发布**终点**到 `/waypoint_goal`（PoseStamped latch）
- 效果：goal=终点（1 个）+ guide_path=完整轨迹 → 全程沿轨迹飞到终点，无逐点切换

## 实飞故障链（GeneratePolytopeFromLine failed）
```
Using FULL GLOBAL PATH, idx=62 total=1555 pts
GenerateExpTrajectory SUCCESS
GeneratePolytopeFromLine failed, force return
generateBackupTrajectory return [FAILED], force return
PlanFromRest failed, try replan   ← 无限循环
（最终）SUPER command timeout → POS_HOLD（bridge 安全兜底，飞机原地悬停不乱飞）
```
原因：全局路径附近地图有障碍/未知（测试时人站在飞机旁=人即障碍）→ 走廊生成失败。
处置：人远离 2m+ / 起飞前雷达转一圈清地图 / raycasting 开启（见下）。

## 日志判读要点（real_flight.log）
- `Using FULL GLOBAL PATH, idx=N total=M pts` → 全局路径引导生效
- `close to goal and return NONEED` → FSM 认为到达目标（轨迹终点）
- `OptimizationExpTrajInPolytopes for new path failed` → 走廊/MINCO 优化失败
- `SUPER command timeout. Switching to position hold.` → bridge 0.5s 无指令，安全悬停
- `[POS+VEL] x=.. y=.. z=.. vx=.. vy=.. vz=.. yaw=.. deg` → 实际飞行轨迹（对比航迹判断是否沿轨迹）
- 高度突降（z 0.82→0.38）是危险信号，立即接管

## 相关坑
- 起飞点必须对齐航迹起点（±0.3m），否则注入块找最近点会先飞向轨迹中段
- 短距离（0.6m）测试建议：max_vel 2.0→0.3、max_acc 4.0→1.0、velocity_ff_gain 0.7→0.3（防 MINCO 短轨迹 NaN）
- 全局路径点全量发布（不抽稀）：SUPER 内部滚动取段 + 走廊会处理密度；抽稀只用于 waypoint_mission 航点
- global_path_publisher 是 Python 脚本（task_run_scripts/scripts/），改了不需要编译，但**同名节点重启要先杀干净**
