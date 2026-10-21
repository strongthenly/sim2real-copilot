# 路径A：外部全局路径引导 — 实现细节（2026-08）

## 目标与验证结果
- 飞机除非必须避障绕行，否则一直沿录制轨迹（全局路径）飞
- 地面验证已通：`GLOBAL PATH received: 705 pts`、`Using FULL GLOBAL PATH, idx=55 total=650 pts`、RViz 绿色轨迹线（MarkerArray）
- 遗留：`GeneratePolytopeFromLine failed` 走廊失败（需开 raycasting 清地图）

## 数据流
```
guidepoints_new.txt (705点 CSV x,y,z,qx,qy,qz,qw)
  → global_path_publisher.py (min_pt_dist=0 全发, 不抽稀!)
  → /global_path (nav_msgs/Path, frame=map) + /global_path_marker (MarkerArray LINE_STRIP 绿线 8cm)
  → fsm_ros1.hpp globalPathCallback → planner_ptr_->setGlobalPath(path, roll_dist)
  → super_planner.cpp generateExpTraj 注入块 → guide_path = 全局路径全程段
  → SearchPolytopeOnPath 走廊 → MINCO → 飞行
```

## 代码改动点（SUPER 仓库 dev/4uav，全部有 .bak_globalpath 备份）

### fsm_ros1.hpp
- 成员：`ros::Subscriber global_path_sub_;`
- init()：`global_path_sub_ = nh_.subscribe("/global_path", 1, &FsmRos1::globalPathCallback, this);`
- 回调（waypointCallback 后）：
```cpp
void globalPathCallback(const nav_msgs::PathConstPtr &msg) {
    if (!planner_ptr_) return;
    rog_map::vec_Vec3f path_pts;
    for (const auto &ps : msg->poses)
        path_pts.emplace_back(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    if (path_pts.empty()) { planner_ptr_->clearGlobalPath(); return; }
    double roll_dist = 5.0; nh_.param("global_path/roll_dist", roll_dist, 5.0);
    planner_ptr_->setGlobalPath(path_pts, roll_dist);
    cout << GREEN << " -- [Fsm] GLOBAL PATH received: " << path_pts.size() << " pts, roll_dist=" << roll_dist << " m" << RESET << endl;
}
```

### super_planner.h
- 成员：`rog_map::vec_Vec3f global_path_; std::mutex global_path_mtx_; bool global_path_valid_{false}; double global_roll_dist_{5.0}; bool global_used_prev_{false}; bool global_skip_this_round_{false};`
- public：setGlobalPath / clearGlobalPath / globalPathValid

### super_planner.cpp — generateExpTraj 注入块（"路径A" 注释段，在 Second Part 之前）
1. 找最近索引：全路径线性扫描 `(global_path_[i] - robot_state_.p).norm()` 取最小
2. 取全程段：`sub_path` 从 nearest_idx 到末尾全收
3. 补 goal：`if ((sub_path.back() - gi_.goal_p).norm() > cfg_.resolution*5) sub_path.push_back(gi_.goal_p);` → 使 Second Part 走 `(guide_path.back()-goal).norm() < res*5` 的 NO NEED 分支，不再 A* 续搜
4. guide_stamp：按 max_vel 均匀分配 `cum_t += dist / max(max_vel, 0.1)`
5. `guide_path_end_vel = 0.0;`
6. 日志节流：`global_used_prev_` 只在首次打印

### fallback（走廊失败处，SearchPolytopeOnPath 返回 false 时）
```cpp
if (global_used_prev_) {
    warn("Global path blocked by obstacle, fallback to A* replan");
    global_skip_this_round_ = true;   // 本轮跳过, 不永久禁用
    global_used_prev_ = false;
    guide_path.clear(); guide_stamp.clear();
    if (PathSearch(robot_state_.p, gi_.goal_p, cfg_.planning_horizon, new_path) && new_path.size()>=2) {
        guide_path = new_path; /* 重建时间戳 */
        bool_ret = cg_ptr_->SearchPolytopeOnPath(...);  // 走廊沿 A* 绕行路径
    } else return FAILED;
}
```
注入块开头判断：`if (global_path_valid_ && !global_path_.empty() && !global_skip_this_round_)`；注入块尾部 `global_skip_this_round_ = false;`（每轮重置 → 障碍消失自动恢复沿轨迹）。

## global_path_publisher.py 要点
- `min_pt_dist = rospy.get_param("~min_pt_dist", 0.0)` — **0=不抽稀全发**（全局路径要保真）
- 发布 `/global_path`（Path, latch）+ `/global_path_marker`（MarkerArray LINE_STRIP, scale.x=0.08, 纯绿, ns="global_path"）
- 坑：`geometry_msgs.msg.Point()` 写法错误 → 已 import `Point` 直接用 `Point()`；同名节点冲突需先清理旧进程
- 默认文件：`data_guidepoints/guidepoints_new.txt`，frame_id=map，周期 1Hz 重发

## real_flight.yaml 参数
```yaml
global_path:
  roll_dist: 5.0      # 滚动取段长度(m) — 注意当前实现取全程, roll_dist 仅存档
```

## 验证命令
```bash
# 发布器
cat log/global_path_publisher.log                    # 应含 "发布 N 个航点"
rostopic echo /global_path -n 1 --noarr | grep length  # 应 705+ 点
# FSM 收到
grep "GLOBAL PATH" log/real_flight.log | tail -2     # GLOBAL PATH received: 705 pts
# 规划沿轨迹
grep "FULL GLOBAL PATH" log/real_flight.log | tail -2
# RViz 订阅
rostopic info /global_path_marker | grep -A3 Subscribers  # 应含 rviz
```
