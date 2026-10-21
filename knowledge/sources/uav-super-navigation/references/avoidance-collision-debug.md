# SUPER 避障防撞调试实录（2026-08-14 晚）

## 1. "卡死"崩溃完整堆栈（fsm_node 崩溃特征）

人多次穿行 → 日志尾部出现：

```
#12 Object "/opt/ros/noetic/lib/libroscpp.so", in ros::CallbackQueue::callOne
#9  Source ".../super_core/fsm.cpp", line 77, in callReplanOnce
        >77: RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
#8  Source ".../super_core/super_planner.cpp", line 191, in ReplanOnce
        >191: RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
#7  Source ".../super_core/super_planner.cpp", line 845, in generateExpTraj
        >845: bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, ...);
#6  Source ".../super_core/corridor_generator.cpp", line 113, in SearchPolytopeOnPath
        >113: throw std::runtime_error("seed line too long");
first: 1.124782 -1.420199 0.555561
 second: 0.000000 0.000000 0.750000
 seed line max: 1.2
#0  Source "../sysdeps/unix/sysv/linux/raise.c", line 51, in raise
Aborted (Signal sent by tkill() ...)
```

**判读要点**：
- `throw` 未捕获 → `terminate()` → 进程 abort → fsm_node 消失 → 无 pos_cmd → bridge 超时悬停
- 触发条件：A* 绕行路径相邻点距离 > `seed_line_max_length_ * 1.5`（1.2×1.5=1.8m）
- 为什么 A* 会出长段：fallback 时 A* 从当前位置搜到 goal，路径稀疏化后可能出现大跳点

**修复代码**（corridor_generator.cpp SearchPolytopeOnPath 内，~109 行）：

```cpp
// 长段插值: 相邻点距离 > seed_line_max_length_ 时中间插点(每段≤max)
if ((path[first_id] - path[second_id]).norm() > seed_line_max_length_) {
    const double seg_len = (path[second_id] - path[first_id]).norm();
    const int n_split = static_cast<int>(std::ceil(seg_len / seed_line_max_length_));
    const Vec3f step_dir = (path[second_id] - path[first_id]) / n_split;
    for (int k = 0; k < n_split; k++) {
        seed_lines.emplace_back(path[first_id] + step_dir * k,
                                path[first_id] + step_dir * (k + 1));
    }
} else {
    seed_lines.emplace_back(path[first_id], path[second_id]);
}
```

**验证**：`grep -c 'seed line too long' real_flight.log` = 0；`ps aux | grep fsm_node` 存活；AVOID_WAIT 循环正常恢复。

## 2. 防撞三层配置组合（2026-08-14 晚实测）

| 层 | 参数 | 值 | 防什么 | 机制 |
|---|---|---|---|---|
| 未知区 | `frontend_in_known_free` | true | 撞墙（雷达范围外） | A* `UNKNOWN_AS_OCCUPIED`，只搜已知自由区 |
| 已知障碍 | `robot_r` | 0.35（桨0.23+余量） | 撞障碍物 | CIRI 走廊要求 seed line 离障碍 ≥ robot_r，超近 return FAILED |
| 已知障碍 | `penna_pos`（主轨迹） | 1e7 | MINCO 穿走廊 | 位置惩罚软约束加大，守 vPolytopes |
| 已知障碍 | `inflation_step` | 2（空间窄）/3（空间够） | A* 路径贴障碍 | A* 搜索膨胀距离 |
| 已知障碍 | `corridor_bound_dis` | 0.39/0.6 | 轨迹偏离走廊 | 走廊搜索盒/边界 |

**矛盾与取舍**：
- inflation 大（0.6m）→ A* 路径远但**室内小空间走廊扩不出** → fallback 全失败 → 乱转
- robot_r 大 → 走廊更瘦但安全（硬约束，扩不出就 FAILED=停车，不撞）
- **空间够：inflation 3 + robot_r 0.35 双保险；空间窄：inflation 2 + robot_r 0.35 + penna_pos 1e7 保底线**

## 3. 乱转三种形态区分

| 形态 | 日志特征 | 根因 | 修复 |
|---|---|---|---|
| 崩溃型 | 进程死 + 栈帧 | seed line throw（坑33） | 长段插值 |
| 边界型 | `Using FULL` idx 正常但 SearchPolytope failed 爆发 | frontend=true 未知边界抖动（坑36） | 全局地图稳定边界 |
| 残局型 | `idx=N total=1` | 航迹末端 + 终点判定 0.1m 太严（坑38） | 终点判定 0.1→0.3 |

## 4. 参数语义速查（防再翻车）

- `p_miss` = 光线穿过格子的**目标概率**，必须 < `p_free`(0.4999)，否则清除失效全屏障碍。**别动它**（0.49 已是最优贴边值）
- `virtual_ground_height` = 配置值会被量化钳制，**以日志 `[ProbMap] virtual_ground_height: X` 为准**（-0.9→+0.15 地面被拒；-3.0→-1.95 可用）
- `fix_map_origin` z 偏移 = 坐标系整体抬升，错配 = 全屏障碍（1.5 是 odin 正确值，别改 0）
- `robot_r` 是 CIRI 唯一硬安全约束入口；`penna_pos` 是 MINCO 软约束（可能穿）
- `frontend_in_known_free` 是 A* 的"未知=障碍"开关（撞墙 vs 乱跳的权衡）

## 5. 测试要点

- 静态测试（飞机架上）：验证"规划轨迹"看 RViz 绿线，不是看飞机动没动
- 绕行方向偏好（总走左边/右边）= A* 邻居扩展顺序 + 该方向未知/障碍分布，先查该方向是否已知
- 撞墙 vs 撞障碍物是两个独立问题：未知区（frontend）vs 已知障碍距离（robot_r/inflation）
- 每次改 1 个参数 → 重启按钮3 → 验证；2-3 轮无效 → 回退 `.bak_odintopic` 锚点重来
