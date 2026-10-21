# 仿真规划死锁实录 + 实机「任务级超时放弃」机制（2026-09-22）

配套 SKILL.md 坑 Q。记录"飞机能飞之后"暴露的两个问题：**最终规划不出来（死锁）** 与 **撞障碍**。

---

## 1. 死锁的完整证据链

### 1.1 SUPER 侧：安全走廊退化

`/tmp/super_fsm.log` 中反复出现的四行组（实测 `GenerateExpTrajectory failed` 52 次，
`BackOpt` 284 次，`FOLLOW_TRAJ` 168 次，`AVOID_WAIT` 仅 7 次）：

```
-- [Fsm] ReplanOnce succeed.
-- [EXPTrajOpt] Ill corridor! Forced return.            ← 走廊退化标志
-- [TrajOpt] Cannot simplify sfcs.
[WARN] -- [SUPER] OptimizationExpTrajInPolytopes for new path failed
[WARN] -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return
```

另一种形态（MINCO 直接判定不可行）：

```
[WARN] -- [ExpOpt] Opt failed, Omg or thr or Pos violation.
-- [MINCO] TrajOpt failed, (unknown)
-- [SUPER] Minco exp_traj opt failed.
[WARN] -- [SUPER] OptimizationExpTrajInPolytopes for new path failed
[WARN] -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return
```

### 1.2 FUEL 侧：反复发同一个目标

`grep -E "Next view" /tmp/fuel_real_super.log | tail -2`：

```
Next view:    8.82579 -0.0399559   0.787528, -2.54212
Next view:    8.82579 -0.0399559   0.787528, -2.54212      ← 完全相同，重复
```

**距起飞点 8.8 m** —— 远超 rog_map 实际建过的范围。

### 1.3 仲裁器侧：20 秒"防死锁"放的还是同一个目标

```
[arbiter] FUEL 候选目标: (8.83, -0.04, 0.79)
[arbiter] >>> 放行目标 (8.83, -0.04, 0.79) | 原因: 保持超时 20.2s（强制放行，防死锁） | 上一目标: (9.33, -0.40, 0.79)
[arbiter] >>> 放行目标 (8.83, -0.04, 0.79) | 原因: 保持超时 20.2s（强制放行，防死锁） | 上一目标: (8.83, -0.04, 0.79)
```

注意最后一行：**上一目标 == 本次目标** ⇒ 放行没有带来任何新信息，死锁原样保留。

### 1.4 FSM 停在 WAIT_GOAL

```
-- [Fsm 184.216] Current state: WAIT_GOAL
-- [Fsm 185.221] Current state: WAIT_GOAL
...
-- [Fsm] Get goal at  8.775 -0.075  0.825          ← 目标在收，但出不了轨迹
-- [Fsm 188.236] Current state: WAIT_GOAL
...
-- [Fsm 194.272] Current state: WAIT_GOAL
```

**量化**：`WAIT_GOAL` 34 行 / `FOLLOW_TRAJ` 126 行 / `AVOID_WAIT` 3 行
⇒ 前期正常飞，末尾卡在收目标不动的状态。

---

## 2. 为什么 FUEL 不换目标（机制层，不是 bug）

链路：

```
FUEL fast_exploration_manager
  → /exploration_goal            (FUEL 的"要去哪")
  → goal_arbiter 限频
  → /waypoint_goal
  → fsm_node (SUPER 规划)
  → /planning/pos_cmd            ← 被 remap 到 /fuel/pos_cmd_sink（防抢控制）
```

**关键**：`traj_server` 仍按原逻辑运行（只是输出被丢弃），它依据 **`/odom_world` 的真实位置**
判断轨迹是否执行完、是否到达目标。**飞机没动 ⇒ traj_server 判"没到达" ⇒ FUEL 的 FSM 一直认为
当前目标未完成 ⇒ 反复产出同一个目标**。

⇒ **"改道"只切断了输出，没切断 FUEL 的到达判定**。这是架构耦合，不是缺陷；
**正因如此，不能指望 FUEL 自己跳出死锁**，必须在仲裁器侧加"任务级放弃"。

实测辅助证据：`/odom_world` 当时位置正常（`x=0.077 y=0.054 z=0.643`，四元数无 NaN），
排除"里程计坏了导致目标算错"这一支。

---

## 3. 实机的两层机制（第一层 SUPER 已有）

### 3.1 第一层：SUPER `super_core/fsm.cpp:167-193`（已有）

```cpp
// 轨迹生成失败 → 进入"停车避让"状态(主动悬停+限频重试, 不再高频空转)
cout << YELLOW << " -- [Fsm] PlanFromRest failed, switch to AVOID_WAIT (park & wait)." << RESET;
publishHoldPose();  // 主动发悬停指令给 bridge(不是等超时)
avoid_wait_t_ = 0.0; // 重置避让计时
ChangeState("MainFsmCallback", AVOID_WAIT);
...
case AVOID_WAIT: {
    // 停车避让: 悬停 + 限频重试(0.3s), 轨迹成功→恢复飞行, 障碍不消失→安全等待
    avoid_wait_t_ += 0.1;  // FSM 主循环 ~10Hz
    publishHoldPose();     // 持续发悬停(保持位置)
    if (avoid_wait_t_ >= 0.3) {  // 每 0.3s 重试一次
        avoid_wait_t_ = 0.0;
        int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (retcode == SUCCESS || retcode == FINISH) {
            gi_.new_goal = false;
            plan_from_rest_ = true;
            finish_plan = false;
            if (retcode == FINISH) { finish_plan = true; }
            publishPolyTraj();
            cout << GREEN << " -- [Fsm] AVOID_WAIT: path recovered, resume flight." << RESET;
            ChangeState("MainFsmCallback", FOLLOW_TRAJ);
        } else {
            // ← 无限重试分支：没有任何超时/计数/放弃逻辑
        }
    }
}
```

接口：`include/fsm/fsm.h:81` 枚举 `AVOID_WAIT`、`:90` `MACHINE_STATE_STR`、
`:95` `double avoid_wait_t_{0.0};`、`:168` `virtual void publishHoldPose() = 0;`。

### 3.2 第二层：任务节点 `task_run_scripts/scripts/task_node_nav_LowVoltage.py`（SUPER 没有）

```python
90:  def navigate_to_waypoint(wp, timeout=20):
121:      while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time) < timeout:
139:      rospy.logwarn(f"导航到航点超时! 最终距离: {distance:.2f}米, 最终航向误差: {math.degrees(yaw_error):.1f}°")
236:      rospy.logwarn(f"导航到航点 {i+1} 失败，尝试下一个航点")
```

**每个航点最多等 20 秒 → 超时告警 → 放弃 → 试下一个。**
这正是用户说的"实机的悬停重试机制"里 SUPER 缺失的那一半。

---

## 4. 撞障碍：参数对比法

`odin_real_flight.yaml`（实机） vs `sim_real_flight.yaml`（仿真），

```bash
for k in robot_r obstacles_inflation inflation_resolution inflation_step \
         resolution max_vel max_acc max_acc_thr; do
  echo "--- $k ---"
  grep -E "^\s*$k:" ~/super_sim_ws/src/task_run_scripts/config/odin_real_flight.yaml  | head -1
  grep -E "^\s*$k:" ~/super_sim_ws/src/task_run_scripts/config/sim_real_flight.yaml | head -1
done
```

实测结果：

| 参数 | 实机 | 仿真 | 差异 |
|---|---|---|---|
| `robot_r` | 0.35 | 0.35 | 相同 |
| `inflation_resolution` | 0.2 | 0.2 | 相同 |
| `inflation_step` | 2 | 2 | 相同 |
| `resolution` | 0.15 | 0.15 | 相同 |
| `max_vel` | 0.3 | **1.0** | 3.3× |
| `max_acc` | 0.4 | **1.0** | 2.5× |
| `max_acc_thr` | 20 | 20 | 相同 |

**安全裕度一致而速度差 2.5~3.3 倍** ⇒ 物理上是撞障碍的候选原因；
但**地图/定位错位**（FAST-LIVO2 点云外参、FOV 在仿真的偏差）同样会撞，
且两者修法相反 ⇒ **必须先做降速判别实验**（降到 0.3/0.4 跑一次二分）。

---

## 5. 一次观测把上面全拿到的命令

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
source /opt/ros/noetic/setup.bash
M(){ timeout 10 rostopic hz "$1" 2>/dev/null | grep average | head -1; }

# 卡死定性
printf "  pos_cmd  %s\n" "$(M /planning/pos_cmd)"
printf "  plan_goal %s\n" "$(M /waypoint_goal)"
for k in "Current state: WAIT_GOAL" "Current state: AVOID_WAIT" "Current state: FOLLOW_TRAJ" \
         "GenerateExpTrajectory failed" "Ill corridor" "Cannot simplify sfcs" "BackOpt"; do
  printf "  %-34s %s\n" "$k" "$(grep -c "$k" /tmp/super_fsm.log)"
done
grep -n -B4 -A4 "GenerateExpTrajectory failed" /tmp/super_fsm.log | head -20
tail -6 /tmp/goal_arbiter.log            # 看"上一目标 == 本次目标"就说明死锁
grep -E "Next view" /tmp/fuel_real_super.log | tail -3
```

**判读口诀**：
- `Ill corridor` / `Cannot simplify sfcs` ⇒ **不是规划器坏，是走廊无据可依**（目标超范围 / 地图未建到）
- 仲裁器连续两次放行**同一目标** ⇒ 死锁，别再等它自己解
- FUEL `Next view` 连续重复 ⇒ 飞机没动导致它判"未到达"（见 §2）
