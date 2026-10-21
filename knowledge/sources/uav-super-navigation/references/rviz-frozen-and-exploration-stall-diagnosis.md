# "RViz 画面不动" 诊断手册（FUEL + SUPER + AirSim 栈）

## 先分清三类，再动手（90% 的误判出在没分清）

| 类 | 特征 | 查法 |
|---|---|---|
| **A 数据没在变** | `/clock` 停、odom 不变 | 采两次 `/clock`、`/mavros/local_position/pose` 对比 |
| **B 数据在变但 RViz 不画** | 话题有 Hz，画面静止 | Fixed Frame 是否存在、`use_sim_time`、点云 `Decay Time` |
| **C 没东西可画** ← **最常见** | 待画话题本身无数据 | `rostopic info` 逐个看发布者 / `rostopic hz` 看频率 |

**本次实测结论**：`/clock 168 Hz`、odom/pose 23 Hz 都在变，但
`/planning_vis/frontier`、`/fsm_node/visualization/exp_traj`、`/fsm_node/fsm/path` **全无数据**
→ **RViz 没坏，是无物可画** ✓ 属 C 类。

## ⚠️ 三个诊断方法错误（我都犯过，会得出反向结论）

### 错误 1：用 `rosrun tf tf_echo` 判 TF 断没断
```
$ rosrun tf tf_echo world map
Exception: "world" passed to lookupTransform argument target_frame does not exist.
```
**这不代表 TF 真断** ✗ —— `tf_echo` 是独立节点，若没读到 `/use_sim_time` 就用**墙上时间**查询，
而 /tf 是**仿真时间**（本机两者差 5 天）→ 必然查不到 ✗

```bash
rosparam get /use_sim_time                     # 先确认口径
rostopic echo -n 30 /tf | grep -E "frame_id|child_frame_id" | sed 's/^ *//' | paste - - | sort | uniq -c
rostopic echo -n 5 /tf_static | grep -E "frame_id|child_frame_id" | paste - - | sort -u
```
**从消息里直接读 frame 对**，不依赖 tf 工具的时钟口径 ✓

### 错误 2：用 `pgrep -f <节点名>` 判节点死活
节点名 ≠ 进程名（`exploration_node` 这类名字在 `rostopic info` 里有，但命令行匹配不到）。
```bash
rosnode list                                   # ✓ 权威
rostopic info /exploration_goal                # ✓ 看 publisher 的 URI（还活着就有）
```

### 错误 3：只抓 1 条 `/tf` 消息就断言"TF 树里只有 X→Y"
一条 `/tf` 消息可含**多个** transform ✗ 必须 `-n 20` 以上再 `uniq -c` ✓

## 本次的完整因果链（FUEL 栈，可作模板）

```
① exploration_node（FUEL）SIGABRT: process has died, exit code -6
② → /exploration_goal 断流
③ → goal_arbiter 靠"保持超时 20s 强制放行"空转，在【两个缓存目标】间来回放
     放行 (5.78, 4.27, 1.13) | 原因: 保持超时 20.2s（强制放行，防死锁）
     放行 (4.73,-2.77, 1.17) | 原因: 目标不可达（15.0s 内位移 0.01m、仍差 10.12m）→ 收缩
     ← 两点相距 ~7m，来回切；arbiter 日志直接给出位移只有 0.01m = 飞机根本没动
④ → SUPER 每 20s 目标反向 180° → 规划需求 yaw 4.9~6.0 rad/s
     vs 配置上限 yaw_dot_max: 2.5 / max_omg: 2.5
     → yaw_traj_opt.cpp:188 "Yaw rate too large" → 轨迹被拒（本次刷 110 次）
⑤ → /planning/pos_cmd 不发（`rostopic info` 有 publisher 但 hz 无数据）
⑥ → super_to_px4_bridge 收不到 → [POS_HOLD] x/y/z/yaw 值完全不变
⑦ → 飞机悬停不动 → RViz 无物可画
```

**判读要点**：
- `Yaw rate too large` **不是** `max_vel` 改动的锅，是**目标反复反向**的结果 ✓
  （降速不会降低 yaw 需求；朝向跳变才会）
- **`/fsm_node/*` 有 publisher 但 hz 无数据** = FSM 在 AVOID_WAIT 之类状态反复重规划、不输出 ✓
  （配 `-- [Fsm] ReplanOnce succeed.` 刷屏 + `Current state: FOLLOW_TRAJ` 佐证）
- 桥日志 `[POS_HOLD] ... ` 每秒刷且**数值完全不变** = 飞机真的停着 ✓

## ⭐ 判断"崩溃是老问题还是新引入"（很有用的一招）

```bash
grep -rE "died|core dump|exit code" ~/.ros/log/ | tail -20
```
本次据此翻出 **`exploration_node` 从 2026-09-12 起反复 `exit code -6`**
（9/12、9/22 ×2、9/27、9/28 都有）→ **是新场景暴露的老 bug，不是本次改动引入的** ✓
（若不做这一步，很容易把方向浪费在"我上次改的参数"上）

配套：
```bash
dmesg | grep -iE "segfault|oom|killed"     # 段错误/OOM
free -h                                    # 排除 OOM
ls -t ~/.ros/log/*/exploration_node*       # 找崩溃节点自己的日志
```

## 全套一次性只读体检脚本（崩溃前/后都能跑）

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311; source /opt/ros/noetic/setup.bash
rosparam get /use_sim_time
rosnode list
timeout 8 rostopic hz /clock                                   # 在走 = LockStep 在步进
for t in /mavros/local_position/pose /planning/pos_cmd /waypoint_goal /exploration_goal \
         /fsm_node/fsm/path /fsm_node/visualization/exp_traj /planning_vis/frontier /cloud_registered; do
  echo "$(printf '%-38s' $t) $(timeout 4 rostopic hz $t 2>&1 | grep -m1 'average rate' | awk '{print $3}')"
done
timeout 5 rostopic echo -n1 /mavros/state        # connected/armed/mode
tail -20 /tmp/goal_arbiter.log /tmp/super_bridge.log /tmp/super_fsm.log /tmp/fuel_real_super.log
```

## 心跳快的对照量（本机正常值，便于快速判偏）

`/clock` 168 Hz · `/twin/imu/data_raw` 195 Hz · `/cloud_registered` 8.9 Hz ·
`/mavros/local_position/odom|pose` 23 Hz · `/fsm_node/rog_map/occ|frontier` 2.0 Hz · `/tf` 34 Hz

`/twin/lidar` 无数据是**正常的**（点云走 `/cloud_registered`）—— 别把它当故障。
