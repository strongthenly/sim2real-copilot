# 仿真平台启停脚本套件 与 PX4 掉线诊断（2026-09-22 实测）

FUEL(决策) → SUPER(执行) 在 `ros1-twin` + AirSim 上的**日常运维面**：两个一键脚本、各自的实测行为、
以及一套「看起来像参数错、其实是 PX4 死了」的诊断链。

---

## 一、两个一键脚本

源码都在 `~/fuel_ws/src/fuel_bridge/scripts/`，`~/` 下各有软链接（与既有 `fuel_all.sh` / `fuel_stop.sh` 同风格）。

### `super_explore_all.sh`（启动，12 KB）

```bash
bash ~/super_explore_all.sh 1                 # 参数 1 = 退出时保存建图 PCD
bash ~/super_explore_all.sh 0                 # 不保存（默认；等价 --pcd 强制开）
bash ~/super_explore_all.sh 1 --print-params  # ★ 只打印解析结果后退出（不启动任何东西）
bash ~/super_explore_all.sh --dry-run         # 预演：仲裁器只打印不发布（首次必用）
bash ~/super_explore_all.sh --no-fuel         # 只起 SUPER（单独验证）
bash ~/super_explore_all.sh --no-super        # 只起 FUEL 侧
bash ~/super_explore_all.sh --no-trigger      # 起了但不自动触发探索
bash ~/super_explore_all.sh 1 5.0             # 存 PCD + 仲裁器最小放行间隔 5s（裸数字）
```

7 步，每步**硬检查 + 明确诊断**（不干等）：

| 步 | 内容 | 硬检查 |
|---|---|---|
| 0 | AirSim RPC | `41451` 可连，否则直接退出并提示点 Play |
| 1 | twin 链路（FAST-LIVO2） | `/clock` 在步进 **且** `/cloud_registered` 有数据 |
| 2 | PX4 MAVLink 14610/14611 | PX4 未跑 → 只提示，**不阻塞**话题级自检 |
| 3 | MAVROS | SUPER 的 odom 来自 `/mavros/local_position/odom` |
| 4 | SUPER（fsm_node + bridge，sim_real_flight.yaml） | fsm_node 存活，死了就报日志路径 |
| 5 | FUEL（`fuel_cmd_topic:=/fuel/pos_cmd_sink` 改道） | `/exploration_goal` 是否出现 |
| 6 | 目标仲裁器 | 存活；支持 `--dry-run` |
| 7 | **链路自检 + 触发** | ⭐ `/planning/pos_cmd` 发布者数必须 = 1 |

第 7 步是**防抢控制的总闸**：>1 个发布者说明 FUEL 的 remap 没生效、两条控制链在抢，
脚本直接打红并指向 `/tmp/fuel_real_super.log`。脚本内已叠加 `ROS_PACKAGE_PATH`（见 SKILL.md 坑 G）。

### `super_explore_stop.sh`（关闭，5.9 KB）

```bash
bash ~/super_explore_stop.sh                    # 常规：保留 master，PCD 会保存
bash ~/super_explore_stop.sh --all              # 全停（连 ROS master）
bash ~/super_explore_stop.sh --no-save          # 不等 PCD，SIGKILL 快速关
bash ~/super_explore_stop.sh --keep-px4 --keep-mavros   # 只停上层应用
bash ~/super_explore_stop.sh --print-params     # 只打印参数，不关任何东西
```

8 步：① SUPER（bridge→fsm_node，**先停控制输出端**）② FUEL（exploration_node / traj_server /
waypoint_generator / 两个桥）③ goal_arbiter ④ MAVROS ⑤ **FAST-LIVO2（SIGINT → savePCD，等 ≤25s 并列出 PCD 文件）**
⑥ 仿真桥 + RViz ⑦ PX4 ⑧ ROS master（默认保留）。**结尾自动逐进程核对是否真停了。**

**设计要点（照抄即可）**：

- `ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}` **可覆盖** + 从中提取 `MASTER_PORT`
  → `--all` 精确匹配该端口，**测试时指向一次性端口就绝不碰用户常驻实例**（见 SKILL.md「启停脚本验证」节）。
- 容忍误传位置参数（`0|1)` 忽略），停止脚本不需要 pcd 参数。
- `--print-params` 打印一行 `PARAMS all=… no_save=… keep_px4=… keep_mavros=…` 后退出。

**实测行为表**（隔离端口 11321，全程未碰用户 11311）：
`--print-params` 不关任何进程 ✅ | 默认停八个进程全停 ✅ | ⭐ SIGINT 触发保存（marker 生成）✅ |
master 默认保留 ✅ | `--keep-px4`/`--keep-mavros` 正确保留 ✅ | `--no-save` 走 SIGKILL 且**不**触发保存 ✅ |
⭐ `--all` 真停掉目标 master、用户 11311 毫发无损 ✅

---

## 二、⭐ 「参数看着都对、话题却没数据」= 先查 PX4 死没死

联调时 `super_explore_all.sh` 跑完「完成」，但系统其实没在工作。症状与真因：

| 症状 | 真因 |
|---|---|
| `/tmp/fuel_real_super.log` **0 字节** | FUEL 的 roslaunch 压根没起来（PX4 链路检查未过即退出） |
| `super_fsm.log` 刷满 `[SUPER] in [PlanFromRest]: No odom, force return.`（2227 行） | SUPER 收不到里程计 → 无法规划 |
| `/mavros/state` 无数据、`/mavros/local_position/*` 无数据 | MAVROS 起来了（127 个话题在）但**没连上 FCU** |
| `14610/14611` 未监听；`/clock` 不步进 | **PX4 没在跑** |
| `rostopic info` 里出现已死节点的名字 | **僵尸注册**（见第三节） |

**完整因果链**：

```
twin_livo_fast.sh 用 tmux 拉 PX4
   ↓
PX4 启动成功（Simulator connected on TCP port 4560 → EKF aligned → GPS fusion）
   ↓
⭐ tmux server 消失（no server running on /tmp/tmux-1000/default）
   ↓  PX4 收到 SIGHUP
pxh> Exiting NOW.            ← /tmp/px4_run.log 最后一行
   ↓
14610/14611 关闭 → MAVROS 无 FCU → 没有 /mavros/local_position/odom
   ↓
SUPER 永远 No odom
```

**判据（照这几条看，不要瞎调参数）**：

```bash
pgrep -af "bin/px4|px4_sitl"          # 空 ⇒ PX4 不在
tmux ls                                # no server running ⇒ tmux 会话整个没了
tail -25 /tmp/px4_run.log              # 找 "Exiting NOW."
ss -tln | grep -E "14610|14611"        # 未监听 ⇒ PX4 侧 MAVLink 没起
rostopic echo -n1 /mavros/state        # 空 ⇒ MAVROS 没连 FCU
rostopic hz /clock                     # 不步进 ⇒ LockStep 冻结（点云也会跟着没）
```

**注意 `twin_livo_fast.sh` 的 PX4 分支**：`if ! pgrep -f "bin/px4"; then tmux kill-session -t px4;
tmux new-session -d -s px4 "cd ~/PX4-AirSim && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log"; fi`
—— 「PX4 已退出但 tmux 会话还在」时它**不会重启**；而 tmux server 消失后连 `capture-pane` 都看不到了。

**改进建议（未实施，待用户确认）**：给会话加 `remain-on-exit`，PX4 若再挂会话仍保留、可直接
`tmux capture-pane -p -t px4` 看到退出原因，不必像本次那样靠日志反推：

```bash
tmux kill-server 2>/dev/null
tmux new-session -d -s px4
tmux set-option -t px4 remain-on-exit on
tmux send-keys -t px4 'cd ~/PX4-AirSim && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log' Enter
# 等 25s：看到 "Simulator connected on TCP port 4560" + "EKF aligned"
# 收工前必须确认两件事：ss -tln | grep 14610 有监听、rostopic hz /clock 在走
```

---

## 三、⚠️ 僵尸注册：`rostopic info` 里的发布者可能是**已经死掉的节点**

排查「两条控制链抢 `/planning/pos_cmd`」时，`rostopic info` 列出：

```
* /traj_server            ← 进程根本不存在！
* /fsm_node
* /super_to_px4_bridge
```

**节点死了但 master 仍记着它的 publisher 注册**，直到 master 自己清理或重启。
我据此一度判定「FUEL 改道失败、两链在抢」——**结论是错的**。

**规矩**：看到「多余的发布者」先 **`pgrep -f <节点名>` 交叉确认进程是否真的活着**，
再决定是不是抢控制。真凭据优先级：`pgrep` > 发布者注册表。
（另注：`super_to_px4_bridge` 自己也会出现在该话题的发布者里，不必紧张——它是转发/调试用途。）

**要干净的话**：`bash ~/super_explore_stop.sh --all` 停掉 master 重起，注册表即清空。
