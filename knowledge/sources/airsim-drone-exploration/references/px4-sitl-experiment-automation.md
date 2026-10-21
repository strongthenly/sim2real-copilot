# 全自动 SITL 实验入口脚本（run_auto.sh 契约 + 实测坑）

场景：把「前置检查 → 起后端 → 等就绪 → 跑一次飞行试验 → 记录 → 只清理自己的进程」做成**一个 bash 入口**，
让用户只跑一行就能出一组数据。本文件来自 `px4_brake_lab`（附加停车指令延迟 → 停止距离基线实验，
交付物 `run_auto.sh`，Windows 侧 `C:\Users\<USER>\Documents\Codex\2026-09-17\wo-xi\outputs\px4_brake_lab\`）。

适用面比这个实验宽：**任何「AirSim + PX4 SITL + MAVROS + 一个飞行节点」的自动化实验**。

---

## 1. MAVROS `setpoint_raw/local` 的坐标语义（源码级确认，别再猜）

**结论：`coordinate_frame` 只要不是 `BODY_*`，MAVROS 一律把输入当作 ENU，并自己转成 NED 发给 PX4。**
所以 **ROS 侧永远填 ENU 值，绝不手工转换 x/y/z**（手工转会双重转换）。

MAVROS 1.20.1 `mavros/src/plugins/setpoint_raw.cpp` 的 `local_cb`（ROS → PX4 方向）：

```cpp
// Transform frame ENU->NED
if (req->coordinate_frame == FRAME_BODY_NED || req->coordinate_frame == FRAME_BODY_OFFSET_NED) {
    position = ftf::transform_frame_baselink_aircraft(position);
    velocity = ftf::transform_frame_baselink_aircraft(velocity);
    af       = ftf::transform_frame_baselink_aircraft(af);
    yaw      = ...transform_orientation_absolute_frame_aircraft_baselink...
} else {                                   // ← FRAME_LOCAL_NED 走这里
    position = ftf::transform_frame_enu_ned(position);
    velocity = ftf::transform_frame_enu_ned(velocity);
    af       = ftf::transform_frame_enu_ned(af);
    yaw      = ftf::quaternion_get_yaw(
                   ftf::transform_orientation_aircraft_baselink(
                       ftf::transform_orientation_ned_enu(
                           ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
}
```

配套要点：
- **yaw 也被转换**（ENU yaw → NED yaw，即 `2*atan2(q.z,q.w)` 那套）。所以从 MAVROS 读回的位姿 yaw 直接回填 setpoint 是自洽的。
- **掩码 2503 = `IGNORE_PX|IGNORE_PY|IGNORE_PZ|IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ|IGNORE_YAW_RATE`**
  （1+2+4+64+128+256+2048）= 忽略位置/加速度/yaw_rate，**使用「速度 + yaw」**，
  **`FORCE`(512) 不置位** ⇒ `acceleration_or_force` 字段是**加速度**语义而非推力。
  ⚠️ **别照抄 FUEL 控制桥 raw 分支**：那边把 `FORCE=512` 当「忽略 force」写，语义是错的
  （本实验另起掩码，没复用）。
- 纯速度模式下 **PX4 不管高度** → 需要自己带一个高度回路；本实验用
  `vz = clamp(target_z - p.z, ±0.4)`（固定的「高度误差→垂向速度」规则，两组实验用同一规则才可比）。
- **「零速度指令」≠「最大制动力」**；**「零附加延迟」仍含系统固有延迟**。汇报时必须写清口径，
  且应用层发送时间**不是**电机实际开始制动时间（要更硬的证据得上 ULog/内部 setpoint 交叉核查）。

### 校验方法（想再确认 MAVROS 行为时）
本地是 apt 二进制、无源码。**别用 `find /` 全盘扫**（会超时）：
1. 本机 `grep -rn "setpoint_raw" /opt/ros/noetic/share/mavros/launch/`（只看得到 `thrust_scaling`，不够）；
2. 从 Windows 侧直接拉源码（WSL 里 Clash 代理常常没开，Windows 侧反而通）：
   `Invoke-WebRequest https://raw.githubusercontent.com/mavlink/mavros/1.20.1/mavros/src/plugins/setpoint_raw.cpp`

---

## 2. 入口脚本的契约（照这个骨架写）

```
[0] 环境自举        source ROS（见 §3.1）+ export ROS_MASTER_URI/ROS_IP
[1] 前置检查        AirSim RPC ping + simIsPaused；**不清理他人进程**，有残留就拒绝启动
[2] 残留检查        已有 master / PX4 / MAVROS / 其他控制话题发布者 → 报错退出（严禁双控制节点）
[3] 起后端          记录每个 PID 到 OWNED_PIDS 账本：rosmaster → rosout → PX4(tmux) → 时钟桥 → MAVROS
[4] 等「实际就绪」  数据驱动，不看日志关键词（见 §2.1）
[5] 跑试验          调既有飞行节点 + 记录退出码 + 抓 RESULT_DIRECTORY
[6] 记录            版本（PX4 git describe / mavros rosversion / python）、参数、AirSim settings.json 快照、各节点日志
[7] 清理            逆序只杀 OWNED_PIDS；**未确认落地 → 保留控制链路不杀**
```

硬性规则：
- **只清理本脚本创建的进程**：维护 `OWNED_PIDS` 数组，**不用 `pkill -f`**。tmux 会话也记 `PX4_TMUX_OWNED` 标志。
- **未确认落地时不得杀控制链路**：判据用试验 summary 里有没有 `landed_disarmed` 事件（不是看退出码）。
  没有 → 保留 MAVROS/PX4/master，并在终端用红色明确告知"请人工确认飞行器状态"。
- **`--keep-running`** 开关（调试时保留后端）；`--skip-trial`（只联调后端不飞，可作零风险的回归测试）。
- **每阶段都要有超时 + 进度输出**（用户会以为"卡住"）；异常时打印**最后几行真实日志**，而不是只说"失败"。
- `trap on_sig INT TERM` + `trap cleanup EXIT`；cleanup 首行 `trap - INT TERM EXIT` 防重入。
- **不要 `set -e`**：要自己判断每一步并在失败路径上做清理。

### 2.1 「实际就绪」怎么判（本会话采用的判据）
一个 Python 探针同时订阅，**全部满足**才算就绪（给 120s 预算、每 10s 打进度）：
- `/clock` 在**步进**（连续若干次 `rospy.Time.now()` 递增）—— 这是 LockStep 下"仿真到底在不在跑"的唯一判据；
- `/mavros/state`：`connected == True`、`armed == False`；
- `/mavros/extended_state`：`landed_state == 1`（ON_GROUND）；
- `/mavros/local_position/pose` 有数据；
- `rospy.get_param('/use_sim_time') == True`。
再到 ROS master 上查 8 个 control setpoint 话题的发布者，**必须为空**（否则"双控制节点"，拒绝起飞）。

---

## 3. 实测缺陷清单（**只有真实运行才抓得到**，离线语法检查全过）

写这类脚本时逐条自查。

### 3.1 脚本没自己 `source` ROS → `rosmaster: command not found`
"全自动"的前提是**不依赖调用者的 shell**。脚本开头自举：
```bash
ROS_SETUP=/opt/ros/noetic/setup.bash
[ -f "$ROS_SETUP" ] || { echo "找不到 $ROS_SETUP" >&2; exit 1; }
set +u                      # ← 见 3.2
source "$ROS_SETUP"
[ -f "$TWIN_WS/devel/setup.bash" ] && source "$TWIN_WS/devel/setup.bash"
set -u
command -v rosmaster >/dev/null 2>&1 || { echo "ROS 环境加载失败" >&2; exit 1; }
export ROS_MASTER_URI="http://127.0.0.1:$MASTER_PORT" ROS_IP=127.0.0.1
```

### 3.2 `set -u` 与 ROS 的 setup.bash 冲突
症状：`/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: line 3: ROS_DISTRO: unbound variable`，脚本秒退。
根因：ROS 那批 `1.ros_distro.sh` / `2.ros_*.sh` 会引用未定义变量，撞上 `set -u`。
**修法：只在 source 期间 `set +u`，source 完 `set -u`**（不要为了它整体放弃 nounset）。

### 3.3 PX4 SITL 必须用 `make` 入口，别直接调二进制
```bash
# ✗ 错（本会话踩）：把 -d 后面的路径当 rcS，PX4 自己建 symlink 后找不到启动文件
./build/px4_sitl_default/bin/px4 -d build/px4_sitl_default/etc
#   → INFO  [px4] Creating symlink build/px4_sitl_default/etc -> /home/<USER>/PX4-AirSim/etc
#   → ERROR [px4] Error opening startup file, does not exist: etc/init.d/rcS
# ✓ 对（README 的权威入口，会准备 etc/init.d-posix/rcS）
cd $PX4_DIR && make px4_sitl_default none_iris
```
放 tmux 里跑（PX4 无 tty 会 `pxh> Exiting NOW.`），日志 `| tee $RUN_TMP/px4.log` 便于 grep。
就绪判据用 `Ready for takeoff|Simulator connected|INFO  \[simulator|INFO  \[mavlink`，
等不到时**把最后 12 行日志打给用户**（比只说"失败"有用得多）。然后 `tmux send-keys` 下发
`mavlink start -u 14610 -o 14611 -r 4000000 -m onboard`。

### 3.4 `Clock` 消息在 `rosgraph_msgs`，不在 `sensor_msgs`
`from sensor_msgs.msg import Clock` → `ImportError: cannot import name 'Clock'`。
正确：`from rosgraph_msgs.msg import Clock`。（锁步仿真里其实用 `rospy.Time.now()` 递增判断即可。）

### 3.5 `.launch` / launch 参数的两个连带坑（写脚本时会顺手踩）
- `<arg name="x" value="..."/>` **不可被命令行 `x:=...` 覆盖** → 要可覆盖必须写 `default=`。
- `$(arg foo)` **必须先定义后使用**（前向引用直接报错）→ 把被引用的 arg 定义提到文件前部，删掉原处重复定义。
- 参数真实路径是 **node 名 + param 名叠加**：`<node name="traj_server">` 里的
  `<param name="traj_server/init_z">` → 实际是 `/traj_server/traj_server/init_z`。
  查错路径（`/traj_server/init_z`）会**假报"参数不存在"**（本会话据此白追一轮）。
- **`init_z` 必须等于 `seed_z`**：`traj_server` 启动时会发一条 `(0,0,init_z)` 的**占位指令**，
  控制桥一旦收到就走"规划器分支"、不再发 seed，于是**无人机锁死在 init_z 高度**等不到目标高度。

### 3.6 PowerShell 会吃掉 `2>/dev/null`（写成 `C:\dev
ull`）
```
out-file : 未能找到路径"C:\dev\null"的一部分。
```
`wsl.exe -d X -- bash -c "... 2>/dev/null ..."` 里，PowerShell 把 `2>` 当成它自己的重定向。
→ **凡命令里含 `2>`/引号/中文/括号，一律 base64 传脚本**：
```powershell
$b = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($s))
wsl.exe -d ros1-twin -u <USER> -- bash -c "echo $b | base64 -d > /tmp/x.sh; bash /tmp/x.sh; rm -f /tmp/x.sh"
```

### 3.7 `nohup ... &` 在 `wsl.exe` 返回后被 WSL 会话杀掉
症状：临时目录建好了、日志文件却不存在、进程也没了（"启动了又像没启动"）。
→ **长流程直接前台跑**（Hermes 前台 timeout 可到 600s，够起全套后端）；
确需后台时用 `setsid` 或干脆放 tmux。

### 3.8 干净 shell 才能验出自举缺陷
验证入口脚本时，**故意不要预先 `bash -c 'source ...; bash run_auto.sh'`**，直接用
`bash -c "echo <b64> | base64 -d > /tmp/x.sh; bash /tmp/x.sh"` ——
3.1/3.2/3.6 这些缺陷全都是"在干净 shell 里真跑一次"才暴露的。

---

## 4. 「运行前确认飞行方向空旷」——用 AirSim RPC 探，别靠猜

`scripts/airsim_airspace_probe.py`：纯 socket+msgpack 调 AirSim RPC（**不需要装 airsim pip 包**），
取 `getMultirotorState`（位姿/四元数）+ `getLidarData(<lidar名>)`，
把 `SensorLocalFrame` 点云用机体系四元数旋到世界 NED，再按 ENU 四个方向统计
「1m 宽走廊内点数 / 最近障碍距离」，直接告诉你能不能飞、哪个方向最空。

- AirSim 默认 `SensorLocalFrame` = 机体系（x 前 / y 右 / **z 下**）。
- 世界系是 **NED**（x 北 / y 东 / z 下，地面 z=0、向上为负）；
  MAVROS/ENU 的 **+X = 东 = NED 的 +y**，`--direction-deg 0` 就是往东飞。
- 只保留"空中带"的点（NED `-3.0 < z < -0.3`，即离地 0.3~3.0m）再统计，否则**地面会淹没一切**。
- 判据：走廊内点数 + 最近距离。本会话实测 Blocks 场景四方向最近障碍 3.76~5.07m，
  而该实验前向距离约 2~3.3m（≥2s 巡航 × 1m/s + 刹车）⇒ 边际 1~2m，**能飞但要如实告知边际有多薄**。

### AirSim 场景无法自动启动（本机实况）
- `E:\AirSim\Unreal\Environments\Blocks\` 下**只有 `Blocks.uproject`，没有 `Blocks.exe`** ⇒
  没有打包版、**无法自动进入 Play**；UE4Editor 进程在跑 ≠ AirSim 可用。
- 所以「**用户先打开场景并 ▶ Play**」是**唯一保留的手动前置**；脚本负责**自动检测**它
  （RPC `ping` + `simIsPaused`）并给出可执行的提示，而不是替用户猜路径。
- 一般性纪律：**无法确认的启动方式不要猜**，如实保留一个人工步骤并把它写进提示里。

---

## 5. 定级口径（交付时必须分开说，别混）

| 级别 | 含义 | 本会话证据 |
|---|---|---|
| 已编写 | 文件存在 | `run_auto.sh` |
| 离线通过 | 语法/参数/单元测试 | `bash -n`、`--help`、6 组非法参数被拒 |
| **后端联通** | 真的把后端串起来并读到就绪信号 | `READY connected=True armed=False landed_state=1 clock_steps=10` → 逆序清理 → **RC=0** |
| 实际飞行 | 真的飞过一轮并出数据 | ✅ 停车延迟 **0/100/200/300 ms 各 5 次**（交错）+ 速度 **0.5/1.0/1.5 m/s 各 5 次**，全部 `landed_disarmed`、**0 失败** |

用户会明确要求区分这四级 —— **不要用"跑通了"一句话糊过去**；没做的就写没做。

---

## 6 跑出第一组数据之后（别停在"飞通了"）

单次成功只证明链路通。要继续做**可重复实验**，看 **`references/control-experiment-metrics.md`**，要点先记在这里：

- **批量重复的契约**：交错顺序（`0 200 200 0 | 0 200 200 0 | ...`）抵消时间漂移；
  **★ 每轮前必须 AirSim `reset` 复位场景**（否则第二次起点 = 上次降落点 ⇒ 不可配对）；
  **保留全部运行含失败**（`*_FAILED_runN/` + `FAIL_REASON.txt`）；轮次间 sleep 3 s；失败判定看 `landed_disarmed` 而非退出码。
- **拆口径**：`t_send / t_decel / t_low / t_stop` 四事件 + 派生 `cmd_latency / brake_actual / settle_confirm`；
  **`cmd_latency ≈ 85 ms`（跨延迟跨速度恒定）**，"低速→停稳 0.50 s"只是 dwell **人为窗口**（物理制动完成看 `t_low`）。
- **名义延迟 ≠ 实测延迟**（50 Hz 量化 ⇒ 名义 200 ms 实测 214 ms），两个口径都要报，并用
  「实测延迟 × 巡航速度 = 延迟窗口位移」做物理自洽核对。
- **自变量用实测值**（名义 1.0 m/s 实测 1.058），且取"请求前 0.25 s"的平均速度，**别用 CRUISE 全段平均**。
- **扩展新档位前先判稳定性 + 探空域**：逐方向探测要多次采样取最保守（AirSim GPU LiDAR 有随机性）；
  本机 Blocks **只有 ENU 45° 的 6.12 m 能容纳 1.5 m/s**，其余方向边际不足 ⇒ 换速度档位往往要换方向，且三档必须同方向才可比。

