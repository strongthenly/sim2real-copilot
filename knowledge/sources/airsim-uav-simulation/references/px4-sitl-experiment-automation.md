# PX4 SITL 仿真实验的自动化编排（一键脚本）

搭建层见 SKILL.md 与 `references/airsim-px4-twin-stack.md`。本文件讲**搭好之后**：
把「启动后端 → 等就绪 → 跑实验 → 记录 → 清理」做成一个可反复运行的一键脚本。
适用场景：停车延迟/控制切换/算法对比等**纯仿真实验**，需要两组或多组配对数据。

---

## 一、骨架（顺序不能乱）

```
[1] 前置检查    AirSim RPC 可达且未暂停（唯一手动前置，见下）
[2] 残留检查    ROS master / PX4 / MAVROS / 控制话题发布者 —— 有就【拒绝退出】，不要粗暴清理
[2b] 场景复位    AirSim simReset → 保证各组起点/姿态一致（配对公平性前提）
[3] 启动后端    ROS master → PX4 SITL(tmux) → 时钟桥 → MAVROS
[4] 等就绪      /clock 步进 + 数据新鲜 + FCU connected + landed_state=1 + armed=false
                + 无其他控制话题发布者（严禁双控制节点）
[5] 跑实验      调用实验节点（默认只读，显式 --execute-sitl 才飞）
[6] 记录+清理   写版本/配置/日志；【只清理本次启动的 PID】逆序 TERM
```

### 就绪判据要用"数据"而不是"日志关键词"
```python
# 正确：看数据本身
ok = (state.connected and ext.landed_state == 1 and not state.armed
      and rospy.Time.now().to_sec() > 0 and clock_steps >= 10)
```
只看日志出现某关键词容易假就绪。

### 清理只动自己启动的进程
`declare -a OWNED_PIDS`，启动时逐个记 PID（后台 `$!` / tmux 会话名），退出逆序 TERM，
**不要用 `pkill -f`**（会误杀别人的、或杀死自己）。

### 未确认落地时不得杀控制链路
从实验节点的 `summary.json` 里查有没有 `landed_disarmed` 事件：
有 → 安全收尾；**没有 → 保留 MAVROS/PX4/master 并明确告警**，交人工处理。

---

## 二、写这类脚本必踩的 6 个坑（全部实测）

| # | 症状 | 根因 / 修法 |
|---|---|---|
| 1 | `rosmaster: command not found` | 脚本**必须自己 source ROS**，不能依赖调用者先 source。开头 `source /opt/ros/noetic/setup.bash` + 工作区 `devel/setup.bash`，并 `command -v rosmaster` 自检 |
| 2 | `ROS_DISTRO: unbound variable` | **`set -u` 与 ROS `setup.bash` 冲突**（ROS 脚本引用未定义变量）。修：`set +u; source …; set -u`（仅 source 期间关闭 nounset） |
| 3 | PX4 起不来：`Error opening startup file, does not exist: etc/init.d/rcS` | **别直接调 `build/.../bin/px4`**。必须用 `make px4_sitl_default none_iris`（它会准备 etc/rcS 并让 none 模拟器在 4560 等 AirSim） |
| 4 | `ImportError: cannot import name 'Clock' from 'sensor_msgs.msg'` | `/clock` 的消息类型在 **`rosgraph_msgs.msg`**，不是 `sensor_msgs` |
| 5 | 参数校验把合法用法也拒了 | 加 `--check`/`--skip-trial` 这类"不飞行"的模式后，别忘了把它们从"必须给 `--delay-ms`"的校验里排除 |
| 6 | `--help` 打印出代码行（如 `set -u -o pipefail`） | `sed -n '2,20p' ${BASH_SOURCE[0]}` 行号范围越界；先数清注释块实际行号再写死 |

### 另两条环境性坑（不是脚本的错，但会让调试跑偏）
- **WSL 后台进程会被杀**：`wsl.exe … bash -c "nohup … &"` 在 `wsl.exe` 返回后，后台进程随会话消失
  （临时目录建了但进程没了）。→ **前台跑**（加长 timeout），或用 tmux/setsid 保活。
- **PowerShell 会破坏 `2>/dev/null` 与引号**：`wsl.exe … bash -c "… 2>/dev/null"` 被解释成
  `C:\dev
ull` 路径重定向；嵌套引号也常被吃掉。→ **一律 base64 传入**：
  `$b=[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($s)); wsl … bash -c "echo $b | base64 -d > /tmp/x.sh; bash /tmp/x.sh; rm -f /tmp/x.sh"`

---

## 三、AirSim 侧要点

- **无法自动进入 Play**：只有 `Blocks.uproject`（编辑器工程）而没有打包的 `Blocks.exe` 时，
  命令行进不了 PIE 模式 → **"用户先打开场景并点 ▶ Play"是必须保留的唯一手动前置**，
  脚本不要猜路径、不要硬编自动启动。改为**自动检测 + 明确提示**（见下）。
- **Play 状态检测**（RPC `ping` + `simIsPaused`，不需要 airsim pip 包，直接 msgpack over TCP 41451）：
  ```python
  # 返回 OK / PAUSED / DOWN
  call('ping', []); paused = call('simIsPaused', [])
  ```
  DOWN → 打印"请打开 Blocks 工程并点 ▶ Play"；PAUSED → 提示取消暂停。给不同退出码便于自动化判别。
- **配对公平性靠 `simReset`**：每次实验前 RPC `reset`（`call('reset', [])`），
  让飞行器回到初始位置/姿态。**必须在启动 PX4 之前做**（复位会切断既有连接）。
  复位后轮询确认 `ping` 通且 `simIsPaused is not True` 才算成功。
  复位完再核对两组 `trace.csv` 首帧 `(x,y,z)` 差值 → 才敢说"可作严格配对比较"。
- **飞行方向空旷性可用 LiDAR 自检**（比肉眼可靠）：RPC `getLidarData(['<LidarName>',''])` 拿
  `point_cloud`（扁平 `[x,y,z,…]`），注意 **`DataFrame: SensorLocalFrame` 是机体系**
  （x 前 y 右 **z 下**），先按无人机姿态转到世界系再按方向统计"最近障碍距离"。
  用它能提前发现"四个方向都只有 4~5 m 余量"这种风险。

---

## 四、MAVROS 坐标约定核对（别凭记忆，读源码）

关于"ROS 侧填 ENU 还是 NED、会不会重复转换"，**读 `mavros/src/plugins/setpoint_raw.cpp` 的
`local_cb`**（不是 `handle_position_target_local_ned`，那是反方向 PX4→ROS）：

```cpp
if (req->coordinate_frame == FRAME_BODY_NED || FRAME_BODY_OFFSET_NED) {
    ... 机体系处理 ...
} else {                                  // ← FRAME_LOCAL_NED 等走这里
    position = ftf::transform_frame_enu_ned(position);
    velocity = ftf::transform_frame_enu_ned(velocity);
    af       = ftf::transform_frame_enu_ned(af);
}
```
**结论：只要 frame 不是 `BODY_*`，MAVROS 一律把输入当 ENU 并转 NED。**
所以 ROS 侧填 ENU 值 + `coordinate_frame = FRAME_LOCAL_NED` 是**正确且无重复转换**的。

掩码自检（纯速度 + yaw，忽略位置/加速度/yaw_rate）：
`IGNORE_PX|PY|PZ|AFX|AFY|AFZ|YAW_RATE = 1+2+4+64+128+256+2048 = 2503`；
**`FORCE(512)` 不要置位**（置位会改变 `acceleration_or_force` 的语义为"力"——
旧 FUEL raw 分支把它当"忽略 force 位"是错的）。

MAVROS 是 apt 二进制、无源码时，从 Windows 侧直连拉：
`Invoke-WebRequest https://raw.githubusercontent.com/mavlink/mavros/<ver>/mavros/src/plugins/setpoint_raw.cpp`
（WSL 里 git clone 走代理常被 Clash 未开挡住，Windows 侧直连往往更快）。

---

## 五、分级诚实交付（用户明确要求）

汇报时**必须把结论按证据强度分开**，不能含混：

| 级别 | 含义 |
|---|---|
| 已编写 | 文件写好了 |
| 离线通过 | 语法/参数/单元测试过 |
| 后端联通 | 真起了后端并观察到就绪信号（如 `connected=True landed_state=1 clock_steps=10`） |
| **实际飞行通过** | 真跑了飞行，且有 `landed_disarmed` 之类的落地确认 |

并且：**不得用合成数据冒充仿真结果**；合成轨迹只能用于验证分析器，要显式标注。

### 接手他人产出的做法
用户会把另一个 agent（Codex）的产出目录交过来继续做。此时：
1. **先通读全部文件再动手**，明确"哪些声称已验证、哪些没验证"；
2. **不要假设已通过端到端验证** —— 按需求方的验收标准逐条实测，实际运行会扫出静态审查发现不了的缺陷
   （本次 6 个坑里有 3 个是"跑起来才暴露"的）；
3. 审查既有代码时给出**源码级证据**（如上面 MAVROS 那条），不是"我觉得应该没问题"。

---

## 六、交付物清单模板

```
<lab>/run_auto.sh                   一键入口（--check / --delay-ms N / 可选方向、高度、速度）
<lab>/README_AUTO.md                中文启动说明（前置、用法、结果、定级、限制）
<lab>/runs/<ts>_delay000/           每组一个目录
    trace.csv                       逐帧记录
    summary.json                    事件时间、结束原因、指标
    preflight.json                  起飞前只读预检快照
    environment.txt                 版本/参数/settings 快照/进程快照
    px4_console.log / mavros.log / trial_console.log
<lab>/comparison/parking_response.png + metrics.json    对照图与指标
```
脚本结尾直接打印 `RESULT_DIRECTORY=…` 并给出绘图命令，便于下一步串起来。

### 分析口径要写进交付物
- 报告的是**应用层发送时间**，不是电机实际开始制动时间 → 下一阶段用 ULog 交叉核对。
- "零附加延迟"仍含系统固有延迟；"零速度指令"≠"最大制动力"。
- 干净对照的判据：**制动阶段帧数两组应完全相同**（本次 0 ms 与 200 ms 的 BRAKE 都是 74 帧）
  → 说明差异确实只来自被隔离的那个变量。

---

## 七、批量重复层（单次跑通之后：每组 N 次 + 统计）

单次只够证明"通路能跑"，要给导师/论文用的结论必须**每组 N 次**并给出离散度。

### `run_batch.sh`（交错重复）

- **交错序列，不要跑成连续块**：基本块 `0 → 200 → 200 → 0`，重复到每组达标。
  连续跑 `0 0 0 0 0 200 200 200 200 200` 会把时间漂移混进组间差；交错能把漂移分摊到两组。
  每轮仍各自 full-restart 后端 + `simReset`（配对公平性见 §三），不要为省时间复用后端 ——
  复用会让后一组继承前一组的 EKF 状态，起点不可比。
- **保留全部运行，包括失败与异常**（用户明确要求，也是数据完整性底线）：
  - 每次运行写一行 jsonl 台账（**成功与失败都写**）：
    `index / delay_ms / status / rc / landed_confirmed / duration_s / result_dir / ts`
  - 成功：结果目录正常落 `runs/<ts>_delayXXX/`
  - **失败：单建 `runs/<ts>_delayXXX_FAILED_runN/`**，内含 `FAIL_REASON.txt`（rc、是否确认落地、耗时）、
    完整 stdout/stderr、以及 PX4/MAVROS 日志副本 —— **不丢弃任何一次运行**
  - **单次失败不得中断后续轮次**（继续跑，最后汇总时把失败数一起报出来）
  - 轮间 `sleep 3` 让上一轮的清理落定再进下一轮
- 判定"这次算不算成功"用**落地确认**（`summary.json` 里有 `landed_disarmed`），不是只看退出码。

### `summarize.py`（汇总统计）

- 按延迟分组给 **mean ± std** 与**每次原始值**（散点数据要能被引用，不能只给均值）
- **稳定性判据：组内 CV = std/mean**，`CV < 15%` 才判"一致"，作为**能否扩展到新档位（100/300 ms）的门槛**；
  CV 偏大就先加次数或排查干扰，别急着扩档
- **请求时刻状态**（速度 / 高度 / roll,pitch,yaw）逐次记录，用于**核对初始条件一致性** ——
  这是"配对是否公平"的直接证据，比口头保证强
- 四联散点图：停车距离、停止时间、纯制动时间、初始条件核对

### ⚠️ 取"请求那一刻"状态的坑（本次实际踩到）
要取请求时刻的位姿/速度，**必须取 `trace.csv` 里 `ros_s` 最接近 `request_ros_s` 的那一行**：

```python
row = min(rows, key=lambda r: abs(float(r["ros_s"]) - req))
```

**不要用 `phase == 'DELAY'` 的首帧** —— 零延迟组**根本没有 DELAY 阶段**（请求即进 BRAKE），
取空列表会落到默认值，产出**误导性的"请求时高度 = 0.000"**（看起来像数据坏了，其实是取值写错）。
凡是"某阶段不存在"的对照组，都要用**时间就近**而不是阶段名去取。

### 本次实测（1 m/s、1.5 m、ENU -Y，交错；0/100/200/300 ms 每档 5 次，20/20 成功）

| 名义延迟 | n | 停车距离 mean±std | 停止时间 mean±std | **实测延迟** mean±std | 纯制动 mean±std |
|---|---|---|---|---|---|
| 0 ms | 5 | 0.5800 ± 0.0046 m (CV 0.8%) | 1.2504 ± 0.0278 s | 0.0000 s | 1.2504 ± 0.0278 s |
| 100 ms | 5 | 0.6938 ± 0.0189 m (CV 2.7%) | 1.3680 ± 0.0168 s | 0.1110 ± 0.0021 s | 1.2570 ± 0.0168 s |
| 200 ms | 5 | 0.8314 ± 0.0361 m (CV 4.3%) | 1.4766 ± 0.0645 s | 0.2142 ± 0.0062 s | 1.2624 ± 0.0631 s |
| 300 ms | 5 | 0.9013 ± 0.0205 m (CV 2.3%) | 1.5282 ± 0.0520 s | 0.3078 ± 0.0027 s | 1.2204 ± 0.0535 s |

- **纯制动时间四档几乎不变（1.25/1.26/1.26/1.22 s）→ 延迟只推迟制动【起点】，不改制动【能力】**
  —— 这才是"干净对照"该有的结论形态（若纯制动时间也跟着变，说明隔离失败，要查）
- 初始条件四档重合：速度 1.0564~1.0589、高度 1.5404~1.5416、偏航 82.089~82.149 → 配对公平

### ★ 名义延迟 ≠ 实测延迟（本次关键发现；**不要**再归因"PX4 加减速动态"）

名义 200 ms 的逐次实测间隔是 `0.2130 / 0.2190 / 0.2040 / 0.2160 / 0.2190 s`
→ **均值 0.2142 s，比名义值多约 7%**。

原因：发布循环 **50 Hz（周期 20 ms）**，延迟判定发生在 tick 内 —— 实际生效时刻是
**第一个越过 `request + delay` 的帧**，平均多出半个周期（10 ms），再叠加调度抖动。
**这是量化/调度问题，不是飞控动态问题**（早期把它归因成"PX4 加减速动态"是错的）。

**位移分解法**（用 `trace.csv` 做梯形积分，把账算平，不要凭感觉编理由）：

```
请求后位移 = 【延迟窗口位移】 + 【制动段位移】
延迟窗口位移 = ∫ v(t)dt ，积分区间 [request_ros_s, zero_sent_ros_s]
```

| 组 | 总距离 | 延迟窗口贡献 | 制动段 |
|---|---|---|---|
| 0 ms | 0.5800 m | 0.0000 m | 0.5800 m |
| 200 ms | 0.8314 m | 0.2267 m | 0.6047 m |

物理核对（"账算平了"的证据，应吻合到 mm 级）：
```
窗口均速 1.0581 m/s × 实测延迟 0.2142 s = 0.2266 m
实测窗口位移                              = 0.2267 m     ← 吻合
100ms 0.1156 vs 0.1175   300ms 0.3249 vs 0.3252
```

**口径结论：名义口径（+0.251 m）与物理口径（0.2267 m 由 0.2142 s 产生）两者都报**，
并写明偏差来自 50 Hz 量化；想更接近名义值就把发布频率提到 100 Hz。

**推论（可迁移到任何"指令延迟型"实验）**：必须同时记录【请求时刻】与【指令实际发出时刻】
两个时间戳，否则名义值与物理值会被混为一谈；只看名义值会得出"系统固有延迟"这种含糊结论。
同理，只报"延迟 vs 距离"的一个数而不做分解，会把延迟贡献与制动段差异混在一起。

### 扩展到新档位（CV 达标之后）

- 门槛：先在已有档位把 **CV < 15%** 跑出来（本次 0.8%~4.3%），再扩档。
- 扩档同样用**交错块**：`100 → 300 → 300 → 100` 重复到每组达标
  （`SEQ="100 300 300 100 100 300 300 100 100 300" bash run_batch.sh --dir-deg -90`）。
- **不要为省时间跨档复用后端**：每轮 full-restart + `simReset`，否则后一组继承前一组的
  EKF 状态，起点不可比。
- 多档总报告用单独的 `sweep_report.py`，出**三联图**：
  ① 名义延迟 vs 停车距离（带 mean±std 误差棒）
  ② **按实测延迟** vs 停车距离（物理口径）
  ③ 纯制动时间 vs 名义延迟（**应当是平的** ← 第 ③ 张是"隔离是否成功"的直观证据）。

### ⚠️ 绘图必须用隔离 venv（本次实际踩坑）

分析脚本 `import matplotlib` 若用**系统 python3** 会直接炸：
`ImportError: Matplotlib requires numpy>=1.20; you have 1.17.4`
（ROS 环境锁死 numpy 1.17.4，**升级会破坏 ROS —— 不要动全局依赖**）。

→ 一律用项目预先建好的隔离环境，并把命令写进文档与脚本结尾提示：
```bash
PLOT_PY=<lab>/../../work/px4_plot_venv/bin/python
PYTHONDONTWRITEBYTECODE=1 "$PLOT_PY" sweep_report.py --runs <lab>/runs --output <lab>/runs/summary4
```
**纯数值统计可以不依赖它，但只要画图就必须用**；把数值分析与绘图写进同一个
内联 `python3 - <<'PY'` 里同样会踩 —— 本次就是"数值全对、最后一步画图失败"。

### 交付时把"口径"和"归属"说清
- 统计里 `n` 是多少要交代清楚：若把之前手工跑的样本也留在 `runs/` 里，总表和"严格每组 N 次"的表**分别给**，
  别让读者以为 n 就是批量次数（本次总表 n=6、严格口径 n=5）。
- 每组的 `environment.txt`（版本/参数/settings 快照）要跟着存，否则后面说不清数据来源。
