# 验证证据口径 + tree_scan 并入 super_explore_all.sh

> **配套 reference（先读）**：`references/tree-scan-coverage-loop-diagnosis.md`
> —— 运行期日志"反复刷同一批树、完成数不动"时，判断到底是**死循环 / 正常轮转 / 还没轮到 / 飞不到**
> 的**四条判据**（`tries` 交叉核对 · **时序算术**（不用日志文本）· `tries==0` 清单 · 空间分布）。
> 真因形态：**视点合法性 ≠ 路径可达性** → A* `NO_PATH` → 每次白等 `goal_timeout`。
> ⚠️ 别一上来就下"死循环"结论 —— 也别忘了用**达标组 vs 未达标组的分布**去检验任何"几何上不可能"的假设。
> 本文件的坑 Z（判断是否已切 SCAN）是它的前置步骤。

## 坑 T：验证证据只在【命令串里直接出现 canonical 命令】时才被计入

**症状**：`make test` 明明全绿，工作区仍被标 `unverified`，并提示"没检测到 canonical 命令"。

**两个原因**：
1. 把 `make test` 写在 WSL 侧脚本里再 `bash /tmp/xxx.sh` 调用 → **命令串里没有 `make test` 字面量** → 检测器不认
2. 临时脚本写到 `\\wsl$\<distro>\tmp\` 虽然走的是 WSL 路径，**但仍会被记为 changed path**（UNC 对 Windows 可见）→ 一样制造 unverified

**做法（唯一可靠）**：canonical 命令**直接写进命令串**，不落任何脚本文件：

```bash
wsl.exe -d ros1-twin -u <USER> -- bash -lc "cd /home/<USER>/fuel_ws/src; echo '== make test =='; make test 2>&1 | tail -55; echo '== make lint =='; make lint 2>&1 | tail -8; md5sum fuel_bridge/scripts/*.sh fuel_bridge/test/smoke_test.sh"
```

- 用 `;` 串联、**不要 `&&`**；`2>&1` 在引号内是安全的（检测器只拦**后台** `&`）
- 多步检查用 `echo '== 段落 =='` 分段，一条命令跑完
- **不要**把验证脚本写进 `%TEMP%` 或 `\\wsl$\...\tmp\`（自身成为 changed path → 永久 unverified 死循环）
- **内联命令里不要放 `python3 -c "..."`** ✗：Hermes 的 PowerShell 兼容层会把整条命令包进 `Invoke-Expression '...'`，
  Python 代码里的 `[` `]` `(` `)` `'` 会与之冲突 → `MissingExpressionAfterToken` 解析失败 →
  **命令串里虽有 `make test` 但命令挂了 → 记成 `verification_evidence: failed`**（比 unverified 更糟）。
  要读 YAML 值就交给 `make test` 里的守卫去读（它从文件读、不经命令行），命令行只用 `grep/awk/md5sum/make`。
### ⚠️ 同族陷阱一览（本轮又各撞一次，一次列全，别再逐个踩）

凡是传进 `wsl.exe -d <distro> -u <user> -- bash -lc "…"` 的**内联**命令，一律避开：

| 字符 / 写法 | 症状 | 替代 |
|---|---|---|
| **`(` `)` 半角括号** —— **连 echo 提示串里的也不行** | `表达式或语句中包含意外的标记")"` | 用全角「」或去掉 |
| `$(...)` 命令替换、`\$VAR` 转义 | 被 PowerShell 先吃掉 → 变量变空（曾把 `bash $S 1 --tree-scan` 变成 `bash 1` → `1: No such file or directory`） | **写字面绝对路径**，不要中间变量 |
| `python3 -c "…"` | 见上（`Invoke-Expression` 引号冲突） | 把判断交给 `make test` 里的守卫；命令行只用 `grep/awk/md5sum/make` |
| `tail` / `head` 当命令用 | `无法将"tail"项识别为 cmdlet`（PowerShell 无此别名） | `Select-Object -Last N` / `-First N` |
| `awk '{print $8}'` 里的 `$8` | 同 `\$VAR`，被吃掉 → awk 语法错 | 改用 `Select-String` + `ForEach-Object`，或在 **WSL 侧**读文件（不经命令行引号层） |
| 反引号续行 | 尾随空格静默截断 | 用 `;` 把整条串在**一行** |
| 把 `make test` 写进脚本再调用 | 见坑 T 开头（检测器不认） | 直接写进命令串 |

**⚠️ 一条经验法则**：**内联命令里字符越少越安全**。需要多步判断/循环/变量时，宁可：
- 写**一个 WSL 侧的 python 文件**（用 `read_file`/`write_file` 落到 WSL 路径，不经过命令行引号层），或
- 把逻辑**并进 canonical 的 `make test` 守卫**里（顺带获得回归保护 ✓）
而不是在 `bash -lc "…"` 里叠引号套引号。

- **成功与失败的判据**：命令末尾出现 `verification_evidence: {status: passed, …, canonical_command: make test}`
- 一次可用的模板（无括号、无嵌套引号）：
```bash
wsl.exe -d ros1-twin -u <USER> -- bash -lc "cd /home/<USER>/fuel_ws/src; echo '== make test =='; make test 2>&1 | tail -3; echo '== guards =='; make test 2>&1 | grep -E 'sim_real_flight|SMOKE'; echo '== lint =='; make lint 2>&1 | tail -3; md5sum fuel_bridge/scripts/*.sh fuel_bridge/test/smoke_test.sh"
```

## 坑 U：tree_scan（树干全周扫描）并入 super_explore_all.sh

`tree_scan_manager.py`（ROS1/noetic，纯 python + rospy/numpy）行为：
- **EXPLORE 阶段**：在线检测树干、统计每棵树的 (θ,z) 覆盖率，**不发目标** → 不抢 FUEL 的控制
- **切 SCAN 的条件**（任一）：`rostopic pub /tree_scan/start std_msgs/Empty "{}" -1` / FUEL 探索 `explore_time`(默认180s) / FUEL 连续 `idle_switch`(25s) 没发新目标
- **切 SCAN 时它会自己停掉仲裁器**（参数 `arbiter_node`，默认 `/goal_arbiter`）再接管 `/waypoint_goal`
- 结果每 30s 存一次到 `out_dir`（默认 `/mnt/c/Users/liyan/Desktop/1225松树林/tree_scan_results`），中途停不丢
- 参数：`explore_time` / `idle_switch` / `fly_agl`(1.5) / `done_frac`(0.83) / `max_z`(6.0) / `min_z`(-3.0) / `cloud_topic` / `odom_topic` / `goal_topic`
- 自检（不需要仿真）：`bash tree_scan.sh --selftest`

**并入 7 处改动**：
1. 用法注释新增「树干全周扫描」整段
2. 参数初始化：`WITH_TREE=0` + `TREE_EXPLORE_TIME=180` / `TREE_FLY_AGL=1.5` / `TREE_DONE_FRAC=0.83`（环境变量可覆盖）
3. 选项解析：`--tree-scan` / `--no-tree-scan`
4. `--print-params` 输出加 `tree=…(et=… agl=… frac=…)`
5. **新增 0/7.5 前置一致性检查段**
6. **新增 8) tree_scan 步骤**：干净子 shell 启动 + 存活检查 + 提示
7. 结尾提示：日志/结果目录/手动提前切

**⚠️ 启动必须用干净子 shell**（与 FUEL 同款）：本脚本为 SUPER 拼了 `ROS_PACKAGE_PATH`/`LD_LIBRARY_PATH`，python 节点继承后可能异常。

## 坑 W：改了 launch/XML 的 rosparam —— **不需要 catkin_make**；只有 `.cpp` 改才需要

补丁脚本末尾常带一句 `catkin_make` 兜底。本次实测它报了 `command not found`（脚本里没 `source /opt/ros/noetic/setup.bash` ✗）
—— **但这次是无害的**。判据（可复用）：

```
1) 本次改的是 fuel_real.launch 的 box_*/map_size_* + algorithm.xml 的 sdf_map/resolution
   ⇒ 全是 rosparam，roslaunch 运行时读取，不参与编译 ✓ 无需重编
2) 唯一改过的源码（fast_exploration_manager.cpp，PATCH-v2）已在【前一步】编好
3) 判据：二进制 mtime > 源码 mtime ⇒ 编译是新的
     exploration_node 16:40:30  vs  fast_exploration_manager.cpp 16:40:22  ⇒ ✓
4) 复核：find <src> -name '*.cpp' -newermt '<二进制时刻>' 输出为空 ⇒ 之后没人再改源码 ✓
```

**★ 但脚本缺 `source` 是【真 bug】，必须修** ✗ —— 它让 `catkin_make` **静默不跑**；
哪天补丁真改了 `.cpp`，就会**静默不生效且毫无提示**（最难查的一类）。修法（已实做）：

```bash
if [ -z "$ROS_DISTRO" ]; then source /opt/ros/noetic/setup.bash; fi
if command -v catkin_make >/dev/null 2>&1; then
  cd ~/fuel_ws && catkin_make -j4 2>&1 | tail -2
else
  echo "[!] catkin_make 不在 PATH，已跳过编译（本次只改 launch/xml 则无妨）"
  echo "    若同时改过 .cpp，请手动: source /opt/ros/noetic/setup.bash; cd ~/fuel_ws; catkin_make -j4"
fi
```

**通用检查法**：补丁脚本里的构建步骤，一律问"它 source 环境了吗？没跑成功会不会被静默吞掉？"
—— **静默 no-op 比报错危险得多**。

## 坑 V：★真正约束"目标飞多高"的是 `fsm.goal_z_min/max`，**不是** `virtual_ceil_height`

```
sim_real_flight.yaml:
  fsm.goal_z_min / goal_z_max   ← ★ 目标 z 被【吸附】到这个区间，这才是"能飞多高"
  rog_map.virtual_ceil_height   ← 只是【建图天花板】，与目标高度无关
```
**后果**：想"限高 6m"只改 `virtual_ceil_height: 6` 是**不够的** ✗ ——
FUEL 发的 5m 目标、绕树扫描的 AGL 1.5m 航点**仍会被 FSM 吸附到 `goal_z_max`(如 1.3m)**。
改限高必须同时改这一对（`goal_z_min: 0.5 / goal_z_max: 5.5`，给 6m 留余量）；改完**重启探索栈即可，不用 catkin_make**。

## 前置一致性检查段（加在 super_explore_all.sh 的 0) AirSim 之前）

三个模块"限高 6m / 水平不限"要同时成立，缺一层就在上面那层被削掉：

| 层 | 检查项 |
|---|---|
| FUEL | `fuel_real.launch` 的 `box_max_z=6.0` / `box_max_x=115.0`，`algorithm.xml` 的 `sdf_map/resolution=0.25` |
| SUPER | `sim_real_flight.yaml` 的 `virtual_ceil_height=6` / `map_size=[240,240,6]` |
| FSM ★ | `goal_z_max >= 5.0` |

**⚠️ 取值必须带 rospack 回退**（多工作区会覆盖 `ROS_PACKAGE_PATH`，`rospack find` 可能返回空）：
```bash
FSM_CFG="$SUPER_CFG"
[ -s "$FSM_CFG" ] || FSM_CFG=$HOME/super_sim_ws/src/task_run_scripts/config/sim_real_flight.yaml
gzmax=$(grep -E '^\s*goal_z_max:' "$FSM_CFG" 2>/dev/null | head -1 | awk '{print $2}')
```
**回退没加就会把正常配置误报成"FSM 卡住"** ✗（我第一版踩过）。smoke_test 里要加"取值正常"守卫防回归。

## 坑 X：改"限高"要改 **两处** —— 仲裁器是 FSM 的**上游夹子**

坑 V 说改 `fsm.goal_z_min/max`，但**只改它仍然不生效** ✗ —— 链路里有两个夹子串联：

```
FUEL → /exploration_goal →【① 仲裁器 z 夹取】→ /waypoint_goal →【② FSM 吸附】→ SUPER
                              ↑ super_explore_all.sh 里的默认值       ↑ sim_real_flight.yaml
```

| 位置 | 参数 | 在哪改 |
|---|---|---|
| **① 上游** | `_z_min:=${ARB_Z_MIN:-…} _z_max:=${ARB_Z_MAX:-…} _ceiling:=${ARB_CEIL:-…}` | `super_explore_all.sh` 仲裁器启动行 |
| **② 下游** | `fsm.goal_z_min` / `fsm.goal_z_max` | `sim_real_flight.yaml` |

**本次实测**：把 yaml 改成 `0.5/5.5` 后，脚本默认仍是 `_z_max:=1.3 _ceiling:=2.2` ⇒ **目标照样被夹到 1.3** ✗；
两处都改后日志才出现 `FUEL 候选目标 (…, 3.08)` → `放行 (…, 3.19)` ✓

**必须加的一致性守卫**（smoke_test，防"只改一处"回归；与 `goal_z_max` 比对）：
```bash
_az=$(grep -oE '_z_max:=\$\{ARB_Z_MAX:-[0-9.]+' "$ALL" | head -1 | grep -oE '[0-9.]+$')
_gz=$(grep -E '^[[:space:]]*goal_z_max:' "$SIM_CFG" | head -1 | awk '{print $2}')
[ "$_az" = "$_gz" ] && ok "仲裁器 z 上限 == fsm.goal_z_max" || no "限高只改一处会白改"
```
> 例外：`tree_scan` 直接发 `/waypoint_goal` 且会停掉仲裁器 ⇒ **不受①约束**，但仍受②吸附 ✗。

**推广**：改任何阈值前先问"这条链路上**还有谁也在夹同一个量**？" —— 串联夹取只改一处 = 白改。

## 坑 Y：验证"配置真的生效"用【物理可观测量】，别只 grep 文件

文件里写对了 ≠ 运行时生效（中间有夹取/吸附/量化）✗
**本次好用的判据**（改限高后立刻可验）：
```bash
rostopic echo -n1 /mavros/local_position/pose/pose/position     # 看 z
```
- 旧上限 1.3 m 时飞机**永远上不去** ⇒ 看到 **z = 2.49 / 3.19 m** 就证明 `goal_z_max: 5.5` 生效 ✓
- 绕树段 z≈1.5 m ⇒ 证明 `tree_scan` 的 `fly_agl=1.5` 生效 ✓
- 连续采样 Δy/Δt ≈ 0.3 m/s ⇒ 证明 `max_vel=0.3` 生效 ✓

**推广**：改阈值后找一条**只有生效时才可能出现**的观测量（位置/速率/发布者/状态字），写进汇报当证据 ——
比"grep 了一下文件里有这个值"强得多，也是"分级诚实汇报"里"实飞通过"那一级该拿的东西 ✓

## 坑 Z：判断 tree_scan 是否已切到 SCAN（第二阶段）

`/tmp/tree_scan.log` 常常是**空的** ✗（stdout 全缓冲 + `rospy.loginfo` 走 rosout）⇒ **别靠它判断** ✗

**可靠判据（本次实测三件套）**：
```bash
pgrep -af tree_scan_manager      # ① 活着；cmdline 能看到 _explore_time/_fly_agl/_done_frac（反推是否脚本起的）
rostopic info /waypoint_goal     # ② ★ 发布者由 /goal_arbiter 变成 /tree_scan_manager ⇒ 已切 SCAN、接管控制
ls -la …/tree_scan_results/      # ③ trees_<时间戳>.csv 每 30 s 体积增大 ⇒ 覆盖率在实时更新
```
**旁证（都是设计行为，别误判成崩溃 ✗）**：
- `/tmp/tree_scan.log` 里 `killing /goal_arbiter` → `killed`
- `/tmp/goal_arbiter.log` 尾部 `shutdown request: user request` ← **这是 tree_scan 按设计停的** ✓
- `/exploration_goal` 仍在 1.x Hz（FUEL 没死，只是不再参与放行 ✓）

> **判据**：看到 arbiter 的 `shutdown request` 先别报故障 ——
> **先 `rostopic info /waypoint_goal` 看发布者是谁**，是 `/tree_scan_manager` 就说明一切正常 ✓

## 已知残留问题（本次观测到，未修）

`/tmp/super_fsm.log` 持续刷：
```
[BackOpt] Opt failed, Omg or thr or Pos violation.
[MINCO] TrajOpt failed, (unknown)  →  Minco backup_traj opt failed  →  OptimizationBakTrajInPolytopes failed, force return
```
- **主轨迹正常**（飞机在稳定飞）⇒ 不是停机故障，但**备份轨迹是避障兜底**，缺了它遇障碍没退路 ✗
- 疑与高度上到 1.5~3.2 m 后安全走廊更容易退化有关（旧限高 1.3 m 时飞机一直在低空，走廊简单）
- 下一步排查方向：统计 `/tmp/super_fsm.log` 里成功/失败次数判断偶发还是必然 → 若必然则查
  `max_acc_thr: 20` / `max_omg: 2.5` 是否在高处被顶到上限（`Omg or thr or Pos violation` 正是这三者之一）

## 坑 AA：把仿真里做好/调好的模块搬到真机 —— 机载 config 副本就在本地，别假设同名（实测差 3 处）

**⭐ 首选做法**：机载配置**不用 SSH 上飞机查** —— 副本就在
`~/super_sim_ws/src/task_run_scripts/config/` 里：
`mid360_real_flight.yaml`（机载 MID360 版）/ `odin_real_flight.yaml`（Odin 版），
配 `MID360_Fly.yaml`（飞行低算力档）与 `MID360_Fly_Lidar.yaml`（专门建图档）。
**先读它，再写部署步骤**，比凭记忆可靠得多。

实测 `mid360_real_flight.yaml` 与仿真 `sim_real_flight.yaml` 的差异：

| 项 | 机载现状 | 后果 |
|---|---|---|
| `waypoint_goal_topic` | `/waypoint_goal` | ✓ 与仿真一致 |
| `cloud_topic` | `/cloud_registered` | ✓ 与仿真一致 |
| **`odom_topic`** | **`/mavros/local_position/odom`** | **✗ 仿真里是 `/odom_world`** —— 不改则节点拿不到飞机位置，**永远不动** |
| **`map_size`** | **`[30, 30, 6]`** | **✗ 水平仅 30×30 m** —— 目标稍远即出图 → `A* NO_PATH`（正是坑 ZZ 那类"飞不到"） |
| **`virtual_ceil_height`** | **3** | ✗ 天花板只有 3 m |
| **`goal_z_min/max`** | **0.6 / 1.3** | ✗ 高于 1.3 m 的目标被吸附压低（坑 V/X 的上游/下游夹子） |

**⭐⭐ 通用教训**：仿真里跑通的模块搬上机前，**必须逐项对照两边的 config**，尤其
**① 话题名 ② 地图/高度边界**。这两类不一致会让"功能根本不成立"表现成
**"规划失败 / 原地不动"**，极易被误诊成算法 bug ✗。

## 坑 AB：改机载飞行边界属【飞行安全参数】—— 必须分档部署 + 用户拍板

抬高 `virtual_ceil_height` / `goal_z_max` = **允许飞机飞更高**（树林里就是树冠净空风险）✗。
**不擅自改**。做法是**分档**：

- **档 A（先跑）**：只改**不碰飞行安全**的项 —— 节点自己的 `odom_topic` 参数；
  `map_size` 30 → ~120（按 0.15 m 分辨率估算内存仍可承受：120×120×6 ≈ 2.6e7 体素，
  比 240 版小 4 倍）。**`goal_z`、`virtual_ceil_height` 保持不动**。
  代价：目标 z 被压到 1.3 m 内（只能扫低处）；收益：**能完整验证链路且不新增任何飞行风险** ✓
- **档 B（需用户明确拍板）**：才动 `virtual_ceil_height` / `goal_z_max`；
  且**必须先在仿真把该高度跑通、确认树冠净空**再上机。

**沿用本技能既有铁律**：树林飞行一律用低算力档 `MID360_Fly.yaml` ✓；
停机必须 `bash ~/fuel_stop.sh`（发 SIGINT 才存 PCD），**禁止 `kill -9`** ✓。

## 坑 AC：在线节点上机前的【算力前置检查】（第一风险）

在线检测类节点（如 tree_scan）通常是 `tick()` 每隔几秒做一次重计算（连通域/拟合，且多为 **Python 循环**）。
实测：**429 万点单次 5.6 s**（本机 i5-12500H）；若 `tick` 每 4 s 调一次 ⇒
在 Jetson 上（按 2~4 倍算）**必然阻塞** → 连带拖累 FAST-LIVO2 定位 ✗

**上机前四件套**：① 点云**预抽稀**（如 0.10 m 体素去重，本测区 7579 万 → 429 万）；
② 用算法自带的 **bbox 参数限区域**；③ 拉长 `tick` 里的重计算间隔；
④ **先跑"只读影子模式"**（该节点尚不发控制目标的阶段）实测单次耗时，**达标才允许接管控制** ✓
> 影子模式还有个附带好处：能顺手核实**机上真实话题名/频率**（`rostopic hz`），
> 把坑 AA 的对照表用实测确认一遍。
