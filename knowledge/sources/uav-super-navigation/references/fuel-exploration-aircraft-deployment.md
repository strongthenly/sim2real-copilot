# 把 WSL 仿真里的 FUEL 探索栈部署到实机 fouruav（2026-10-08 实测全流程）

> 场景：WSL `ros1-twin` 上跑通了「FUEL 决策 + SUPER 执行」的探索闭环，现在要把它搬到
> 实机 fouruav（Orin NX + MID360S + D435 + FAST-LIVO2 + PX4）。**自用部署，非对外交付**。

## 配套文件（本技能目录）
- **`references/ros-topic-dataflow-verification.md`** — ★★ **通用方法**：「节点都在、话题都在，
  但功能就是不动（且无报错）」怎么查。判据只有两个：`rostopic info <t>` 的
  **Publishers 是否 `None`**、`rostopic hz <t>` 有没有数。含"从消费者嘴往回查"三步法、
  判读表、以及**仿真栈隐藏运行时依赖**的排查规程（本次两次踩坑：`waypoint_generator` 整包漏拷、
  深度图没人发布 → §2/§12）
- `scripts/build_explore_rviz.py` — 从 FUEL 的 `exp.rviz` 派生「实机自主探索」RViz（见 §10）
- ★ `scripts/mid360_fov_measure.py` — **实测雷达真实垂直视角 / 盲区 / 安装俯仰角**（见 §39/§43）。
  规格书和 AirSim 建模值都**不能当结论** ✗；`live <topic> <sec>` 模式 LiDAR 一开就能跑（不用整套栈），
  飞机**在地面静止**测最准（地面必然充满下视角 ⇒ 直方图下端截断即真实下界 ✓）。
- `templates/safe_stop_exploration.sh` — 停止探索 / 任务急停的**已知可用**模板（见 §11）
- `references/ros-startup-and-rviz-diagnostics.md` — **通用**排查手册：启动慢/`~/.ros/log`
  膨胀护栏 · 「操作员说看不到 RViz」的正确第一步（先抓屏取证）· 地面站串流本质 ·
  `xwininfo -tree` 读窗口几何 · `QT_QPA_PLATFORM=offscreen` 无显示器验 rviz 配置（见 §20）

★ **本技能的 `SKILL.md` 已顶到 100,000 字符上限** → 无法再直接 patch 追加内容
  （会被拒：`SKILL.md content is 100,xxx characters (limit: 100,000)`）。
  **新知识一律写 `references/`，指针挂到本清单里**；SKILL.md 本身待瘦身/拆分。

## ★★ 停机类脚本的两条铁律（用户明确要求过，别再犯）
1. **绝不停 FAST-LIVO2 定位链**：`FAST-LIVO2 → odom_to_mavros.py → /mavros/local_position/odom`
   → PX4 视觉定位源。树林里没 GPS ⇒ **连 POSCTL 定点都靠它**，
   断掉就是用户原话的"一瞬间失去 SLAM，飞不了定点"。
2. **只用 `rosnode kill` + 精确节点名，绝不 `pkill -f`**（pattern 有撞名误杀风险）。
   精确名的额外好处：SUPER 的 rviz 叫 `/rviz`、FAST-LIVO2 的叫 `/rviz_<数字>`
   ⇒ 杀 `/rviz` 天然只命中 SUPER 的。**并打停前/停后"保护名单快照"给用户核对**。
   ⚠️ **§6 里那段 `pkill` 写法已废弃，以 §11 为准。** 详见 §11。

## 0. 最关键的架构结论（先读这条）

实机上 **FUEL 是"决策层"，叠加在 SUPER 之上，不替代它**：

```
你切 OFFBOARD（SUPER 的 bridge 负责起飞悬停）
  → 拨/双击 CH10         : waypoint_mission 被触发（既有机制，一字未动）→ ★ 开始探索
                           （它的目标输出已改道到 /waypoint_goal_task，只当"信号"用）
  → 仲裁器看到信号        : ①发 /move_base_simple/goal 唤醒 FUEL  ②接管 /waypoint_goal
  → FUEL                 : /exploration_goal → 仲裁器 → /waypoint_goal → fsm_node 执行
```

**最终按钮表（2026-10-08 定案，5 条）**：
```
UserCmd1 "自主探索(SUPER+FUEL)"     → bash ~/fuel_explore_all.sh        ← 用户按的就是这条
UserCmd2 "停止SUPER(先定点再杀进程)"  ← ★ 原样未改
UserCmd3 "停止探索(保留SUPER)"       → bash ~/fuel_stop_all.sh
UserCmd4 "原SUPER航点任务(备用)"      ← ★ 改动前 UserCmd1 的命令原文，一字未改
UserCmd5 "★任务急停"                → bash ~/emergency_stop.sh          ← 停探索+SUPER，保留 FAST-LIVO2
```
⚠️ 这个布局与本文 §7 段首的「过程记录」不同 —— **§7 段首是走过的弯路，以本节 + §7b 为准**。

★★ `enable_ctrl` **永远 false** ★★ —— `fuel_ctrl_bridge` 在实机上**根本不启动**。
FUEL 唯一输出是一个话题 `/exploration_goal`。出问题杀 FUEL 进程即可，飞机始终在 SUPER 手里。

## 1. 输入桥是硬件无关的（重要发现）

`fuel_bridge/scripts/fuel_input_bridge.py` 订阅 `/aft_mapped_to_init` + `/cloud_registered`
—— **实机跑 FAST-LIVO2 时话题名完全一致**，输出 `/odom_world` + `/map_ros/{pose,cloud}` 是
FUEL 内部约定。**这个文件不需要任何修改**。

## 2. 打包（WSL 侧）

```bash
# 只拷 fuel_planner（FUEL/fuel_planner），排除 .bak / bag/ / *.log / log.md / test.txt
# 只拷 fuel_bridge
# ❌ 不拷 uav_simulator —— 它要 CUDA + Gazebo + rotors，实机用不上
```

### ⚠️ 坑：`waypoint_generator` 住在 uav_simulator 里，容易漏
```
FUEL/uav_simulator/Utils/waypoint_generator      ← 必须单独拷这个包！
依赖只有 roscpp / tf / nav_msgs（干净，不牵扯仿真）
而 fuel_real.launch 里有 <node pkg="waypoint_generator" type="waypoint_generator"/>
→ 漏了它 launch 起不来。补法：
   tar czf wpgen.tgz waypoint_generator
   # 上机后解到 src/fuel_planner/utils/waypoint_generator（位置随意，ROS 靠 package.xml 找）
```

## 3. 依赖（实测只需补 NLopt 一项）

FUEL 全部 `find_package`：`catkin · Eigen3 · OpenCV · PCL 1.7` —— **全是标准库**
系统依赖实测：`libarmadillo-dev ✓ libpcl-dev ✓ libeigen3-dev ✓ qtbase5-dev ✓` 都已装。

### ⚠️⚠️ 唯一的坑：NLopt（必踩）
```
fuel_planner/bspline_opt/CMakeLists.txt:
   set(NLOPT_INCLUDE_DIR "/usr/local/include")      ← 硬编码！
   set(NLOPT_LIBRARY     "/usr/local/lib/libnlopt.so")
```
- Debian/Ubuntu 的 `libnlopt-dev` **不含 `nlopt.hpp`**（C++ 接口）✗ 而 `bspline_optimizer.cpp`
  恰恰 `#include <nlopt.hpp>` → **apt 装不了，必须源码编译**
- 版本要对齐：WSL 侧 `libnlopt.so.0.11.1` ↔ **NLopt 2.7.1**（查 `/usr/local/lib/cmake/nlopt/NLoptConfigVersion.cmake`）
- **免 sudo 编译法**（agent 不需要 sudo 密码）：
  ```bash
  # 下载源（tuna 镜像优先，github 兜底）→ cmake → make -j8
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DNLOPT_CXX=ON -DNLOPT_PYTHON=OFF -DNLOPT_OCTAVE=OFF -DNLOPT_MATLAB=OFF \
        -DNLOPT_GUILE=OFF -DNLOPT_SWIG=OFF -DNLOPT_JAVA=OFF -DNLOPT_FORTRAN=OFF ..
  make install DESTDIR=/tmp/nlopt_stage           # ← 暂存，不要 sudo
  ```
  然后**只留一条 sudo 命令给用户亲手跑**：
  ```bash
  sudo cp -a /tmp/nlopt_stage/usr/local/. /usr/local/ && sudo ldconfig
  ```
- **⚠️ Hermes 会拦截 `echo pw | sudo -S`**（判定为密码爆破 ✗）→ **不要试图绕过**，
  必须让用户亲手执行 sudo 那一步。

## 4. 落位（在飞机的母仓库开分支，只做加法）

```bash
cd ~/catkin_ws
git branch --show-current                 # master
git status -s                             # 大概率有 M src/SUPER / M src/task_run_scripts（子模块本地改动）
                                          # ★ 开分支【不影响】子模块工作区改动，用户要求"别提交，保持 M"

# 记录回退基线（关键！）
md5sum devel/lib/super_planner/fsm_node   # 记下这个值

git checkout -b feature/fuel-exploration
cp -a fuel_real_pkg/src/fuel_planner src/
cp -a fuel_real_pkg/src/fuel_bridge  src/
cp -a fuel_real_pkg/launch/fuel_real_airborne.launch src/fuel_bridge/launch/
cp -a fuel_real_pkg/scripts/*.sh ~/ && chmod +x ~/fuel_*.sh

git status -s                             # 只应出现 ?? src/fuel_bridge/  ?? src/fuel_planner/
md5sum devel/lib/super_planner/fsm_node   # 必须与之前一字不差
```

**回退**：`git checkout master && rm -rf src/fuel_planner src/fuel_bridge`（子模块指针不变）

## 5. 编译

```bash
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash
nohup stdbuf -oL -eL catkin_make > /tmp/fuel_build.log 2>&1 &     # 后台，约 10~20 min
```
- catkin 会说 `traversing 30 packages`（原有 20 + FUEL 10），拓扑顺序自动排 ✓
- **"Built target X" 只表示 make 检查过该目标，不表示重新编译**（无变更时同样打印）
- **编译失败不会破坏已有产物**：devel/lib 里的旧 ELF 不动 → 回退只删新目录即可
- 编完做**完整性检查**（判据最强）：
  ```bash
  source devel/setup.bash
  for L in src/fuel_bridge/launch/fuel_real_airborne.launch \
           src/fuel_planner/exploration_manager/launch/algorithm.xml; do
    grep -oE 'pkg="[A-Za-z_]+"' $L | sed 's/pkg="//;s/"//' | sort -u | while read pk; do
      printf "  %-24s " "$pk"; rospack find "$pk" >/dev/null 2>&1 && echo OK || echo MISSING
    done
  done
  ```
  **全 OK 才算齐活**（本次就是靠它抓出 waypoint_generator 漏了）

## 6. 实机版脚本（与仿真版的关键差异）

| 项 | 仿真 `fuel_all.sh` | 实机版 |
|---|---|---|
| AirSim RPC 41451 检查 | 有 | **删** |
| `/clock` 步进检查 | 有 | **删**（实机真实时钟）|
| `twin_livo_fast.sh` 启动 | 有 | **删**（实机跑真 livox + FAST-LIVO2）|
| PX4 MAVLink `tmux send-keys mavlink start` | 有 | **删**（真飞控）|
| MAVROS `udp://:14611@127.0.0.1:14610` | 有 | **只检查，不启动**（实机 MAVROS 已在跑）|
| 起飞（seed 悬停→auto OFFBOARD）| 有 | **删**（OFFBOARD 由用户切、起飞由 SUPER 负责）|
| `fuel_cmd_topic` | `planning/pos_cmd` | **`/fuel/pos_cmd_sink`**（绝不能和 fsm_node 抢）|
| `map_size` | 240×240×11 | **120×120×6**（Orin NX 内存）|

### ★★ 停止脚本：先定点再断流（★ `pkill` 写法已废弃，见 §11）★★

**要保留的核心思想（照搬 `UserCmd2`）**：**先让飞机进入定点悬停，再断 setpoint 流** ——
否则可能触发 failsafe。
```bash
rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}' 2>/dev/null || true
sleep 1
# ↓↓↓ 下面这种 pkill 写法【已废弃】：pattern 有撞名误杀风险，
#     且 pkill -f 在 plink -m 里会自匹配。正确做法见 §11（rosnode kill + 精确节点名）
# pkill -SIGINT -f 'fuel_real_airborn[e][.]launch'
```
**⚠️ 实机差异**：仿真 `fuel_stop.sh` 会 `pkill MAVROS` ✗ —— 实机上那等于断 OFFBOARD
setpoint 流 → 可能触发 failsafe ✗ → 实机版**默认不碰 MAVROS / PX4 / FAST-LIVO2**。

## 7. 按钮接入（ch10 / UserCmd）

> ⚠️ **下面 `UserCmd4/5` 的「独立按钮」方案已被用户否掉** —— 用户明确要求
> 「**ch10 依旧作为 super 探索开始的开关**」（原话「启动方式：应该是我切换 offboard，
> 然后双击 ch10，启动自主探索，**和 super 一样**」）。
> **最终定案见本节末尾「7b」**；本段保留为过程记录，用来理解为什么走 watch 方案。

`~/catkin_ws/src/task_run_scripts/scripts/NX_code/User_define_commands.yaml`：
```yaml
UserCmd4:
  name: "FUEL自主探索(决策层)"
  command: "... && (nohup bash /home/fouruav/fuel_explore_real.sh > .../log/fuel_launch.log 2>&1 &) && echo ok"
UserCmd5:
  name: "停止FUEL(先定点再杀进程)"
  command: "... && bash /home/fouruav/fuel_stop_fuel.sh && echo done"
```
- 逻辑放 .sh，YAML 只调用 —— **避免 YAML 转义踩坑**
- 用 `(nohup ... &)` 后台化，和 `UserCmd1` 一致（否则按钮被阻塞）
- `waypoint.yaml` 里 `1 for Mavros RC ch10` —— **CH10 触发的是 SUPER 那条链**，
  FUEL 是叠加层不占那条链 → 用独立按钮

### 7b. ★ 最终定案（2026-10-08）：CH10 保持为探索开关，新层「挂」上去

**用户三次原话**：①「启动方式：应该是我切换 offboard，然后双击 ch10，启动自主探索，**和 super 一样**」
②「**ch10 之前 super 就可以触发，你不用管**」③「**②**，ch10 依旧作为 super 探索开始的开关」

**★ 设计原则（用户明确的偏好）：接新功能到既有系统 = 只做加法，不动能用的机制。**
我第一版提的是"改造 CH10 的触发机制"，被否 —— **先问清哪些是既有可用、不要去动它**。
问题本质是两层抢同一话题：
```
既有：CH10 → waypoint_mission ──┐
新增：FUEL → /exploration_goal → 仲裁器 ──┤→ /waypoint_goal → fsm_node
                                 ✗ 两个发布者抢同一话题
```
**通用解法 = 纯信号话题 + watch 仲裁器**：
1. **既有配置一字不动**：`waypoint.yaml` 原样 → 原航点任务随时可用
2. **复制一份只改话题** → `waypoint_explore.yaml`：
   `sed 's|^goal_pub_topic:.*|goal_pub_topic: "/waypoint_goal_task"|' waypoint.yaml > waypoint_explore.yaml`
   → 探索模式下 waypoint_mission 发的点变成**纯信号**，没有任何节点执行它
3. **仲裁器加 watch 模式**（`goal_arbiter.py`，本次已实现 + dry-run 实测通过）：
   新增 `~watch_topic`（空 = 旧行为）、`~trigger_topic`（默认 `/move_base_simple/goal`）
   ```python
   self.armed = not bool(self.watch_topic)   # ★ 不配 watch ⇒ 与旧版逐字一致（向后兼容）
   def on_watch(self, msg):                  # 收到信号 = 操作员按了按钮
       self.trigger_pub.publish(t)           # ① 发触发唤醒 FUEL（离开 WAIT_TRIGGER）
       self.armed = True                     # ② 此后独占 /waypoint_goal
   def tick(...):
       if not self.armed: return             # ★ 未触发前一概不放行
   ```
   **⚠️ 同批必须修 z 夹取默认值**：`z_max 1.3→5.0` · `ceiling 2.2→6.0`（见 §8「三处同改」）
4. **launch 用参数切换**：`real_flight.launch waypoint_config_path:=.../waypoint_explore.yaml`
   → **原 launch 文件不动**
5. **按钮只调脚本 + 命令原文另存**：
   ```
   UserCmd1 "自主探索(SUPER+FUEL)"    → bash ~/fuel_explore_all.sh
   UserCmd2 "停止SUPER(先定点再杀进程)"  ← ★ 原样未改
   UserCmd3 "停止探索(保留SUPER)"       → bash ~/fuel_stop_all.sh
   UserCmd4 "原SUPER航点任务(备用)"      ← ★ 改动前 UserCmd1 的命令原文，一字未改
   ```
   → 随时能切回原航点任务；`fuel_stop_all.sh` 默认**不停 SUPER**（飞机始终有人管）
6. **先 dry-run 再交付**：`_dry_run:=true` 灌一条假信号，验"收到→触发→放行"全链（零风险）

★ 好处：新层不进控制链（安全）· `.bak` + 原按钮保留（可回退）· 不破坏用户练熟的肌肉记忆

## 8. 飞行安全参数（改前必须两处同改）

`task_run_scripts/config/mid360_real_flight.yaml`：
```yaml
fsm:
  goal_z_min: 0.6
  goal_z_max: 1.3      # ← 真正的限高（不是 virtual_ceil_height）
rog_map:
  map_size: [ 30, 30, 6 ]
  fix_map_origin: [ 0, 0, 1.5 ]
  virtual_ceil_height: 3       # 建图天花板
  virtual_ground_height: -3.0
```
**★ 三者耦合**：要飞 5 m 必须同时改
`goal_z_max: 5.0` + `map_size: [30,30,10]`（原点 1.5 → 覆盖 z∈[-3.5,6.5]）+ `virtual_ceil_height: 6.0`
否则 FSM 放行 5 m 目标、规划器却看不到那片空间 → A* 找不到路。
**★ 而且仲裁器在 FSM 上游** —— 改限高**必须两处都改**（`goal_arbiter.py` 的 z 夹取参数）。

**★ 飞树林必须用低算力配置**：`MID360_Fly.yaml` 的 `dense_map_en` 必须 `false`
（`true` = 建图配置，算力约 2 倍 → 树林中位姿延迟 → 定点漂移 → 迫降）。
实测在机上发现过该值被误改成 `true` 的未提交改动。

### ⭐⭐⭐ FUEL 的 `sdf_map` 与 SUPER 的 `rog_map` —— z 语义【完全不同】，极易搞错

```cpp
// fuel_planner/plan_env/src/sdf_map.cpp:26,33
nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
mp_->map_origin_ = Eigen::Vector3d(-x_size/2.0, -y_size/2.0, mp_->ground_height_);
//                                                     ↑★★ 原点 z = ground_height（不是"地面高度"！）
// → FUEL 地图 z 范围 = [ground_height, ground_height + map_size_z]   【从下往上】
//
// SUPER rog_map:  map_size + fix_map_origin=[0,0,1.5]  → z 方向是【居中】
```
**实机教训（2026-10-08）**：照搬仿真的 `ground_height=-4.0` →
FUEL 认为地面在 -4 m、地图 z∈[-4,+2] → **`new num: 0`（找不到任何未知前沿）→ FSM 直接 FINISH**，
表现为"`/exploration_goal` 永远没数据"，看着像 FUEL 没工作，其实是参数错。
改成 `ground_height=-1.0`（配 `map_size_z=6.0` → 地图 z∈[-1,+5]，与 `goal_z_max=5.0` 对齐）
→ **立刻 `new num: 1, to visit: 1`，代价矩阵出现内容** ✓

**★ 另一个易忽略**：`virtual_ceil_height` 源码是 `if (mp_->virtual_ceil_height_ > -0.5)`
→ **负值 = 不启用天花板裁切**（所以仿真里 `-10` 是有意的，不要"顺手改成正数"）。

### ★ 实机探索模式的参数落点（2026-10-08 定案）
```
fsm (mid360_real_flight.yaml) : goal_z_min 0.6 / goal_z_max 5.0
                                map_size [30,30,10] / virtual_ceil_height 6.0
FUEL (algorithm.xml)          : sdf_map/ground_height -1.0  → 地图 z∈[-1,+5]
goal_arbiter.py 默认           : z_min 0.6 / z_max 5.0 / ceiling 6.0   ← 必须与上面两处一致
★ 改限高必须【这三处同改】，仲裁器在 FSM 上游，漏了它就是"改了一处等于没改"
```

## 9. Windows 侧 SSH 调用法（踩过才知道）

### ⭐⭐⭐ 最重要的坑：`plink -m <file>` 不等于"上传脚本再执行"
```
plink -m 是把【文件内容当命令串】交给远端 shell 执行
→ 远端进程 cmdline 里包含【整个脚本文本】
→ 脚本里任何 pkill -f '<X>' 都会匹配到脚本自己（因为文本里有 X 的明文）→ 自杀 → exit 128

实测症状：脚本只打了第一行 echo 就退出，exit code = 128
★ 正解：pscp 把脚本【落到磁盘】，再 ssh 用 bash /abs/path/script.sh 跑
   → 此时 cmdline 只有 "bash /abs/path/script.sh"，无自匹配 ✓
   → 方括号技巧 [e] 只能防"模式匹配模式自己"，防不了"模式匹配同文件里的明文"
```
```powershell
# ★ 正确姿势
cmd /c "`"C:\Program Files\PuTTY\pscp.exe`" -scp -batch -hostkey SHA256:xxx -pw <pw> `"local.sh`" user@ip:/home/user/x.sh"
cmd /c "`"C:\Program Files\PuTTY\plink.exe`" -batch -hostkey SHA256:xxx -pw <pw> user@ip `"sed -i 's/
$//' /home/user/x.sh; bash /home/user/x.sh`""
```
- **不要** `& $plink ...` ✗（Hermes 终端拒绝 `&`）
- **不要** `"C:\...\plink.exe" -batch ...` 裸写 ✗（兼容层 `Invoke-Expression` 拆坏引号）
- **不要** `echo pw | sudo -S ...` ✗（Hermes 拦为密码爆破）→ sudo 那一步必须留给用户亲手跑
- 短命令可以直接内联进 `cmd /c "..."`（用反引号转义内层引号）；但**含 `&` / pkill / 复杂逻辑一定落盘**
- 落盘脚本要 **LF 换行**（`(Get-Content $p -Raw).Replace("`r`n","`n")`）
- 中文经 plink 传输会显示成 `????`（可能影响 bash 解析）→ **脚本正文尽量纯 ASCII**

## 10. RViz：复刻仿真那套探索视图（实机用）

```bash
# 蓝本就在 FUEL 里，不用自己画：
#   fuel_planner/plan_manage/config/exp.rviz      ← 仿真探索视图（FUEL 论文风格）
#   task_run_scripts/rviz/mid360_top_down.rviz    ← 实机 SUPER 俯视图（绿色航迹在这）
```

**做法**：读 `exp.rviz` → **删掉仿真专有的三组**（`Simulation` / `Estimation` / `Tracking`）
→ **从 `mid360_top_down.rviz` 里把现成的 `Path(/waypoint_mission)` dict 整个 deepcopy 过来**
→ **手写两个 `rviz/Pose`**（`/exploration_goal` 红箭头 · `/waypoint_goal` 绿箭头）
→ 存成 `rviz/explore_fuel.rviz`。

**坑**：
- `exp.rviz` 的显示树在 `Visualization Manager.Displays`（**不是** 顶层 `Displays`）——
  取模板时要走对层级，否则 `find_class` 全返回 None，新增项静默丢失
- `exp.rviz` 里**没有 `rviz/Pose` 模板** → `/exploration_goal` 那项得**手写 dict**
  （RViz 会自动补默认值，只给 Class/Name/Enabled/Topic/Shape/Color/Alpha/尺寸 即可）
- `exp.rviz` 用的**全是标准 `rviz/*` 插件**（Axes/Grid/Group/Image/Marker/MarkerArray/
  Odometry/PointCloud2/Pose）→ **实机上不会因缺插件而崩** ✓
  （若引用了 `uav_simulator/Utils/rviz_plugins` 的自定义类，实机没编 → 必须删）
- **RViz 只在启动时读配置** → 改完要重新起（重按按钮）
- 脚本里做成可切换：`RVZ=${FUEL_RVIZ:-$S/rviz/explore_fuel.rviz}`（可临时换回俯视图）

## 11. 任务急停 / 停止脚本的正确写法（保 SAFETY）

### ⭐⭐⭐ 铁律：**绝不触碰 FAST-LIVO2**
```
FAST-LIVO2 → odom_to_mavros.py → /mavros/local_position/odom → PX4 视觉定位源
树林里没有 GPS → 连 POSCTL 定点都靠它 → 一断就是"一瞬间失去 SLAM，飞不了定点"
```
**⇒ 停止脚本【不要用 `pkill -f`】**（pattern 有撞名误杀风险）→ **只用 `rosnode kill` + 精确节点名**：
```bash
FUEL_NODES="/goal_arbiter /exploration_node /fuel_input_bridge /traj_server /waypoint_generator"
SUPER_NODES="/fsm_node /super_to_px4_bridge /waypoint_mission /map_to_world /global_path_publisher"
RVIZ_NODES="/rviz"        # ★ 精确名 —— 只命中 real_flight.launch 声明的 /rviz
                          #   FAST-LIVO2 的 rviz 叫 /rviz_179143... → 不会被碰 ✓
PROTECT="/mavros /laserMapping /livox_lidar_publisher2 /imu_tf \
/odom_to_map_broadcaster /camera_initial_to_map_broadcaster /odom_to_mavros /rosout"
```
**顺序 = 安全顺序**：① `rosservice call /mavros/set_mode POSCTL`（脱 OFFBOARD→定点悬停）
→ ② 停探索 → ③ 停 SUPER → ④ **打印停前/停后"保护名单快照"供核对**（实测证明一字未变 ✓）

### ⭐⭐⭐ 坑：脚本里写 `set -u` 会直接炸
```
/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: 行 3: ROS_DISTRO：未绑定的变量
```
ROS 的 `setup.bash` 内部读 `$ROS_DISTRO`，首次 source 时它还没定义 →
`set -u` 把它变成致命错误 → **脚本第一步就死**（按钮静默失败）。
**✅ 解：从所有 ROS 脚本里删掉 `set -u`**（`set -e` 同理慎用）。

### ⭐⭐ 坑：NX 按钮白名单是硬编码的
`task_run_scripts/scripts/NX_code/nx_command_server.py`（约第 2653 行）原本：
```diff
- elif cmd in ["UserCmd1", "UserCmd2", "UserCmd3"]:   # 只有 1~3 生效，后面加的都不响应
+ elif cmd.startswith("UserCmd"):                     # 改成动态匹配，一劳永逸
```
**改完必须重启该服务才生效**（实际跑的是 wrapper `nx_command_server2.py`）：
```bash
pkill -f nx_command_server2
cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code
nohup python3 -u nx_command_server2.py > /tmp/nxcmd.log 2>&1 &
# 或 bash ~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_qgc_servers.sh
```
> 注意 `~/NX_code/` 下还有一套**旧副本**（3 月版 12KB），正在用的是 `catkin_ws/src/...` 里那个 106KB 的。

## 12. ★★★ 头号根因：FUEL 建图靠【深度图】，仿真里是 uav_simulator 提供的

```
现象：实机上 FUEL 一切节点都在、话题都对，但 RViz 里 frontier 一片红且不更新，
      /sdf_map/depth_cloud、occupancy_* 全无数据，FSM 一直 FINISH
         ↓
真因：FUEL 的地图（plan_env/sdf_map + map_ros）靠【深度图 /map_ros/depth】做 raycast
      → exploration_node 订阅了 /map_ros/depth ✓ 但【没有任何节点发布它】✗（Publishers: None）
      → 因为仿真里深度图来自 uav_simulator 的 pcl_render_node
      → 而部署时为避开 CUDA/Gazebo 把整个 uav_simulator 排掉了 ✗
      → 连带丢掉了深度图源（和 waypoint_generator 是同一个坑！）
```
**✅ 修法（一行，不用写代码）**：把 depth_topic 直接指向实机 D435 的深度图
```xml
<!-- fuel_real_airborne.launch -->
<arg name="depth_topic" value="/camera/depth/image_rect_raw"/>   <!-- 原 /map_ros/depth -->
```
**为什么刚好匹配**（实测）：
- D435 深度的 encoding = `16UC1`（uint16）✓ 而 FUEL `map_ros.cpp:188` 正是 `ptr<uint16_t>` ✓
- `map_ros.cpp:185` `inv_factor = 1/k_depth_scaling_factor`，`depth = raw * inv_factor`
  → `algorithm.xml` 里 `k_depth_scaling_factor = 1000.0` ✓ → 毫米→米 ✓ 正好
- `depth_filter_maxdist 5.0` ✓ 匹配 D435 量程
- `skip_pixel = 2` ✓ 640×480 抽稀成 320×240 条 ray（算力可控）
- **顺手把内参也校准成 D435 实测值**（launch 里的 cx/cy/fx/fy 原来是仿真相机的）：
  `cx 319.5  cy 239.015  fx 382.721  fy 382.721`（来自 `/camera/depth/camera_info`）

**验证判据（地面就能验，不用飞）**：
```bash
rostopic info /camera/depth/image_rect_raw    # Subscribers 里应出现 /exploration_node ✓
rostopic hz /sdf_map/depth_cloud              # 应 ~10Hz ✓
grep -a "Frontier t:" .../log/fuel.log        # 应持续打印（FUEL 在实时算前沿，~1.1ms/轮）✓
```

## 13. ★★ 坑：exp.rviz 的话题名与实机这份 FUEL 不匹配

`exp.rviz`（仿真）里配的话题名，在实机这份 FUEL 上**有一半不存在**：
| exp.rviz 里 | 实际有数据的话题 |
|---|---|
| `/sdf_map/occupancy_inflate` | **`/sdf_map/occupancy_local_inflate`**（20Hz）|
| `/sdf_map/unknown` | **不存在** |
| `/sdf_map/esdf` | **不存在** |
| `/heading_planner/frontier` / `/visib` | 无数据 |
| （无） | **`/planning_vis/frontier`（Marker, 354Hz）← 真·前沿可视化** |
| （无） | `/sdf_map/occupancy_all`(7Hz) · `/sdf_map/occupancy_local`(20Hz) · `/sdf_map/depth_cloud`(10Hz) |
| （无） | `/planning_vis/{trajectory,topo_path,yaw,visib_constraint,prediction,viewpoints}` |
| （无） | `/planning/travel_traj`(9Hz) · `/planning/bspline` |

**⇒ 改 RViz 配置前先 `rosnode info /exploration_node` 看它【发布】什么 + 逐个 `rostopic hz` 测，
不要照搬仿真配置的话题名**（否则 RViz 里永远是空的，而且看不出错在哪）。

**⇒ 另外**：`exp.rviz` 里 `Simulation`/`Estimation`/`Tracking` 三组是仿真专有的
（`/map_generator/*`、`/pcl_render_node/*`、`/uart_odom`、`/Robot_1/*`、`/dynamic/*`），
实机上只会显示 "No message received"，应禁用掉，否则左侧显示树被撑得很长。

## 23. ★★★ 实机 RViz 不要自己配！照搬仿真的 `sim_fuel_style.rviz`

**踩过的最大弯路**：我在实机上凭 exp.rviz 自己拼 RViz 配置，来回改了十几轮，
颜色/透明度/重复项越改越乱，用户反馈"杂乱""闪烁""全灰""爆红"。**根子上就错了** ✗

### 正确姿势：仿真里用哪份，实机就用哪份
```bash
# 仿真的一键脚本 super_explore_all.sh 里写着：
grep -n 'RVIZ_CFG' ~/fuel_sim.../fuel_bridge/scripts/super_explore_all.sh
#   RVIZ_CFG="${RVIZ_CFG:-$RV_DIR/sim_fuel_style.rviz}"
# smoke_test.sh 断言原文："all: 默认 RViz 用 FUEL 论文风格"
#   文件：~/super_sim_ws/src/task_run_scripts/rviz/sim_fuel_style.rviz
```
**⇒ 找仿真的 RViz 配置，看它引用哪个，直接把那份搬到实机** ✓

### 仿真配置的真实内容（关键！和我之前的猜测完全不同）
```
Frame: world | View: rviz/ThirdPersonFollower  Pitch=0.21  Dist=9.3

[组] active_perception   ← ★ 整组都是 OFF（unknown / visib_frontier / ds_frontier）
[组] LocalMap
    local_map  /fsm_node/rog_map/inf_occ   ON  AxisColor sz=0.10   ← ★★ SUPER 的 rog_map
    depth_cloud /sdf_map/depth_cloud       off
[组] Mapping
    real_map   /fsm_node/rog_map/occ       ON  AxisColor sz=0.20   ← ★★ SUPER 的 rog_map
    simulation_map /cloud_registered       ON  FlatColor 161,161,161 a=0.1（很淡）
[组] Estimation
    uart_odom  /mavros/local_position/odom ON
    obj_pose_1~9 /dynamic/pose_*           ON  ← 仿真专有的动态障碍物演示
```
**★★★ 三个颠覆性认知**：
1. **仿真画面的地图点云来自【SUPER 的 rog_map】**，不是 `/sdf_map/*` ✗
   - `/fsm_node/rog_map/inf_occ`（膨胀后占据）· `/fsm_node/rog_map/occ`（原始占据）· `unk`
   - **实机上这些话题【全都有】且 2Hz 有数据** ✓（SUPER 在实机跑就有）
2. **仿真里 frontier 相关显示项【全是关的】** —— 用户要的"探索边界"在仿真画面里
   其实是靠 **rog_map 的彩色点云** 体现的，不是红色 frontier 点 ✗
3. **`/sdf_map/depth_cloud` 在仿真里是关的**（我却在实机上把它当主力 ✗）

### ★ "物块在黄/蓝之间闪烁"的真因
```
我配的：occupancy_all 8.05Hz（黄） + occupancy_local 20.01Hz（青）【都开着】
→ 两者内容大量重叠（local ⊂ all）但刷新频率差 2.5 倍
→ 同一个物块被轮流画成黄/青 → 视觉上就是闪烁 ✗
★ 仿真配置里【根本没有这两个话题】→ 搬过来就不会闪 ✓
```

### ★ "全灰"的真因
```
我用了 Color Transformer: AxisColor → 它默认配色是【黑→白灰度】✗
→ 几千个点全灰阶 → 满屏灰 ✗
（仿真配置里 local_map/real_map 也用 AxisColor，但 rog_map 发的点云自带强度/颜色字段，
  和 /sdf_map/* 的纯 xyz 点云不是一回事 → 不能照搬 Color Transformer ✗）
```

### 搬到实机的做法（只做减法）
```bash
# 1) 从仿真拷出来
cp ~/super_sim_ws/src/task_run_scripts/rviz/sim_fuel_style.rviz /tmp/out.rviz
# 2) 关掉实机不存在的仿真专有项（保持其余一字不改）
#    /dynamic/pose_0..9 · /pcl_render_node/camera_pose · /Robot_1/pose
# 3) Views.Saved 里加一个 TopDown（XYOrbit Pitch=π/2）方便在 RViz 下拉里一键切俯视
# 4) 推到实机替换 explore_fuel.rviz（fuel_explore_all.sh 默认就用这个名）
```
**验证**：`rostopic hz` 确认 4 个要显示的话题都有数据：
`/fsm_node/rog_map/inf_occ` · `/fsm_node/rog_map/occ` · `/cloud_registered` · `/mavros/local_position/odom`

## 24. ★★ 教训：不要自己"造"一份 RViz 配置

用户原话："**我之前在仿真上面的 rviz 画面，并没有这么杂乱，你去读一下仿真的配置**"
—— **该去读现成的，而不是凭 exp.rviz + 话题表自己拼** ✗
自己拼的后果：十几轮来回、颜色越改越乱、引入了闪烁（重叠话题）和全灰（AxisColor）两个自造问题 ✗

**⇒ 遇到"RViz 显示不对"，第一步永远是：找仿真/同事在用的那份 .rviz，看它引用什么话题，
逐条 `rostopic hz` 验证实机有没有；有的搬过来，没有的关掉。不要从头配** ✓

## 26. ★★★ 膨胀参数：FUEL 有 `ceil` 量化，改数值可能【完全无效】

```
源码 plan_env/src/sdf_map.cpp:434
    int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
resolution = 0.25  (algorithm.xml:33)

  0.45 → ceil(1.80) = 2 格 → 实际膨胀 0.50 m
  0.30 → ceil(1.20) = 2 格 → 实际膨胀 0.50 m   ← ★ 和 0.45 完全一样！改了白改 ✗
  0.25 → ceil(1.00) = 1 格 → 实际膨胀 0.25 m   ← ★ 只有 ≤0.25 才真降到 1 格
```
**⇒ 改这个参数前必须先算 `ceil(inflation/resolution)`，否则会"改了没效果"而误判**

### FUEL 侧三个耦合参数（同一批被 "FM" 从 ~0.2 调到 0.45）
```
algorithm.xml:38   sdf_map/obstacles_inflation       0.45 → 【已改 0.25】
                   <!-- FM: 0.199->0.45 防撞安全余量 45cm -->
algorithm.xml:107  frontier/min_candidate_clearance  0.45   ← 候选视点净空
                   <!-- FM: 0.21->0.45 -->
                   frontier_finder.cpp:722  vox_num = floor(clearance/resolution) = floor(1.8) = 1 格
algorithm.xml:139  manager/clearance_threshold       0.45   ← 管理器净空门限
                   <!-- FM: 0.2->0.45 -->
```
**只改地图厚度 `obstacles_inflation`；另两个是"候选点要留多少净空"，保持 0.45 是偏保守方向 ✓**

### SUPER 侧的膨胀（另一套，别混）
```
config/mid360_real_flight.yaml:45   robot_r: 0.35      ← CIRI 走廊硬要求（离障≥0.35m）
                          :132      inflation_resolution: 0.2
                          :133      inflation_step: 2    ← 膨胀半径 0.4m（注释: "3→2: 0.6m 室内小空间堵死绕行"）
                          :134      unk_inflation_en: false
```
**⇒ FUEL 膨胀(0.45=0.5m 实际) 比 SUPER 走廊要求(0.35m)还大 ✗
   树林里树干间隙 <1.0m 时 FUEL 会认定"不可通行" → 算不出前沿 → 假 FINISH** ✓
**缓解后（0.25=0.25m 实际）与 SUPER robot_r 0.35 匹配，且 SUPER 自己仍有兜底 ✓**

## 27. ★★ "无法规划"的诊断：先看 CIRI 报的 min dis，再看飞机高度

```
real_flight.log 里的诊断：
  -- [CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: 0.150
  -- [BackOpt] Opt failed, Omg or thr or Pos violation.
  -- [MINCO] TrajOpt failed, (unknown)
  -- [SUPER] Minco backup_traj opt failed.
```
**实机实测（飞机在地面 z=0.0075m）**：
```
24 个样本：min dis 最小/中位/最大 = 0.015 / 0.150 / 0.340 m
★ 100% 小于 robot_r(0.35) 也小于膨胀(0.40) → CIRI 走廊必然无解
★ 0.150m ≈ 【飞机机身高度的离地间隙】
  → 因为飞机在地面上，D435 把【地面】拍成点云 → 地面成了障碍物
  → 飞机就"压"在障碍物上 → 离障碍只有 15cm → 走廊无解 → 规划失败
⇒ ★ 这是【地面测试的必然现象】，不是膨胀参数的问题
   判据：起飞到 1.5m 悬停后再看还报不报 not feasible
```
**看统计的正确姿势**：
```bash
grep -ac "not feasible"          log   # 走廊无解次数
grep -ac "Opt failed"            log   # 优化失败次数（含重试，会很大）
grep -ac "SUCCESS"               log   # ★ 也要看成功次数！别只看失败
grep -a "min dis to obstacle"    log | tail -5
```

## 29. ★★★ RViz 配置检查的坑：Marker 项字段名是 `Marker Topic` 不是 `Topic`

```yaml
# rviz/PointCloud2 · Image · Pose · Odometry 的字段名是 "Topic"
# ★ rviz/Marker · MarkerArray 的字段名是 "Marker Topic"   ← 差一个字！
- Class: rviz/Marker
  Marker Topic: /planning_vis/frontier     # ← 用 "Topic" 去 grep/遍历【找不到它】✗
  Enabled: true                            # ← 漏检 → "明明没配却出现在画面上"的怪现象
```
**⇒ 检查 RViz 配置里有哪些显示项在跑，必须同时匹配 `Topic` 和 `Marker Topic`** ✓

## 30. ★★★ FUEL 的前沿可视化到底在哪：`/planning_vis/frontier`（Marker, 不是点云）

**"RViz 里 frontier 爆红"的完整答案**：
```
话题   : /planning_vis/frontier   (visualization_msgs/Marker)
发布者 : /exploration_node   → planning_visualization.cpp:23
          frontier_pub_ = advertise<Marker>("/planning_vis/frontier", 10000)

实测内容（飞机在地面，FUEL 算出 to visit:2 时）：
  ns='frontier' id=0  CUBE_LIST  【147 点】 color=(1.00,0.00,0.00) a=0.40  ← ★ 纯红 = "爆红"元凶
  ns='frontier' id=1  CUBE_LIST  【174 点】 color=(0.00,1.00,1.00) a=0.40  ← 青色（次级候选）
  其余 id=2..49 全是 0 点（清屏用的空 marker）
  scale=(0.10,0.10,0.10)  frame=world
位置: 实测 x -8.4~-6.1  y -3.4~1.1  z 0.6~4.1（是【一小簇】，不是铺满全图）
```
**⇒ 321 个 0.1m 方块 + Alpha 0.4 叠在一起 → 视觉上"糊成一片红"** ✓
**⇒ 不是 bug**：仿真配置 `sim_fuel_style.rviz` 里这个 `frontier` 项【也是 Enabled=True】，
   仿真里只是前沿少 + 视角远，看着不显眼 ✓

**⚠️ 别和这些混淆**：
```
/heading_planner/frontier  (PointCloud2)  ← 另一个！FUEL 的 frontier 点云，探索中才有
/heading_planner/visib     (PointCloud2)  ← 死代码，publish 全被注释，永远空
/fsm_node/rog_map/unk      (PointCloud2)  ← SUPER 的未知区域
/planning_vis/frontier     (Marker)       ← ★ 上面讲的那个（"爆红"= 它）
```

## 31. ★ FUEL 前沿开始有数据的标志：日志里 `to visit:` 非 0

```bash
grep -aE "new num|to visit|FSM" .../log/fuel.log | tail
#   new num: 0, new dormant: 0
#   to visit: 2, dormant: 12        ← ★ "to visit" 非 0 = 算出了可访问前沿 ✓
#   [FSM]: state: FINISH            ← 但飞机不动 → 去不了 → 仍 FINISH
```
**实测**：`to visit: 2` 时 `/planning_vis/frontier` 上出现 147+174 个方块 ✓
**⇒ 判据：`to visit > 0` 代表前沿检测在工作；`FINISH` 只代表"当前去不了"** ✓

## 31. ★★★ 转向（yaw）由谁控制 + 频繁转向怎么治

**结论：yaw 完全由 SUPER 的 `fsm_node` 控制，FUEL 碰不到。**

```
fsm_node (super_planner) → YawTrajOpt 优化 yaw
   → /planning/pos_cmd (quadrotor_msgs/PositionCommand: position + yaw + yaw_dot)
   → super_to_px4_bridge  → PX4 setpoint_raw
★ FUEL 其实【也算 yaw】(fast_exploration_manager.cpp:288 createQuaternionMsgFromYaw(next_yaw))
  但仲裁器 goal_arbiter.py 转发时 orientation.w = 1.0 → 【yaw 被强制归零丢弃】
  ⇒ 所以 FUEL 的 heading_planner 参数改了没用（改它不影响飞行）
```

**实机运行时参数**（证据 = `real_flight.log` 里 fsm_node 打印的 Load param，比看 yaml 更权威）：
```bash
grep -a "Load param super_planner" ~/catkin_ws/src/task_run_scripts/log/real_flight.log
grep -a "Max yaw rate\|Yaw control mode" ~/catkin_ws/src/task_run_scripts/log/real_flight.log
```
```
task_run_scripts/config/mid360_real_flight.yaml
    :34 goal_yaw_en: true
    :46 yaw_dot_max: 2.5      # 143 deg/s，允许很猛
    :48 yaw_mode: 2           # ★1=朝向速度方向  2=朝向目标点 ← 频繁转向的机制
task_run_scripts/config/super_to_px4_bridge.yaml
    :7  max_yaw_rate: 1.5          # 85.9 deg/s（仅速率模式用）
    :8  max_yaw_angle_rate: 0.5    # 28.6 deg/s —— 角度模式下的硬限幅（源码 :389 max_change = rate*dt）
    :13 use_yaw_angle_control: true  # → 走 ANGLE 模式，实际由 max_yaw_angle_rate 限幅
    :12 use_current_yaw: true
    :29 scan_yaw_rate: 0.0873      # 5 deg/s，仅在 scan_after_takeoff_en（默认 false）用
```

**降频繁转向的手段（按有效性）**：
1. ★★★ `yaw_mode: 2 → 1`（朝向速度方向）—— 探索时目标点频繁变，yaw_mode=2 会让机头
   跟着目标点来回甩；改成跟速度方向后，直飞时机头不动，只在拐弯时跟着转。**最治本**
2. ★★ `yaw_dot_max: 2.5 → 0.6~0.8`（SUPER 规划阶段就出慢的 yaw 轨迹）
3. ★ `max_yaw_angle_rate: 0.5 → 0.25~0.3`（bridge 硬限幅；太小会跟不上位置轨迹）
4. `goal_yaw_en: true → false`

**⚠️ ★★★ 限 yaw 转向速度：只有 `max_yaw_angle_rate` 有效，`max_yaw_rate` 是死参数**

```cpp
// super_to_px4_bridge.cpp:814-822
if (use_yaw_angle_control) {                       // ← 实机 = true
    setpoint_raw.type_mask |= IGNORE_YAW_RATE;
    current_yaw = calculateYawCommand(last_super_cmd.yaw, current_yaw, dt);  // ★走这条
    setpoint_raw.yaw = current_yaw;
} else {
    setpoint_raw.type_mask |= IGNORE_YAW;
    setpoint_raw.yaw_rate = limitValue(last_super_cmd.yaw_dot, max_yaw_rate); // ✗ 只有这条用它
}
// :384-395 calculateYawCommand —— 唯一限速点
double max_change = max_yaw_angle_rate * dt;       // ★★★ 每帧最多转这么多 = 实际硬上限
```
⇒ `use_yaw_angle_control: true` 时 **`max_yaw_rate: 1.5` 完全不起作用（改它白改）**；
真正限制飞机实际转向速度的是 **`max_yaw_angle_rate`**（0.5 rad/s = 28.6°/s）。
SUPER 侧 `yaw_dot_max: 2.5` 只决定规划器想转多快（yaw 轨迹插值密度 `interp_dt = π/yaw_dot_max`），
执行仍被 bridge 掐住 → 两者差 5 倍属"意图与执行不一致"。

**参考值**：0.5=28.6°/s（原） · 0.3=17.2°/s · 0.1745=10°/s（★用户选定，90°转9s/180°转18s） ·
0.2=11.5°/s · 0.0873=5°/s（用户给扫描选的值，注"最稳"——说明用户偏好慢转）。
★ 2026-10-09 实机已改 `0.5 → 0.1745`（10°/s）。再小会跟不上位置轨迹，转弯时机头长期滞后
→ 雷达视野偏 → 建图变差。
配套：`mid360_real_flight.yaml` 的 `yaw_dot_max: 2.5`(143°/s) 是 SUPER 规划器的目标速率，
与 bridge 的 0.1745 差 8 倍 → 存在"规划想快转、执行被掐住"的滞后感；要一致就把
yaw_dot_max 也降到 ~0.5（用户尚未拍板）。


**手持步行/静止台架**测试（判据：`/mavros/local_position/pose` 的 z 范围 < 1.3m 且
均速 ~0.2 m/s = 人拿着走），**不是飞行数据**；全机也没有 PX4 `.ulg`（在飞控 SD 卡里）。
不要拿这些数据冒充飞行结果 ✗。
- 量化工具：`~/yaw_monitor.py`（订阅 `/planning/pos_cmd` 的 yaw/yaw_dot + 
  `/mavros/local_position/pose`，实时打印"方向翻转次数/秒、yaw 总行程、行程/净变化比"，落 CSV）
- 关键指标：**yaw 总行程 vs 净变化**（悬殊 = 白转）、**方向翻转 次/秒**（= 抖）



| 级 | 做什么 | 风险 | 判据 |
|---|---|---|---|
| **L0 影子** | 只起 FUEL 决策（`enable_ctrl:=false`）。**只需传感器+定位在跑，飞机不用起飞** | **零** | 4 节点起得来 · `rosnode info /exploration_node` 订阅正确 · `/fuel/pos_cmd_sink` 的 Subscribers=None · **FUEL 日志 `new num > 0`** · CPU/内存余量。**⚠️ 不要拿"`/exploration_goal` 有数据"当 L0 判据**（飞机不动就没有 goal，见 §13）|
| L1 台架 | 通电不打桨，看目标→仲裁→fsm 链路 | 低 | `/waypoint_goal` 发布者 = goal_arbiter |
| L2 系绳/开阔 | 限高不变、小范围、急停在手 | 中 | 自主移动、无 A* NO_PATH |
| L3 树林 | 小范围、时间短 | 高 | 覆盖率上升、一键能停 |

---

## 15. ★★★ 头号坑：`sdf_map/ground_height` 是【地图原点的 z】，不是"地面高度" ★★★

2026-10-08 实测：把这个参数按仿真值原样搬到实机 ⇒ **前沿恒为 0、FUEL 立刻 `FINISH`、
`/exploration_goal` 永远没有数据**（看起来像"FUEL 坏了"，其实就差这一个数）。

源码依据（`fuel_planner/plan_env/src/sdf_map.cpp`）：
```cpp
nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
mp_->map_origin_ = Eigen::Vector3d(-x_size/2.0, -y_size/2.0, mp_->ground_height_);
//                                                          ↑↑★ 地图原点 z = ground_height ★
```
⇒ **FUEL 的地图 z 范围 = `[ground_height, ground_height + map_size_z]`**

**⚠️ 与 SUPER 的 `rog_map` 语义不同 —— 搬参数时最容易踩的正是这里**：

| 地图 | z 方向语义 |
|---|---|
| SUPER `rog_map` | `map_size` + `fix_map_origin: [0,0,1.5]` → **居中** |
| FUEL `sdf_map` | `map_origin.z = ground_height` → **从 ground_height 往上** |

实机实测：FAST-LIVO2 起点 z ≈ **0.0028**（地面就在 z≈0），而仿真搬来的是 `-4.0`
⇒ 地图整体下移 4 m ⇒ 飞机贴在"人工地面"上、raycast 全打在地面里 ⇒ `new num: 0`。

**正确取值**：`ground_height = -1.0` + `map_size_z = 6.0` ⇒ 地图 z ∈ **[-1, +5]**，
正好"地面到 5 m"，与 `goal_z_max: 5.0` 配套（这两个数要一起想）。**改完前沿立刻出现**（见 §12）。

`virtual_ceil_height` 的语义（`sdf_map.cpp:464`）：`if (virtual_ceil_height_ > -0.5) {…}`
⇒ **负值 = 不启用天花板裁切**，所以 `-10` 是"关闭"，别想当然改成 3/6。

## 16. 用 FUEL 自己的日志判断"建图到底通没通"（比看话题可靠）

`exploration_node` 在 WAIT_TRIGGER 期间每个 tick 都打印，**这四个数就是体检表**：

| 打印 | 含义 | 判读 |
|---|---|---|
| `new num: N` | 本 tick 新发现的前沿簇数 | **恒为 0 ⇒ 建图或参数坏了**（本次即 ground_height）|
| `to visit: N` | 待访问的前沿数 | >0 才会去做规划/TSP |
| `dormant: N` | 被判"暂不值得去"的前沿 | 远多于 to visit 是正常的（太远/太小）|
| `cost mat size final: (1,1),` | TSP 代价矩阵内容 | 有内容 = 前沿真的进了规划 ✓ |

```
病态（ground_height 错）: new num: 0,  to visit: 0     + [FSM] FINISH 刷屏
健康（修正之后）:          new num: 1,  to visit: 1     + cost mat size final: (1,1),
```
状态机正常流转：`INIT → WAIT_TRIGGER → (收到 /move_base_simple/goal) → FINISH|PLAN_TRAJ`。
**WAIT_TRIGGER 期间 `/exploration_goal` 无数据是设计如此**，不是故障 —— 别在这上面排查。
`no odom.` **只在启动瞬间出现 1 次是正常的**（odom 还没到），持续刷才是问题。

## 17. L0 影子模式能验什么 / 不能验什么（别把它的极限当故障）

**能验（本次全部验过 ✓）**：
- 4 个节点起得来；`rosnode info /exploration_node` 的订阅恰为
  `/odom_world` + `/map_ros/{pose,cloud,depth}` ✓
- 输出改道：`rostopic info /fuel/pos_cmd_sink` → `Subscribers: None` ✓（绝不抢 `/planning/pos_cmd`）
- `fuel_ctrl_bridge` 不存在 ✓（用 `ps -eo pid,cmd | grep -F …` 复核；**别用会自匹配的 `pgrep -f`**）
- 建图 → 前沿 → 代价矩阵 → TSP（看 §12 四个数）
- **算力余量**：Orin NX 上 FAST-LIVO2 + FUEL 并行 `load average ≈ 2.8`（8 核）、可用内存 ~10 Gi
  ⇒ **算力不是瓶颈**。只有实机能测这一条，建议每次 L0 都记下来

**不能验（本次的真实极限）**：
- **`/exploration_goal` 不会输出目标** —— 影子模式下飞机不动 ⇒ 地图不增长 ⇒ FUEL 判定
  "这一小片已探索完" ⇒ 直接 FINISH。要让 goal 出来，飞机必须真的动（L1 起）。
- 所以 L0 的结论要这样表述：**FUEL 侧链路已证明可用，剩下的是"飞机动起来"这一步** ——
  不要因为看不到 goal 就继续在 L0 里深挖。

## 18. 写 launch 文件：XML 注释里的连续 `--` 会让 roslaunch 直接拒绝

```xml
<!-- ============== -->
       -----------------            ← ★ 连续 `--` 在 XML 注释里【非法】
```
报错：`RLException: Invalid roslaunch XML syntax: not well-formed (invalid token): line 4, column 9`
（列号指的就是那串 `-` 的中间）。**照抄仿真版注释时多画一行横线就会中招** —— 用 `====` 代替。

**发货前本地必验**（别等 roslaunch 报错）：
```powershell
[xml]$x = Get-Content $path -Raw      # 不合法会抛异常
```
远端再验一次：`python3 -c "import xml.etree.ElementTree as ET;ET.parse('<path>');print('XML OK')"`

## 19. ★★★ 两个"看起来像坏了、其实是环境"的症状：启动慢 / rviz 没显示

> 2026-10-08 用户报「启动太慢」+「启动智能航线任务后，rviz 没有任何显示」。
> 这两条**都不是代码问题**，排查顺序错了会白烧一轮。

### A. 启动慢 → 先量 `~/.ros/log`
```bash
du -sh ~/.ros/log        # 实测涨到 4.9 GB / 941 个目录 ✗
```
`roslaunch` 每次启动都要先 `Checking log directory for disk usage. This may take a while.`
⇒ **GB 级日志会卡几分钟**，表现就是"没反应 / 没显示 / 像死了"。
- 机上 **`rosclean` 可能根本没装**（`rosclean check` → command not found）⇒ 从来没人清 ⇒ 一路涨
- **FIX（纯日志，零风险）**：`rm -rf ~/.ros/log/* && mkdir -p ~/.ros/log`
- **把护栏写进一键脚本**（防复发）：
  ```bash
  LOGSZ=$(timeout 15 du -sm ~/.ros/log 2>/dev/null | cut -f1); LOGSZ=${LOGSZ:-0}
  [ "$LOGSZ" -gt 1024 ] && { rm -rf ~/.ros/log/*; mkdir -p ~/.ros/log; }
  ```
- **同一类"启动慢"**：脚本里别写死 `sleep 12` 等节点 → 轮询等节点出现，秒级返回：
  ```bash
  wait_node(){ local n="$1" t="${2:-20}" i=0
    while [ "$i" -lt "$t" ]; do timeout 3 rosnode list 2>/dev/null | grep -qx "$n" && return 0
      sleep 1; i=$((i+1)); done; return 1; }
  ```

### B. 「rviz 没有任何显示」→ 先分清【配置问题】还是【没有显示器】
```
qt.qpa.xcb: could not connect to display
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" ...
→ rviz 崩溃 / 核心转储
```
**⚠️⚠️ agent 用 ssh 起 roslaunch 时【必然】踩这个**：ssh 会话没有 X display，
roslaunch 会把 `rviz` 节点也拉起来 → 立刻崩 → `/rviz` 不在 `rosnode list` 里。
**⇒ 结论：不要用 ssh 会话去判断"rviz 能不能正常显示"——那个结论无效。**
必须让用户在**飞机本机的桌面会话**上按按钮验证（脚本里自动提示，见下）。

**排障顺序（信息量由高到低）**：
1. `ps -eo pid,etime,cmd | grep 'rviz -d'` —— 有没有真 rviz 进程、跑了多久
2. `cat ~/.ros/log/latest/rviz-*-stdout.log` —— rviz 节点是 `output="log"`，stdout 在这
   （**文件存在但内容为空** = 进程被 spawn 了但秒退）
3. `tail -40 <任务日志>` —— **roslaunch 在某节点退出后会把其余节点全部 `killing on exit`**
   ⇒ **"rviz 空白"常常只是"别的节点先死了、rviz 被带着杀"，不是 rviz 自己的问题** ★
4. `echo $DISPLAY` —— 空 = 无显示器（ssh 场景）
5. **别被 `Error: Could not open file for writing.` 带偏** —— 它可能只是 `rosclean` 自己
   写不进临时文件（同样是日志撑的），是障眼法；真线索在同一段的 `Checking log directory…`

**一键脚本里加自检**（一眼定位）：
```bash
if [ -z "${DISPLAY:-}" ]; then
  say "⚠️ 没有 DISPLAY —— rviz 起不来（ssh 进来跑的？请在飞机本机按按钮）"; HAZE_DISPLAY=0
else
  say "DISPLAY=$DISPLAY ✓ rviz 可以起"; HAZE_DISPLAY=1
fi
# ★ 无 DISPLAY 时不要 wait_node /rviz，否则白等几十秒（本次实测总耗时 88s 里 40s 是白等）
```

### C. 改 `.rviz` 的安全做法（补充 §10）
基于现成 `.rviz` 派生新配置时，**不要 `yaml.safe_load` + `yaml.safe_dump` 重写整个文件** ✗
（`.rviz` 是 Qt 的类型化 YAML，整体重新序列化有加载失效风险，且难以二分定位）。
**先原样 `cp` 一份**（保证一定能加载）→ 确认可用后再做增量修改（或纯文本级追加）。
**改完必须重起 roslaunch**（RViz 只在启动时读配置，不是热加载）。

## 20. 起停 FUEL 必须拆成两个 `plink -m` 脚本（否则 exit 128 自杀）

`pkill -f 'roslaunch fuel_bridge'` 会撞上脚本自己那行启动明文 ⇒ 脚本把自己杀了，exit 128
（机制详见 `windows-ssh-automation` 的 pkill 自匹配条目；本次一天内踩了三次）。
**kill 脚本里不要出现任何"要去启动的进程名明文"**；更稳的是语义化 + 排除自身：
```bash
# kill.sh —— 只杀，没有 nohup/roslaunch 明文
rosnode list | grep -E '^/(exploration_node|fuel_input_bridge|traj_server|waypoint_generator)$' \
  | while read -r n; do rosnode kill "$n"; done
selfpid=$$                                   # ★ 排除自己
for p in $(pgrep -f 'roslaunch'); do
  [ "$p" != "$selfpid" ] || continue
  c=$(tr -d '\0' < /proc/$p/cmdline); case "$c" in *fuel_real_airborne*) kill -INT "$p";; esac
done
# launch.sh —— 只启（可含 nohup/setsid … &）
```

## 21. ★★★ 用户报「frontier 一片爆红 / 看不到边界画面」= FUEL 已 FINISH，那片红是【过期可视化】

这是本次最后才定位的症状，**最容易被误判成"算法坏了"**：

```
现象：RViz 里 frontier 一片爆红；用户说"没有显示出对应的边界画面"
真因：日志 new num: 0 / to visit: 0 / dormant: 6 / [FSM]: state: FINISH / "finish exploration."
链  ：飞机不动 → 地图不增长 → 唯一的前沿被标 dormant → 前沿数归零 → FSM = FINISH
      → FUEL【不再刷新】 frontier / unknown 的 Marker
      → 屏幕上留下的是【最后一次计算的残留】→ 一片红（★ 不是实时前沿）
```
**判据（区分"正常不动"与"真坏了"）**：
- `/map_ros/cloud` ~10Hz ✓ **且** `/sdf_map/occupancy_inflate` 有数据 ✓
  ⇒ 建图链路正常，**只差"动起来"**
- 上面任一没数据 ✗ ⇒ 才是真问题（查 `fuel_input_bridge` / remap）

**★ 不要在这上面调算法**（`ground_height` / `resolution` / 前沿阈值都不会有效）——
和 §15 是同一件事：**FUEL 侧链路已证明可用，剩下的是"飞机动起来"**。
**★ 唯一真验证 = 实飞**：拨 CH10 → 飞机真的动 → 前沿收敛成零星边界 → 那时画面才和仿真一致。

## 22. RViz / 启动类疑难 + 「没数据在流」：通用流程见同目录两份手册

本文件 §17 讲的是**本机的具体症状**；**可复用的排查流程**在两份通用手册里：

- **「节点都在、话题都在，但功能就是不动」→ `references/ros-topic-dataflow-verification.md`** ★
  （判据只有两个：`rostopic info <t>` 的 **Publishers 是否 `None`**、`rostopic hz <t>` 有没有数。
   本域已踩过两次：`waypoint_generator` 整包漏拷（吵）、深度图没人发布（静 → §12）。
   含"排除任何包之前先 grep 所有 launch 的 `pkg=` / `$(find ...)` 逐个 `rospack find` 验证"的规程。）
- 启动慢 / 操作员说「RViz 没显示」→ `references/ros-startup-and-rviz-diagnostics.md`

下面三条来自后者，最值钱：

1. **操作员说"看不到"时，先抓一帧他实际看到的画面（`mss` 抓整个 X 屏幕），并问清
   "是黑屏，还是内容不对"** —— 本次因为这个描述天然歧义（窗口没出现／被盖住／内容不对
   三种含义），我按"窗口没出现"排查了 6 轮，最后用户补一句"frontier 是爆红的"才明白
   RViz 一直正常。**别改配置，先取证。**
2. **地面站串流的本质 = 整个屏幕截图**（实测 `NX_code/rviz_full_stream.py` 就是
   `import mss` + `sct.grab(monitors[1])`，参数只是缩放）⇒ **屏幕上最上层是什么，
   操作员就看什么**；多开 rviz 会互相遮挡，**被盖住的 = "没有显示"**。
3. **读窗口真实几何用 `xwininfo -root -tree`，不要用 `-children`** —— `-children` 只给根的
   直接子窗口，会看到 `20x20` / `640x480` 的内部小窗，**极具误导**（我据此误判过一轮）；
   另外 `QT_QPA_PLATFORM=offscreen rviz -d x.rviz` 可在无显示器下验证配置能否加载
   （输出里的 Qt/OGRE/GLX 报错是 offscreen 固有噪音，**务必拿已知可用配置做对照**）。

## 23. ★★★ 用户报「双击 ch10 之后没有出现轨迹」+「rviz 黑屏」= 三个独立根因（2026-10-08 收尾）

这两条是在 §12/§13 修完之后才暴露的**下一层问题**，全部定位并修掉。逐条留档：

### 23.1 ★★★ `/waypoint_goal` 出现了【第三个】发布者：`global_path_publisher`

```
症状：CH10 拨了、仲裁器也在，但"没有轨迹"；SUPER 日志刷
      -- [Fsm] Current state: FOLLOW_TRAJ
      -- [Fsm] GLOBAL PATH received: 106 pts, roll_dist=5.0 m
真相：rosnode info /waypoint_goal →
      Publishers: * /global_path_publisher   ← ★ 我照抄原 UserCmd1 时带进来的
                  * /goal_arbiter
      ⇒ fsm 被那条 106 点【固定航点】牵着走，探索目标被它打架
```
**⚠️ 来源 = "只做加法"时的副作用**：原 `UserCmd1` 里有
`rosrun task_run_scripts global_path_publisher.py _file:=.../guidepoints_new.txt`，
抄进新脚本时没意识到**它也往 `/waypoint_goal` 发**。
**✅ 解法：探索模式的一键脚本里【不要启动 `global_path_publisher`】**（固定航点任务模式下才需要它）。

★ **通用检查（改完必做）**：`rostopic info /waypoint_goal` 的 Publishers
**必须只有 `/goal_arbiter` 一个**。多一个就是有人在抢。§7b 解决了
`waypoint_mission` 那条（改道 `/waypoint_goal_task`），这里解决的是第三条线。

### 23.2 ★★ 一键脚本必须【幂等】，否则关键节点被同名顶掉

```
arbiter 日志:
  [arbiter] ★watch 模式: 等 /waypoint_goal_task ... ✓
  shutdown request: [/goal_arbiter] Reason: new node registered with same name   ← ★★★
⇒ 第二次启动把第一次挤掉，活下来那个【没带 _watch_topic】→ 永远等不到 CH10
```
**✅ 解法：启动前先按精确节点名清一遍旧实例，再起新的**（同时也就解决了"按两次按钮"）。

### 23.3 ★★ 「黑屏」的判定：Fixed Frame 依赖的 TF 分支

```
explore_fuel.rviz  Fixed Frame = world      → 依赖 real_flight.launch 的 map_to_world 静态 TF
slam_rviz.rviz     Fixed Frame = map        → 所以它一直正常 ✓（用户的那份一直好好的）
mid360_top_down    Fixed Frame = world
```
**判据：`rosrun tf tf_echo world map`**
- 成功 → `world` 通，黑屏原因在别处（十有八九是 §19B 的"ssh 无 DISPLAY"）
- `Could not find a connection between 'world' and 'map' … two or more unconnected trees`
  → **TF 树断开** ⇒ RViz 报 "Fixed Frame does not exist" ⇒ **3D 视图全黑** ★

**排查用 `rosrun tf tf_monitor`**（能列出每个 frame 的 parent / 发布者 / 延迟），比 `tf_echo` 信息量大。
本次实测事后确认 `world exists with parent map` ✓ ⇒ 说明黑屏另有其因（就是 ssh 起 rviz 那条 ✗）。
**⇒ 顺序仍应是：① 先 `ps | grep 'rviz -d'` 看那个 rviz 是谁起的（ssh 起的必黑）→ ② 再查 TF。**

### 23.4 一键脚本的设计原则（用户明确抱怨过「按下到响应的时间太长」）

- **只「发起 + 快速确认」，不要阻塞到所有节点就绪**：本次 88s → **8s**
- 别写死 `sleep N` → `wait_node` 轮询；**无 DISPLAY 时干脆不等 rviz**（§19B）
- 给一个 **`--status` 子命令**（只查不启），用户自己复验：
  ```bash
  bash ~/fuel_explore_all.sh --status
  # 逐项 OK/缺 · 仲裁器 watch 状态 · /waypoint_goal 的发布者
  ```
- **自检项要挑关键的**：`/fsm_node /super_to_px4_bridge /waypoint_mission /rviz
  /exploration_node /fuel_input_bridge /traj_server /goal_arbiter`

### 23.5 收尾验证清单（地面就能跑，不用飞）

```bash
rostopic info /waypoint_goal        # ★ Publishers 只应有 /goal_arbiter
tail -6  ~/catkin_ws/src/task_run_scripts/log/arbiter.log   # ★ 应有"watch 模式"且无 shutdown
bash ~/fuel_explore_all.sh --status                          # 8 个节点逐项 OK

# 数据侧（FUEL 真的在干活）
rostopic hz /planning_vis/frontier            # ~370Hz 前沿 Marker
rostopic hz /planning/travel_traj             # ~9Hz   ★ 真·轨迹（RViz 里要勾这个）
rostopic hz /sdf_map/occupancy_local_inflate  # ~20Hz  已建地图
rostopic hz /sdf_map/depth_cloud              # ~10Hz  深度点云
rostopic hz /camera/depth/image_rect_raw      # ~30Hz  D435 深度图
# TF 侧
rosrun tf tf_echo world map                   # 必须成功（否则 RViz 黑屏）
```

**★ 最终交付的一句话结论**：FUEL 侧链路（深度→建图→前沿→TSP→仲裁→SUPER）
**全部已在地面验证通过**；剩下的唯一未验证项是 **"飞机真的动起来"（实飞）**。

## 41. ★★★ 高度自适应的收尾三课：树林偏好反转 · 评分函数吃掉自由度 · 树冠风险的正确表述

### ① ★★ 树林场景下「优先选更高的高度」必须【反转】✗
```
用户原方案写"优先选离地更高、更安全的高度"✗（理由=地面效应/触地风险 ✓ 开阔地成立 ✓）
但树林里恰好相反:
   低处 = 稀疏树干 ✓ 可绕 ✓（树干细、间距 2~3m）
   高处 = 树冠/枝条密集 ✗ ← 危险区在这 ✗
⇒ 【树林里低才是安全】✗ ⇒ 同分候选应取【更低】✓
★ 用户自己会追问这点（原话:"那我要到树林里岂不是一直上高度撞到树叶吗？"）——
  说明他已经在脑内跑过场景 ✓ 汇报时必须先主动讲清场景相关的偏好方向 ✗
```

### ② ★★★ 关键算法洞察：纯按「覆盖最多」排序 ⇒ 必然贴上限 ⇒ 自适应失效 ✗
```
飞得越高，同一水平距离处视点的【垂直视野范围越大】✗ ⇒ 看到的未知格越多 ✗
⇒ 纯按 visib_num_ 排序 ⇒ 永远选最高那档 ✗ ⇒ 加了高度自由度也等于没加 ✗✓✓
```
**修法（已实施 `frontier_finder.cpp` 排序 lambda，原行只有一行、替换成 8 行）**：
```cpp
sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
     [](const Viewpoint& v1, const Viewpoint& v2) {
       int m = std::max(3, (int)(0.10 * std::max(v1.visib_num_, v2.visib_num_)));
       if (std::abs(v1.visib_num_ - v2.visib_num_) > m) return v1.visib_num_ > v2.visib_num_;
       return v1.pos_[2] < v2.pos_[2];   // ★ 覆盖接近 → 取更低、更安全的
     });
```
**★★ 通用教训（可迁移到任何"加自由度"的改动）**：
给系统加**自适应自由度**时，必须检查**评分函数是否天然偏向某个极端** ✗ ——
否则自由度会被评分吃掉，改了等于没改 ✓。**先用量纲/单调性推一下偏好方向再动手** ✓。

### ③ 树冠风险的准确表述（别笼统说"会撞树"✗ —— 用户会追问细节）
```
【不会直冲树冠】✓: 雷达对上覆盖 52° ✓✓（正是它最强方向）
  → 树冠/枝条对雷达就是障碍物 → SUPER 会绕开 ✓ robot_r 0.35 + 膨胀 0.25 ⇒ 约 0.6m 余量 ✓
【真实风险三条】✗:
  ① 目标落进树冠 ✗ → SUPER 算不出可行轨迹（"not feasible"）→ 原地卡住/反复重试 ✗（不撞但白飞）
  ② 细枝/叶片点云稀疏 ✗ → 雷达扫不到 → 膨胀层做不出来 → 【这才是真可能刮到的】✗
  ③ 限位链不一致 → 莫名触发 geofence 悬停（见 §39 五处链条）✗
⇒ 汇报必须区分"直撞"与"卡住/稀疏漏检"，笼统说"会撞树"既不准确也无法据此决策 ✓
```

### ④ 机上改 C++ 的锚点三步法（本次连续踩两次 ✗）
```
✗ "锚点假设成一行": 我写 `std::sort(inserted->...` 一行 ✗ 实际源码是【分三行的 `sort(`】✗
   → 匹配失败（源码用的是 `sort(` 不是 `std::sort(` ✗ 且参数换行 ✗）
✗ 锚点末字符漏掉: XML 里行尾其实是 `/>;` ✗ 我按 `/>` 匹配 → 失败
✓ 正解三步:
  ① 先 grep -n / sed 打【原文行号】看真实文本 ✗ 别凭记忆写锚点
  ② 按【行】定位，只把【最小的那一行】替换成多行块（不 insert ✗ 避免行号位移 clobber
     别的行 —— 见 §39 里「改 Python 代码的坑」，C++ 同理）
  ③ 改完: 打印改后原文 → 重编译 → 确认产物【时间戳更新】✓
    （`make` 打 "Built target X" 不代表真的重编了 ✗ 必须看 .so / ELF 的 mtime ✓）
✓ 找不到锚点时【不要硬改】✗: 打印 `repr(原文片段)` 供下一次定位 ✓（本次靠它定位到 `sort(`）
```

---

## 40. 两个高频追问的答法 + 编译小坑（都会反复出现，先备好）

### Q1 「我的雷达是 Mid-360，应该没有死角呀？」

**答法 = 先澄清 + 给数字 + 落到实测**（★ 别直接下"扫不到"结论 ✗）：

```
· Mid-360 的 "360" 是【水平面】360° ✓；垂直只有 −7° ~ +52° ✗ —— 【不是球面全向】✗
  （设计取向：给机器人/无人机看四周+上方 ✓ 避障好用；向下只给 7° ✗）
· 最低射线(下7°)落地距离  d = h / tan7° = 8.14·h
    h=1.5m → 12.2m ✗   h=3.0m → 24.4m ✗  ⇒ ±6m 内的【地面】全在盲区 ✓
· ★ 必须区分两类目标（否则会答错 ✗）:
    树干/灌木等【竖直结构】→ 任何高度都看得见 ✓（够高，总会被某个仰角的射线打中）
    【地面 / 比飞机低 0.3m 以内的矮物】→ 近处看不见 ✗✓✓
  ⇒ "扫不完全"说的是【地面那一层】✗ 不是树干 ✓
· 证据来源要交代: Livox 官方规格 ✓ + ★用户自己的 AirSim settings.json
  (TwinLidarPreview{ VerticalFOVUpper:52, VerticalFOVLower:-7, HorizontalFOV ±180 }) ✓
  ★ 但厂商 SDK 里【没有 FOV 数值】✗（livox_ros_driver2 只有 blind_spot_set 这类设置接口 ✓）
    ⇒ 规格/建模值都不算实测 ✗ 必须量 ✓
· ★ 肉眼自检法（最快，不用脚本）: 悬停时看 RViz 正下方有没有【地面空洞】✗
    洞半径 ≈ 8.14 × 飞行高度（1.5m→~12m / 3.0m→~24m）
    若真有这个洞 ⇒ −7° 成立 ✓；若洞很小/没有 ⇒ 假设错 ✗ 立刻改参数 ✓
· 实测脚本: `~/mid360_fov_live.py /livox/lidar 15`（★放地面静止测最准 ✓
  地面必然充满下视角 ⇒ 直方图下端截断就是真实下界 ✓ 不会因"那个方向没东西"误判 ✓）
  · 若实测更大（如 −20°）→ 只改 algorithm.xml 的 down_angle 一个数 ✓【不用重编译】✓
```

### Q2 「飞机为什么一直在 3.0m 飞？是雷达的问题吗？」

**★ 答案: 是【三层叠加】，雷达只是第一环** ✓（别说成"全是雷达" ✗）

```
① 【雷达视野不对称】根本诱因: 上52°/下7° ⇒ 地图往上长得远 ✗
   ⇒ 前沿(known/unknown 边界)大量出现在【高处】✗ ⇒ FUEL 挑的视点自然在高处 ✗
② 【FUEL 视点无高度自由度】直接原因: sampleViewpoints 里 sample_pos[2] = 前沿自身高度 ✗
   ⇒ 视点被"前沿的高度"绑架 ⇒ 没有"为了看清而降低"的余地 ✗
③ 【限位压在上限】: goal_z_max=3.0 + arbiter z_max=3.0 ⇒ 高处视点全被压成 3.00 ✗

★ 硬证据（判读方法可复用）:
   arbiter 放行 107 次，统计后【几乎所有目标的 z 都恰好 = 3.00】✗（只有 2 个是 2.49）
   ⇒ 说明 FUEL 原始选择的高度普遍 ≥3m ✓（被夹取痕迹 = 恰好等于上限 ✓）
★ 还有一个与雷达无关的放大器 ✗: 后半段卡死同一目标 (2.47,0.03,3.00) ✗
   ⇒ 89~231s 高度锁死 3.08m = 191s 里的 142s（74%）
   ⇒ 实测"69% 时间在 3.0m"主要来自这里 ✓（前段 44~80s 高度其实有变化 ✓）
⇒ 四个改动（雷达模型 / 高度候选 / 放宽限位 / 修卡死）已全部落地 ✓ 见 §39
```

**★ 下一步旋钮（当时还没动）**：`frontier/cluster_size_z = 10.0` ✗ 太大
⇒ 把 10m 范围内不同高度的前沿【全并成一簇】✗ ⇒ 低处前沿永远不单独出现 ✗
⇒ 改小（如 2.0）才能让【低处前沿独立进候选表】✓ 飞机才会为它降下来 ✓

**★ 通用判读技巧**：**夹取痕迹 = 大量目标恰好等于某个限位值** ✗
```bash
grep -a "放行目标" arbiter.log | sed -E 's/.*放行目标 \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\).*/\1 \2 \3/' | sort | uniq -c | sort -rn
# z 值扎堆在某个整数（如 3.00）= 被限位夹取 ⇒ 上游原始值普遍越界 ✓
```

### 编译小坑
```
· `catkin_make --pkg <name>` 【不是有效参数】✗
· 正解: `cd ~/catkin_ws/build && make <target> -j4`，目标名先查:
    `make help | grep -iE "perception|exploration"`
    （★ exploration_manager ✗ 不是目标名；active_perception / exploration_node ✓ 才是）
· 只编一个 target 不会动到别的二进制 ✓（实测 fsm_node md5 一字未变 ✓）—— 可放心改 ✓
· ★ 改完 perception/frontier 这类库，运行时验证只需起 exploration_node 看头几行日志 ✓
    （期望: `[perception] FOV model: up=52.0deg down=7.0deg horizontal=360` +
            `[frontier] multi-altitude viewpoints: z in [0.80, 4.50] step 0.50` ✓）
```

## 39. ★★★ 高度自适应探索：把 FUEL 的相机 FOV 模型换成 Mid-360 真实视野（2026-10-10 已实施）

### 根因：FUEL 的可见性模型和真机雷达【完全不符】✗✗
```
algorithm.xml 原值: perception_utils/top_angle = 0.56125 rad = 32.2°  ← 上下【对称】✗
                    left/right = 39.7°/39.5°                          ← 水平只有 ±40° ✗
真机 Mid-360:       垂直 −7° ~ +52°（【故意不对称】）/ 水平 360°
⇒ 模型【高估了近处/低处的可见性】✗ 又【低估了水平范围】✗
⇒ FUEL 判定"这个视点能看到前沿"是错的 ✗ → 选出的视点常常看不见目标 ✗
   ⇒ 这正是「探索覆盖率只有 20~28% 就 FINISH、且原因说不清」的根子 ✓✓✓
★ 几何公式（飞机 h、目标高 th、水平距离 r 可见条件）:
     r ≤ 8.14·h              （否则最低射线(7°)从目标下方掠过 ✗）
     r ≥ (h − th)/1.28       （否则最高射线(52°)从目标上方飞过 ✗）
  ⇒ 6m 处地面(th=0) 需 h ≤ 0.74m ✗（不可行）；6m 处 0.5m 高物体需 h ≤ 1.24m ✓
  ⇒ r=2m 观察一个 z_f 高的前沿: 飞行高度 ∈ [z_f−2.56, z_f+0.25]（用户公式 ✓）
```

### 改动（3 个文件 + 配置，全部已编译验证 ✓）
```
① active_perception/src/perception_utils.cpp + .h  ★最高杠杆（一处改，全系统受益）
   对称 top_angle → 上 up_angle / 下 down_angle（不对称）+ horizontal_360 开关
   insideFOV(): 水平 360° 时只判垂直（r_h<1e-6 时正上可见/正下不可见 ✗）
   getFOVBoundingBox(): xy 半径 max_dist_ 的圆盘；z 用不对称上下视角
   加了 getUpAngle/getDownAngle/isHorizontal360/getLeft/RightAngle 取值接口 ✓
   ★ 兼容: 没给 up/down 时回退 legacy top_angle ✓

② frontier_finder.cpp ★ 顺带修了原版一个真 bug ✗✗
   findViewpoints() 里 `double left_angle_, right_angle_, vertical_angle_, ray_length_;`
   【声明了四个局部变量但从未赋值】✗ → 用未初始化随机值算 info gain ✗
   已改用 percep_utils_->getUpAngle()/getDownAngle()/getMaxDist() ✓
   ★ 编译时 `right_angle_ was not declared` 反而【证实】它本来就不是成员、纯裸声明 ✓
   垂直射线循环: 对称 ±vertical_angle_ → 不对称 −dn_a ~ +up_a ✓
   水平 360° 时: yaw 不影响覆盖 ⇒ 取全周总和、朝向指向前沿 ✓

③ frontier_finder.cpp: sampleViewpoints() 加多高度候选 ★核心功能
   `sample_pos = average_ + rc*(cosφ, sinφ, 0)` ← z 恒为前沿高度 ⇒ 无高度自由度 ✗
   → 加 `sample_pos[2] = min(zmax, zmin + iz*zstep)` 在 [0.8,4.5] 每 0.5m 取高度 ✓
   + 提前结束（viewpoints≥20 且覆盖≥95% 就 return）防 候选数×高度 组合爆炸 ✓
   + cluster UNOBSERVABLE 日志（允许高度内无任何视点可见 → 计数 + ROS_WARN ✓）

配置 ★【五处链条，少改一处就出矛盾 ✗】（2026-10-10 最终定稿值）:
   algorithm.xml            : perception_utils/up_angle=0.90757(52°) · down_angle=0.12217(7°)
                              · horizontal_360=true
                              · frontier/candidate_zmin|zmax|zstep = 0.8 | 3.0 | 0.5
   mid360_real_flight.yaml  : goal_z_min/max = 0.8 / 3.0        （SUPER FSM 也夹高度 ✗）
   fuel_explore_all.sh:112  : _z_min:=0.8 _z_max:=3.0 _ceiling:=3.5 _xy_limit:=6.0
   super_to_px4_bridge.yaml : geofence z −1.0 ~ 3.8             （物理兜底，≥目标上限 ✓）
   地图天花板               : SUPER virtual_ceil_height 6.0
                              ★ FUEL map_size_z 【实际 6.0】⇒ 地图 z∈[−1,5] ✗不是10！
                                （fuel_bridge/launch/fuel_real_airborne.launch:21 ✓
                                  别抄 exploration_manager/launch/exploration.launch 的 10.0 ✗
                                  那是【没被 include 的副本】⇒ 见 §42 的运行时常量教训 ✗✗）
   ★ 约束链: goal_z_max ≤ arbiter _z_max ≤ _ceiling < geofence_z_max < 天花板
             且 box_z ⊆ FUEL 地图 z ✓
   ★★★ 完整清单是【8 处】（§42 有表）—— 只对 5 处仍会漏 ✗
   ★★★ 本次踩到: 把 goal/candidate 放宽到 4.5，但【geofence 还停在 3.8】✗
        ⇒ 飞到 3.8~4.5 会触发"越界→保持悬停" ✗ 现象 = 飞机【莫名停在半空】✗
        ⇒ 用户拍板 **限高 3.0**（树林保守）→ 五处对齐后才一致 ✓
   ★ 改高度前先把这张表列出来逐项核 ✗（比改完再回头查便宜得多）
     clamp_z 已确认是【纯夹取】max(z_min, min(hi, g[2])) ✓ 不强制值 ✓
     ⇒ 所以"高度自适应"只靠把限位放宽即可 ✓ 仲裁器代码不用改 ✓
```

### ★★★ 血泪教训：机上雷达 FOV 必须【实测】，不能拿仿真/规格当结论（2026-10-10）

```
用户质疑「检查一下雷达点云确定扫不到吗？这个和仿真不一样是实机」→ 完全正确 ✓
① ✗ 上次飞行【没存点云】: pcd_save_en=false
   （证据: log 下目录名 fastlivo2_20261010_202922_false ← 尾部 _false ✓）
   ⇒ 想用飞行点云回看，必须先把 pcd_save_en 打开 ✗✗（否则永远无数据可查 ✓）
② ⚠️ 机上有 4 个 bag，但都是 odin_data_*.bag = 【odin 设备的 dToF 雷达】✗
   不是机上的 Mid-360 ✗ ⇒ 拿它量出的视角（实测 −20°/+72°）【与 Mid-360 无关】✗
   ★★ 教训: 别拿"机上的 bag"当"这个雷达的数据" ✗ 先确认是哪个传感器录的 ✓
③ 我的 −7°/+52°/360° 出处: 用户自己的 AirSim settings.json
   TwinLidarPreview{ VerticalFOVUpper:52, VerticalFOVLower:-7, HorizontalFOVStart/End:±180 }
   + Livox Mid-360 官方规格 ✓ —— 是【建模值/规格值】，不是实机实测值 ✗
```

**★ 实测方法（已备好脚本 `~/mid360_fov_live.py`，LiDAR 一开就能跑 ✓ 不需要整套栈）**：
```bash
python3 ~/mid360_fov_live.py /livox/lidar 15      # 飞机在地面静止测最佳 ✓
#   → 打印 1° 分箱的仰角直方图 → 两端【陡峭截断】= 真实 FOV 边界 ✓
#   → 同时给最近点距离（真实盲区）+ 反推安装俯仰角 ✓
```
**★ 为什么"地面静止测"最靠谱**：地面必然充满下视角 ✗ 不会因为"场景没东西"而误判 ✓
⇒ 截断位置就是真实下界 ✓✓✓

**★★ 我的实现让这个修正零成本**：up/down 是 `algorithm.xml` 的**参数** ✗
```xml
<param name="perception_utils/up_angle"   value="0.90757"/>  <!-- 52° -->
<param name="perception_utils/down_angle" value="0.12217"/>  <!-- 7°  -->
```
⇒ 若实测是 −20°，只改这个数（−20° = −0.34907 rad）✓ **不用重编译** ✓✓✓
   同时 `candidate_zmin` 也可以下探（0.8 → 0.5）✓ 因为盲区没那么大 ✓


```bash
cd ~/catkin_ws/build && make active_perception exploration_node -j4     # 目标名要 make help 查 ✗
# ★ 运行时验证（临时 master 11399，不碰真机 11311）:
rosparam load <algorithm.xml> 失败时→单独起节点会用默认值✗
rosrun exploration_manager exploration_node
# ⇒ 期望看到（本次已确认 ✓）:
#   [WARN] [perception] FOV model: up=52.0deg down=7.0deg horizontal=360
#   [WARN] [frontier] multi-altitude viewpoints: z in [0.80, 4.50] step 0.50
# 之后再因 map size=-1 崩 Astar 是【正常的】✗（没加载 algorithm.xml → 负数尺寸向量）
```
**★ 教训**：单独起节点看到 `map size: -1 -1 -1` 后 `std::length_error vector::_M_default_append`
= 参数没加载 ✗，不是代码 bug ✓ —— 看日志要分清新模型日志 vs 参数缺失崩溃 ✓



### 分层判读结论（4 个独立机制，全部有数据）
```
① ✓ 探索范围/高度限制【完全有效】: 飞行段越界样本 = 0 次 ✓ 高度稳 3.08m ✓ 夹取 0 次 ✓
   （FUEL 目标全在 ±6 内 ⇒ 用户的 box ±6 改动生效 ✓）
② ✓ 坐标系【彻底排除】: Δxy 均值 0.02 / 峰值 0.22 m ⇒ FUEL odom 与真实位置一致 ✓
   ⇒ 「红轨迹不跟飞机」不是坐标问题 ✓
③ ✗ 徘徊 = arbiter 重复放行同一个点: 放行 107 次里 62 次都是 (2.47,0.03,3.00) ✗
   原因分布「已到达当前目标」占 99/107 ✗
④ ✗ 无新轨迹 = FUEL 先 FINISH（`from PLAN_TRAJ to FINISH`），不再发新目标 ✗
   但 arbiter 的"保持超时 20s 强制放行"仍在推同一个点 ✗ → 叠加成"一直徘徊" ✗
```

### ★★★ arbiter 的设计缺陷（我写的）
```python
# 原逻辑（错）:
if arrived:  reason = "已到达当前目标"        # 到了就无条件放行 → 放行的还是同一个点 ✗
elif shifted and dt >= min_interval: ...
elif is_stuck(dt): ...收缩...
elif dt >= max_hold: reason = "保持超时"     # 目标没变也照放 ✗
if reason: 放行; self.cur_goal = 同一个点
# ⇒ 到达 → 放行同一点 → 飞机本来就在旁边 → 又"到达" → 又放行 ✗ 死循环 = 原地徘徊
```
**修法（已实施）**：加 `goal_same_eps`(默认 0.3m) + `changed = dist(candidate, cur_goal_raw) > eps`；
`arrived` 与 `max_hold` 两个分支都改成 **`if not changed: return`**（FUEL 没给新目标 → 不重复放行，保持悬停 ✓）。
**★ 关键**：`elif shifted…` 分支必须保留 ✗（它才是"FUEL 给了新目标"的正常放行路径 ✓）。

**★ 测试（临时 master 11399 + 假数据，dry_run）：**
```
测试1 目标不变(飞机就在目标点): 放行 = 1 次（仅冷启动）✓  修前会几十次 ✗
测试2 10s 后给新目标:          放行 = 2 次 ✓ 新目标照常放行 ✓ 没改坏功能 ✓
```

### ★★★ 改 Python 代码的坑：**用 insert 改多行会让后续行号位移，覆盖掉别的行** ✗
```
✗ 错: new[i+1]=A; new.insert(i+2,B); new[i+3]=C   ← i+3 原本是【别的分支的行】✗
   → 把 `elif shifted and dt >= self.min_interval:` 顶掉了 ✗
   → py_compile 竟然【通过】(语法合法，只是逻辑死了) ✗✗✗
✓ 对: 把"要替换的那一行"【赋成含 \n 的多行字符串】（不 insert/不位移）✓
   new[i] = "if not changed:\n<ind+4>return\n<ind>reason = ..."
★★ 校验必须【grep 关键分支行是否还在】+ 人工看 diff 全文 ✗ 不能只看 py_compile ✓
   grep -n "elif shifted and dt is not None and dt >= self.min_interval:" goal_arbiter.py
```

### ★★ 用 pos_monitor CSV 定量复盘飞行（很有用，注意剔除落地段 ✗）
```python
# ★ 落地后位置会漂移(FAST-LIVO2 静止漂移/被搬动) → 必须只统计【飞行段 z>=1.0】✗
#    本次: 全程 Y 到 -11.41 ⇒ 看着像"飞出 ±6 一倍" ✗
#          飞行段(z>=1.0) Y 只有 -3.91~5.85 ⇒ 0 次越界 ✓✓✓
fly = [r for r in rows if r[3] >= 1.0]
# 覆盖: 0.5m 栅格 20.1% / 1.0m 栅格 27.8%  ⇒ FUEL 3 分钟只探了 ±6m 的约 1/4~1/3 ✗
```
**读 CSV 前必须清洗 NUL** ✗（监控被强杀会留下 NUL 空洞）：
`open(p,'rb').read().replace(b'\x00',b'').decode('utf-8','ignore')`

### ★ 本次遗留（下一步）
- **FUEL 只覆盖 20~28% 就 FINISH** ✗（`to visit`→0, `dormant`=3 ⇒ 前沿全判休眠 ✗）
  ⇒ 可能不是真探完，而是【LiDAR 下视角只有 -7°】+ 前沿可达性判定 ✗ —— 值得查
- **geofence 在落地后产生 784 次噪音** ✗（z≈0.7 仍按 x/y 判越界 → 一直触发"保持悬停"✗）
  ⇒ 可选改进: `checkGeofence` 加 `if (current_pose.z < 0.5) return true;`（落地不判 x/y ✓）
- **arbiter 只约束 FUEL 目标** ✗ —— SUPER 自己的航点任务/RViz 点击目标【不经过它】✗
  ⇒ 这正是 geofence 必须存在的原因 ✓（本次它确实拦住了越界 ✓ 打印了 hold ✓）



**需求**：同时看实时地图与累计地图（看探索时地图积累）+ 上帝视角覆盖 ±6m。

### 先查清「哪个话题是真累计」——三个候选只有一个合格
```
✗ /Laser_map (FAST-LIVO2) —— 【死话题】！LIVMapper.cpp:215 只有 advertise，
   全文件【没有任何 .publish 调用】✗ → 永远不发数据 ✗
   ★ 教训：判断话题可用性要 grep 【.publish 调用点】，不能只看 advertise ✗✗✗
   （FAST-LIVO2 这份配置下实际只发 /cloud_registered = 当前帧 ✗ 不发布累计点云 ✗）
✗ /fsm_node/rog_map/occ · inf_occ (SUPER) —— rog_map 开了 map_sliding(threshold 0.3) ✗
   ⇒ 滑动 = 局部窗口 ✗ 不是累计 ✗
✓ /sdf_map/occupancy_all (FUEL) —— sdf_map.cpp:33 map_origin 固定(-x/2,-y/2,ground) ✓
   无 slide 逻辑 ⇒ 固定 30×30 范围内的【累计占据】✓ 随探索一直增长 ✓ 8Hz 很轻(实测 4040 点) ✓
```

### 落地（`explore_fuel.rviz` 的 Mapping 组）
```
★实时地图(当前帧)              /cloud_registered        FlatColor 161,161,161(灰) α1
real_map(SUPER 滑动局部)       /fsm_node/rog_map/occ    AxisColor(保留原样)
★累计地图(已探索,随探索增长)    /sdf_map/occupancy_all   FlatColor 255,215,0(黄) α0.65 sz0.10
⇒ 灰=当前帧、黄=累积区域，对比一目了然 ✓
★ 只开 occupancy_all，【不要同时开 occupancy_local】(20Hz) ✗ —— 两者重叠会来回闪 ✗（§旧坑）
```

### 上帝视角（覆盖 ±6m）
```yaml
Current:
  Class: rviz/XYOrbit
  Distance: 18            # RViz FOV 0.785rad(45°) → 半宽 = 18*tan(22.5°) = 7.45m ⇒ 覆盖 ±6m ✓
  Pitch: 1.5708           # 90° 正俯视
  Yaw: 0
  Focal Point: {X: 0, Y: 0, Z: 1.5}   # ★对准探索起点(arbiter origin 实测 ≈ (0.01,0.05)) ✓
  Name: ★上帝视角(±6m)
  Value: XYOrbit (rviz)
```
**★ 还要在 `Saved:` 里存一份同名的** → 用户在 Views 下拉里可一键切回 ✓（本次 Saved 现有 6 个）

### ★★★ 改 .rviz 的致命坑：文本插项的锚点必须包含【前一项的完整尾部】
```
✗ 错误做法：锚点从「前一项中间的某行」开始（如 `      Topic: /fsm_node/rog_map/occ\n...`）
   → 新项被插进【前一项的中间】✗ → 前一项的后半段属性被"吞"成新项的
   → 结果: real_map 丢了 Topic ✗，新项话题错成 rog_map/occ ✗✗
✓ 正确做法：锚点要包含前一项的【最后一行】+ 后一项/组属性，从 item 边界处插
   例: 锚点 = `      Value: true\n    Enabled: true\n    Name: Mapping\n`（`Name: Mapping` 保证唯一）
   替换 = `      Value: true\n` + 新项 + `    Enabled: true\n    Name: Mapping\n`  ← 别漏掉 Value: true ✗
★ 且 .rviz 里有 `&id001` YAML 锚点 → 【不能 yaml.safe_load+dump 整体重写】✗（会破坏锚点）
  ⇒ 只能做定向文本替换；改完【用 yaml.safe_load 只读校验】 ✓
★★ 校验方式（必做）：遍历 Displays，打印每一项的 [组]/Name/Enabled/Topic，
   确认【名字↔话题 一一对应】✓ —— 本次就是靠它抓到错位 ✗



**现象**：用户报「FUEL 红色轨迹不以真实飞机位置为准 / 飞机在一个地方来回摇摆徘徊」。

**根因（从 arbiter 日志直接量出）**：
```
arbiter.log: 放行 183 次，统计后
    42 次 → (-5.99, -5.95, 2.70)
    30 次 → (-5.99, -5.95, 2.76)
    30 次 → (-5.99, -5.95, 2.68)   ... 绝大多数是【同一个点】
而 FUEL 的候选目标: y 全在 -9 ~ -13（例 (-6.55,-12.51) (-5.71,-13.51) (-9.88,-9.44)）

机制: 当时 FUEL 的探索 box 还是 ±60m ✗，而 arbiter 已收紧到 ±6m ✓
  → FUEL 合法地选区域外的点
  → clamp_xy 把它夹到【x 边界 ∩ y 边界】那个角点 (-5.99,-5.95)
  → 反复放行同一点 → 飞机被来回指向同一处 → 原地摇摆 ✗✓✓✓
```
**⇒ 修法：把 FUEL 的 box 收到【比 arbiter 的夹取范围更小】**
（`fuel_real_airborne.launch` 的 `box_min/max_x/y`：±60 → ±6；建议再留余量，如 box ±5 / arbiter ±6，
否则目标会正好落在夹取边界线上，仍被夹但不会全挤到一个角 ✓）

**★ 诊断手法（很通用）**：
```bash
grep -a "放行目标" arbiter.log | sed -E 's/.*放行目标 \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\).*/\1 \2 \3/' | sort | uniq -c | sort -rn | head -20
grep -a "FUEL 候选目标" arbiter.log | head -25     # 看 FUEL 想去的点是否在区域外
grep -ac "Geofence violation" real_flight.log      # 物理兜底是否被触发
```
**若「放行目标」高度集中在少数几个点 ⇒ 就是夹取把目标挤到边界** ✓

## 35. 到达判定补丁（用户自加 `[PATCH-arrival]`）的审查要点

用户在 `fast_exploration_fsm.cpp` + `algorithm.xml` 加了「目标保持到真实飞机到达 / 超时兜底」：
```bash
# algorithm.xml
fsm/replan_on_arrival=true   fsm/arrive_r=0.5   fsm/arrive_timeout=8.0
# fsm.cpp 里 若 (goal - fd_->odom_pos_).head<2>().norm() < arrive_r → 换下一个目标；超时也换
```
**审查结论**：
1. **方向正确** ✓ —— FUEL 原本每次 replan 都换目标 → 目标抖动；保持到到达能消除抖动。
2. **`arrive_r` 必须 ≥ arbiter 的 `arrive_radius`(0.5)** ✗ —— 否则 FUEL 比 arbiter 更早认为
   "到了"，两者判定不一致。0.3 太小（飞机定点精度 ~0.2~0.4m + 风 → 够不到 → 全靠 timeout）✗；
   建议 0.5~1.0 ✓（用户最终取 0.5）。
3. **`arrive_timeout` 要 ≥ 一段行程耗时** —— v=0.6 m/s 时 6m 行程要 10s，5s 会在半路换目标 ✗；
   建议 15~25s ✓（用户最终取 8.0，偏短，需飞行验证）。
4. **★ 代码默认值 ≠ yaml 值，yaml 生效**（源码 `0.6/40.0` vs yaml `0.5/8.0`）→ 改 yaml ✓
5. 判据用的是 `fd_->odom_pos_`（= `/odom_world` = FAST-LIVO2 位姿），
   与真实定位**同源** ✓；若显示上红轨迹不跟飞机，那是【坐标系/显示】问题，不是这里 ✗。

**★★ 改完必须重编译 + 核对二进制时间**：
```bash
ls -l devel/lib/exploration_manager/exploration_node   # 二进制时间要 ≥ .cpp 时间
grep -ac "PATCH-arrival" log/real_flight.log           # 日志里出现才算真跑过
```
**★ 日志时间 vs 改动时间**：先比时间戳再下结论 ✗ —— 本次用户 16:42 改完，日志是 15:16 的，
**日志验证不了改动** ✗，必须再飞一次。

## 36. `~/fuel_pos_monitor.py`：位置一致性监控（已接进一键启动）

**一次飞行同时回答**：① FUEL 认为的位置 vs 真实位置差多少 ② 红轨迹起点 vs 真实位置差多少
③ 目标链（FUEL 原始 → 仲裁器夹取 → SUPER 指令）④ 夹取位移多大。

同时订阅 6 路并落 CSV：`/odom_world`（FUEL 位置）· `/mavros/local_position/pose`（真实位置）·
`/exploration_goal`（FUEL 原始目标）· `/waypoint_goal`（夹取后）· `/planning/pos_cmd`（SUPER 指令）·
`/planning/travel_traj`（**红轨迹 Marker**，取 `points[0]` = 轨迹起点 = `fd_->odom_pos_`）。

**★★ 两个必须记住的坑**
1. **必须 `source ~/catkin_ws/devel/setup.bash`** ✗ —— 否则 `import quadrotor_msgs` 失败，
   脚本**静默崩溃** ✗（只写进日志，按按钮的人完全看不出来）。已在脚本里做 try/except 降级
   （缺 msgs 只跳过 pos_cmd，其余 5 路照常）+ 打印醒目警告。
2. **固定节点名 `fuel_pos_monitor`**（不用 `anonymous=True`）→ 停机脚本可用
   `rosnode kill /fuel_pos_monitor` 精确停 ✓（符合"零 pkill"铁律）。

**已接进一键流程**：`fuel_explore_all.sh` 里作为 `[4/4]` 后台启动
（`nohup python3 ~/fuel_pos_monitor.py "$LOG/pos_monitor.csv"`），并加入 `NODES_MINE` 幂等清理；
`fuel_stop_all.sh` 的 `FUEL_NODES` 也加了它 ✓。

**★ 部署前必须做的冒烟测试（用假数据，不碰真机栈）**：
起一个临时 master（`roscore -p 11399`，**绝不用 11311** ✗）+ 假发布器，喂已知数值，
对照手算验证输出（Δ、红轨迹差、夹取位移）、CSV、统计、清理。本次正是靠它抓到了
`quadrotor_msgs` 缺失导致静默崩溃 ✗ —— **任何"按按钮后台跑"的脚本，交付前都该这样验一遍** ✓。

### ★★★ 后台 Python 脚本的两个通病（2026-10-10 实机踩到）

```bash
# 现象: /goal_arbiter 与 /fuel_pos_monitor 节点都在跑 ✓，但它们的 .log 都是【0 字节】✗
```
1. **stdout 被缓冲** ✗ —— `nohup python3 x.py > x.log` 时 Python 默认块缓冲，
   长期运行从"看不到任何输出"。**修法**（二选一）：
   - `python3 -u x.py`（自带脚本）
   - `nohup env PYTHONUNBUFFERED=1 rosrun pkg x.py`（**rosrun 的脚本**只能这样加 ✗ 不能传 `-u`）
2. **只在退出时写文件** ✗ —— 若脚本异常退出/被 kill，数据全丢。
   **修法**：改成**周期性增量落盘**（每 N 秒 `open(OUT,'a')` 追加 `rec[_written:]`，
   用全局 `_written` 记账 ✓；启动时先写表头 ✓）。这样飞行中随时能 `scp` 走 ✓。

**★ 验证"改完真的在产出"**（不能只看文件存在 ✗）：
```bash
ls -l  pos_monitor.log pos_monitor.csv        # 大小必须 >0
head   pos_monitor.log                        # 必须有实时内容
wc -l  pos_monitor.csv                        # 行数必须随时间增长
rosnode info /fuel_pos_monitor | sed -n '/Subscriptions/,/Services/p'   # 6 路订阅必须齐
```
**★ 重启只读监控节点零风险**：`rosnode kill /fuel_pos_monitor` 后重起即可
（它只订阅、不发任何 setpoint ✓）；**必须同时快照保护名单确认没碰 FAST-LIVO2/mavros** ✓。

**★★ 早期实测结论（静止状态）**：`odom_world`(FUEL) vs `mavros/local_position/pose`(真实)
的 **Δxy = 0.00~0.01 m ⇒ 两者坐标系一致** ✓ ⇒ 用户说的「红轨迹不跟飞机」**不是坐标偏移** ✗，
更可能就是 §34 的「目标被夹到同一角点」机制 ✓（轨迹当然指向那个角落 ✗）。
静态 Δ≈0 也说明 `/odom_world = /aft_mapped_to_init` 与飞控视觉定位同源 ✓。




用户问「首飞前你觉得我该加哪些限制」时，**不要直接甩一张泛泛的参数清单** ✗ ——
`super_to_px4_bridge.cpp` 里其实已经实现了一整套保护，其中**最硬的物理兜底是关着的**。
先逐条查清现状，再按「层次」给建议（用户最认"哪一层是独立于算法的物理兜底"）。

### 已有且真正生效（源码行号 = 证据）

| 保护 | 配置 | 动作 | 代码 |
|---|---|---|---|
| 低电量 | `battery_threshold: 20.0` | **自动降落** | `:310-313 batteryCallback → triggerLanding()` |
| 遥控丢信号 | `rc_loss_timeout: 1.0` s | **自动降落** | `:325 → triggerLanding()` |
| SUPER 指令超时 | `timeout_threshold: 0.5` s | 位置保持 | 主循环 |
| MAVROS 断连 | — | 位置保持 | `:702` |
| 需手动解锁/切模式 | `auto_arm/auto_offboard: false` | 防误触发 ✓ | `:39-40` |
| 速度 | SUPER `traj_opt/max_vel: 0.3` m/s | 很慢 ✓ | `mid360_real_flight.yaml:61` |
| 降落 | `land_speed 0.3` · `auto_disarm: true` | — | — |

### ✗✗✗ 缺口 1：`geofence_en: false` —— 越界自动降落【已实现但关着】

```cpp
// super_to_px4_bridge.cpp:694
if (!checkGeofence()) { ROS_WARN_THROTTLE(2.0, "Geofence violation! Position hold + landing.");
                        triggerLanding(); ros::spinOnce(); rate.sleep(); continue; }
```
配置 `super_to_px4_bridge.yaml:51 geofence_en: false`，`:52-58` 是 x/y ±10 · z 0~5。
**这是不依赖规划器 / FUEL / SUPER 的物理兜底，首飞就该开** ✓

> ⚠️ **★★ z_min 陷阱（开了反而当场降落）**：`geofence/z_min: 0.0`，而飞机在地面时
> `/mavros/local_position` 的 z ≈ 0（实测 **−0.00**）⇒ **一开 geofence 立刻判越界 →
> `triggerLanding()` → 起飞即降落** ✗
> **⇒ 打开前必须把 `z_min` 降到地面以下**（如 `-0.5`）。**这是"加保护反而炸机"的典型**。

### ✗✗ 缺口 2：`goal_arbiter.py` 只夹 z，不夹 x/y

有 `clamp_z()`（`z_min 0.6 / z_max 5.0 / ceiling 6.0`），**水平范围完全不限制** ✗。
FUEL 的 `sdf_map/map_size` = 30×30（launch arg）⇒ 目标可能落到 **15 m 外**。
**⇒ "限制探索范围"的正确落点就在这里**（仲裁器里加 `x_min/x_max/y_min/y_max` 夹取），
**不是**去改 FUEL 的 `map_size`（那只是地图分辨率/体量，不是探索预算 ✗）。

### ✗ 缺口 3：FUEL 没有"探索时长 / 距离预算"

`algorithm.xml` 里只有 `perception_utils/max_dist: 4.5`（**感知**距离，不是探索预算 ✗）。
要"飞 N 秒就停"只能外部加：`nohup bash -c 'sleep 180; bash ~/emergency_stop.sh' &`

### ★ 推荐的分层（首飞最小集）

```
第0层 分级验证（最重要，不是参数）：先【不接 FUEL】只飞 SUPER 航点任务（按钮④），
      验证 定位/起飞/悬停/绕障/降落/急停 全通，再接 FUEL
第1层 物理硬兜底：geofence_en → true + 收范围 + ★z_min 下调到 -0.5
第2层 限探索范围：仲裁器加 x/y 夹取（±5 m）
第3层 高度：goal_z_min 0.6→1.0（防树枝/地形）· goal_z_max 5.0→3.0，仲裁器同步
第4层 速度：SUPER max_vel 0.3 保持；bridge max_velocity_xy 1.2→0.6（对齐，防意外）
      ★ 注意 max_velocity_xy(1.2) 只是上限，实际受 SUPER max_vel(0.3) 支配
第5层 时间预算：N 秒自动急停（FUEL 无内建，见缺口 3）
第6层 随时接管（都已就绪）：遥控器切 POSCTL（不依赖任何软件 = 最终保险）·
      按钮⑤任务急停 · CH10 控制探索启停
```

**★ 讲法很重要**：按「分层兜底」讲，并**明确标出哪一层独立于所有算法**（第 1 层 geofence +
第 6 层遥控器）—— 这是用户唯一完全信任的部分 ✓

### ✅ 已实施（2026-10-09，全部备份 `.bak_before_safety_20261009_173157`）

用户拍板：**速度 0.6 m/s · 探索区域 ±6 m · 高度 1~3 m · 不限时长**。四处改动：

| 参数 | 文件:行 | 改前 → 改后 | 作用 |
|---|---|---|---|
| `max_vel` | `mid360_real_flight.yaml:61` | 0.3 → **0.6** | 规划速度上限 |
| `max_acc` | 同上:62 | **1.0**（2026-10-09 追加，原 0.4） | 制动 0.18m ✓ 更短更安全；起停更跟手 |
| `goal_z_min/max` | 同上 `:21,22` | 0.6/5.0 → **1.0/3.0** | FSM 目标高度区间 |
| arbiter 启动参数 | `~/fuel_explore_all.sh:112` | **`_z_min:=1.0 _z_max:=3.0 _ceiling:=3.5 _xy_limit:=6.0`** | 与 FSM 两处同改 ✓ |
| `geofence_en` + 范围 | `super_to_px4_bridge.yaml:51-58` | false/±10/0~5 → **true / ±7.5 / −1.0~3.8** | ★物理兜底 |
| `goal_arbiter.py` | 新增 `clamp_xy()` + 参数 | `xy_limit=6.0`，origin 取首个 odom / CH10 时刻 | ★探索区域 ±6 m |

**★ 双保险的余量设计**：仲裁器夹 **±6 m**（规划层，只管 FUEL 的目标），
geofence 设 **±7.5 m**（物理层，比它宽 1.5 m）——
因为 geofence 用 `/mavros/local_position/pose`（PX4 局部系，原点≈起降点），
仲裁器用 `world`（SLAM 原点），**两者同源但有 cm~m 级偏移** → 必须留余量，
否则 geofence 会因坐标系偏移而误触发降落 ✗。
**★ 分层要点**：SUPER 的 RViz 点击目标（`/move_base_simple/goal` → `fsm.click_goal`）
**不经过仲裁器** ✗，只有 geofence 能兜住它 ✓ —— 这正是"物理层不可省"的证明。

### ★★★ 改 geofence 的越界动作（降落 → 悬停）+ 只编一个目标（2026-10-09 实测）

**需求**：用户要"真越界只保持悬停，不要自动降落"。
**位置**：`SUPER/super_planner/Apps/super_to_px4_bridge.cpp` 主循环（`ros::Rate rate(70.0)`，
`while (ros::ok())` @ :598）内的失效保护段，原代码：

```cpp
// :692-698（改前）
if (!checkGeofence()) {
    ROS_WARN_THROTTLE(2.0, "Geofence violation! Position hold + landing.");
    triggerLanding();                 // ← 会走 :614 降落分支 → land 服务 → auto_disarm
    ros::spinOnce(); rate.sleep(); continue;
}
```
**改法**：`triggerLanding()` → 复用"位置保持"逻辑（与 RC 丢失/MAVROS 断连同一套）：
```cpp
if (!checkGeofence()) {
    ROS_WARN_THROTTLE(2.0, "Geofence violation! Position hold (hover, NO landing).");
    if (super_cmd_active) { super_cmd_active = false; hold_position = current_pose;
                            current_yaw = tf::getYaw(current_pose.pose.orientation); }
    setpoint_position_pub.publish(hold_position);   // 70Hz 持续发定点 → 稳住 OFFBOARD
    ros::spinOnce(); rate.sleep(); continue;
}
```
**★ 关键点**
1. `continue` 会跳过本帧其余控制逻辑，所以**必须自己发 setpoint**（这里发 `/mavros/setpoint_position/local`），
   否则 PX4 因 OFFBOARD 失去 setpoint 流而触发 failsafe ✗。循环 70 Hz，够 ✓。
2. **只改 geofence 那处**；`triggerLanding()` 另有两处调用必须保留：电池低（`:310-313`）、
   land 服务（`:325`）—— 它们本来就该降落 ✓。改完用 `grep -n -B3 "triggerLanding()"` 核对只剩这两处 + 定义。
3. **★ 替换代码块时别忘了它上面那两行注释**（`// 地理围栏检查` / `// 越界后触发降落`）——
   只从 `if (!checkGeofence())` 开始替换会留下**陈旧注释**（改后 diff 里能看出来）✗。

**★ 只编一个可执行目标（26 秒，surgical，不动其它二进制）**
```bash
cd ~/catkin_ws/build && source /opt/ros/noetic/setup.bash
make super_to_px4_bridge -j4          # ★ 只编这个 target，不要 catkin_make 全量
```
**★★ 实测：改 super_to_px4_bridge.cpp 后重编，`fsm_node` 的 md5 一！字！不！变！**
（`a970854c2d12bd9d43c535200d78398b` 编译前后一致 ✓）—— 所以**不必担心动 bridge 会污染
fsm_node 基线** ✓。（但 `super_to_px4_bridge` 自身 md5 会变，且**删一行注释也会变** ✗
—— 说明它嵌入了 build-id/调试信息，md5 只能用来判断"是否变了"，不能判断"功能是否变了"。）



用户原话「根据传感器数据计算频率」→ 要给**算式**，不要给感觉：

```
实测频率: LiDAR MID360 10 Hz · /cloud_registered (FAST-LIVO2) 10.02 Hz · replan_rate 10 Hz
          rog_map 膨胀层 = inflation_step 2 × inflation_resolution 0.2 = 0.40 m
          robot_r 0.35 · 可靠感知距离（树林细杆保守取）5.0 m
t_react = t_perc(雷达积分0.1 + FAST-LIVO2处理0.05~0.10) + t_plan(1/10Hz=0.1) + t_act(0.10~0.15)
        ≈ 0.35~0.45 s  取 0.40 s

判据① 一个反应周期位移 ≤ 膨胀层(0.40m)  →  v × 0.40 ≤ 0.40  →  v ≤ 1.0 m/s  ★最严
判据② 两次雷达扫描间位移 ≤ 树干半径      →  v × 0.10 ≤ 0.15  →  v ≤ 1.5 m/s
判据③ 制动距离 v²/(2a) + 余量 ≤ 可用感知 →  可用=5.0−0.35−0.40−0.30=3.95m, a=0.4 → v ≤ 1.78
判据④ 加速时间 t=v/a：a=0.4 时 v=1.0 → 2.5 s（起停拖沓，故同时提 a 更好）
⇒ 交集 v ≤ 1.0 m/s。用户最终取 **0.6**（保守侧，且 a 保持 0.4 → 制动仅 0.45 m ✓ 安全）
```

**★ 只提速度不提加速度会吃掉安全余量** ✗：`d_brake = v²/(2a)` ——
v=1.0,a=0.4 → 1.25 m；v=1.5,a=0.4 → 2.81 m（危险）；v=1.0,a=2.0 → 0.25 m ✓。
**⇒ 提速度时必须同时核算 `max_acc`**，否则等于偷偷削减避障余量。

**★ 对照 SUPER 官方配置**（说服力最强，直接反驳"0.3 是不是太慢"）：

| 官方配置 | max_vel | max_acc | robot_r | replan |
|---|---|---|---|---|
| `click_smooth_ros1.yaml`（ROS1 实机·避障优先） | 1.2 | 2.0 | 0.40 | 15 Hz |
| `real_flight.yaml`（名字就叫实飞） | 2.0 | 4.0 | 0.25 | 10 Hz |
| `static_dense.yaml` | 8.0 | 15.0 | 0.2 | 15 Hz |
| **实机原值** | **0.3** | **0.4** | 0.35 | 10 Hz |

⇒ **实机原值比官方最低配置还低 4~5 倍**，用户直觉"太慢"是对的 ✓；
官方 `vel/acc ≈ 0.5~1.0`，实机 0.6/0.4 = 1.5（偏"快而钝"），若要更跟手应把 `max_acc` 提到 1.0~1.5。

## 33. ★★★ 用户自己打了补丁、让你「审查日志看更改是否有效」：先验时间线，再看机制

**不要拿到日志就读、直接下结论** ✗ —— 先做三件事，否则会把**改动前**的日志
当成**改动后**的证据（本次差点误判）。

### 第一步：日志在改动【之前】还是【之后】？
```bash
ls -lht <log 目录>/*.log                            # 日志 mtime
find <ws>/src -newermt "-24 hours" -type f          # 用户改了哪些文件
find <ws>/src -name "*.bak*" -newermt "-24 hours"   # ★ 连备份名一起看（备份名常直接点题）
```
**本次**：日志 15:16，改动 16:42~16:46 ⇒ **日志是改动前的，验证不了补丁** ✗。
⇒ 诚实说「这份日志验证不了你的改动」，同时**把改动前的日志当"现象日志"用** ——
它照样能证明根因 ✓。

### 第二步：补丁真的编译进去了吗？
```bash
ls -l <ws>/devel/lib/<pkg>/<node>       # 二进制 mtime
ls -l <ws>/src/<pkg>/src/<file>.cpp     # 源码 mtime（源码更新 ⇒ 改了没重编 ⇒ 补丁没生效 ✗）
grep -ac "<补丁自己的打印标记>" <日志>   # ==0 ⇒ 补丁一次都没跑过
```
**给补丁留一个可 grep 的标记**（本次 `[PATCH-arrival]`）—— 一 grep 就知道跑没跑过 ✓。

### 第三步：diff 改动 vs 它的 .bak，读用户意图
`diff <file>.bak_before_xxx <file>` —— 备份名常把意图写死：
`bak_before_arrival` / `bak_before_box6` / `bak_before_arrive_params` ✓

### ★★★ 「飞机在原地来回摇摆徘徊」的头号机制：夹取层把区域外的目标全压到同一个角点

**现象**：飞机在一个地方反复摇摆；用户以为是「FUEL 的轨迹不以真实飞机位置为准」。

**日志证据（本次改动前那次飞行）**：
```
arbiter 放行 183 次，绝大多数是【同一个角点】(-5.99, -5.95) ✗
FUEL 的候选目标 y 全在 −9 ~ −13 ✗（当时 FUEL box 还是 ±60，arbiter 夹取已是 ±6）
⇒ FUEL 在 ±60 里正常选点 → arbiter 把它们全夹到 (x_min, y_min) 那个角
  → 反复放行同一个点 → 飞机被反复指向同一处 → 摇摆徘徊 ✓✓✓
```
**⇒ 分层限位的通用铁律：上游生成器的范围必须【明显小于】下游夹取的边界** ✗
- `上游范围 == 夹取边界` ⇒ 目标一超界就被压到**边界线上**；x、y 同时超界 ⇒ 永远压到**同一个角** ✗✗
- 正确配比：**上游 box(±5) < 仲裁器夹取(±6) < geofence(±7.5)** —— 每层都留余量 ✓
  （与 §32「geofence 要比仲裁器宽」是同一条道理）

**★ 一行命令定性这类问题**：
```bash
grep -a "放行目标" arbiter.log \
  | sed -E 's/.*\(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\).*/\1 \2 \3/' \
  | sort | uniq -c | sort -rn | head
```
**前几名是同一个点 ⇒ 夹取塌缩**，不是 FUEL 的算法问题 ✓

### FUEL 的「红色轨迹」数据链（用户问「为什么红轨迹不以真实飞机位置为准」时照这个答）
```
红轨迹   = /planning/travel_traj （RViz 里 Marker 名 executed_traj；
           ★ RViz 项字段是 Marker Topic 不是 Topic，见 §29）
轨迹起点 = fd_->start_pt_ = fd_->odom_pos_          (fast_exploration_fsm.cpp:92)
odom 源  = 订阅 /odom_world ← fuel_input_bridge ← /aft_mapped_to_init（FAST-LIVO2 位姿，原样转发）
           odometryCallback 填 fd_->odom_pos_        (fsm.cpp:379-385)
⇒ FUEL 的「飞机位置」= FAST-LIVO2 位姿，与真实定位【同源】✓
⇒ 所以「轨迹不以真实飞机位置为准」只可能是【坐标系/帧】问题 ✗：
   FUEL 发目标时 header.frame_id 写死 "world"，数值却来自 camera_init 系 ✗
   ⇒ 要实测 world ↔ camera_init 是否重合（只要有平移就整体错位）
★ 判据别靠眼看 RViz —— 同时抓 /odom_world 与 /mavros/local_position/pose 比数值 ✓
```

### 「到达后才换目标」类补丁的参数经验（本次用户加了 replan_on_arrival）
```
arrive_r 太小（0.3）✗ → 定点精度 ~0.2~0.4m + 风 ⇒ 永远进不去 ⇒ 全靠 timeout 兜底 ⇒ 仍频繁换目标 ✗
   ⇒ 建议 0.8~1.0，且【必须 ≥ 下游仲裁器的 arrive_radius(0.5)】✗
     否则上游比下游更早认为「到了」，空转
arrive_timeout 太短（5.0s）✗ → v=0.6m/s 时 5 秒只飞 3m，而 6m 行程要 10s
   ⇒ 建议 15~25s
★ 源码默认值 vs yaml 值：yaml 覆盖默认 ✓（本次源码 0.6/40.0、yaml 0.3/5.0 ⇒ 生效的是 0.3/5.0）
★ 保持上游目标稳定的补丁方向对，但**参数必须与下游阈值对齐**，否则只是把
  「每帧换目标」变成「每 timeout 换目标」，现象缓解但没根除 ✓
```

---

## 42. ★★★★ 最高价值的一课：参数值去【运行时日志】读，别猜 launch 文件

### 我犯的错（会让结论与事实相反 ✗）
```
要改"探索框高度 box_min_z/box_max_z"：
✗ 我 grep exploration_manager/launch/exploration.launch → box z = 0.0 ~ 2.0
  → 于是告诉用户"你说的 -2.5/6.0 不符" ✗✗ （用户当场说的是对的 ✓）
✓ 真相: 启动脚本 fuel_explore_all.sh:106 实际调的是
    fuel_bridge/launch/fuel_real_airborne.launch  → box z = −2.5 / 6.0 ✓
    证据在【运行日志】里，roslaunch 会把生效参数全打印:
      * /exploration_node/sdf_map/box_min_z: -2.5
      * /exploration_node/sdf_map/box_max_z:  6.0
      * /exploration_node/sdf_map/map_size_x: 120.0
```
### ★ 正确顺序（三步，别跳）
```bash
# ① 先读运行时生效值（最权威 ✓ roslaunch 启动时逐个打印）
grep -aE "^ \* /(exploration_node|super_to_px4_bridge|fsm_node)/" log/real_flight.log
grep -a "Load param" log/real_flight.log
# ② 再顺着【启动脚本真正调用的那个 launch】找定义处
grep -nE "roslaunch" ~/fuel_explore_all.sh        # → 106: fuel_real_airborne.launch
# ③ 其它同名文件可能只是【没被 include 的副本】✗ —— 同参数不同值 ✗
```
**⇒ 同一个参数在多个 launch 里有不同值是常态 ✗；猜错就得出【相反结论】✗✓✓**
**★ 用户对自己机器的参数往往是准的 ✓ —— 当我"核对出不符"时，先怀疑【自己查错了文件】✓**

### box 的语义 = 【前沿生成边界】（所以改它＝直接控制前沿出现在哪 ✓）
```
sdf_map.cpp: box_mind_/box_maxd_ ← param sdf_map/box_min_*/box_max_* → posToIndex → box_min_/box_max_
frontier_finder.cpp:152   !edt_env_->sdf_map_->isInBox(nbr)   ← 前沿扩展时检查 ✓
⇒ 原 −2.5 ~ 6.0 ⇒ 头顶/地下都在生前沿 ✗（追看不到的区域 ✗）
⇒ 收到 0.3 ~ 3.5 ⇒ 只在【扫得到的高度层】生前沿 ✓
```

### `cluster_size_z` 10 → 1.0：解释「视点为什么总在 3.9m」（★ 用户先看出来的坑 ✓）
```
10m 的高度容差 ⇒ 高处未知区与低处未知区【聚成一簇】✗ ⇒ 簇"平均高度"被高处拉高 ✗
实测 24 个候选目标 z: 1.61 1.80 2.11 2.91 | 3.01 … 4.25（中位 3.92，>3.0 的 20 个）✓
⇒ 改成 1.0 ⇒ 前沿按高度分层 ⇒ 每层算出自己的高度 ✓
★ 我原先只归因"雷达上视角大" ✗ —— 实际是【雷达不对称 + cluster_size_z=10】叠加 ✓
```

### 最终 8 处高度参数（一次列全，少一处就出矛盾 ✗）
| 项 | 值 | 位置 |
|---|---|---|
| box z | **−2.5 ~ 6.0** ← ★★【必须宽松 ✗ 收到 0.3~3.5 会造成悬停，见 §43】 | `fuel_bridge/launch/fuel_real_airborne.launch` |
| `frontier/cluster_size_z` | 1.0 | `algorithm.xml` |
| `candidate_zmin/zmax/zstep` | 0.8 / 3.0 / 0.5 | `algorithm.xml` |
| `goal_z_min/max` | 0.8 / 3.0 | `mid360_real_flight.yaml`（FSM 也夹 ✗）|
| arbiter `_z_min/_z_max/_ceiling` | 0.8 / 3.0 / 3.5 | `fuel_explore_all.sh` |
| geofence `z_min/z_max` | −1.0 / 3.8 | `super_to_px4_bridge.yaml` |
| SUPER `virtual_ceil_height` | 6.0 | `mid360_real_flight.yaml` |
| FUEL `map_size_z` | 6.0（z∈[−1,5]）| `fuel_real_airborne.launch` |

### 判读技巧：**夹取痕迹 = 大量目标恰好等于某个限位值**
```bash
grep -a "放行目标" arbiter.log \
 | sed -E 's/.*放行目标 \(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\).*/\1 \2 \3/' \
 | sort | uniq -c | sort -rn | head
# z 扎堆在整数(如 3.00) ⇒ 上游原始值普遍越界、被限位夹取 ✓
# x/y 扎堆在同一点 ⇒ 夹取塌缩到角点 ✓（§34）
```

**★ 与用户协作的要点**：用户会给出一份**结构完整、数据自洽**的改动方案（含统计/因果/落地表），
且**他的统计通常经得起复算** ✓。汇报时应：① 先全量复算核对他的数字 ✓ ② 明确指出哪几项
**已经做过** ✗（避免重复劳动）③ 把**数值冲突**摆出来请他定 ✓（例：他先说"限高3.0"、后提
"3.5"，必须问清 ✓）④ 自己额外发现的问题（如"同分优先更低"不做则自适应失效）要主动讲 ✓。

### ⚠️ ★★ 但用户方案里的参数语义必须先自己验证再动手（2026-10-11 我照做造成真实故障 ✗✗✗）

用户给方案时带了一句「探索框高度 box_min_z/box_max_z：−2.5~6.0 → 0.3~3.5，不再追头顶和地下看不到的区域」。
**我直接照做了** ✗ —— 结果飞机**全程悬停**。原因：**box 的语义是「感兴趣区域」边界，
box 外面一律算未知** ✗，所以 box 是**"让边界远离飞机"的宽松装置**，不是限位 ✗：

```
box z 收到 0.3~3.5 ⇒ 边界正好压在飞行高度层 ⇒ 头顶 3.5m 以上/脚下 0.3m 以下【永远未知】✗
⇒ 前沿(已知/未知交界)全部长在 box 面上（就在飞机附近）✗ = 用户看到的【"飞机附近一圈红色"】✓
⇒ 视点在 box 内看不到它们 ⇒ 128 个 cluster 全判 UNOBSERVABLE ✗
⇒ `to visit: 0`（全程）✗ ⇒ FUEL 无目标 ⇒ 【悬停不动】✓
实测佐证: 飞行段 69s 只走 7.3m ✗，停在 (−5.12,−0.86,1.03)（x≈box 边界 ✗）；仲裁器只放行 1 次(冷启动)
修法: box 还原 −2.5~6.0（一行配置，不用重编译 ✓）
```
**★★ 结论：想"别追头顶/地下"要用 `candidate_zmin/zmax` + `goal_z/arbiter 限位`，绝不能用 box** ✗✓✓

### ★★★ 高度体系三层语义（再强调一遍，这是最容易混的地方 ✗）
| 层 | 参数 | 语义 | 收敛方向 |
|---|---|---|---|
| ① 感兴趣区域 / **前沿生成边界** | `box_min/max_x/y/z` | box 外一律算未知 ⇒ **前沿长在 box 面上** | ★ 必须**宽松**（远大于可达范围）|
| ② 视点高度候选 | `frontier/candidate_zmin/zmax/zstep` | FUEL 愿意飞的高度档位 | 收紧 ✓ 安全 |
| ③ 安全限位 | `goal_z_min/max` + arbiter `_z_min/_z_max/_ceiling` | 硬夹取 | 收紧 ✓ 安全 |
⇒ ②③ 是"收紧"工具，① 是"放宽"工具 ✗ 用反了就是本次这个故障 ✓

### ★★★ 两个高价值诊断手法（值得复制到任何机器人工程 ✓）
```
① 给【失败路径】加带坐标+数量的日志 ✓✓✓
   在"前沿簇判为不可观测"处加:
     "[frontier] cluster UNOBSERVABLE at (x,y,z) cells=N | 累计=K"
   ⇒ 一次飞行直接抓出上面的 box 坑（128 条带坐标告警 ⇒ 一眼看出"前沿全贴在 box 面"✓）
   ⇒ 没有它，只能看到"不明原因悬停" ✗ 要绕好几轮 ✓
   ★ 通用: 凡是"东西被丢弃/判失败"的分支，都该打【位置 + 数量 + 累计】
② 完成判定处打印【分层覆盖率】✓（★ 用户自加，非常好 ✓）
   "[PATCH-cov] FINISH known: 97.1% | z<1m 88.8% | 1-2m 99.6% | >2m 99.5% | occupied=230 | frontiers=0"
   ⇒ 一眼判断"FINISH 是否合理" ✓
   ⇒ 分层 known% 还能【反证雷达 FOV】: z<1m 覆盖偏低 ✓ 恰与"下视角仅 7°"一致 ✓✓✓
```

### ★★ 「覆盖率」有两种口径，别看错尺子 ✗（纠正 §38 的一条错误表述）
```
· 路径覆盖: 飞机轨迹走过 ±6m 区域的多少格   ← 实测仅 20~28% ✗
· 地图覆盖: 已知格子占地图的比例           ← 用户 PATCH-cov 实测 known 97.1% ✓
⇒ 两者可差极大（雷达水平 360° + 上 52° ⇒ 高处一转就把一片"看满"了 ✓）
★ §38 曾用"路径 20~28%"推论"FUEL 误判完成"✗ —— 实际地图已 97.1% 已知 ⇒
  【FINISH 判定是合理的 ✓】真正原因是【场地几乎空（occupied 仅 230 格 ✗）+ 雷达视野大】✓
★ 判据: 想知道"是真探完还是漏探"，必须看【地图覆盖/occupied 数】，
  而不是"飞机走过多少路" ✗（后者与探索充分性无关）
```

### ★ 本轮其他踩坑
```
· 一次只改一个变量 ✗: box 收紧 + cluster_size_z 10→1 两个改动叠加才暴露问题 ⇒ 定位困难 ✓
  ⇒ 改完一个先飞一次（或用二分法回滚 ✓）；cluster_size_z=1.0 本身是对的 ✓ 不是它的问题 ✓
· ★ 写进 C++ 的中文 ROS_WARN 在日志里打印成 ?????? ✗（编码 ✗）
  ⇒ 【诊断日志一律用纯 ASCII/英文】✓ 否则判读时抓瞎 ✗
· 被强杀的进程会在日志/CSV 尾部留 NUL 空洞 ⇒ 读之前清洗 `.replace(b'\x00',b'')` ✓
· ★ 高度自适应【成功】的判据 = pos_monitor.csv 飞行段 Z 有变化（本例 1.00~1.57m ✓）；
  若 Z 恒定 ⇒ 说明又被压到某个限位了 ✗
· ★ box 的 x/y 也受同一规则约束: 若 box x/y == arbiter 夹取范围 ⇒ 前沿正好落在夹取边界上 ✗
  ⇒ box 应比 arbiter(±6)/geofence(±7.5) 都宽（±8~10 ✓）
· 新脚本 `scripts/mid360_fov_measure.py`: 实测雷达真实视角/盲区/安装角
  （live 模式 LiDAR 一开就能跑，不需要整套栈；bag 模式注意确认 bag 是这个雷达录的 ✗）
```


