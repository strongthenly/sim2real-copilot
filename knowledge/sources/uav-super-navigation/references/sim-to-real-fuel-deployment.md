# 把仿真里跑通的 FUEL 自主探索搬回实机 fouruav（自用部署）

> 场景：探索栈（FUEL + fuel_bridge）在 WSL `ros1-twin` 仿真里跑通了，要导出一版**实机可用**的。
> 自用（不是对外交付 ⇒ 不需要防源码泄露，直接给全源码。对外交付走 `uav-delivery-packaging`）。

---

## 一、🔑🔑 最重要的一条：实机上 FUEL 是【决策层】，永不直接控制飞机

仿真里 `fuel_all.sh` 用 `enable_ctrl:=true`，让 FUEL 的控制桥**负责起飞**
（seed 悬停 → 自动 OFFBOARD → 起飞到 `seed_z`）。
**实机上不能这样** —— 实机的 OFFBOARD 切换、起飞、执行，全都由 SUPER 负责。

```
人工切 OFFBOARD
   ↓
CH10 / UserCmd1  → SUPER 起、飞机起飞悬停          ← 执行层（原有的，一个字都不动）
   ↓
UserCmd4         → FUEL 决策层起（enable_ctrl:=false）
                   → /exploration_goal → goal_arbiter → /waypoint_goal → fsm_node
   ↓
UserCmd5         → 先切 POSCTL → 杀掉 FUEL         ← SUPER 与飞机不动，稳稳悬停
```

**`enable_ctrl` 在实机【永远 false】** ⇒ `fuel_ctrl_bridge` 根本不启动 ⇒ **FUEL 直接碰不到 PX4** ✓
FUEL 唯一输出 = 一个话题 `/exploration_goal` ⇒ 出问题杀 FUEL 进程即可，**飞机始终在 SUPER 手里** ✓

**收益**：风险面从「FUEL 会发控制指令」降到「FUEL 只发一个目标话题」。
**这是本次最重要的架构决策** —— 也是"FUEL 是叠加层、不替代 SUPER"这个定位的直接后果。

---

## 二、实机的启动/停止入口约定

- **`ch10` = MAVROS RC 通道 10**。出处：`task_run_scripts/config/waypoint.yaml:3` 的注释
  `# 0 for RVIZ and; 1 for Mavros RC ch10; 2 for constant time` —— 它是 waypoint 任务的**触发源**
- **SUPER 的按钮入口 = `task_run_scripts/scripts/NX_code/User_define_commands.yaml` 的 `UserCmd1`**
  ```
  UserCmd1: name "SUPER航点任务"
    command: source 两套 setup && mkdir -p log
             && (nohup rosrun task_run_scripts global_path_publisher.py _file:=guidepoints_new.txt ... &)
             && sleep 1
             && (nohup roslaunch task_run_scripts real_flight.launch
                   config_path:=.../config/mid360_real_flight.yaml
                   rviz_config:=.../rviz/mid360_top_down.rviz
                   use_waypoint:=true waypoint_data_path:=.../endpoint.txt > log/real_flight.log 2>&1 &)
  ```
- **加 FUEL 按钮就照 UserCmd1/2 的写法**：`source` 两套 setup + `mkdir -p log`
  + `(nohup bash /home/fouruav/fuel_explore_real.sh > log/fuel_launch.log 2>&1 &)`
  ⇒ **必须后台化 `( ... & )`**，否则按钮会被脚本里的 `sleep` 阻塞 ✓

---

## 三、🔑🔑 停止命令的精髓（照搬 `UserCmd2`，逐字继承）

```bash
rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}'   # ★ 第 0 步：先让飞机进入【定点悬停】
sleep 1
pkill -SIGINT -f 'real_flight[.]launch'                       # ★ 第 1 步：再断 setpoint 流
sleep 2
pkill -f 'fsm_[n]ode'                                         # ★ [x] 方括号 = 防自匹配
pkill -f 'super_to_px4_bridg[e]'
pkill -f 'waypoint_missio[n]'
pkill -f 'map_to_worl[d]'
pkill -f 'top_down[.]rviz'
```

**为什么必须先切 POSCTL**：OFFBOARD 下直接杀掉 setpoint 发布者 ⇒ 飞控失去指令流 ⇒
**可能触发 failsafe** ✗。先切 POSCTL 让飞机自己稳住，再断流，是安全的。

**`pkill -f 'name[e]'` 的方括号技巧**：`pkill -f` 会匹配到**它自己所在的命令行**（`bash -c` 全文），
加一层字符类 `[e]` 就能自排除 ✓（同理 `real_flight[.]launch` 里的 `[.]`）

---

## 四、⚠️⚠️ 仿真停止脚本**不能**直接搬到实机

`fuel_stop.sh`（仿真版）会 `pkill -f roslaunch mavros` / 杀 PX4 / 杀 FAST-LIVO2 ——
仿真里无害，**实机上 MAVROS 一断 = OFFBOARD setpoint 流中断** ✗。

⇒ 实机版 `fuel_stop_real.sh` 的规矩：
- **默认只停 FUEL** ✓
- MAVROS / PX4 / FAST-LIVO2 **必须显式加参数**（`--with-mavros` / `--with-px4` / `--with-livo`）才动 ✓
- 停 MAVROS 前要求输入 `yes` 二次确认 ✓
- 结尾打印「仍存活的安全链路」状态行 ✓

其它差异：
- **PCD 保存路径不同** —— 仿真 `~/twin_ws/src/FAST-LIVO2/Log/pcd` vs 实机 `~/catkin_ws/src/FAST-LIVO2/Log/pcd` ✓
- **FAST-LIVO2 一律 `SIGINT`（触发 savePCD），绝不 `kill -9`** ✓

---

## 五、什么能原样搬、什么必须剥掉

| 组件 | 处置 | 依据 |
|---|---|---|
| `fuel_input_bridge.py` | **原样** ✓ | 订阅 `/aft_mapped_to_init` + `/cloud_registered` —— **实机跑 FAST-LIVO2 时话题名完全一致** ⇒ 这层桥本来就硬件无关 |
| `fuel_ctrl_bridge.py` | **不带到实机** ✓ | 见第一节（`enable_ctrl` 永远 false）|
| `goal_arbiter.py` | **原样** ✓ | 纯话题映射（`/exploration_goal` → `/waypoint_goal`）|
| `FUEL/fuel_planner` | **原样** ✓ | 纯算法，10 个包 |
| `FUEL/uav_simulator` | **✗ 必须排掉** | `find_package(CUDA REQUIRED)` + Gazebo + rotors —— 实机用不上；**排掉它 = 不用碰 CUDA/gazebo/rotors** |
| 系统依赖 | 只补 **`libarmadillo-dev`** | FUEL 里 `find_package(Armadillo REQUIRED)`；Eigen3/PCL/OpenCV/Boost/Qt5 装 FAST-LIVO2+rviz 时已有 |
| **LKH** | **不是外部依赖** ✓ | `utils/lkh_tsp_solver/src/LKHmain.c` 在包内，**编译自带**；`PATH` 里没有 `LKH` 是正常的 |
| `twin_bridge` / `twin_livo_fast.sh` / AirSim `41451` 检查 / `/clock` 检查 / PX4 SITL / `14610` tmux / MAVROS 的 `udp://:14611@127.0.0.1:14610` | **全删** | 全是仿真专有 |

**包体积极小**：只带 `fuel_planner` 时 **503 KB**（`fuel_planner` 252 文件 + `fuel_bridge` 18 文件）✓
排除清单：`.bak*` / `bag/` / `log.md` / `test.txt` / `__pycache__` / `.git` ✓

**⚠️ 打包脚本要记录关键文件的 MD5**（`fuel_input_bridge.py` / `fuel_ctrl_bridge.py` /
`goal_arbiter.py` / launch / 脚本），并写明「与仿真版逐字节相同」——上机后逐项复核 ✓

---

## 六、实机配置适配（★ 只动 FUEL 自己的，飞行安全参数另议）

| 位置 | 仿真值 | 实机 |
|---|---|---|
| `map_size` | 240/240/11 | **120/120/6**（Orin NX 内存；只减体素数，不涉安全）|
| `box_min/max_x,y` | ±115 | 按实机作业区 |
| `fuel_cmd_topic` | `planning/pos_cmd` | **`/fuel/pos_cmd_sink`** —— 联调时**绝不能**让 FUEL 和 SUPER 的 `fsm_node` 抢 `/planning/pos_cmd` |
| `enable_ctrl` | 默认 `false` ✓ | **永远 `false`** |
| `goal_z_min/max`、`virtual_ceil_height`、`box_*_z`、`max_vel/max_acc`、`seed_z` | — | ⚠️ **飞行安全参数，不擅自改**；在 launch 里保留原值 + 注释标明「仅在 `enable_ctrl:=true` 时生效，开启前需单独评审」|

**⚠️ 实机 `mid360_real_flight.yaml` 的现状会限制探索**：
```
goal_z_min/max = 0.6 / 1.3   ⇒ FUEL 发的目标被【吸附到 1.3 m】⇒ 三维探索退化成贴地一层
map_size       = [30,30,6]   ⇒ 目标一飞出 30 m 就 A* NO_PATH
virtual_ceil_height = 3
```
**这几个参数其实是同一件事**（飞机能飞多高 / 探索范围多大），而且
**仲裁器在 FSM 上游 ⇒ 改限高必须两处都改** ✓（`fsm.goal_z_*` 和仲裁器的 z 夹取）

---

## 七、上机流程（分支 + **只做加法**）

```bash
cd ~/catkin_ws && git status               # ⚠️ 子模块 src/SUPER、src/task_run_scripts 常带 M（本地修改）
git checkout -b feature/fuel-exploration   # ★ 子模块的本地修改【不受父仓库开分支影响】，安全
tar xzf ~/fuel_real_pkg.tar.gz
cp -a fuel_real_pkg/src/fuel_planner src/ && cp -a fuel_real_pkg/src/fuel_bridge src/
catkin_make                                 # 只编新包；编完复核 devel/lib 里 SUPER/FAST-LIVO2 产物 mtime 未变
cp fuel_real_pkg/scripts/*.sh ~/
# 回退: git checkout master && catkin_make    （或直接删掉那两个新目录）
```

- **先 `ls -d src/fuel_planner src/fuel_bridge` 确认无同名冲突** ✓
- 实机 `src/` 现有内容（供对照）：
  `cv_bridge / ddynamic_reconfigure / FAST_LIO / FAST-LIVO2 / livox_ros_driver2 /
   mavlink_udp_bridge / odin_ros_driver / realsense-ros / rpg_vikit-master / SUPER / task_run_scripts`
- 改 `User_define_commands.yaml` **前先备份 + 贴 diff** ✓（属用户核心配置文件）

---

## 八、验证阶梯（逐级放行）

| 级 | 做什么 | 风险 | 判据 |
|---|---|---|---|
| **L0 影子** | 只启 FUEL 决策（`enable_ctrl:=false`）| **零** | `/exploration_goal` 持续发布且在范围内；**`/planning/pos_cmd` 发布者仍只有 `fsm_node`**；CPU/内存余量 |
| L1 台架 | 通电不打桨 | 低 | 目标经仲裁器到达 `/waypoint_goal`、fsm 收到、SUPER 轨迹正常 |
| L2 系绳/开阔 | 限高不变、小范围 | 中 | 飞机按 FUEL 目标自主移动、无 `A* NO_PATH` |
| L3 树林 | 小范围、短时长 | 高 | 覆盖率上升、一键停可用 |

**L0 是"零风险"的关键设计**：`enable_ctrl` 默认 `false` ⇒ 控制桥**根本不启动** ✓
不需要额外开关，也不需要"记得别开控制"这种人为约束 ✓

**L0 的前置闸（脚本里必须有，任一不过就拒绝启动、绝不半启动）**：
```
① /mavros/state: connected=true · armed=true · mode=OFFBOARD   ← 你已切好
② 高度 > 0.3 m（确认真的在空中悬停，不是在地面）
③ FAST-LIVO2: /aft_mapped_to_init 与 /cloud_registered 都有数据
④ SUPER: fsm_node 进程存活（探索要靠它执行）
⑤ /planning/pos_cmd 的 Publishers 只有 fsm_node
```

---

## 九、缺 SSH 凭据时的正确做法

**不要翻用户凭据文件** ✗。推荐**配公钥免密**（用户跑一次，agent 全程看不到密码）：
```powershell
ssh-keygen -t ed25519 -f "$env:USERPROFILE\.ssh\id_ed25519"        # passphrase 两次回车
type "$env:USERPROFILE\.ssh\id_ed25519.pub" | ssh fouruav@<NET_WLAN_C>.131 "mkdir -p ~/.ssh; chmod 700 ~/.ssh; cat >> ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys; echo KEY_INSTALLED"
ssh -o BatchMode=yes fouruav@<NET_WLAN_C>.131 "hostname; whoami; echo OK"
```

**⚠️ `~/.ssh/config` 里可能有多个同名 `Host fouruav`**（本次有 4 个、`User` 字段各不相同 ⇒
`ssh fouruav` 会歧义）⇒ **一律用 `fouruav@<NET_WLAN_C>.131` 显式写死** ✓

**⚠️ 权限边界**：即使有 SSH 通路，**改 `User_define_commands.yaml`、启动节点、动 `mid360_real_flight.yaml`
之前都要先备份 + 贴 diff 给用户过目** ✓；编译（只加包）不影响飞行，可以自行做 ✓

---

## 十、⚠️⚠️ 上机前的强制预检：**飞行配置有没有被悄悄改过**

**本例实测踩到（未提交的本地改动，就躺在实机上）**：
```diff
~/catkin_ws/src/task_run_scripts/config/MID360_Fly.yaml   ← 树林飞行用的【低算力】配置
  publish:
-   dense_map_en: false     ← 低算力（树林安全：关稠密建图 + 抽稀）
+   dense_map_en: true      ← ★★ 被改开了 ⇒ 算力约翻倍 ⇒ 树林中位姿延迟 → 定点漂移 → 迫降
```
这正是本技能反复强调的失效路径：**把"飞行配置"改成"建图配置"的参数** ✗。
**⇒ 每次上机部署/起飞前，一条命令必查**：
```bash
cd ~/catkin_ws/src/task_run_scripts
git status -s                       # 哪些配置有未提交改动（本例: MID360_Fly.yaml / odin_real_flight.yaml / NX_code）
git diff -- config/MID360_Fly.yaml  # ★★ 飞行安全配置的每一处改动都必须看清
git diff --stat                     # 其余改动概览
git branch --show-current           # 子模块各自停在哪个分支
```
**发现后的处理：报告 + 问，不要擅自改回** ✗（可能是用户正在做建图测试）。

**★ 由此推出的通用规矩**：`git status -s` 里任何 ` M config/*.yaml` 都要按"可能影响飞行安全"对待，
逐条 `git diff` 看清后再决定；**"未提交"≠"不重要"**。

## 十一、recon 命令（一条看全，别逐个猜）

```bash
df -h / | tail -1
dpkg -l libarmadillo-dev libpcl-dev libeigen3-dev qtbase5-dev 2>/dev/null | grep ^ii   # ★ 一条看全依赖
nproc; free -h | head -2
ls -l --time-style=long-iso ~/catkin_ws/devel/lib/super_planner/fsm_node               # ★ 回退基线
grep -nE "goal_z_min|goal_z_max|map_size|virtual_ceil" ~/catkin_ws/src/task_run_scripts/config/mid360_real_flight.yaml
find ~/catkin_ws/src -iname "*arbiter*"                                                # FUEL 侧新增件（应为空）
```
**本例实测结论**：依赖**一个都不缺**（`libarmadillo-dev 1:9.800.4` 已在 ⇒ 上文"只补 libarmadillo-dev"
这一步可以省掉）；磁盘 233G 可用 157G；8 核 / 15Gi（可用 12Gi）。

**★ 回退基线要连具体数字一起记**：本例 `fsm_node` = **215,655,832 字节 · 2026-08-24 14:26**
（母仓库 remote = `git@github.com:<ORG>/<PRIVATE_WS_REPO>.git`）。编完新包后**逐位比对 size + mtime**。

## 十二、分支策略的推广：**不必纠结当前在哪个分支**

本例母仓库 `~/catkin_ws` 在 `master`，但两个子模块各停在**不同的 feature 分支**
（`SUPER` → `feat/guide-path-c`；`task_run_scripts` → `feat/long-distance-replay`）
—— 也就是说"**能飞的系统 = master + 两个 feature 分支**"的组合。

**⇒ 正确心理模型**：不要去纠结"该基于哪个分支"，因为
**只要只往 `src/` 加纯新目录、完全不碰任何子模块，母仓库开在哪个分支都等价** ✓
- 回退 = `git checkout <原分支>` + 删掉那两个新目录
- **子模块指针不变 ⇒ 现有可飞系统一字不动**（回退零风险）
- 落位用 `cp -a`（直接放 `~/catkin_ws/src/` 下，**不要嵌套工作区**）

**★ `goal_z` 那类"注释写着临时改"的参数，先查 git 历史再问用户**：
本例 `git log -p -- config/mid360_real_flight.yaml | grep -E "^[-+].*(goal_z|map_size)"`
⇒ **没有 goal_z 的变更记录** ⇒ 说明它从第一次提交起就是这个值，注释里的"临时改"一直没改回来
⇒ **没有"正式原值"可恢复** ⇒ 只能让用户定，别自己臆测一个"原本的值"。
