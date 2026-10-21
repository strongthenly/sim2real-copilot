# 把仿真里做出来的探索层（FUEL）导出到实机 fouruav

> 场景：WSL(`ros1-twin` + AirSim) 里跑通了「FUEL 决策 + SUPER 执行」探索闭环，
> 现在要**导出一版给实机 fouruav（Orin NX）用**。
> ★ 注意与 `uav-delivery-packaging` 的区别：那个是**对外交付防源码泄露**（二进制/脱敏）；
> 这里是**自用部署**（全源码 + 一键脚本），走的是完全不同的流程。

---

## 一、先判"哪些硬件无关、哪些是仿真专用"（决定包的内容）

### ✅ 硬件无关、可直接搬的
| 组件 | 为什么不用改 |
|---|---|
| `fuel_input_bridge.py` | 订阅 `/aft_mapped_to_init` + `/cloud_registered`，**实机跑 FAST-LIVO2 时话题名完全一致** ✓；输出 `/odom_world` + `/map_ros/pose|cloud` 是 FUEL 内部约定 ✓ |
| `fuel_ctrl_bridge.py` | 输出 `/mavros/setpoint_raw/local`，实机 MAVROS 相同 ✓ |
| `goal_arbiter.py` | 只看话题，与硬件无关 ✓ |
| `FUEL/fuel_planner` | 纯算法 ✓ |

### ❌ 仿真专用、**不能**带上去的
```
twin_bridge            从 AirSim 合成 LiDAR/IMU
twin_livo_fast.sh      起 twin 链路
PX4 SITL               tmux 里的 pxh + mavlink start -u 14610
AirSim 检查            TCP 41451 探测
/clock 检查            use_sim_time 步进（实机是真实时钟）
sim_real_flight.yaml   仿真配置
uav_simulator          见下（会拖进 CUDA + Gazebo + rotors）
```

### ⭐ 关键：`FUEL/` 下只有两个目录，只带走一个
```
FUEL/fuel_planner      ← 10 个包，2.9 MB，实机需要 ✓
FUEL/uav_simulator     ← 需要 find_package(CUDA) + Gazebo + rotors ✗ 实机不需要
```
**排掉 `uav_simulator` = 一次性甩掉 CUDA / Gazebo / rotors 三个依赖** ✓
`fuel_planner` 内的包：`active_perception` `bspline` `bspline_opt` `exploration_manager`
`path_searching` `plan_env` `plan_manage` `poly_traj` `traj_utils`
`utils`（内含 `lkh_tsp_solver` / `cmake_utils` / `pose_utils` / `uav_utils`）

**系统依赖只需补一项**：`sudo apt install -y libarmadillo-dev`
（Eigen3 / PCL / OpenCV / Boost / Qt5 通常随 FAST-LIVO2 + rqt 已装）
**`LKH` 求解器不需要外部可执行** ✓ —— `utils/lkh_tsp_solver/src/LKHmain.c` 在包内，编译自带
（`which LKH` 返回空是正常的，别据此判定缺依赖）

---

## 二、实机上已有、**不要重复带**
`~/catkin_ws/src` 通常已含：`SUPER` · `task_run_scripts` · `FAST-LIVO2` · `livox_ros_driver2`
· `odin_ros_driver` · `realsense-ros` · `rpg_vikit-master` · `mavlink_udp_bridge` 等（子模块）
⇒ **增量只有 `fuel_planner` + `fuel_bridge`** ✓

---

## 三、工作方式：在飞机上开分支，只做加法
```bash
cd ~/catkin_ws
git status                    # ★ 先看有没有 M（子模块本地修改是常态，别动它们）
git checkout -b feature/fuel-exploration
# ... 落位 src/fuel_planner + src/fuel_bridge、改 launch、放脚本 ...
# 回退: git checkout <原分支> && catkin_make     ← 原有可飞版本原样恢复
```
**要点**：
- **只新增目录，不改动 SUPER / FAST-LIVO2 / task_run_scripts 里的任何文件** ⇒ 回退零风险
- **子模块的 `M` 与父仓库开分支无关**（`M src/SUPER` 是子仓库的本地修改，切父分支不动它）；
  用户常有意保持 `M` 不提交 —— **不要替用户 commit/push**
- 子模块的 `.git` 是**文件**不是目录，用 `git -C <dir> rev-parse --git-dir` 判断

---

## 四、配置适配（★ 只动 FUEL 自己的，绝不碰飞行安全参数）

| 位置 | 仿真值 | 实机 | 性质 |
|---|---|---|---|
| `map_size_x/y/z` | `240/240/11` | `120/120/6` | 资源（Orin NX 内存）|
| `box_min/max_x/y` | `±115` | 按实机作业区（如 `±60`）| **飞行安全** ← 需拍板 |
| `box_min_z/max_z` | `-2.5 / 6.0` | 按实测地面 | **飞行安全** ← 需拍板 |
| `resolution` | `0.25` | 保持 | 资源 |
| `ground_height` | `-4.0` | 按实机实测 | 资源/正确性 |
| `seed_z` | `1.5` | 与实机起飞悬停高度一致 | **飞行安全** |
| `max_vel` / `max_acc` | `0.8 / 1.0` | 评审后再定 | **飞行安全** |
| `mid360_real_flight.yaml` 的 `fsm.goal_z_min/max` | 实机现状 `0.6/1.3` | 单独评审 | **飞行安全** ← 真正的限高在这里 |

**★ 安全参数的处理办法**：原值保留 + 注释标红 + 说明"只在 `enable_ctrl:=true` 时生效，
开启前必须单独评审"。**不要替用户改这些**。

---

## 五、实机脚本的设计（剥仿真 + 换闸）

### 前置闸：仿真三道 → 实机三道
| 仿真版检查 | 实机版换成 |
|---|---|
| AirSim RPC 41451 | ✗ 删 |
| `/clock` 步进（LockStep 冻结检测）| ✗ 删（实机真实时钟）|
| `/cloud_registered` 有数据 | ✓ **保留** |
| PX4 专用 MAVLink 14610/14611（`tmux send-keys`）| ✗ 删（实机 PX4 是硬件）|
| MAVROS 用 `udp://:14611@127.0.0.1:14610` **启动** | ✓ 改成**只检查** `/mavros/state` `connected: True`（实机上 MAVROS 通常已在跑）|
| — | ➕ `/aft_mapped_to_init` 有数据 |
| — | ➕ `fsm_node` 存活（要用 SUPER 执行时）|

### ★★ 停止脚本体现在实机上的安全差异（最容易致命的一处）
仿真版 `fuel_stop.sh` 会 `pkill mavros_node` ✗ **实机上 MAVROS 一断 = OFFBOARD setpoint 流中断
→ 可能触发 failsafe** ✗✗
```
实机版默认：不停 PX4、不停 MAVROS、不停 FAST-LIVO2（它可能还在给 SUPER 供位姿）
只有显式 --with-mavros / --with-px4 / --with-livo 才去动它们
--with-mavros 还要 read -r -p 让操作员二次确认"飞机已落地且已 disarm"
```
另外 **FAST-LIVO2 的 PCD 路径两者不同**：
仿真 `~/twin_ws/src/FAST-LIVO2/Log/pcd` → 实机 `~/catkin_ws/src/FAST-LIVO2/Log/pcd` ✗
**必须用 SIGINT（触发 savePCD），绝不 `kill -9`** ✓

### ★ 影子模式是默认状态（L0 零风险）
`fuel_real.launch` 里 **`enable_ctrl` 默认 `false`** ⇒ `fuel_ctrl_bridge` **根本不启动** ⇒
FUEL 只发布目标、**碰不到 PX4** ✓✓ **这是现成的、天然安全的起点，先跑它**。

### ★ `fuel_cmd_topic` 必须改道
与 SUPER 联调时传 **`fuel_cmd_topic:=/fuel/pos_cmd_sink`**（无人订阅的话题），
否则 FUEL 的 `traj_server` 会和 SUPER 的 `fsm_node` **抢 `/planning/pos_cmd` 的控制权** ✗
**验收判据：`rostopic info /planning/pos_cmd` 的发布者必须仍然只有 `fsm_node`。**

### ⚠️ 一个必须先问清的设计差异
仿真版 `fuel_all.sh` 里**FUEL 的控制桥负责起飞**（seed 悬停 → 自动 OFFBOARD → 起飞到 `seed_z`）。
实机上**起飞由 SUPER 的 `super_to_px4_bridge` 或人工/QGC 负责** ⇒
**实机版脚本不要写"起飞"这段**，前提是「飞机已在空中悬停」。
**动手前先向用户确认实机起飞流程。**

---

## 六、验证阶梯（逐级放行）
| 级 | 做什么 | 风险 | 通过判据 |
|---|---|---|---|
| **L0 影子** | 只启 FUEL，`enable_ctrl:=false` | **零** | `/exploration_goal` 频率正常、目标在作业区内、`/planning/pos_cmd` 仍只有 `fsm_node` |
| **L1 台架** | 通电不打桨 | 低 | FUEL 目标 → 仲裁器 → `/waypoint_goal` → fsm 收到；SUPER 轨迹正常 |
| **L2 系绳/开阔** | 限高不变、小范围、急停在手 | 中 | 自主移动、无 `A* NO_PATH` |
| **L3 树林** | 小范围、`TREE_EXPLORE_TIME` 短 | 高 | 覆盖率上升、能一键停 |

**最大风险是算力**：Orin NX 上 FAST-LIVO2 + SUPER + FUEL 并行 ⇒
**L0 就要实测 CPU/内存余量**（`top -bn1 | head -12` / `free -h`），
并确认飞行用低算力配置 `MID360_Fly.yaml`（`dense_map_en=false` + 抽稀 3 倍）✓

---

## 七、上机操作序列（模板）
```
1. pscp 传包 → fouruav:~/                （实测 fuel_planner+fuel_bridge 打包只有 ~500 KB / 274 文件）
2. cd ~/catkin_ws && git status          （确认那两个 M 是用户的，别动）
3. git checkout -b feature/fuel-exploration
4. tar xzf ~/fuel_real_pkg.tar.gz
   cp -a fuel_real_pkg/src/fuel_planner src/
   cp -a fuel_real_pkg/src/fuel_bridge  src/
5. cp fuel_real_pkg/launch/fuel_real_airborne.launch src/fuel_bridge/launch/
   cp fuel_real_pkg/scripts/*.sh ~/ && chmod +x ~/*.sh
6. catkin_make                            （只编新包）
   ★ 编完**复核 devel/lib 里 SUPER/FAST-LIVO2 产物 mtime 未变**，确认没破坏原构建
7. bash ~/fuel_shadow_real.sh             （L0）
```

## 八、包内必备的自证材料
`MANIFEST.md` 里放**关键文件 MD5**（两个桥、仲裁器、新 launch、新脚本），
上机后逐项 `md5sum` 复核 —— 证明传过去的是原样，而不是"应该是原样"。
同时写明"与仿真版的关系"：`fuel_bridge` 的 `.py` **逐字节相同**（直接 cp，未改）✓

## 九、动手前的机上侦察（一条命令，只读）
```bash
cd ~/catkin_ws && echo "== 分支/状态 ==" && git branch --show-current && git status -s | head -20 \
 && echo "== src ==" && ls src \
 && echo "== 有无同名 ==" && ls -d src/fuel_planner src/fuel_bridge 2>&1 \
 && echo "== 磁盘 ==" && df -h ~ | tail -1 \
 && echo "== 依赖 ==" && (dpkg -l | grep -E "^ii  libarmadillo-dev|^ii  libpcl-dev|^ii  libeigen3-dev|^ii  qtbase5-dev" | awk '{print $2, $3}') \
 && echo "== ROS ==" && echo ${ROS_DISTRO:-未设置} \
 && echo "== devel 回退基线 ==" && ls -l --time-style=long-iso devel/lib/super_planner/fsm_node 2>&1
```
**判读要点**：① `src` 里有没有同名包（本次无 ✓）② `libarmadillo-dev` 在不在
③ `devel/lib/super_planner/fsm_node` 的 mtime **记下来当回退基线** ④ 磁盘余量够不够编译
