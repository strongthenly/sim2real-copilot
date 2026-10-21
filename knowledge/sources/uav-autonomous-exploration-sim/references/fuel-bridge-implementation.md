# FUEL 桥实现（接真实平台，阶段④⑤ 实测通过）

> 场景：ROS1 Noetic WSL `ros1-twin`，链路 = AirSim(Windows) + PX4 v1.11.3 + `twin_bridge` + FAST-LIVO2 + MAVROS，把 **FUEL** 作为探索算法接进来。
> 规划路线见 `fuel-integration.md`；本文件是**已落地的实现细节 + 验证配方**。

---

## 1. 交付物：独立小包 `fuel_bridge`（不改 FUEL 源码）

```
~/fuel_ws/src/fuel_bridge/
├── package.xml            # 依赖加 quadrotor_msgs + mavros_msgs
├── CMakeLists.txt         # catkin_install_python 各桥/脚本
├── scripts/
│   ├── fuel_input_bridge.py    # FAST-LIVO2 → FUEL
│   ├── fuel_ctrl_bridge.py     # FUEL → MAVROS/PX4（含 seed 悬停）
│   ├── start_fuel_explore.sh   # 单起 FUEL（前置检查 + roslaunch）
│   ├── fuel_all.sh             # ★ 全链路一键启动（见 §11）
│   └── fuel_stop.sh            # ★ 一键关闭（见 §11，SIGINT 保 PCD）
└── launch/
    └── fuel_real.launch        # 只含算法，不含自带仿真
```
Windows 侧留备份同构目录，改完 **`cp` 过去 + md5 比对**（见 §5 坑 3）。

---

## 2. 输入桥 `fuel_input_bridge.py`

FUEL 地图模块（`MapROS`）要 **3 个输入**，从 FAST-LIVO2 一对一转发并**统一 frame_id**：

| FUEL 要的 | 来源（FAST-LIVO2） | 处理 |
|---|---|---|
| `/odom_world` (nav_msgs/Odometry) | `/aft_mapped_to_init` | `frame_id=world`, `child_frame_id=body` |
| `/map_ros/pose` (geometry_msgs/PoseStamped) | 同一位姿 | 抽出 `pose.pose` 发 PoseStamped（**raycast 用**）|
| `/map_ros/cloud` (sensor_msgs/PointCloud2) | `/cloud_registered` | `frame_id=world` |

要点：
- **不做坐标变换**——FAST-LIVO2 的点云已在世界系配准好，只把 `frame_id` 从 `camera_init` 改写为 `world`。
- 位姿同时发给 `/odom_world` 与 `/map_ros/pose`（相机↔机体外参量级几 cm，探索用途可忽略；要精确再乘外参）。
- 加个定时日志（`odom=N cloud=M`）便于判断桥是否在工作。

## 3. 输出桥 `fuel_ctrl_bridge.py`

```
订阅 /planning/pos_cmd (quadrotor_msgs/PositionCommand)
发布 /mavros/setpoint_position/local (PoseStamped, ENU)   ← 默认
     /mavros/setpoint_raw/local    (PositionTarget)        ← mode:=raw 时
服务 /mavros/set_mode + /mavros/cmd/arming（自动切 OFFBOARD + Arm）
```

四个必须做对的点：
1. **坐标系**：走 `setpoint_position/local`（ROS 侧 ENU）→ **MAVROS 自己转 PX4 的 NED**，不要手算 ENU↔NED。yaw 转四元数：
   `(0, 0, sin(yaw/2), cos(yaw/2))`。
2. **OFFBOARD 预发**（PX4 硬要求）：收到首条指令后先连续发 `offboard_wait`（默认 3.0s）setpoint，才调 `set_mode(OFFBOARD)`，再 `arming`。
3. **不许阻塞**：`rospy.Timer` 回调里**不能** `wait_for_service`（会卡死 50Hz 循环）→ 惰性建 `ServiceProxy`，调用处 try/except。
4. **状态变更类服务调用必须节流**：`set_mode` / `arming` 写在 50Hz 的 tick 里、又不加门控 ⇒ **日志刷屏**（实测用户 tail 到的是连续几十行 `[fuel_ctrl] → Arm`）。加 `self._next_action = now + rospy.Duration(2.0)`，每 2 秒最多动作一次。
   ⚠️ 顺带一个**误读陷阱**：`armed` 字段在 MAVROS 侧可能滞后/误报（无人机明明在飞却一直读到 `armed: False`）⇒ 别靠"日志还在打 Arm"就断定"没解锁"，**判飞行状态要看 `/mavros/local_position/pose` 有没有在动**。
5. **朝向对齐**（见 §12）——不做就会撞墙。

`mode:=raw` 时发 `PositionTarget`，`coordinate_frame=FRAME_LOCAL_NED`，`type_mask` 用"忽略位"组合（忽略 force + yaw_rate），这样才吃得下 FUEL 的速度/加速度前馈。

## 4. `fuel_real.launch` 要点

- **不要 include `simulator.xml`**（那是 `pcl_render_node`/`map_generator`/`so3_control`/`quadrotor_simulator_so3` 的 demo 仿真栈）。
- 保留：`algorithm.xml`（FUEL 核心 + 全部参数）、`traj_server`、`waypoint_generator`。
- `algorithm.xml` 的**必传 args**：`map_size_x_/y_/z_`、`box_min_*`/`box_max_*`、`odometry_topic`、`sensor_pose_topic`、`depth_topic`、`cloud_topic`、`cx/cy/fx/fy`、`max_vel`、`max_acc`。
- `traj_server` 里 `remap /position_cmd → planning/pos_cmd`、`remap /odom_world → $(arg odom_topic)`。
- 控制桥用 `<arg name="enable_ctrl" default="false"/>` + `<group if=...>` 包起来——**默认关闭，确认输入侧无误再开**，避免误触发飞行。
- 开关 `map_ros/frame_id`（默认 `world`）与 `sdf_map/resolution`（0.1）、`map_ros/max_ray_length`（4.5，要匹配自己的 LiDAR 量程）。

## 5. 验证配方（本轮实际用的，可复用）

**核心原则：ROS 转发验证必须"先订阅、后发布"**（publisher 默认不 latch，pub 之后才起的 `echo` 会错过消息，看起来像"桥没转发"）。

```bash
# 起独立 master（换端口，别用主 master）
setsid rosmaster --core -p 11340 >/dev/null 2>&1 </dev/null & sleep 3
setsid rosrun fuel_bridge fuel_ctrl_bridge.py _auto_offboard:=false >/dev/null 2>&1 </dev/null & sleep 4
# ① 先订阅
timeout 13 rostopic echo /mavros/setpoint_position/local >/tmp/f.txt 2>/dev/null &
sleep 3
# ② 再发布完整消息（别省字段！缺 velocity/acceleration 会让 pub 无效）
timeout 8 rostopic pub -1 /planning/pos_cmd quadrotor_msgs/PositionCommand \
 "{header: {frame_id: 'world'}, position: {x: 2.0, y: 3.0, z: 1.5}, \
   velocity: {x: 0.0,y: 0.0,z: 0.0}, acceleration: {x: 0.0,y: 0.0,z: 0.0}, \
   yaw: 1.5707963, yaw_dot: 0.0, trajectory_flag: 1, trajectory_id: 0}" >/dev/null 2>&1
sleep 5
grep -q "x: 2.0" /tmp/f.txt && grep -q "z: 0.70710" /tmp/f.txt   # 位置 + yaw→四元数
```
- `yaw=π/2` ⇒ 期望 `orientation.z = w = 0.7071`（验证 yaw 转换正确）。
- 输入桥同理：喂假 `/aft_mapped_to_init`(Odometry) + `/cloud_registered`(PointCloud2)，断言 3 个输出话题**都收到且 `frame_id=world`**。
- 端到端（需 AirSim Play）：`rostopic hz /sdf_map/occupancy_all` 有数据，且其点云坐标范围应≈自己的 LiDAR 量程（实测 x/y ∈ ±4.5m、z 0~2.45m）→ **这就是"FUEL 用真实数据建图成功"的判据**。

## 6. 本轮踩过并已修的坑

| 坑 | 症状 | 修法 |
|---|---|---|
| **launch 注释含 `--`** | roslaunch 报 XML 解析失败；`ET.parse` 也失败 | `<!-- 标题 -->`，别写 `<!-- ---- 标题 ---- -->` |
| **`wait_for_service` 写在 Timer 回调** | 桥 3s 后"卡住"、setpoint 不再发 | 惰性 `ServiceProxy` + try/except |
| **改了 Windows 侧没同步到 WSL** | 静态检查莫名 FAIL（跑的是旧版） | `cp` + **md5 对比** |
| **测试消息字段不全** | pub 无效 → 断言全 FAIL（误判成桥坏） | 发完整 `PositionCommand` |
| **测试时序反了** | "桥没转发" | **先订阅再发布** |
| **`grep wait_for_service` 命中注释** | 假 FAIL | 匹配真实调用 `rospy\.wait_for_service(` |
| **内联 `bash -c` 里 `pkill -f`** | 命令自杀（无输出/退出 15） | 清理逻辑落 `.sh` 文件 |
| **把 FAST-LIVO2 初始化当故障** | 刷 `[LIO]: No point!!!` | 等 30~60s 到 `Update Voxel Map` |
| **FUEL 起来了但无人机原地悬停** | 链路全绿，日志还在打 `to visit: N` / `cost mat size` | FUEL 停在 `WAIT_TRIGGER`，**必须显式触发**；`cost mat` 出自 0.5s 的 `frontierCallback`、**不代表探索在跑** —— 判据看 `Triggered!` 与每秒一行 `[FSM]: state:`（见 §9）|

## 9. ★ 阶段⑤ 头号误判：**FUEL 不会自己飞**（必须显式触发）

FUEL 的 FSM 启动后停在 **`WAIT_TRIGGER`**
（`state_str_ = INIT / WAIT_TRIGGER / PLAN_TRAJ / PUB_TRAJ / EXEC_TRAJ / FINISH`），
收不到 `/waypoint_generator/waypoints` 就**永不规划** → 无人机原地悬停（用户 2026-09-12 实报）。

- **触发**：`rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped
  "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"`
  （RViz 的「2D Nav Goal」发的是同一个话题）。
  ⚠️ **`z` 必须 ≥ −0.1** —— `triggerCallback` 里 `if (msg->poses[0].pose.position.z < -0.1) return;` 会**静默丢弃**负 z。
- **日志陷阱**：`to visit: N` / `cost mat size final: (N,N)` 出自 **`frontierCallback`（`createTimer(0.5s)`，只要收到 odom 就一直在跑）**，
  与"探索是否启动"**无关**。判据只看 **`Triggered!`**（`triggerCallback` 打印、一次性）与
  **`[FSM]: state: XXX`**（`FSMCallback` 每秒一行）；停 `WAIT_TRIGGER` = 没触发。
- 触发后应立刻出现：`Triggered!` → `[FSM]: state: PLAN_TRAJ → PUB_TRAJ → EXEC_TRAJ`、
  `Next view: ...`、`[Kino replan] ... Total time: 0.014`，且 `/planning/pos_cmd` ~100Hz、无人机位置开始变。
- **纪律**：无人机不飞时先 grep `Triggered!`/`[FSM]: state:`，别先查 OFFBOARD/控制桥/坐标对齐（本会话这三样当时都正常）。
- 完整源码与对照日志见 `airsim-drone-exploration` 的 `references/fuel-planner-integration.md` §11。

## 10. 相关运维：PX4 一重启，专用 MAVLink 专线就没了 → MAVROS 静默掉线

`hover_verify*.py` 卡在 `等待 FCU 连接 ...`（阻塞式、**不会超时**）＝ `/mavros/state.connected = False`。
**九成是 MAVROS 连的那条专线已不存在**：接入时开的是**专用** MAVLink
（`mavlink start -u 14610 -o 14611`），它**随 PX4 进程存在**；用户重跑一键脚本拉起**新 PX4** 后，
新进程上没有这条专线，旧 MAVROS 还连着老端口 → 掉线（`mavros_node` 进程还在但 `connected:False`，
`/mavros/state` 的 `stamp` 停在掉线那一刻）。
**重建顺序**：① 确认 `ss -tn | grep ':4560'` ESTAB → ② `tmux send-keys -t px4 "mavlink start -u 14610 -o 14611" Enter`
→ ③ 重启 `roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610` → ④ 验 `connected: True`
（`VER: 1.1: ...` 打印 = 已握手；`CMD: Unexpected command 520` 非致命）。
⇒ 一键脚本若含 PX4，就该把"开专线 + 起 MAVROS + 自检 connected"一起做掉。

## 7. 阶段实测结果（可作为论文素材）

| 阶段 | 结果 |
|---|---|
| ① 依赖 | `libarmadillo-dev` 自带；`nlopt` 源码装 v2.7.1；**LKH 随 FUEL 编译**（`file(GLOB src/*.c)`）|
| ② 编译 | 25 个包，`catkin_make -j4` **rc=0、0 error** |
| ③ 自带 demo | 自主探索跑通；**规划周期 Total 14.4ms**（Cost mat 3.5 / TSP(LKH) 1.9 / Local refine 0.36 / Kino 0.2 / Traj 5.8），比论文 i7-8550U 的 24-36ms 更快 |
| ④ 真实数据 | `/odom_world` 10.4Hz、`/map_ros/cloud` 9.8Hz → `/sdf_map/occupancy_all` 4.1Hz；**建图范围 ±4.5m 与 LiDAR 量程吻合**；frontier 簇 20、TSP 矩阵 (20,20) 正常维护 |
| ⑤ 控制桥 | 映射逻辑（位置 + yaw→四元数）单测通过；**端到端飞行未跑**（需人工在 UE4 点 Play）|

> 论文可用的对比口径：规划耗时（ms）、探索时间/路径长度/覆盖率、frontier 簇数随探索收敛曲线。

## 8. 基础论文与 baseline 备份

- FUEL 论文 `arXiv:2010.11561`（RA-L 2021），DOI `10.1109/LRA.2021.3051563`。
- 同方向免费可下的后续（IEEE 付费墙的 FGIP/EFP 无 OA，Unpaywall 查 `is_oa:false`）：
  - **EPIC**（轻量 LiDAR 探索框架，`arXiv:2410.14203`）
  - **SHIELD**（球面投影 hybrid-frontier，LiDAR 探索，`arXiv:2512.23972`）
- 取论文文本：`python -c "import fitz..."`（pymupdf）按页 `page.get_text()` 写 txt，再逐段读；
  脚本内路径用 `os.path.dirname(__file__)` 拼，**别硬编码绝对路径**（文件夹一挪就失效）。

---

## 11. 一键生命周期 + `init_z`/`seed_z` 耦合 + PCD 保存（2026-09-12 实测）

### 11.1 为什么要"seed 悬停"（先有鸡还是先有蛋）

矛盾链条：**FUEL 未触发 → 不出 `pos_cmd` → 控制桥无 setpoint → PX4 不许切 OFFBOARD → 无人机起不来 → 也就永远无法触发探索**。

解法：控制桥加 **`~seed_z`**（默认 1.5）—— 在收到首条 FUEL 指令前，用 `~` 的当前位姿持续发 `(cur_pos.x, cur_pos.y, seed_z)` 的 `PoseStamped`（订阅 `/mavros/local_position/pose` 拿 `cur_pos`），把无人机**先托到该高度并切 OFFBOARD**；一旦 FUEL 出 `pos_cmd` 就无缝接管。`seed_z<=0` 关闭该行为。

### 11.2 ★ 头号隐蔽坑：`init_z` 必须 = `seed_z`

`traj_server.cpp`（`nh("~")` 读参）**启动时无条件先发一条占位指令**：

```cpp
nh.param("traj_server/init_x", init_pos[0], 0.0);
nh.param("traj_server/init_z", init_pos[2], 0.0);
...
cmd.position.x = init_pos[0];  cmd.position.z = init_pos[2];   // ← 启动即发出
```

后果：控制桥的 `last_cmd` 立刻非空 ⇒ **永远走 FUEL 分支、再也不走 seed 分支** ⇒ 无人机飞到 `init_z` **锁死悬停**（实测：设 1.5m 却停在 **1.0m**，`/planning/pos_cmd` 显示的是 `(0, 0, 1.0)` 而非 `(cur_pos.x, cur_pos.y, 1.5)`——一眼可辨），而 `fuel_all.sh` 死等 `seed_z` 永不满足 ⇒ 卡在 `5/6`。

修法两条（都要做）：
1. `<arg name="init_z" default="$(arg seed_z)"/>` —— **单一来源**，别再写死 1.0；
2. 等待逻辑加兜底：**高度连续 N 次几乎不变且 >0.3m 即认为已稳定悬停**，不必硬等具体数值。

排查要点：
- param 真实路径是 **`/traj_server/traj_server/init_z`**（`<node name="traj_server">` + `<param name="traj_server/init_z">` 叠加，且代码用 `nh("~")`）—— 查 `/traj_server/init_z` 会报 "not set" 而误判"参数没生效"。
- **判定是"命令错"还是"跟不动"**：同时 echo `实际位置 /mavros/local_position/pose` 与 `setpoint /mavros/setpoint_position/local` + `rostopic info` 看发布者。本轮 setpoint=(0,0,1.0) 而 `cur_pos`=(-0.004,-0.007) ⇒ 说明**不是我的 seed 在发**，一步定位到 traj_server 占位指令。

### 11.3 PCD 保存（必须用 SIGINT 关）

| 项 | 值 |
|---|---|
| 开关 | `pcd_save/pcd_save_en` (bool，默认 false) + `pcd_save/interval` (默认 -1) |
| 落盘 | `~/twin_ws/src/FAST-LIVO2/Log/pcd/all_raw_points.pcd` + `all_downsampled_points.pcd` |
| 触发时机 | `main()` 在 `ros::spin()` **返回后**调 `savePCD()` ⇒ **只有 SIGINT/SIGTERM 正常退出才存**，`kill -9` 直接丢图 |

- 启动脚本加 `PCD_SAVE=1` 环境变量：起 FAST-LIVO2 **之前** `mkdir -p $ROOT/Log/pcd` + `rosparam set /pcd_save/pcd_save_en true` + `rosparam set /pcd_save/interval -1`。
- ⚠️ **参数在节点构造时读一次** ⇒ FAST-LIVO2 **已在跑时改它无效**，脚本要主动提示"先 stop 再起才生效"。
- 关闭脚本：`pkill -INT -f fastlivo_mapping` 然后**轮询等它退出（最多 ~25s）**，退出后 `ls -lh $ROOT/Log/pcd` 回报文件。
- ⚠️ **兜底别再发 `SIGTERM`——roscpp 只响应 SIGINT，不响应 SIGTERM**（见 `uav-ros-mapping-lifecycle` 的「PCD 保存的 SIGINT 语义」节，那里有完整机制：`savePCD()` 只在 `while(ros::ok())` 退出后跑一次）。所以「等 N 秒还在就 `kill -TERM`」这种写法是**无效兜底**，进程会一直挂着。正确做法：超时后**重发 `SIGINT`**，或干脆 `SIGKILL` 并**明确告知用户这次不保存**（`--no-save` 的语义）。

### 11.4 一键脚本的参数与开关

```bash
# 启动：seed_z / mode / goal_x / pcd
bash ~/fuel_all.sh 1.5 position 3.0 1     # 开 PCD 保存
# 关闭：默认保留 ROS master 常驻（重启只要 4s）
bash ~/fuel_stop.sh                       # 停 FUEL+MAVROS+FAST-LIVO2+桥+PX4+RViz，留 master
bash ~/fuel_stop.sh --all                 # 连 master 一起停
bash ~/fuel_stop.sh --no-save             # FAST-LIVO2 用 SIGKILL（不等存 PCD）
bash ~/fuel_stop.sh --keep-px4            # 保 PX4（LockStep 下停了 AirSim 就冻结）
```
停止顺序：FUEL(含两个桥) → MAVROS → **FAST-LIVO2(SIGINT 存 PCD)** → 仿真桥 → RViz → PX4 → (可选) master。

### 11.5 roslaunch 参数三个坑（都是"静默失效"型）

| 坑 | 症状 | 修法 |
|---|---|---|
| `<arg value="x"/>` **不可被命令行覆盖** | 传 `init_z:=1.5` 完全无效、静默用写死的 1.0 | 要可覆盖必须写 **`default=`**（`value=` 是固定值） |
| `$(arg X)` **前向引用** | `RLException: arg 'X' is not defined` | 被引用的 arg 必须**先定义**（把 `seed_z` 的定义提到 `init_z` 之前） |
| **param 路径叠加** | `rosparam get /traj_server/init_z` 报 not set | `<node name="N">` + `<param name="N/x">` = **`/N/N/x`** |

验证姿势：`roslaunch <pkg> <file>.launch <args> --dry-run` 能**真解析**（含 include）而不启动——前向引用/XML 非法会被它抓出来。

### 11.6 "点云时有时无" = 仿真冻结（不是等不够久）

```
PX4 死 / tmux server 死 → TCP4560 断 → LockStep 下 AirSim 不步进
   → /clock 停 → 所有传感器话题（/twin/lidar、/cloud_registered）全无数据
```
**一条命令判死**：`rostopic hz /clock` —— 不走就是冻结，等再久也没用（用户明确说过"不是没等 90s 的问题"）。配套：
```bash
ss -tn | grep ':4560'    # 无 ⇒ PX4 未连 AirSim
pgrep -f bin/px4         # 无 ⇒ PX4 没起来
tmux ls                  # 无 px4 会话 ⇒ tmux server 死了（PX4 被 SIGHUP 杀）
```
健壮启动（`--full`）按序自检并报**具体**错误：`4560 连接 → /clock 步进 → /twin 出数据 → /cloud_registered`（最后一项超时则**自动重启 FAST-LIVO2，最多 3 次**）。

### 11.7 验证脚本自身的两个小坑

- **`grep -q "--no-save"` 会把 `--no-save` 当成 grep 选项**报 `unrecognized option` ⇒ 用 `grep -q -e "--no-save"`。
- **浮点别做字符串相等**：`pos_cmd.z` 实测 `1.51`（vs 期望 `1.5`）⇒ 用 `awk -v a="$z" -v b="$T" 'BEGIN{exit !(a>b-0.1 && a<b+0.1)}'` 容差比较；抽数字用 `tr -dc '0-9.-'`。
- 验证 `init_z` 是否真继承：**真实起两次 `roslaunch`（`seed_z:=1.5` / `2.5`），读 `/traj_server/traj_server/init_z` 并订阅 `/planning/pos_cmd/position/z` 看占位指令**——比静态 grep 有力得多。

---

## 12. ★ 无人机「疯狂撞墙 / 飞错方向」：先切分「算法层 vs 执行层」

**第一件事是分层，别乱查。** 判据：

| 现象 | 结论 |
|---|---|
| FUEL 日志里 `Next view: x y z, yaw` **在持续更新**、`[FSM]: from EXEC_TRAJ to PLAN_TRAJ` 正常循环 | **算法层是好的** —— 它在正常规划。问题在**执行层**（发的目标 ≠ 无人机实际飞的方向） |
| `Next view` 不动 / 没有 `[FSM]` 行 | 回到 §9：没触发，或仿真冻结（§11.6） |

**执行层头号原因：两个坐标系的 yaw 没对齐**（实测 2026-09-12 用户报"疯狂撞墙"）

```
算法侧 world = SLAM 的 camera_init        ← 原点/朝向 = SLAM 初始化那一刻 LiDAR 的朝向
PX4 侧  local = EKF home / MAVROS ENU     ← 朝向来自磁力计 / 初始姿态
```
两者**初始朝向大概率不同** ⇒ 算法说"往左 3m"，PX4 理解成"往前 3m" ⇒ 朝真实墙壁全速飞。
（位置原点差通常是次要的，因为两边都在起飞前初始化；**yaw 才是主犯**。）

**一键诊断（只读，不碰控制）**：`scripts/check-frame-alignment.sh [master_uri]`
- 对比 `/aft_mapped_to_init`(算法) 与 `/mavros/local_position/pose`(PX4) 的 **yaw 与位置**
- **YAW 偏差 > 10°** ⇒ 就是它，修法见下方「对齐的落地实现」
- 位置偏差 > 0.5m ⇒ 再加**平移对齐**（起飞时记一次位置差）——本实测**只用 yaw 就够**（原点差仅 5mm）
- 两者都 < 阈值 ⇒ 去查下面的次要原因

**实测数值（2026-09-12，ros1-twin）**：静止时算法侧 yaw = **0.00°**、PX4 侧 yaw = **82.24°**；桥自动估计得到 **psi_offset = 82.33°** ✓ 二者吻合 ⇒ 说明这条链路每次新起点会得到不同的固定偏移（取决于 SLAM 初始化那刻的机头朝向），**必须每次自动估、不能写死**。

#### 对齐的落地实现（`fuel_ctrl_bridge.py`，已验证）

```python
psi_offset = yaw_px4 - yaw_fuel          # 两坐标系的固定朝向差，与无人机姿态无关
p_px4     = Rz(psi_offset) · p_fuel      # 位置：旋转
yaw_px4   = yaw_fuel + psi_offset        # 朝向：加偏移
```

四个实现要点：
1. **订阅两个位姿做差**：`/odom_world`（算法系 yaw）与 `/mavros/local_position/pose`（PX4 系 yaw），`psi = wrap_pi(yaw_px4 - yaw_fuel)`。
2. **用 sin/cos 平均，别用算术平均**：取够 `~align_samples`（默认 15）个样本后 `psi = atan2(Σsin, Σcos)` —— 算术平均在 ±180° 附近会算错。估一次后**锁定**（`psi_offset is not None` 就 return）。
3. **★ 未对齐前拒绝转发**（防撞保险）：tick 里收到 FUEL 指令后先 `if not self.aligned(): logwarn_throttle(...); return` —— 宁可让无人机悬停，也别把目标朝错误方向送出去。这条保险实测**真的触发过**（日志里能看到 `朝向对齐未完成，暂不转发 FUEL 目标（防撞）` 紧跟着一秒后的估计完成）。seed 悬停阶段**不做转换**（`cur_pos` 本来就取自 PX4 系）。
4. **`raw` 模式连 velocity / acceleration 一起旋转**，不只转位置。
   参数：`~auto_align`(默认 true) / `~align_samples` / `~psi_offset_deg`（手动指定则跳过自动估计）。

#### 实时残差验证（判"对齐到底对不对"的最强判据）

原理：若对齐正确，则应有 **`p_px4 ≈ Rz(psi)·p_fuel`**。脚本 `fuel_align_check.sh` 连续采样算残差：

```bash
bash ~/fuel_align_check.sh 12      # 采样 12s，只读
# 平均残差 < 0.3m ⇒ 对齐正确；> 1m ⇒ psi 符号反了（脚本会给出 -psi 的手动补救值）
```

⚠️ **两个读图陷阱（本轮都踩了）**：
- **无人机在快速旋转时，psi 会乱跳（实测同一段日志里 16.4° / 84.4° / 119.3° / 149.2°）** —— 因为算法侧和 PX4 侧两个话题的 yaw 是**不同时刻**采样的，姿态一变快，做差就没意义。**只看姿态稳定那几个采样点**（本轮稳定的两点给出 psi=84.4°/82.8°、残差 0.57/0.66m ✓），别被抖动值误导成"对齐坏了"。
- 残差 0.5~1m 属**滞后**（无人机正在追目标），不是错；判据是量级不是 0。

**次要原因：FUEL 只避让「已知」障碍**
`algorithm.xml` 的 `max_ray_length`（默认 4.5 = 自家 demo 的值）决定它**只看得见这个距离内、且已被 LiDAR 扫到的**障碍。
在它自己的仿真里安全，但在高保真场景 / 实机：**未知区域被当作可通行**，还没扫到的墙会被直接规划穿过去 ⇒ 撞。
出路：把 `max_ray_length` 调到**匹配自己 LiDAR 的实际量程**（如 8~10m），并压低 `box_max_z`（如 2.0m）减少撞高墙概率。

**安全纪律（重要）**：用户报"撞墙"时**先 `bash ~/fuel_stop.sh`**，然后**只起 twin 链路 + MAVROS** 做诊断 —— 别让 FUEL/控制桥继续接管无人机；诊断脚本一律**只读**（不 `publish`、不 `set_mode`、不 `arming`）。
本仓库 `fuel_frame_check.sh` / `check-frame-alignment.sh` 就是按这个纪律写的（可作为"只读诊断脚本"的模板：只 `rospy.wait_for_message`，不建任何 Publisher）。

---

## 13. ★ 防撞参数：FUEL 默认值是给"它自家 demo"调的，接真实平台必须重调

坐标对齐修好之后**仍可能贴墙** —— 因为 FUEL 的安全余量是**按它自己小场景 + 点式无人机**配的。用户报"疯狂撞墙"（2026-09-12）时，参数层的账是这样：

| 敏感参数 | FUEL 默认 | 本机改为 | 为什么 |
|---|---|---|---|
| `sdf_map/obstacles_inflation` | **0.199** | **0.45** | 安全余量只有 20cm，而真实机身 ~0.5m ⇒ **等于没有余量**。这是最该先动的一个 |
| `sdf_map/max_ray_length` | 4.5 | **8.0** | 只看 4.5m ⇒ 目标常落在**还没扫到的**墙里（见 §12 次要原因）|
| `manager/clearance_threshold` | 0.2 | **0.45** | 规划要求的最小安全间隙 |
| `frontier/min_candidate_clearance` | 0.21 | **0.45** | 视角点候选也要离墙够远，否则 viewpoint 本身就贴着墙 |
| `manager/max_vel`（经 arg） | 1.5 | **0.8** | 1.5 m/s 配 20cm 余量 ⇒ 容错时间 ~0.13s，建图/控制稍微滞后就撞 |
| `manager/max_acc`（经 arg） | 1.5 | **1.0** | 同上 |

**改哪里**：
- 前四个是 `algorithm.xml` 里**写死的 `<param>`**（不是 arg）⇒ 只能改文件本身。
- `max_vel` / `max_acc` 是 **arg**（`<param name="manager/max_vel" value="$(arg max_vel)"/>`）⇒ **在 `fuel_real.launch` 里改默认值即可，别动 FUEL 源码**。

**改动约定（便于日后回溯）**：
1. 就地在 `algorithm.xml` 改值，并在同一行尾加 `<!-- FM: 旧值->新值 理由 -->` 注释；
2. 备份 `algorithm.xml.bak_fm`（同目录）+ 桌面工作区放一份 `algorithm_FM.xml`；
3. 改完**必须 `ET.parse` 验一次 XML 仍合法**（沙盒里手写 XML 极易破）。

**顺手确认 / 别误判**：
- `heading_planner/max_yaw_rate` **默认已是 10°/s** —— 说明 FUEL 内部规划的 yaw 本来就是平滑的。所以看到无人机"疯狂自转"时，**不要归咎于 FUEL 的规划**，要去查实际姿态（`/mavros/local_position/pose` 的 yaw 是否在跳）与控制指令是否跳变。
- 调完参数**不等于不会碰墙**：FUEL 的信息增益驱动本质决定了它把**未观测区域当可通行**，"刚转过墙角、LiDAR 还没扫到"时仍可能贴墙。要更保守：`max_vel` 再降到 0.5，或**缩小探索边界**（`box_max_*`，如收到 ±10）在更小场景里跑。
- 调参前先确认**不是坐标对齐问题**（§12 分层），否则调参数白费功夫。
