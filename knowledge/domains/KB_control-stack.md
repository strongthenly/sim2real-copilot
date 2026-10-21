# 规划与自主飞行（SUPER / 复飞 / 避障 / 目标仲裁）

> 领域 `control-stack` ｜ 由 4 个原始技能合并而成 ｜ 生成于 2026-10-21

**这一层解决什么**：飞机飞哪里、怎么绕障、怎么复飞、多个模块抢目标时谁说了算——这一层出问题的后果最直接（撞、悬停、原地徘徊）。

**建议读法**：A 两套复飞系统的区别 → B 按钮与起飞状态机 → C 高度三层语义 → D 坑位速查 → E 参数/日志判读

**包含的原始技能**：`uav-super-navigation`、`uav-trajectory-replay`、`uav-super-flight-replay`、`super-planner-replay`


---

### 来源技能 `uav-super-navigation`

> fouruav（Orin NX + MID360S + D435 + FAST-LIVO2 + PX4）上 SUPER 规划器轨迹复飞功能的运维与验证：两套复飞系统（Path Replay HTTP/UDP 回放 vs SUPER 航点任务）的区别、地面验证流程、航迹录制与抽稀、QGC 按钮 1/2/3 逻辑、bridge 起飞状态机、已知坑位清单。另含把 SUPER 移植到纯仿真（ros1-twin + AirSim + FAST-LIVO2）并与 FUEL 组成「FUEL 决策 + SUPER 执行」探索闭环：一键启停脚本、目标仲裁与防抢控制、PX4/tmux 掉线诊断。适用于"指挥官模式"出命令判读，或直执行排查。

#### ⭐ "RViz 画面不动 / 探索卡住" 的第一反应

**先分清三类再动手**（误判几乎全出在没分清）：
**A** 数据没在变（`/clock` 停）· **B** 数据在变但 RViz 不画（Fixed Frame / use_sim_time / Decay）·
**C** 没东西可画（最常见）。

**完整诊断手册 + 三个会得出反向结论的诊断方法错误 + 本次完整因果链 + 一次性只读体检脚本见
`references/rviz-frozen-and-exploration-stall-diagnosis.md`。**

三个必须记住的方法错误：
1. `rosrun tf tf_echo` 查不到 frame **不代表 TF 断** —— 它没读 `/use_sim_time` 就用墙上时间查仿真时间，
   必然查不到。要看 frame 对就直接：
   `rostopic echo -n 30 /tf | grep -E "frame_id|child_frame_id" | sed 's/^ *//' | paste - - | sort | uniq -c`
2. `pgrep -f <节点名>` 判节点死活**不可靠**（节点名 ≠ 进程名）→ 用 `rosnode list` / `rostopic info`
3. 只抓 **1 条** `/tf` 消息就断言 TF 树内容 ✗（一条消息可含多个 transform）

**已知长期故障（不是本次某次改动引入的）**：`exploration_node`（FUEL）反复 `exit code -6`（SIGABRT）——
用 `grep -rE "died|exit code" ~/.ros/log/` 可判定"老问题 vs 新引入"，**不修它探索永远跑不完**。
典型死锁链：FUEL 崩 → arbiter 靠"保持超时 20s 强制放行"在**两个缓存目标间空转** →
目标每 20s 反向 180° → `yaw_traj_opt.cpp:188 "Yaw rate too large"`（需求 4.9~6.0 vs 上限 2.5）
→ `/planning/pos_cmd` 不发 → 桥 `[POS_HOLD]` 数值全不变 → 飞机悬停 → RViz 无物可画。
**`Yaw rate too large` 不是 `max_vel` 的锅，是目标反复反向的结果。**

#### ⭐ 新场景：松树林（真实点云 → UE4/AirSim 世界）操作要点

**迭代期经验（"加树密度 / 加粗壮大树 / 改起降坪尺寸"这类需求）见
`references/pointcloud-world-v4-v5-tree-density-and-inplace-updates.md`** ——
含：树计数的**双方法交叉验证**（冠层峰值法 ~230 vs 树干聚类法 187，停在实测上限不合成）、
人工增补树的选位规则（**林内空隙**，不是林子外）、起降坪压平（⚠️ 必须从原始地形重算）、
**"只改局部就原地更新资产、不新建关卡"**、UE 编辑器定位、交付文档两份制。

**管线全貌 + 12 个实测坑见技能 `pointcloud-to-ue4-airsim-world`**（真实 LiDAR .las → 地面滤波 →
AGL 分层 → 体素合并碰撞盒 → OBJ → UE4 Python 自动建关卡 → 启动验证）。这里只记交接事实：

- **关卡** `/Game/PineForestV4/PineForestV4`（当前版；v1→v2→v3→v4 逐版演进，v2/v3 保留可切换对比）
  ——从 `/Game/FUEL/FUEL_RandomRoom` **派生**，
  World Settings 的 GameMode 是 `AirSimGameMode` ✓ **必须**是它（AirSim 的 SimMode+RPC 靠它拉起；
  自己 `new_level()` 建的关卡是默认 `BlocksGameMode` → 关卡能加载但 41451 **永不**监听）
- **一键双击入口**（用户明确要"自己可以点击运行那种"）：桌面 **`启动松树林仿真.cmd`**
  （⚠️ 不是 `.lnk`：同名 `.lnk` 会与同名**文件夹**在资源管理器的"文件夹区/文件区"分开显示而看漏；
  且 `.cmd` 内容必须 **100% ASCII**——cmd 解析器按 ANSI/GBK 走，中文路径会乱码并打乱引号配对，
  详见 `powershell-automation` 的 `references/cmd-launcher-encoding.md`）
  → 菜单 `[1]等就绪 [2]PX4+探索栈 [3]一键全部 [4]全部停止 [5]状态 [6]打开UE编辑器`；
  也支持 argv 模式 `python forest_sim.py 1|2|3|4|5` 供 agent 自动验证
- **起 UE4 必须加 `-nosound`**：命令行启动时音频线程死循环（每帧刷 `Waited 800 ms for audio thread`，
  帧号 38→323 花 11 分钟 ≈ 0.4 fps）→ 世界几乎不 tick → AirSim 初始化完不成 → RPC 起不来
- **⭐ 绝对不要用 `Test-NetConnection` 探 4560**：AirSim 的 HIL 口是 TCP server、接受任何连接，
  一探就被它**当成 SITL 连上** → 之后永远刷 `not receiving any messages from HIL`，PX4 再连也占不上 ✗
  → 判据只用 **41451**；要看 4560 就用 `netstat -ano` 解析 LISTENING（零连接零副作用）
- **启动顺序（错了就卡死）**：① UE4 树林关卡 → 等 41451（实测 **27 秒**）→ ② WSL 起 PX4
  （`~/px4_sitl.sh` = `cd ~/PX4-AirSim && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log`）
  → ③ `~/super_explore_all.sh 1`
- **AirSim 生成点**已按 NED 写进 settings.json（`NED_X=UE_Y, NED_Y=UE_X, NED_Z=−UE_Z`）：
  `X=−23.88 Y=5.26 Z=−6.58` = UE4 世界 (5.26, −23.88, 6.58) m，离地 1.5 m 的 3 m 净空点
- **场景内容**：真实起伏 **38.2 m** 的坡地 + **22,259 个方块**（AGL 0.3~5.5 m 的碰撞体，
  `CTF_USE_COMPLEX_AS_SIMPLE`）——**所见即所撞**，飞 0~5 m 时真的会撞树干/灌木
- **回退**：`settings.json.bak_before_pine_20260927_170159` + 关卡参数换回 `/Game/FUEL/FUEL_RandomRoom`
- **造世界时踩过的自伤**：`EditorLevelLibrary.new_level()` 对**已存在**的关卡返回 False 且**不新建**，
  此时 spawn 的东西会落进**当时打开的那个关卡**（= `DefaultEngine.ini` 的 `EditorStartupMap`）
  → 我曾把 3 个 actor 灌进用户的 `FUEL_RandomRoom` 并保存 ✗（已删回，留底 `E:\pine1225\FUEL_RandomRoom.umap.bak_before_fix`）。
  **教训：每次 spawn 前打印 `get_editor_world().get_path_name()` 确认操作的是对的关卡。**

#### ⚠️ 诊断纪律（本会话连错 4 次的教训，先读这条再下结论）

**我这次有 4 次"从代码/配置推断根因"，4 次全错，每次都要用户贴现象或查日志才纠正。**
结论：**在这个项目里，读源码/配置得到的只是【假设】，运行时日志与实测数据才是【证据】。**

| 我的错误断言 | 实际 | 正确做法 |
|---|---|---|
| "FUEL 的 `/planning_vis/frontier` 不发布"（只 grep 了 `drawFrontier`） | `fast_exploration_fsm` 里用 `drawCubes(..., "frontier", i, 4)` 画同一件事，**一直在发** | **grep 一个函数名不能证明功能不存在** —— 同一个能力常有多条调用路径（不同 FSM/不同重载）。断言"没实现"前必须搜**能力关键词**（话题名/常量/字符串），不是某个函数名 |
| "WSL 下 RViz 拿不到 GPU，必须 chmod /dev/dri" | `/tmp/rviz.log` 明写 `OpenGL device: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)` | **先读该进程自己的日志**（RViz → `/tmp/rviz.log`），再谈环境归因 |
| "`use_sim_time` 没设，所以 RViz 用墙上时钟" | `bridge.launch` 里 `<param name="use_sim_time" value="true"/>` 早设了 | **参数存在与否要问运行时**（`rosparam get` / 节点日志），别只看"我启动的那个脚本里没写" |
| "地面以上 6 m 内 13,389 个体素" | 用了全局 1% 分位当"地面"，而地形起伏 50 m ⇒ 那个数其实是"最低洼地带" | **统计量必须先确认基准是否成立**（见 `airsim-uav-simulation` 的 AGL 陷阱） |

**四条硬规矩**：
1. **先读运行日志再断言**：`/tmp/*.log`（rviz / bridge / fsm / px4_run / fastlivo）优先于源码阅读。
   用户"报现象"时，第一句先问清**现象的形状**（完全没有 / 间歇 / 卡住 / 变慢），别猜。
2. **"多来源"必查**：`/tf`、话题发布者这类东西**天然可能有多个发布者**（本次：`/laserMapping`+`/mavros`+`/fsm_node` 都发 `/tf`）。
   断言"谁发的/哪个是对的"前先 `rostopic info` 列全，并**两次采样看数值是否变化**。
3. **`grep` 一个符号 ≠ 功能不存在**：搜**话题名/字符串常量/枚举**，并看**所有**调用点（本例 4 条路径分属不同 FSM）。
4. **诚实标注"我错了"并当场更正**：本次三次更正都是先认错再给新结论（用户明确要"分级诚实汇报"）。
   错了就撤回（含撤回已经给用户的命令，例如那个不必要的 `sudo chmod 666 /dev/dri`），别让它留在对话里继续误导。

> ⚠️ **本文坑位字母有重复**（`I`/`L`/`M`/`O`/`P`/`Q` 各出现两次，是分批追加时撞号）。
> **不要用字母交叉引用** —— 按**标题关键词**检索（如"14610 是 UDP 端口""RViz 启动前必须显式 use_sim_time"）。
> 需要引用时写标题不要写编号。

#### 核心架构：两套独立复飞系统（先分清再动手）

| | A: Path Replay（纯回放） | B: SUPER 航点任务（规划+避障） |
|---|---|---|
| 实现 | `nx_command_server.py` 内嵌 daemon | `real_flight.launch`：fsm_node + super_to_px4_bridge + waypoint_mission |
| 指令通道 | UDP 6003 明文（RecordPathStart/Stop/Replay/AbortReplay） | QGC 按钮 → UDP 6003 → User_define_commands.yaml |
| 数据源 | `My_Flight_Path.txt`（CSV 7列 x,y,z,qx,qy,qz,qw） | 航点文件（空格4列或 CSV） |
| 输出话题 | `/mavros/setpoint_position/local` | `/planning/pos_cmd` → bridge → `/mavros/setpoint_raw/local` |
| 特点 | 无避障，录什么飞什么 | 有规划/避障，需 FSM |

**实际控制中心是 `task_run_scripts`，不是 SUPER 仓库**：所有生效配置在 `~/catkin_ws/src/task_run_scripts/config/`（real_flight.yaml 已适配 FAST-LIVO2 `cloud_topic: /cloud_registered`；waypoint.yaml 已修复坑1 `goal_pub_topic: /waypoint_goal`）。SUPER 仓库里那份是原始版（坑未修），别改错副本。

#### 关键文件

- `launch/real_flight.launch` — 支持 `config_path` / `waypoint_data_path` 命令行覆盖
- `config/real_flight.yaml` — FSM/规划器/ROG-Map 参数
- `config/waypoint.yaml` — 航点触发/切换/抽稀参数
- `config/super_to_px4_bridge.yaml` — **前馈参数在这**（use_velocity_ff/use_accel_ff/velocity_ff_gain/init_goal/geofence）
- `data/real_mission.txt` — 空格格式 `x y z switch_dis`
- `scripts/NX_code/User_define_commands.yaml` — 按钮1/2/3
- `scripts/NX_code/nx_command_server.py`（v2 只是包装器，逻辑全在 v1）
- `scripts/NX_code/My_Flight_Path.txt` — 录制轨迹

#### 地面验证流程（按序）

1. **输入源**：`start_fastlivo2.sh false` 拉起传感器+SLAM+MAVROS；验证 `/cloud_registered` ~10Hz、`/mavros/state` connected
2. **A 系统**：`curl :6004/check_replay_status`（IDLE）→ UDP 触发 RecordPathReplay → 预期 `RUNNING_WAITING`（不 armed 不会飞，安全）→ AbortReplay → IDLE
3. **B 系统**：启动 real_flight.launch → 日志 `Load N waypoints` + `WAYPOINT GOAL ENABLE` → 发 `/waypoint_goal` 测试目标 → 日志 `Get goal at [x,y,z]`（z 不被覆盖=坑1已修）
4. **录轨迹**：UDP RecordPathStart → 走轨迹 → RecordPathStop → 检查行数 ≈ 秒×40+1
5. **抽稀验证**：waypoint.yaml 改 `waypoint_downsample_dist` → 重启 → `Load N waypoints` 点数变化（0.1m 把 1038 点抽到 83）
6. **RViz 看轨迹**：确认 `/fsm_node/rog_map/occ` 有 Hz（无数据=白屏根因），再拨杆触发

#### 按钮逻辑（2026-08-10 最终版）

- **按钮1** = 启动 SUPER（默认航点 real_mission.txt）
- **按钮2** = 安全关闭：先 `set_mode POSCTL`（**失败则 && 短路不杀进程**，用户选定保持）→ SIGINT roslaunch → 兜底杀 fsm_node/bridge/waypoint_mission/map_to_world/top_down.rviz
- **按钮3** = 启动 SUPER 复飞（`waypoint_data_path:=My_Flight_Path.txt`）+ **手动拨杆触发**（start_trigger_type: 1）
- 改完即生效（nx_command_server 每次点按钮重读 yaml，无需重启服务）

#### 路径A：外部全局路径引导（2026-08 新增，源码已改）

**目标**：飞机除非必须避障绕行，否则一直沿录制轨迹（全局路径）飞。SUPER 源码注入实现，非纯配置。

**架构**：`guidepoints_new.txt` → `global_path_publisher.py`（新节点，发布 `/global_path` + `/global_path_marker`）→ FSM 订阅 → `planner_ptr_->setGlobalPath()` → `generateExpTraj` 里 guide_path = 全局路径全程段（替代 A* 搜索）→ 走廊沿轨迹生成 → MINCO 优化。

**改动文件**（SUPER 仓库 dev/4uav 分支，全有 .bak_globalpath 备份）：
- `include/ros_interface/ros1/fsm_ros1.hpp`：订阅 `/global_path` + `globalPathCallback`（转 vec_Vec3f 存 planner）
- `include/super_core/super_planner.h`：成员 `global_path_` / `global_skip_this_round_` + `setGlobalPath/clearGlobalPath`
- `src/super_core/super_planner.cpp`：generateExpTraj 注入块（滚动取段→guide_path）+ 走廊失败 fallback
- `config/real_flight.yaml`：`global_path/roll_dist` 参数

**核心注入逻辑**（generateExpTraj，`路径A` 注释段）：
1. 找当前位置在全局路径上的最近索引（全路径线性扫描）
2. 取从最近点到终点的全程段（非 5m 滚动！），最后点离 goal 远则补 goal → 使 Second Part 走 "NO NEED" 分支（不再 A* 续搜）
3. guide_stamp 按 max_vel 均匀时间分配
4. 走廊失败（`SearchPolytopeOnPath` 返回 false）→ fallback：`global_skip_this_round_=true` 本轮跳过全局路径 → 用 `PathSearch` A* 绕行 → 每轮 replan 末尾重置 skip 标志 → 障碍消失后自动恢复沿轨迹

**日志判读**：`Using FULL GLOBAL PATH, idx=N total=M pts` = 沿轨迹正常；`Global path blocked by obstacle, fallback to A* replan` = 绕行中；`A* fallback path corridor OK` = 绕行成功。

**global_path_publisher.py 关键坑**：**全局路径不能抽稀！** 默认 `min_pt_dist=0.0`（0=全发 705 点）。若设 0.3 会把 705 点抽到 7 点（航迹是稀疏大间距），轨迹线只剩几段。抽稀只用于 waypoint_mission 航点，不用于全局路径。

#### 触发与飞行流程（bridge 状态机）

切 OFFBOARD 后 **bridge 自动起飞到 init_goal（1.2m）悬停**（Phase4：|误差|<0.1m 或 30s 超时），**CH10 只控制 waypoint_mission 何时发航点**，两套独立。CH10 是**下降沿触发**（last>1500 && now<1500，且 CH2<1200）——一直拨低位不会触发，必须先拨高再拉低。

#### 坑位清单（全部本会话实测）

1. **话题不匹配**：SUPER 仓库版 waypoint.yaml `goal_pub_topic=/move_base_simple/goal` 会走 FSM click 通道，航点高度被 `click_height` 覆盖；task_run_scripts 版已改 `/waypoint_goal` 修复
2. `/planning/state` 话题**不存在**，WAIT_GOAL 只能看日志
3. 前馈参数在 bridge yaml 且默认全开（vel+accel, gain 0.7）
4. `geofence_en` 默认 false
5. 空格航点文件第4列=**切换距离**（米），不是 yaw；CSV 无第4列则用 yaml `switch_dis`（3.0）
6. **多 launch 同名日志陷阱**：grep 旧 `real_flight.log` 看到的是旧进程输出；判进程用 `ps -eo lstart` + `rosparam get /waypoint_mission/`（看 data_path 指向）
7. **按钮2 原版 `pkill -f 'rviz'` 模糊匹配误杀推流(rviz_full_stream2.py)和 SLAM(命令行含 rviz:=false)**——已改精确 `pkill -f 'top_down.rviz'`
8. 录制终止文件尾部 **NUL 损坏**（RecordPathStop 杀进程打断写入）→ python 截断到首个 `\x00` 再验证
9. 同名节点顶替（`new node registered with same name`）→ 按钮1/3 勿同时按
10. RViz 白屏先查 `/fsm_node/rog_map/occ` 有无数据，不是配置问题
11. **RViz Path 显示项不工作**：`rviz/Path` + `Topic: /global_path` 配置正确也不订阅（rostopic info 的 Subscribers 里无 RViz）。**改用 `rviz/MarkerArray` 显示**（发布 `/global_path_marker`，LINE_STRIP 绿线）——MarkerArray 与 Waypoints 同机制，已验证可靠。**改 top_down.rviz 必须重启 SUPER（按钮2→按钮3）让 RViz 重新加载配置**，否则显示项不生效
12. **top_down.rviz 是 RViz 压缩单行格式**（一行塞满所有字段），直接 python 字符串插入多行 YAML 会破坏结构 → 显示项全部失效。**正确改法：`yaml.safe_load` 读 → 列表 append 显示项 → `yaml.dump` 写回**（保持同格式）。误改后恢复：`top_down.rviz.bak_oldformat` 备份
13. **按钮3 command 的 `& &&` 写法卡死**：`nohup cmd & && sleep 1 && nohup cmd2 &` 会让 nx_command_server 超时（timed out），什么都不启动。**修复：后台命令包子括号 `(nohup cmd &) && sleep 1 && (nohup cmd2 &)`**——`&` 只作用于子 shell 内部，外部命令立即成功返回
14. **raycasting.enable: false = 动态障碍永不清除**：ROG-Map 的 p_miss（射线穿过清除）随 enable 关闭，人走过留下的障碍块只增不减 → 地图越积越脏 → `GeneratePolytopeFromLine failed`（走廊生成失败）→ 规划无限重试。`real_flight.yaml` 里 `raycasting.enable` 和 `esdf.enable` 当前都 false（概率地图只有 p_hit 命中更新）。需动态避障则开 raycasting（p_miss 恢复清除），注意可能误清静态障碍
15. **GeneratePolytopeFromLine failed 无限重试**：`GenerateExpTrajectory SUCCESS`（A* 找到路径）但走廊生成失败 → `generateBackupTrajectory FAILED` → `PlanFromRest failed, try replan` 循环。根因=轨迹附近地图障碍/未知（见坑14），或起飞高度≠轨迹起点高度触发 FSM `getNearestInfCellNot` 吸附（把起点从 0.55m 吸到 0.975m）
16. **分支切换被未提交修改/未跟踪文件阻挡**：避障分支有未提交修改（MID360_Fly.yaml 等）时 `git checkout` 失败。流程：**先备份 `cp` 到 `~/obstacle_changes_backup/` → `git stash push -m` → `git checkout -f` 强切 → 用 `git stash apply stash@{N}` 恢复**。未跟踪文件（data_guidepoints/）用 `git checkout stash@{0}^3 -- <path>` 从 stash 第三父树恢复
17. **同名节点顶替会让新节点崩溃**：`global_path_publisher` 重复启动时老进程和新进程抢名字（`new node registered with same name`），旧的可能带崩新的。重启前先精确清理：`for pid in $(pgrep -f "python3.*global_path_pub"); do kill $pid; done`（注意 pkill -f 会匹配到脚本自身 → 自杀 exit 128）
18. **⭐ FSM 实际加载的 real_flight.yaml 是 `$(find task_run_scripts)/config/real_flight.yaml`，SUPER 仓库那份不生效**（2026-08-11 实机确认）：改 raycasting/global_path 参数必须改 task_run_scripts 那份，否则白改。排查：`ps aux | grep fsm_node | tr ' ' '
' | grep yaml` 或 `rosparam get /fsm_node/config_path` 确认实际加载路径
19. **纯全局路径模式（use_waypoint:=false）消除乱转**（2026-08-11 实飞）：waypoint_mission 逐点发 goal + 全局路径 guide_path 同时跑 → FSM 在"追点"与"沿轨迹"间切换 → 飞机乱转 + `close to goal` + `OptimizationExpTrajInPolytopes failed` + `SUPER command timeout` → POS_HOLD。修复：real_flight.launch 已有 `use_waypoint` arg → 按钮3 加 `use_waypoint:=false`；global_path_publisher.py 补发**终点**到 `/waypoint_goal`（PoseStamped latch）→ goal=终点(1个)+guide_path=完整轨迹。验证：`grep 'Pub goal' real_flight.log` 只出现一次

#### odin 传感器适配与地图/动态障碍（2026-08-14 实机）

传感器从 MID360 换 odin 后，只改传感器层（点云话题/位姿桥/启动链），导航核心不动。三件套：
- `real_flight.yaml` `cloud_topic: /odin1/cloud_slam`（odin 自带 SLAM，frame=odom 可直接用）
- `odom_to_mavros.py` 桥 `/odin1/odometry_highfreq` → `/mavros/vision_pose/pose`（**必须限速 20Hz + pid 单实例锁**，见坑21）
- 启动链：Radar_Launcher.yaml `active_profile: "odin"`，cleanup 含 host_sdk_sample/odom_to_mavros

#### 坑位清单（续，2026-08-13~14 实测）

20. **vision_pose 超频 → not ready**：odin odometry 400Hz 全转发 → vision_pose 500-1000Hz → PX4 EKF 拒收/判视觉不可信 → NOT READY。**限速 ~20Hz（0.05s/帧）后 EKF 融合正常**。PX4 EKF 处理上限 ~100Hz；1m/s 飞行 20Hz 足够（EKF 内部用 IMU 250Hz 内插，vision 只是"对表"）。多实例锁用 **pid 文件锁**（grep 进程数会把 bash 包装命令行也计入，要精确 grep `python3.*odom_to_mavros`）
21. **动态障碍残留的机制（源码级）**：ROG-Map raycasting = 从飞机位置向**每个点云点**发光线、沿途标 free → **只有"点云有点的方向"被清除，空旷方向（人已走）无光线 → 旧障碍残留**。`cloud_slam`（累积配准点云）人走点留 → 残留最重；`cloud_raw`（当前帧）人走即消失，但 frame=lidar 需桥接变换
22. **cloud_raw → odom 桥接三坑**（scripts/cloud_raw_lidar_to_odom.py）：① TF 用 `msg.header.stamp` 会失败（odin 设备时间戳≠主机时间，差数百秒 → extrapolation into the past）→ **必须 `rospy.Time(0)` 取最新 TF**；② 49152 点全量给 FSM → CPU 116% → `Replan over time` → 必须降采样（numpy 体素 5cm → ~13000 点）；③ 地面点（z<0.1，占 19%）被扫成障碍 → 桥接里过滤 z<0.1
23. **virtual_ground/ceil 量化 bug**：prob_map.cpp `posToGlobalIndex`（ORIGIN_AT_CENTER: `int(res_inv*pos + sign*0.5)`）+ 地图边界钳制 → **yaml 配置值 ≠ 代码实际值**。实测：-0.9 → +0.15（地面 z≈0 被拒 → `Odom below virtual ground` → 无地图）；-3.0 → -1.95；ceil 3 → 1.95。**排查必核对日志 `[ProbMap] virtual_ground_height: X` 实际值，别信 yaml**
24. **fix_map_origin z 偏移 = 全屏障碍**：`fix_map_origin: [0,0,1.5]` 抬高地图坐标系 1.5m → 点云 z 相对地图原点减 1.5 → 地面/低处点全变"空中障碍"散布全屏。**改 0 会雪上加霜**。地图坐标系必须与点云坐标系约定一致；全屏障碍排查顺序：①核对 FSM 是否重启加载新配置 ②核对 [ProbMap] 实际值 ③二分 cloud_topic ④调参 2-3 轮无效直接回退 `.bak_odintopic` 锚点（见 references/odin-map-raycasting-debug.md §10）
25. **改 yaml 后 FSM 必须重启才生效**：FSM 启动时加载 rosparam。改配置不重启 = 白改。排查"改了没生效"：`ps -eo lstart -p $(pgrep fsm_node)`（启动时间）vs yaml 修改时间 + `grep "Load param rog_map/"` 日志实际值
26. **二分测试法隔离问题**：地图异常/全屏障碍 → 切 `cloud_topic`（cloud_slam vs cloud_raw_odom）二分"地图参数" vs "点云链路"；再核对 FSM 重启与加载值。本会话靠它定位到 fix_map_origin
27. **绕行"暂停原地"根因**：A* 绕行路径贪心最短 → 贴障碍 0.04-0.2m → 走廊凸分解要求离障碍≥安全距离 → `GeneratePolytopeFromLine failed` → MINCO `Opt failed`（`The problem is not feasible, min dis to obstacle: 0.04`）→ 无轨迹 → bridge 0.5s 超时 → position hold。解法：`inflation_step: 3`（0.6m 膨胀）+ fallback 大盒子（方案A `corridor_small_` 标志：循迹小立方体 0.2m 硬约束，fallback 置 false 用大盒子 0.39m）
28. **绕行回正俯冲修复（双保险）**：绕行→回正轨迹切换 → MINCO 加速度突变 → PX4 俯冲（z 0.68→1.02→0.84）。①控制层：bridge `acc_ff_smooth: 0.5` 加速度前馈一阶低通（30%新+70%旧）②轨迹层：`penna_jerk` 主轨迹 -7.5e5（原-5e5，平滑与响应折中）、backup -5e5。**俯冲=加速度突变，轨迹层 jerk + 控制层低通双管齐下**
29. **omni 全向光线 = 主动清除残留（已重新启用且有效）**：解决坑21"残留要飞机扫过才清"——在 raycastProcess 点云光线后加**全向稀疏光线**（config.hpp 加 `raycast_omni_*` 参数 + prob_map.cpp 循环，水平 step_h°×垂直 step_v°×fov_v°×range m，遇"当前帧命中格"停止不清真障碍）。**实现坑：`hit_cnt` 是 `std::vector<uint16_t>` 不是 map，用索引+越界检查，`.find()` 编译报错**。配置基线正常（fix_map_origin 1.5 + virtual_ground 修好）后重启用 `omni_en: true` → 残留大幅减少（occ 7649→2136 格）。**omni 逻辑本身安全（只清 free），之前回退是 fix_map_origin 误改 0 的连带误判**
30. **雷达贴地几何遮挡 = 地面测试残留主因**：用户实测"飞机拿起来残留就消失"——雷达离地 0.1-0.3m 时激光从低处扫，障碍背后无光线 → 残留；雷达升到 1m+ 俯视全览 → 残留消失。**飞行作业（0.6m+）残留本来就少，地面测试残留严重是正常现象**。地面测试垫高雷达更接近飞行状态，别为地面测试的残留过度调参
31. **残留剩余小部分的三来源与对策**：①光线间隔空隙（10°×10° 网格在 8m 处间距 ~1.4m，细障碍漏清）→ 加密 `omni_step_h: 10→5`、`omni_step_v: 10→7.5`（**先查 FSM CPU，有余量才加密**；实测 43% 可加）②概率衰减慢 ③膨胀层残留 → `inflation_step` 缩小（但影响绕行空间，最后动）。**⚠️ p_miss 是"清除目标概率"不是"清除强度"——必须 < p_free(0.4999)！** 贝叶斯更新把光线穿过格子的概率拉向 p_miss；p_miss=0.6 > 0.4999 时格子永远降不到自由阈值 → **清除失效 → 全屏障碍**（2026-08-14 实机翻车）。p_miss 保持 0.49（刚好低于 0.4999）。要加速残留清除用 omni 加密/加大 range，**别动 p_miss**
32. **停车避让 AVOID_WAIT 状态机（2026-08-14 实机验证，根治"卡死"）**：动态障碍 → 轨迹生成失败 → 原来靠 bridge 0.5s 超时被动悬停 + 高频重试空转 = 永久悬停/卡死。解法：**新增 FSM 状态 AVOID_WAIT**，三文件改动：①`include/fsm/fsm.h` 枚举加 `AVOID_WAIT`（MACHINE_STATE_STR 同步加，顺序必须一致）+ 成员 `avoid_wait_t_` + 虚函数 `publishHoldPose()` ②`src/super_core/fsm.cpp` GENERATE_TRAJ 失败分支 → `publishHoldPose()` + `ChangeState(AVOID_WAIT)`；AVOID_WAIT 每帧发悬停 + `avoid_wait_t_` 累积 **0.3s 才重试**一次 PlanFromRest → 成功 publishPolyTraj + ChangeState(FOLLOW_TRAJ) ③`include/ros_interface/ros1/fsm_ros1.hpp` 实现 publishHoldPose：**当前位置+零速度 PositionCommand → `cmd_pub`（无下划线！`cmd_pub_` 是未初始化变量，编译过但发布会崩）** → `/planning/pos_cmd`（bridge 订阅的正是它，PositionCommand 类型；`mpc_cmd_pub_` 才是 PolynomialTrajectory）。**逻辑**：主动悬停（不靠超时）+ 限频重试（防高频空转拖垮 CPU）+ 成功自动恢复。**验证日志**：`switch to AVOID_WAIT (park & wait)` → `AVOID_WAIT: path recovered, resume flight`（实测恢复仅 0.06s）。编译后确认 roslaunch 用 devel/lib 路径（无 install 版），`strings fsm_node | grep -c AVOID_WAIT` 确认新版二进制
33. **⭐ "多次穿行卡死"真凶 = fsm_node 崩溃（2026-08-14 晚查明，已根治）**：人多次来回穿行 → A* 绕行路径出现**相邻点距离 > seed_line_max_length_×1.5(1.8m) 的长段** → `corridor_generator.cpp:113` `throw std::runtime_error("seed line too long")` **异常未捕获 → terminate → fsm_node 进程死** → 无 pos_cmd → bridge 超时悬停 = 视觉"卡死"。**崩溃堆栈特征**：日志尾部出现 `Source ".../corridor_generator.cpp", line 113` + `Aborted (Signal sent by tkill...)` + `#0..#12` 栈帧。**排查"卡死"必查 fsm_node 进程是否还活着**（`ps aux | grep fsm_node`，崩溃后进程消失）。**修复：长段自动插值切分**（corridor_generator.cpp SearchPolytopeOnPath 内，相邻点 > seed_line_max_length_ 时 `ceil(seg/max)` 等分插入 seed_lines，每段 ≤ max；同时删掉 throw，失败 return false）。修复后实测：`seed line too long` = 0、AVOID_WAIT 11 次全恢复、崩溃根治。**终点判定 0.1m 已用注释法排除**（注释后仍卡死），并已恢复原值。排查顺序：先看进程死没死（崩溃）→ 再看日志尾部有无栈帧 → 再看状态机循环
34. **ESDF 不建议开（esdf_en: false 保持）**：ESDF 从占用图派生 → 残留同源不解决；3D ESDF 每帧全图更新 CPU 开销大（当前 45% 会推到 80%+）；当前膨胀层（inflation 0.4m）+ A* 绕行已够用。ESDF 只对"绕行轨迹距离梯度精度"有增益，是锦上添花非瓶颈
35. **FAPP 动态避障算法评估（arXiv 2312.08743，港科大 Lu Peng 团队 2023）**：Fast and Adaptive Perception and Planning for UAVs in Dynamic Cluttered Environments。四模块与融合价值：①点云分割（静态/动态区分，**价值最大**——动态物不进静态地图→残留根治，直击本会话坑21/29/30）②自适应运动估计（协方差自适应预测多动态物，前瞻避让）③高效轨迹优化（本项目 MINCO/FAST-Planner 系已强，价值低）④自适应重规划（无解兜底，与 AVOID_WAIT 同思路，已实现）。**落地路线：阶段1（性价比最高）= 动态点云分割，在 Python 桥接 `cloud_raw_lidar_to_odom.py` 里做差分（cloud_raw 当前帧 vs cloud_slam 静态累积：当前帧有+静态图没有 = 动态物 → 剔除不进地图），不碰 C++；阶段2 = 动态障碍线性预测（卡尔曼）；阶段3 = 完整 FAPP（现阶段不推荐）**。**2026-08-14 实测：纯时间一致性分割（体素计数 CONFIRM_N）在 odin dToF 点云上不可靠**——点云帧间闪烁/体素边界抖动（0.2m 跨界）→ 静态物计数到不了阈值 → 误分 dynamic（84% 散布）；CONFIRM_N 调大则真静态物也到不了；体素加大 0.4m 部分缓解但闪烁点仍误分。**结论：odin 点云不稳定，时间一致性分割方案暂缓；待全局地图/更稳点云后再做**
36. **⭐ 撞墙根因 = 未知被当自由 + 雷达范围小（2026-08-14 实机撞墙）**：odin 雷达有效探测仅 ~4.5m，墙在 5m+ 扫不到 = 地图"未知"。`frontend_in_known_free: false`（默认）时 A* 把未知当自由（`UNKNOWN_AS_FREE`，super_planner.cpp:1305/1330 flag 三元）→ 飞机直飞穿未知区 → 撞墙。**修复：`frontend_in_known_free: true`**（未知=障碍 `UNKNOWN_AS_OCCUPIED`）→ A* 只搜雷达确认自由区 → 前方未知(墙)无通路 → 停车（宁停勿撞）。**代价/副作用**：A* 在"已知/未知边界"反复尝试（点云抖动→边界抖动→搜索结果每帧不同）→ **轨迹乱跳**（SearchPolytope failed 爆发）；全局地图接入后边界稳定、副作用消失。**当前路线：用户决定加全局地图约束飞行范围（把墙/边界事先标进地图），frontend=true 配合使用；无全局地图时保持 false**
37. **绕行撞障碍物 = 膨胀不足（区别于撞墙）**：已知障碍但轨迹贴太近（`min dis to obstacle: 0.1` 日志）→ 桨(0.23m)扫到。①`robot_r`（yaml，当前 0.25→**0.35** = 桨0.23+余量）——CIRI 生成走廊时要求 seed line 离障碍 ≥ robot_r，超近则 `return FAILED`（ciri.cpp:102 `dis < robot_r_ - 1e-2`）——**这是硬约束入口**；②`penna_pos`（MINCO 位置惩罚 5e6→**1e7**）——MINCO 轨迹只受走廊 vPolytopes 约束、无独立碰撞检查，penna_pos 是软约束可能穿走廊，加大惩罚守走廊。**飞机半径必须显式纳入 robot_r**（系统不会自动知道桨多大）；**膨胀(A* 用) 与 robot_r(CIRI 用) 是两套独立安全距离，都要调**。⚠️ inflation_step 加大（0.6m 膨胀）在室内小空间会**堵死绕行**（走廊扩不出→fallback 全失败→乱转），与防撞矛盾：空间够用大膨胀，空间窄用 robot_r+penna_pos 保底线
38. **航迹末端"1 点残局"乱转（idx=N total=1）**：飞机飞到航迹最后一点（`Using FULL GLOBAL PATH, idx=50 total=1`）但距终点 > 终点判定阈值(0.1m) → guide_path 只剩 [终点] 1 点 → 每轮 PlanFromRest 生成"原地微调轨迹" + GeneratePolytopeFromLine failed（1 点无走廊）→ 反复循环 = 视觉乱转。**修复：终点判定 0.1→0.3m**（super_planner.cpp:340，距 goal<0.3m 即悬停，不留 1 点残局）。乱转排查：`grep 'Using FULL GLOBAL PATH' | tail` 看 idx/total 是否 total=1；与坑33 崩溃、坑36 边界乱跳区分（idx 正常=total 大但 fallback 失败循环）
39. **动态避障调参铁律（本会话多次翻车总结）**：①**一次只改一个参数+每次验证**——连锁改动（virtual_ground/map_size/ceil/fix_map_origin/cloud_topic 叠加）导致全屏障碍排查数小时，最终靠回退 `.bak_odintopic` 锚点恢复；②**备份必须保留"已知正常"版本**（.bak_odintopic 恢复法最可靠）；③**参数语义先确认再改**（p_miss 翻车：以为是清除强度调大，实际是目标概率必须<p_free）；④**改 yaml 必须重启 FSM 才生效**；⑤静态测试（雷达贴地）的残留/遮挡与飞行不同，别为地面测试过度调参
40. **起飞左右摇摆 → 禁用速度/加速度前馈排查**（2026-08-14）：飞机起飞后左右摇摆，先排除规划参数（robot_r/penna_pos 只影响轨迹不控制姿态），嫌疑集中在 bridge 前馈——`super_to_px4_bridge.yaml` 的 `use_velocity_ff: true`（gain 0.7，注释明言"过大会引入抖动"）和 `use_accel_ff: true`（我们加的加速度低通 `acc_ff_smooth: 0.5` 在**代码硬编码**、只被 use_accel_ff 开关控制）→ **`use_velocity_ff/use_accel_ff: false` 即可禁用，纯 yaml 无需编译**（bridge 代码尊重开关，else 分支 `prev_acc_ff.setZero()`）。禁用后 setpoint 只剩位置（纯位置跟踪），更稳但抗风/跟踪精度略降（0.3m/s 慢速影响小）。**排查摇摆顺序**：先看 `/mavros/local_position/pose` 姿态是否真的在摆（orientation 变化<1e-4 则不是姿态问题）→ 再禁用前馈验证 → 还有摇摆则查 PX4 PID/视觉链路。恢复时先 acc 后 vel 逐个找平衡点

41. **⭐ "轨迹在地下/目标高度异常" = goal_z_min/goal_z_max 地面测试临时值残留（2026-08-19 实机，复飞轨迹入地）**：
   - **参数语义**：`real_flight.yaml` `fsm` 段 `goal_z_min/goal_z_max`（代码默认 0.6/0.825，config.hpp:49-50）是路径A加的**吸附后目标 z 钳制**——fsm.cpp:231 `getNearestInfCellNot` 吸附后执行 `if (gi_.goal_p.z()>z_max) gi_.goal_p.z()=z_max`。某次地面测试临时改成 `0.0/0.3` 忘改回 → 航点 z=0.75 的目标被压到 0.3（地图系），配合 `fix_map_origin: [0,0,1.5]`（地图原点抬高 1.5m，rog_map.cpp:36 `robot_state_.p=fix_map_origin`）→ 目标在 odom 系实际 -1.2m = **地下**，RViz 里轨迹贴地/入地
   - **判据（铁证）**：日志 `-- [MISSION] Pub goal to 2.5 0 0.75`（waypoint_mission 发布值）vs `-- [Fsm] Get goal at 2.325 0.375 0.300`（FSM 吸附+钳制后）z 不一致 = 钳制发生。**只发一次 /waypoint_goal 触发即可复现，无需拨杆**
   - **查参数来源**：`git show <commit>:config/real_flight.yaml | grep goal_z`——commit 402a86e 前是 0.6/0.825，402a86e 起改 0.0/0.3（"地面测试临时改"注释），d906e20 仍残留
   - **修复**：按实际航点高度设覆盖区间（按钮3 复飞 guidepoints/endpoint z=0.75、按钮1 real_mission.txt z=1.2 → 设 `goal_z_min: 0.5 / goal_z_max: 1.3`），**改完必须重启 SUPER**（坑25）。验证：重启后发 /waypoint_goal，`Get goal at` 的 z 应落在合理高度（实测 0.825 ✅）
   - **教训**：**带"临时改"注释的参数是事故源**——飞行前 `grep -n '临时' config/*.yaml` 检查所有临时值残留；git diff 未提交改动也要查（本案例 max_acc 0.6→0.2 未提交残留导致 MINCO TrajOpt failed 循环）

42. **⭐ "避障太近" = CIRI min dis < robot_r → 走廊失败 → 反复 AVOID_WAIT → 飞不到终点（2026-08-19 实机，复飞复盘）**：
   - **现象**：用户报"飞行中离障碍太近"。日志刷屏 `[CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: 0.186` → `GeneratePolytopeFromLine failed`（2000+ 次）→ `AVOID_WAIT`（80 次）→ 飞机停在半路（实测 (2.05, -1.12, ~0) vs 终点 (2.5, 0, 0.75)），**没到终点**
   - **机制**：ciri.cpp:102 `if (dis < robot_r_ - 1e-2) return FAILED`——CIRI 检查**每个障碍点到 seed line 的最短距离**，< robot_r(0.35) 就拒绝生成走廊 → 无轨迹 → AVOID_WAIT 停车等待 → 每轮重规划撞同一障碍 → 无限循环。**0.186m < 桨半径 0.23m = 真会擦桨，用户观感正确**
   - **⚠️ 距离判定关键：CIRI 检查的是原始点云/占用格距离，不是膨胀后的**——A* 用膨胀地图（inflation_step 2=0.4m）找路径，但 CIRI 量的是到真实障碍的 min dis。A* 路径离障碍 0.186m < 膨胀 0.4m = 该障碍**没被膨胀覆盖**（新出现/动态障碍，膨胀层未更新）或路径贴障碍走
   - **修复方向（分清两层安全距离）**：① **`inflation_step 2→3`（0.4→0.6m 膨胀）** = 让 A* 路径直接离障碍更远，是"避让更远"最直接手段（代价：室内窄空间堵死绕行，坑37/39）② **⚠️ 别动 `robot_r`**——它是 CIRI 的**拒绝阈值**，加大只会更容易 FAILED（方向反了！）③ corridor_bound_dis 0.6 已加宽保持 ④ 障碍若是动态（人），等走开自动恢复；若是地图残留，起飞前原地转一圈重扫
   - **判读**：`min dis to obstacle is only: X` 的 X 反复出现且相同 = 同一固定障碍反复挡路；X 与 robot_r 比较定级（<0.2 危险贴桨、<0.35 走廊必失败）

43. **⭐ 重启 SUPER 必须"停止/启动拆两个脚本"——pkill 自杀坑（2026-08-19 复现，exit 128）**：plink `-m` 脚本全文进入远程 `bash -c` argv，脚本内若先 `pkill -f 'global_path_publisher.py'` 后面又有 `nohup ... global_path_publisher.py ...` 启动命令 → **pkill 匹配到脚本自身 argv → kill -9 自杀 → 后半段（启动）不执行**，表现为"停了没起来"。**规矩**：①停止脚本只含 pkill（用正则字符类 `[g]lobal_path_publisher[.]py` 规避自匹配），启动脚本只含 nohup，**两个脚本分开传**；②或先 `ps aux | grep` 拿精确 PID 再 kill；③重启后必须验证：`grep 'Load param fsm/goal_z'` 日志新值 + 发一次 /waypoint_goal 看 `Get goal at` z 正常。**④停止脚本必须"纯净"（2026-08-19 服务重启再复现）**：除 pkill/pgrep 行外什么都别放——连 `cat <服务文件名>`、tail 日志等**含文件名字面量的命令**都会触发 pkill 自匹配自杀（停止脚本第 3 步 `cat nx_command_server2.py` 被第 1 步 `pgrep -f '[n]x_command_server2'` 匹配 → exit 128，服务只杀一半）。查看文件内容/确认状态拆到独立脚本，或用单独 plink 命令

44. **⭐ "明明右边有空间却不绕行，贴着障碍走" = frontend_in_known_free=true 把未知当障碍，A* 搜索空间被限死（2026-08-19 实机，用户观察"右边飞高一些有路能绕行"）**：
   - **因果链**：`frontend_in_known_free: true`（当前配置，防撞墙）→ super_planner.cpp:1305/1330 `UNKNOWN_AS_OCCUPIED` → A* 只搜"已知自由"区，**雷达没扫过的右侧/上方绕行空间=未知=被当墙** → A* 唯一能走的路 = 贴着已知障碍的狭窄通道 → CIRI min dis 0.186 < robot_r → 走廊失败 → AVOID_WAIT 反复卡死。**min dis 0.186 每次完全相同 = A* 确定性搜索永远给同一条贴障碍路径，从未考虑过右侧**
   - **与坑36 区分**：坑36 是 frontend=false 撞墙（未知当自由）；本坑是 frontend=true 过度保守（未知当墙，明明有路不让走）。**两个方向都危险，frontend 没有"刚好"的全局解，必须配全局地图/先扫后飞**
   - **方案A（不改成配置，先扫后飞）**：让雷达在飞行前扫过右侧 → 未知变已知 → frontend=true 时 A* 立即可绕。落地方式：①地面手持/推车扫（未解锁安全）②**起飞后扫描（用户首选，见下）**③起飞后手动转一圈再拨 CH10（零改动验证法）
   - **验证判读**：扫完看 RViz `/fsm_node/rog_map/unk`（橙色未知）在绕行方向是否消失变绿；绕行成功日志 = `Global path blocked by obstacle, fallback to A* replan` + `A* fallback path corridor OK`
   - **方案B**：`frontend_in_known_free: false` 允许穿未知区绕行，但未知区当自由可能撞墙（坑36）——只能当临时测试手段

45. **⭐ bridge 的 pos_cmd 注入点：任何节点发 /planning/pos_cmd 都被 bridge 接受并转发（2026-08-19 源码确认，"起飞后先转向扫描再飞行"方案的实现基础）**：
   - **bridge 机制**：`superCmdCallback`（super_to_px4_bridge.cpp:244）**无条件** `last_super_cmd=*msg; super_cmd_active=true`——不校验发布者是谁，FSM 发的和外部节点发的没区别。`super_active = super_cmd_active && (now-last_super_cmd_time)<timeout_threshold(0.5s)` → 外部节点持续发 pos_cmd 就能接管飞机
   - **FSM 在 WAIT_GOAL 不发 pos_cmd**（fsm.cpp:133 `case WAIT_GOAL: if(!gi_.new_goal) return;`）→ CH10 触发前飞机由 bridge 自己发 init_goal 悬停（Phase4 起飞后 Phase5 主循环）→ **Phase4→Phase5 之间、CH10 触发前是天然的空窗期**，可插扫描动作
   - **起飞后扫描方案（用户需求"起飞后先转向扫描再飞行"）**：
     - **方案①（C++ 自动化，2026-08-19 已落地 ±90° 版）**：bridge（`SUPER/super_planner/Apps/super_to_px4_bridge.cpp`）Phase 4 结束处（`Drone reached target altitude` 日志后、`Starting main control loop` 前）插入 **Phase 4.5 扫描循环**——位置保持 init_goal 只转 yaw：`std::vector<double> scan_offsets = {+angle, -angle, 0.0}`（±90° 往返回中，比 360° 少一半时间更安全；用户点名要 ±90° 而非整圈），每方向用 `tf::Quaternion setRPY(0,0,yaw)` 构造 PoseStamped 发 `setpoint_position_pub`（不是 pos_cmd！Phase4.5 还没进主循环，直接发 setpoint_position 最稳），保持 `scan_hold_time` 秒让雷达充分扫过。**⚠️ 必须渐进旋转防甩头（2026-08-19 用户实测第一版"旋转太快吓人"）**：直接发目标四元数（setRPY 到 +90°）会让 PX4 以最大速率瞬间甩头——**每帧只能发 `cur_yaw += clamp(target-cur, ±scan_yaw_rate*dt)` 的渐进逼近值**（dt 限 0.02~0.1s 防异常），到位判定 |err|<0.02 rad 后再保持 scan_hold_time。**最终转速参数（2026-08-19 用户选定）**：`scan_yaw_rate: 0.0873`（=**5°/s**，90° 转 18s；0.35≈20°/s 用户仍嫌快"吓人"）；5°/s 时 ±90° 全程约 **74s**（右90°18s+停1s+左180°36s+停1s+回90°18s），用户接受。**同批修复**：扫描序列 0→+90°→-90°→回中三段（含回中，不用另写回中块）；yaml 新开关（`task_run_scripts/config/super_to_px4_bridge.yaml`）：`scan_after_takeoff_en: true`（**false=完全关闭扫描，起飞后直接进主循环，但会回到坑44 卡死——与 frontend=true 配套使用**）/ `scan_angle_deg: 90` / `scan_hold_time: 1.0` / `scan_yaw_rate: 0.0873`。**编译坑**：①用到 `std::vector` 必须显式 `#include <vector>`（该文件原本没有，靠其他头间接包含不保险）②未使用变量（如 `scan_dt`）删掉否则编译警告 ③**单目标快速编译：`cd ~/catkin_ws/build && make super_to_px4_bridge -j4`（1-2 分钟）**，全量 `catkin_make -j4 -DROS_EDITION=ROS1` 要 10-20 分钟；nohup 后台 + 查 `/tmp/build_fast.log` 的 BUILD_EXIT/error: ④改前备份 `.bak_scan_<时间戳>`（本会话：super_to_px4_bridge.cpp.bak_scan_20260819_140738、.bak_fastscan_20260819_142553、yaml .bak_rate_20260819_143111）⑤编译完成需重启 SUPER 验证日志 `Scan phase: rotating +/- 90.0 deg` → `Scan phase done`；验证新二进制含代码：`strings devel/lib/super_planner/super_to_px4_bridge | grep 'Scan phase'` ⑥**yaml 改转速只需重启 SUPER 无需重编译**（`sed -i 's/scan_yaw_rate: 0.0873/scan_yaw_rate: 0.2/'` + 按钮2→按钮3）
     - **方案②（零改动验证）**：按按钮3 → bridge 自动起飞悬停 → **先别拨 CH10**，遥控器打 yaw 杆转 360°（雷达扫过右侧）→ 回正 → 再拨 CH10 触发。5 分钟验证"扫完就能绕"，再上方案①自动化
   - **⚠️ 扫描必须是 yaw 旋转（位置不动）**：位置动了会偏离轨迹起点，触发后 guide_path 起点吸附错位（坑15/34）；yaw 旋转对 odin cloud_slam（frame=odom）无影响，累积点云自动补充右侧
   - **判别"谁在发 pos_cmd"**：`rostopic info /planning/pos_cmd` 看 Publishers——正常只有 fsm_node；多发布者=门禁被绕过/外部注入

46. **⭐ ROG-Map 已知/未知"记忆"机制（先扫后飞方案为什么有效）+ RViz 障碍块"先出现后消失"判读（2026-08-19 用户两问）**：
   - **扫描的已知/未知在飞行中会被记住，三个原因**：① `map_sliding_en` 关闭（rog_map.cpp `Map sliding disabled, set origin to [0,0,1.5]`）→ 地图固定在起飞点坐标系，飞机飞 2.5m 轨迹地图不滑动、旧区不丢弃；② ROG-Map 概率**无时间遗忘机制**（不像 OctoMap 衰减）——被扫过标记自由/占用后，无新射线就不会变；③ 飞行中持续扫描只更新被扫到的格子，不把已扫区打回未知
   - **会"忘"的三种情况**：重启 FSM（地图重建，但 odin cloud_slam 累积点云还在 → 已知区可恢复）；重启 odin SLAM/host_sdk_sample（点云重新累积 → 全忘，从头扫）；超出地图边界 30×30×6m（边界外丢弃）
   - **三态区分**（给用户解释用）：未知=概率 0.5~0.85 灰（没扫过）；已知自由=<0.50 绿（扫过确认空）；已知障碍=>0.85 红/黑（扫过确认有物）。**障碍块消失 ≠ 变回未知**——消失后是"已知自由"，记忆还在
   - **RViz 看到"扫过的空间先出现障碍块、后面消失" = 正常概率自我修正，不是故障**：p_hit=0.9（>p_occ 0.85）→ 第一帧命中显示障碍块；飞机转头/移动后光线从新角度穿过该格 → p_miss=0.49（<p_free 0.4999）→ 概率被洗回自由 → 障碍块消失。**类比**：暗房间闪光灯转圈，第一下照到反光以为是东西，换角度发现是空的
   - **判读：哪些消失哪些保留**——真实静态障碍（墙/箱）每个角度都打中、p_hit 反复拉高 → **保留**；噪点/点云边缘/动态物（人走过）→ 被后续光线 p_miss 洗掉 → **消失**。只有"真实静态障碍也反复横跳"才是异常（概率参数或点云抖动，查 p_hit/p_miss 相对 p_occ/p_free 的位置，坑31/53）
   - **验证"扫完记住"**：扫描完成后拨杆前看 RViz `/fsm_node/rog_map/unk` 在绕行方向是否消失变绿；拨杆后到障碍前日志 `fallback to A* replan` + `corridor OK` = 用上了记忆的已知区

47. **⭐ omni 全向光线"当前帧命中"保护 → 转头扫描时误清真墙（2026-08-19 用户实测"墙面消失了"）**：
   - **现象**：开启起飞后扫描（Phase 4.5 转头 ±90°）后，RViz 里**真实静态墙面**先出现障碍块、转头后消失，反复横跳——这不是坑46 的正常概率修正（噪点消失正常，**真墙消失是 bug**）
   - **根因（源码级）**：prob_map.cpp:821 omni 光线停止条件 = `raycast_data_.hit_cnt[hash_id] > 0`——**只看"当前帧"命中**。odin 雷达视场有限（config/calib.yaml `maxIncidentAngle: 120`，非 360°）→ 转头朝墙时命中保留；**转头背对墙时该帧没扫到（hit_cnt=0）→ omni 光线直接穿过墙格 → p_miss(0.49) → 真墙概率被洗到自由 → 消失**。之前不转头时墙总在视野内，问题不暴露；±90° 扫描让墙周期性离开视野 → 每次离开都被误清
   - **与坑46 的关系**：坑46 的"先障碍后消失=正常"仅适用于**噪点/动态物**；静态障碍消失 = omni 保护失效。判读：消失的块是不是真实存在的墙/箱（多次确认过的）——是则中招
   - **修复方向（用户问"会不会识别不到动态障碍"，权衡如实）**：
     - **方案A（简单）**：omni 停止条件加 `getGridType(cur_ray_id_g)==OCCUPIED`（地图概率>p_occ 也不清）→ 真墙保留 ✅；**代价：人走开后的残留格概率还是 0.90(>p_occ) → omni 不再清 → 残留清除变慢**（但飞机移动中 raycasting 点云光线扫过仍会清，不是永远残留）
     - **方案B（两全，推荐）**：raycast_data_ 加"上次命中时间戳"数组，停止条件 = 当前帧命中 或（概率>p_occ 且 最近 T=30s 内命中过）→ 转头 18s 短暂离开视野的墙保留（30s 内命中过）+ 人走开残留超 30s 没命中 → omni 照常清。需改 prob_map.cpp + 重编译
   - **用户最终决策（2026-08-19）**：**关闭扫描**——`super_to_px4_bridge.yaml` `scan_after_takeoff_en: false`（false=完全关闭，起飞后直接进主循环；配合 frontend=true 会回到坑44 卡死，当前接受）。**关闭后扫描功能代码保留在仓库**（feat/guide-path-c 分支 + task_run_scripts feat/4uav-guide），后续要启用需先解决 omni 误清（方案B）再开
   - **本次提交记录（2026-08-19 三仓库）**：SUPER `2fbadee`（bridge Phase4.5 扫描，80 行插入）、task_run_scripts `0ae19e3`（goal_z 0.6/1.3 + 扫描 yaml 参数）、NX_code `f1871ec`（odin 适配 Radar_Launcher + odom_to_mavros 限速）——**提交前清 .bak**（本会话 5 个调试备份全删，坑55）；push 后 `git status -sb` 应显示 `## feat/...origin/feat/...` 无 ahead

48. **⭐ 三仓库分支拓扑 + 合并 master 到工作分支的完整流程（2026-08-19 实机，NX_code 合并 17 个提交）**：
   - **拓扑（先分清再动手，别默认 master）**：task_run_scripts 默认分支 = **origin/main**（无 origin/master，`git log origin/master` 空是正常的）；SUPER 默认分支 = **origin/dev/4uav**（`origin/HEAD -> origin/dev/4uav`）；NX_code 默认分支 = **origin/master**。工作分支：task_run_scripts + NX_code 在 `feat/4uav-guide`，SUPER 在 `feat/guide-path-c`。**查"远端最新"先 `git branch -r` 看默认分支名**
   - **合并前准备**：`git fetch origin` → `git rev-list --left-right --count origin/<默认分支>...HEAD` 看差异规模（NX_code 当时 master 独有 17、分支独有 1）→ **备份当前工作区**（`cp` 关键文件到 `~/merge_backup_<时间戳>/`，本会话备份 odom_to_mavros.py/Radar_Launcher.yaml/nx_command_server.py）→ 确认 `git status --short` 干净
   - **冲突预测法**：`git merge-base origin/<默认分支> HEAD` 拿共同祖先 → `git diff --stat <base> origin/<默认>` vs `git diff --stat <base> HEAD` → `comm -12` 取两边都改的文件 = 冲突候选（NX_code 只有 Radar_Launcher.yaml + odom_to_mavros.py 冲突，其余 12 个 master 独有文件自动合并）
   - **执行与安全保证（用户极在意"不污染 master / 分支保持独立"）**：`git merge origin/<默认分支>` 是**本地操作**，不 push 就不碰远端 master；冲突解决后 `git status -sb` 应显示 `## feat/4uav-guide...origin/feat/4uav-guide [ahead N]`（分支身份未变）+ `git rev-parse origin/master` 不变（master 没被污染）；可回滚：未提交前 `git merge --abort`，提交后 `git reset --hard`
   - **冲突解决策略（关键决策）**：**服务函数类文件取 master 版**（nx_command_server.py 等，配套新平板/遥控器，必须跟上）；**odin 保命修复必须保留**——odom_to_mavros.py 的单实例锁 + 20Hz 限速是实机验证过的（坑20/42），master 侧只是删旧注释，**两边叠加手动构造**（保留锁代码 + 丢弃旧注释 + 丢弃 master 空侧）。判定"哪边是保命修复"：看该文件在技能坑位里有无对应条目（坑20/21/42）
   - **⭐ 跨仓库依赖坑（子模块合并 ≠ 主仓库合并）**：NX_code 的 master 版 `start_odin.sh` 调 `roslaunch task_run_scripts odin1_ros1.launch` + `odin_px4_and_others.launch`（注释"含 odom_throttle + odom_to_mavros"），**但这些 launch 文件在 task_run_scripts 仓库的 `launch/odin/` 目录，且只在 origin/main 上**（`git ls-tree origin/main -- launch/odin/` 确认有 odin1_ros1/odin_px4_and_others/odin_real_flight 三个；当前 feat/4uav-guide 工作区没有）。**NX_code merge 完 ≠ 系统配套完成**——start_odin.sh 会找不到 launch/odin/ 启动失败，必须继续 merge task_run_scripts 的 origin/main（main 领先 27 提交，含 launch/odin/ + odin_real_flight.yaml 等）。**合并前检查启动链依赖的每个文件在哪个仓库哪个分支**
   - **⭐ 跨仓库依赖的轻量解法 = 定向拉取（用户选方案A，2026-08-19 实机）**：**不想 merge 整个 task_run_scripts 时，可用 `git checkout origin/main -- <path>` 只拉需要的文件/目录**（不动分支历史，未提交状态，验证正常后再决定提交）。本案例拉取：`git checkout origin/main -- launch/odin/ config/control_command.yaml`。**odin1_ros1.launch 依赖清单（拉完必须逐个查）**：① `config/control_command.yaml`（$(find task_run_scripts)/config/，cloud_reprojection/image_overlay 节点 rosparam load 它，start_odin.sh 也写它；**task_run_scripts 侧这份原本不存在**，odin_ros_driver/config/ 那份是另一个文件别混）② `rviz/odin_rviz_me.rviz`（当前分支已有 ✅）③ odin_ros_driver 包本身（有 launch_ROS1/odin1_ros1.launch 原版，但 start_odin.sh 用的是 task_run_scripts 包装版）
   - **症状与验证（"点启动感知系统 RViz 不弹"排查）**：症状 = 平板点"启动感知系统"→ RViz 不弹、`host_sdk_sample`/`rviz` 进程都没有；日志铁证 = `RLException: [odin1_ros1.launch] is neither a launch file in package [task_run_scripts]`（在 `log/odin_<时间戳>_<true|false>/odin1_ros1.log`）。**拉完验证**：`timeout 15 roslaunch task_run_scripts odin1_ros1.launch 2>&1 | head` 应看到 PARAMETERS 清单（cloud_reprojection_node/register_keys 等）而非 RLException；`start_odin.sh` 还有 `⚠️ 未找到 .../config/control_command.yaml，跳过 recorddata 设置` 警告 = 该文件缺失的独立信号。**注意 roslaunch 解析成功不等于传感器起来**（还要 USB/权限），但 RLException 消失 = 文件依赖问题已解决
   - **合并后验证清单**：master 新增文件都在（nx_file_transfer.py/nx_paths.py/restart_qgc_services.sh/start_odin.sh）→ 保命修复在（`grep -n '_LOCK\|0.05\|_last_pub' odom_to_mavros.py`）→ 取 master 版的文件与 master 零差异（`git diff origin/master -- <file> | wc -l` = 0）→ `python3 -m py_compile` 语法通过 → launch/odin/ 目录存在
   - **平板按钮机制（用户问"按钮3变成了哪一个"的根源）**：平板/遥控器端定义按钮**显示文案**（如"智能航线任务"、"系统自检"），点击后发 **UserCmd1/2/3**（按序号）；机载 `nx_command_server.py:2585` 收到 → 读 `User_define_commands.yaml` 对应 `UserCmdN.command` 执行。**`name` 字段只是显示名，机载不校验**——平板显示什么由平板 APP 决定，机载只看指令序号。改按钮功能 = 改 yaml 的 command 字段；**"智能航线"字样不在机载任何文件里**（grep 无结果 = 平板端定义），别在机载代码里找。分支间 User_define_commands.yaml 内容不同（本地 feat/4uav-guide 的 UserCmd3="按钮3"+real_flight.launch；master 的 UserCmd3="系统自检"+odin_real_flight.launch），改前先确认平板加载哪份
   - **子模块提交顺序（坑16/21 补充）**：NX_code 是 task_run_scripts 的嵌套子模块，提交必须**先 NX_code 内部 commit → 再 task_run_scripts 里 `git add scripts/NX_code` 提交指针更新 → push 子模块再 push 主仓库**（顺序反了主仓库指针悬空）；SUPER 独立提交互不影响。push 后验证 `git status -sb` 显示 `## feat/...origin/feat/...` 无 ahead 才算同步
   - **判断机载实际生效的按钮命令**：`grep -A3 'UserCmd3' User_define_commands.yaml` 看命令本体；按钮每次点击重读 yaml，改完即生效无需重启服务（2026-08-10 确认）；对比分支差异用 `git diff origin/master -- User_define_commands.yaml`
   - **服务重启（master 版统一入口，2026-08-19）**：`start_qgc_servers.sh`（NX_code master 新增）= roscore + nx_touch_server2 + nx_command_server2 + rviz_full_stream2 三服务统一启动，日志 `log/<时间戳>/` 独立目录 + `log/latest` 软链（保留最近 10 份）。重启验证：三服务日志各自就绪（`HTTP 6004` 指令 / `UDP 6003` 指令通道 / `UDP 6002` 触控 / `等待遥控器连接 (端口 6000)` 推流），`雷达配置已加载: profile='odin'` 确认雷达 profile 正确。停止必须用"纯净"停止脚本（坑43④），wrapper 脚本（nx_command_server2.py 等）只是 import 包装器，逻辑全在 v1 同名文件

49. **⭐ 平板按钮配置三原则（2026-08-19 用户拍板，改按钮前必读）**：
   - **名字≠功能**：`User_define_commands.yaml` 的 `name` 字段机载端**完全不读**（grep 无引用点），只供人看；平板按钮文案由平板 APP 硬编码。**master 提交 4345230 把 UserCmd3 名字改"系统自检"但 command 没动 → 平板点"系统自检"会启动复飞（名字/功能错位坑）**。改名字必须同步检查 command
   - **映射依据**：master 提交 4345230 说明"UserCmd3 显示名与平板按钮文案一致"→ 平板"系统自检"= UserCmd3，类推"智能航线任务" = UserCmd1。机载只认 UserCmd1/2/3 序号（nx_command_server.py:1099 `elif cmd in ["UserCmd1","UserCmd2","UserCmd3"]`），每次点击实时读 yaml 无需重启服务
   - **改前先问清要哪套功能**（2026-08-19 用户两次澄清）：A=SUPER 动静态避障（real_flight.launch 默认航点 real_mission.txt）vs B=复飞（global_path_publisher guidepoints_new.txt + real_flight.launch use_waypoint:=true endpoint.txt）。**用户明确"只要我之前推送分支配置的按钮3的功能"——别拿 master 版替代推送分支版**
   - **最终配置（2026-08-19）**：UserCmd1=智能航线任务=复飞功能（原按钮3）、UserCmd2=停止SUPER（不变）、UserCmd3=系统自检=空（`echo "[系统自检] 未配置功能, 仅占位" && true`）。修改=备份 .bak_btn_<时间戳> + python 精确替换（锚点断言）+ `yaml.safe_load` 验证
   - **跨仓库坑症状**：UserCmd1 若指向 `odin_real_flight.launch`（当前分支不存在，在 task_run_scripts origin/main 的 launch/odin/）→ 点击启动失败 = 未合并 task_run_scripts 的直接症状（坑48 跨仓库依赖）

50. **⭐ odin RViz 空白 = Fixed Frame 与 TF 链不匹配（2026-08-19 实机，RViz 弹出但无点云无图像）**：
   - **症状**：感知系统启动后 RViz 窗口正常弹出（说明 launch/odin/ 依赖已解决，坑48），但点云/图像显示项全空白
   - **排查顺序（先确认数据在流，再查坐标系）**：① `rostopic hz /odin1/cloud_raw` ~10Hz、`/odin1/image` ~10Hz = 数据在发（不是传感器问题）② 查 `rviz/odin_rviz_me.rviz` 的 `Fixed Frame`（grep -A5 'Global Options:'）vs 话题实际 frame_id（`rostopic echo -n1 /odin1/cloud_slam/header/frame_id` = odom；cloud_raw = lidar；image = 空）vs TF 树（`rosrun tf tf_echo odom imu` 有值 = map→odom→imu 链通）③ **Fixed Frame=map 但 TF 树里没有 map→点云/图像全不显示**
   - **根因**：当前工作区 `odin_rviz_me.rviz` 的 Fixed Frame 被改成 `map`（某次编辑引入），**之前提交版 d58d645 / origin/main 版都是 `odom`**（cloud_slam 是 odom 帧，用 odom 完全正确）。RViz 需要 Fixed Frame 到话题 frame_id 的完整 TF 链，map 帧不在 odin 的 TF 树里 → 显示空白
   - **修复**：`Fixed Frame: map → odom`（python 替换 + 备份 .bak_fixedframe_<时间戳>），改完**必须重启 RViz/感知系统**才生效（rviz 启动时读配置）
   - **⚠️ cloud_raw(lidar 帧) 显示不了是正常现象**：odin 只发 odom→imu TF，无 odom→lidar 链 → RViz 无法把 lidar 帧点云变换到 Fixed Frame。**看 cloud_slam(odom 帧) 即可，不是故障**（之前提交版也如此）。若必须看 cloud_raw，需静态 TF 桥接（坑22 cloud_raw_lidar_to_odom.py 思路）
   - **RViz 配置三态验证速查**：Fixed Frame=odom + cloud_slam 显示 = 正常；Fixed Frame=map 全空 = 本坑；Fixed Frame=odom 但 cloud_raw 空 = 正常（无 lidar TF）
   - **⭐ "显示界面和推送分支不一样" = 两套 odin rviz 配置，启动链决定用哪份（2026-08-19 用户追问"之前点启动雷达弹出的 RViz 不是这样"）**：odin 有**两套 rviz 配置**——① `odin_ros_driver/config/odin_ros.rviz`（**32 个显示项**：cloudslam_reprojected/reprojected_image/raw/render/Odometry/Odometry_high 等，驱动自带，**合并前** Radar_Launcher.yaml 直接 `roslaunch odin_ros_driver odin1_ros1.launch` 用的就是它）② `task_run_scripts/rviz/odin_rviz_me.rviz`（**10 个显示项**，精简版，**合并后** start_odin.sh 用 `roslaunch task_run_scripts odin1_ros1.launch` 引用它）。**用户看到的界面差异不是配置损坏，是启动链换了 launch 文件**（坑48 跨仓库依赖的连带）。恢复旧界面：改 `launch/odin/odin1_ros1.launch` 的 `rviz_config` arg 指向 `$(find odin_ros_driver)/config/odin_ros.rviz`。判据：`grep -c 'Class: rviz/' <两个 rviz>` 对比显示项数（32 vs 10）；`grep rviz_config <两个 launch>` 看各自引用。**改 rviz 显示布局类配置后必须重启感知系统/RViz 才生效**

#### 坑位清单（续，2026-08-25 实测）

51. **⭐ 启动智能航线任务 → 雷达界面退出 = 两个 launch 各带一个 rviz，同名顶替（2026-08-25 实机）**：
   - **症状**：点"智能航线任务"后，雷达界面（odin RViz）消失/黑屏；智能航线任务自身进程（fsm_node/bridge/waypoint_mission）其实都活着
   - **日志铁证**（px4_and_others.log 或 ~/.ros/log 尾部）：`[WARN] Shutdown request received.` + `Reason given for shutdown: [[/rviz] Reason: new node registered with same name]`
   - **机制**：start_odin.sh 已启动一个 `name=rviz` 的雷达界面；`real_flight.launch` 里 `<node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rviz_config)"/>` 也叫 `rviz` → ROS 同名节点互斥，新节点注册时把旧的强制 Shutdown → 雷达界面退出，平板推流画面也断（推的就是那个被杀掉的 rviz）
   - **修复（已落地）**：real_flight.launch 第 38 行 `name="rviz"` → `name="rviz_super"`（备份 `.bak_rviziname_20260825`）。改 launch 即时生效（roslaunch 每次读文件），无需重启服务/编译
   - **推流无副作用确认**：rviz_full_stream2.py 用 `mss` **整屏截图**（monitor 级，`LIYUMEN_MONITOR_INDEX` 可指定），完全不按节点名/窗口抓取 → 改名后平板画面不受影响
   - **与坑9/17 区分**：坑9/17 是**重复启动同一 launch** 顶替；本坑是**两个不同 launch（start_odin.sh 的雷达界面 vs real_flight.launch 的规划视图）各带一个 rviz 互顶**。排查"界面退出"先看进程谁还活着 + 找 `new node registered with same name` 日志
52. **SUPER 原生点击导航（RViz 2D Nav Goal）—— "智能航线任务"改回点选导航的实现路径（2026-08-25 源码确认）**：
   - **fsm 双目标通道**（`task_run_scripts/config/real_flight.yaml`）：`click_goal_topic: /move_base_simple/goal`（RViz 2D Nav Goal 默认话题）+ `waypoint_goal_en: true/false`（是否订阅航点任务）+ `waypoint_goal_topic: /waypoint_goal`（waypoint_mission 发的复飞终点）
   - **bridge 也订阅 /move_base_simple/goal**（super_to_px4_bridge.cpp:204 goalCallback，只缓存 current_goal 供起飞阶段用）
   - **fsm 状态机**：WAIT_GOAL（无 goal 干等，日志 `Wait for goal.`）→ 收到 goal（`setGoalPosiAndYaw` → `gi_.new_goal=true`）→ GENERATE_TRAJ（PlanFromRest）→ FOLLOW_TRAJ
   - **切纯点击导航四步**：① real_flight.yaml `waypoint_goal_en: false` ② 启动加 `use_waypoint:=false`（不启 waypoint_mission）③ 不启 global_path_publisher（无 guide_path 硬约束 → 纯 A*+MINCO 自由规划）④ RViz 用 2D Nav Goal 点目标（top_down.rviz / odin_rviz_me.rviz 都自带该工具）
   - **待验证点**：无 guide_path 时 SUPER 的纯 A* 分支受 `frontend_in_known_free` 影响（true=未知当墙坑44 可能卡死，false=未知当自由坑36 可能撞墙）——点击导航模式也要按"先扫后飞"原则用
53. **判断"二进制是否最新"用文件 mtime，不是 git 提交时间（2026-08-25 用户疑问"为什么我 catkin_make 没编译到最新"）**：
   - **make 增量编译判据 = 源码文件 mtime vs 目标文件 mtime**（源码更新才重编）。git commit 时间 ≠ 文件修改时间（提交可以晚于编译完成）
   - **正确判断命令**：`find <src> -name '*.cpp' -newer <binary>` —— 空输出 = 二进制已含全部最新代码。**别拿 git log 时间对比**（会把"只改了其他文件"的提交误判成需要重编）
   - **本会话实例**：fsm_node 二进制 8-14，但 8-19 的提交（2fbadee）只改了 bridge 文件 → fsm_node 源码 8-14 后没动 → 8-14 二进制就是最新，catkin_make 快速结束且无 Built 输出 = 增量判定无需重编，**不是失败**
   - **强制重编**：`touch <source_file>` 后再 catkin_make（mtime 变新必然触发），或删 build 目录全量

54. **⭐ 子模块 detached HEAD 提交后 push 被拒：commit 落在游离头，本地分支指针没跟上（2026-08-25 发布 v1.0.0 实机）**：
   - **场景**：NX_code 子模块日常停在 detached HEAD（`## HEAD（非分支）`），直接 `git commit` 会创建"不属于任何分支"的提交（本会话 b623bfd）；随后 `git push origin master` 推的是**本地 master 分支指针**——它可能还指向历史旧点（本会话 4ed08bb，与远端 464cc63 分叉）→ non-fast-forward 被拒（提示"推送的分支最新提交落后于其对应的远程分支"）
   - **判读**：push 被拒先别慌，查 `git log -1 <本地master>` vs `git log -1 HEAD`——两者不同 = 提交落游离头、推的不是刚提交的那个。另外 `git branch --contains <newsha>` 确认新提交不在任何分支
   - **正确姿势**：① `git push origin HEAD:master`——HEAD 的父 == origin/master 当前点（快进）即成功（b623bfd 父=464cc63=远端 HEAD ✅）；② `git branch -f master <newsha>` 同步本地 master 指针防后续再落游离头；③ `git ls-remote origin master` 验证远端已更新
   - **配套（task_run_scripts 主仓库）**：子模块 push 完还要在父仓库 `git add scripts/NX_code` + commit 更新指针 + push（顺序：先子模块 commit/push → 再父仓库指针，见坑48）

#### ⚠️ 实机访问与仓库位置（2026-09-22 核实，先读这节再动手）

**局域网里有两台都叫 `fouruav` 的机器，版本完全不同，别搞混：**

| | ✅ 正确的那台（含 real_flight 全线） | ❌ 旧的那台 |
|---|---|---|
| IP | **<NET_WLAN_C>.131**（曾用 <NET_WLAN_D>.131） | <NET_LAN2>.110 |
| SUPER 位置 | `~/catkin_ws/src/SUPER` + `~/catkin_ws/src/task_run_scripts` | `~/odin_ws/src/{SUPER,task_run_scripts}` |
| SUPER 提交 | 分支 `feat/guide-path-c` @ **`2fbadee`**(2026-08-19，"Phase4.5 扫描") | 分支 `dev/4uav` @ `5ad78c9`(2025-09-26，"纯位置控制") |
| task_run_scripts | 分支 `feat/long-distance-replay` @ **`a24bec4`**(2026-08-24) | 无 `real_flight` 系列配置（只有 `realman.*`） |
| 判定 | **用它** | 版本太旧，没有 real_flight / waypoint 通道 |

- **hostkey 固定 `SHA256:<ONBOARD_HOSTKEY_FP>`，IP 会变** → 连之前先 `ssh-keyscan -t ed25519 <ip> | ssh-keygen -lf -` 核对该指纹即可确认"还是那台机器"
- 仓库 remote 是 `git@github.com:jihaofly/{SUPER,task_run_scripts}.git`（**不是** strongthenly 的仓库）
- `~/catkin_ws` 里两者都是 **git 子模块**（`.git` 是文件，真库在 `.git/modules/src/`）→ **拷贝必须用 tar 不要 clone**，否则丢掉未提交的调参（实测 task_run_scripts 有 4 处未提交改动）
- **打包瘦身**：`task_run_scripts/log/` 622M（px4_and_others.log 单文件 225M）、`SUPER/mars_uav_sim/`(dae+pcd 150M)、`SUPER/misc/`(gif 75M) 全是无用体积。排除后**只有 3.0M / 966 条目**即可完整带走代码+配置
- **依赖很干净**：`quadrotor_msgs`（从 `~/fuel_ws/src/FUEL/uav_simulator/Utils/` 拷，同一上游）+ 系统 Eigen/PCL/yaml-cpp/Boost 即可；**不需要 osqp**（CMakeLists 没要求）；`rog_map` 就在 SUPER 仓库内
- **`mars_uav_sim` 要打 `CATKIN_IGNORE`**：它是 SUPER 自带的仿真包，含同名 `mars_quadrotor_msgs` 会与 `quadrotor_msgs` 冲突，且 `marsim_render/perfect_drone_sim` 需额外渲染依赖
- 编译命令：`catkin_make -j4 -DROS_EDITION=ROS1`

#### 🏗️ SUPER 移植到纯仿真环境（2026-09-22 实战，编译全通）

目标：把 SUPER 编进 `ros1-twin`（Ubuntu20.04 + Noetic，无 libdw-dev）与 AirSim+FAST-LIVO2 组成探索闭环。
工作空间：`~/super_sim_ws`（与 `~/fuel_ws`/`~/twin_ws` 隔离）。

**依赖很干净**：系统 Eigen/PCL/yaml-cpp/Boost 已有；`rog_map` 在 SUPER 仓库内；**不需要 osqp**。

#### ⚠️ 坑 A：`quadrotor_msgs` 必须用 SUPER 自带的那份

`SUPER/mars_uav_sim/mars_quadrotor_msgs/` —— **目录名叫 mars_quadrotor_msgs，但 `package.xml` 里的包名是 `quadrotor_msgs`**！
（我曾误以为它是"额外的仿真包"而排除，并从 FUEL 拷了 `quadrotor_msgs` → 61 个编译错误。）

两份 `PolynomialTrajectory.msg` 字段完全不同：
| | SUPER 自带（对） | FUEL 版（错） |
|---|---|---|
| 字段 | `type` / `piece_num_pos` / `order_pos` / `start_WT_pos` / `time_pos` / `coef_pos_x\|y\|z` | `action` / `num_order` / `num_segment` / `coef_x` / `time` / `order` |
| 常量 | `HEART_BEAT` / `POSITION_TRAJ` / `YAW_TRAJ` | `ACTION_ADD` / `ACTION_ABORT` |

→ **必须保留 `mars_quadrotor_msgs`**，只 `CATKIN_IGNORE` 掉 `mars_uav_sim` 里的 `marsim_render`(渲染) 和 `perfect_drone_sim`(自带场景)。

#### ⚠️ 坑 B：`backward.hpp` 的 libdw 依赖（三处都要改，缺一不可）

仿真机没装 `libdw-dev` → `fatal error: dwarf.h` + `cannot find -ldw`。**光改一处不够**，上游有三层"合力"引用它：

1. **调用方硬编码**（真正的主因，最隐蔽）：`#define BACKWARD_HAS_DW 1` 直接写在 4 个 App 里 ——
   `super_planner/Apps/fsm_node_ros1.cpp`、`fsm_node_ros2.cpp`、`mission_planner/Apps/ros1_waypoint_mission.cpp`、`ros2_waypoint_mission.cpp`
   → 改成 `0`（放在 `#include` 之前才是它生效的原因）
2. **头文件兜底**：3 份 `backward.hpp` 的 `#else` 分支里 `#undef BACKWARD_HAS_DW` 后又 `#define BACKWARD_HAS_DW 1`
   （`super_planner/include/utils/header/`、`mission_planner/include/utils/`、`rog_map/include/super_utils/`）
   → 该分支只在四个 `#elif` 全不命中时才走；上游注释第 183 行自己写着"默认应是 `BACKTRACE_SYMBOL == 1`"，兜底却反手开 DW（上游 bug）
   → 改 `#define BACKWARD_HAS_DW 0` + 加 `DW_DISABLED_FOR_SIM` 注释便于日后 grep
3. **链接项**：`super_planner/`、`rog_map/`、`mission_planner/` 三个 `CMakeLists.txt` 的 target_link_libraries 里有 `-ldw`
   → 注释掉（`ros/*.CMakeLists.txt` 是 ROS1/ROS2 模板，不参与编译，可不动）

**更稳的替代**：编译时加 `-DBACKWARD_HAS_BACKTRACE_SYMBOL=1` 也能让代码走 backtrace 分支（`add_definitions` 实测有效），
但**挡不住第 1 条的调用方硬编码** → 仿真移植时**直接改源文件最可靠**。
**验证方法（强烈推荐）**：预处理实测比看源码可靠 ——
`echo '#include "utils/backward.hpp"' > /tmp/t.cpp && g++ -E -DUSE_ROS1 -Iinclude -x c++ /tmp/t.cpp | grep -c dwarf.h` → 应为 **0**。

#### ⚠️ 坑 C：两个调试 App 需要 ncurses 开发头文件

`super_planner/Apps/traj_opt_tuning.cpp`（交互式调参 UI）和 `read_replan_log.cpp`（日志阅读器）都 `#include <ncurses.h>`。
仿真机只有运行时 `libncurses6`、没有 `libncurses-dev`，而装它需要 sudo。
→ **注释掉这两个 `add_executable` 块**即可，不影响核心的 `fsm_node` / `super_to_px4_bridge`（已留说明注释，日后 `sudo apt install libncurses-dev` 可一行恢复）。

#### 编译命令与产物

```bash
cd ~/super_sim_ws && source /opt/ros/noetic/setup.bash
catkin_make -j4 -DROS_EDITION=ROS1

```

#### 经验教训（重要）

- **改文件前先想清楚"改动会不会落到注释/文档行上"**：我写的一个批量 sed 式脚本按"行不以 # 开头"判断，
  结果把 `// - g++/clang++ -ldw ...` 这种文档注释行也插入了 `# LDW_DISABLED_FOR_SIM`，
  在 **C++ 头文件里产生 `invalid preprocessing directive`**（6 个新错误）。脚本批量改代码务必限定文件类型 + 先 dry-run。
- **排错顺序**：`编译错误` → `链接错误` → 分开看。本例 61 个编译错误全是 `quadrotor_msgs` 版本（坑A），
  1 个链接错误是 `-ldw`（坑B 第 3 条）。混淆在一起时容易误判。
- **备份要带时间戳并保留"已知良好"版本**：`.bak_<原因>_<时间戳>` 的命名在回滚时比 `.bak` 可靠得多。

#### 🔗 FUEL 探索目标点 → SUPER（2026-09-22，目标点发布已验证）

**需求**：FUEL 只负责"去哪"（frontier 决策），SUPER 负责"怎么去"（规划+避障）。

**现状事实（实测）**：FUEL 原本**不发布**目标点 —— `next_pos/next_yaw` 只是
`fast_exploration_manager.cpp` 里的局部变量，直接喂给 `planExploreMotion()`；
`/planning_vis/viewpoints` 是 **Marker**（仅供 RViz），不能当目标用。

#### 给 FUEL 加目标点发布（3 处改动，纯新增不改原逻辑）

文件：`fuel_planner/exploration_manager/{include/.../fast_exploration_manager.h, src/fast_exploration_manager.cpp}`

1. **头文件 `private:` 段**加成员：`ros::Publisher explore_goal_pub_;`
2. **`initialize()` 开头**：`explore_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/exploration_goal", 10);`
   （需 include `<geometry_msgs/PoseStamped.h>` 和 `<tf/transform_datatypes.h>`）
3. **插入点选 `std::cout << "Next view: " ...` 这一行之后** ——
   这是**所有 return 之前**的唯一必经处，能覆盖全部目标选择分支（放 `planExploreMotion` 里更靠后的位置会被 early-return 漏掉）：
   ```cpp
   { geometry_msgs::PoseStamped m; m.header.stamp=ros::Time::now(); m.header.frame_id="world";
     m.pose.position.x=next_pos.x(); ...y(); ...z();
     m.pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);
     explore_goal_pub_.publish(m); }
   ```
   （`next_pos` 在 125~219 行多个分支里赋值，224 行时已确定）

#### ⚠️ 坑 D：别用 `strings`/`grep -a` 验证二进制里有没有某个字符串

我改了代码、编译 error=0，但 `strings exploration_node | grep exploration_goal` 一直返回 **0**，
误判成"编译没生效"，白查了好几轮（删 .o 强编、查 DependInfo、查 flags.make…都正常）。
真相：**ELF 字符串表会做尾部合并**，`/exploration_goal` 可能不以独立 `\0` 结尾，`strings` 不单独输出它。
反证：把话题临时改成 `/exploration_goal_ZZTEST`（更长的唯一串）→ 立即可 `strings` 到 → 证明编译链完全正常。

**权威验证 = 运行时**：
```bash
export ROS_MASTER_URI=http://localhost:11321
rosmaster --core -p 11321 & sleep 3
rosrun exploration_manager exploration_node & sleep 6      # 裸起会在 initialize 后 abort（缺参数yaml），但不影响 advertise
rostopic info /exploration_goal     # → Type: geometry_msgs/PoseStamped, Publishers: /exploration_node  ✓
```
（裸起 abort 是正常的：缺 launch 提供的参数；话题注册发生在 abort 之前，所以仍可验证。）

#### 架构与冲突（重要）

```
FAST-LIVO2 ──/cloud_registered──┬─> FUEL(frontier 决策) ─> /exploration_goal ─> 仲裁器 ─> /waypoint_goal ─> SUPER ─> /planning/pos_cmd
                                 └─> SUPER(rog_map+MINCO 规划)
```
- ⚠️ **两条控制链会抢飞机**（坑45：SUPER 的 bridge 无条件接受任何节点的 `/planning/pos_cmd`）：
  FUEL 的 `traj_server` 也在发这个话题 → 必须把 FUEL 的 `traj_server` 输出 **remap 到 `/fuel/pos_cmd_sink`**（内部逻辑照常跑、输出被丢弃），**不要**停掉它（FSM 依赖它）。
- ⚠️ **目标要限频**：FUEL 每轮 replan（约 1Hz）都产出新目标，直接转发会每秒打断 SUPER 轨迹 → 必须加仲裁（"已到达当前目标"或"当前目标失效"时才放行）。
- `quadrotor_msgs` 的 `PositionCommand` 是两者共用类型（FUEL 与 SUPER 各自带一份、字段可能不同，见坑 B）。

#### 仿真配置（`sim_real_flight.yaml`）—— 2026-09-22 实测

从实机 `odin_real_flight.yaml` 复制后改了 4 处（`config_path` 支持命令行覆盖，实机配置不动）：

| 参数 | 实机 | 仿真 | 理由 |
|---|---|---|---|
| `rog_map/ros_callback/cloud_topic` | `/odin1/cloud_slam` | **`/cloud_registered`** | 用 FAST-LIVO2 点云（与 FUEL 同源） |
| `rog_map/fix_map_origin` | `[0,0,1.5]` | **`[0,0,0]`** | 1.5 是雷达安装高度；`/cloud_registered` 原点即起飞点 |
| `traj_opt/boundary/max_vel` | `0.3` | **`1.0`** | 实机保守值 |
| `traj_opt/boundary/max_acc` | `0.4` | **`1.0`** | 同上 |

**`odom_topic` 保持 `/mavros/local_position/odom`**（仿真同样有）；`robot_r 0.35` / `inflation_step 2`（0.4m 膨胀）/ `frontend_in_known_free: true` 保守沿用。

**预检方法（不需要 AirSim，强烈建议改配置后先跑这个）**：
```bash
cd ~/super_sim_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash
pkill -f "rosmaster --core -p 11321"; rosmaster --core -p 11321 & sleep 3
export ROS_MASTER_URI=http://localhost:11321
rosrun super_planner fsm_node _config_path:=$(pwd)/src/task_run_scripts/config/sim_real_flight.yaml > /tmp/fsm.log 2>&1 &
sleep 10 && grep -E "Load param|ROG-Map" /tmp/fsm.log       # 逐条核对实际加载值
```
正常表现：所有参数打印 `success`，末尾只有 `[ROG WARN] No point cloud input` / `[Fsm] No odom`（没点云/里程计的**正常**提示），**无 error**。

#### ⚠️ 坑 E：`virtual_ground_height` / `virtual_ceil_height` 会被量化，yaml 值 ≠ 实际值

实测：yaml 写 `virtual_ceil_height: 3`、`virtual_ground_height: -3`，而
`[ROG-Map] Init, resetMapSize` 打印的**实际值是 ±2.25**（≈15×0.15，被量化到 `resolution` 的整数倍）。
后果：设得太小会被量化到 0 附近（skill 旧记录：`-0.9` 量化成 `+0.15` → 地面 z=0 触发 `below ground` 报错）。

**判断方法**：只看 `resetMapSize` 那一行打印的实际值，**别信 yaml**。
当前仿真配置实际为 `virtual_ground=-2.25 / virtual_ceil=+2.25`，而 `/cloud_registered` 原点（起飞点）z=0 落在中间 —— **可用，无需再改**（若把 `fix_map_origin` 抬到 1.5 则需重算）。

#### ✅ 接口连通性验证（无需 AirSim / 点云，改完必跑）

**目标**：确认 `FUEL目标 → 仲裁器 → /waypoint_goal → fsm_node → FSM状态机 → 规划` 全链路贯通，
不需要传感器数据、不需要 AirSim Play。这是"接上传感器前的最后一道自检"。

```bash
source /opt/ros/noetic/setup.bash
source ~/fuel_ws/devel/setup.bash
source ~/super_sim_ws/devel/setup.bash

export ROS_PACKAGE_PATH=$HOME/super_sim_ws/src:$HOME/fuel_ws/src:$ROS_PACKAGE_PATH

export ROS_MASTER_URI=http://localhost:11321
rosmaster --core -p 11321 & sleep 3

~/super_sim_ws/devel/lib/super_planner/fsm_node \
    _config_path:=$HOME/super_sim_ws/src/task_run_scripts/config/sim_real_flight.yaml > /tmp/fsm.log 2>&1 &
~/fuel_ws/src/fuel_bridge/scripts/goal_arbiter.py _odom_topic:=/nonexistent > /tmp/arb.log 2>&1 &
sleep 8

rostopic info /waypoint_goal     # Publishers: /goal_arbiter, Subscribers: /fsm_node   ← 这就是通了
```

然后注入一个假目标：
```bash
python3 -c "
import rospy; from geometry_msgs.msg import PoseStamped
rospy.init_node('inj', anonymous=True)
p = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=1); rospy.sleep(1.5)
m = PoseStamped(); m.header.frame_id='world'
m.pose.position.x, m.pose.position.y, m.pose.position.z = 3.0, 0.0, 1.0
m.pose.orientation.w = 1.0; p.publish(m)"
```

**期望 fsm_node 日志**（实测）：
```
[Fsm] Get goal at 3.075 0.075 0.975
[Fsm] Receive click goal at: [3.075 0.075 0.975]; goal yaw: 0 deg
[Fsm]: [MainFsmCallback] change state from [INIT] to [WAIT_GOAL].
[Fsm]: [MainFsmCallback] change state from [WAIT_GOAL] to [GENERATE_TRAJ].
[Fsm] PlanFromRest failed, switch to AVOID_WAIT (park & wait).   ← 无点云/无里程计时【正常】
```

#### ⚠️ 坑 F：fsm 会把目标**吸附到栅格中心**（±半分辨率偏移）

发 `(3.0, 0.0, 1.0)` → fsm 内部变成 `(3.075, 0.075, 0.975)`，偏移量 = `resolution(0.15)/2 = 0.075`。
**这是正常行为不是 bug** —— 实机上"目标点总有厘米级偏移"就来自这里。做精度对比实验时要剔除这半格偏移。

#### ⚠️ 坑 G：多工作区 source 顺序会互相覆盖 ROS_PACKAGE_PATH

`source fuel_ws/devel/setup.bash` 之后再 `source super_sim_ws/devel/setup.bash`，
`super_planner` 会找不到（`rospack: package not found`）—— 后 source 的覆盖了前者。
**解法**：手动叠加 `export ROS_PACKAGE_PATH=$HOME/super_sim_ws/src:$HOME/fuel_ws/src:$ROS_PACKAGE_PATH`，
或**直接调 `devel/lib/<pkg>/<node>` 二进制**（最省事，不依赖 rospack）。

#### 🚀 一键启动 FUEL→SUPER 联调（`~/super_explore_all.sh`）

联调别手敲 5 条命令（多工作区 source 顺序、改道开关、仲裁器、触发都容易漏）。
源码 `~/fuel_ws/src/fuel_bridge/scripts/super_explore_all.sh`（`~/` 有软链接，与 `fuel_all.sh` 同一套写法）：

```bash
bash ~/super_explore_all.sh 1                 # 参数 1 = 退出时保存建图 PCD
bash ~/super_explore_all.sh 0                 # 不保存（默认；等价 --pcd 可强制开）
bash ~/super_explore_all.sh 1 --print-params  # ★ 只打印参数解析结果后退出（不启动任何东西）
bash ~/super_explore_all.sh --dry-run         # 预演：仲裁器只打印不发布（首次必用）
bash ~/super_explore_all.sh --no-fuel         # 只起 SUPER（单独验证）
bash ~/super_explore_all.sh 1 5.0             # 存 PCD + 仲裁器最小放行间隔 5s（裸数字）
bash ~/super_explore_all.sh 1 --no-rviz       # 不起可视化 RViz（无头跑／省 GPU）
```
> `--no-rviz` 关掉的是「点云 + rog_map + 轨迹」那个超级 RViz（坑 N）。
> 默认开启；脚本会在起它之前补 `world→camera_init` TF，否则点云不显示（坑 M）。

7 步**每步带硬检查 + 明确诊断**（不干等）：0 AirSim RPC → 1 twin 链路（`/clock`+`/cloud_registered`）
→ 2 PX4 MAVLink 14610 → 3 MAVROS（SUPER 的 odom 来自 `/mavros/local_position/odom`）
→ 4 SUPER（fsm_node + bridge）→ 5 FUEL（改道）→ 6 仲裁器 → 7 **链路自检 + 触发**。

**第 7 步是防抢控制的总闸**：断言 `/planning/pos_cmd` 的**发布者数必须 = 1**；
>1 说明 FUEL 的 remap 没生效、两条控制链在抢（脚本直接打红报警并指向 `/tmp/fuel_real_super.log`）。
脚本内已叠加 `ROS_PACKAGE_PATH`（见坑 G），结尾打印各节点日志路径。

**顺序建议**：先跑上一节的**接口连通性验证**（无需 AirSim）确认链路 → 再 `--dry-run` 预演仲裁器 → 才真跑。

#### ★ PCD 保存的三个事实（查过全仓，别再折腾）

1. **点云地图只能由 FAST-LIVO2 产生**。grep 过 `SUPER/rog_map/` 与 `super_planner/`：
   **没有任何写出 PCD 的代码**（只有 `load_pcd_en` 之类的**读取**参数）。所以"保存最终建图 PCD"
   这个开关**只能作用于 FAST-LIVO2**，与 SUPER 无关——别去 SUPER 侧找保存开关。
2. 机制 = `twin_livo_fast.sh` 读**环境变量 `PCD_SAVE`**（默认 0）；为 1 时
   `rosparam set /pcd_save/pcd_save_en true` + `interval -1`，输出到
   `~/twin_ws/src/FAST-LIVO2/Log/pcd/{all_raw_points.pcd, all_downsampled_points.pcd}`（原始可达 GB 级）。
   上层脚本只需 `export PCD_SAVE=$PCD` 转交，**不用自己 set rosparam**。
3. **必须 SIGINT 才落盘** → 关闭一律 `bash ~/fuel_stop.sh`（内部 `pkill -INT -f fastlivo_mapping`），
   **`kill -9` 不会保存**。已写进脚本头注释 + 结尾提示 + 集成文档。

#### ★ 多次实验不互相覆盖：PCD 按时间戳归档（2026-09-22 落地）

**问题**：FAST-LIVO2 的文件名是**硬编码**的（`LIVMapper.cpp` 的 `savePCD()` 写死
`Log/pcd/all_raw_points.pcd` + `all_downsampled_points.pcd`）→ 第二次实验直接**覆盖上一组数据**。

**不通的路**：改 C++ 源码加时间戳。→ **走脚本层归档**，零编译风险。

**做法：在两处归档，双保险**（缺一不可）：

| 位置 | 时机 | 为什么需要 |
|---|---|---|
| `twin_livo_fast.sh` | **启动 FAST-LIVO2 之前**（在 `if ! pgrep -f fastlivo_mapping` 分支内） | 防上次被 `kill -9` → 停止脚本没跑到 → 裸名文件残留被本次覆盖 |
| `super_explore_stop.sh` | **SIGINT 保存完成、确认退出之后** | 正常路径的归档点 |

**核心规则（照抄，实测 12/12 通过）**：
```bash
for f in "$PCDDIR/"*.pcd "$PCDDIR/lidar_poses.txt"; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  # 已带时间戳的跳过 —— 这条让归档【幂等】，可反复运行
  case "$b" in
    *_[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]_[0-9][0-9][0-9][0-9][0-9][0-9].*) continue ;;
  esac
  mt=$(date -r "$f" +%Y%m%d_%H%M%S 2>/dev/null) || continue   # ★ 时间戳取【文件自身 mtime】
  stem="${b%.*}"; ext="${b##*.}"
  mv "$f" "$PCDDIR/${stem}_${mt}.${ext}" 2>/dev/null && n=$((n + 1))
done
```
- **为什么用 mtime 而不是 `date +%s`**：mtime 让**历史遗留的裸名文件也被归档到它自己的时间**
  （`all_raw_points.pcd` 的 mtime 就是它那次的保存时刻），一个循环同时处理"本次的"和"历次的"。
- **幂等**是硬要求 —— 启动脚本和停止脚本都会跑这段，二次运行必须零改动。
- 效果：`all_raw_points_20260922_204530.pcd` / `all_downsampled_points_*.pcd` / `lidar_poses_*.txt`，
  每次实验一组，互不覆盖。

**⚠️ 测这段归档逻辑时绝不能在真实 `Log/pcd/` 上跑**（那里有用户的历史建图数据，实测 116MB）。
用 `python3` 从脚本里**按锚点切出这段代码**，把 `$ROOT/Log/pcd` 替换成临时目录再执行，
并断言"真实 `all_raw_points.pcd` 仍在原处"作为最后一条用例 —— 本次就是这么做的。

#### ★ 位置参数要写成「顺序无关」（本次实测的设计教训）

最初写成 `PCD=${1:-0}` + `case 0|1) shift` —— **参数顺序敏感**：
`1 --dry-run` 正确，但 `--print-params 1` 会把 `--print-params` 当作 pcd、回退为 0 且**不 shift**，
后续参数全部错位（实测 `4.5 1` 被解析成 `min="4.5 1"`）。
改为**在循环里认第一个"裸 0/1"**即根治：
```bash
PCD=0; PCD_SET=0
for a in "$@"; do
  case "$a" in
    0|1)   if [ "$PCD_SET" = "0" ]; then PCD="$a"; PCD_SET=1; fi ;;
    --pcd) PCD=1; PCD_SET=1 ;;
    ...
  esac
done
export PCD_SAVE=$PCD
```
**通则：位置参数与选项混用时，一律在循环里识别，不要用 `${1:-}` + shift** —— 否则调用方换个顺序就静默解析错。

#### ★ 给它加 `--print-params` 自证开关

打印一行解析结果后 `exit 0`（形如 `PARAMS pcd=1 dry=no super=1 fuel=1 trig=no min=3.0`）。
既让**离线验证退化成字符串比对**（比抽 `sed` 代码片段来测可靠得多，
后者因提取范围/函数体拼接出错反而会误导——本次就因此误判过"脚本有 bug"），
又让**用户联调前能自己核对参数** —— 可测试性顺带变成真功能。
该类"参数解析"逻辑一律优先加自证开关，详见 `project-verification-entrypoint` 的
"验证脚本自身最容易出的六个错"第 5、6 条（harness 传参 bug / 自证优于切片）。

#### ★ 「这个要单独跑还是集成？」→ 一律改在**实际启动者**那里治本

用户会问这类架构问题（本次原话："按之前步骤用 remain-on-exit 重启 PX4…这个要单独跑还是集成在
super_explore_all.sh"）。答案固定为**两处、各司其职**：

| 改哪里 | 改什么 | 为什么是这里 |
|---|---|---|
| **实际启动它的脚本**（本例 `twin_livo_fast.sh` —— PX4 是它拉的） | 启动方式本身：`remain-on-exit`、启动后验证目标、失败时 dump 现场 | **治本** —— 所有调用它的上层脚本（`fuel_all.sh` / `super_explore_all.sh`）自动受益 |
| **编排脚本**（`super_explore_all.sh`） | "提示"升级为"硬检查 + `exit 1`"，并**继承下层退出码**：`if ! bash ~/twin_livo_fast.sh --full; then … exit 1; fi` | **快速失败**，别让后面几步白跑（本次就是第 1 步只警告不退出，导致 5 步全白跑还"看起来成功"） |

**⚠️ 不要把操作留成"请手动去 tmux 里敲 XXX"** —— **凡是能脚本化的操作都脚本化**。用户两次点名要求
（"这部分能不能也用 sh 脚本去做"）。手动步骤的代价是：每次都要人记得、新人必漏、出错无痕迹。
**先翻已有脚本找现成手法**：本例 `fuel_all.sh` 第 60-61 行**早就有**
`tmux send-keys -t px4 "mavlink start -u 14610 -o 14611" Enter`，直接沿用比自己重新设计可靠
（连参数写法都别自己编 —— 我早期提示里多写的 `-t 127.0.0.1` 就是错的）。

**每轮可操作的改动都要附带一条「照做版测试流程」**：前置 → 清场 → 启动 → **逐项判据**
（每步给出预期输出）→ 触发 → 观察闭环 → 停止 → 产物验证 → **故障速查表**。
写进交付文档的同时**在对话框贴一份**（用户偏好，别只给文件路径）。

#### 🛑 一键关闭 FUEL→SUPER 联调（`~/super_explore_stop.sh`）

`super_explore_all.sh` 的配套停止脚本（`~/fuel_ws/src/fuel_bridge/scripts/`，`~/` 有软链接）：

```bash
bash ~/super_explore_stop.sh                            # 常规关闭：保留 master，PCD 会保存
bash ~/super_explore_stop.sh --all                      # 全停（连 ROS master）
bash ~/super_explore_stop.sh --no-save                  # 不等 PCD，SIGKILL 快速关
bash ~/super_explore_stop.sh --keep-px4 --keep-mavros   # 只停上层应用，留底层链路
bash ~/super_explore_stop.sh --print-params             # 只打印参数，不关任何东西
```

8 步：① SUPER（bridge→fsm_node，**先停控制输出端**）② FUEL（exploration_node / traj_server /
waypoint_generator / 两个桥）③ goal_arbiter ④ MAVROS ⑤ **FAST-LIVO2（SIGINT → savePCD，等 ≤25s 并列出 PCD 文件）**
⑥ 仿真桥 + RViz ⑦ PX4 ⑧ ROS master（默认保留）。**结尾逐进程核对是否真停了。**

**两个设计要点（照抄）**：
- `ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}` **可覆盖**，并从 URI 提取 `MASTER_PORT`
  给 `--all` 精确匹配 —— 这样**测试能指向一次性端口，绝不碰用户常驻实例**。
- `--print-params` 打印一行 `PARAMS all=… no_save=… keep_px4=… keep_mavros=…` 后退出，与启动脚本同一套自证写法。

**实测行为表**（隔离端口 11321，全程未碰用户 11311）：`--print-params` 不关任何进程 ✅；
默认八个进程全停 ✅；⭐ SIGINT 触发保存（marker 生成）✅；master 默认保留 ✅；
`--keep-px4`/`--keep-mavros` 正确保留 ✅；`--no-save` 走 SIGKILL 且**不**触发保存 ✅；
⭐ `--all` 真停掉目标 master、用户 11311 毫发无损 ✅。

启停脚本的**验证手法**（假进程造信号环境、marker 断言、一次性端口、wipe 防污染）见
`project-verification-entrypoint` 的「验证『启停/生命周期』类 shell 脚本：用假进程造信号环境」一节。

#### ⚠️ 坑 H：`rostopic info` 里的发布者可能是**已经死掉的节点**（僵尸注册）

排查「两条控制链抢 `/planning/pos_cmd`」时它列出 `/traj_server` + `/fsm_node` + `/super_to_px4_bridge`，
但 **`/traj_server` 进程根本不存在**——节点死了 master 仍记着它的 publisher 注册。
**我据此一度误判「FUEL 改道失败、两链在抢」，结论是错的。**

**规矩**：看到「多余的发布者」先 `pgrep -f <节点名>` 交叉确认进程是否真活着，再下结论。
真凭据优先级 **`pgrep` > 发布者注册表**。要清干净就 `super_explore_stop.sh --all` 停掉 master 重起。
（`super_to_px4_bridge` 自身也出现在该话题发布者里，属转发/调试用途，不必紧张。）

#### ⚠️ 坑 I：联调「跑完了但没数据」先查 PX4 死没死，别先调参数

`super_explore_all.sh` 跑到「完成」不等于系统在工作。典型症状 → 真因：

| 症状 | 真因 |
|---|---|
| `/tmp/fuel_real_super.log` **0 字节** | FUEL 的 roslaunch 压根没起来 |
| `super_fsm.log` 刷 `No odom, force return.`（实测刷了 2227 行） | SUPER 收不到里程计 |
| `/mavros/state` 空、`/mavros/local_position/*` 无数据 | MAVROS 起了但**没连上 FCU** |
| `14610/14611` 未监听、`/clock` 不步进 | **PX4 没在跑** |

**完整链**：tmux 拉起的 PX4 曾成功（`Simulator connected on TCP port 4560` → `EKF aligned`）→
**tmux server 消失**（`no server running /tmp/tmux-1000/default`）→ PX4 收 SIGHUP →
`pxh> Exiting NOW.`（`/tmp/px4_run.log` 最后一行）→ 14610 关 → MAVROS 无 FCU → 无 odom → SUPER 永远 No odom。

**判据（照这几条看）**：`pgrep -af "bin/px4"`（空=不在）· `tmux ls` · `tail -25 /tmp/px4_run.log`（找 `Exiting NOW.`）
· `ss -tln | grep -E "14610|14611"` · `rostopic echo -n1 /mavros/state` · `rostopic hz /clock`。

**改进建议（未实施，待用户确认）**：给 tmux 会话加 `remain-on-exit`，PX4 再挂时会话仍在、可直接
`tmux capture-pane -p -t px4` 看退出原因，不必像本次靠日志反推。
细节与恢复命令见 `references/sim-ops-scripts-and-px4-exit.md`。

#### ⚠️ 坑 J：`twin_livo_fast.sh --full` 的检查**只警告不退出**（已修复，2026-09-22）

**症状**：联调跑完发现 SUPER 一直在刷 `[SUPER] in [PlanFromRest]: No odom, force return.`（2227 行），
FUEL 的 launch 日志 0 字节 —— 但脚本"看起来跑完了"。

**根因链（实测证据）**：
```
tmux new-session -d -s px4 "make px4_sitl..."   ← 旧写法：PX4 退出后 tmux server 整个消失
      ↓
/tmp/px4_run.log 尾部: "pxh> Exiting NOW."       ← PX4 自己退出了
tmux ls → no server running on /tmp/tmux-1000/default
      ↓
14610/14611 未监听 → MAVROS 无 FCU → 无 /mavros/local_position/odom
      ↓
SUPER 卡在 No odom，什么都不干
```
**而 `--full` 的检查段当初写的是 `[ $okclk -eq 0 ] && echo "[X] ..."`** ——
**只打印，不 `exit`** → 脚本继续往下跑，后面的步骤全在白跑，看起来"成功"。

**修复（两处，各司其职）**：

1. **`~/ros1_ws/scripts/twin_livo_fast.sh`**（PX4 的实际启动者，治本）：
   ```bash
   tmux new-session -d -s px4
   tmux set-option -t px4 remain-on-exit on          # ← PX4 退出后会话保留，可回看原因
   tmux send-keys -t px4 "cd ~/PX4-AirSim && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log" Enter
   ```
   - 用 `send-keys` 而非 `new-session "cmd"`：配合 `remain-on-exit` 才能在 PX4 退出后仍 `capture-pane` 到画面
   - `--full` 的 `/clock` 检查失败 → **`exit 1`**，并**打印 `tmux capture-pane -p -t px4 | tail -12` + `/tmp/px4_run.log` 尾部**
   - `--full` **新增 14610 硬检查**（MAVROS 依赖它；没监听必然 `No odom`）→ 失败也 `exit 1`
   - **14610 未监听时【自动开启】**（`fuel_all.sh` 早有此手法，直接沿用）：
     ```bash
     tmux send-keys -t px4 "mavlink start -u 14610 -o 14611" Enter   # 送进 pxh
     → 等 10s 验证；失败再补发一次（防 pxh 尚未就绪）；仍失败才 FATAL + dump pxh 画面
     ```
     ⚠️ **正确写法不带 `-t 127.0.0.1`** —— `fuel_all.sh` 用的就是 `mavlink start -u 14610 -o 14611`，
     早期提示里写的 `-t 127.0.0.1` 是错的。
     ⚠️ 发送前先 `pgrep -f bin/px4 && tmux has-session -t px4`，否则会写进不存在的会话。
2. **`super_explore_all.sh`**：`if ! bash ~/twin_livo_fast.sh --full; then ... exit 1; fi`（继承退出码），
   步骤 2 从"提示"升级为**硬检查**：14610 未监听 → 打印诊断 → **exit 1**
   （仅在 `WITH_SUPER=1` 时要求；`--no-super` 只起 FUEL 时跳过此要求）

**诊断 tmux/PX4 死亡的通用手法**（这次靠它定位）：
```bash
tmux ls                                  # no server running -> server 整个没了
cat /tmp/px4_run.log | tail -20          # 找 "Exiting NOW." 之类
ss -tln | grep 14610                     # PX4 专用 MAVLink 是否在
timeout 6 rostopic hz /clock             # /clock 不走 = 仿真冻结 = PX4 没连上 AirSim
env ROS_MASTER_URI=http://127.0.0.1:11311 rosnode list   # master 是否活着（最可靠）
```
> ⚠️ **别用 `pgrep -f "rosmaster --core -p <port>"` 判断 master 死活** —— 实测匹配不到，
> 会让"master 是否被停掉"的测试出现假 PASS。用 `rosnode list`。

#### 配套脚本：启动 / 停止 / 自身验证

| 脚本 | 作用 |
|---|---|
| `~/super_explore_all.sh [pcd] [选项]` | 一键启动（7 步，每步硬检查）；`pcd=1` 退出时存建图 PCD；`--print-params` 自检；**参数顺序无关** |
| `~/super_explore_stop.sh [--all\|--no-save\|--keep-px4\|--keep-mavros]` | 一键关闭（8 步 + 结尾逐个核对残留）；FAST-LIVO2 走 **SIGINT 触发 savePCD** |

**测试这两个脚本的手法**（隔离、不碰用户常驻资源）：
```bash

ln -sf /usr/bin/sleep /tmp/t/fsm_node; /tmp/t/fsm_node 900 &

printf '%s\n' '#!/bin/bash' 'trap "touch /tmp/t/saved; exit 0" INT TERM' \
              'while true; do sleep 1 & wait $!; done' > /tmp/t/fastlivo_mapping

ROS_MASTER_URI=http://127.0.0.1:11321 bash ~/super_explore_stop.sh --all
```
> 坑：假进程若写成 `while true; do sleep 1; done`，bash **不会立即处理 trap**（要等 sleep 返回），
> 会让 SIGINT 测试假失败。必须 `sleep 1 & wait $!`。

#### ⚠️ 坑 K：直接调二进制绕过 launch，会丢掉 `<rosparam command="load">`

**症状**：脚本直接 `setsid $BRIDGE_BIN &` 启动 `super_to_px4_bridge`，看似正常，实则
`super_to_px4_bridge.yaml` 里的 `max_velocity_xy` / 控制增益 / `land_speed` / `failsafe`
**全都没生效**（用了源码默认值）。

**根因**：`real_flight.launch` 里是这么写的
```xml
<node pkg="super_planner" type="super_to_px4_bridge" name="super_to_px4_bridge">
  <rosparam command="load" file="$(find task_run_scripts)/config/super_to_px4_bridge.yaml"/>
</node>
```
`roslaunch` 帮你做了这件事，直接调二进制就没有这一步。

**修法**（效果等同 launch，且可覆盖个别开关）：
```bash
rosparam load ~/super_sim_ws/src/task_run_scripts/config/super_to_px4_bridge.yaml /super_to_px4_bridge
rosparam set  /super_to_px4_bridge/auto_arm true        # 仿真没遥控器，必须自动 arm
rosparam set  /super_to_px4_bridge/auto_offboard true
setsid "$BRIDGE_BIN" > /tmp/super_bridge.log 2>&1 < /dev/null &
```
**同理**：`fsm_node` 用 `_config_path:=` 私有参数是对的（它就是读这个），但 bridge 是读 yaml 的 ——
**两者机制不同，别混用**。

#### ⚠️ 坑 L：`rostopic echo -n1 <topic>/field` 的输出含 `---` 分隔符

**症状**：脚本里 `z=$(rostopic echo -n1 /mavros/local_position/pose/pose/position/z | tail -1)`
拿到的是字符串 **`---`**，不是数值 → 判据永远不成立（`z=---` 刷屏）。

**根因**：`rostopic echo` 每输出一条消息后会打印一行 `---`，`tail -1` 正好取到它。

**修法**：
```bash
z=$(rostopic echo -n1 /mavros/local_position/pose/pose/position/z 2>/dev/null \
    | grep -vE '^---' | head -1 | tr -d '[:space:]')
```
**注意**：用 `awk '/^field:/{print $2}'` 解析整条消息（如 `/mavros/state`）不受影响 ——
所以同一段脚本里两种写法混着用很容易只错一处。**统一加 `grep -vE '^---'` 最省心**。

#### ⚠️ 坑 M：仿真没有遥控器 → 必须 auto_arm + auto_offboard

**症状**：脚本跑完飞机纹丝不动，`/mavros/state` 显示 `armed: False` / `mode: "AUTO.LOITER"`，
`/mavros/local_position/pose` 的 z ≈ 0。

**根因**：实机上 OFFBOARD 靠遥控器拨杆切（bridge 日志 `Waiting for OFFBOARD mode engagement
(manual switch required)`）；仿真里没有遥控器，而 `super_to_px4_bridge.yaml` 默认
`auto_arm: false` / `auto_offboard: false`。

**修法**：见坑 K 的 `rosparam set`。**顺序还要对**：先悬停稳定，再触发探索算法
（`armed=True` + `mode=OFFBOARD` + 离地 >0.4m + 连续 3s 高度波动 <2cm）。
起飞高度实测是 **1.0m**（源码默认 `init_goal/z`），不是 yaml 里写的 0.6 —— 判据别写死高度。

#### ⚠️ 坑 Q：**"已探索/未探索分界线"**怎么做（RViz 复刻 FUEL 论文界面）

**两个来源都能用，建议都开**（实测均已 advertise）：

**① FUEL 的 `/planning_vis/frontier` —— 论文里那条彩色格（首选，最像论文）**
- `FastExplorationFSM::visualize()`（`exploration_manager/src/fast_exploration_fsm.cpp:202`）
  **每次 visualize 都调**：
  ```cpp
  visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                            visualization_->getColor(double(i)/ed_ptr->frontiers_.size(), 0.4),
                            "frontier", i, 4);        // pub_id=4
  ```
- `pubs_[4]` = `frontier_pub_` = `/planning_vis/frontier`
  （`planning_visualization.cpp` 的 push_back 顺序：0=trajectory, 1=topo_path, 2=prediction,
   3=visib_constraint, **4=frontier**, 5=yaw, 6=viewpoints）
- **颜色由消息自带**（`getColor(i/N, α)` 按 frontier 序号着色）→ **RViz 里不要设 Color，开出来就是论文那套配色**
- ⚠️ **别被 `drawFrontier()` 误导**：那个函数只在没跑的 `topo_replan_fsm.cpp:332` 被调用；
  但同一个 FSM 家族里 `fast_exploration_fsm` 用 `drawCubes(..., "frontier", i, 4)` 画的是同一件事。
  只 grep `drawFrontier` 会得出"FUEL frontier 不发布"的**错误结论**（我犯过）。

**② rog_map 的 `/fsm_node/rog_map/frontier`（与规划器所用地图完全一致的边界）**
- 开关 `rog_map/frontier_extraction_en`（`sim_real_flight.yaml`，默认 **false** → 必须打开）；
  代码里是 `if (frontier_extraction_en && frontier_pub.getNumSubscribers() >= 1)` → **惰性、免费**
- `boxSearch(..., FRONTIER, ...)` 里的 `FRONTIER` **只是搜索模式**，真正判定在
  `prob_map.cpp:191 isFrontier(id_g)`：**该格 UNKNOWN 且 `fcnt_map_->getFreeCnt(id_g) > 0`**（邻接已知自由格）
  → 实现完整，勿因"全仓搜不到 `= FRONTIER` 赋值"而误判为空
- 颜色需自己指定（纯色点云，无 per-point color）

**实测话题清单（起临时 master 只跑 fsm_node + fuel_real.launch，无需 AirSim 即可复现）**：
```
SUPER: /fsm_node/visualization/{goal, exp_traj, committed_traj, backup_traj, yaw_traj,
                                exp_sfc, backup_sfc, frontend_path, astar_debug,
                                ciri_debug_mkr, ciri_debug_pc, replan_log_mkr, replan_log_pc}
       /fsm_node/fsm/path · /fsm_node/rog_map/{frontier, occ, unk, inf_occ, inf_unk, map_bound}
FUEL:  /planning_vis/{frontier, trajectory, viewpoints, visib_constraint, yaw, topo_path, prediction}
       /planning/position_cmd_vis · /exploration_goal · /fuel/pos_cmd_sink
```
**验证手法（很有用）**：起 `rosmaster --port 11331`（别用 11311），分别用**干净环境**跑
`fsm_node` 和 `roslaunch fuel_bridge fuel_real.launch`，然后 `rostopic list` —— 能拿到
"到底 advertise 了哪些话题"的**实测清单**，改 RViz 配置前先核对，避免写一堆不存在的话题。

**FUEL 论文 RViz 复刻：必须"只改话题、其余一字不动"**（我第一版自己加图层/改 Style/改 FrameRate 被用户否掉）

- **蓝本 = `fuel_planner/plan_manage/config/exp.rviz`**（由 `plan_manage/launch/tro20/rviz_exp.launch` 用 `args="-d .../config/exp.rviz"` 引用；`tro20` 就是论文那组探索实验）。
  另三份都不是：`plan_manage/config/traj.rviz`(bg 255;255;243, 给 topo 用)、
  `exploration_manager/config/traj.rviz`、`uav_simulator/*/config/rviz*.rviz`(bg 深灰, 是 sim/仿真器自带)
- **正确做法**：加载 exp.rviz → **只把 FUEL 的话题换成我们的话题**（其余 Global Options / 分组 /
  层级 / Style / Color / Alpha / Size / Decay / Queue / Views / Tools / Panels **全部原样保留**）→ 另存。
  `Enabled` 只有一种情况可动：**该话题在本机没有对应物时改成 False**（否则 RViz 空转）。
  ⚠️ **已关闭组里的条目一律不要碰**（如 Tracking 组关闭时其 `obj_pose_*` 在 FUEL 里仍是 true，改了树就不一致）
- **话题映射表（FUEL → 我们）**：
  ```
  /sdf_map/occupancy_inflate -> /fsm_node/rog_map/inf_occ        # 膨胀占据 (local_map, 开)
  /laser_cloud_surround      -> /fsm_node/rog_map/occ            # 已建图点云 (real_map, 开)
  /map_generator/global_cloud-> /cloud_registered                # 真值地图没有 -> 用稠密点云顶, 保住"淡灰环境底图"
  /uart_odom/out_odom        -> /mavros/local_position/odom
  /sdf_map/unknown           -> /fsm_node/rog_map/unk            # (FUEL 默认关)
  # 本机无对应 -> 关闭: /planning/travel_traj(只有 traj_server_backup 发)
  #   /odom_visualization/robot · /vins_estimator/camera_pose_visual
  #   /pcl_render_node/camera_pose · /Robot_1/pose
  ```
- **验收口径（写进 smoke_test）**：与 exp.rviz 比 —— (a) `Global Options` 逐键相等
  (bg `255; 253; 224` / `world` / `30` / `Default Light` True)；(b) 显示项条目数相等（38=38）；
  (c) 差异只允许出现在上面那张映射表里；**(d) 开启项的话题必须都实测存在**
- ⚠️ **别自作聪明映射稠密点云**：FUEL 的 `real_map` 用 `Style: Boxes size 0.2` 是因为
  `/laser_cloud_surround` 是**抽稀**的；把它指向我们的稠密 `/cloud_registered` 会变成
  "满屏方块 + 帧率崩"。FUEL 的 `simulation_map` 是 `Alpha 0.1 + Flat Squares`（便宜），
  稠密点云只适合放这一层。
- RViz 配置里 Marker 的键名是 **`Marker Topic`**（不是 `Topic`）；FUEL 所有 marker 的
  `frame_id` 就是 `world`（`planning_visualization.cpp` 5 处），`world→camera_init` TF 脚本里已补

#### ⚠️ 坑 J（**已更正**）：RViz 点云 `Decay Time` 的正确取值 —— "闪烁"与"整屏完全空白"是同一根因的两端

**RViz 的 `Decay Time` 语义：点云保留多少秒，`0` = 永久保留。**
**`Queue Size`：RViz 缓存多少条消息**（不是保留时间；`100000` 是内存炸弹，必须降到 1）。

- **原始故障（闪烁）**：`/mapping/currPoints` `Queue Size: 100000` → RViz 缓存十万条点云，
  内存/GC 抖动 → 画面闪。
- **我当时"修"错了**：把 `Decay Time` 0 → 1、`surround` 10000 → 3 —— **这治不了闪烁，反而更糟**：
  **只要 `/clock` 冻结（或点云断流）超过 1~3 秒，画面就被清空**。
  而 LockStep 仿真里 `/clock` 冻结是常态（PX4 未连上、暂停、丢帧）→ 现象就是
  **"启动 RViz 后经常性完全不显示"**。
- **正确配法**：**`Decay Time = 0`（永久保留，画面不会因时间冻结而清空）+ `Queue Size = 1`（防内存爆）**。
  既治闪烁又不空白。（若确实想要"只显示近 N 秒"，再按需设 Decay，但要知道代价。）
- 建议写进回归守卫：**启用的点云层 `Decay Time` 必须为 0**。

#### ⚠️ 坑 T：RViz 启动前必须显式 `rosparam set use_sim_time true`

RViz **在启动那一刻**读取 `use_sim_time`。`use_sim_time` 是由 `twin_bridge/launch/bridge.launch`
（`<param name="use_sim_time" value="true"/>`，在 `<node>` 外 → 设成全局参数）提供的。
**若先起 RViz、后起桥**，RViz 就固定用**墙上时钟**，而 TF/点云时间戳是**仿真时间**
（`/clock`，SteampableClock，实测仿真时间 ≈ 2026-09-22、墙上时间 ≈ 2026-09-27，**差 5 天**）
→ TF 查询全部失败 → **整屏空白且不会自愈**（只有重启 RViz 才行）。
→ 在**每次起 RViz 之前**都加 `rosparam set use_sim_time true`（两个脚本都要）。

**怎么确认 RViz 真的在用 sim time**：看 `/tmp/rviz.log` 的行首时间戳 ——
`[INFO] [1790497833.310860944, 1790497721.559509248]` **双时间戳**（前=墙上、后=仿真）
= 已用 sim time ✓；只有单个时间戳 = 还没用上 sim time ✗。
**同时那条日志还会告诉你渲染器**：`OpenGL device: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`
—— **WSLg 下 RViz 其实能拿到 GPU**（即使 `/dev/dri/*` 权限是 `crw------- root root` 且用户不在
render/video 组），所以**别急着归因"没 GPU"**：先看这行日志再下结论。

#### ⚠️ 坑 S：RViz **帧率过低 + 满屏"大量物块"** = 点云 `Style: Boxes` + WSL 没有 GPU 权限

**症状**：RViz 拖动卡顿、帧率极低；画面上每个点都变成一个方块，像一层方块壳。

**根因一（主因）：`Style: Boxes` 太贵**
- RViz 的 PointCloud2 `Boxes` 样式 = **每个点渲染一个 3D 立方体**（24 顶点 + 法线）；
  `Flat Squares` 只要 4 顶点；`Points` 只要 1 顶点 → **Boxes 比 Points 贵 ~24 倍**
- ⚠️ **不能照抄 FUEL 的 `exp.rviz`**：它给 `/laser_cloud_surround` 用 `Boxes size=0.2` 是因为
  那个话题是**抽稀过的**（几千点）；我们仿真的 `/cloud_registered` 是**稠密原始扫描**、
  `/fsm_node/rog_map/unk` 是**地图框内全部未知体素**（可达几十万点）→ 照抄就跪
- **正确配法**：稠密扫描 → `Points`(Size (Pixels)=2)；体素栅格(occ/unk/frontier) → `Flat Squares`；
  并把 `Decay Time: 0`（不累积）、`Queue Size: 1`
- **`unk`（未探索）建议默认关闭**：它最重，而且信息冗余 —— **FUEL 论文图里未探索区就是"空背景"**，
  分界线靠 frontier 层表达就够了

**根因二（更正：WSLg 下 GPU 其实是可用的，别乱归因）**
```bash
ls -l /dev/dri/          # crw------- root root   <- 看着像没权限
id -Gn                   # liyan sudo            <- 不在 render/video 组
```
**但 `/tmp/rviz.log` 实测**：`OpenGL device: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`、
`OpenGl version: 3.1 (GLSL 1.4)` → **WSLg 有自己的 GL 通路，能拿到硬件 GPU，不需要 chmod /dev/dri**。
→ 所以**帧率问题优先怀疑点云规模与 Style**，不要先去动 `/dev/dri` 权限。
（曾据此错判为"必须 chmod 666 /dev/dri + 改 wsl.conf 的 [boot] command"，实测不必要。）
**注意**：`Global Options → Frame Rate` 在软件渲染下 30 太高，**降到 10** 即可（视觉无差别）。

#### ⚠️ 坑 R：**给 WSL 传含 `&` 的脚本**必须先落盘再 base64

前台检测器会扫**整条命令（含 here-string 里的内容）**，所以内联在 PowerShell here-string 里的
bash 脚本**不能用 `&`**（后台符会被拦）。但很多脚本必须用 `setsid ... &` 起后台进程。
**可靠姿势**：
- 用 `write_file` 直接写到 **`\\wsl$\<distro>  mp\xxx.sh`**（WSL 侧路径，**不产生 Windows changed path**），
  再 `wsl.exe -d <distro> -u <USER> -- bash /tmp/xxx.sh` → 脚本里的 `&` 完全没问题 ✅
- 或先 `write_file` 到 Windows 再 base64 传（但会多一个 Windows changed path）
- ⚠️ **`setsid cmd` 不加 `&` 会在前台阻塞**（曾把整条命令卡到 550s 超时）——SETSID 必须配 `&`

#### ⚠️ 坑 P：SUPER 的 `AVOID_WAIT` 是**无限重试、永不放弃** → 目标不可达时永久死锁

**症状**（实测）：探索跑一阵后飞机悬停不动，`/planning/pos_cmd` 没数据，fsm 日志刷
```
-- [EXPTrajOpt] Ill corridor! Forced return.        ← SFC(安全飞行走廊) 退化
-- [TrajOpt] Cannot simplify sfcs.
[SUPER] OptimizationExpTrajInPolytopes for new path failed
[SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return
```
仲裁器日志则是同一个目标被反复放行：`放行目标 (8.83, -0.04, 0.79) | 原因: 保持超时 20.2s（强制放行，防死锁）`。

**死锁链**：FUEL 探到 **8.8 m 外**的 frontier（超出已建图范围）→ SFC 退化成 "Ill corridor"
→ 规划失败 → `AVOID_WAIT` 每 0.3 s 重试（**无上限、不放弃**）→ 飞机不动
→ FUEL 依据 `traj_server` 的执行反馈判定"未到达" → **反复发同一个目标** → 永久卡住。

**实机怎么做**（`task_run_scripts/scripts/task_node_nav_LowVoltage.py`）：
任务层有超时放弃 —— `navigate_to_waypoint(wp, timeout=20)`，
超时打 `导航到航点超时!` 然后 `导航到航点 {i+1} 失败，尝试下一个航点`。
**SUPER 缺的就是这一层。**

**解法**（写在 `fuel_bridge/scripts/goal_arbiter.py`，不动 SUPER/FUEL 源码）：
1. **z 夹取**：FUEL 视点会超出 FSM 区间(0.6~1.3)与 rog_map 顶棚(2.25) → 夹取后再放行
   （实测 FUEL 给过 z=2.42，是 "Ill corridor" 的直接诱因之一）
2. **卡死检测**：放行后 `stuck_timeout`(15s) 内飞机位移 < `stuck_move_eps`(0.5m)
   且离目标仍 > `arrive_radius` → 判不可达
3. **目标收缩**：沿"飞机→目标"方向把目标收回到 `shrink_radius`(3m) 内重试 → 飞机先动
   → FUEL 随之推进换新目标 → 自然脱困
4. **诚实放弃**：连续收缩 `max_shrinks`(3) 次仍无进展 → 停止放行 + 告警（不静默死锁）；
   FUEL 给出**明显不同的新目标**时计数清零
   ⚠️ 关键实现细节：用 `cur_goal_raw`（FUEL 原始候选）判"目标是否真的变了"，
   否则收缩后的点会被误判成"目标已失效"而立刻把远目标又放行回来。

**单测**：`test/test_goal_arbiter.py` 共 33 项（A 限频 / B 到达 / C 超时 / D 无odom /
E 高度夹取 / F 卡死收缩 / G 放弃 / H 复位 / I 收缩方向）。

#### ⚠️ 坑 O：**有规划轨迹但飞机不动** —— bridge 的 RC 失效保护（仿真无遥控器）

**症状**：`/planning/pos_cmd` 有数据、fsm_node 报 `GenerateExpTrajectory SUCCESS`，
但飞机原地悬停不动；`/tmp/super_bridge.log` 里反复出现
```
[POS_HOLD] x=-0.08, y=0.03, z=0.45, yaw=16.02 deg
[WARN] RC signal lost for 1790085011.4 s! Position hold.
```

**根因**（`super_to_px4_bridge.cpp`）：
```cpp
181: private_nh.param<bool>("failsafe_en", failsafe_en, true);           // 默认 true
183: private_nh.param<double>("rc_loss_timeout", rc_loss_timeout, 1.0);  // 1 秒
208: rc_sub = nh.subscribe("/mavros/rc/in", 10, &BridgeController::rcCallback, this);
677:  if (failsafe_en) {
680:      if ((now - last_rc_time).toSec() > rc_loss_timeout) {
681:          ROS_WARN_THROTTLE(2.0, "RC signal lost for %.1f s! Position hold.", ...);
```
**仿真里根本没有遥控器** → `/mavros/rc/in` 永远无数据 → `last_rc_time` 永不更新
→ 1 秒后判定 RC 丢失 → 强制 Position hold → **规划轨迹一概不跟**。
同一个开关还带 `battery_threshold: 20%` 的电池失效保护，SITL 里也容易误触发。

**修法**（rosparam 覆盖，不动实机的 yaml）：
```bash
rosparam set /super_to_px4_bridge/failsafe_en false
```

**排查口诀**：**飞机不动先看桥日志有没有 `POS_HOLD` / `RC signal lost`**，
而不是去查规划器 —— 规划器 `GenerateExpTrajectory SUCCESS` 说明它没问题，
是**执行端**把指令丢了。

#### ⚠️ 坑 N：**工作区环境混用会让 FUEL 的节点起不来**（roslaunch 静默卡死）

**症状**：`roslaunch fuel_bridge fuel_real.launch` 看起来启动了，但
- `/tmp/fuel_real_super.log` **0 字节**
- roslaunch 自己的日志里只有 `process[fuel_input_bridge-1]: started with pid`，
  `exploration_node` / `traj_server` / `waypoint_generator` **只打印了 env 就没下文**
- 100 秒后 roslaunch 直接 `killing os process` 退出
- `rostopic info /exploration_goal` 只有仲裁器订阅，**没有发布者**

**根因**：为 SUPER 拼的 `ROS_PACKAGE_PATH` / `LD_LIBRARY_PATH` 把 `super_sim_ws` 排在最前，
FUEL 的三个节点在这种环境下起不来。

**A/B 实测铁证（2026-09-22）**：
| 环境 | LD_LIBRARY_PATH 首位 | 14s 后存活的 FUEL 节点 |
|---|---|---|
| 只 source `/opt/ros` + `fuel_ws` | `fuel_ws/devel/lib` | **4/4**（expl/traj/wp/in 全起） |
| 脚本的 source 顺序（含 `super_sim_ws`） | `super_sim_ws/devel/lib` | **1/4**（只剩 fuel_input_bridge） |

**修法**：FUEL 用**子 shell 干净环境**启动，不继承为 SUPER 拼的变量
```bash
(
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
  export ROS_IP="${ROS_IP:-127.0.0.1}"
  source /opt/ros/noetic/setup.bash
  source ~/fuel_ws/devel/setup.bash          # ← 只这两层
  setsid roslaunch fuel_bridge fuel_real.launch enable_ctrl:=false \
      fuel_cmd_topic:=/fuel/pos_cmd_sink > /tmp/fuel_real_super.log 2>&1 < /dev/null &
)
```
> **排查口诀**：一个脚本要同时拉起两个工作区时，**每个工作区各自用子 shell 起**，
> 别指望"先 source A 再 source B"能共存 —— ROS 的 `setup.bash` 会
> **覆写** `LD_LIBRARY_PATH`（不是追加），后 source 的把前面的挤掉。

#### ⚠️ 坑 I：**14610 是 UDP 端口，必须用 `ss -tuln` 查（`-tln` 永远查不到）**

**症状**（实测 2026-09-22）：脚本报
```
[X] 14610 仍未监听 → MAVROS 无法连 FCU → 必然 'No odom'
PX4 的 pxh 画面:
  pxh> mavlink start -u 14610 -o 14611
  INFO  [mavlink] mode: Normal, data rate: 2880 B/s on udp port 14610 remote port 14611   ← 其实成功了！
  pxh> mavlink start -u 14610 -o 14611                                                   ← 我的重试
  ERROR [mavlink] Maximum MAVLink instance count of 4 reached.                           ← 实例数满
```
**根因**：PX4 的 MAVLink 是 **UDP**（日志自己写了 `on udp port`），
而我用 **`ss -tln`（TCP LISTEN）** 去判断 → **UDP socket 根本不出现** → 永远判「未开」
→ 触发无意义的重试 → 第 2 次因实例数上限失败，看起来像"起不来"。

**铁证**（起一个真 UDP listener 后对比）：
```bash
ss -tuln | grep 14610   # → udp  UNCONN  0  0  0.0.0.0:14610  0.0.0.0:*      ← 有！
ss -tln  | grep 14610   # → (空)                                              ← 查不到
```
> 注意状态是 **`UNCONN`** 而不是 `LISTEN` —— UDP 没有 LISTEN 状态，别用 `grep LISTEN` 过滤。

**修复**：写一个多口径判据 + 用 PX4 自己的日志兜底，并在检测到已开时**不再重发**：
```bash
port_open(){
  ss -tuln 2>/dev/null | grep -q ":$1" && return 0
  ss -uln  2>/dev/null | grep -q ":$1" && return 0
  ss -tln  2>/dev/null | grep -q ":$1" && return 0
  # 兜底：PX4 会打印 "on udp port <p>"，直接问 tmux 画面
  tmux capture-pane -p -t px4 2>/dev/null | tail -40 | grep -q "port $1" && return 0
  return 1
}
```
**教训**：**判「端口是否开着」必须先确认协议** —— PX4 里 4560 是 TCP（`ss -tn` 对），
14610/14611 是 UDP（要 `ss -tuln`）。混用会得到 100% 错误结论。

> 附带的 `pgrep` 陷阱：`pgrep -f <pattern>` 会匹配**父 shell 的命令行**（只排除 pgrep 自己）。
> 所以 `bash -c "pgrep -f bin/px4"` 这种写法**会匹配到自己**，永远返回"在跑"。
> 在 `.sh` 脚本里安全（`bash xxx.sh` 的命令行不含该串），但**在 `bash -c "..."` 交互诊断里必须避开**——
> 用 `pgrep -af` 打印出来核对，或把模式拆成 `pgrep -f "bin/px""4"` 之类。

#### ⚠️ 坑 L：RViz 显示问题 —— **先分清「完全没有」还是「闪烁」，两者根因毫不重叠**

> ★ **排查纪律（2026-09-22 教训，代价半场会话）**：用户说"RViz 点云显示有问题"时，
> 我按"**时隐时现**"去查 `Decay Time` / `Queue Size`（那确实是个隐患），查了半天才发现
> 用户的真实症状是 **完全没有任何点云** —— 两者**根因、查法、修法毫无重叠**。
> **开口第一句必须问清**：「是一帧都没有，还是有一阵没一阵？」「地图/轨迹有吗，还是只有点云没有？」

| 症状 | 根因 | 去哪看 |
|---|---|---|
| **完全看不到点云** | ① 订阅话题是**实机残留**（仿真里不存在）② Fixed Frame 与点云 frame 之间**没有 TF 链** | 坑 M |
| **有，但闪烁/断续** | `Decay Time: 0`（每帧独立不累积）+ `Queue Size` 过大 + 软渲染跟不上 | L-1 |
| **什么都没有（含地图/轨迹）** | 仿真脚本**绕过 `real_flight.launch`**，压根没起 RViz | 坑 N |

##### L-1 闪烁/断续：`Decay Time: 0` + `Queue Size: 100000`

**症状**：RViz 里点云流闪烁、时有时无（SLAM 与规划本身正常，纯显示层）。

**先排除两个常见误判**（本项目实测都不是）：
```bash
grep -n "Fixed Frame:" ~/twin_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz   # → camera_init ✓

```
**真因**（用 YAML 解析列出所有 PointCloud2 项即现形）:
```
/mapping/currPoints   topic=/cloud_registered  Decay Time: 0       Queue Size: 100000   ← 元凶
/mapping/surround     topic=/cloud_registered  Decay Time: 10000   Queue Size: 1        ← 这个是对的
```
- **`Decay Time: 0`** = 每帧独立显示、不含历史 → **任何一帧丢失/渲染延迟 → 画面瞬间空白**
- **`Queue Size: 100000`** = 积压十万条点云消息 → RViz 内存涨、渲染越来越落后
- 所以稳定显示的是 `surround`（有累积），**闪的是 `currPoints`**

**修复（RViz 配置文本级改，改完必须重启 RViz）**：
```
/mapping/currPoints:  Decay Time 0 → 1      Queue Size 100000 → 1
/mapping/surround:    Decay Time 10000 → 3  （10 秒累积会让点云堆太多）
Global Options:       Frame Rate 30 → 15    （点云本身才 10 Hz，30 帧纯浪费）
```
> 排查 RViz 配置要用 YAML 解析而不是 grep —— `Fixed Frame` 嵌在
> `Visualization Manager.Global Options` 下，grep 只会匹配到 Grid 的 `<Fixed Frame>` 引用。
> 遍历脚本：`d['Visualization Manager']['Displays']` 递归取 `Class == 'rviz/PointCloud2'` 且 `Enabled` 的项。

##### 坑 M：**点云一帧都没有 = 实机话题残留 + 缺 `world↔camera_init` TF（两个叠加，都得修）**

仿真里默认起的是**实机那份** `task_run_scripts/rviz/top_down.rviz`。它有**两个**仿真下必然空白的原因：

1. **话题是实机残留**：点云项 `/Sensing/GlobalCloud` 订阅 **`/odin1/cloud_slam`**（实机 odin 的话题），
   仿真里该话题**根本不存在** → 一帧都不会有。
2. **即使改了话题，还是空白**：`/cloud_registered` 的 frame 是 **`camera_init`**，而该配置的
   **Fixed Frame = `world`**，两者之间**没有 TF 链** → RViz 无法变换，显示项会报
   `Fixed Frame [world] does not exist` / 静默空白。

**TF 链现状（查过，别想当然）**：
```
real_flight.launch        → 只发 static_transform_publisher "0 0 0 0 0 0 map world"（地图↔世界）
fuel_input_bridge.py      → "把 frame_id 从 camera_init 统一改写成 world" —— 但只改【它自己发的】消息，
                             不改 FAST-LIVO2 的原始 /cloud_registered
twin_bridge / 各 launch   → 都不发 world↔camera_init
⇒ 结论：camera_init 是【孤儿帧】，仿真里没人给它接上 world
```

**修法（两条都要做，缺一个就还是空白）**：
```bash

setsid rosrun tf static_transform_publisher 0 0 0 0 0 0 world camera_init 100 >/dev/null 2>&1 &
```
> ⚠️ 若日后把 `fix_map_origin` 改回 `[0,0,1.5]`（雷达安装高），这条 TF 的 z 必须同步改成 1.5，否则点云整体错位。

**判据**：`rostopic echo -n1 /cloud_registered/header/frame_id` → `camera_init`；
`rosrun tf tf_echo world camera_init` **有值**（之前是 `does not exist`）；RViz 里点云与 rog_map 重叠显示。

##### 坑 N：**仿真里要「像实机一样有 RViz 看地图+轨迹」= 脚本绕过了 launch**

**现象**：实机启动 SUPER 自动弹 RViz（`real_flight.launch` 里有
`<node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rviz_config)"/>`，默认 `top_down.rviz`），
**仿真里跑 `super_explore_all.sh` 什么都没有**。

**根因**：仿真脚本为了少一层 roslaunch 依赖，**直接调二进制**：
```bash
setsid "$FSM_BIN" _config_path:="$SUPER_CFG" > /tmp/super_fsm.log 2>&1 < /dev/null &
setsid "$BRIDGE_BIN" ...
```
→ 绕过了 launch ⇒ **launch 里那个 rviz 节点从来不会起**。（脚本的 4/7 检查只看 `pgrep fsm_node`，所以不报错。）

**修法（已落地在 `super_explore_all.sh` 步骤 4a/4b）**：
1. **新建仿真版 rviz**（`task_run_scripts/rviz/sim_top_down.rviz`，从实机 `top_down.rviz` 派生）——
   生成命令与字段映射见 `references/sim-rviz-and-tf.md`，
   可重跑的生成脚本 `scripts/make_sim_rviz.py`。
2. **脚本里补 TF**（坑 M 那条）+ **起 rviz**：
   ```bash
   pkill -x rviz 2>/dev/null      # ★ 先关掉 FAST-LIVO2 那个，避免两个 RViz 抢渲染，也避免同名顶替（坑51）
   RVIZ_CFG="$(rospack find task_run_scripts)/rviz/sim_top_down.rviz"
   setsid rviz -d "$RVIZ_CFG" > /tmp/rviz_super.log 2>&1 < /dev/null &
   ```
3. **加 `--no-rviz` 开关**（只要无头跑时用），并让 `--print-params` 输出带 `rviz=$((1-NO_RVIZ))` 便于自检。

**实机 → 仿真的话题映射表（派生 sim 版时照这张表改）**：

| 显示项 | 实机话题 | 仿真话题 | frame |
|---|---|---|---|
| 点云 | `/odin1/cloud_slam` | **`/cloud_registered`** | `camera_init` |
| rog_map 占据 | `/fsm_node/rog_map/occ` | 同（不变） | `world` |
| rog_map 未知 | `/fsm_node/rog_map/unk` | 同 | `world` |
| 膨胀层 | `/fsm_node/rog_map/inf_occ` | 同 | `world` |
| 里程计 | `/mavros/local_position/odom` | 同（仿真也有） | — |
| 规划轨迹 | `/fsm_node/visualization/{exp_traj,backup_traj,yaw_traj}` + `/fsm_node/fsm/path` | 同 | `world` |

**判据**：RViz `Displays` 面板各项 Status 全 OK；`rostopic hz /fsm_node/rog_map/occ` 有数据
（旧 skill 就记过「RViz 白屏先查 occ 有无数据」）；点云 + 地图 + 轨迹在**同一个窗口**里叠加显示。
**改 rviz 配置后必须重启 RViz 才生效**（RViz 启动时读一次配置）。

#### ⚠️ 坑 O：**脚本跑完但飞机根本没起飞** —— 三个叠加原因（2026-09-22 实测）

**症状**：`super_explore_all.sh` 报「完成」、7 步全过、防抢控制也 OK，但飞机停在地面不动。
铁证三连：
```
rostopic echo -n1 /mavros/state   →  armed: False      ← 没解锁
                                     mode: "AUTO.LOITER" ← 不是 OFFBOARD
rostopic echo -n1 /mavros/local_position/pose/pose/position/z  →  -0.045   ← 在地面
```

##### 原因 1：bridge 的 `auto_arm` / `auto_offboard` 默认关闭

`task_run_scripts/config/super_to_px4_bridge.yaml` 里两个开关都是 **`false`**（源码默认也是 false）：
```yaml
auto_arm: false
auto_offboard: false
```
**实机上是靠遥控器手动拨杆切 OFFBOARD**（bridge 日志会停在
`Waiting for OFFBOARD mode engagement (manual switch required)...`）——
**仿真里没有遥控器 ⇒ 永远不解锁、永远不进 OFFBOARD**。

解法：这两项**不改实机配置文件**，用私有参数/rosparam 覆盖（见原因 2 的写法）。

##### 原因 2（更隐蔽）：**直接调二进制会绕过 launch 里的 `<rosparam command="load">`**

`real_flight.launch` 里是这样的：
```xml
<node pkg="super_planner" type="super_to_px4_bridge" name="super_to_px4_bridge" output="screen">
  <rosparam command="load" file="$(find task_run_scripts)/config/super_to_px4_bridge.yaml"/>   ← 关键
</node>
```
而 `super_explore_all.sh`（坑 N）为了少一层 roslaunch **直接调二进制** ⇒ **这行 load 从未执行**
⇒ `max_velocity_xy` / `use_yaw_angle_control` / `land_speed` / `failsafe` / `init_goal` 等**实机调优参数全部丢失**，
只剩源码默认值。（只传 `_auto_arm:=true` 是**不够的**——飞行安全类参数也一起丢了。）

**正确写法（`super_explore_all.sh` 步骤 4 已落地）**：
```bash
BRIDGE_YAML=~/super_sim_ws/src/task_run_scripts/config/super_to_px4_bridge.yaml
rosparam load "$BRIDGE_YAML" /super_to_px4_bridge   # ① 先把实机调优参数 load 进私有 ns
rosparam set /super_to_px4_bridge/auto_arm true     # ② 再覆盖两个开关（仿真无遥控器）
rosparam set /super_to_px4_bridge/auto_offboard true
setsid "$BRIDGE_BIN" > /tmp/super_bridge.log 2>&1 < /dev/null &
```
> **通则**：凡是「直接调二进制 + 有配套 yaml」的节点，都必须**手动 `rosparam load` 那份 yaml**
> 再 `rosparam set` 覆盖少数需要改的项 —— 否则静默丢失全部调参。
> **实测验证**：bridge 日志出现 `Auto arm: true` / `Auto OFFBOARD: true` /
> `Publishing initial setpoints for takeoff to 1.00 meters...` 三行即证明参数已生效。

##### 原因 3：起飞高度是 **代码默认的 1.0m**，不是 yaml 里的 `init_goal.z: 0.6`

因为原因 2（yaml 没被加载），实际用的是源码默认值 ——
`private_nh.param<double>("init_goal/z", ..., 1.0)`（super_to_px4_bridge.cpp:195）。
**任何写死高度阈值的判据都会失效**（我第一版写 `z∈[0.45,0.90]` 就永远等不到）。

##### ★ 用户工作流偏好：**先悬停稳定，再跑探索算法**

用户明确要求（原话：「我建议是先悬停再去跑算法」）——**这是默认该有的顺序，不是可选项**。
原脚本第 7 步直接触发 FUEL，而飞机那时可能还没起飞 ⇒ 算法白跑。
`super_explore_all.sh` 新增 **`6.5)` 步骤**（在 `WITH_SUPER=1` 时执行，位于步骤 6 仲裁器之后、步骤 7 触发之前）：

```bash
ok_hover=0; armed=""; mode=""; z=""; zprev=""; stable=0
for i in $(seq 1 45); do
  st=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null)
  armed=$(echo "$st" | awk '/^armed:/{print $2}')
  mode=$(echo "$st"  | awk '/^mode:/{gsub(/"/,"",$2); print $2}')
  z=$(timeout 4 rostopic echo -n1 /mavros/local_position/pose/pose/position/z 2>/dev/null | tail -1)
  if [ "$armed" = "True" ] && [ "$mode" = "OFFBOARD" ] && [ -n "$z" ]; then
    # ★ 自适应判据：不写死高度，看「离地 + 连续稳定」
    if [ -n "$zprev" ]; then
      if awk -v a="$z" -v b="$zprev" 'BEGIN{exit !(a>0.4 && ((a-b<0.02 && b-a<0.02)))}'; then
        stable=$((stable + 1)); else stable=0; fi
    fi
    zprev="$z"
    [ $stable -ge 3 ] && { ok_hover=1; break; }      # 连续 3s 波动 <2cm = 悬停稳定
  fi
  sleep 1
done
```
- **判据必须自适应（`离地>0.4m` + `连续 3 次相邻波动<2cm`），不要写死目标高度** —— 原因 3 会让写死值失效。
- 稳定性用「相邻两次差值」而不是「与目标值的误差」，因为目标高度可能来自代码默认而非 yaml。
- 45s 未达 → 打印当前 `armed/mode/z` + 指向 `/tmp/super_bridge.log`，并列出两种常见原因
  （未解锁=预检没过；未进 OFFBOARD=`commander mode offboard` 被拒），**仍继续触发**但明确告知飞机可能没起。
- 验证时**用脚本里逐字相同的 `awk` 表达式**去测边界（见 `project-verification-entrypoint`）——
  我第一次自测写成 `got = 0 if rc == 0 else 1` 后按 `if got` 判 stable，**语义反了，4/4 全报 WRONG，
  差点误判脚本有 bug**。正解：`stable = (rc == 0)`。

细节（参数清单、实测日志、`6.5)` 完整代码）见 `references/sim-takeoff-arm-and-hover.md`。

#### ⚠️ 坑 P：**14610 的 `mavlink start` 默认 Normal 模式会把 odom 掐到 ~1 Hz**（2026-09-22 定量实测）

**症状链**：飞机**能起飞悬停，但一动不动**。
```
/mavros/local_position/odom     0.98 Hz     ← 该 ≥50Hz
/mavros/imu/data                15 Hz       ← 该 100Hz+
/planning/pos_cmd               无数据      ← SUPER 出不了轨迹（fsm_node 是唯一发布者）
/waypoint_goal                  无数据
SUPER fsm 刷屏: [BackOpt] Opt failed, Omg or thr or Pos violation
                [MINCO] TrajOpt failed, (unknown)          ← 实测刷了 4975 次
                generateBackupTrajectory return [OPT_FAILED] → AVOID_WAIT
飞机位置: z 在 0.445~0.458 之间抖 = 只是 POS_HOLD 噪声，没在飞
```

**根因**：`mavlink start -u 14610 -o 14611` 不带模式参数 ⇒ 默认 **`Normal`**（给数传的省流模式）。
`mavlink status` 直接给出答案：
```
instance #3:  mode: Normal    tx rate max: 2880 B/s    UDP (14610, remote port: 14611)   ← 我们的
instance #2:  mode: Onboard   tx rate max: 4000 B/s
```
`mavlink start` 成功时那句 `INFO [mavlink] mode: Normal, data rate: 2880 B/s` 就是**全部线索**
—— 2880 B/s 要养活 heartbeat/attitude/local_position/imu 等一堆流，
MAVROS 依赖的 `LOCAL_POSITION_NED` 只能拿到 **~1 Hz** ⇒ SUPER 每轮 replan 都在用陈旧状态
⇒ 备份轨迹优化必然失败。（`/clock` 本身有 189 Hz，**不是仿真慢**，别往那个方向查。）

**修法（运行时生效，不必重启实例）**：
```bash

mavlink stream -u 14610 -s LOCAL_POSITION_NED -r 50
mavlink stream -u 14610 -s ATTITUDE            -r 50
mavlink stream -u 14610 -s HIGHRES_IMU         -r 20

for s in ODOMETRY ATTITUDE_QUATERNION RC_CHANNELS SERVO_OUTPUT_RAW VFR_HUD \
         GLOBAL_POSITION_INT SYSTEM_TIME TIMESYNC STATUSTEXT; do
  mavlink stream -u 14610 -s $s -r 0
done
```
实测：**odom 0.98 → 23 Hz（23 倍）**，`imu/data` 15 → 23 Hz。
语法依据 = `mavlink stream` 的 usage 明写 `[-u <val>] Select Mavlink instance via local Network Port`；
流名写错会回 `WARN [mavlink] stream XXX not found`，好识别。

**★ 反直觉坑：加流反而更慢。** 只设 NED+ATTITUDE → 22.6 Hz；再补
ODOMETRY/ATTITUDE_QUATERNION/HIGHRES_IMU 三条 → **掉到 5.6 Hz**。
因为 `tx rate max` 是**总预算**，超了 PX4 自动下调 `tx rate mult`（实测 0.986），**所有流一起被压**。
⇒ **调流率必须「一手加关键流、一手关无关流」**，不是只加不减。用
`mavlink status` 的 `tx` / `tx rate mult` / `tx rate max` 三值判断是否超预算。

**已固化进 `twin_livo_fast.sh`**：14610 一就绪就自动设上述流率并关停无关流（不再手动）。
> 该实例是我们自己 `tmux send-keys` 起的 runtime 实例，**没有 `MAV_x_MODE` 参数可控**，只能后调。
> 也试过 `mavlink start ... -m onboard`：**旧实例停不掉**（`mavlink stop -u <port>` 语法不对会打 usage，
> 只剩 `stop-all` 全杀，风险大）⇒ 放弃，走 `mavlink stream` 路线。

**判据速查：追问「飞机不动」时按这个顺序逐层卡**：
| # | 检查 | 正常 | 异常指向 |
|---|---|---|---|
| 1 | `rostopic echo -n1 /mavros/state` | `armed: True` + `mode: "OFFBOARD"` | 未解锁/未切模式 → 坑 O |
| 2 | `rostopic hz /mavros/local_position/odom` | **≥20 Hz** | ~1 Hz = MAVLink 流率被掐 → **本坑** |
| 3 | `rostopic hz /planning/pos_cmd` | ~100 Hz | 无数据 = SUPER 没出轨迹 → 下一层 |
| 4 | `tail /tmp/super_fsm.log` | `FOLLOW_TRAJ` | `TrajOpt failed` / `AVOID_WAIT` → 地图/目标 |
| 5 | `rostopic echo -n1 /waypoint_goal` | 目标 z 在地图垂直范围内 | z 超 `±virtual_ceil`（量化后 ±2.25）→ 安全走廊退化 → 备份轨迹必失败 |

> **附带判据：判断「整栈同时死」是人为停的还是自己崩的。** 本次实测一次全灭：所有 `/tmp/*.log` 的
> mtime 都落在 40 秒窗口内（21:35:30~21:36:07），`rosmaster`/`mavros`/`px4`/`fastlivo`/`rviz`/`fsm_node` 全 0，
> **但手工加的 `static_transform_publisher` 独独活着**——`super_explore_stop.sh --all` 不认识节点 ⇒ 特征吻合。
> 三条判据：① 日志 mtime 是否**彼此接近**（自然崩溃不会这么齐）② 是否有「本不该被停的进程」幸存
> ③ `free -h` + `dmesg | grep -i oom` 排除 OOM（本次内存 7.5G 空闲、swap 未动、无 OOM 记录 ⇒ 先排除 OOM 再怀疑人为）。

#### ⚠️ 坑 Q：**「最后完全规划不出来」= 安全走廊退化 + 无人放弃的死锁**（2026-09-22）

飞起来之后会遇到的两个后续问题（用户原话：「飞机可以正常飞行，但是我发现还是存在飞机与障碍物撞击，
以及最后无法规划的场景，可以借鉴一下实机的悬停重试机制」）。**这两件事根因完全不同，别混着查。**

##### Q-1 「最后无法规划」——死锁三连

**日志铁证（一眼定性）**：
```
-- [Fsm] ReplanOnce succeed.
-- [EXPTrajOpt] Ill corridor! Forced return.            ← ⭐ 安全走廊退化的标志
-- [TrajOpt] Cannot simplify sfcs.
[SUPER] OptimizationExpTrajInPolytopes for new path failed
[SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return
```
配 `[ExpOpt] Opt failed, Omg or thr or Pos violation.` + `[MINCO] TrajOpt failed, (unknown).`（实测 52+ 次）。

**死锁三连（谁也别指望谁）**：
| 环节 | 行为 | 问题 |
|---|---|---|
| FUEL | 探到 **8.8 m 外**的 frontier 当目标，**反复发同一个**（`Next view: 8.82579 -0.0399559 0.787528` 连续重复） | 目标**超出已建图范围** → 走廊无据可依 |
| SUPER | `GenerateExpTrajectory failed` → `AVOID_WAIT`，**每 0.3s 重试、无限、永不放弃** | 目标不可达时**永久卡住** |
| 仲裁器 | `>>> 放行目标 (8.83,-0.04,0.79) \| 原因: 保持超时 20.2s（强制放行，防死锁）` —— 放行的**还是同一个目标** | "防死锁"实际没解开死锁 |

**FUEL 为什么会反复发同一个目标（关键机制，别误判成 FUEL 的 bug）**：
FUEL 的输出被 remap 到 `/fuel/pos_cmd_sink`（防抢控制），但**它的 FSM 仍靠 `traj_server` 的执行反馈判"到达"**，
而 `traj_server` 拿 `/odom_world` 的真实位置判轨迹终点 —— **飞机没动 ⇒ 永远不到达 ⇒ 永远发同一个目标**。
⇒ **"改道"只切了输出，没切断它的到达判定**，这是设计上必须接受的耦合。

##### Q-2 实机的机制：**SUPER 那层 SUPER 已经有了，缺的是「任务级超时放弃」**

用户的直觉是对的，但要说清**实机是两层，SUPER 只复刻了第一层**：

**第一层（SUPER 已有，`super_core/fsm.cpp:167-193`）**——主动悬停 + 限频重试：
```cpp
// 轨迹生成失败 → 进入"停车避让"状态(主动悬停+限频重试, 不再高频空转)
cout << " -- [Fsm] PlanFromRest failed, switch to AVOID_WAIT (park & wait).";
publishHoldPose();          // 主动发悬停指令给 bridge（不是等超时）
avoid_wait_t_ = 0.0;
ChangeState("MainFsmCallback", AVOID_WAIT);

case AVOID_WAIT: {
    avoid_wait_t_ += 0.1;   // FSM 主循环 ~10Hz
    publishHoldPose();      // 持续发悬停(保持位置)
    if (avoid_wait_t_ >= 0.3) {        // 每 0.3s 重试一次
        int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (retcode == SUCCESS || retcode == FINISH) {
            publishPolyTraj();
            cout << " -- [Fsm] AVOID_WAIT: path recovered, resume flight.";
            ChangeState("MainFsmCallback", FOLLOW_TRAJ);
        }
        // else → 无限重试，永不放弃   ← ★ 缺口就在这一支
    }
}
```
**第二层（实机有、SUPER 没有，在任务节点里）**——`task_run_scripts/scripts/task_node_nav_LowVoltage.py`：
```python
90:  def navigate_to_waypoint(wp, timeout=20):
121:      while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time) < timeout:
139:      rospy.logwarn(f"导航到航点超时! 最终距离: {distance:.2f}米, ...")
236:      rospy.logwarn(f"导航到航点 {i+1} 失败，尝试下一个航点")     # ★ 超时 → 放弃 → 换下一个
```
⇒ **实机是"每个航点最多等 20s，超时就放弃换下一个"，所以永远不会卡死。**

##### Q-3 加固落点：**改 `goal_arbiter.py`（我们自己的组件），不碰 SUPER/FUEL 源码**

仲裁器正好在"FUEL 目标 → SUPER"的咽喉位置，是补第二层最合适的地方：

| 加什么 | 做法 | 解决 |
|---|---|---|
| **z 夹取** | 目标 z 夹到 FSM 允许的 `[goal_z_min, goal_z_max]`（实机 0.6~1.3），且 ≤ `virtual_ceil` 量化后的 **±2.25** | 实测见过 FUEL 发 `z=2.416`（超顶棚）→ 走廊必退化 |
| **卡死检测** | 监视 `/planning/pos_cmd`：目标放行后 **N 秒（建议 15s）内 SUPER 没出轨迹** → 判该目标不可达 | 替代 SUPER 的无限重试 |
| **目标收缩** | 把不可达目标沿「飞机→目标」方向**收回 R 米内**（建议 3m）再放行 | 让飞机先动 ⇒ FUEL 认为推进了 ⇒ 换新目标，**解开死锁** |
| **放弃计数** | 连续收缩 K 次仍不行 → 日志告警并停止放行 | 可观测，不静默死锁 |

> **实现前先按用户偏好讲方案**（用户明确要求「复杂改动实施前先讲方案再动手」）——
> 本次先在对话框给出方案表、等确认后才动手 ✅。

##### Q-3b **已落地**（2026-09-22，`goal_arbiter.py` + `test_goal_arbiter.py`）

用户确认后已实现，四件事全部进代码，**单测 33/33**（新增 E/F/G/H/I 五组）：

| 实现点 | 落法 | 单测 |
|---|---|---|
| z 夹取 | `clamp_z()`：`max(z_min, min(z_max, ceiling, z))` | E1/E1b/E2/E3 |
| 卡死判定 | `is_stuck(dt)`：`dt≥stuck_timeout` 且 `位移<stuck_move_eps` 且 `离目标>arrive_radius` | F1b/F2 |
| 目标收缩 | `shrink_toward(pos, goal, radius)` 沿「飞机→目标」等比收回 | I1/I2/I3 |
| 诚实放弃 | `shrink_count ≥ max_shrinks` → `gave_up=True` + 停止放行 + 告警 | G1–G4 |
| 新目标复位 | 目标真变了 ⇒ `shrink_count=0; gave_up=False` | H1–H3 |

**★ 实现里最容易漏的一处：用 `cur_goal_raw`（FUEL 原始候选）判「目标是否真的变了」。**
收缩后 `cur_goal` 是缩过的点，若拿它去算 `dist(candidate, cur_goal)`，
下一帧就会认为「目标变化 > goal_shift」→ 走 `目标已失效` 分支 → **把远目标立刻放行回来**，
收缩白做。故必须**分别存**：`cur_goal`（真正下发的点）与 `cur_goal_raw`（对应那次 FUEL 原始候选）。
（单测 F5 专门盯这条。）

**分支顺序也是要点**：`is_stuck` 必须排在 `dt >= max_hold 强制放行` **之前** —— 否则超时分支会抢先
把同一个不可达目标再发一遍，死锁照旧。

**新参数**（`super_explore_all.sh` 步骤 6 已成对显式传入，可用环境变量覆盖便于调参）：
`_z_min`(0.6) `_z_max`(1.3) `_ceiling`(2.2) `_stuck_timeout`(15.0) `_stuck_move_eps`(0.5)
`_shrink_radius`(3.0) `_max_shrinks`(3)。

**判据（下次跑起来看这行才算生效）**：
```
[arbiter] >>> 放行目标 (3.00, 0.00, 1.00) | 原因: 目标不可达（15.0s 内位移 0.02m、仍差 8.80m）→ 收缩到 3.0m 内重试（第 1/3 次）
```
若仍看到同一目标被 `保持超时 20.2s（强制放行，防死锁）` 反复放行，说明收缩分支没生效，回查参数是否真的传进节点
（`rosparam get /goal_arbiter/stuck_timeout`）。

##### Q-4 撞墙：**先做判别实验，别急着调参**

**实机 vs 仿真防撞参数对比（同一张表照抄，本会话实测）**：

| 参数 | 实机 `odin_real_flight.yaml` | 仿真 `sim_real_flight.yaml` | 判定 |
|---|---|---|---|
| `robot_r` | 0.35 | 0.35 | 相同 ✓ |
| `inflation_resolution` | 0.2 | 0.2 | 相同 ✓ |
| `inflation_step` | 2 | 2 | 相同 ✓ |
| `resolution` | 0.15 | 0.15 | 相同 ✓ |
| **`traj_opt/boundary/max_vel`** | **0.3** | ~~1.0~~ → **0.3** | ⚠️ 原快 **3.3×**，已降速（见下） |
| **`traj_opt/boundary/max_acc`** | **0.4** | ~~1.0~~ → **0.4** | ⚠️ 原快 **2.5×**，已降速（见下） |

**安全裕度完全相同、速度/加速度却高 2.5~3.3 倍** —— 这是撞墙的**物理候选**，
但**不能就此定论**：也可能是**地图与真实几何错位**（FAST-LIVO2 点云在仿真的外参/FOV 偏差）
—— 那样调参只能缓解、治不了根。

**判别实验（一步二分）**：把 `max_vel/max_acc` 降到实机同值（0.3 / 0.4）跑一次
- 明显不撞了 → 是速度问题 ⇒ 再逐步往上找拐点
- 照撞 → 是地图/定位错位 ⇒ 走 PCD 与 AirSim 场景**叠加对比**查错位

**✅ 该实验已执行（2026-09-22，用户拍板）**：`sim_real_flight.yaml` 的 `traj_opt/boundary` 两行
`max_vel: 1.0 → 0.3`、`max_acc: 1.0 → 0.4`（第 61-62 行，**手工编辑**），
备份 `sim_real_flight.yaml.bak_before_slowdown_20260922_221349`。**结论待用户跑完回填** ——
别在拿到结果前把"降速就好了"写成结论。
> **副作用要提前告诉用户**：整轮探索慢约 **3 倍**（0.3 vs 1.0 m/s），原 10 分钟的跑要 30 分钟。
> **且 FUEL 侧 `max_vel` 仍是 0.8**（没动）⇒ FUEL 按 0.8 规划、SUPER 按 0.3 执行，
> FUEL 的 `traj_server` 会看到「跟不上」而更频繁重规划。**若要动就等观察到目标抖动再动** ——
> 一次只改一个变量，否则无法归因（与 Q-4 通则同源）。

**★ 手工编辑 yaml 必须加回归守卫**：这文件是手改的，改坏一个缩进整个 FSM 起不来。
已在 canonical `smoke_test.sh` 的 G 段加一条：
```bash
SIM_CFG=$HOME/super_sim_ws/src/task_run_scripts/config/sim_real_flight.yaml
python3 -c "
import yaml,sys
d=yaml.safe_load(open(sys.argv[1])); b=d['traj_opt']['boundary']
def find(x,k):
    if isinstance(x,dict):
        for kk,vv in x.items():
            if kk==k: return vv
            r=find(vv,k)
            if r is not None: return r
    return None
print('  PASS sim_real_flight.yaml 可解析 (max_vel=%s max_acc=%s robot_r=%s)' % (b['max_vel'], b['max_acc'], find(d,'robot_r')))
" "$SIM_CFG" || no "sim_real_flight.yaml 解析失败（手工改动改坏了？）"
```
> 注意 `robot_r` **不在顶层**（嵌在某个 namespace 下），要**递归找**；直接 `d.get('robot_r')` 会打出 `robot_r=?`。
- 明显不撞了 → 是速度问题 ⇒ 再逐步往上找拐点
- 照撞 → 是地图/定位错位 ⇒ 走 PCD 与 AirSim 场景**叠加对比**查错位

> **通则**：**"撞障碍"这类现象先分清是「避障参数不足」还是「地图本身偏了」** ——
> 两者的修法相反（前者调参、后者修标定），用一次降速实验就能二分，
> 比对着参数表空想快得多。

细节（完整日志切片、FUEL 目标重复证据、`task_node_nav` 行号原文）见
`references/sim-planning-deadlock-and-real-aircraft-timeout.md`。

#### 文档可靠性验证法

用户问"文档可靠吗"时：把文档每条断言（话题/参数/流程）grep 到代码/配置逐一核对，输出**证据表**（断言 vs 实测 vs 判定）。debug-flight-plan.md 主体可靠但有 5 处与当前配置不符（见坑位）。


---

### 来源技能 `uav-trajectory-replay`

> 无人机（fouruav，MID-360S+D435+FAST-LIVO2+PX4）轨迹复飞功能验证与运维——两套系统：①Path Replay（nx_command_server.py 的 UDP6003/HTTP6004 纯路径回放）②SUPER 航点任务（QGC UserCmd → real_flight.launch 规划器+避障）。含录制→清理→抽稀→加载→复飞全链路地面验证步骤、航点文件格式、参数坑。触发词：轨迹复飞、航点任务、Path Replay、RecordPath、My_Flight_Path、real_flight、waypoint_mission、SUPER 航点、复飞验证。

#### 路径A 方案C/F（2026-08-12 演进）

- **C=guide_path直发**(循迹好避障空壳)；**F=MINCO必经点**(仍斜切)；**F2=vPolytopes小立方体硬约束**(斜切根治, references/plan-c-f-guidepath.md)
- **A=corridor_small_大盒子/odin单锁+20Hz限速/振荡修复/cloud_raw桥接降采样/omni清残留(勿回退)/全屏障碍=fix_map_origin误改/雷达贴地遮挡/二分测试/停车避让AVOID_WAIT已实施(cmd_pub无下划线坑)**(详见 references/planA-odin-oscillation.md)
- **参数调整经验**(速度/平滑/走廊/地图/终点/视觉桥七类+方法论, references/parameter-tuning-experience.md)
- **航迹生成器** scripts/generate_waypoints.py(straight/lshape/cshape, CSV+圆角+自交检查)
- 分支: SUPER=`feat/guide-path-c`, task_run_scripts/NX_code=`feat/4uav-guide`; 坑见 references 各文件 启动命令）、SUPER 地图不显示排查链（cloud_topic 改 /odin1/cloud_slam + TF frame 核对 + fsm_node 是否真在跑/"killing on exit"=外部关闭判读）**）见 **`references/plan-c-f-guidepath.md`**。
- **按钮类坑（2026-08-13 新增）**：UserCmd2 等按钮命令用 `&&` 串联时，第一环失败（如 MAVROS 未跑 → rosservice call set_mode 报 Service not available）会短路，后续 pkill 全不执行 → 按钮"停止"不生效，修复是 `&&` 改 `;`；plink 执行 `pkill -f` 会匹配到自身命令行（exit 128），改用精确 PID 杀。详见 reference。
- **新机载电脑部署**（<NET_WLAN_B>.31）：拉取分支流程（stash→checkout→pull→stash pop）+ livox_ros_driver2 DoubleEcho 编译错（SDK 头版本不匹配诊断）也在此 reference。
- **"启动雷达"按钮 = Radar_Launcher.yaml 的 active_profile 机制**（odin/mid360 切换；改 yaml 后必须重启 nx_command_server，配置启动时加载；两个 nx_command_server 版本并存别搞混）——见 `references/plan-c-f-guidepath.md` odin 传感器切换节。ference。
- **odin 传感器切换**：MID360→odin1 时导航只改传感器接入层（cloud_topic→/odin1/cloud_slam，用 start_all_odin_apm.launch，odin 自带定位不需要 FAST-LIVO2）；"启动雷达"按钮可能启动的是 FAST-LIVO2 而非 odin，先查按钮定义（reference 末尾新增章节）。ference。

#### 核心事实（先记住，避免走弯路）

- 实机 = MID-360S + D435 + FAST-LIVO2 + PX4（**不是 odin**）；SSH 入口 <NET_WLAN_A>.45（DHCP 会变，连不上先 ping 扫；**2026-08-12 实测：IP 变了但 `plink fouruav@fouruav`（主机名/mDNS）能连**——<NET_WLAN_A>.45 超时先试主机名，Test-NetConnection 确认 22 端口通=网络 OK 再重试，别反复白试）
- **轨迹复飞配置全在 `~/catkin_ws/src/task_run_scripts/`**（定制副本，已适配 FAST-LIVO2）。**SUPER 仓库（~/catkin_ws/src/SUPER/）里的配置是 odin 旧版，别动**
- 启动链路：`scripts/NX_code/start_fastlivo2.sh false`（拉起 D435→Livox→IMU_TF→RViz→FAST-LIVO2→px4_and_others，日志 task_run_scripts/log/fastlivo2_*/）
- 常驻服务：start_qgc_servers.sh 拉 roscore + nx_command_server2.py（**只是包装器，逻辑在 nx_command_server.py**）+ touch + stream

**两套复飞系统**

#### ① Path Replay（纯路径回放，无避障）

| 通道 | 端口 | 内容 |
|---|---|---|
| 指令 UDP | **6003** | 明文命令：`RecordPathStart`/`RecordPathStop`/`RecordPathReplay`/`AbortReplay` |
| 状态 HTTP | **6004** | `/check_replay_status`（IDLE/RUNNING_WAITING/RUNNING_FLYING/DONE_SUCCESS/DONE_ABORTED/ERROR）、`/check_pose`（起飞安全闸：z 0.4~1.6m 且距原点<0.2m）、`/check_file` |

- 回放文件：`scripts/NX_code/My_Flight_Path.txt`（CSV `x,y,z,qx,qy,qz,qw`；环境变量 LIYUMEN_PATH_FILE 可改）
- 录制定位源：`/mavros/local_position/pose`，40Hz 写文件
- 复飞 daemon（/tmp/path_replay_daemon.py）：先发首点高度悬停指令等 OFFBOARD+armed（WAITING）→ 逐点 40Hz 发 `/mavros/setpoint_position/local`（FLYING）→ 结束切 POSCTL
- **地面验证安全**：不装桨不 armed 会停在 WAITING 不飞；AbortReplay 自动切 POSCTL 保安全
- 模拟触发（QGC 界面外）：`python3 -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b'RecordPathReplay',('127.0.0.1',6003))"`

#### ② SUPER 航点任务（规划器+避障）

- 触发：QGC UserCmd1（启动）/ UserCmd2（关闭，先 POSCTL 再杀，安全顺序）/ **UserCmd3（按钮3 = 启动 SUPER 复飞，加载 `data_guidepoints/guidepoints_new.txt`，2026-08 已从占位按钮改造）**
- **航迹录制归档机制（2026-08-10 改造，nx_command_server.py 已改）**：
  - `RecordPathStart` → **直接生成 `task_run_scripts/data_guidepoints/guidepoints_YYYYMMDD_HHMMSS.txt`**（时间戳命名，永不覆盖；PATH_FILE 全局变量动态指向它）
  - `RecordPathStop` → 同步复制为 `data_guidepoints/guidepoints_new.txt`（**始终最新**，按钮3 固定加载它）
  - **目录在 `task_run_scripts/data_guidepoints/`**——代码里 `_guide_dir` 用 `os.path.dirname(os.path.abspath(__file__))` ×3 上溯（NX_code→scripts→task_run_scripts），别改成 NX_code/data_guidepoints（按钮3 的 ls/路径全用 task_run_scripts 下的）
  - 时间戳文件=历史归档；guidepoints_new.txt=当前最新；旧 My_Flight_Path.txt 保留作暂存不再写入
- **⚠️ 录制航迹终点必须保持高度，不能落到地面**（用户 2026-08-10 实测踩坑）：末尾降落段（z<0.3m，可占 38% 点）混入航迹 → 复飞时无人机冲向地面 + RViz 终点被密集低点淹没分不清。**用户最终决策（2026-08-10 会话末拍板）：不做代码过滤**（曾提出 RecordPathStop 时剔 z<0.3m 点，用户说"不用改，我以后注意"）——靠录制习惯解决：走完轨迹**保持高度直接 Stop**，别降到地面。**不要擅自给 RecordPathStop 加过滤逻辑**。
- **按钮机制**：QGC 按钮 → UDP 6003 → nx_command_server.py 每次点击**重新读 User_define_commands.yaml** → 执行 command 字段 → **改 yaml 即生效，无需重启服务**
- **按钮2 关闭逻辑（修复版）**：`rosservice call /mavros/set_mode POSCTL && sleep 1 && pkill -SIGINT -f 'roslaunch task_run_scripts real_flight.launch' && sleep 2 && pkill -f 'fsm_node'; ...; pkill -f 'map_to_world'; pkill -f 'top_down.rviz'`
  - **对按钮1/按钮3 通用**：`-f 'roslaunch ... real_flight.launch'` 匹配所有含该子串的进程（无论带不带 waypoint_data_path 参数）
  - **`&&` 链语义**：切 POSCTL 失败则后续杀进程不执行（用户明确选择保守不改——宁可关不掉也不能失控时贸然杀）
  - **别用 `pkill -f 'rviz'`**：模糊匹配会误杀推流服务 rviz_full_stream2.py（遥控器黑屏）和 SLAM 的 mapping_MID360.launch（命令行含 "rviz:=false"）
- launch：`roslaunch task_run_scripts real_flight.launch [waypoint_data_path:=<csv路径>]`（可覆盖航点源）
- 配置：`task_run_scripts/config/real_flight.yaml`（**cloud_topic 已是 /cloud_registered**；odom_topic 仍 /mavros/local_position/odom）、`waypoint.yaml`（**goal_pub_topic 已是 /waypoint_goal**）、`super_to_px4_bridge.yaml`（前馈全开 use_velocity_ff/use_accel_ff=true gain 0.7；geofence_en=false 默认关）
- 航点触发：waypoint.yaml `start_trigger_type: 1` = RC **CH10 下降沿**（`last_ch_10 > 1500 && ch_10 < 1500`）；**⚠️ 代码铁证（2026-08-10 实读 ros1_waypoint_planner.hpp:68-84）`pitch_up = channels[1] < 1200` 计算了但没参与 if 判断——CH2 条件不生效，唯一条件就是 CH10 高→低下降沿**；0=RViz；2=延时自动
  - **拨杆操作坑（实测：贴地转桨不飞）**：CH10 **一直停在低位（如 1050）永远不会触发**——必须"先拨高(>1500)等 1 秒 → 再拉低(<1500)"制造下降沿。判定：日志出现 `-- [MISSION] Mavros triggered.` 才算触发成功；FSM 一直 WAIT_GOAL + 无 triggered = 拨杆问题（不是规划问题）

#### 航点文件格式（读 config.hpp 确认）

- **空格格式**：`x y z switch_dis`（每行第 4 列 = 该点切换距离(米)，覆盖 yaml switch_dis 默认 3.0）——real_mission.txt 用这个
- **CSV 格式**：`x,y,z,qx,qy,qz,qw`（7 列）——My_Flight_Path.txt 用这个，切换距离统一用 yaml 默认（3.0）
- 切换判定：`(waypoint - cur_position).norm() < switch_dis`，3D 欧氏距离(米)
- 抽稀：`waypoint_downsample_dist`（米）只在 CSV 分支生效；实测 1038 点 → 0.1m 抽稀 = 83 点（0.5m = 14 点）；抽稀判定是**3D 欧氏距离**（`(wp - last_kept_wp).norm()`，Vec3f 含高度差，config.hpp 注释"与上一个保留航点距离"），不是 2D 平面距离
- switch_dis 与抽稀匹配原则：若抽稀 0.1m 而 switch_dis=3.0，则相邻航点 0.1m 全在 3m 到达圈内会瞬间连跳多个点、抽稀白做；当前配置 switch_dis=0.1 与抽稀 0.1 自洽

#### 抽稀的替代思路（2026-08-10 用户拍板"抽稀思路不好"，讨论产出）

- **cmd_type（0=PATH/1=waypoints）是死代码**：config.hpp 只 `LoadParam("cmd_type", ...)`，ros1_waypoint_planner.hpp 里从未使用——waypoint_mission 只有"逐点发目标"一种模式，path_pub_ 只用于可视化不控制飞行。**别指望改 cmd_type 切换模式**
- 抽稀的根本缺陷：点仍几十个 → SUPER 每点重规划一次（A*+MINCO）→ MINCO NaN 概率高；点间距 0.1m 时飞行是"点到点停顿式"；把规划器当点跟踪器用，大材小用
- **方案 A（推荐，符合 SUPER 设计哲学）**：拐点提取——只保留起点/终点/方向角变化>阈值(15~30°)的转弯点（~8-15 个），让 SUPER 自己 A*+MINCO 规划平滑轨迹。SUPER 是"目标点→平滑轨迹"规划器，录的轨迹应是参考路线而非逐点追踪表
- **方案 B**：B 样条/多项式拟合原始轨迹生成平滑点列（保形，仍几十点但易收敛）
- **方案 C**：只给起点+终点（最简单最稳，但不保证沿录制轨迹走，可能抄近路）
- 拐点提取实现思路：遍历轨迹点算相邻线段方向角，变化>阈值保留，首尾必留；可写 Python 脚本录制后自动跑
- **独立全局 A* 规划器节点（用户追问"先全局规划再 SUPER 局部规划"时产出，2026-08-10 设计未实现）**：
  - **先泼冷水**：SUPER **没有**外部全局路径输入接口——FSM 只有 `goal_sub_`（点击）和 `waypoint_sub_`（航点）两个订阅；Astar 只接受"起点+终点"无参考走廊/参考线输入；`reference_path` 只是日志记录（log_utils.hpp）。两级规划（A*前端全局 + MINCO后端局部）在 SUPER **内部一体，不对外开放**
  - 方案 A：新写 `global_planner_node`（放 task_run_scripts/src/，复用 SUPER 的 Astar 类）——订阅 `/fsm_node/rog_map/occ`（占用点云，现成）+ `/move_base_simple/goal`（目标）→ `pointToPointPathSearch(当前位置, 目标, flag, horizon, out_path)` 得全局路径 → 按**路径弧长切段**（每 2.5m 一航点，不是抽稀）→ 逐个发 `/waypoint_goal`（FSM 已订阅）。与抽稀本质区别：抽稀=死轨迹删点；切段=**地图上实时算路径，能避新障碍**
  - **Astar 接口（astar.h 确认）**：构造 `Astar(cfg_path, ROGMapROS::Ptr, RosInterface::Ptr)`（直接持有同一 ROG-Map 实例→坐标系天然一致）；搜索 `RET_CODE pointToPointPathSearch(start, end, flag, searching_horizon, out_path, time_out=0.1)`；**`flag` 是位标志**（USE_INF_MAP/USE_PROB_MAP/UNKNOWN_AS_OCC/UNKNOWN_AS_FREE 等，setup() 里按位与解析），不是简单 0/1/2——且 inf_map 和 prob_map 不能同时用
  - **⚠️ 关键隐患（决定方案复杂度）**：A* 搜索范围 = 局部地图窗口（real_flight.yaml `map_size: [30,30,6]`），`searching_horizon>0` 时以 start 为中心；地图是**随飞机滑动**的 robocentric 机制 → 目标超出窗口/地图边缘 → 搜索失败或路径到窗口边为止，**必须加"滚动重规划"**（每到达切段航点或窗口边缘，用新位置重跑全局 A*）。**飞行范围 ≤30m 可做简化版（隐患可控），>30m 长距离必须滚动规划，复杂度上一个台阶**。其他隐患：一次性规划不实时避新障碍（靠 SUPER 每段局部绕行兜底）、坐标系对齐（/move_base_simple/goal 的 map 系 vs A* 地图系，RViz Fixed Frame 设对）、3D A* 性能（160 万体素 0.5~2s，可降为 2D+固定高度）、复用 Astar 要链接 super 静态库（CMake `add_library(super STATIC)`）
- **⭐ 方案收敛（2026-08-10 用户拍板方向；2026-08-11 已完整实施并编译通过 ✅）**：全局路径 = 飞一遍录制的轨迹（人工飞行=人工全局规划），注入 SUPER 的 guide_path——不再写外部 A* 节点。关键注入点已定位（super_planner.cpp）：
  - `generateExpTraj()` 第 386 行：`vec_Vec3f guide_path;` 是**局部变量**，由 A* 生成（1108-1171 行 `pointToPointPathSearch`/`escapePathSearch`），随后第 710 行 `cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, ...)` 沿它生成凸走廊，MINCO 在走廊内优化（754 行 `OptimizationExpTrajInPolytopes`）——**在 A* 前检查 `global_path_` 非空则直接用录制轨迹作为 guide_path（跳过 A*），SUPER 就会沿你的轨迹做局部规划+动态避障**（走廊每次 replan 重建、地图实时更新，避障保留）
  - **✅ 已实施的改动（2026-08-11，SUPER 仓库 dev/4uav 分支，全部有 .bak_globalpath 备份）**：
    1. `include/super_core/super_planner.h`：加成员 `vec_Vec3f global_path_` + `std::mutex global_path_mtx_` + `bool global_path_valid_` + `double global_roll_dist_{5.0}` + `bool global_used_prev_`；加 public setter `setGlobalPath(path, roll_dist)` / `clearGlobalPath()` / `globalPathValid()`
    2. `include/ros_interface/ros1/fsm_ros1.hpp`：加 `global_path_sub_` 订阅 `/global_path`（nav_msgs/Path）+ `globalPathCallback`（转 vec_Vec3f → `planner_ptr_->setGlobalPath`；读 `~global_path/roll_dist` 参数；空路径调 clearGlobalPath；日志 `-- [Fsm] GLOBAL PATH received: N pts, roll_dist=5 m`）
    3. `src/super_core/super_planner.cpp` generateExpTraj "Second Part" 注释前注入块（约 40 行）：锁 global_path_mtx_ → findNearestIdx（当前位置在全局路径上的最近点，线性扫）→ **全程取段**（从 idx 一直到 global_path_ 末尾，不是前 5m——保证 guide_path.back() 贴近终点，使 Second Part 走 "NO NEED" 分支不再 A* 续搜）→ 若段尾距 goal>`res*5` 补上 goal → 替换 guide_path + 重建 guide_stamp（cum_t += 段长/`std::max(cfg_.exp_traj_cfg.max_vel, 0.1)`）+ `guide_path_end_vel = 0.0`；else 分支 `global_used_prev_ = false`；注入块尾部每轮重置 `global_skip_this_round_ = false`
    3b. **fallback（2026-08-11 补，用户明确"除非必须绕行否则全程沿全局路径"）**：`SearchPolytopeOnPath` 失败（756-758 行原代码 `return FAILED`）处注入——若 `global_used_prev_` 且走廊失败：置 `global_skip_this_round_ = true`（**不是** `global_path_valid_ = false`，否则永久禁用下一轮不恢复）→ 清 guide_path → `PathSearch(robot_state_.p, gi_.goal_p, planning_horizon, new_path)` 用 A* 绕行 → 重建 guide_stamp → 重新 SearchPolytopeOnPath（成功继续，失败才 return FAILED）。super_planner.h 加成员 `bool global_skip_this_round_{false}`。**行为**：障碍挡住轨迹 → 本轮 A* 绕行 → 下一轮 replan 自动恢复沿全局路径
    4. `config/real_flight.yaml`：追加 `global_path:` 段（`roll_dist: 5.0`）
    5. 外部数据源：新增 `task_run_scripts/scripts/global_path_publisher.py`（读 guidepoints_new.txt CSV → nav_msgs/Path frame=map，latch + 1Hz 周期重发；用法 `rosrun task_run_scripts global_path_publisher.py _file:=<csv>`）
  - **⚠️ 全局路径绝不抽稀（2026-08-11 用户纠正"为什么还是会抽稀，我传输的是全局路径"）**：global_path_publisher 默认 `min_pt_dist: 0.0`（不抽稀，全发 743 点）。**抽稀是给 waypoint_mission 航点任务用的，全局路径点越密轨迹保真度越高**——SUPER 内部自己滚动取段（findNearestIdx→全程取段）+ SearchPolytopeOnPath 处理点密度 + MINCO 时间分配，不需要外部抽稀；抽稀反而抹平拐弯/起伏细节。若发现 `/global_path` 点数骤减（如 705 点→7 点），先查是不是 min_pt_dist 被设了非 0 值。判读：`rostopic echo /global_path -n 1` 的 poses length 应与文件点数接近
  - **⭐⭐ 点密度陷阱（2026-08-11 会话末实机，飞直线的新根因——修正上面"不抽稀"建议的边界）**：**外部发布不抽稀没问题，但注入后的 guide_path 点间距过密（<1cm）会让走廊生成器崩溃**。实测：930 点 / 4.31m 总路程 = **平均间距 0.46cm**（idx 0→1 仅 0.1cm）→ SearchPolytopeOnPath 要沿 929 个微段逐段生成凸多面体 → **`Cannot find continuous corridor on path, overlap only -326`（相邻盒子重叠为负=断裂）+ `Box search failed, box size is zero`** → 走廊失败 → fallback A* → **无障碍空间 A* 直接直线到终点 → 飞机"沿直线飞"（完全不走航迹转弯）**。判读：触发成功 + `Using FULL GLOBAL PATH` 生效（idx 正常推进）+ 但 RViz 轨迹是直线 → **先 python 算航迹文件点间距**（`总路程/点数`），若 <0.01m 就是密度问题。**✅ 正解（2026-08-11 会话末已实施+编译通过 0 错误，fsm_node 21:54 新编译）**：注入块取段后按 `global_path_downsample`（0.1m）降采样再赋给 guide_path（4.31m/0.1m ≈ 43 点，走廊生成器轻松处理）。改动清单：① `super_planner.h` 加成员 `double global_path_downsample_{0.1}`，`setGlobalPath(path, roll_dist, downsample=0.1)` 加第三参数；② `super_planner.cpp` 注入块 "步骤 3) 补终点" 与 "步骤 4) 替换 guide_path" 之间插入 3.5 步降采样（保留每 ≥ds 距离一个点，`ds_path` 首点必留、若降采样吃掉终点则补回 `sub_path.back()`，保证 Second Part 的 NO NEED 判断成立），打印 `-- [SUPER] Downsampled global path: N pts (ds=0.1m)`；③ `fsm_ros1.hpp` globalPathCallback 读 `nh_.param("global_path/downsample", downsample, 0.1)` 传入 setGlobalPath；④ `task_run_scripts/config/real_flight.yaml` 的 `global_path:` 段加 `downsample: 0.1`。**验证**：触发后 `grep "Downsampled" real_flight.log` 应出现 `~43 pts (ds=0.1m)`；同时确认无 `Cannot find continuous corridor` / `Box search failed`。注意区分两个"直线"根因：① 起点附近障碍挤走廊（occ 探针 190 占用点，见坑 13）② 点密度过密崩走廊（本坑）——共同表现都是 fallback A* 直线，先查地图占用（occ_probe）再算点密度（`总路程/点数`，<0.01m 即本坑）
  - **⭐ 会话末用户决策（2026-08-12 上午，覆盖"降采样是正解"的边界）**：用户判断降采样（0.1m）可能抹掉轨迹细节导致走廊问题，**拍板：移除降采样（还原 930 点全量）+ `corridor_bound_dis: 1.0→0.39`**（已实施+编译 0 错误）。**依据**：corridor_bound_dis 是 getSeedBBox 的"搜索盒扩展范围"（corridor_generator.cpp:201-202 `box_min -= Vec3f(bound_dis_,...)`），**不是安全距离**——盒子沿路径点向外扩 bound_dis 作为障碍搜索范围，IRIS 在其中扩凸多面体；**真正的碰撞安全由 `robot_r`（0.25m）保证，调小 bound_dis 更保守、不增加危险**（搜索盒小→盒内障碍少→IRIS 更易扩出有效盒子→小空间也能生成）。**⚠️ 尚未实飞验证**：移除降采样后 930 点/0.46cm 微段是否重现 overlap -326 未知——若崩则"点密度过密"仍是走廊失败根因（回到降采样），若好则之前失败是搜索盒过大（盒内含远处障碍挤压 IRIS）。**判读**：重启后日志有 `Using FULL GLOBAL PATH` 但**无 `Downsampled` 行** = 降采样移除生效；corridor_bound_dis 生效需重启 FSM（启动时读配置）。**教训**：corridor_bound_dis（搜索盒）与 robot_r（安全距离）是独立参数，调"走廊宽容度"改前者，调"碰撞距离"改后者
  - **⚠️ 编译坑（2026-08-11 实机，第一次编译 90% 报错）**：注入块里写 `last_exp_traj_time_pos.clear()` 会报 `'last_exp_traj_time_pos' was not declared in this scope`——该变量在 generateExpTraj **函数后段**才声明，注入块在其前 → **删掉这行**（guide_path 已非空，热启动分支不会走，不影响功能）。修复后增量编译 ~3 分钟，`grep -c "error:"` = 0，`[100%] Built target fsm_node` 出现
  - **⚠️ 括号平衡检查勿误判**：改前原文件括号就差 1（左190右189，模板/宏/字符串里有括号），**比较"改前改后差值"而非绝对平衡**（注入块差=0，净变化=0 即正常）
  - **按钮3 同步改造（2026-08-11，NX_code feat/4uav-guide 分支提交 cffadd1 + d7d573b）**：command = 先启动 global_path_publisher 再启动 SUPER。**⚠️ `& &&` 语法坑（cffadd1 踩过，d7d573b 修复）**：`nohup rosrun ... & && sleep 1 && nohup roslaunch ...` 会让 nx_command_server 报 `❌ 指令执行出错: timed out`、两个进程都不启动（`&` 后台化后 `&&` 等它返回但 shell 卡住）。**正确写法：后台命令包进子括号 `(nohup cmd &)`**——`&& (nohup rosrun task_run_scripts global_path_publisher.py _file:=.../guidepoints_new.txt > log 2>&1 &) && sleep 1 && (nohup roslaunch task_run_scripts real_flight.launch waypoint_data_path:=.../guidepoints_new.txt > log 2>&1 &)`。子括号让 `&` 只作用于子 shell、外层立即返回，`&&` 链正常走。**用 `;` 分隔也行但会失去失败短路**，子括号是正解。改完 yaml 提交后**重启 nx_command_server** 才生效
  - **路径 A 测试判读**：按按钮3 → `grep "GLOBAL PATH" real_flight.log` 预期 `-- [Fsm] GLOBAL PATH received: N pts, roll_dist=5 m`（FSM 收到全局路径 = 链路通）；**先 `rostopic echo /global_path -n 1` 看 poses length 应与航迹文件点数接近（705 点文件应显示 ~700，若只有 7 = 发布器被抽稀/加载错文件）**；RViz 加 /global_path 显示看蓝色全局路径线（在 feat/4uav-guide 分支的 top_down.rviz 才有此显示项，避障分支没有）；动态避障测试 = 纸箱放轨迹上 → 走廊失败 → A* fallback 绕行 → 移开纸箱 → 恢复沿轨迹
  - 回退：SUPER 仓库 4 个 `.bak_globalpath` 逐个 cp 回；task_run_scripts 侧 `git checkout feat/4uav-guide`
  - 方案 B 已澄清（用户确认）：当初是"写在 SUPER fsm.cpp 里读航点文件分段发送"，但当前代码/git 无痕迹（fsm.cpp 仅 230 行、dev/4uav 分支）——已回退或别处试过。**用户试过 B 后转向 A，此话题直接按 A 推进，不必再问 B 细节**

#### 地面验证标准流程（2026-08 全通过）

1. **传感器链路**：start_fastlivo2.sh false 后查 `rostopic hz /cloud_registered`（~10Hz）、`/aft_mapped_to_init`（~10Hz）、`/livox/lidar`、`/livox/imu_tf`（~192Hz）、`/mavros/state`（connected）
   - D435 话题是 `/camera/color/image_raw`（**FAST-LIVO2 视觉配置 `img_en: 0` + `img_topic: /usb_cam/image_raw`——话题名不匹配且视觉融合关闭，VIO 从未工作，定位是纯 LiDAR+IMU**，见坑 20；`/rgb_img` 空 = FAST-LIVO2 视觉输入断，属正常现状非故障）
2. **Path Replay 状态机**：curl /check_replay_status=IDLE → 触发 RecordPathReplay → RUNNING_WAITING → AbortReplay → IDLE（+ps 确认 daemon 无残留）
3. **录制**：RecordPathStart → 手持无人机走轨迹 → RecordPathStop → **检查文件尾部 NUL**（Stop 用 terminate() 杀进程，尾部留 ~200 字节 \x00，CSV 解析报 `line contains NUL`）
4. **清理 NUL**：python3 读字节截到首个 \x00 前、补 
；先备份 .bak_nul
5. **SUPER 加载**：`roslaunch task_run_scripts real_flight.launch waypoint_data_path:=<csv>`；**前台 roslaunch 输出在终端不在 real_flight.log**，用 `rosparam get /waypoint_mission/` 确认 data_path 生效，日志看 `-- [MISSION] Load N waypoints`
6. **抽稀对比**：waypoint_downsample_dist 0.0 vs 0.5 重启加载，看 Load 点数变化
7. **发测试航点**：`rostopic pub -1 /waypoint_goal geometry_msgs/PoseStamped "{header:{frame_id:'world'}, pose:{position:{x:1.0,y:0.0,z:1.2}, orientation:{w:1.0}}}"` → fsm 日志 `Get goal at [1.0, 0.0, 1.2]`（z 不被覆盖 = goal_pub_topic 修好的证据）
8. **RViz 复飞轨迹可视化**：SUPER 启动成功后 `/fsm_node/rog_map/occ` 应有 ~2Hz 持续数据（地图正常）；top_down.rviz 窗口应在 :0 屏（`xwininfo -root -tree | grep top_down`）；拨杆后 RViz 出现 frontend_path 轨迹线。**rviz 由 real_flight.launch 启动，输出 output=log**
9. **RViz 里分辨"该去哪个航点"（2026-08 实测）**：
   - **黄色 goal 标记**（`/fsm_node/visualization/goal`，代码 ros1_interface.hpp:192-195 用 `Color::Yellow()` 画当前目标，每次换点先 deleteAll 再画）= **飞机当前正飞向的航点**，动态跳变——**判断依据只看它**
   - **蓝色 Waypoints 标记**（`/waypoint_mission/mkr`，waypoint_mission 发布：SteelBlue 航点连线 + Orange 半透明球(半径=switch_dis×2) + 黑色编号）= 全部航点静态全显示，只用来对照"黄点在整条路径第几个"
   - **发布时机坑**：mkr 只在 RViz 订阅者数量变化时发一次（ros1_waypoint_planner.hpp:85-88），若 RViz 后启动看不到蓝点 → 把 Waypoints 显示项取消勾选再勾上触发重发
   - top_down.rviz **原本没配 mkr 显示**——已加 Waypoints 项（订阅 /waypoint_mission/mkr）；以后若重做 rviz 配置要记得加，否则用户分不清目标点
10. **抽稀本地模拟 + 航迹可视化检查（录完先自查再飞）**：Python 复刻 config.hpp 抽稀逻辑（距**上一个保留点** 3D 距离 ≥ waypoint_downsample_dist 才保留，首点必留）跑 CSV 文件，点数应和真实 waypoint_mission `Load N waypoints` 一致（实测 972→64、1856→68 全吻合）；再用 matplotlib 画俯视(原始+抽稀)/3D/高度剖面三图，检查：终点 z 是否 <0.3m（降落段混入）、高度是否稳定、起终点位置。航迹文件经 plink cat 拉回本地 Temp 再画（见 ssh-remote-linux-ops 的 plink -m 读法）

#### git 结构与拉取/合并（2026-08-10 实机操作）

- **task_run_scripts 是 catkin_ws 的子模块**（`.git` 是 48 字节文件，内容 `gitdir: ../../.git/modules/src/task_run_scripts`）；**scripts/NX_code 是嵌套子模块**（.git 指向 `.git/modules/src/task_run_scripts/modules/scripts/NX_code`）
  - 远程：task_run_scripts = `git@github.com:jihaofly/task_run_scripts.git`（分支 main，另有 **`feat/obstacle-to-px4`**）；NX_code = `jihaofly/NX_code.git`（master）+ upstream `Liu404mob/NX_code.git`
  - **本地未提交修改**：`config/*.yaml`、`rviz/top_down.rviz`、`NX_code/nx_command_server.py`、`NX_code/User_define_commands.yaml`（M）+ `data_guidepoints/`、`guide_Readme.md`（??）——**全部未提交，git 帮不上忙，拉取前必须 tar 快照**
- **拉取前备份（用户改的文件多、git 未提交时）**：
  ```bash
  # ① 停服务避免文件占用 ② tar 快照（排除 log/ 与 .git）③ 抽查关键文件在包里
  PID=$(ss -tlnp | grep 6004 | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2); [ -n "$PID" ] && kill $PID
  cd ~ && tar czf ~/task_run_scripts_backup_$(date +%Y%m%d).tar.gz \
    --exclude='task_run_scripts/log' --exclude='task_run_scripts/.git' -C ~/catkin_ws/src task_run_scripts
  tar tzf ~/task_run_scripts_backup_*.tar.gz | grep -cE 'config/waypoint|nx_command_server.py$|data_guidepoints/guidepoints_new'  # 抽查
  ```
- **拉取前先 fetch 侦察（无风险）**：`git fetch origin` → `git log --oneline HEAD..origin/main`（看落后几个提交）→ `git branch -a`（找新分支）→ `git remote -v`（看 upstream）。**有未提交修改时 git pull 会报 `local changes would be overwritten`——不要 force，先备份再定策略**
- **恢复**：整体 `tar xzf ~/task_run_scripts_backup_*.tar.gz -C ~/catkin_ws/src`；单文件 `tar xzf ... -C ~/catkin_ws/src task_run_scripts/config/waypoint.yaml`；**别人也改了 nx_command_server.py 时用 diff 合并，勿整体覆盖**（可能覆盖别人新功能）
- **线索（2026-08-10）**：远程 `feat/obstacle-to-px4` 分支含 `5edb1bb feat: 点击固定高度、航点跟随航点高度`——**可能已修复本机高度覆盖问题**（本机 z=0.554→0.825 吸附偏移问题），拉新代码时优先看该分支；另有 `efcb65b feat: PX4 obstacle avoidance node (OBSTACLE_DISTANCE)` 避障节点

#### 把本地修改落成独立分支（2026-08-11 实机，feat/4uav-guide 流程）

**场景**：本地有多处未提交修改（用户改的 yaml/rviz/NX_code py），既要不影响避障分支、又要让修改可回退可切换 → 建专属分支。
- **先验证零冲突**：`git diff --name-only main..feat/obstacle-to-px4` 看避障分支改哪些文件，`git status --short` 看本地改哪些，**两者交集为空才安全**（实测：用户改的 5 文件避障分支全没动）
- **⚠️ 踩坑（checkout 被未提交修改挡住）**：直接从避障分支 `git checkout main` 会报 `error: 您对下列文件的本地修改将被检出操作覆盖`（避障分支的未提交修改占用）→ 盲目 `checkout -b` 会在**错误基底**（避障分支）上建新分支。正确流程：
  ```bash
  # ① 先整体 stash（含未跟踪，-u）——当前所有修改+未跟踪全收进临时 stash
  git stash push -u -m "temp_all_before_rebuild"
  # ② 切回正确基底 main → 删错分支 → 重建
  git checkout main && git branch -D feat/4uav-guide && git checkout -b feat/4uav-guide
  # ③ 只恢复用户修改的 3 个文件（git checkout <stash> -- <file> 逐个，别 pop 全部——pop 会把避障修改也带回）
  git checkout stash@{0} -- config/super_to_px4_bridge.yaml config/waypoint.yaml rviz/top_down.rviz
  # ④ 未跟踪文件（data_guidepoints/ guide_Readme.md 等）在 stash 的 ^3 树里（git stash push -u 的第三个 parent）：
  git checkout stash@{0}^3 -- data_guidepoints guide_Readme.md troubleshooting_Readme.md config/mid360.yaml.bak_livox
  # ⑤ 提交 → 切回避障分支 → git stash apply 还避障修改 → git checkout -- <用户文件> 清掉混入的
  ```
  - **未跟踪文件恢复技巧**：`git checkout stash@{N} -- <path>` 对已跟踪文件有效；**未跟踪文件在 stash 的 ^3 树**（`git log stash@{0}` 可见 `untracked files on ...` 提交），用 `git checkout stash@{0}^3 -- <path>` 恢复
  - **stash pop vs apply**：pop 会删 stash 条目，apply 保留——**多分支场景用 apply**（避障分支还要它）
- 子模块 NX_code 同样操作：`cd scripts/NX_code && git checkout master && git checkout -b feat/4uav-guide && git stash pop`（NX_code 自己的 stash 在它自己的仓库）
- 结果：三个分支互不干扰——`feat/4uav-guide`（用户修改+路径A按钮3）、`feat/obstacle-to-px4`（避障）、`main`（基准）；tar.gz 快照作第四层保险

#### 编译 catkin_ws（B 分支新代码后必做，2026-08-10 实机）

- **必须带 `-DROS_EDITION=ROS1`**，否则 livox_ros_driver2 编译失败：
  ```bash
  cd ~/catkin_ws && source /opt/ros/noetic/setup.bash
  catkin_make -j4 -DROS_EDITION=ROS1
  ```
  - **坑**：livox_ros_driver2 的 CMakeLists 用 `ROS_EDITION` 变量切换 ROS1/ROS2 分支（`if(ROS_EDITION STREQUAL "ROS1")`，224 行附近 `find_package(ament_cmake_auto REQUIRED)` 是 ROS2 分支）。直接 `catkin_make` 不带参数默认走 ROS2 → Noetic 没装 ament_cmake_auto → `CMake Error ... ament_cmake_auto not found`。**这不是环境坏了，是缺编译参数**
  - 失败后清理：`rm -rf ~/catkin_ws/build/livox_ros_driver2` 再重编
- **B 分支拉取完整流程（有未提交修改时，2026-08-10 实机验证）**：
  ```bash
  # ① 备份（见上节 tar 快照）② stash 本地修改（NX_code 是嵌套子模块，两处各自 stash）
  cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code && git stash push -m "before_feat"
  cd ~/catkin_ws/src/task_run_scripts && git stash push -m "before_feat"
  # ③ 切换分支 + 更新嵌套子模块
  git checkout feat/obstacle-to-px4
  git submodule update --init --recursive
  # ④ 编译（上面带 ROS_EDITION 参数）⑤ 验证新节点产物
  ls ~/catkin_ws/devel/lib/task_run_scripts/obstacle_to_px4_node
  ```
  - **未跟踪文件（??）不在 stash 里**（data_guidepoints/、guide_Readme.md、*.bak_newfile）——checkout 不会覆盖它们，安全
  - 恢复本地修改：`git stash pop`（在 NX_code 里 pop NX_code 的 stash，在 task_run_scripts 里 pop 顶层的；冲突时 diff 合并）
  - 拉取后**验证避障节点产物**：`devel/lib/task_run_scripts/obstacle_to_px4_node`（B 分支新功能：PX4 OBSTACLE_DISTANCE 避障）
- 编译时长参考：Jetson 全量 ~15 分钟（-j4），进度 `tail -f /tmp/catkin_make_B.log`，看 `Built target` 递增；完成后 `grep -c "error:"` 应 0

#### obstacle_to_px4 避障链路（B 分支新功能，2026-08-10 实机调试）

- 链路：`/cloud_registered` → obstacle_to_px4_node（36 扇区）→ `/mavros/obstacle/send`（LaserScan 20Hz）→ MAVROS distance_sensor 插件 → MAVLink OBSTACLE_DISTANCE → PX4
- launch：`roslaunch task_run_scripts obstacle_to_px4.launch drone:=mid360`；配置 `config/mid360.yaml`
- **坑 A（no cloud received）**：launch 静态变换是 livox_frame→aft_mapped，但 **livox_frame 在 TF 树中不存在**（FAST-LIVO2 不发布它）→ lookupTransform 失败丢点云。修复：`mid360.yaml` 的 `cloud_topic: /livox/lidar` 改 `/cloud_registered`（camera_init 系，camera_init→aft_mapped 变换存在）。判读正常：日志 `cloud pts=3905 processed=229` + `pub bin0=... min=0.37`
- **坑 B（节点正常但 PX4 不避障）**：PX4 `COM_OB_ACT=0`（默认关）——`rosservice call /mavros/param/get "param_id: 'COM_OB_ACT'"` 查，=1 开启 + COM_OB_RADIUS 设生效半径。**节点工作 ≠ 避障生效，最后一段是 PX4 参数**
- 详见 references/obstacle-to-px4-chain.md

#### bridge 起飞悬停状态机（super_to_px4_bridge.cpp，用户说的"切OFFBOARD先起飞悬停"）

```
Phase1 预发布指令(切OFFBOARD前必须有指令流) → Phase2 自动解锁/切OFFBOARD(auto_arm/auto_offboard=false 时跳过)
→ Phase3 等 OFFBOARD 生效(期间持续发 init_goal) → Phase4 自动起飞到 init_goal 高度(|z-1.2|<0.1m 算到，30s超时兜底)
→ Phase5 主控制循环(跟踪 /planning/pos_cmd；RC丢失>1s/越界/MAVROS断连 → 位置保持或降落)
```
- 高度参数：`task_run_scripts/config/super_to_px4_bridge.yaml` 的 `init_goal/z`（**实际加载 task_run_scripts 副本，不是 SUPER 仓库的**；2026-08-10 用户已从 1.2 改成 **0.6**——起飞目标高度，改了重启 launch 生效）
- **起飞高度 vs 航迹高度不一致**（常见坑）：bridge 起飞 1.2m 但录制航迹 z≈0.44~0.75m → 拨杆后飞机从 1.2m 下探到航迹高度，有下降过渡。要么 init_goal/z 改成航迹高度，要么重新录 1.2m 航迹

#### 起飞前检查（真飞前逐项过）

1. 传感器链路：/cloud_registered ~10Hz、/aft_mapped_to_init ~10Hz、/mavros/state connected
2. 确认 guidepoints_new.txt 时间戳是最后录制那次；抽稀后点数（本地模拟或 Load N 日志）
3. 关键配置：init_goal/z 与航迹高度一致；geofence_en 建议改 true（默认 false！）；首飞 max_vel 降到 1.0（当前 2.0）；velocity_ff_gain 首飞降到 0.3~0.5（当前 0.7 全开）
4. 起飞点 ≈ 录制起点（误差 <0.5m，复飞从当前位置规划到 1 号航点）
5. 悬停稳定 3~5s 后看 RViz 飞机位置与地图对得上再拨杆
6. 紧急接管：遥控器切 Position 优先于一切；按钮2 关闭（先切 POSCTL 再杀）

#### 坑清单

1. **/planning/state 话题不存在**（文档说 echo 它会失败）——FSM 只发 /planning/pos_cmd、/planning_cmd/poly_traj、fsm/path；状态看日志 `-- [Fsm]` 前缀（WAIT_GOAL/GENERATE_TRAJ）
2. **click 通道覆盖高度**：/move_base_simple/goal → goalCallback(override_height=true) → z 强制成 click_height(1.0)；/waypoint_goal → waypointCallback(false) 不覆盖。**别把 goal_pub_topic 改回 /move_base_simple/goal**
3. **无限高 1m 配置**：click_height=1.0 是点击目标固定高度不是限高；真限高 = real_flight.yaml `virtual_ceil_height`(3m) 或 bridge geofence（默认关）
4. **录制文件 NUL 损坏**（见验证流程第 3-4 步）
5. **SUPER 仓库副本 vs task_run_scripts 副本**：改 task_run_scripts 的，别改 SUPER 仓库的。**⚠️ 实机确认（2026-08-11）：FSM 实际加载 `task_run_scripts/config/real_flight.yaml`**（real_flight.launch 的 `config_path=$(find task_run_scripts)/config/real_flight.yaml`），**SUPER 仓库的 real_flight.yaml 不被加载**——改 raycasting/global_path 等参数必须改 task_run_scripts 那份，SUPER 仓库那份改了不生效（路径A 的 `global_path/roll_dist` 加在 SUPER 仓库那份实际靠默认值 5.0 兜底，凑巧一致才没暴露）。排查"改了配置不生效"先 `ps aux | grep fsm_node | tr ' ' '
' | grep yaml` 确认 FSM 加载哪份
6. 改 yaml 后**必须重启 real_flight.launch** 才生效（waypoint_mission 启动时读文件）
7. **RViz 一片白 = 先查 fsm_node 存活 + `/fsm_node/rog_map/occ` 有无数据，别先查 rviz 配置**。白屏根因通常是地图无数据（occ 话题 `no new messages`），而地图无数据根因是 fsm_node 死了
8. **同名节点顶替杀进程**：重复启动 real_flight.launch → 日志 `Reason: new node registered with same name` → 旧实例全部退出（"all processes on machine have died"）→ RViz 白屏。按钮1/2/3 同一时间只应有一个 SUPER 在跑
9. **前台 roslaunch 输出在终端不在 log**：`ps` 查进程 `fd/1` 指向 `/dev/pts/N` 即前台；核对实际加载用 `rosparam get /waypoint_mission/`（data_path/config_path）
10. **用户偏好手动拨杆**：start_trigger_type 保持 1（RC ch10），**不要改成 2（自动触发）**——用户明确要求保留手动控制才启动航点任务
11. **重启 nx_command_server 服务（改完 py 必做）**：改 nx_command_server.py 后必须重启才生效。正确姿势：
    ```bash
    # ① 用 ss 找 6004 监听 PID 再 kill（别用 pkill -f 'nx_command_server2.py'——plink -m 脚本命令行含同名字符串会自杀！用正则技巧 nx_command_server[2]\.py 或先取 PID）
    # ② 启动必须带 ROS 环境，否则 ModuleNotFoundError: rospy：
    cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code
    nohup bash -c 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export DISPLAY=:0 && exec python3 -u nx_command_server2.py' > ~/catkin_ws/src/task_run_scripts/log/nx_command_server2.log 2>&1 &
    # ③ 验证 ss -tlnp | grep 6004 + curl /check_replay_status
    ```
    注意 start_qgc_servers.sh 可能被自启/守护机制重新拉起，重启后确认监听的是新 PID
12. **录制文件尾部 NUL 损坏**：RecordPathStop 用 terminate() 杀 daemon，文件尾部常留 ~200 字节 \x00（CSV 解析报 `line contains NUL`）。清理：python3 读字节截到首个 \x00 前、补 
，先备份 .bak_nul
13. **"切 OFFBOARD 后没起飞/贴地转桨"排查（2026-08-10 两次实机案例）**：先读 `task_run_scripts/log/real_flight.log`，按时间线拆：
    - bridge 段看起飞是否成功：`Waiting for OFFBOARD` → `OFFBOARD mode engaged` → `Current altitude: x.xx m, Target: 0.60 m` 逐秒爬升 → `Drone reached target altitude`。**若这段成功 = 起飞链路正常**
    - fsm 段看是否收到航点：`Current state: WAIT_GOAL` 一直刷 + **没有 `-- [MISSION] Mavros triggered.` = waypoint_mission 从未被触发（拨杆问题，见上面触发坑）**，飞机没指令可跟 → 悬停不住落回地面贴地转桨
    - 若已 triggered 但规划失败：`PlanFromRest failed, try replan` + `CIRI WARNING! ... min dis to obstacle is only: 0.00000001` = **地图障碍问题**（ROG-Map 认为飞机贴着障碍，常见于起飞点太挤/坐标系错位）；`GenerateExpTrajectory failed` = 轨迹生成器内部失败（前馈/边界/时间分配约束问题）
    - **⭐ 已 triggered 且 Pub goal 出现，但规划失败（2026-08-10 第二次实机案例，最隐蔽）**：对比两行日志——waypoint_mission 的 `Pub goal to x y z`（原始航点）vs fsm 的 `Get goal at x y z`。**坐标不一致 = `getNearestInfCellNot` 障碍吸附**（fsm.cpp:191：FSM 收到航点后找"离该点 3m 内最近空闲格"作为实际目标，`Get goal at` 打印的是吸附后的坐标）——航点在 ROG-Map 里被判占用（人走过的障碍块残留/起飞点拥挤）→ 被吸到旁边空闲格 → 目标高度突变 → `[MINCO] TrajOpt failed, The function value became NaN or Inf` → GenerateExpTrajectory failed → 复飞中断落地。根治：起飞前清地图残留（原地转圈让激光重扫 / 重启 SUPER / p_free 调高），起飞点选空旷处。详见 references/super-planner-arch-and-map.md 第 3 节
    - **⭐ 路径 A 实飞新故障：沿轨迹乱转 + close to goal + 超时停住（2026-08-11 实飞日志判读，最终根因=路径A与航点任务双系统冲突）**：现象——起飞 0.6m 悬停成功 → CH10 触发成功（`Mavros triggered.` + `Pub goal`）→ `FOLLOW_TRAJ` + `ReplanOnce succeed` 连续成功，**但 POS+VEL 轨迹显示飞机在回字形内乱转**（(0.35,0.35)→(0.09,-0.02)→(1.12,0.37)…来回打转），最终 `-- [SUPER] Replan, close to goal and return NONEED.`（FSM 判"接近目标"停止推进，但实际位置≠goal）→ `OptimizationExpTrajInPolytopes for new path failed` → **`SUPER command timeout. Switching to position hold.`**（bridge 0.5s 超时→POS_HOLD）→ 停在 (1.08,0.44,0.82) 不动。**根因**：路径A（guide_path=全程全局路径）和 waypoint_mission（逐点发 /waypoint_goal，switch_dis=0.1 到达才切下一个）**两个系统同时在跑互相打架**——FSM 一会儿沿轨迹飞、一会儿追逐点 goal（goal 卡在航迹第 1 点不更新），方向冲突→乱转；"close to goal" 误判到达→停止推进；随后走廊优化失败 + 超时→bridge 切 POS_HOLD。**⚠️ 注意先查航迹文件再下结论**：本案例航迹起点 z=0.849 是文件本身值（1617 点，x:-0.14~1.88 y:-1.98~0.44 z:0.61~0.87），**不是 FSM 吸附**（之前判断"吸附到 0.825"是错的——用 python 读 guidepoints_new.txt 起终点核对）。**方案 A（推荐）**：去掉 waypoint_mission 逐点逻辑（只发最终终点一个 goal + 全程 guide_path），FSM 全程沿轨迹飞不逐点切换，两者不再冲突；**方案 B**：回到纯航点模式（不用 global_path）。**⭐ 方案 A 落地 → 方案 C 最终收敛（2026-08-11 实机）**：
    - **方案 A 实测（use_waypoint:=false）发现新坑：CH10 门禁丢失**——`use_waypoint:=false` 禁掉 waypoint_mission 后，**global_path_publisher 若直接发布 /waypoint_goal 会绕过 CH10 直接触发 FSM**（waypointCallback 收到 goal 就 setGoalPosiAndYaw，**CH10 检查只存在于 waypoint_mission 内部**，与 global_path_publisher 无关）→ 现象"**没按 CH10 无人机自己开始循迹**"。判读：日志有 `Get goal at` 但**没有 `-- [MISSION] Mavros triggered.`** = 触发门禁被绕过；goal 值固定（如 0.075,-0.075,0.825）且非航迹终点 = 发布器发的 goal 而非 waypoint_mission 发的
    - **方案 C（最终落地，2026-08-11 提交 8f816a3 + 发布器移除 goal）**：**保留 waypoint_mission（CH10 门禁）+ 只发终点 + 全程全局路径引导**。三件事：① 生成 `data_guidepoints/endpoint.txt`（从 guidepoints_new.txt 取**最后一点**；**⚠️ 必须空格格式 4 列 `x y z switch_dis`，如 `0.072888 -0.047980 0.607613 0.1`**——config.hpp 空格格式分支要求 `result.size() >= 4`（第 4 列=切换距离），**3 列会被静默跳过 → `Load 0 waypoints` → CH10 触发了但没 goal 可发**（2026-08-11 实机坑，提交 ba5680c 修复））② 按钮3 改 `use_waypoint:=true waypoint_data_path:=.../endpoint.txt`（waypoint_mission 读终点文件 → CH10 下降沿触发 → 发终点 goal）③ **global_path_publisher.py 移除 /waypoint_goal 发布**（只保留 /global_path + /global_path_marker，让 waypoint_mission 独占 goal 通道）。最终链路：`global_path_publisher→/global_path(完整轨迹)` + `waypoint_mission(endpoint.txt)→CH10 触发→/waypoint_goal(终点)` + `FSM: goal=终点, guide_path=完整轨迹→沿轨迹飞`。**验证**：`grep "Pub goal"` 应只有 1 次（终点）；`ps aux | grep waypoint_mission` 在跑；未拨杆时日志无 `Mavros triggered`。判读铁证：`Replan, close to goal and return NONEED` + `SUPER command timeout. Switching to position hold` = 停住；POS+VEL 坐标序列能看出乱转轨迹。
    - **⭐ 方案 C 后续演进（2026-08-11 会话末实机，全部已提交 feat/4uav-guide）**：
      - **endpoint.txt 自动更新（提交 8f865cc + ba5680c）**：用户问"每一次 endpoint 都会自动更新终点吗"——**不会**，endpoint.txt 是手动生成的静态文件，重新录制后不跟随。修复：nx_command_server.py 的 `RecordPathStop` 同步 guidepoints_new.txt 后**自动提取最后一点写 endpoint.txt**（读 CSV 取末行前 3 列，**空格格式 4 列 `x y z 0.1`——第 4 列 switch_dis 必须有**，否则 waypoint_mission `Load 0 waypoints`；失败打印 `提取终点失败(文件格式?)`）。这样每次录新航迹 → 终点自动跟随，按钮3 无需手动改。**教训：方案 C 依赖的中间文件（endpoint.txt）必须由录制流程自动维护，否则录一次废一次**
      - **⭐ 诊断链："CH10 触发了但无人机在起点没动"（2026-08-11 实机，方案 C 测试）**：日志出现 `-- [MISSION] Mavros triggered.`（触发成功 ✅）但**没有 `Pub goal` / `Get goal`**、FSM 一直 `Wait for goal` → 不是拨杆问题、不是规划问题，是 **waypoint_mission 加载了 0 个航点**：查 `-- [MISSION] Load N waypoints` 若 N=0 → endpoint.txt 格式不对（最常犯=空格格式缺第 4 列 switch_dis，或路径不存在/文件为空）。**判读顺序：Mavros triggered → Pub goal → Get goal → Load N**，缺哪一环查哪一环——triggered 有了但 goal 没发 = waypoint_mission 侧（文件/格式），goal 发了但没规划 = FSM 侧（吸附/地图/走廊）
      - **终点吸附高度限制 goal_z_min/max（用户"限制吸附高度必须在 0.6-0.825m 之间"）**：`getNearestInfCellNot` 会把终点吸附到奇怪高度（本会话终点 (0.073,-0.048,0.608) 被吸到 z=0.825）。实现：fsm.cpp `setGoalPosiAndYaw` 吸附后 clamp `gi_.goal_p.z()` 到 `[cfg_.goal_z_min, cfg_.goal_z_max]`（参数 `fsm/goal_z_min: 0.6`、`fsm/goal_z_max: 0.825`）；config.hpp `Config` 加两字段 + `LoadParam`。编译 0 错误，fsm_node 100%。**注意**：clamp 是"吸附后限制"不是"跳过吸附"——终点仍在障碍里时 z 被压到边界，不保证可达；根治仍是地图干净
      - **⭐ yaml 重复键覆盖坑（2026-08-11 实机，差点搞坏配置）**：往已有 yaml 用**字符串追加**（`s += '\
fsm:\
  goal_z_min: 0.6...'`）会生成**第二个同名顶层键** → yaml 后键覆盖前键 → **原始 fsm 段全部配置失效**（click_height/waypoint_goal_topic 等全丢）。**教训：改已有 yaml 必须 `yaml.safe_load` → 修改 dict → `yaml.dump` 写回，或精确 replace 原段落；绝不用 += 追加顶层键**。验证：`python3 -c "import yaml; d=yaml.safe_load(open(f)); print(sorted(d.keys()))"` 看键数量；本案例修复=删重复段 + 把参数合并进原始 fsm 段（`timer_en` 行后插入）
      - **"SUPER 知道终点会不会直接飞到终点"（用户架构疑虑）**：不会直线飞——guide_path=完整轨迹 → 走廊沿轨迹生成（SearchPolytopeOnPath）→ MINCO **只能在走廊内优化**（物理约束）。三个注意点：① MINCO 会"切角/拉直"（走廊内平滑，拐角圆滑过渡，特性非 bug）② 终点可能被 getNearestInfCellNot 吸附抬高（用 goal_z clamp 解决）③ 起飞点不对齐轨迹起点时先飞向最近点（看起来绕路）。**验证吸附是否发生**：对比日志 `Pub goal`（原始）vs `Get goal`（吸附后）；先 python 读 guidepoints_new.txt 起终点核对文件本身值，别把文件高度误判成吸附
    - **动态障碍残留细化判读（2026-08-11 用户实测"走一圈大部分消失、小部分不消失"）**：raycasting.enable=true 已生效（大部分能清），残留的小部分 = **激光没重新扫到那些位置**（雷达盲区/被遮挡/扫描间隙，射线式地图固有特性，非故障）——验证法：飞机原地转一圈看残留块是否消失，消失=正常；不消失且残留块在空旷区才是概率参数问题（p_miss 0.49→0.7、p_free 0.499→0.45 可调但会误清真实静态障碍，不建议乱调）。注意 raycasting 参数要改 task_run_scripts 那份 real_flight.yaml（坑 5）
    - **⭐ 路径 A 地面验证故障（未起飞时）：`GeneratePolytopeFromLine failed, force return`（2026-08-11 实机，触发成功后规划死循环）**：现象——OFFBOARD 切成功、CH10 触发成功（`Mavros triggered.` + `Pub goal` 出现）、**FULL GLOBAL PATH 生效**（`Using FULL GLOBAL PATH, idx=55 total=650 pts`），但每轮 replan 都是 `GenerateExpTrajectory SUCCESS`（前端 A* 找到了路径）→ `GeneratePolytopeFromLine failed, force return`（走廊生成失败）→ `generateBackupTrajectory return [FAILED]` → `PlanFromRest failed, try replan` 无限重试。**与 MINCO NaN 的区别**：MINCO NaN 是"走廊生成了但优化数值崩溃"；GeneratePolytopeFromLine failed 是"**走廊根本建不出来**"（在 seed line 上找不到足够大的凸多面体——轨迹附近地图有障碍/未知空间/起点被吸附抬高）。日志里 `box_min/box_max`（走廊范围）+ `seed line`（起点→终点连线）可看：若 seed line 起点 z 明显高于航点高度（如 0.975 vs 航点 0.554），是 FSM 吸附把起点抬高了。**排查顺序**：①先确认地图干净（raycasting.enable:true 让动态障碍能清，且必须改 task_run_scripts 那份 yaml——坑 5）②起飞高度（init_goal/z）与轨迹起点高度对齐（不一致→吸附→seed line 被抬高）③起飞前雷达转一圈清残留障碍块。**注意**：这是路径 A（guide_path 注入）下特有的失败链——FULL GLOBAL PATH 生效不代表走廊能建出来，走廊失败才是关键
      - **⭐⭐ 起飞点被障碍包围 = 走廊失败/飞直线的最终根因（2026-08-11 会话末实机定论，occ 点云解析铁证）**：现象——CH10 触发成功、`Using FULL GLOBAL PATH` 生效、idx 单向推进正常（97→171 不跳段），但**每轮 replan 死循环失败**：`SearchPolytopeOnPath for new path failed` → `Global path blocked by obstacle, fallback to A* replan` → **`The start point in obstacle, this should not happen since the start point should be shift before pathsearch.`**（飞机当前位置在地图里被判占用！）→ `A* fallback path search failed` → `GenerateExpTrajectory failed with FAILED` → 循环。另两条关键日志：**`Cannot find continuous corridor on path, overlap only -326`**（相邻走廊盒子重叠=负数→盒子之间断裂）和 **`Box search failed, box size is zero`**（seed line 附近找不到任何有效盒子——起点空间被障碍挤压）。**与"飞直线"是同一根因的两面**：起点附近有障碍但起点本身不在障碍里 → fallback A* 直线绕行到终点（飞直线）；起点本身被判占用 → A* 也失败 → 完全不动。**验证法（occ 点云解析，可复用的脚本 scripts/occ_probe.py）**：`/fsm_node/rog_map/occ` 是 PointCloud2，`data` 每 16 字节一点（x,y,z float32 + 4 字节 padding，`struct.unpack_from('<fff', data, i*16)`），统计查询点 radius 内占用点数——**本实测：起点 (0,0,0.6) 1m 内 190 个占用点，最近 0.44m 处有垂直障碍面**（x≈-0.38~-0.52、y 对称分布、z 0.38~0.82 = 测试时有人/物体站飞机前方 0.4m）→ 走廊连最小盒子都撑不出来。**正解（操作层面，非代码）**：**起飞点周围清空 1.5m+（人退远、障碍物移开），起飞后再触发**；起飞前雷达转一圈清残留。**`start point in obstacle` 的源码位置（2026-08-11 实读）**：super_planner.cpp:1216-1221——PathSearch 开头 `if (start_type == rog_map::GridType::OCCUPIED || start_type == OUT_OF_MAP) { warn("The start point in obstacle..."); return false; }`，即**起点在地图被判占用/越界就拒绝搜索**。**⭐ 地面触发无效判读（2026-08-11 实机）**：飞机未起飞（`/mavros/local_position/pose` z≈0.0001）时拨杆触发 → 起点 z=0 低于地图下界（occ box_min z=0.1）→ A*/走廊从地面层开始（seed line 起点 z=0.075 接近地面）→ 走廊必然失败 → fallback A* 也因起点越界失败 → 死循环。**判读**：`rostopic echo /mavros/local_position/pose` 看 z——地面触发=无效测试，必须先 OFFBOARD 起飞到 0.6m 悬停稳定再拨杆。**教训**：路径 A 的"沿轨迹飞"完全依赖走廊沿轨迹生成，而起飞点环境（人站多近）直接决定走廊能否生成——排查顺序：走廊失败 → 先查 occ 点云看起点附近占用（5 分钟出结论），再动代码
      - **⭐⭐ 全局路径"最近点取段"跳段 → 飞机飞直线到终点（2026-08-11 会话末实机，最终根因+单向推进修复，已编译 0 错误）**：现象——CH10 触发成功、`Using FULL GLOBAL PATH` 在跑（idx 正常推进 103→131），**但 idx 突然从 131 跳到 775** → 飞机**从悬停起点直线飞到终点**（POS+VEL 坐标序列 x 0.08→1.32, y -0.08→-0.87 恒定方向），完全没走航迹的转弯段。**先核对航迹再下结论**：python 读 guidepoints_new.txt——总路程 4.31m vs 直线距离 1.50m（比值 2.88=有明显转弯），说明航迹不是直线、是飞机抄了近道。**根因**：注入块 `findNearestIdx`（当前位置在全局路径上的最近点）**跳到轨迹中后段**（idx 775≈84% 位置）→ 取段 [775→终点] = 后半段直线 → 前半段转弯被跳过。**为什么跳段**：航迹起点在原点密集绕圈（前 10 点 x≈0,y≈0 附近微动），飞机起飞点 (0.08,-0.08) 离后半段点更近 → 线性扫全段找最近点找到后半段。**修复（super_planner.cpp + super_planner.h）**：① 加成员 `int global_last_idx_{-1}`；② 找最近点**单向推进**——`search_from = (global_last_idx_ > 0) ? global_last_idx_ : 0`（只在上次索引之后找，防跳段/回头），找到后 `global_last_idx_ = nearest_idx`；③ **首次（global_last_idx_<0）搜索限制前 20% 段**（`search_to = std::max((int)(global_path_.size()/5), 2)`，搜索循环 `i < search_to`）——触发时从轨迹起点附近开始。**教训**：起点在原点绕圈的密集航迹 + "全段找最近点" = 跳段风险；取段必须单向推进（沿轨迹顺序），首次限前段。**验证**：飞完后 `grep "Using FULL GLOBAL" log/real_flight.log | head -5` 看 idx 从 ~0 开始（不是 775）
      - **FSM 订阅点云检查（走廊失败/地图不更新的诊断补充）**：`rostopic info /cloud_registered | grep -A5 Subscribers` 看 **fsm_node 在不在订阅列表**——配置了 `cloud_topic: /cloud_registered` + `ros_callback/enable: 1` 但 fsm_node 没订阅 = 启动时订阅失败或 odom 依赖卡住（地图更新还依赖 `odom_topic: /mavros/local_position/odom`，**MAVROS 没跑时地图更新被卡**）→ 地图不更新 → 走廊生成失败。数据链路三连验证：`/cloud_registered` ~10Hz（数据有）+ Subscribers 含 fsm_node（订阅有）+ `/fsm_node/rog_map/occ` 有数据（地图在更新）
    - 排查时先 `ps aux | grep waypoint_mission` 确认节点还活着；若用户按过按钮2 则全部进程已死，日志是历史
14. **日志磁盘满**：`~/.ros/log` 超 1GB → 日志报 `Error: Could not open file for writing` + `WARNING: disk usage in log directory ... over 1GB`。清理：`rosclean purge -y`（只清 ~/.ros/log 旧日志，不影响 task_run_scripts/log/real_flight.log）
15. **同名节点顶替会让新实例也崩（2026-08-11 实机）**：残留旧 global_path_publisher + 新启动的同名节点 → 日志 `shutdown request: [global_path_publisher] Reason: new node registered with same name` → **新实例随后也 FileNotFoundError 退出**（不是只有旧的被杀）。重复启动前先 `pgrep -f global_path_publisher` 清残留；global_path_publisher 崩溃后 FSM 收不到新路径（但旧路径仍缓存）
16. **⚠️ 分支切换会带走 data_guidepoints/（2026-08-11 实机 FileNotFoundError 根因）**：`data_guidepoints/` 是 **feat/4uav-guide 分支跟踪的文件**，切到避障分支后该目录**从工作区消失**（git 按分支切换工作区内容）→ 按钮3 的 global_path_publisher 报 `FileNotFoundError: .../data_guidepoints/guidepoints_new.txt`。**恢复**：`git checkout feat/4uav-guide -- data_guidepoints/guidepoints_new.txt`（只取文件不切分支）。测路径 A 前先 `ls data_guidepoints/` 确认航迹在；**按钮3 无反应/发布器报 FileNotFound 优先查分支 + 目录存在性，别先查代码**
17. **"老是切 HOLD"诊断（2026-08-11 实机）**：先确认是不是真 HOLD——`rostopic echo /mavros/state -n 5` 连续采样看 mode 是否跳变。**地面待机常见是 `AUTO.LOITER`**（遥控器模式开关位置，RC ch5=1950 等），中文界面/遥控器显示可能翻成"悬停/Hold"造成误会，不是故障。**真 HOLD 的两个根因（按 MAVROS 是否存活分）**：
    - **根因 A：MAVROS 进程挂了** → PX4 检测到 OFFBOARD 指令流丢失 → 自动回退 HOLD（安全机制，正常行为）→ 修法是重启 start_fastlivo2.sh 拉起 MAVROS。**判 MAVROS 挂**：`ps aux | grep mavros_node` 无进程 + 所有 `/mavros/*` 话题为空 + `/mavros/state` 无输出。**⚠️ pgrep 误匹配坑**：`pgrep -f "livox_ros_driver2|fastlivo_mapping|mavros_node"` 会匹配到**执行命令自身的 shell/plink 进程**（显示同一个假 PID，`ps -p <pid>` 查无此进程）——多进程 OR 匹配时用 `ps aux | grep -E` 逐个核对，别信 pgrep -f 的多模式结果
    - **根因 B：EKF 定位质量不足（2026-08-11 实机，MAVROS 正常但切 POSCTL 后过一会儿自动回 HOLD）**——室内/无 GPS 场景，PX4 在 POSCTL 模式要求可靠位置估计，EKF 判定定位源不足就安全回退 HOLD。**证据三连**：① `rostopic echo /mavros/estimator_status -n 1` → `pos_horiz_abs_status_flag: False`（无绝对水平定位=无 GPS）+ `pos_vert_agl_status_flag: False`（无离地高度）；② `/diagnostics` → `level: 2` + `name: "mavros: GPS"` + `message: "No satellites"`；③ MAVROS 报 **`navigation error: No valid global position estimate`**（PX4 内部报错，切需要全局位置的模式时直接拒）。**与 QGC "Ready to fly" 的区别**：Ready 是解锁条件检查（IMU/传感器健康），模式切换时 PX4 实时评估定位质量——两套检查，Ready 不代表定位够用。**⭐ 总阀门参数：`EKF2_AID_MASK = 0`**（2026-08-11 实机查证：`rosservice call /mavros/param/get "{param_id: 'EKF2_AID_MASK'}"` → integer: 0）——**AID_MASK 决定 EKF 融合哪些外部源**（bit0=GPS, bit2=视觉位置, bit4=视觉速度），**=0 表示 EKF 只用 IMU 推算、无视一切外部定位**（即使 `EKF2_EV_CTRL=11` 视觉融合已开、`/mavros/vision_pose/pose` 有 FAST-LIVO2 数据、坐标系 TF 全通——**水管都接好了但总阀门关着，数据进不去**）。**推荐值 20**（4+16 = 视觉位置+速度，室内无 GPS 场景）。**⚠️ param/set 可能被 PX4 拒写**：`success: False`（关键导航参数飞行安全校验/固件版本限制），且 param/get 也可能 `success: False`（参数通道异常）——**MAVROS 通道改不了就用 QGC 参数界面改**（QGC 直接和飞控握手，最可靠）。**根治方向**：写 FAST-LIVO2→PX4 桥接（订阅 `/aft_mapped_to_init` → 发布 `/mavros/vision_pose/pose`；MAVROS 视觉通道已就绪，只是没人发数据；现有 `odom_to_mavros.py` 整文件被注释且订阅 /odin1/odometry 是 odin 专用，需新写）+ QGC 配 `EKF2_AID_MASK=20`。**坐标系检查（用户问"有没有可能是坐标系"）**：TF 树里 `map→odom`、`map→camera_init` 静态变换**均存在**（odom_to_map_broadcaster / camera_initial_to_map_broadcaster，零偏移），MAVROS local_position frame=map——**坐标系是通的，别误判为孤儿**；确认方法 `timeout 4 rosrun tf tf_echo camera_init map`（能输出变换=通）
    - **⭐ 定制固件信号（2026-08-11 实机："QGC 中没有这个参数"）**：QGC 参数面板**搜不到 EKF2_AID_MASK** + 一堆 EKF2 参数读出来全 0（`EKF2_ACC_NOISE=0`、`EKF2_GYR_NOISE=0`、`EKF2_IMU_POS_X/Y/Z=0`——噪声=0 意味着 EKF 认为传感器完美，数值病态）→ **这个飞控（微空 nxtpx4v2）不是标准 PX4 参数体系**，可能裁剪了 EKF2 或改用别的定位逻辑。**别指望标准 PX4 文档的参数名/位定义完全适用**；查参数用 QGC 搜（搜不到=固件没有），MAVROS param/get 只能当辅助（且可能 success:False）。**"昨天好今天坏"排查法**：fastlivo2 日志目录按时间戳归档（`log/fastlivo2_20260811_120800_true/`）——`px4_and_others.log` 里有 MAVROS 启动时的**完整 rosparam 转储**（含 `vision_pose/tf/child_frame_id: vision_estimate`、`fake_gps/use_vision: False`、`odometry/fcu/odom_parent_id_des: odom` 等），**对比昨天/今天的该日志**可发现配置差异；对比前先 `md5sum` 相关 yaml（本会话实测：当前 MID360_Fly.yaml 的 md5 与 main 分支 git 版一致=排除文件变更嫌疑）。**EKF 健康判读**：`const_pos_mode_status_flag: True` = EKF 假定"飞机没动"（无外部位置源兜底），`accel_error_status_flag: False` + IMU `z≈9.8` = IMU 健康（排除 IMU 故障）；`velocity_horiz/pos_horiz_rel True` 但 `pos_horiz_abs False` = 姿态/速度都有、唯独缺绝对位置——**问题收敛到"外部位置源没进 EKF"（AID_MASK 或固件裁剪），不是传感器坏了**。**注意别无限排查**：坐标系、IMU、launch、yaml 都验证过没问题后，剩下的就是 PX4 侧参数/固件，直接引导用户在 QGC 确认定位状态图标（右上角卫星图标灰=无定位）收敛
    - **⭐⭐ 避障参数锁死 POSCTL = 本次最终根因（2026-08-11 用户提供参数文件 + .ulg 后确认）**：切 POSCTL 能切但几秒后回 HOLD、MAVROS 报 "No valid global position estimate"，**真凶不是定位而是碰撞预防参数**：参数文件里 `CP_DIST=1`（避障开启）+ `CP_GO_NO_DATA=0`（无数据方向不可飞）+ `obstacle_to_px4` 数据源未运行 → **PX4 认为全向 1m 内都是障碍 → 拒绝位置控制 → 自动回退**。**.ulg 铁证**：`estimator_aid_src_ev_pos.fused=1` 全程 + `estimator_status_flags.cs_ev_pos=1`（视觉融合正常，**彻底排除定位问题**——之前 ros 话题 `const_pos_mode=True` 是 flag 布局误导）。**正解**：不用避障就 `CP_DIST=-1`（负值=关闭，一条 rosservice/QGC 即可）；要用避障必须配齐数据源且 `CP_GO_NO_DATA=1`。**排查顺序修正**：遇 "切 POSCTL 自动回退" **先查 CP_DIST/CP_GO_NO_DATA + obstacle_to_px4 进程**（5 秒），再查定位——本会话在 EKF/AID_MASK/坐标系/视觉上花了大量时间全排除，最后一步才定位到避障参数。详细文档已存机上 `task_run_scripts/scripts/NX_code/obstacle_warning.md`（含排查链/参数速查/检查清单，2026-08-11 提交 ecb0846）
    - **`.ulg` 定位诊断要点（2026-08-11 实机，别信 ros 话题 flag）**：诊断"定位失效"直接分析 .ulg——`estimator_aid_src_ev_pos` 的 `fused`（连续=1 且 `time_last_fuse` 递增=视觉位置持续融合）+ `estimator_status_flags.cs_ev_pos/cs_ev_hgt/cs_ev_yaw`（=1=视觉辅助启用）+ `estimator_aid_src_fake_pos.fused`（=1=EKF 在用"假位置"兜底，与视觉并存是不健康信号）。**`/mavros/estimator_status` 的 status_flag 位布局与 .ulg 的 `estimator_status_flags` 字段不同**，ros 侧显示 const_pos=True 而 .ulg 显示视觉融合正常——**以 .ulg 为准**。nav_state 全程不变 = 模式切换请求从未被接受（`vehicle_control_mode` 无 position/velocity 环佐证）；`valid_nav_states_mask & (1<<bit)` 反查固件允许的模式。详见 px4-ulog-analysis 技能
18. **RViz Path 显示项不显示（2026-08-11 实机，GlobalPath 绿线缺失，最终根因+完整修复）**：话题有数据、FSM 收到（`GLOBAL PATH received: N pts`）、但 RViz 无轨迹线 → **先 `rostopic info /global_path` 看 RViz 在不在 Subscribers 列表**。不在 = 显示项没订阅。两层根因与修复：
    - **第一层（Color 语法）**：Path 显示项 `Color: 0, 255, 0`（逗号）不是 RViz 标准格式 → 显示项静默失效；必须分号 `Color: 0; 255; 0`（提交 174085f）。    - **第二层（文件格式，真正的病根）**：**top_down.rviz 是"压缩单行式"非标准格式**（每行 `- Alpha: 0.5      Cell Size: 1` 字段用多空格挤在一行，老版本 RViz/工具生成）——python `yaml.safe_load` 解析出 `Displays` 数量 = 0！用 python 标准 YAML 多行块**混插**进这种文件 → RViz 解析错乱、显示项全部不生效（即使语法看着对）。**诊断命令**：`python3 -c "import yaml; print(len(yaml.safe_load(open('top_down.rviz'))['Visualization Manager']['Displays']))"` 若 0 或 KeyError = 文件格式非标准。
    - **根治**：用 python `yaml.dump` **整体重建标准格式 top_down.rviz**（9 个显示项：Grid/GlobalCloud/LocalCloud/Occ/Waypoints/GlobalPathMkr/Goal/FrontendPath/ExpTraj，Fixed Frame=map），旧文件备份 .bak_oldformat。重建后重启 SUPER（按钮2→按钮3）让 RViz 加载新文件。
    - **兜底（显示最可靠方案）**：**MarkerArray 比 Path 显示项可靠**——global_path_publisher.py 额外发布 `/global_path_marker`（MarkerArray LINE_STRIP，绿色线宽 0.08，ns=global_path，latch），RViz 用 `rviz/MarkerArray` 显示项订阅它（Waypoints 同款，已被验证稳定）。这是最终生效的显示方式。
    - 注意：**rviz 启动早于显示项加入配置时加载的是旧配置**（对比 rviz 进程 lstart vs top_down.rviz mtime 可判）；改完 top_down.rviz 必须重启 SUPER；别在运行的 RViz 里手动加显示项（重启会丢）
19. **⚠️ pkill/pgrep 自杀坑（2026-08-11 实机，exit 128）**：用 plink -m 执行的脚本若**命令行/脚本内容含目标进程名字符串**（如脚本里写 `pkill -f "global_path_publisher.py"` 而脚本文件本身叫 super_kill_pub.sh 但**内容含该字符串**，或 `pgrep -f "python3.*global_path_pub"` 匹配到执行脚本自身的 bash -c 命令行）→ **把自己杀了**，plink 返回 exit 128。规避：① `pkill -f` 用**更精确的子串**（如 `scripts/global_path_publisher.py` 完整路径，脚本命令行通常不带 scripts/ 前缀）② 或用 `/proc/<pid>/cmdline` 逐个核对再 kill（`pgrep -x python3` 后 `tr '\0' ' ' < /proc/$p/cmdline | grep 目标`）③ plink -m 脚本里避免出现会被 -f 匹配的模式。重启 global_path_publisher 或 nx_command_server 时都适用（坑 11 同理）
20. **⭐ FAST-LIVO2 视觉配置 img_en=0 + img_topic 不匹配（2026-08-11 实机，VIO 从未工作的根因）**：查 `config/MID360_Fly.yaml`（和 Fly_Lidar 版）——
    - `img_topic: "/usb_cam/image_raw"`（第 8 行）但 D435 实际发布 `/camera/color/image_raw`（30Hz）→ **话题名不匹配，FAST-LIVO2 订阅不到图像**；`img_en: 0`（第 11 行）→ **视觉融合被关闭**。两个配置（Fly 飞行版注释写"视觉+雷达融合 img_en=1"但实际值 0）**所有分支（main/避障/4uav-guide）一致，是提交 7b83f8c 起就写死的**——不是最近改坏的。
    - **验证链**：`rostopic hz /camera/color/image_raw`（~30Hz ✅ 相机正常）vs `rostopic hz /rgb_img`（**no new messages** = FAST-LIVO2 视觉输入断）；`rostopic info /camera/color/image_raw` 的 Subscribers 只有 rviz 没有 laserMapping（FAST-LIVO2 没订阅）；FAST-LIVO2 日志里 `img_cbk`/`img_en` 相关行 = 0（两天日志都加载了 cam_model/cam_fx/cam_fy 相机内参但从不消费图像）。
    - **结论**：这套系统实际跑在**纯 LiDAR+IMU 定位**（FAST-LIVO2 的 LiDAR 里程计），**视觉里程计(VIO)从未工作**；`/mavros/vision_pose/pose` 的内容是雷达里程计位姿（frame_id 硬编码 camera_init，LIVMapper.cpp:1402）。**若想真正用视觉融合**：改 img_topic=/camera/color/image_raw + img_en=1（改完重启 FAST-LIVO2，不是重启 SUPER）；但改前确认 D435 相机标定参数（cam_fx/cam_fy 等，mid360_imu_tf 里的外参）。
    - **"昨天好今天坏"排查法（两天对比）**：fastlivo2 日志按时间戳归档（`log/fastlivo2_20260811_120800_true/` 是昨天建图模式 true、`fastlivo2_20260811_162056_false/` 今天飞行模式 false），**对比两天的 `fastlivo.log` 里 img/vision 相关行数**（本实测均为 0 = 两天视觉状态一致 → "坏"不是视觉断线导致，收敛到 PX4 侧模式/参数）；`md5sum` 相关 yaml 对比 main 分支 git 版排除文件变更嫌疑。
21. **启动雷达后 RViz 没点云但数据链路正常（2026-08-11 实机）**：先验数据再查显示——`rostopic hz /livox/lidar` 与 `/cloud_registered` 都 ~10Hz = 数据没问题，问题在 RViz。**`rviz/slam_rviz.rviz` 是 odin 设备旧配置**：订阅 `/odin1/cloud_raw`、`/odin1/cloud_render`、`/odin1/cloud_slam` 等不存在话题（只有第 273 行附近一个 `/cloud_registered` 显示项有效且常被折叠在 slam 组里）→ MID360 场景点云显示用 **top_down.rviz**（slam 组里唯一的 /cloud_registered 项要勾选 + Size (Pixels) 调 3~5）。**判读法**：`rostopic info <话题> | grep -A4 Subscribers` 看 RViz 在不在；在但没画面 = 显示项 Enabled=false / Size 太小 / 组被折叠 → 取消勾选再勾上强制刷新订阅
22. **RViz Fixed Frame 设为 map 无点云、改 camera_init 就显示 = map frame 消失（2026-08-11 实机，用户问"为什么坐标系调为 camera_init 就可以了"）**：FAST-LIVO2 所有点云 frame_id 硬编码 `camera_init`（LIVMapper.cpp:1402）。Fixed Frame=map 时 RViz 需要 `map↔camera_init` TF 变换才能显示；**该变换由 `px4_and_others.launch` 里的 `static_transform_publisher`（camera_initial_to_map_broadcaster，零偏移）发布**——**px4_and_others.launch 没启动（MAVROS 没跑）→ map frame 从 TF 树消失 → Fixed Frame=map 报 `"map" passed to lookupTransform ... does not exist` → 点云不显示**。切 camera_init 就显示 = 点云自己的 frame 无需变换。**诊断**：`timeout 4 rosrun tf tf_echo map camera_init` 报 `Exception: "map" ... does not exist` + `ps aux | grep mavros_node` 无进程 = MAVROS 没启动。**修复**：`roslaunch task_run_scripts px4_and_others.launch`（拉起 MAVROS + map→camera_init 静态 TF）。**注意**：map frame 消失还会连带影响别处（如 RViz 其他以 map 为 Fixed Frame 的显示、依赖 map 的 TF 查询），先拉起全套再判断显示问题
23. **px4.launch 的 gcs_url 被改动 = QGC 连不上飞控（2026-08-11 发现、08-12 提交确认）**：`launch/px4.launch` 里 `<arg name="gcs_url" default="udp-b://@" />`（MAVROS 地面站广播监听，udp-b=广播模式，QGC 能连）被改动。**⚠️ 两种改动形态，诊断方向不同**：① 被注释（`<!-- ... -->`）→ gcs_url 空/无效 → QGC 完全连不上；② **被替换成固定 IP**（本会话实机 diff：`udp-b://@` → `udp://:14550@<NET_GCS>.123:14550`）→ 只连指定 IP 地面站，QGC 默认广播连不上（除非地面站就是该 IP）。**排查 "QGC 连不上" 先 `git diff launch/px4.launch` 看 gcs_url 是哪一种改动**；需要 QGC 就恢复 `udp-b://@`（或注释掉固定 IP 行），只用遥控器/MAVROS 则无所谓。**⚠️ 提交前必须和用户确认这个改动是否故意**（本会话 diff 后问用户，用户确认"故意改的"= 连固定地面站 <NET_GCS>.123，**已提交 1bb7d24**——不要默认是误改去恢复）。注意 px4.launch 是配置文件，map frame 消失是 px4_and_others.launch 没跑（坑22），两者独立
24. **0.6m 短距离导航测试的参数调整（2026-08-11 用户"悬停前进不超过 0.6m"，方案给出）**：短轨迹两个坑——MINCO 短轨迹优化 NaN（0.6m 目标 + 大速度高 NaN 风险，用户此前 46.3s 切段即栽在短轨迹）+ 速度太快（max_vel 2.0 会让 0.6m 在 0.3s 内飞完）。**参数修改**：`real_flight.yaml` 的 `max_vel` 2.0→**0.3**、`max_acc` 4.0→**1.0**；`super_to_px4_bridge.yaml` 的 `velocity_ff_gain` 0.7→**0.3**（前馈减小防短距超调）；`switch_dis` 0.1→0.05 可选。**操作注意**：起飞高度 init_goal/z=0.6 vs 航迹起点 z≈0.55 → 拨杆后先降 5cm 再前进（正常）；起飞点必须对齐轨迹起点（±0.3m），否则规划器先绕回起点。**更稳的方案**：0.6m 太短不适合测完整路径 A——**先做纯悬停**（OFFBOARD→bridge 起飞 0.6m→拨杆→目标=起飞点附近，验证触发+规划链路，不前进），**或录 2-3m 低高度轨迹再测**（0.6m 对 MINCO 太短）。飞行安全：悬停验证链路通了再录长轨迹测前进
25. **会话末段判读三点（2026-08-11 实机，路径 A 反复"飞直线/不动"后沉淀）**：
    - **"RViz 里的轨迹线是历史规划遗留，不代表新测试"**：RViz 的 exp_traj/frontend_path 显示**最后一次规划结果**（静态保留，不是实时流）——系统待机未触发时看到"直线/旧轨迹"只是上一次失败的遗留，**不能据此判断新一次会怎样**；判读必须以新触发后的 real_flight.log 为准（`Mavros triggered` → `Pub goal` → `Get goal` → `Using FULL GLOBAL` → 走廊成败）。同理 `/global_path` 用 `rostopic echo -n 1` 数 poses length 时若 grep 计数为 0，先怀疑输出格式解析（PoseStamped 数组的 x 字段计数方式），用 python 数 `position:` 出现次数核对，别误判发布器挂了
    - **"人退开验证占用源"（occ 探针的活用）**：起点 1m 内 190 个占用点 → **让操作员退到 3m 外等 10 秒再解析一次** → 若变 0 = 占用源是操作员本人（人=障碍，测试站位问题，非代码问题）；若不变 = 静态障碍/机架被扫到。这是区分"人挡路" vs "机架自扫"的 1 分钟判别法（脚本 scripts/occ_probe.py 支持任意查询点 radius 统计）
    - **系统健康四连查（走廊失败时先确认系统活着再深挖）**：① `rostopic info /cloud_registered | grep -A5 Subscribers` 看 **fsm_node 在订阅列表**（配置了 cloud_topic 但没订阅 = 启动失败）② `/cloud_registered` ~10Hz（数据有）③ `/mavros/local_position/odom` ~60Hz（地图更新依赖，MAVROS 挂则卡）④ `/fsm_node/rog_map/occ` 有数据（地图在更新）。**四连全绿 = 系统链路通，走廊失败是地图内容问题（起点环境/占用）而非链路断**；全空 = 系统已关（测试完停了），日志是历史
    - **⭐⭐ 起飞后走廊仍失败 = 排查进入"未知区域/box 范围"层（2026-08-11 会话末实机，排除地面触发假设后）**：依次排除了——地面触发（起飞到 z=0.76 再触发）、goal 被吸附（`Get goal at 1.275 -0.825 0.825` 正常，clamp 生效）、点密度（`Downsampled 39 pts` 生效）、链路（四连全绿）——**起飞+goal 正常+降采样生效，`GeneratePolytopeFromLine failed` 依然无限循环**。此时看走廊失败细节两个新判读点：
      - **① box 范围异常 = 搜索窗口含大量未知区域**：`box_min x=-2.125 y=-1.075 z=0.1 / box_max x=1.075 y=3.475 z=1.575`——**y 上限 3.475 远超航迹范围**（本航迹 y 最大 0.44）→ 走廊搜索窗口里有大片**激光没扫到的未知空间** → 走廊生成器把未知当障碍（或找不到已知空闲多面体）→ 失败。**对比法**：box 范围 vs 航迹 x/y/z 范围（python 读 guidepoints_new.txt），box 明显大于航迹 = 未知区域嫌疑
      - **② `frontend_in_known_free: false` = A* 与走廊生成器对未知的处理不一致**：super_planner.cpp:1245 `int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);`——**false 时 A* 把未知当空闲（能搜出直线路径→GenerateExpTrajectory SUCCESS），但走廊生成器（corridor_generator）对未知区域生成不出多面体** → `GenerateExpTrajectory SUCCESS` 与 `GeneratePolytopeFromLine failed` 同时出现 = A* 过了但走廊崩 = 典型"未知区域"特征。改 `frontend_in_known_free: true` 会让 A* 也在未知区失败（更糟），**正解是让激光充分扫描飞行路径区域（起飞悬停后转一圈飞机 / 等待地图覆盖）再触发**，或确认 RViz 里飞行路径方向是"已知空闲"（灰色）而非"未知"（黑色）
      - **教训**："起飞后仍走廊失败"的三层排查顺序——① 链路四连（系统活着吗）② 降采样/吸附（guide_path 干净吗）③ **box 范围 vs 航迹范围对比 + frontend_in_known_free（地图覆盖够吗）**；前两层全过仍失败 = 地图覆盖/未知区域问题，**先让雷达扫清路径方向再触发，别急着改代码**


26. **⭐ 地图可视化话题家族 + 未知区域显示（2026-08-11 会话末，用户问"RViz 看到的障碍分布和飞机实际一样吗"时产出）**：
    - **ROG-Map 发布 4 个 PointCloud2 话题，全部 16B/点（xyz float32 + padding）**：`/fsm_node/rog_map/occ`（占用）、`/fsm_node/rog_map/inf_occ`（无限地图占用）、`/fsm_node/rog_map/unk`（未知）、`/fsm_node/rog_map/inf_unk`（无限地图未知）。**unk 话题可用于在 top_down.rviz 加显示项**（Add → PointCloud2 → Topic=/fsm_node/rog_map/unk，橙色区分障碍）——**一眼看出轨迹方向是"未知雾"（橙）还是"真障碍"（白/彩）**，是走廊失败排查的关键可视化
    - **判读：RViz 显示的障碍 = 规划器眼中的障碍（同一 ROG-Map，Occ 显示项订阅 occ）**，但有三个差异会误导：① 只显示 p_occ>0.85 阈值的点，低于阈值的"可疑障碍"不显示但走廊仍受影响 ② **未知区域（黑色/空白）不显示障碍块——你看到的"干净"可能是"未扫描"而非"空闲"**（走廊失败主因之一）③ top-down 俯视切片可能遮低空障碍。**区分"未知"vs"空闲"**：RViz 里灰色/有网格=已知空闲，黑色/空白=未知未扫描
    - **走廊生成机制（用户问"走廊怎么产生的、失败原因"，2026-08-11 讲解版）**：① 输入 guide_path（降采样后）→ ② 沿路径切 seed line（每段种子线）→ ③ 每段用 `maxVolInsEllipsoid`（最大内接椭球）从中心向外扩凸多面体，扩到碰到障碍/未知/地图边界为止 → ④ 检查相邻盒子 overlap>0（连续）→ 全部通过=走廊成功。**MINCO 只能在走廊内优化**（物理约束，不会直线穿出）。**失败分类表**：`Box search failed, box size is zero`=某段找不到任何空闲盒子（起点被障碍/未知包住）；`Cannot find continuous corridor, overlap -326`=相邻盒子完全没重叠（断裂，常见于点过密微段）；`maxVolInsEllipsoid failed`=盒子扩不出来（周围障碍/未知多）；`The start point in obstacle`=起点在地图占用格（见坑 13）；`OUT_OF_MAP`=路径超地图边界。**本质一句话**：走廊失败 = 路径周围"已知空闲空间"不够（障碍多→撞墙，未知多→撞雾，点密→断裂，起点被占→第一个盒子都建不了）。**排查顺序沉淀**：走廊失败 → ①链路四连（系统活着）②降采样/吸附（guide_path 干净）③occ 探针看起点占用 ④**RViz 加 unk 显示项看轨迹方向是未知还是障碍** → 未知多=先让雷达扫清再触发，别急着改代码
    - **⭐ unk 话题默认不发布（2026-08-11 会话末实机，加了显示项也看不到的坑）**：`/fsm_node/rog_map/unk`（未知区域）**默认无数据（no new messages）**——根因是 real_flight.yaml 第 172 行 `rog_map/visualization/pub_unknown_map_en: false`（FSM 启动日志 `Load param rog_map/visualization/pub_unknown_map_en success: 0` 可证）。**加了 RViz 显示项但无橙色点 = 先查这个开关**（改 true 后重启 FSM/按钮3 生效，提交 dc596eb）。**同族话题发布状态对比判读**：只 `inf_occ` 2Hz 在发、occ/unk/inf_unk 全 `no new messages` = 可视化配置只开了部分话题（常见于 FSM 刚启动或配置关闭）；`rostopic hz` 逐个话题确认哪个活着。**注意**：`pub_unknown_map_en` 是 ROG-Map 可视化开关，与 `visualization_en`（SUPER 规划可视化，real_flight.yaml:29）是两回事
    - **Unknown 显示项已落地（2026-08-12 提交 c7ccf9c）**：top_down.rviz 已加 `Name: Unknown` 的 PointCloud2 显示项（Topic=/fsm_node/rog_map/unk，橙色 255;128;0，Size(Pixels)=2，Use rainbow:false）。**⚠️ 注意两个 RViz 配置**：用户实机界面是 **slam_rviz.rviz**（订阅 /odin1/* 旧话题，**未加 Unknown 显示项**），top_down.rviz 只有按钮3 启动 SUPER 时才加载——**"看不到橙色"先确认当前 RViz 加载的是哪个配置**（`ps -eo lstart,args | grep 'rviz -d'`）+ 该配置有没有 Unknown 项。**occ/unk 二进制解析**：`data` 每 16 字节一点（xyz float32 + 4B padding），`struct.unpack('<fff', chunk[:12])`——实测 `unk` 12 万点、飞机 (0,0,0.6) 1m 内 443 未知点 + occ 1m 内 193 占用点（最近 0.39m 垂直面=人站飞机前）→ 起点被障碍+未知双重包围=走廊失败（scripts/occ_probe.py 支持任意查询点）

27. **⭐⭐ "前半段沿轨迹正常、后半段直线到终点"（2026-08-12 实机，走廊成功后 MINCO 拉直 + idx 追平）**：现象——起飞 0.6m 触发后，**idx 单调推进（110→204）且 POS+VEL 沿轨迹**（前半段循迹成功 ✅），但 **idx 突然从 204 跳到 764**，飞机从 (0.95,-0.01) 斜切到终点 (1.27,-0.82)（后半段直线）。**关键特征：全程无 `GeneratePolytopeFromLine failed`、两次 `GenerateExpTrajectory SUCCESS`——走廊生成成功了，但轨迹还是直线！** 判读两步：
    - **① idx 跳变 = "飞机位置追平"不是跳段 bug**：idx 推进速度（每轮 replan 找最近点）< 飞机实际飞行速度 → 飞机已飞到 idx 764 对应的位置 (1.27,-0.82) → 最近点自然跳到 764（单向推进只防"回头"，不防"追平"，这是正常行为）。判据：idx 跳变后的值对应的航迹坐标 ≈ 飞机实际位置（python 读 guidepoints_new.txt 对比）
    - **② 飞机斜切 = MINCO 在走廊内"拉直"（真正的根因）**：对比航迹形状——航迹 idx 250~600 是向右凸出段（x 到 1.73 再绕回，idx 400=(1.731,0.054)），但飞机从 (0.95,-0.01)（≈idx 204）直接斜切到终点方向，**跳过了整个凸出段**。**MINCO 在走廊（凸多面体）内优化出最短轨迹（走弦/捷径），走廊盒子足够大时拉直可以严重到跳过整个弯曲段**——不再是"拐角圆滑"（坑 13 里说的特性），而是明显抄近道。**教训：走廊生成成功 ≠ 沿轨迹飞行**；沿轨迹程度由走廊盒子大小决定（盒子小→MINCO 无法斜切→贴轨迹；盒子大→MINCO 走捷径→跳过弯曲段）
    - **解决方向（会话末给出，未实施）**：a) **收紧走廊**——corridor_bound_dis 调更小 / `seed_line_max_length`（corridor_generator 分段长度，real_flight.yaml:37 当前 1.2）调小，让 MINCO 在盒内没有斜切空间；b) **改 MINCO 权重**——路径距离项加大，惩罚偏离 guide_path；c) 接受平滑化（若用户接受"大致沿轨迹"）。**先问用户"完全贴轨迹还是允许圆滑"再动手**——完全贴轨迹要改 MINCO 权重或走廊更紧，允许圆滑则现状可接受
    - **判读命令**：`grep -a "Using FULL GLOBAL" log/real_flight.log | tail -30` 看 idx 序列（找跳变点）；`grep -a "POS+VEL" log/real_flight.log | tail -25` 看飞机实际轨迹；python 读 guidepoints_new.txt 的 idx 150~900 坐标对照"航迹该往哪走" vs "飞机实际去哪"——**飞机路径与航迹形状分歧 = MINCO 拉直**；若同时有走廊失败日志 = 先按坑 13/25 查走廊，本坑是"走廊成功但拉直"，两者不同

28. **⭐⭐ MINCO 贴路径机制 = waypoint_attractor/dead_d（"完全沿轨迹"的控制点，2026-08-12 源码深挖，用户明确"完全沿着轨迹"后定位）**：MINCO 是否斜切由优化器的"航点吸引子"机制决定（exp_traj_optimizer_s4.cpp）：
    - **`waypoint_attractor` = 每个走廊盒子的中心**（`opt_vars.waypoint_attractor.col(i) = curIV.colwise().mean()`，384 行——盒子顶点均值，**不是 guide_path 点**！盒子是方形，中心可能与路径点偏离）
    - **`waypoint_attractor_dead_d` = 死区半径** = `geometry_utils::findInteriorDist(相邻盒子重叠区) / 2`（重叠区内切半径的一半）
    - **代价公式（140-147 行）**：`p_a = pos - attractor; 代价 += |p_a|² - dead_d²`——**轨迹点在死区半径内零代价（MINCO 可自由斜切），超出死区才被拉回**。盒子大 → dead_d 大 → 斜切自由度高 → 跳过弯曲段；盒子小 → dead_d 小 → 轨迹贴盒子中心序列
    - **改"完全沿轨迹"的两条路（会话末给出，未实施）**：a) `dead_d` 压小（如 `opt_vars.waypoint_attractor_dead_d(i) = 0.02`，一行改动，强制轨迹路过每个盒子中心附近）——注意吸引子是盒子中心不是路径点，压死区后轨迹贴"盒子中心序列"≈贴路径（盒子小则偏差有限）；b) 改 attractor 来源为 guide_path 点（要改优化器接口，复杂）。**风险**：dead_d 压太小会让 MINCO 优化变硬（轨迹沿折线、机动性差），且盒子中心偏离路径时轨迹仍偏——先试 a) 看效果
    - **走廊生成器源码定位（corridor_generator.cpp）**：`SearchPolytopeOnPath`（55 行起）主循环——① **`isOccupiedInflate(path[first_id])` 起点被膨胀占用则 first_id++ 跳过**（起点占用 → 走廊从移位后的点开始，与飞机脱节）；② `isLineFree(path[first_id], path[j], seed_line_max_length_)` 判断路径段无碰撞来决定 seed line 分段长度（**与点密度无关**——降采样不改变 isLineFree 判断，降采样不是走廊失败根因）；③ 每段 `GeneratePolytopeFromLine` → `boxSearch(box_min, box_max, OCCUPIED, pc)` 搜盒内占用 → **`if (pc.empty())` 直接返回"路径点 ±bound_dis 的方块"**（337-345 行——盒内无障碍时盒子就是 bound_dis 大小）；④ 相邻盒子 `CrossWith` + `interior_depth < min_overlap_threshold_`（= cfg_.resolution）则失败
    - **corridor_bound_dis 与 robot_r 的精确语义（2026-08-12 源码确认）**：`getSeedBBox`（198-205 行）`box_min -= Vec3f(bound_dis_...)`——bound_dis 是"搜索盒"沿路径点的扩展范围（**不是安全距离**）；`robot_r` 只用于 z 方向收缩（`box_min.z() += robot_r_` / `box_max.z() -= robot_r_`，214-215/310-311 行）——**水平安全由 IRIS 扩盒时遇障碍自动内缩 + min_overlap_threshold 保证**。所以调 corridor_bound_dis 不影响碰撞安全（安全由 robot_r=0.25 保证），调小更保守
    - **用户决策链沉淀（2026-08-12 上午已实施+编译 0 错误）**：先移除降采样（还原 930 点全量，判断降采样抹掉细节）+ `corridor_bound_dis: 1.0→0.39`。判读：日志有 `Using FULL GLOBAL PATH` 无 `Downsampled` 行 = 降采样移除生效；`Load param super_planner/corridor_bound_dis success: 0.39` = 配置生效（需重启 FSM）。**后续实飞结果**：前半段沿轨迹正常（idx 单调 110→204、POS+VEL 贴合航迹），后半段仍直线——走廊成功但 MINCO 拉直（本坑上文），于是走上 dead_d 方案
    - **⭐⭐ dead_d 钳制已实施（2026-08-12 中午，编译 0 错误 fsm_node 12:10）**：改 `exp_traj_optimizer_s4.cpp` 两处 `dis = geometry_utils::findInteriorDist(curIH, interior) / 2;` 后加 `dis = std::min(dis, 0.1);`（需 `#include <algorithm>`）。**⚠️ dead_d 取值三档实测（2026-08-12 全天）**：0.39（原值）= MINCO 在盒内自由斜切跳过弯曲段；**0.1 = 太紧，MINCO 优化崩溃**——日志 `[ExpOpt] Opt failed, Omg or thr or Pos violation`（角速度/推力/位置违反硬约束）+ `[MINCO] TrajOpt failed` + `OptimizationExpTrajInPolytopes for new path failed` 无限循环，飞机前半段沿轨迹（沿用上次成功轨迹）后半段停/乱转；**0.2 = 折中甜点（用户拍板，2026-08-12 已实施 `std::min(dis, 0.2)` 编译 0 错误 fsm_node 12:28）**——比 0.1 松（MINCO 不崩）、比 0.39 紧（减少斜切）。**教训：dead_d 不是越大越好也不是越小越好——太大斜切、太小 MINCO 硬约束冲突（Opt failed），中间值要实飞试；调完看日志有无 `Opt failed`（崩=太小）+ POS+VEL 是否斜切（斜切=太大）**⚠️ 关键：两处都要改，且实际生效的是第二处**——① ~386 行在 `processCorridor()` 里（default_init=true 才走，**本版本会 throw "Not support default init"**，死代码）；② ~459 行在 **`processCorridorWithGuideTraj()` 里（default_init=false 默认路径，super_planner 调用的就是它）**——只改 386 行=白改。验证：`grep -c "dis = std::min(dis, 0.1);"` = 2 处。**语义再确认**：WithGuideTraj 版吸引子=重叠区内切点（`opt_vars.waypoint_attractor.col(i) = interior`，不是盒子顶点均值），且第 3 步用 guide_path 对吸引子做时间/航点热初始化（480 行附近 `(opt_vars.guide_path[i] - attractor).norm()`）——**这正是"沿 guide_path"的版本，dead_d 压 0.1 后 MINCO 必须贴 attractor（≈guide_path 采样）0.1m 内**。**判读**：重启后 `Load param` 无 dead_d 参数（无 yaml 开关，硬编码）；行为验证看 POS+VEL 是否在 L 形转弯处清晰转折而非斜切
    - **⭐ L 形航点生成（2026-08-12 用户"先前进再右移、z 不变"，替代原回形航迹）**：原 930 点航迹在转弯处有重叠（idx 200~700 x=1.7 来回绕 + z 0.56→0.89 变高），用户要简化成 L 形。生成脚本要点：**0.1m 间距均匀采样**、**z 恒定**、**7 列 CSV 格式（x,y,z,qx,qy,qz,qw，四元数 0,0,0,1）与旧文件一致**、同步更新 `endpoint.txt`（空格 4 列）。**⭐ x 上限按测试空间定（2026-08-12 用户"你的x限制在1.6m吧"——防靠墙）**：最终 L 形 = 前进段 x 0→**1.6m** @0.1m（17 点）+ 右移段 y 0→-1m @0.1m（10 点）= **27 点**，转弯点 **idx=16 (1.6, 0, 0.75) 是明确的中间点**（既是前进段终点也是右移段起点）。**测试航迹范围必须对照实际空间**：航迹四周至少留 0.5~0.7m 无障碍（飞机半径 0.25 + 走廊搜索盒 0.39），终点/转弯方向靠墙就有走廊失败+贴墙绕行风险。**⚠️ 最终状态（2026-08-12 同日下午实机）**：用户先要求 **z 统一到 0.75**（`z=0.750000`，endpoint.txt 同步），随后要求**恢复旧录制航迹**（930 点回形，起点 -0.001,0,0.566 → 终点 1.205,-0.836,0.860，z 0.557~0.888）——**当前 guidepoints_new.txt = 旧航迹**，L 形备份在 `guidepoints_new.txt.bak_lshape` 可回退；endpoint.txt 同步回旧终点。**教训**：改用户航点文件前先问清"保持录制高度 vs 统一固定高度"（用户说"z高度不变"有歧义），改完保留 .bak 备份供回退；恢复旧航迹 = 从 `data_guidepoints/guidepoints_YYYYMMDD_HHMMSS.txt` 时间戳归档复制回 guidepoints_new.txt + 重算 endpoint.txt。**教训**：录制航迹的质量直接影响测试——转弯重叠/高度起伏的航迹会让"沿轨迹"判读混乱（分不清是 MINCO 拉直还是航迹本身重叠），**测试用航迹要刻意简单**（L 形/直线+单转弯），验证通过再录复杂航迹。判读：`grep "Using FULL GLOBAL" | head -5` idx 应从 ~0 开始。**⚠️ 高度语义澄清（用户"z高度不变"的歧义）**：改用户航点文件前必须先确认——"保持录制时每个点的 z 曲线" vs "统一到某个固定高度"（0.6/0.75）是完全不同的操作；本会话先误改为 0.6 恒定，用户澄清要 0.75，又改回恢复旧航迹——三次返工。**先问清再改，改完立即验证 z 集合 + 告知用户改了哪些值**
    - **⭐⭐ 终点段 MINCO NaN = goal 吸附与 guide_path 终点不一致（2026-08-12 实机，"终点附近乱转"最终根因）**：idx 推进到最后一段时（`Using FULL GLOBAL PATH, idx=52 total=1`，total=1 即只剩终点自己），日志 `[MINCO] TrajOpt failed, The function value became NaN or Inf` + `[ExpOpt] Opt failed, Omg or thr or Pos violation` → `OptimizationExpTrajInPolytopes for new path failed` 无限循环 → 飞机在终点附近反复重飞乱转（POS+VEL 在 (1.57,-0.97)→(1.69,-1.1)→(1.28,-0.83) 来回摆）。**根因**：**两个"终点"不一致**——guide_path 终点 = 航迹文件值（z=0.75），而 goal 被 `getNearestInfCellNot` 吸附成 z=0.825（`Get goal at 1.575 -0.975 0.825`，吸附限制 [0.6,0.825] 只压上限没压到一致）→ MINCO 要同时满足"轨迹终点=guide_path 终点(0.75)"和"目标=goal(0.825)" → 约束冲突 → NaN。**解决方向（会话末给出未实施）**：`fsm/goal_z_max` 从 0.825 降到 **0.76**（≈航迹 z=0.75），让吸附后的 goal 与 guide_path 终点一致 → MINCO 无冲突。**MINCO 失败两种判读区分（关键）**：`grep -a "Opt failed\|TrajOpt failed"`——**`Omg or thr or Pos violation` = 死区太紧（dead_d 太小，轨迹动力学达不到盒子中心序列）**；**`The function value became NaN or Inf` = 终点约束冲突（goal 吸附值 vs guide_path 终点不一致）**——前者调大 dead_d，后者调 goal_z_max 让终点一致，别混为一谈
    - **⭐ 圆角转弯航点生成（2026-08-12 用户"直角转弯处设置为平滑曲线 + 点密度到110"）**：L 形直角 → 圆弧过渡（半径 R=0.3m，圆心在转角内侧 = (直线1末端x, 直线2起点y) = (1.3,-0.3)）。生成要点：直线段 0.032m 间距 + **圆弧从 90° 扫到 0° 每 2° 采样**（`px = cx + R*cos(θ), py = cy + R*sin(θ)`；θ=90° 落在直线1末端 (1.3,0)，θ=0° 落在直线2起点 (1.6,-0.3)）→ 总 109 点 ≈ 目标 110。**目的**：圆弧过渡让 MINCO 无急转弯（转弯急=dead_d 硬约束冲突 Opt failed），轨迹平滑。**注意**：圆弧半径 0.3 意味着直线段实际只到 1.3（留 0.3 给圆弧），总路径 = 1.3 直线 + 0.47 弧 + 0.7 直线；生成后 `python` 验证点数/起终点/z 集合（z 恒 0.75）。**点到 110 的密度迭代法**：先粗（0.1m/5°）看点数，再按比例收紧直线间距，弧保持 2° 密采样（弧段点数=46 固定，直线段补齐到目标）
    - **⭐ 航迹穿过障碍会怎样（2026-08-12 用户"如果航迹在障碍物中，会发生危险吗，会避让吗"）——三层保护，不会硬撞**：① **走廊生成 isLineFree 检查**（corridor_generator.cpp 主循环 `isLineFree(path[first_id], path[j], ...)` 失败 → 该段走廊生成失败，不沿"穿墙路径"规划）；② **fallback A* 绕行**（走廊失败 → `Global path blocked by obstacle, fallback to A* replan`，global_skip_this_round_ 每轮重置，障碍消失自动恢复沿轨迹）；③ **MINCO 走廊约束**（轨迹只能在凸多面体内优化，物理上出不了走廊）。**安全结论**：航迹穿已知障碍 = 走廊失败 → A* 绕行，安全；**航迹穿未知区域（激光没扫到）** = frontend_in_known_free=false 时 A* 把未知当空闲可能直穿"雾" → **起飞前让雷达扫清轨迹方向（转圈/等待）是唯一兜底**——这就是"扫雾"建议反复出现的原因
29. **⭐⭐ 在别处先启动雷达 → SLAM 地图原点错位 → 飞机显示位置偏移 → 起点不动/走廊失败（2026-08-12 实机，用户"是不是因为我在其他地方先启动了雷达导致的"）**：现象——触发成功、`Using FULL GLOBAL PATH` 生效、但飞机在起点不动，`GeneratePolytopeFromLine failed` 无限循环；查 `/mavros/local_position/pose` 发现**飞机位置 (x=-1.10, y=2.49) 离测试起点 (0,0) 有 2.7m**（POS+VEL 最后记录 (0.92,-0.55)，位置跳变）。**根因**：FAST-LIVO2 的 SLAM 地图原点 = **雷达启动时的位置**——在别处（A 点）启动雷达 → 地图原点=A → 搬到测试点 B 放飞机不重启雷达 → 地图仍以 A 为原点 → 飞机在坐标系里显示为 A 相对 B 的偏移 → 走廊生成器在"错误起点"（A 区域）找空闲 → A 区域没扫/有障碍 → 走廊失败 → 不动。**判读铁证**：飞机"应该"在 (0,0) 但 `rostopic echo /mavros/local_position/pose` 显示明显偏移值（>1m）= 坐标系错位。**解决**：**在测试点重新启动雷达/重启 FAST-LIVO2**（坐标系归零到测试起点）；或航迹文件整体平移 A→B 偏移（不推荐，重启更干净）。**测试纪律**：先放好飞机（测试起点）再启动雷达，或启动后核对 `local_position/pose` ≈ (0,0) 再触发——**"起点不动"先查飞机显示位置 vs 测试起点是否一致，再查走廊/地图**。与坑 13 的"起点被障碍包围"区分：本坑是坐标系整体偏移（位置显示异常），坑 13 是位置对但环境障碍多（occ 探针有占用点）
30. **⭐⭐ 终点吸附最终方案 = 接受吸附（偏移<10cm）+ 悬停保护（偏移≥10cm）（2026-08-12 会话末用户拍板，优于 goal_z_max 限值）**：坑 28 的 `goal_z_max=0.76` 是"拒绝吸附/猜值"思路（只压 z、xy 偏移仍在）；用户提出更优组合——**既然 getNearestInfCellNot 找的吸附点就是最近空闲格（安全），小偏移就接受它作为最终终点（两终点一致→不 NaN），大偏移说明终点区域不可靠就悬停**。逻辑：FSM 吸附后算 `offset = (gi_.goal_p - click_point).norm()`——**offset < 0.1m → 接受吸附点**（guide_path 终点也替换成它，MINCO 无冲突）；**offset ≥ 0.1m → 拒绝该 goal**（不设 goal → FSM 保持 Wait for goal → bridge 悬停保护，不飞向错误位置）。**实现两处**：① fsm.cpp `setGoalPosiAndYaw` 吸附后 `if (offset > 0.1) { warn("Goal吸附偏移过大(%.2fm), 悬停保护, 拒绝目标"); return; }`；② super_planner.cpp 注入块把"条件追加"（`if ((sub_path.back()-gi_.goal_p).norm() > res*5) push_back(goal)`）改成**无条件替换** `sub_path.back() = gi_.goal_p;`（终点统一为吸附后的目标）。**数值对照（本会话实测）**：航迹终点 z=0.75 → 吸附 z=0.825（偏移 8.3cm < 10cm）→ 接受，飞到 0.825，不乱转；若吸附到 3m 外 → >10cm → 悬停保护。**教训：终点问题的最优解不是"限制吸附范围"而是"信任吸附+偏移阈值兜底"——吸附点=空闲格本身是安全保证，阈值只拦截异常大偏移；goal_z_max 限值方案可弃用（保留默认不影响）**。注意 10cm 阈值要覆盖"正常小吸附"（0.75→0.825 的 8.3cm）又不放行异常——别设太小（如 5cm 会把正常吸附也拒了）。**✅ 已实施（2026-08-12 会话末，编译 0 错误 fsm_node 15:46，备份 .bak_goaloffset）**：两处改动落地细节——① fsm.cpp `setGoalPosiAndYaw`：**`getNearestInfCellNot` 是原地覆盖（输入输出同一变量 `gi_.goal_p`），必须先 `const Vec3f goal_orig = gi_.goal_p;` 保存原终点再吸附**，然后 `if ((gi_.goal_p - goal_orig).norm() > 0.1) { ROS_WARN("[Fsm] Goal吸附偏移过大(%.2fm>0.1m), 悬停保护, 拒绝目标 ..."); return; }`（return 在函数正确作用域=提前退出，不设 goal → FSM 保持 WAIT_GOAL → bridge 悬停）；② super_planner.cpp 注入块把条件追加（`if ((sub_path.back()-gi_.goal_p).norm() > res*5) push_back(goal)`）替换为**无条件** `sub_path.back() = gi_.goal_p;`（一行，终点统一为吸附后目标）。**验证**：触发后 `grep -a "Goal吸附偏移" real_flight.log` 出现 = 悬停保护触发（终点区域脏）；无该日志且终点不乱转 = 接受吸附生效。**配合闭环航迹（坑 31）测试**：闭环终点≈起点，起点区域常有机架/人 → 偏移>10cm 悬停保护触发是预期行为（安全特性），测试时起点区域清干净即可
31. **⭐⭐ 闭环航迹循迹（终点≈起点，2026-08-12 会话末分析 + 航点生成已落地）**用户问"闭环形状能否正常循迹"——**机制上能飞一圈**：单向推进（global_last_idx_）从 idx0 出发 + guide_path=[起点→绕一圈→终点≈起点] → 飞机绕一圈回原点停。**四个要点**：
    - **不能自动循环**：飞到终点（≈起点）FSM 判到达停止，无"回到起点重新出发"逻辑——要持续绕圈巡逻需额外开发（到达终点后重置 goal 回起点）
    - **拐角多 → 走廊更容易失败**：闭环至少 4 个直角转弯，每个转弯=走廊盒子方向突变 → MINCO 可能 Opt failed；**转弯处用圆弧过渡**（R=0.3m，每 2° 采样，直线段留 R 给圆弧）——与 L 形圆角同法（坑 28）
    - **起点=终点重叠区的悬停保护风险**：终点≈起点 → 起点区域常有机架/人 → 终点吸附偏移可能 >10cm → **触发坑 30 的悬停保护（不飞）**——这是安全特性但会让闭环测试停住，测试时起点区域要清干净
    - **闭环航点生成已落地（155 点）**：圆角矩形 `(0,0)→(1.6,0)→(1.6,-1)→(0,-1)→(0,0)`（顺时针，4 个 R=0.3 圆角），z 恒 0.75，总路程 5.29m，起点=终点距离 0.000m（完美闭合），endpoint.txt 同步（终点=最后一点≈起点）。**验证要点**：`grep "Using FULL GLOBAL"` 看 idx 从 0 推进到 154（完整一圈）、POS+VEL 看 4 个圆角转弯是否顺畅、有无悬停保护触发（终点吸附偏移）。**测试建议**：闭环先"飞一圈"验证（机制能走通）→ 再决定要不要加循环逻辑；若持续巡逻要额外开发"到达终点重置回起点"
    - **⭐⭐ 闭环实测：触发成功但飞机原地不动（2026-08-12 实机，闭环终点的天然困境）**：闭环航点 + endpoint=闭环终点(≈起点) 实飞——`Mavros triggered` + `Pub goal to -0 0 0.75` + `Get goal at 0.075 0.075 0.825`（吸附保护没触发，偏移 0.13m 边缘）+ `ReplanOnce succeed` ×15（**规划全成功，无走廊失败**），但飞机原地微动（POS+VEL 停在起点附近）。**根因不是走廊/吸附/代码，是闭环结构本身**：闭环终点=起点=飞机起飞位置 → 触发时 goal≈飞机当前位置 → FSM "close to goal" 判定（`(guide_path.back()-goal).norm() < res*5`）→ 认为已在目标附近 → 不规划不飞。**类比**：发令枪响时人已在起跑线，若终点=起跑线则原地不动；必须让他"先跑到圈上某点再绕圈"。**开环 vs 闭环本质**：开环 goal 在远处（必须飞），闭环 goal 就在脚下（不用飞）——闭环是"绕圈巡逻"语义（单次任务架构=点到点，到了就停），在"单次任务"架构里闭环终点=起点=无处可去。**✅ 方案 A（零代码，用户拍板，已实施）**：endpoint.txt 改**圈上任意一点**（起点前方 0.5m，闭环轨迹上 idx=5 (0.5,0,0.75)，`echo "0.500000 0.000000 0.750000 0.1" > endpoint.txt`）→ 触发后飞机从起点飞向圈上 (0.5,0) → 沿闭环走完一圈。**备选方案对比**：B=完整循环模式（代码开发大，真正巡逻）；C=动态 goal 前移（代码~15行：goal 离飞机<阈值时自动改为轨迹上 idx+N 的点，闭环/开环通用，性价比最高）；D=改"到达判定"（只有轨迹完整执行完才算到达——**风险高**：全局影响所有场景、执行完定义模糊（时间/位置/速度都不干净）、轨迹中途失败时永不"执行完"→状态机卡死、终点附近追精确终点振荡、与吸附保护冲突、回归成本高）；E=航点留间隙（终点离起点 0.2m 变"近闭环开环"，零代码但不完美）。**教训：闭环要能循迹，goal 必须在圈上（方案A/C），不能是闭环终点本身；方案 D（改全局到达判定）是用大手术解决小问题，别选**
32. **⭐⭐ MINCO 只约束段端点 = dead_d 管不住"段内斜切"（2026-08-12 会话末源码深挖，C 形航迹实飞仍直线后的架构性结论）**：L 形/C 形实飞多次"走廊成功但飞机直插终点"后，深挖 exp_traj_optimizer_s4.cpp 的代价函数（140-147 行）发现**架构性限制**：attractor 代价**只作用在每个 piece 的端点**——`is_waypoint = (j==0) && (i!=0)`（段起点）+ `is_end = (j==integralResolution) && (i!=piece_num-1)`（段终点），`if (is_waypoint || is_end)` 才算 `|pos - attractor|² - dead_d²` 代价——**MINCO 轨迹在段与段之间完全自由（只要在走廊内）**。所以：
    - **dead_d 只能收"段端点附近"，收不住"段内"**——走廊 4.41m C 形只有 ~4 个盒子 → attractor 只有 4-5 个点 → MINCO 在 4-5 个点之间走"盒子中心连线"≈直线（跳过整个弯曲段）。**这就是"走廊成功（GenerateExpTrajectory SUCCESS ×4 无失败）但飞机直插终点"的机制**——不是走廊失败、不是 idx 跳段（idx=3→144 是飞机物理位置追平，见坑 27），是 MINCO 架构上就不做贴路径（=走廊内最短时间轨迹，非路径跟随）
    - **三条路线（会话末给出，未实施）**：路线 1 **走廊窄隧道**——corridor_bound_dis 调更小让盒子窄（MINCO 无斜切空间）；路线 2 **段数加密（推荐试）**——`corridor_line_max_length: 1.2→0.4`（real_flight.yaml:37），每 0.3~0.4m 一个盒子 → attractor 点密集 → MINCO 被约束的点多 → 贴路径（改一个参数不编译，C 形 4.41m/0.4m ≈ 11 个盒子）；路线 3 接受"平滑化"（轮廓对、弯角切弦，SUPER 设计特性）。**判读**：改了 corridor_line_max_length 后重启 FSM（启动时读配置）；看 POS+VEL 是否转弯清晰
    - **⭐ 路线 2 实测（2026-08-12 下午，corridor_line_max_length 1.2→0.4，C 形实飞——轮廓改善但 x 截断 + idx 追平）**：用户拍板试路线 2（改参数不编译，重启 FSM 生效）——`corridor_line_max_length: 0.4` 后实飞 C 形：**POS+VEL 出现 C 形轮廓了**（先向右到 x≈1.0 再转左下 y→-0.97，不再是起点直插终点的直线——盒子加密确实让 MINCO 贴轨迹了 ✅），**全程 0 走廊失败**（GenerateExpTrajectory SUCCESS ×3，无 GeneratePolytopeFromLine failed）。但两个新问题：**① x 只到 1.00（C 形前进段应到 1.6）**——飞机在 x≈1.0 就提前转弯；**② idx 从 19 跳到 101**（不是跳段 bug，是飞机物理位置追平——单向推进只防回头不防追平，见坑 27；idx=101 对应的航迹坐标 ≈ 飞机实际位置 (1.00,-0.43)）。**用户精度要求（明确）**："必须严格到1.6，可以接受10cm的误差"——x 只到 1.0 不达标。**用户追问"单向递推没有起作用吗"的答复**：单向推进代码确认生效（`search_from = (global_last_idx_ > 0) ? global_last_idx_ : 0`，super_planner.cpp:598），idx 19→101 是飞机飞行速度 > replan 推进速度的"追平"（飞机沿上轮轨迹已物理到达轨迹中后段位置），不是逻辑失效。**corridor_line_max_length 三档经验曲线（完整版，2026-08-12 全天实测）**：**1.2（原始）= 4 盒子 → MINCO 斜切直线（跳过弯曲段）**；**0.4 = 11 盒子 → C 形轮廓出来（x 到 1.0，全程无走廊失败）——甜点**；**0.25 = 18 盒子 → 走廊崩**（实飞：`SearchPolytopeOnPath for new path failed` + **corridor_generator.cpp:113 崩溃堆栈**（seed line 太短/相邻盒子重叠失败）+ `GeneratePolytopeFromLine failed` → 无轨迹 → 飞机不动）→ **已回退 0.4**（super_backto04）。**教训：MINCO 贴轨迹程度 ≈ attractor 密度（corridor_line_max_length 倒数），但盒子太密走廊生成失败率上升——0.25 是下限（走廊崩），0.4 是"轮廓对、细节截断"，"完全贴轨迹（x 到 1.6±0.1）"参数已到极限，需换思路（方案 C）；改 corridor_line_max_length 只改 task_run_scripts/config/real_flight.yaml（坑 5），重启 FSM 生效，无需编译**
    - **⭐ 方案 A 实测局限（用户预测验证："经过这个点不会直接判断抵达终点吗"——对！）**：闭环方案 A（endpoint=圈上点 (0.5,0)）被用户问穿——查代码确认 **close to goal 判定（super_planner.cpp:478-491）**：`if (!gi_.new_goal && (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3)`（阈值 = res*3 = 0.45m）→ 飞机沿轨迹飞到 (0.5,0) 时距 goal=0m < 0.45m 且非新目标 → 判定"接近目标"→ NONEED 停住 → **闭环依旧走不完，只是把"起点停"变成"起点前 0.5m 停"**。**方案 A 不解决闭环**——真正要闭环要么 goal 动态前移（方案 C：每轮 replan 把 goal 设为轨迹当前位置前方 N 米，close to goal 永远不满足→走完全程，改动集中在注入块，比方案 D 局部）、要么改到达判定（方案 D，风险高）。**结论**：闭环暂时搁置，改回开放轨迹（C 形：前进→右移→后退）
    - **⭐ 方案 D（改全局到达判定）风险清单（用户问"有什么风险"，已完整分析）**：① **全局影响**——到达判定是所有任务的公共逻辑（开环/航点/点击/闭环/避障全受影响），改它=所有场景行为都变；② **"执行完"定义模糊**——按时间（轨迹 duration 到但跟踪误差未到位）/按位置（回到老问题）/按速度（终点附近本来有调整动作）都不干净；③ **失败死锁**——轨迹中途失败（走廊失败/MINCO NaN）时没有轨迹在"执行"→"执行完"永不触发→状态机卡"执行中"→后续命令不响应；④ **终点振荡**——严格"执行完才停"→飞机追精确终点来回微调（之前 19:14 实飞已见终点乱转）；⑤ **与吸附保护冲突**（坑 30）——goal 被拒（悬停保护）但 FSM 认为"未执行完"→状态机既无 goal 又无完成→卡死；⑥ **回归成本高**——改完必须装桨起飞完整回归所有场景。**结论：方案 D 是"用大手术解决小问题"，闭环只是"goal=起点"特例，用方案 C 局部改动就能解决**
    - **C 形航点生成（2026-08-12 用户"保留先前进、再右移、再后退"，弃闭环）**：形状 `(0,0)→前进+1.6m→(1.6,0)→右移-1m→(1.6,-1)→后退-1.6m→终点(0,-1.0)`，3 个圆角（R=0.3，圆心在转角内侧），5cm 间距，151 点，总路程 4.41m，z 恒 0.75，endpoint.txt=终点 (0,-1,0.75)。**⚠️ 首版生成失误：圆角4 回起点段把轨迹画成了闭环**（终点回到 (0,0)）——用户要的"前进→右移→后退"终点应在 (0,-1) 不回起点；修正=去掉最后两段（直线4/圆角4 的回起点部分），直线3 后退到 (0.3,-1) 后圆角3 接直线4 继续后退到终点 (0,-1)。**教训：生成"开放"轨迹时终点要独立于起点（终点≠起点），否则又是闭环**；生成后验证 `起点-终点距离` 应 >0
    - **C 形实飞结果：仍直插终点（2026-08-12 会话末实机，MINCO 架构限制的实锤）**：C 形 151 点 + endpoint=(0,-1) 实飞——`Mavros triggered` + `Using FULL GLOBAL PATH, idx=2→3 total=149` + `GenerateExpTrajectory SUCCESS ×4`（**全程无走廊失败**），但 POS+VEL 从 (0.08,0.07) 直插 (0.08,-0.97)（x 恒 0~0.2，y 单调下降——**没走 C 形的前进段**！）→ 飞机直飞终点 (0,-1)。**至此确认：走廊成功+注入生效+idx 正常，MINCO 仍拉直 = 架构性**（本坑上文的 attractor 段端点机制）——下一步就是路线 2（corridor_line_max_length 段数加密）或接受平滑化。**教训：路径 A 的"沿轨迹"程度 = MINCO 的 attractor 密度决定，不是注入逻辑决定；注入只管"guide_path 是什么"，MINCO 管"轨迹长什么样"，两者脱节**
    - **⭐⭐ 方案 C 可行性确认：/planning/pos_cmd 是单点位置指令，不是整条轨迹（2026-08-12 会话末源码核对，方案 C 可实施的依据）**：用户否掉方案 A（航点加长补偿——利用切弦特性预补偿）和方案 B（attractor 改 guide_path 点——只改段端点、段内照样切弦，且盒子↔路径点映射复杂）后，深挖发布链路发现关键事实：**bridge（super_to_px4_bridge.cpp:241）订阅的是 `quadrotor_msgs::PositionCommand`（单点位置指令），不是整条 MINCO 轨迹**——`superCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)` 每个 pos_cmd = 一个目标点，bridge 用 position_control_gain 位置控制飞向该点（`/mavros/setpoint_raw/local`）。**含义**：SUPER 每 0.1s 发布"轨迹当前点"（MINCO 计算的位置），bridge 逐点跟随——**发布链路天然支持"逐点发 guide_path 航点"**。**方案 C 本质**：跳过 MINCO（或将其降级为可选），把 guide_path（录制轨迹点本身）直接作为逐点发布的轨迹源——`guide_path 点 → 每 0.1s 按进度取点 → 发布 pos_cmd` → 飞机沿航点走 = **100% 贴轨迹（误差≈0，航点 5cm 密≈平滑）**。**实现要点**：① 注入块生成 guide_path 后存入"待发布轨迹"（不是丢给 MINCO）；② 新增发布逻辑：按 max_vel 匀速推进取 guide_path 点发布（~50 行）；③ **保留走廊检测做障碍 fallback**（走廊失败→A* 绕行→恢复），否则失去动态避障。**代价/风险（诚实）**：失去 MINCO 动力学平滑（速度/加速度不连续，但 5cm 密点折角小）；轨迹可能"折线化"；动态避障依赖保留的走廊检测。**用户决策链**：A（补偿）否、B（attractor 改点）否——方案 C（guide_path 直接发布）是用户接受的"严格循迹（±10cm）"正解。**✅ 已实施（2026-08-12 会话末，编译 0 错误 fsm_node 16:34，备份 .bak_gpdirect）——实施三处**：① super_planner.h 加成员 committed_guide_path_ / committed_guide_stamp_ / committed_guide_valid_ / committed_guide_mtx_；② super_planner.cpp 注入块 global_used_prev_=true 后保存 guide_path+guide_stamp 到 committed（valid_=true）；③ 核心：getOneCommandFromTraj 里 pvaj=cmd_traj_info_.posTraj().getState(eval_t) 前插入线性插值块——committed_guide_valid_ && size>=2 && !robot_on_backup_traj_ 时按 eval_t 在 committed_guide_stamp_ 定位段 → 段内线性插值得 pvaj（速度=段内平均速度，加速度/加加速度置 0，eval_t>=gs.back() 时 traj_finish=true）；否则走原 MINCO。保安全设计：走廊检测照跑——SearchPolytopeOnPath 失败→fallback A*→robot_on_backup_traj_=true→getOneCommandFromTraj 检测到就走原 MINCO 绕行（动态避障保留），障碍消失恢复沿 guide_path。发布链路关键确认：pos_cmd 发布在 fsm_ros1.hpp getOnePositionCommand（135-144 行）调 planner_ptr_->getOneCommandFromTraj，每 0.1s 发单点。验证：触发后 POS+VEL 应严格沿航点（x 到 1.6±0.1）；grep Using FULL GLOBAL 看 idx 单调；中途放障碍→走廊失败→A* 绕行→移开恢复。尚未实飞验证（编译刚完成）。**教训：先查消息类型再设计绕过方案——pos_cmd 是单点指令让"跳过 MINCO 直接发航点"成为可能，如果它是整条多项式轨迹（PolynomialTrajectory）方案 C 就不成立**
    - **⭐⭐ 方案 C 实飞结果：严格循迹成功 ✅ + 新崩溃坑 seed line too long（2026-08-12 会话末，两项实测）**：
      - **① 方案 C 本身成功了（飞跃性验证）**：按按钮3 起飞 0.6m → 拨 CH10 → POS+VEL 显示 **飞机严格沿 C 形前进段直线飞行**——`(0.08,0) → (0.25,0) → (0.51,0) → (0.71,0) → (0.97,0) → (1.17,0)`，**y 恒=0、z 恒=0.75、vx=0.30 匀速**——不再斜切/直插终点，**100% 贴航点（方案 C 的线性插值发布生效，误差≈0）**。此前所有 MINCO 贴路径折腾（dead_d 0.2 / corridor 0.4 / attractor）全部被方案 C 取代。**判读**：POS+VEL 严格沿航点 + 无 Opt failed/TrajOpt failed = 方案 C 生效
      - **② 但飞到 x=1.17 后崩溃：`what(): seed line too long`（corridor_generator.cpp:113 SearchPolytopeOnPath 抛 std::runtime_error，未捕获 → FSM 整进程崩 → 不再发 pos_cmd → 飞机悬停）**。**根因**：当时 corridor_line_max_length=0.4（坑 32 路线 2 遗留）→ SearchPolytopeOnPath 主循环里 `isLineFree(path[first_id], path[j], seed_line_max_length_, ...)` 在**无障碍直线段一直返回 true → second_id 一路延伸** → `(path[first_id]-path[second_id]).norm() > seed_line_max_length_ * 1.5`（0.4*1.5=0.6m）→ **throw "seed line too long"**。航点间距 5cm 全正常（python 验证最大 0.05m），崩的是"跨多个点的整条 seed line"不是相邻点。**判读**：崩溃堆栈 `what(): seed line too long` + `#6 corridor_generator.cpp line 113 SearchPolytopeOnPath` + `#7 super_planner.cpp line 808 generateExpTraj` + `#8 line 190 ReplanOnce` + `#9 fsm.cpp line 77 callReplanOnce`——**崩溃在走廊生成（安全网），不在方案 C 的发布逻辑**；`ps aux | grep fsm_node` 空 = FSM 已崩
      - **修复（已实施）**：**corridor_line_max_length 回 1.2**（task_run_scripts/config/real_flight.yaml）——方案 C 下循迹走 guide_path 直发，**走廊/MINCO 只是避障检测安全网**，1.2m 阈值（1.2*1.5=1.8m）不会 seed line 超长 → 不崩。**教训：方案 C 生效后 corridor_line_max_length 必须回默认 1.2——它不再影响循迹（循迹=guide_path 直发），只影响避障检测的粒度，调小只有崩溃风险没有收益**。**参数三档经验补完**：1.2=避障安全网正常（方案 C 用）｜0.4=MINCO 贴路径甜点（纯 MINCO 模式用，但有 seed line 崩溃风险）｜0.25=走廊必崩
    - **⭐ frontend_path 显示项（2026-08-12 会话末实机，方案 C 下"模拟实飞轨迹"可视化）**：用户要求"把模拟实飞轨迹显示一下"验证动态避障——**top_down.rviz 加 FrontendPath 显示项**（`rviz/MarkerArray`，订阅 **`/fsm_node/visualization/frontend_path`**，Enabled:true，字符串插入在 GlobalPathMkr 块后——**压缩格式 rviz 用字符串插入模板复制法，不用 yaml.dump 重建**，见坑 18）。**语义**：绿线（/global_path_marker）= 完整目标航点；**红线（frontend_path）= 飞机正在执行的 guide_path 当前段（方案 C 下即实际飞行轨迹）**——动态避障时红线会绕开障碍（fallback A* 段），障碍移走恢复沿绿线。**验证命令**：插入后 `grep -n -A3 "Name: FrontendPath"` + `python3 -c "import yaml; yaml.safe_load(open('top_down.rviz'))"` 不报错（显示项在 `Visualization Manager/Displays` 嵌套列表，顶层 `len(d.get('Displays'))`=0 是结构差异非插入失败）；重启 SUPER 生效
    - **⭐ 方案 C 下动态避障测试流程（2026-08-12 会话末给出，尚未实飞）**：障碍物放 **C 形前进段（(0.5~1.2, 0) 之间）**——飞机必经之路；**放障碍时机 = 飞机走到 (0.5,0) 左右**（太早飞机未起飞触发、太晚来不及）；流程 = 测试点启动雷达 → 按钮3 → 起飞 0.6m → 拨 CH10 → 飞机沿 C 形前进 → 前进段放障碍（人站 3m 外，**长杆/绳子放障碍，随时遥控器 Position 接管**）→ 观察：走廊检测到障碍 → `Global path blocked by obstacle, fallback to A* replan` → 红线绕行 → 移走障碍 → 恢复沿轨迹。**判读**：`grep -aE "fallback|backup" real_flight.log` 有 fallback 标记 + POS+VEL 绕开障碍坐标 = 动态避障在；注意方案 C 下 fallback 时 `robot_on_backup_traj_=true` → getOneCommandFromTraj 走原 MINCO（代码里已按此设计，避障段轨迹是 MINCO 绕行、非 guide_path 直发）
    - **⭐ 地面测试（不起飞）适配：goal_z_min/max 必须压到地面高度，否则 MINCO 爬升失败无轨迹（2026-08-12 实机，用户"全程地面测试"想先看轨迹再当动态障碍物）**：地面测试 = 不装桨不起飞，只拨 CH10 看规划轨迹变化（验证避障逻辑，零风险，是个好方法）——但直接拨杆会失败：**飞机 z=0（地面）而 goal 被吸附到 z=0.825 → MINCO 要从 0 爬到 0.825 → `OptimizationExpTrajInPolytopes for new path failed` → `GenerateExpTrajectory FAILED` → PlanFromRest failed 循环 → 无轨迹可看**（与坑 13 的"地面触发走廊失败"不同：这次走廊能过、是 MINCO 爬升约束失败）。**修复（临时改配置，已实施）**：`fsm/goal_z_min: 0.6→0.0`、`goal_z_max: 0.825→0.3`（地面测试专用，**测完必须改回 0.6/0.825 再起飞**——备份 .bak_gz03）→ goal 钳制到 0.3 → MINCO 小爬升可成功 → 轨迹出现。**⚠️ 地面测试仍可能因起点在地图下界（occ box z_min=0.1）走廊失败——见坑 13 的"地面触发无效判读"**；两条失败链区分：`OptimizationExpTrajInPolytopes failed`（MINCO 爬升，改 goal_z 解决）vs `GeneratePolytopeFromLine failed`（走廊起点越界，必须起飞）。**判读顺序**：`grep -aE "OptimizationExpTrajInPolytopes|GeneratePolytopeFromLine" real_flight.log` 看是哪个在失败
    - **⭐ 方案 C 的可视化缺口已修复（2026-08-12 会话末实机，编译 0 错误 fsm_node 17:06，备份 .bak_gpdirect）**：RViz 轨迹线（frontend_path/exp_traj）**由 generateExpTraj 的可视化发布——MINCO 失败（如地面测试 goal_z 未压地面→OptimizationExpTrajInPolytopes failed）→ 没有 exp_traj → 话题 0 点 → RViz 无轨迹**（判读：`rostopic echo /fsm_node/visualization/frontend_path -n 1` 0 点 ≠ 显示项配置问题，而是规划没成功，先看 real_flight.log 的 GenerateExpTrajectory SUCCESS/FAILED）。**方案 C 自身还有个可视化缺口**：getOneCommandFromTraj 直接发 guide_path 时不发布这条线的 marker（只有 MINCO 成功时 exp_traj 才画线）——**修复：方案 C 分支线性插值后加一行 `ros_ptr_->vizFrontendPath(gp);`**（ros1_interface.hpp:121 现成接口，发布 /fsm_node/visualization/frontend_path 粉色线，ns=guide_path；**不是新写 marker，直接复用现成方法**）。修复后 RViz 的 FrontendPath 显示项（top_down.rviz 已加，坑 32）显示粉色线 = 飞机正在执行的 guide_path 段；**注意原 vizFrontendPath 调用在 generateExpTraj 806 行，方案 C 绕过 MINCO 后必须在该分支自调否则永远不发布**。**"看到一条连接起点终点的曲线"的三种可能**：① 残留旧 marker（系统重启/未触发时 RViz 显示最后一次规划结果，坑 25）② global_path_marker 绿线（frame=map 需 TF 通，坑 22）③ 其他显示项串线——**先 `rostopic echo` 确认话题数据再归因显示**
33. **⭐⭐ 方案 C 可视化不显示的最终根因：RViz 帧不匹配 + 话题前缀坑（2026-08-12 会话末实机，"加了 vizFrontendPath 还是没粉色线"）**：修好可视化调用、编译重启后 RViz 仍无粉色线，逐层排查沉淀两个独立坑：
    - **① 话题带节点名前缀（查错话题会误判"没数据"）**：`ros1_interface.hpp:50` `guide_path_pub_ = nh_.advertise<MarkerArray>("visualization/frontend_path", 100)` 的**相对话题名经 rospy/roscpp 节点命名空间展开成 `/fsm_node/visualization/frontend_path`**——`rostopic echo /visualization/frontend_path`（无前缀）永远 no data，**必须用 `rostopic list | grep -iE "frontend|visualization"` 拿实际话题名再 echo**。同理 frontend_path/exp_traj/exp_sfc 全部带 `/fsm_node` 前缀。**教训：怀疑"话题没数据"先 `rostopic list` 确认实际名字（含节点前缀），再下"发布失败"结论**
    - **② RViz 帧不匹配（粉色线真不显示的根因）**：即使话题对、数据有（`echo /fsm_node/visualization/frontend_path -n 1` 有 marker，但**点数异常少/全在原点**——本实测 3 点 x:0~0 y:0~0），RViz 仍不显示。**根因**：ros1_adapter.hpp:43 `const std::string DEFAULT_FRAME_ID = "world";`——**SUPER 所有可视化 marker 帧=world**；而 **global_path_publisher.py 发布 frame_id="map"**（能显示，绿线）；用户 RViz Fixed Frame=map → **world 帧无 TF（map↔world 无人发布）→ 粉色线 marker 无法变换 → 不显示**。**判读**：`rostopic echo /fsm_node/visualization/frontend_path -n 1 | grep -A2 frame_id` 看 = world vs map；对比 global_path_marker 的 frame_id（map）。**解决方向（会话末给出未实施）**：方案 C 的可视化**不能直接复用 vizFrontendPath（world 帧）**，需在方案 C 分支**自己构造 marker（frame_id="map"，LINE_STRIP，与绿线同帧）发布到 /fsm_node/visualization/frontend_path**——super_planner.h 加发布器 + ~20 行构造代码；或让 RViz 加一个 Fixed Frame 切 world 的显示项（不推荐，全局帧乱）。**教训：SUPER 内置可视化默认 world 帧，与系统全局的 map 帧不一致——任何"复用现成可视化接口"的显示都要先验 frame_id，否则白改白编译**
    - **地面测试确认流程（用户"全程地面测试看轨迹变不变"）**：先 `ps aux | grep 'rviz -d'` 看 RViz 加载哪个配置（slam_rviz.rviz 无 FrontendPath 显示项，top_down.rviz 才有——两个 RViz 并存时用户看的是哪个要确认）；再 `rostopic info /fsm_node/visualization/frontend_path | grep -A4 Subscribers` 看 RViz 在不在订阅列表（None=显示项没加对话题或 RViz 配置不对）——**Subscribers 是空的（连 RViz 都没订阅）说明问题在 RViz 显示项配置，不是发布端**


35. **⭐⭐ 方案 C 会话末状态评估 = 下一会话起点（2026-08-12 结束时的诚实盘点）**：
    - **循迹 ✅ 已解决**：方案 C（guide_path 直发）严格沿航点（实测 y=0、z=0.75、vx=0.30 匀速直线前进，误差≈0），corridor_line_max_length 回 1.2 后不崩溃（seed line too long 消失）。**循迹这块不要再动 MINCO/走廊参数了**
    - **动态避障 ⚠️ 是"半套"（当前最大遗留）**：检测 ✅（走廊失败 → `Global path blocked` → fallback 标记会出），**但绕行轨迹生成常失败**——地面/飞行日志反复出现 `[MINCO] TrajOpt failed, (unknown)` / `[BackOpt] Opt failed, Omg or thr or Pos violation` / `[MINCO] TrajOpt failed, The function value became NaN or Inf` / `generateBackupTrajectory return OPT_FAILED, replan Failed return`——**fallback A* 找到绕行路径后 backup MINCO 优化失败 → 没有绕行轨迹 → 飞机悬停（安全但不会绕）**。下一会话要解决：backup 轨迹 MINCO 失败（A* 绕行路径太曲折/动力学不可行 → Opt failed；或两终点不一致 → NaN），方向：a) backup 轨迹也用 guide_path 直发（方案 C 同款逻辑，不优化直接发 A* 路径点）b) 修 backup MINCO 的终点/约束。**判读**：`grep -aE "fallback|backup|blocked" real_flight.log`——有 `Global path blocked` 无有效 backup 轨迹 = 检测到障碍但绕行失败
    - **地面测试（不起飞）是验证避障逻辑的零风险方法（用户认可）**：不装桨不起飞，拨 CH10 看规划轨迹是否随障碍变化——但必须先改 `fsm/goal_z_min: 0.0` / `goal_z_max: 0.3`（地面 z=0，否则 MINCO 爬升失败无轨迹，坑 32），**测完改回 0.6/0.825 再起飞**（备份 .bak_gz03）
    - **可视化现状**：绿线（/global_path_marker，frame=map）= 完整航点；红线（/fsm_node/visualization/committed_path，方案 C 自发 frame=map LINE_STRIP，top_down.rviz FrontendPath 显示项已改 rviz/Marker 类型订阅它）= 实际执行轨迹；橙色（unk）= 未知区域。**验证红线**：`rostopic echo /fsm_node/visualization/committed_path -n 1 | grep -c "x:"` ≈ 航点数
    - **地面测试看到"一条连接起点终点的曲线"的归因**：若所有轨迹话题（frontend_path/exp_traj/committed_path）都 0 点 = 规划没成功（MINCO 失败）→ 先看 real_flight.log 的 `GenerateExpTrajectory SUCCESS/FAILED`，别归因于显示配置；若话题有数据但 RViz 无 = 帧不匹配（world vs map）或显示项没订阅（坑 33/34）

- **⭐ 文档工作流补遗：残留清单章节的删除（2026-08-12 用户"md还需要第六章吗"）**：残留文件全部清理完后，更改记录 md 里的"残留文件清单"章节已无意义——用户会主动问要不要删（"还需要第六章吗"），**直接删除该章节**（残留已清，清单过时），文档保持"更改记录+参数表+待办"的干净结构。配套：残留清单的子节编号（6.1/6.2...）在插入参数表章节后要同步改（参数表占五章，残留改六章）
- **提交前汇总工作流（2026-08-12 用户要求"列所有更改+残留文件写 md 到桌面"）**：git 提交前主动产出两份交付物——① 更改记录 md（写到宿主机桌面）：三仓库（task_run_scripts/NX_code/SUPER）分别 `git log --oneline <base>..<branch>` 列提交 + `git status --short` 列未提交，SUPER 仓库未提交的 .cpp/.hpp 是核心代码改动（本会话 7 个文件），按 架构/代码更改/排查结论/待办 分章；② 残留 .bak 清单：三仓库 `find . -name "*.bak*"`（NX_code 6 个 + task_run_scripts 10 个 + SUPER 11 个）+ 用户目录 tar.gz 快照（`~/task_run_scripts_backup_*.tar.gz`/`obstacle_branch_*.tar.gz`/`obstacle_changes_backup/`）+ 未提交修改待确认项（如 px4.launch 的 gcs_url 改动是否恢复）。**清理前先列清单给用户确认**。
- **⭐ 提交顺序与待确认项（2026-08-12 实机，用户"先提交"）**：先提交 **SUPER（dev/4uav）**——路径A核心 7 个 .cpp/.hpp/yaml 一次 commit（本会话 8cafee3，+173 行，提交后 `git status --short` 应为空）→ 再提交 **task_run_scripts（feat/4uav-guide）**（real_flight.yaml + 航迹 + endpoint.txt + NX_code 指针 + 旧航迹删除，本会话 9fcf976，15 文件）→ **px4.launch 的 gcs_url 改动**（用户确认故意改后提交，1bb7d24，含清理 bak_livox 残留）。NX_code 子模块指针随 task_run_scripts 一起提交（`git add scripts/NX_code`）。**检查提交完整性**：提交后 `git log --oneline -3` 看新提交在顶 + `git status --short` 看剩余；`git log --oneline origin/<branch>..<branch>` 空 = 无未推送提交。**注意**：SUPER 仓库在 dev/4uav（远程已有），task_run_scripts/NX_code 在 feat/4uav-guide（**本会话最初未推远程，origin/feat/4uav-guide 不存在**）
- **⭐ 推送顺序（2026-08-12 实机完成，三个仓库全部推送成功）**：**先推子模块 NX_code，再推 SUPER，最后推主仓库 task_run_scripts**——因为主仓库的提交里含 `scripts/NX_code` 子模块指针（指向新 commit），若先推主仓库，远程引用的子模块 commit 尚不存在会造成悬空。逐个 `git push -u origin <branch>`（task_run_scripts/NX_code 是新建分支 `[new branch]`；SUPER 是 `6be0fb8..8cafee3 dev/4uav -> dev/4uav`），每个推送后 `git log --oneline origin/<branch>..<branch>` 空 = 已同步。**推送后的 GitHub PR 链接**（git push 输出会打印 `pull/new/<branch>` 链接）可给用户作 PR 入口。**三个仓库的远程分支全景（2026-08-12 最终）**：task_run_scripts = main / feat/4uav-guide（新推）/ feat/obstacle-to-px4（已有）；NX_code = master / feat/4uav-guide（新推）；SUPER = dev/4uav（已更新）
- **⭐ 文档上传到 GitHub 文档仓库（2026-08-12 实机，Drone-Testing-Documentation）**：用户要求把更改记录 md 传到 `github.com/jihaofly/Drone-Testing-Documentation` 新文件夹。流程：① 远程 clone 空仓库（`git clone https://github.com/jihaofly/Drone-Testing-Documentation.git ~/Drone-Testing-Documentation`，空仓库 clone 会警告但成功）② **宿主桌面 md → 远程用系统 pscp**（`pscp -scp -batch -hostkey 'SHA256:xxx' -pw PASSWORD 本地.md user@host:/home/user/`，见 ssh-remote-linux-ops 的 pscp 修正）③ 远程 `mkdir -p SUPER && mv ~/SUPER.md SUPER/SUPER.md` ④ `git add SUPER/SUPER.md && git commit` ⑤ **⚠️ 空仓库默认分支是 `master` 不是 `main`——`git push -u origin main` 报 `error: 源引用规格 main 没有匹配`**（本地分支名 master 不匹配）；用 `git push -u origin master` 成功（`[new branch] master -> master`）。**教训：新 clone 的空仓库先 `git branch --show-current` 看默认分支名，push 用实际分支名，别假设 main**
  - **⭐ 文档仓库 push 被拒 → `git pull origin master --rebase` 再推（2026-08-12 晚实机，方案F 章节上传时）**：`git push origin master` 报 `更新被拒绝，因为远程仓库包含您本地尚不存在的提交`（别人/别处已推过，本地落后）→ **不能 force push（会丢远程新提交），用 `git pull origin master --rebase` 把本地提交重放到远程最新之上**（输出 `首先，回退头指针以便在其上重放您的工作... 应用：docs: ...`）→ 再 `git push origin master` 成功（`5b41a20..a6a0fac`）。**教训：共享文档仓库多人/多端推送常态，push 被拒先 pull --rebase（保持线性历史），别 force**
- **⭐ 更改记录 md + SUPER 参数表（2026-08-12 用户要求"列所有更改+残留文件写 md 到桌面"+"写明 SUPER 参数表"）**：提交前主动产出交付物——更改记录 md（写到宿主机桌面，本会话文件名被用户改叫 SUPER.md）：三仓库分别 `git log --oneline <base>..<branch>` 列提交 + `git status --short` 列未提交，SUPER 仓库未提交的 .cpp/.hpp 是核心代码改动（本会话 7 个文件），按 架构/代码更改/排查结论/待办 分章。**参数表章节结构（用户明确要求"我们在参数中写了什么"）**：5.1 路径A 新增/修改参数（goal_z_min/max、global_path roll_dist/downsample、corridor_bound_dis 1.0→0.39，含原值对比）；5.2 地图/可视化参数（raycasting enable、pub_unknown_map_en、p_miss/p_free/p_occ/p_hit）；5.3 未修改但关键（robot_r、iris_iter_num、max_vel 等）；5.4 super_to_px4_bridge.yaml（init_goal/z、velocity_ff_gain 等）；5.5 代码内硬编码（dead_d 钳制 0.2、global_last_idx_ 单向推进、吸附 z 钳制——**不在 yaml 里，改代码才生效**）。**参数表数据来源**：`grep -nE` 抓当前 yaml 实际值 + FSM 启动日志 `Load param ... success: <值>` 交叉确认（yaml 改了没重启=FSM 加载的还是旧值，写文档前要核对日志加载值）。**残留 .bak 清单**：三仓库 `find . -name "*.bak*"`（NX_code 6 + task_run_scripts 10+1 + SUPER 11）+ 用户目录 tar.gz 快照 + 未提交修改待确认项。**清理前先列清单给用户确认**
- **清理执行模式（2026-08-12 实机）**：用户确认后批量 `rm -f` 三仓库 .bak → **必须再 `find` 全扫描确认 0 残留**（首批批量删除漏掉 `global_path_publisher.py.bak_marker`——初始清单外的漏网之鱼，靠复查抓到）→ **删除后验证功能完好**：关键文件存在性（global_path_publisher.py/real_flight.yaml/top_down.rviz/guidepoints_new.txt/endpoint.txt/User_define_commands.yaml/nx_command_server.py/super_planner.cpp/exp_traj_optimizer_s4.cpp/config.hpp 等 10 个）+ 编译产物时间戳（`ls -la devel/lib/super_planner/fsm_node`）+ 发布器语法（`python3 -m py_compile`）。**tar.gz 快照（回退后路）默认保留，用户明确要求删除才删**（本会话用户确认删除 2 个 tar.gz——删除前 `ls -la` 展示内容+大小让用户知情，删完 `ls ~/*.tar.gz` 验证无残留；`obstacle_changes_backup/` 目录因 git 分支已推 GitHub 有副本，删不删由用户定；删除前 `git show feat/obstacle-to-px4:launch/obstacle_to_px4.launch` 确认 git 历史里有副本再删）
- 抽稀生效：Load 点数随 waypoint_downsample_dist 变化（1038→83@0.1m→14@0.5m）
- 复飞安全收尾：AbortReplay 后 /mavros/state mode=POSCTL

34. **⭐ 方案 C 轨迹可视化的最终落地：自己发 frame_id="map" 的红色 LINE_STRIP（2026-08-12 会话末实机，编译 0 错误 fsm_node 17:15，备份 .bak_vizmap）**：坑 33 的 vizFrontendPath（world 帧）方案实测不显示后，**修复已实施**——方案 C 分支不复用 vizFrontendPath，自己构造 marker 发布到**新话题 `/fsm_node/visualization/committed_path`**：super_planner.cpp 顶部加 `#include <visualization_msgs/Marker.h>`；方案 C 分支线性插值后加 `static ros::Publisher committed_path_pub; if (!committed_path_pub) committed_path_pub = ros::NodeHandle("~").advertise<visualization_msgs::Marker>("/fsm_node/visualization/committed_path", 10);`，marker 参数：frame_id="map"、type=LINE_STRIP、scale.x=0.08、color 红(1,0,0,1)、ns=committed_path、每点 push geometry_msgs::Point（static 局部发布器，无需改 super_planner.h）。**RViz 侧配套**：top_down.rviz 的 FrontendPath 显示项话题**必须改成 `/fsm_node/visualization/committed_path`**（新话题，否则 `rostopic info` 显示 Subscribers: None）；用户实机双 RViz（slam_rviz.rviz 无显示项 + top_down.rviz 有），先确认看的是哪个。**教训沉淀**：① 话题名带节点前缀（/fsm_node/...），查数据先 `rostopic list` 拿实际名；② SUPER 内置可视化 frame 写死 "world"（ros1_adapter.hpp:43 DEFAULT_FRAME_ID），与系统全局 map 帧无 TF → 显示不了——**自己发 marker 时 frame_id 显式写 "map"（和 global_path_marker 绿线同帧）**；③ "看不到线"先验数据（echo 话题）+ 订阅（Subscribers 有无 RViz）+ 帧（frame_id vs RViz Fixed Frame）三层，别急着改代码。
    - **⭐⭐ 最后一段诊断收尾的三个增量坑（2026-08-12 会话末尾，数据完美但用户仍看不到红线）**：
      - **① RViz 显示项类型必须匹配发布端：committed_path 发的是单个 `visualization_msgs/Marker`，top_down.rviz 的 FrontendPath 显示项必须改 `Class: rviz/Marker`（不是 rviz/MarkerArray！）**——MarkerArray 显示项订阅单 Marker 话题不会显示（话题对、数据对、但 Subscribers 依然 None）。改配置时 Class 行和 Marker Topic 行要一起改。**判读**：`rostopic info /fsm_node/visualization/committed_path | grep -A4 Subscribers` = None + 话题有数据 = 显示项类型/话题不匹配或 RViz 没加载新配置
      - **② SSH/plink 会话无 DISPLAY 起不了 RViz（图形界面只能用户本地操作）**：`nohup rviz -d ...` 报 `无法运行命令 'rviz': 没有那个文件或目录`（PATH 没 source）+ `DISPLAY=` 空（plink 非交互会话无 X）→ **agent 只能改配置，RViz 启动/重启必须用户本地做**（按钮3 重启 real_flight.launch 会带起 top_down.rviz——real_flight.launch:37 `args="-d $(find task_run_scripts)/rviz/top_down.rviz"`；或用户本地终端 `kill <旧rviz_pid> && rviz -d .../top_down.rviz &`）。**别在 agent 侧反复"重启 RViz"白试**，直接把命令给用户执行
      - **③ 双 RViz 并存 + 配置加载时机**：用户可能手动开 slam_rviz.rviz（无 FrontendPath 显示项）+ 按钮3 的 top_down.rviz 并存——"看不到红线"先 `ps -eo pid,lstart,args | grep 'rviz -d'` 确认用户看的是哪个 + **对比 rviz 进程 lstart vs top_down.rviz mtime**（进程比配置早 = 加载的是旧显示项，必须重启 RViz 才读新配置，运行中改文件不热加载）。本会话最终判读：`committed_path` 数据完美（154 点 C 形，起点(0,0) 中点(1.56,-0.85) 终点(0.075,-0.975)）+ 发布正常 + 配置改对，但用户仍看到"连接起点终点的线"——归因 = **RViz 进程加载的是 slam_rviz（用户手动开的旧界面）或旧配置未重启**，`Subscribers: None` 就是铁证（RViz 压根没订阅 committed_path）。**收尾动作**：确认数据/发布/配置三对后，把"重启 RViz（按钮3 或本地命令）"交给用户，等用户反馈订阅者出现再判显示。**⚠️ 判定用户实际看哪个 RViz 的关键：`rostopic info <话题> | grep -A4 Subscribers`——若话题有数据但 Subscribers 列表里没有 /rviz 条目，先归因"RViz 显示项没订阅"（配置/类型/话题不匹配或加载旧配置），别往发布端查**

36. **⭐⭐ SUPER 分支纪律（2026-08-12 用户明确纠正：路径A/方案C 代码不能提交到 dev/4uav 主分支）**：本会话把 SUPER 的路径A/方案C 代码（8cafee3 路径A核心 + 0ae8f75 方案C）错误提交到了 **dev/4uav**（远程主分支，用户视为主干），用户纠正"SUPER那个不是分支，你应该创建一个新分支"。**正确做法**：SUPER 仓库的路径A/方案C 改动先 `git checkout -b feat/guide-path-c`（从 dev/4uav 当前状态分叉）再提交，dev/4uav 保持原状。用户拍板"保留现状，再有提交就在新分支继续"——**不回退 dev/4uav、不 force push**（远程已含 8cafee3；若用户独占分支想回退需明确确认）。**提交粒度**：SUPER 只 add 代码文件（super_planner.h/fsm.cpp/super_planner.cpp 等 .cpp/.hpp），**不带 .bak 备份文件**（留在工作区等清理）。**分支全景（2026-08-12 最终）**：SUPER = dev/4uav（保留现状）+ **feat/guide-path-c**（路径A+方案C，0ae8f75，已推远程）；task_run_scripts = feat/4uav-guide（402a86e 方案C配套已推）；NX_code = feat/4uav-guide（已推）。**后续提交纪律**：SUPER 路径A/方案C → feat/guide-path-c；task_run_scripts/NX_code → feat/4uav-guide。**推送顺序**：先子模块 NX_code → SUPER → 主仓库 task_run_scripts（主仓库提交含 scripts/NX_code 子模块指针，先推子模块避免远程悬空引用）。**教训：多仓库项目提交前先确认每个仓库的分支策略——用户把 dev/4uav 当主分支看待，功能代码应隔离到 feature 分支；别默认 dev/xxx 就是开发分支随意提交**

37. **⭐⭐ 方案 C 动态避障缺陷 = 障碍出现时飞机沿旧轨迹飞（2026-08-12 会话末，用户问"方案C能避免动态障碍吗"的诚实回答）**：方案 C（guide_path 直发）**不能可靠避障，实飞有撞障碍风险**——两条致命链：
    - **缺陷 1：障碍出现时飞机继续沿"旧轨迹"飞**：方案 C 下飞机执行的是 `committed_guide_path_`（每轮 replan 记住的航点）。障碍出现 → 走廊失败 → fallback → `global_skip_this_round_=true` → **下一轮 replan 不走注入块 → committed 不更新（还是旧航点）** → getOneCommandFromTraj 继续发旧航点 → 飞机沿旧路径飞 → 撞障碍。**机制**：committed 只在注入块（global_path_valid_ && !global_skip_this_round_ 时）保存，fallback 轮跳过注入块 = committed 冻结
    - **缺陷 2：backup MINCO 常失败**：走廊失败 → fallback A* 绕行路径 → backup MINCO 优化 → `[BackOpt] Opt failed, Omg or thr or Pos violation` / `[MINCO] TrajOpt failed` / `generateBackupTrajectory return OPT_FAILED`（常失败）→ 没有绕行轨迹 → 飞机没有"新路径"可切换 → 叠加缺陷 1 = 沿旧轨迹撞
    - **对比路径A（MINCO）为什么能避障**：每轮 replan → MINCO 生成新轨迹（带走廊约束）→ 障碍出现 → 走廊失败 → fallback A* → MINCO 绕行轨迹（若成功）→ 飞机执行新轨迹绕行 ✅。**方案 C 跳过 MINCO = 没有"每轮新轨迹" = 只有旧 guide_path = 避不了**
    - **安全结论（重要）**：**修复 backup MINCO 之前，方案 C 不要用于真机飞行**（有撞障碍风险）；地面测试可以，任何实飞前必须解决"障碍时停住"（方向 B：检测到障碍停止发送新位置→飞机悬停，至少保命）或"障碍时切回 MINCO 模式"（方向 A：混合）。**判读**：`grep -aE "fallback|backup|blocked" real_flight.log`——有 `Global path blocked` 无有效 backup 轨迹 = 检测到障碍但绕行失败（飞机将沿旧轨迹）；实飞前必须验证"障碍时飞机停住或绕开"，不能只看检测日志

38. **⭐⭐ 方案 D（混合 A+C）：guide_path 硬必经点 + 走廊安全检查（2026-08-12 会话末用户问"有没有结合A和C方案"时的设计，未实施）**：既保方案 A 的动态避障检测/路径优化、又保方案 C 的航迹跟踪准确性——**让 guide_path 的点成为 MINCO 的"硬必经点"（piece 边界），走廊从"形状约束"降级为"安全检查"**。代码依据：exp_traj_optimizer_s4.cpp `setupProblemAndCheck`（516-534 行）——`opt_vars.piece_num = hPolytopes.size()`（piece 数=走廊盒子数）、`opt_vars.points.resize(3, piece_num-1)`（中间点=盒子之间）——**MINCO 的 piece 边界（points）本来就是位置约束，若把 points 设成 guide_path 的点（而非盒子中心），轨迹就必须经过这些点（硬约束）**。设计：① guide_path 下采样 → 关键点序列（每 0.5m 一个，4.4m C 形 ≈9 点）② MINCO piece_num=关键点数+1，points=关键点（硬必经点）③ 走廊检测保留：每段检查是否安全 ④ 有障碍 → 走廊失败 → fallback A* 生成新绕行路径 → 新关键点序列 → MINCO 重新优化（轨迹经过新绕行点=避开障碍）⑤ 障碍消失 → 恢复原关键点。**效果对比**：严格循迹（必经点约束）✅ + MINCO 平滑 ✅ + 动态避障（fallback 新路径）✅ + 速度控制 ✅。**实现难点**：points 要匹配 guide_path（现在 points=盒子中心~11 个，要改成关键点~9 个，数量需重映射）；走廊约束从"形状约束"降级成"每段 isLineFree 安全检查"；fallback 路径要下采样成新关键点序列；改动量中等（~100 行）。**实施建议**：先修 backup MINCO 失败（也是 D 的基础——fallback 绕行需要 MINCO 能优化成功），再上 D；先做地面测试版验证轨迹经过关键点，再实飞。**教训：方案 C 是"航点播放器"（轨迹=航点、牺牲平滑和避障完整性），方案 D 是把 C 的严格循迹和 A 的避障重规划统一在 MINCO 框架里的正解，但依赖 MINCO 优化能成功（backup 失败问题必须先解决）**

39. **⭐⭐ 方案 E（双路径直发）被用户否掉："不能让SUPER没有作用"（2026-08-12 会话末）**：提出让 fallback 的 A* 绕行路径也直发（不优化，绕开 backup MINCO 失败）——被用户否：**这样 SUPER 退化成"传声筒"（只用 A* 搜索，MINCO/走廊优化全废）**，违背"用 SUPER 做规划"的初衷。**教训：绕过 SUPER 核心（MINCO/走廊）的"捷径"方案要慎重——用户买 SUPER 就是要它规划，不是当航点播放器；结合方案必须让 SUPER 真正参与（方案 F/D 的 MINCO 必经点方向），而不是绕开它**

40. **⭐⭐ 方案 F（= 方案 D 落地版）已实施三处改动（2026-08-12 会话末，编译中，备份 .bak_planF）**：在方案 C 基础上（SUPER feat/guide-path-c 分支）改 exp_traj_optimizer_s4.cpp + super_planner.cpp：
    - **① points 用 guide_path 点（487 行）**：`opt_vars.points.col(j) = opt_vars.waypoint_attractor.col(j);` → `= opt_vars.guide_path[i];`（作者注释里留的备选）
    - **② attractor 用 guide_path 最近点 + dead_d 压 0.05（465-473 行，核心）**：`waypoint_attractor.col(i) = interior` → 在 guide_path 里找离盒子中心最近的点赋给 attractor；`dis = std::min(dis, 0.2)` → `0.05`
    - **③ 方案 C 直发改兜底（super_planner.cpp:342-350）**：`bool minco_traj_ok = (total_dur > 1e-6) && !traj_finish;` 条件改为 `committed_guide_valid_ && size>=2 && !minco_traj_ok`——**MINCO 有轨迹时走 MINCO（attractor 贴 guide_path），只有 MINCO 无轨迹时才用 guide_path 直发兜底**
    - **⚠️ 关键认知（实施中发现，决定方案 F 可行性）**：MINCO 库 `minco.setParameters(opt_vars.points, opt_vars.times)`（757 行）——**points 是 MINCO 的内部优化变量，不是硬约束**！MINCO 优化时 points 会变化（斜切）。**改 points 只是改初值，不能保证轨迹经过**。**真正让轨迹贴 guide_path 的是 attractor 机制（软约束）**：`waypoint_attractor` + `dead_d`——attractor 用 guide_path 点 + dead_d 压小 → MINCO 优化时 points 被拉向 guide_path 点（强软约束）。**所以方案 F 的核心是改动 ②，改动 ① 只是辅助（好初值）**
    - **注意**：`opt_vars.minco.setConditions(headPVAJ, tailPVAJ, piece_num)`（595 行）只设首尾条件+piece 数，中间点不是 setConditions 的硬约束——**想真正硬约束要改 MINCO 库/代价函数（代价级），软约束（attractor）是现有框架内能做到的极限**
    - **编译状态（会话末）**：make -j4 在 fouruav 上跑，**Orin NX 编译大型模板代码（MINCO/lbfgs）时负载过高会导致 SSH 无响应（plink 超时）**——不是死机，等负载降下来（`ps aux | grep -cE 'make|cc1plus'` = 0）再确认 fsm_node 时间戳；fsm_node_ros1.cpp.o 编译极慢（96% 卡很久）是模板实例化正常现象。**教训：fouruav 上 make -j4 期间 SSH 可能超时，别反复重试连不上（浪费轮次），先 Test-NetConnection 确认 22 端口通=网络没问题，然后等编译完成再连；或让用户本地看屏幕确认是编译中（CPU 高）还是卡死（OOM）**
    - **✅ 编译最终结果（2026-08-12 晚，回原开发机 <NET_WLAN_A>.45 重实施后）**：**`catkin_make -j2 -DROS_EDITION=ROS1` 成功，0 错误，fsm_node 19:18 新编译**（方案F 三处改动生效）——-j2 没卡死系统、SSH 全程可连。**确认编译完成的判读**：`ps aux | grep -E 'make -j2|cc1plus'` = 0 + `grep -aE 'Built target fsm_node' /tmp/catkin_make_*.log` 出现 + `ls -la devel/lib/super_planner/fsm_node` 时间戳=今天。**⚠️ 旧开发机上的二进制可能是 8月2日 旧版**（编译中断残留）——改完代码后必须重新编译确认时间戳，别用旧产物测（旧产物没有方案C/F 逻辑）。**方案F 三处改动被切分支丢弃后可复现**（坑42⑤），本会话重实施 + 编译通过 = 改动清单已验证可复现

41. **⭐⭐ 用户核心需求确认：复飞过程必须有避障能力（静态+动态），避障是第一优先级（2026-08-12 会话末，用户"我的需求就是保证飞机在复飞的过程中有避障能力（静态+动态）"）**：这是方案选择的最高准则，覆盖"严格循迹"——**方案 C（guide_path 直发）不可用于真机飞行**（坑 37：障碍时沿旧轨迹飞+backup MINCO 失败=撞障碍风险），只能地面测试。**所有方案的避障都依赖同一个东西：MINCO 优化成功**（走廊检测→fallback A* 绕行路径→MINCO 优化成轨迹→飞机执行；MINCO 失败=绕行轨迹出不来=避障失效）。**"Minco backup_traj opt failed" / "OptimizationExpTrajInPolytopes failed" 是当前避障失效的根因**，不是方案问题。
    - **MINCO 失败的已知原因（按概率排序）**：① **速度参数不匹配**——`max_vel: 2.0` / `max_acc: 4.0` 对短轨迹（4.4m C 形）太快，MINCO 无法在约束内完成急加速 → `Opt failed, Omg or thr or Pos violation`（坑 24 已建议短距测试 max_vel→0.5 / max_acc→1.0，**避障优先下这是首要尝试**）；② **起点在障碍/未知里**（坐标系错位坑 29 / 起点被包围坑 13）；③ **走廊盒子问题**（corridor_bound_dis=0.39 有时失败）。
    - **避障优先的调试路径（用户同意）**：① 先修 MINCO 失败（速度参数下调 max_vel 0.5 / max_acc 1.0 + 测试点重启雷达保坐标系 + 起点周围清空）→ ② 验证路径A 避障链路（障碍出现→走廊失败→fallback A*→MINCO 绕行轨迹成功→飞机绕行）→ ③ 再调循迹精度（方案 F 的 attractor=guide_path 点改动保留，等 MINCO 能成功时自动生效——**方案 F 的代码改动不要回退，它是"MINCO 成功时贴航迹"的开关**）。
    - **判读"MINCO 失败原因"**：`grep -aE "Opt failed|TrajOpt failed|Omg|thr|Pos violation|NaN" real_flight.log`——`Omg or thr or Pos violation` = 速度/动力学约束（调慢速度）；`function value became NaN or Inf` = 终点约束冲突（坑 28 的 goal 吸附 vs guide_path 终点，已用坑 30 的吸附保护解决）。
    - **编译负载经验补完**：make -j4 在 fouruav 上会把 CPU+内存吃满导致 SSH 完全无响应（连握手都完不成，但 22 端口 Test-NetConnection 显示通）——**编译期间别反复重连，等 10 分钟负载降下来再查**；若用户本地屏幕确认 OOM（内存满+无响应），`sudo reboot` 后重编（代码已保存不受影响）；**可改用 `catkin_make -j2` 降负载**（慢但不会卡死系统）。

42. **⭐ 编译卡死收尾 + 分支切回 + 遥控器虚拟投影（2026-08-12 会话末实机，方案 F 编译 -j4 卡死系统后的操作链）**：
    - **① 停编译进程的权限坑**：系统高负载 SSH 连不上时让用户本地 `pkill -f "catkin_make"` 会报 **`pkill: killing pid XXX failed: 不允许的操作`**——编译进程可能是 root 或系统级启动，普通用户 pkill 权限不够 → **用 `sudo pkill -f "catkin_make"; sudo pkill -f "make -j"; sudo pkill -f "cc1plus"; sudo pkill -f "g++"`**；**更干净的选择是直接 `sudo reboot`**（编译进程随系统重启消失，一了百了，代码改动已写入文件不受影响）。**教训：pkill 权限不够时别纠结逐个杀，sudo reboot 最快；重启前确认代码已保存（git status/文件层面）**
    - **② 遥控器虚拟投影失效（2026-08-12 实机）**：编译卡死期间用户报"遥控器虚拟投影失效"——**虚拟投影（遥控器屏幕地图/姿态显示）依赖 QGC/MAVLink 地面链路**，而 px4.launch 的 gcs_url 已改成固定地面站 `udp://:14550@<NET_GCS>.123`（坑 23，用户故意改的）→ 系统高负载时 MAVROS/QGC 链路响应慢 → 虚拟投影刷新不了。**判断顺序**：先 `sudo reboot` 恢复系统 → 若重启后恢复 = 系统负载问题；若仍失效 = gcs_url/地面站配置问题（查坑 23 的 px4.launch diff，确认地面站 IP 是否匹配）。**教训："遥控器虚拟投影失效"先归因系统负载/地面链路，别往飞控/遥控器本身查**
    - **③ 切回系统初始分支被本地修改挡住（2026-08-12 实机）**：用户要"切回系统刚开始的分支"（SUPER=dev/4uav、task_run_scripts=main），`git checkout` 报 **`error: 您对下列文件的本地修改将被检出操作覆盖`**（SUPER 的 super_planner.cpp 有方案C/F 的本地修改 + task_run_scripts 的 real_flight.launch/imu_tf.cpp）→ **先 stash 再切**：`git stash push -m "planF_20260812" super_planner/src/super_core/super_planner.cpp`（可指定文件）→ `git checkout <目标分支>`；**stash 后改动安全保存在 `git stash list`，切回原分支时 `git stash pop/apply` 恢复**——方案C/F 代码不能丢（未提交的核心改动）。**⚠️ NX_code 可能没有 main 分支**（只有 feat/4uav-guide 或 master——`git branch -a` 先看有哪些分支再切，别假设 main 存在）。**教训：切分支被本地修改挡住 = 先 stash（保存改动）再切，别 `checkout -f` 强切（会丢未提交代码）；多分支切换时 stash 是"移动未提交改动"的标准动作，pop 前确认目标分支**
    - **④ 用户拍板"修改不必保存"时（2026-08-12 会话末实机，与 ③ 相反的决策路径）**：用户明确"现在的修改不必保存"→ 用 `git checkout -- <file>` 逐个丢弃（不是 stash！）：SUPER `git checkout -- super_planner/src/super_core/super_planner.cpp` + 切分支后**还会漏掉 exp_traj_optimizer_s4.cpp**（方案F 的第二处改动文件，`git status --short` 显示 M 才抓得到）→ 切回后 `git status --short` 确认 0 残留 + `git submodule update --init` 同步子模块指针（task_run_scripts 切回 main 后 scripts/NX_code 显示 M）。**教训："丢弃修改"也要逐个文件清（git status 复查），别以为切分支就干净了；切回后必查子模块指针**
    - **⑤ 切回初始分支后回到原开发机重新实施方案 F（2026-08-12 会话末）**：用户"切换回之前的电脑"（<NET_WLAN_A>.45 恢复在线，`Test-NetConnection` 两个地址都通）后确认——SUPER=feat/guide-path-c（方案C 代码 0ae8f75 在）、task_run_scripts=feat/4uav-guide、**但方案F 改动被切分支时丢弃（`grep -c "方案F"` = 0）**、fsm_node 还是 17:15（方案C 编译版可用）。**方案F 三处改动可复现（重实施成功）**：① exp_traj_optimizer_s4.cpp 487 行 `opt_vars.points.col(j) = opt_vars.guide_path[i];`（作者注释留的备选）；② 同文件 465-473 行 attractor 用 guide_path 最近点 + `dis = std::min(dis, 0.05)`；③ super_planner.cpp:342-350 直发改兜底（`bool minco_traj_ok = (total_dur > 1e-6) && !traj_finish;` 条件改为 `committed_guide_valid_ && size>=2 && !minco_traj_ok`）。**教训：方案 C/F 的未提交改动被丢弃后可按上述清单重新实施（改动可复现），前提是记下改动内容——这再次说明"改动清单要随时记录（SKILL.md/SUPER.md）"的价值**
    - **⑥ MINCO 失败的速度假说已排除（2026-08-12 会话末实机确认）**：坑 41 曾把"max_vel 2.0→0.5"列为首要尝试——**实查 task_run_scripts/config/real_flight.yaml 已是 `max_vel: 0.3` / `max_acc: 0.6`**（此前短距测试已调过，坑 24）→ 速度已极低仍出现 `Opt failed, Omg or thr or Pos violation` → **MINCO 失败不是速度问题**，重新聚焦到：坐标系错位（坑 29，测试点重启雷达）/ 起点被障碍包围（坑 13）/ 终点约束冲突（坑 28-30）。**教训：调参前先 `grep -n "max_vel\\|max_acc" config/real_flight.yaml` 看实际值，别凭记忆假设是 2.0/4.0——配置可能已被之前会话改过**
    - **⑦ 编译用 `catkin_make -j2` 防卡死（2026-08-12 会话末实机）**：坑 41 的 "-j4 吃满 CPU+内存导致 SSH 无响应"教训落地——重新编译方案 F 改用 **`catkin_make -j2 -DROS_EDITION=ROS1`**（负载减半，慢但系统不卡死，SSH 保持响应）。**教训：fouruav（Orin NX 8GB）上编译大型模板代码（MINCO/lbfgs）默认用 -j2，-j4 只在系统空闲/用户本地操作时用**；编译期间 `ps aux | grep -E 'make|cc1plus'` 确认进程在跑，日志 `tail -f /tmp/catkin_make_*.log` 看进度
    - **⑧ 方案 F 测试计划（避障优先，会话末给出未执行）**：前提=测试点启动雷达（坐标系正确）+ 按钮3 + 起飞 0.6m；测试1 静态避障（提前放凳子/箱在前进段 (1.0,0) 附近→观察 `Global path blocked` + MINCO 绕行轨迹 SUCCESS + 飞机绕过）；测试2 动态避障（飞行中你走到轨迹前方→走廊失败→fallback→绕行→走开恢复）。判读：`GenerateExpTrajectory SUCCESS`（绕行成功）vs `OptimizationExpTrajInPolytopes failed`（仍失败→查 MINCO 参数）。**用户核心需求（坑 41）贯穿：避障是第一优先级，方案 F 的改动保留不回退（MINCO 成功时自动贴航迹）**


---

### 来源技能 `uav-super-flight-replay`

> 运维 fouruav（Orin NX）上的 SUPER 规划器轨迹复飞系统。覆盖两套复飞架构（Path Replay 纯回放 vs SUPER 航点任务规划避障）、地面验证全流程（不装桨）、QGC 按钮 UserCmd1/2/3 机制、航迹录制与时间戳归档（data_guidepoints/）、super_to_px4_bridge 状态机（自动起飞悬停 + CH10 拨杆触发）、多个已实测验证的坑（航点话题不匹配、pkill 模糊匹配误杀推流/SLAM、录制停止 NUL 字节损坏、服务重启需 ROS 环境、同名节点顶替、odom_to_mavros 单实例锁/vision_pose 限速、virtual_ground 量化 bug、fix_map_origin 全屏障碍、全向光线 omni 清残留、停车避让 AVOID_WAIT 状态机、fsm_node 崩溃(seed line too long)卡死根治、p_miss 概率语义、FAPP 动态分割调研/失败教训、撞墙(未知当自由 frontend_in_known_free)、摩擦(min dis 0.1 膨胀加宽)、robot_r 飞机半径纳入、bridge 前馈禁用排查摇摆、自主探索/全局地图/FUEL/OctoMap/前沿/frontier）。触发条件：用户提到 SUPER/复飞/航点任务/轨迹回放/real_flight/waypoint/My_Flight_Path/guidepoints/按钮1-3/odin/动态避障/障碍残留/全向光线/停车避让/卡死/全屏障碍/not ready/FAPP/动态分割/seed line/崩溃/撞墙/摩擦/未知区域/雷达范围/左右摇摆/前馈/robot_r/飞机半径/自主探索/探索算法/全局地图/前沿/frontier/FUEL/OctoMap/exploration/长距离复飞/滑动窗口/map_sliding_en/fix_map_origin/地图原点/CH9/中断返航/返航方案/mid360_real_flight/odin_real_flight/切分支/git pull。

#### 系统架构：两套复飞，勿混淆

| | 系统A: Path Replay（纯回放） | 系统B: SUPER 航点任务（规划避障） |
|---|---|---|
| 代码 | `nx_command_server.py` 内置 daemon | `real_flight.launch`（fsm_node + super_to_px4_bridge + waypoint_mission） |
| 数据流 | 读 CSV → 发 `/mavros/setpoint_position/local` | 航点文件 → waypoint_mission → `/waypoint_goal` → fsm_node → `/planning/pos_cmd` → bridge → `/mavros/setpoint_raw/local` |
| 指令通道 | UDP 6003（RecordPathStart/Stop/Replay/AbortReplay） | QGC 按钮 UserCmd1/2/3（走 UDP 6003） |
| 状态接口 | HTTP 6004（/check_replay_status、/check_pose、/check_file） | 日志 + `/fsm_node/rog_map/occ` 等话题 |
| 特点 | 无避障，录什么飞什么 | 有地图/规划/避障，安全机制完整 |

**实际生效配置在 `~/catkin_ws/src/task_run_scripts/`，不是 `~/catkin_ws/src/SUPER/`**（SUPER 仓库是上游源码副本）。

#### 关键文件地图

```
~/catkin_ws/src/task_run_scripts/
├── launch/real_flight.launch      # 启动 FSM+bridge+waypoint（支持 waypoint_data_path:= 覆盖数据源）
├── config/mid360_real_flight.yaml # ✅2026-08-24 起 SUPER 规划配置（MID360S 版；⚠️ 原 real_flight.yaml 已改名！）
├── config/odin_real_flight.yaml   # odin 版规划配置（按传感器分文件）
│   ⚠️ 新版 main 中所有对 config/real_flight.yaml 的引用均已失效：MID360S 用 mid360_real_flight.yaml、
│   odin 用 odin_real_flight.yaml；launch 也分家（launch/fastlivo2/mid360_real_flight.launch、
│   launch/odin/odin_real_flight.launch）。改配置前先确认机载实际加载哪份（坑28）
├── config/waypoint.yaml           # 航点任务：goal_pub_topic/odom_topic/start_trigger_type/waypoint_downsample_dist
├── config/super_to_px4_bridge.yaml# bridge 参数：前馈开关、init_goal(z=1.2)、failsafe/geofence
├── data/real_mission.txt          # 按钮1 默认航点（空格格式 x y z switch_dis）
├── data_guidepoints/              # 📁 录制归档目录（guidepoints_YYYYMMDD_HHMMSS.txt，时间戳命名不覆盖）
└── scripts/NX_code/
    ├── nx_command_server.py       # 核心服务：UDP 6003 指令 + HTTP 6004 状态（Cython 保护，慎改）
    ├── nx_command_server2.py      # 包装器（仅 import + 启动），常驻进程跑的是它
    ├── User_define_commands.yaml  # QGC 按钮 UserCmd1/2/3 定义
    ├── My_Flight_Path.txt         # 旧录制文件（改造后只作历史，不再写入）
    └── start_qgc_servers.sh       # 常驻服务启动脚本（roscore + 三大微服务）
```

#### 录制与归档（2026-08 改造后）

- `RecordPathStart` → 走轨迹 → `RecordPathStop`（UDP 6003 发明文）
- **改造后录制直接生成** `data_guidepoints/guidepoints_<时间戳>.txt`（永不覆盖），PATH_FILE 全局变量联动 Replay
- **RecordPathStop 同步更新 `guidepoints_new.txt`**（固定文件名、始终最新）；按钮3 直接加载它：`waypoint_data_path:=.../data_guidepoints/guidepoints_new.txt`（2026-08-10 后不再用 ls -t，固定路径最稳；用户后续若又要 ls -t 版可回退）
- 航点文件格式：CSV 7 列 `x,y,z,qx,qy,qz,qw`（带四元数）；空格格式 `x y z switch_dis` 支持逐点切换距离（CSV 用全局 switch_dis）

#### QGC 按钮机制

- `User_define_commands.yaml` 定义 UserCmd1/2/3；⚠️ **改 command 本身后必须重启 nx_command_server 才生效**（坑22：每次点击重读 yaml 是旧版行为）
- ⚠️ 08-24 按钮最终态（用户拍板）：**按钮1=复飞**（发布器 guidepoints_new.txt→/global_path + **`roslaunch task_run_scripts real_flight.launch config_path:=.../mid360_real_flight.yaml rviz_config:=.../mid360_top_down.rviz use_waypoint:=true waypoint_data_path:=.../endpoint.txt` 全显式（⚠️ wrapper 不透传 waypoint_data_path 会发错 goal，坑71 修正版）**，CH10 触发）；**按钮2=关闭SUPER**（`|| true` 防断链 + 全字符类 pkill + echo button2_done，坑67）；**按钮3="空按钮"**（用户自改，原复飞 command 在备份 .bak_adjust2）
- ⚠️ 按钮 launch 写法（坑69 修正）：**纯文件名 `roslaunch task_run_scripts mid360_real_flight.launch` 可用**（roslaunch 递归搜索 launch/ 树含子目录）；**绝不能用带目录前缀的形式** `fastlivo2/mid360_real_flight.launch` 或 `launch/fastlivo2/...`（含 `/` 被当文件系统相对路径 → RLException）。real_flight.launch 默认 config_path 指向已删除的 config/real_flight.yaml（回归点），用 mid360 包装（默认值正确）或显式 config_path 兜底

#### 地面验证流程（不装桨，分三步）

**① 传感器+SLAM+飞控链路**（复飞前提）：
```bash
cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code && bash start_fastlivo2.sh false
rostopic hz /cloud_registered --window 10      # ~10Hz（SUPER 地图输入，最关键）
rostopic hz /aft_mapped_to_init --window 10    # FAST-LIVO2 里程计
rostopic echo /mavros/state -n 1               # connected: true
```

**② Path Replay 状态机**（只验状态流转，安全）：
```bash
curl -s http://127.0.0.1:6004/check_replay_status   # IDLE

```

**③ SUPER 链路**：启动 real_flight.launch → 发 `/waypoint_goal` 目标 → 日志见 `Get goal at [x,y,z]`（z 不被覆盖=坑#1修复生效）

**单独验证抽稀/加载（不启动整个 SUPER，最安全）**：
```bash
timeout 6 rosrun mission_planner waypoint_mission \
  _config_path:=~/catkin_ws/src/task_run_scripts/config/waypoint.yaml \
  _data_path:=~/catkin_ws/src/task_run_scripts/data_guidepoints/guidepoints_new.txt 2>&1 | grep -E "Load|Waypoint"

```

**航迹可视化检查（本地 Windows，matplotlib）**：拉回 guidepoints_new.txt → 复刻 config.hpp 抽稀逻辑（`.norm()` 3D 距离 ≥ 阈值保留）→ 生成俯视图(原始vs抽稀对比+编号)+3D+高度剖面三张图，可一眼看出：抽稀点数、高度是否稳定（z 全程 >0.3m 无降落段 ✅）、起点终点位置、是否有障碍吸附嫌疑（航点 z 突变）。判读要点：原始点数/抽稀后点数/保留间距(应≈阈值)/z 范围。

#### bridge 状态机（super_to_px4_bridge.cpp Phase 1-5）

```
Phase1 预发布指令(切OFFBOARD前必须已有指令流) → Phase2 自动解锁/切OFFBOARD(auto_arm/auto_offboard=false=手动)
→ Phase3 等OFFBOARD生效(期间持续发init_goal) → Phase4 🚁自动起飞到init_goal(z=1.2m)悬停(误差<0.1m,30s超时)
→ Phase5 主控制循环(跟踪/planning/pos_cmd + RC丢失/越界/断连失效保护)
```

- **起飞悬停是 bridge 自动做的**，与 CH10 无关；CH10 只控制 waypoint_mission 何时开始发航点
- **CH10 触发条件：仅下降沿**（`last_ch_10>1500 && ch_10<1500`）——必须先拨高位再拉低，一直停低位不触发。⚠️ 代码实测 `pitch_up = channels[1]<1200` 计算了但**未参与 if 判断**（文档说"且 pitch<1200"是错的，实际只看 CH10）

#### 已验证的坑（全部实测）

1. **航点话题不匹配（功能性问题）**：`waypoint.yaml` 的 `goal_pub_topic` 必须是 `/waypoint_goal` 才走航点通道（`override_height=false` 不覆盖高度）；若是 `/move_base_simple/goal` 会落进 click 通道（`goalCallback`，`override_height=true`）→ 高度被强制为 `click_height`。判据：发目标后日志 `Get goal at [1.0, 0.0, 1.2]`（z 保留=对）/`[1.0, 0.0, 1.0]`（被覆盖=错）
2. **pkill 模糊匹配误杀**：按钮2 原命令 `pkill -f 'rviz'` 会杀掉 `rviz_full_stream2.py`（遥控器推流→黑屏）和 SLAM roslaunch（命令行含 "rviz:=false" 字样）。**必须精确匹配**：`pkill -f 'top_down.rviz'`；SUPER 相关统一用 `pkill -f 'fsm_node'/'super_to_px4_bridge'/'waypoint_mission'/'map_to_world'`
3. **录制停止产生 NUL 损坏**：`RecordPathStop` 用 terminate() 杀 daemon，文件尾部残留 `\x00` 字节导致 CSV 解析失败。修复：python 截断到首个 NUL 偏移（`raw.find(b'\x00')`），丢弃不完整尾行
4. **nx_command_server 重启必须带 ROS 环境**：nohup 裸启动报 `ModuleNotFoundError: rospy`。正确：`nohup bash -c 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && exec python3 -u nx_command_server2.py' &`
5. **同名节点顶替**：重复启动 real_flight.launch 触发 "Reason: new node registered with same name" 杀掉旧实例，表现为 RViz 白屏+进程全死。同时间只跑一个 SUPER
6. **日志别查错文件**：前台 roslaunch 输出在终端（/dev/pts/N），nohup 输出在重定向文件；判别实际生效参数用 `rosparam get /waypoint_mission/`（比猜日志准）
7. **抽稀**：`waypoint_downsample_dist` 0.1m→83点/1038点，0.5m→14点；改后重启 launch 生效
8. **代码内路径拼接必须与按钮 command 路径逐字符一致**（2026-08-10 实测，用户报"没有保存到相应文件"）：RecordPathStart 第一版用 `os.path.dirname(os.path.abspath(__file__))` 拼 `data_guidepoints` → `__file__` 是 NX_code/nx_command_server.py → 实际生成在 `scripts/NX_code/data_guidepoints/`，而按钮3 的 `ls -t` 找 `task_run_scripts/data_guidepoints/` → 两边不一致、LATEST_GUIDE 为空、roslaunch 不启动。修复：`os.path.dirname` 套三层回到 task_run_scripts 根再拼。**判读顺序**：先看服务日志 `[RecordPathStart] 新航迹文件: <绝对路径>` 打印（确认代码执行 + 实际写到哪），再 `find ~ -name 'guidepoints_*'` 定位，别急着怀疑改动没生效——本会话代码其实生效了，纯粹路径写偏
9. **start_qgc_servers.sh 会被重新拉起顶掉 agent 手动启动的服务**：agent 重启 nx_command_server 后，守护/手动又跑了一次 start_qgc_servers.sh，把 6004 监听换回旧代码进程。**判断当前服务跑哪版代码：对比进程 `ps -eo pid,lstart` 启动时间 vs 文件 `stat -c '%y'` 修改时间**；改动被顶掉就再重启一次，重启后立即 curl 健康检查验证
10. **pkill/pgrep -f 会误杀 plink `-m` 脚本自身**（exit 128 无报错）：`-m` 脚本内容作为 `bash -c '整段脚本'` 的 argv，脚本文本出现在自身命令行里 → pkill 模式匹配到脚本里的启动命令字符串就自杀。规避：①正则字符类 `pkill -f 'nx_command_server[2]\\.py'`；②杀+启拆两段脚本；③`ss -tlnp` 拿 PID 再 kill
11. **复飞起飞后中断：MINCO 轨迹优化 NaN**（2026-08-10 真机实测，现象：起飞悬停成功→第一段真的飞了（FOLLOW_TRAJ + ReplanOnce succeed）→`[MINCO] TrajOpt failed, The function value became NaN or Inf` 无限重试→`SUPER command timeout`→回 WAIT_GOAL→无指令落地贴地转桨）。根因链：FSM `setGoalPosiAndYaw`（fsm.cpp:191）对每个目标调用 `getNearestInfCellNot(OCCUPIED, click_point, gi_.goal_p, 3.0)`——**目标点在地图被判为占用时，被吸附到 3m 内最近空闲格，坐标被改写**（实测 waypoint_mission 发布 z=0.554，FSM 收到 z=0.825，x/y 也偏移 ~0.05m）。目标高度与起飞高度(0.60)/航点高度(0.55)全不一致 → MINCO 短轨迹优化数值无解 → NaN → 复飞死循环。**判读**：对比日志 `Pub goal to x y z`（waypoint_mission 发布值）vs `Get goal at x y z`（FSM 吸附后值），不一致=吸附发生。**验证**：起飞前 RViz 看 1 号蓝点是否落在灰色障碍格里 / 飞机模型与地图是否对齐（区分"地图真实有障碍" vs "坐标系错位"）
12. **"贴地转桨没起飞"判读顺序**：①先 `grep -a 'Mavros triggered.' real_flight.log`（触发有没有——本会话曾误判"没触发"，实际触发了（日志中部 323 行），是规划失败不是触发失败，**别只看 tail -30**）；②再对比 `Pub goal` vs `Get goal`（吸附检查，坑#11）；③最后查 `MINCO`/`PlanFromRest`/`GenerateExpTrajectory` 错误。起飞悬停成功 + FSM 回到 WAIT_GOAL + bridge POS_HOLD = 规划失败落地，不是起飞问题
13. **RViz 航点可视化**：`/waypoint_mission/mkr`（蓝线+橙色到达球+黑色编号，**静态全量**，仅 RViz 订阅者数量变化时发布一次——RViz 后启动看不到就取消勾选再勾上触发重发）；`/fsm_node/visualization/goal`（**黄点=当前目标，动态**，每个新航点删旧画新，Color::Yellow 线宽 0.3）。top_down.rviz 默认无 mkr 显示，需手动加 MarkerArray 订阅 `/waypoint_mission/mkr`（或按本会话改法在 rviz 配置 goal 项后插入 Waypoints 项）
14. **catkin_make 必须带 `-DROS_EDITION=ROS1`**（2026-08-10 实测，裸 `catkin_make -j4` 报 `livox_ros_driver2/CMakeLists.txt:224 find_package(ament_cmake_auto)` 失败）：livox_ros_driver2 是 ROS1/ROS2 双版本 CMake，用 `ROS_EDITION` 变量切换，Noetic 没有 ROS2 的 ament 依赖。**正确：`cd ~/catkin_ws && catkin_make -j4 -DROS_EDITION=ROS1`**。判断：错误里出现 ament/rosidl/ROS2 关键词 = 版本选错。注意 fastlivo_mapping/realsense2_camera 等都在全量编译内，Jetson 全编 10-20 分钟，后台 nohup + 轮询 `/tmp/catkin_make_*.log` 的 `error:` 计数
15. **obstacle_to_px4（B 分支新节点）必须用 `/cloud_registered` 而非 `/livox/lidar`**（2026-08-10 实测，现象：launch 正常启动但日志一直 `[obstacle_to_px4] no cloud received yet, stop publishing`）：节点 `lookupTransform(target_frame=aft_mapped, msg.frame_id=livox_frame)` 失败——**`livox_frame` 不在 TF 树里**（livox 驱动只发点云不发该帧），而 FAST-LIVO2 发布的动态 TF 是 `camera_init→aft_mapped`（`/aft_mapped_to_init` 消息即此变换）。**修复：mid360.yaml 的 `cloud_topic: /livox/lidar → /cloud_registered`**（/cloud_registered 的 frame_id 是 camera_init，有 camera_init→aft_mapped TF，lookupTransform 立刻可用）。launch 里的 `livox_to_aft_mapped` 静态变换（livox_frame→aft_mapped）在 livox_frame 不存在时是死配置。验证：`rosrun tf tf_echo camera_init aft_mapped` 有输出=TF 通；`rosrun tf tf_echo livox_frame aft_mapped` 报 "does not exist"=就是这个坑
16. **拉取别人分支前必须快照保护**（2026-08-10 实操，拉 feat/obstacle-to-px4）：task_run_scripts 是 catkin_ws 子模块（`.git` 是 48 字节 gitdir 文件非目录），scripts/NX_code 是**嵌套子模块**（gitdir 指向 `modules/scripts/NX_code`）。本地修改未提交时 git 帮不上忙，**用 tar 快照兜底**：`tar czf ~/task_run_scripts_backup_<日期>.tar.gz --exclude='task_run_scripts/log' --exclude='task_run_scripts/.git' -C ~/catkin_ws/src task_run_scripts`（含未跟踪的 data_guidepoints/ 和 guide_Readme.md）。拉取流程：①先停服务（杀 6004）②嵌套子模块**各自 stash**（`cd scripts/NX_code && git stash`，子模块有自己独立的 stash 栈）③`git checkout <分支>`（会跟随子模块指针）④`git submodule update --init --recursive` ⑤catkin_make -DROS_EDITION=ROS1。恢复：tar 解包或 stash pop（可能冲突需 diff 合并）。拉取前 `git fetch` + `git log --oneline HEAD..origin/main` 看清差异再动手
17. **ROG-Map 动态障碍物原理（"人走过的障碍块残留"）**：ROG-Map 不追踪移动物体，靠**射线概率更新**（命中点 p_hit↑ 变占用；射线穿过的体素 p_miss↓ 变空闲；p_occ=0.80/p_free=0.30/p_min=0.12/p_max=0.97 为阈值）。**人走开后必须被新激光帧重新扫到（p_miss 作用）障碍才会消失**；飞机没动/人出视野→旧块永远残留。对飞行的影响：残留障碍→航点被判占用→`getNearestInfCellNot` 吸附（坑#11）→MINCO NaN。对策：起飞前原地转一圈重扫工作区 / `use_fov_cut: true` 让视野外旧数据失效 / 调低 p_occ
    - **⚠️ 最终根因检查（2026-08-11 实机：用户报"动态障碍没有被及时去除"时先查这个）**：real_flight.yaml 的 `raycasting.enable: false`（**默认是关的**！注释"与ESDF二选一"）→ **p_miss 射线清除根本不执行**，只有 p_hit 命中更新 → 障碍块**永远不会消失**（不是"延迟"是"永不"）。**修复：`raycasting.enable: true`**（恢复 p_miss 清除，障碍几秒内消失）；若 enable 是被故意关的（性能/地图稳定性考虑），改调 `p_free`/`p_miss` 或检查 ESDF 清除逻辑。**判读顺序：先 `grep -A12 'raycasting:'` 看 enable 值，再谈概率参数**。注意此配置在 SUPER 仓库 dev/4uav 的 real_flight.yaml（路径 A 改的就是这份），task_run_scripts 副本可能另有一份——改 launch config_path 实际指向的那份
18. **obstacle_to_px4 通了之后 PX4 侧避障仍默认关闭**（2026-08-10 实测，B 分支避障链路最后一环）：节点出 LaserScan 到 `/mavros/obstacle/send`（20Hz）≠ PX4 在避障。链路：`/cloud_registered → obstacle_to_px4(36扇区) → /mavros/obstacle/send(LaserScan) → MAVROS distance_sensor 插件(px4_pluginlists.yaml 已启用) → MAVLink OBSTACLE_DISTANCE → PX4`。**PX4 端 `COM_OB_ACT=0`（默认关闭）会直接忽略障碍数据**。需在 QGC 或命令行开：`COM_OB_ACT=1`（启用避障）、`COM_OB_RADIUS=2~5`（生效半径，首测先用 2）、`COM_OB_W_HGT=1.5`（高度带宽）、`COM_OB_PRM_AV=0.5`（安全裕度）；命令行：`rosservice call /mavros/param/set "param_id: 'COM_OB_ACT' value: {integer: 1}"`。验证参数：`rosservice call /mavros/param/get "param_id: 'COM_OB_ACT'"`。⚠️ COM_OB_ACT=0 时别误以为避障在工作；开启后先小半径低速真机验证。另注意节点 status 话题 `min_dist` 若持续 ~0.3-0.4m，可能是机架/雷达自身遮挡或环境近物，PX4 会一直认为有近距障碍

19. **改 SUPER 源码编译错误：`last_exp_traj_time_pos was not declared`**（2026-08 路径 A 开发实测）：在 generateExpTraj 的 guide_path 注入块里引用 `last_exp_traj_time_pos.clear()`，但该变量在函数**后段的热启动分支才声明**，注入块在它之前 → 编译报错。修复：删除该行（guide_path 已非空时热启动分支本就不会走，clear 纯属多余）。教训：**注入点选在函数前部时，只引用函数早期已声明的变量**（如 guide_path/guide_stamp/robot_state_/cfg_/gi_），引用后段局部变量必编译失败。另外括号平衡判断：`grep -c '{'` vs `'}'` 差 1 时先对比 `.bak_globalpath` 备份——**原文件本就差 1（宏/字符串含括号），不是注入引入的**，别浪费时间修

20. **git 切分支前工作区有未提交修改 → checkout 静默失败 → 新分支建错基底**（2026-08-11 实操）：`git checkout main && git checkout -b feat/4uav-guide` 在避障分支有未提交修改（config/MID360_Fly.yaml 等）时，checkout main 报错但仍继续执行了 `checkout -b` → 新分支从 **feat/obstacle-to-px4** 分出而非 main。修正：①`git stash push -u -m "temp"`（-u 连未跟踪一起藏）②checkout main ③`git branch -D` 删错分支 ④重新 checkout -b ⑤从 stash 恢复：跟踪文件用 `git checkout stash@{0} -- <file>`；**未跟踪文件不在 `git checkout --` 能取的范围，要从 `stash@{0}^3`（untracked 树）取：`git checkout stash@{0}^3 -- data_guidepoints guide_Readme.md`**（`git stash show --include-untracked --name-only` 为空时 ^3 树仍有内容）。教训：**checkout 前先 `git status --short` 确认工作区干净；`checkout -b` 前确认 `git branch --show-current` 已在目标基底**

21. **避障分支 ↔ 用户分支来回切换的完整流程**（2026-08-11 实操，路径 A 落地时在 feat/obstacle-to-px4 与 feat/4uav-guide 之间反复切）：
   - **切走前**：避障分支有未提交修改（config/MID360_Fly.yaml、mid360.yaml、launch/obstacle_to_px4.launch）会挡住 checkout → 逐个 `git stash push -m "temp_xxx" <文件>`（**只 stash 该分支的文件**，别 `-u` 全藏，否则未跟踪的 data_guidepoints 会被卷走）
   - **未跟踪文件冲突**：data_guidepoints/ 在用户分支是已跟踪、在避障分支是未跟踪 → checkout 报 "untracked files would be overwritten"。两分支内容一致时直接 **`git checkout -f`** 强制切（安全，因内容相同）
   - **切回后**：`git stash apply stash@{N}` 逐个恢复未提交修改（apply 不删 stash，双保险）；**注意 apply 会把用户分支的修改也带过来**（如 super_to_px4_bridge.yaml），用 `git checkout -- <用户分支的文件>` 丢弃（它们已在 feat/4uav-guide 提交，安全）
   - 验证恢复：`grep cloud_topic config/mid360.yaml` 应仍是 `/cloud_registered`；`wc -l src/obstacle_to_px4.cpp` 应 345 行
   - **NX_code 是嵌套子模块有独立分支/stash**：改 User_define_commands.yaml 要在 `cd scripts/NX_code && git branch --show-current` 确认在 feat/4uav-guide 再改（避障分支指向 fceb9df，用户按钮3 配置在 NX_code 的 feat/4uav-guide）——**按钮3 新配置生效前必须确认子模块指针指向含该配置的分支**

22. **QGC 按钮 command 里 `nohup ... & && ...` 卡死超时**（2026-08-11 实测，现象：按 UserCmd3 无响应、RViz 不弹、日志 `❌ 指令执行出错: timed out`）：`cmd & && cmd2` 是非法/危险写法——`&` 后台化后 `&&` 期望前台命令返回，但整个管道被 nx_command_server 的 subprocess 等住 → timed out → 命令被终止，**什么都没启动**。**修复：后台命令必须用子括号包裹** `(nohup ... &)`，这样 `&` 只作用于子 shell，外部命令立即成功返回、`&&` 链正常。正确按钮3 写法：
   ```
   source ... && mkdir -p ... && (nohup rosrun task_run_scripts global_path_publisher.py _file:=.../guidepoints_new.txt > .../global_path_publisher.log 2>&1 &) && sleep 1 && (nohup roslaunch task_run_scripts real_flight.launch ... > .../real_flight.log 2>&1 &)
   ```
   判读顺序：①`tail nx_command_server2.log` 看有没有 `❌ 指令执行出错: timed out`（=command 语法/等待问题）②`ls log/global_path_publisher.log` 与 `real_flight.log` 时间戳（没生成=命令没执行成功）③注意 real_flight.log 可能是**旧文件**（时间戳是昨天的），别被 tail 误导以为启动过。改完 yaml 必须重启 nx_command_server 才生效（每次点击重读 yaml 是旧版行为；改 command 本身后进程仍持有旧配置直到重启）

23. **pgrep -f 误匹配 plink -m 脚本自身 → 误判"进程全停/全在跑"**（2026-08-11 实测，现象：`pgrep -f livox_ros_driver2` 等 4 个全返回同一 PID 5225，且 `ps -p 5225` 查无此进程——那个 PID 是执行脚本的 bash 自己）：`-m` 脚本整段进 argv，脚本里出现的关键词（如 "fastlivo_mapping"、"mavros_node"）被 pgrep 匹配到脚本命令行本身。**判读纪律：pgrep/pgrep 的结果必须用 `ps -eo pid,lstart,args` 或 `ps -p <pid>` 二次确认；多进程返回同一 PID 几乎必是误匹配**。误判后果：以为传感器在跑（实际全停）→ 按按钮3 后 SUPER 无地图。正确检查：`ps aux | grep -E "fastlivo_mapping|mavros_node" | grep -v grep`（排除 grep 自身，但脚本 argv 含关键词时仍可能混入，最终以 `ps -p` 验证为准）
24. **全局路径发布器不能抽稀**（2026-08-11 实测，现象：RViz 无绿线、FSM 日志 `GLOBAL PATH received: 7 pts` 而源文件 705 行；用户纠正"我传输的是全局路径"）：global_path_publisher.py 第一版默认 `min_pt_dist=0.3` 做了抽稀。**全局路径 ≠ 航点任务**：抽稀（waypoint_downsample_dist）是为了减少 waypoint_mission 逐点到达的停顿；全局路径是给 SUPER guide_path 用的，**必须全量保真**（点越密轨迹细节越完整，SUPER 内部滚动取段+走廊生成会处理密度）。修复：min_pt_dist 默认 `0.0`（0=不抽稀全发）。验证：`rostopic echo /global_path -n 1 --noarr | grep length:` 应为几百点。这是用户直接纠正信号的典型——别把航点抽稀逻辑套到全局路径上。**排查顺序（"RViz 没看到绿线"时）**：①先 `rostopic echo /global_path -n 1 --noarr | grep -E 'length|frame_id'` 确认话题有几百点数据（链路是否通）②再 `grep 'GLOBAL PATH received' real_flight.log` 确认 FSM 收到 ③最后才怀疑 RViz 显示项（Path 显示与 mkr 同理：RViz 启动早于话题出现时可能停在 Unsubscribed，取消勾选再勾上触发重订阅；或确认加载的 rviz 配置是 feat/4uav-guide 分支的 top_down.rviz——避障分支的 top_down.rviz 没有 GlobalPath 项）
25. **RViz 的 `rviz/Path` 显示项会静默不订阅话题**（2026-08-11 实测，现象：`rostopic info /global_path` 的 Subscribers 里**只有 fsm_node、没有 RViz**，即使 top_down.rviz 里 Path 显示项 Enabled:true + Topic 正确 + RViz 重启加载过新配置也无效；Color 逗号/分号格式都试过）：Path 显示项在此环境不可靠，**改用 MarkerArray 显示必然可行**（Waypoints 的 mkr 就是 MarkerArray，已验证可靠）。改造：①发布器**额外发** `/global_path_marker`（`visualization_msgs/MarkerArray`，1 个 `LINE_STRIP` marker，ns=global_path, scale.x=0.08, 绿色 a=1.0，points 逐个 append，latch+周期重发）；②top_down.rviz 加 `Class: rviz/MarkerArray, Topic: /global_path_marker, Namespaces: {global_path: true}` 显示项（格式抄 Waypoints 项）。**判读纪律：`rostopic info <话题>` 看 Subscribers 是判断 RViz 是否真的订阅的铁证**（比肉眼看画面可靠——RViz 显示项没订阅时话题 Subscribers 里就没有它）。另注意发布器脚本内 `geometry_msgs.msg.Point()` 会 NameError（已 import 单个类时直接 `Point()`，勿用模块全名）
26. **MAVROS 挂掉 → PX4 自动回退 HOLD 是安全机制，不是故障**（2026-08-11 实测）：用户问"为什么老是切换 HOLD"，查证发现：①当前模式其实稳定 `AUTO.LOITER`（地面待机正常，landed_state=1；用户看到的"HOLD"多是遥控器/QGC 界面对 LOITER 的翻译或模式开关档位），②真正切 HOLD 的场景=**MAVROS 进程死了 → 没有 OFFBOARD 指令流 → PX4 判定指令丢失自动安全悬停**（防止失控乱飞）。排查顺序：先 `ps aux | grep mavros_node | grep -v grep`（进程在不在）→ `rostopic echo /mavros/state -n 1`（connected/mode）→ **连续采样 5 次看 mode 是否跳变**（稳定=正常待机；跳变=有人/程序在反复 set_mode）→ 检查 bridge/fsm 是否在跑（它们才是 OFFBOARD 指令源）。MAVROS 挂了就 `start_fastlivo2.sh false` 重启全套，**飞行中切 HOLD = 先落地再排查，别在空中反复切模式**
27. **切 POSCTL 报 `navigation error: No valid global position estimate` 自动回 HOLD（2026-08-11 实机，MAVROS 正常但定位报错）**：这是**无 GPS 室内场景**的经典组合。完整排查链（按序走，别跳）：
    - **① 确认 EKF 状态**：`rostopic echo /mavros/estimator_status -n 1` → 看 `pos_horiz_abs_status_flag`（无绝对水平位置=无 GPS）与 `const_pos_mode_status_flag`（True=EKF 假定"飞机没动"，无外部位置源的兜底）。`accel_error_status_flag: False` + IMU z≈9.8 = IMU 健康，排除传感器故障
    - **② 别把时间浪费在 EKF2_AID_MASK 上（本会话教训）**：MAVROS 日志报 `PR: Unknown parameter to get: EKF2_NOAID_MASK` → **固件 1.17.0 参数体系已换代**，旧版 `EKF2_AID_MASK` 被拆成 `EKF2_EV_CTRL`（外部视觉融合，实测=11 已开启）、`EKF2_GPS_CTRL` 等。AID_MASK 读出来 0 只是参数不存在/废弃的默认值——**别按标准 PX4 文档去 set AID_MASK（param/set 也大概率 success:False）**。QGC 里搜不到该参数 = 固件没有，用 QGC 搜参数比 MAVROS param/get 权威
    - **③ vision_pose 数据流全链路验证（逐环）**：FAST-LIVO2 源码 `LIVMapper.cpp:223 advertise /mavros/vision_pose/pose` + `:1399 publish_mavros()`（frame_id **硬编码 camera_init**，代码写死不会变）→ `rostopic info /mavros/vision_pose/pose` 应有 Publisher=/laserMapping + Subscriber=/mavros → **`/mavros/local_position/odom` 数值与 vision_pose 几乎一致（如 -0.0268 vs -0.0274）= 飞控 local position 实际在跟视觉**，EKF 不是完全不用视觉
    - **④ "No valid global position estimate" 精确含义**：PX4 区分 **local position**（相对起飞点，视觉/IMU 能给=有）与 **global position**（经纬度，**必须 GPS**=无）——报错指缺 **GLOBAL** 位置，不是缺 local
    - **⑤ ⭐ 模式选择结论（可能就是"昨天好今天坏"真相）**：`POSCTL`（手动位置模式）**只需 local position**（有视觉就够）；**`AUTO.POSCTL`/`AUTO.LOITER`（AUTO 系列任务模式）要求 global position（GPS）**→ 无 GPS 切 AUTO.* 必报错回 HOLD。先试 `rosservice call /mavros/set_mode "{custom_mode: 'POSCTL'}"`（不带 AUTO）；若遥控器模式开关映射到 AUTO 系列（日志显示 `AUTO.LOITER`），在遥控器/QGC 切手动 POSCTL 档。**用户"之前定点正常"=之前切的手动模式或 local 定位够用；SUPER 复飞走 OFFBOARD（也只需 local）**，不受影响
    - **别无限排查**：坐标系（map→camera_init/odom 静态 TF 均存在=通）、IMU（z≈9.8=健康）、launch/yaml（md5 与 git 版一致=没变）都验证过没问题后，收敛到 PX4 侧模式/参数，引导用户在 QGC 看定位状态图标（右上角卫星灰=无定位）
    - **⭐ 终极根因（.ulg 日志铁证，本会话最终结论）：`CP_DIST` 避障参数开启但数据源没跑 → POSCTL 锁死**。用 ulg_report.py / pyulog 解析 .ulg 才发现真相：`estimator_status_flags` 里 `cs_ev_pos=1`（视觉位置**在融合**！）、`estimator_aid_src_ev_pos: fused=1` 全程——**之前基于 /mavros/estimator_status 的 const_pos 判断是错的（该话题 flag 布局与 .ulg 不同）**。真正锁死原因=参数文件里 `CP_DIST=1`（碰撞预防开启）+ `CP_GO_NO_DATA=0`（无数据方向不可飞）+ obstacle_to_px4 未运行（无 /mavros/obstacle/send 数据）→ PX4 认为全向 1m 内都是障碍 → 位置控制被拒/回退。**修复：`CP_DIST=-1`（负值=关闭避障）或先启动 obstacle_to_px4 再开避障**。完整踩坑文档已写入 `task_run_scripts/scripts/NX_code/obstacle_warning.md`（结论/排查链/参数速查/检查清单）。**教训：PX4 模式锁死问题，别只查 ROS 侧话题/参数，直接导 .ulg 看 `estimator_status_flags` + 查 `CP_*` 避障参数组合**
    - **.ulg 权威判读法**（Windows 本地）：`python <桌面>/ulg_report.py <log.ulg>` 出 HTML 报告；要精确看模式/EKF 用 pyulog 直接读——`vehicle_status.nav_state`（PX4 1.17 枚举：0=MANUAL 1=ALTCTL 2=POSCTL 3=AUTO_MISSION 4=AUTO_LOITER 15=ORBIT，**用户实测 nav_state=15 全程不变=模式切换请求根本没被接受**）、`vehicle_control_mode.flag_control_*`（看哪些环在线：无 position/velocity flag=手动模式，位置控制从未上线）、`estimator_aid_src_ev_pos.fused`（视觉融合）、`estimator_status_flags.cs_ev_pos/cs_constant_pos`。**判据：/mavros/estimator_status 的 status_flag 位布局和 .ulg 的 estimator_status_flags 字段名完全不同，以 .ulg 为准**

28. **⭐ FSM 实际加载的 real_flight.yaml 是 `$(find task_run_scripts)/config/real_flight.yaml`，SUPER 仓库那份不生效**（2026-08-11 实机确认）：launch 里 `config_path` 默认指向 task_run_scripts，路径 A 曾把 raycasting/global_path 参数改在 `SUPER/super_planner/config/real_flight.yaml` → **完全不生效**。排查必做：`ps aux | grep fsm_node | grep -v grep | tr ' ' '
' | grep yaml` 或 `rosparam get /fsm_node/config_path` 确认实际加载哪份，再改那份（改完必须重启 SUPER 才加载）。`grep -A12 'raycasting:'` 看 enable 时同理——先确认看的是被加载的那份
29. **纯全局路径模式（use_waypoint:=false）消除"乱转"**（2026-08-11 实飞发现）：同时跑 waypoint_mission（逐点发 /waypoint_goal）+ 全局路径（guide_path 全程）→ FSM 在"追逐点 goal"与"沿轨迹"间切换 → **飞机乱转**（POS+VEL 轨迹在回字形里反复横跳），最终 `close to goal` + `OptimizationExpTrajInPolytopes for new path failed` + `SUPER command timeout. Switching to position hold`。**修复**：①real_flight.launch 已有 `<arg name="use_waypoint" default="true"/>` → 按钮3 加 `use_waypoint:=false` 禁用 waypoint_mission；②global_path_publisher.py 增加发布**终点**到 `/waypoint_goal`（PoseStamped latch，`goal_msg.pose.position = wps[-1]`）→ FSM 目标=终点(1个) + guide_path=完整轨迹，全程沿轨迹到终点无逐点切换。验证：`grep 'Pub goal' real_flight.log` 应只有一次（终点）
30. **实飞故障链：GeneratePolytopeFromLine failed 无限重试**（2026-08-11 实机）：`Using FULL GLOBAL PATH, idx=62 total=1555 pts` → `GenerateExpTrajectory SUCCESS` → `GeneratePolytopeFromLine failed, force return` → `generateBackupTrajectory return [FAILED]` → `PlanFromRest failed, try replan` 循环 → 飞机在原点附近抖动不前进。**根因：全局路径附近地图有障碍/未知空间（测试时人站在飞机旁=人即障碍）→ 走廊生成失败**。处置：人远离飞机 2m+ / 起飞前雷达原地转一圈清地图 / raycasting 开启（坑17）。安全兜底仍在：规划失败时 FSM 重试、bridge 0.5s 无指令自动 POS_HOLD（飞机原地悬停，不乱飞）——**高度突降（POS+VEL 的 z 0.82→0.38）是危险信号，立即遥控器接管**
31. **实飞日志分析方法**（用户问"怎么分析日志"时的标准答案）：①事件时间线 `grep -aE "Mavros triggered|Pub goal|Get goal|FOLLOW_TRAJ|NONEED|POS_HOLD|timeout|OptimizationExp|SUCCESS" real_flight.log | tail -25`；②飞行轨迹 `grep -a "POS+VEL" | tail -20`（x/y/z/vx/vy/vz 是否沿轨迹）；③对照模板：启动→起飞（Publishing initial setpoints→OFFBOARD engaged→reached altitude）→触发（Mavros triggered→Pub goal→Get goal）→规划（GenerateExpTrajectory SUCCESS→FOLLOW_TRAJ）→飞行（ReplanOnce succeed+POS+VEL 沿轨迹）→停（NONEED/OptimizationExp failed/SUPER command timeout→POS_HOLD）；④停止原因三连判读：`close to goal`=认为到了终点（正常完成或提前）／`OptimizationExpTrajInPolytopes failed`=走廊/MINCO 优化失败／`SUPER command timeout`=bridge 兜底悬停（安全）
32. **短距离测试参数建议**（0.6m 悬停+前进场景）：`max_vel 2.0→0.3`、`max_acc 4.0→1.0`（real_flight.yaml）、`velocity_ff_gain 0.7→0.3`（super_to_px4_bridge.yaml）——0.6m/2.0m/s=0.3 秒飞完，MINCO 短轨迹优化易 NaN；planning_horizon(5m) 不用改（0.6m<5m 一次规划到终点）。另注意起飞高度 init_goal(z=0.6) 与航迹起点高度（如 0.849）不一致 → 起飞后先爬升再前进是正常行为
33. **发布器直接发 /waypoint_goal = 绕过 CH10 门禁，不拨杆也飞**（2026-08-11 实飞确认）：方案 C 前 global_path_publisher 额外发布终点到 /waypoint_goal（PoseStamped latch）→ 用户没拨 CH10 飞机自己开始循迹。**原因：CH10 触发逻辑（start_trigger_type=1 下降沿）在 waypoint_mission 节点内部；任何其他节点直接发 /waypoint_goal，FSM 的 waypointCallback 收到即 setGoalPosiAndYaw 开始规划，门禁形同虚设**。修复：goal 只能由 waypoint_mission 发（它读 endpoint.txt 只含终点，CH10 触发后才发）；发布器只管 /global_path + marker。判读：无触发时日志出现 `Pub goal`/`Get goal` = 门禁被绕过，查谁在发 /waypoint_goal（`rostopic info /waypoint_goal` 看 Publishers）
34. **全局路径模式下终点仍会被 getNearestInfCellNot 吸附**（2026-08-11 实飞）：航迹终点 (0.073, -0.048, 0.608) 被 FSM 吸附成 `Get goal at 0.075 -0.075 0.825`（z 抬高 0.2m+）——`fsm.cpp:191` 对每个 goal 都查 `getNearestInfCellNot(OCCUPIED, p, gi_.goal_p, 3.0)`，终点附近 0.6m 高度有障碍（人/机架/地图残留）就吸附到 3m 内最近空闲格。后果：最后一段轨迹终点被抬高/偏移。**处置顺序**：①起飞前雷达转一圈清地图 + 终点选空旷处（吸附距离内找到终点本身就不偏移）→ ②仍吸附再考虑改代码（给全局路径模式的 goal 加"不吸附"开关，但终点若真在障碍里会导致不可达）。判读：`grep 'Pub goal'`（发布值）vs `grep 'Get goal'`（吸附后值）不一致=吸附发生
35. **"SUPER 知道终点会不会直线飞终点？"**（用户核心疑问，回答模板）：**不会**——注入的 guide_path=完整轨迹 → `SearchPolytopeOnPath` 沿轨迹生成一串凸多面体走廊 → MINCO 只能在走廊内优化（物理约束，类比"沿轨迹修的管道，跳不出去"）。但有两个注意点：①**拐角会切角**（MINCO 平滑优化走"弦"，不严格贴折线，走廊越宽切角越多——特性非 bug）；②**终点可能被吸附抬高**（坑#34）。整体效果=大部分沿轨迹，终点可能偏
36. **raycasting 开启后"大部分障碍块消失、小部分残留"是正常**（2026-08-11 实机，用户报"走一圈有些障碍块不随人离开而消失"）：raycasting 清除依赖**新激光帧重新扫过该体素**（p_miss 作用）——残留块 = 雷达盲区（正上/正下方）/被遮挡（障碍物后面激光穿不过）/扫描线间隙（远距离稀疏处）的位置，**没被重新观测就不会清**，这是射线式概率地图（ROG-Map/OctoMap）的固有特性不是 bug。判读：飞机原地转一圈（雷达扫过所有方向）后残留消失 = 正常；空旷区域转圈后仍残留 = 概率参数问题（调 p_miss↑/p_free↓，但会误清静态障碍，慎调）
37. **走廊生成失败根因之一：全局路径点过密（0.46cm 间距）→ 注入前必须降采样**（2026-08-11 实机，现象：`Downsampled` 之前 `Using FULL GLOBAL PATH` 后 `GeneratePolytopeFromLine failed` 无限循环，`Box search failed, box size is zero` + `overlap only -326`）：SUPER 走廊生成器沿 guide_path **逐段生成凸多面体**，录制轨迹 930 点挤在 4.31m（平均间距 **0.46cm**）→ 每段多面体小到无法生成 → 走廊必失败。**修复：注入块取段后降采样**（`global_path_downsample_` 默认 0.1m：保留每 ≥0.1m 一个点 + 强制保留最后一点终点），配置链：`super_planner.h` 加成员 + `setGlobalPath(path, roll_dist, downsample)` 第三参 → `fsm_ros1.hpp` 读 `nh_.param("global_path/downsample", 0.1)` → `real_flight.yaml` `global_path: {roll_dist: 5.0, downsample: 0.1}`。验证：日志 `Downsampled global path: 39 pts (ds=0.1m)`（930→39 点）。**检查航迹点密度**：python3 读 CSV 算相邻点距离，总路程/段数=平均间距，>5cm 基本安全，<1cm 必须降采样
38. **`pub_unknown_map_en: false` 默认关 → 未知区域不可见 → 走廊失败误判**（2026-08-11 实机）：ROG-Map 可视化配置里 `rog_map/visualization/pub_unknown_map_en: false`（默认）→ `/fsm_node/rog_map/unk`、`/inf_unk` **不发数据**（`rostopic hz` 报 no new messages），RViz 里看不到"未知雾"→ 误以为路径附近干净。打开后 unk 话题 2Hz 有 12 万点（飞机 1m 内 443 个未知点）——**未知区域才是走廊扩不出盒子的主因**（走廊要求"已知空闲"空间，未知=扩不出来）。诊断走廊失败顺序：①`rostopic hz /fsm_node/rog_map/unk` 看是否发数据（不发=pub_unknown_map_en 关）→ ②打开它（改 `pub_unknown_map_en: true` + 重启 SUPER）→ ③解析 unk 点云看路径方向未知占比。top_down.rviz 加 Unknown 显示项（`Class: rviz/PointCloud2, Topic: /fsm_node/rog_map/unk, Color: 255;128;0 橙色, Style: Points`）可肉眼观察
39. **`corridor_bound_dis` 语义：搜索盒范围，不是安全距离——调小不增加碰撞风险**（2026-08-11 源码确认）：`getSeedBBox`（corridor_generator.cpp:198）里 `box_min -= Vec3f(bound_dis_); box_max += Vec3f(bound_dis_)` —— 它决定**沿路径每个点向外扩展多大的"搜索盒"**（在这个盒里 boxSearch 找障碍、IRIS 扩空腔）。**真实碰撞安全由 `robot_r`（0.25m）+ IRIS 内缩保证**（robot_r 只影响 z 方向 `box_min.z() += robot_r_; box_max.z() -= robot_r_`，水平方向不受约束）。调小 bound_dis=搜索盒小→盒内障碍少→IRIS 更容易扩出有效盒子→**更保守不危险**（副作用：盒子小→MINCO 轨迹更贴路径更"硬"）；调大=可能扩出更大盒子但更易失败
40. **RViz 压缩格式（flow-style）编辑必须字符串插入，不能 yaml.dump 重写**（2026-08-11 实机，之前"标准格式重写"翻车：RViz 显示区域变小 + SUPER 界面丢失）：top_down.rviz 是 RViz 保存的**压缩格式**（`Panels:  - Class: rviz/Displays    Help Height: 172` 同行 flow-style）——yaml.dump 会把它重排成 block 格式，RViz 加载后显示项丢失/异常。**正确做法：读原文本 → 字符串 replace 插入新显示项块（缩进与相邻项一致）→ python3 yaml.safe_load 验证仍可解析 → 重启 SUPER 生效**。本会话用此法成功插入 Unknown 显示项（在 Occ 块后，`Name: Unknown / Topic: /fsm_node/rog_map/unk / Color: 255; 128; 0`）。教训：改任何 .rviz 配置先 head -5 看格式，压缩格式一律字符串插入
42. **odom_to_mavros（odin 视觉桥）双坑：多实例 + vision_pose 超频**（2026-08-14 odin 传感器实测）：①按钮一次点击产生 4 个实例（bash 命令链多拉）→ vision_pose 6570Hz 疯狂打架 → EKF 判视觉不可信 → NOT READY。修复：脚本加 **pid 文件锁**（`/tmp/odom_to_mavros.lock` 存 PID，启动时查存在且进程活则退出；不能用 ROS 节点名查重——anonymous=True 节点名带 PID 后缀且启动时序查不到）。②odin odometry 400Hz 全量转发 → vision_pose 500-1000Hz → PX4 EKF 拒收/过载 → not ready。修复：**发布限速 20Hz**（回调里 `if (now-last_pub < 0.05) return`）。判读：`ps aux | grep 'python3.*odom_to_mavros' | grep -v grep | wc -l` 应=1（**grep 会误计 bash 包装进程**，必须用 `python3.*` 精确匹配）；`rostopic hz /mavros/vision_pose/pose` 应 ~20Hz。⚠️ 注意 odin 消息时间戳是设备时钟（与主机差几百秒），**桥接/转发节点查 TF 必须用 `rospy.Time(0)` 取最新 TF，不能用 msg.header.stamp**（否则 `Lookup would require extrapolation` 静默失败）。⚠️ **双副本限速不一致（2026-08-24 实机检查）**：`NX_code/odom_to_mavros.py`（Radar_Launcher.yaml 实际启动的）= **20Hz** ✅，`task_run_scripts/scripts/odom_to_mavros.py`（冗余副本，不在启动路径）= **50Hz**（ccbb463 提交版）——**改限速/单实例锁必须改 NX_code 那份**，改 task_run_scripts 副本=改了不生效（坑：改错副本）；启动路径确认法：`grep -n odom_to_mavros Radar_Launcher.yaml`
43. **⭐ virtual_ground_height 量化 bug：-0.9 → 实际 +0.15 → 地面测试 below ground 无地图**（2026-08-14 实锤，现象：`Odom below virtual ground` 刷屏 + inf_occ 无数据，飞机拿起(z>0.15)就正常）：prob_map.cpp 初始化 `posToGlobalIndex(cfg_.virtual_ground_height, ...)` 后 `cfg_.virtual_ground_height = id * resolution` 回写——**-0.9 量化后变 +0.15**（配置 -3.0 → -1.95 正常）。地面飞机 z≈0 < 0.15 → 地图拒绝更新。**修复：`virtual_ground_height: -3.0`**（量化后 -1.95，地面 z=0 在地图内）。⚠️ ceil 同理：`virtual_ceil_height: 3` 会被**地图尺寸钳制**（map_size z=6 时 → 1.95），点云 z>1.95 的高处全被 getGridType 判"越界=OCCUPIED"→ 视野内全是障碍。**判读顺序：先 `grep -aE '\[ProbMap\] virtual' real_flight.log` 看量化后实际值（不是 yaml 配置值），再谈地图问题**
44. **⭐ "全屏障碍"排查铁律：一次只改一个参数 + 用 git 备份二分**（2026-08-14 血泪教训，全屏障碍从出现到定位耗时 2 小时+）：连锁改 fix_map_origin/map_size/ceil/ground/cloud_topic/omni 后全屏障碍，逐个回退才发现**真凶= `fix_map_origin [0,0,1.5] → [0,0,0]`**（坐标系抬高 1.5m → 地面点变"空中障碍"散布全屏；地面过滤 z<0.1 在地图系里完全错位失效）。排查方法：`ls -la config/real_flight.yaml.bak*` 按时间列备份 → 对比每个 .bak 的关键参数（cloud_topic/fix_map_origin/virtual_ground/inflation_step/p_miss/map_size）→ **cp 回"全屏障碍出现前"的 .bak 一刀切恢复**（本会话恢复 .bak_odintopic 后正常）。⚠️ 注意 .bak_odintopic 是 15:06 的备份，cloud_topic 还是 /cloud_registered（MID360 遗留），odin 需改回 /odin1/cloud_slam。**教训：改完配置必须确认 FSM 重启加载**（`ps aux | grep fsm_node | grep -v grep | awk '{print $9}'` 启动时间 vs 文件 mtime；日志 `Load param` 行是铁证——曾发生"配置改了 FSM 没重启还在用旧参数"白折腾一轮）
45. **⭐ 动态障碍残留根因=雷达贴地几何遮挡（不是代码 bug）**（2026-08-14 用户关键观察："把飞机拿起来就没有残留了"）：地面测试时雷达离地 0.1-0.3m → 激光从低处扫 → 障碍背后的区域被遮挡、无光线 → raycasting 清不到 → 残留；飞机拿起（雷达 1m+）→ 俯视覆盖全 → 残留消失。**结论：残留严重程度与雷达高度强相关，飞行作业（0.6m+）时残留本来就少；地面测试想接近飞行状态就垫高雷达**。配合全向光线（坑46）可进一步缓解
46. **全向光线 omni（主动清除残留，2026-08-14 实施）**：ROG-Map 原生 raycasting 是"被动清除"（只清激光扫过的方向，空旷方向旧障碍残留）→ 在 prob_map.cpp `raycastProcess` 点云光线循环后加**全向稀疏探测光线**（水平 step_h×垂直 step_v×fov_v±60°×range 8m，遇"当前帧 hit_cnt>0 的格子"停止=不清真实障碍，其余全部 free）→ 空旷方向旧障碍自动消失（不用飞机转过去扫）。配置（yaml `raycasting` 段）：`omni_en: true, omni_range: 8.0, omni_step_h: 5.0, omni_step_v: 7.5, omni_fov_v: 60.0`（**step_h 5 加密后残留明显改善，CPU 45% 有余量**；10° 时 8m 处光线间距 1.4m 会漏细障碍）。代码坑：`hit_cnt` 是 `std::vector<uint16_t>` 不是 map，用 `hash_id < size() && hit_cnt[hash_id] > 0` 索引访问（find() 编译报错）。⚠️ 曾误判"omni 误清地面→被包围"回退，实为 fix_map_origin 改坏的坐标系（坑44）——**omni 本身安全，恢复基线后重启用效果良好**
47. **⭐ 停车避让状态机 AVOID_WAIT（动态障碍不卡死）**（2026-08-14 实施）：原本 GENERATE_TRAJ 失败→留在原状态高频空转重试→bridge 0.5s 超时才被动悬停→视觉"卡死"。改 3 文件：①`fsm.h` 加 `AVOID_WAIT` 枚举+字符串 + `avoid_wait_t_` 成员 + `virtual void publishHoldPose() = 0`；②`fsm.cpp` GENERATE_TRAJ 失败分支改为 `publishHoldPose()` + `ChangeState(AVOID_WAIT)`；AVOID_WAIT case 每循环 `publishHoldPose()`（主动悬停防漂移）+ `avoid_wait_t_ += 0.1`，≥0.3s 重试一次 PlanFromRest，成功→恢复 FOLLOW_TRAJ（日志 `AVOID_WAIT: path recovered, resume flight.`）；③`fsm_ros1.hpp` 实现 publishHoldPose（发 PositionCommand 到 `/planning/pos_cmd`：position=当前位置、velocity=0、acceleration=0，`cmd_pub` 无下划线！）。实测：人挡住→停车→人走开→0.06s 内恢复飞行 ✅。⚠️ 障碍持续存在时"恢复→又挡→又停"循环是物理限制（人等走才能继续），不是 bug
48. **"多次穿行后卡死"= 终点判定 0.1m 嫌疑**（2026-08-14 待验证）：多次穿行后日志 `FOLLOW_TRAJ + Using FULL GLOBAL PATH ×258（轨迹一直在生成）` 但飞机不动 → 强嫌疑=`super_planner.cpp:337` 终点判定 `(robot_state_.p - gi_.goal_p).norm() < 0.1 → traj_finish=true → 悬停`——多次穿行时若 goal 位置或飞机位置异常接近某点 → 误判到达永久悬停。**验证方法：注释掉 0.1m 终点判定重新编译测试，或每次 traj_finish 打日志（位置/目标距离）**。**⚠️ 已实施验证（fsm_node 16:03 终点判定注释版）：实测仍卡死 → 终点判定已排除！真凶 = fsm_node 崩溃（坑 52，corridor_generator.cpp:113 未捕获异常）**。另注意 `Using FULL GLOBAL PATH idx=0` 不是 bug（飞机在起点时最近点确实=0；global_last_idx_ 单向推进只在 fallback 时不清除）。**✅ 最终：终点判定已恢复（fsm_node 16:34，0.1m 悬停功能保留；崩溃修复后不再误触发卡死）**
49. **⭐ 绕行回正俯冲修复 = bridge 加速度前馈低通（acc_ff_smooth）**（2026-08-14 实机，现象：1m/s 绕行→回正切换时 z 冲高 0.68→1.02→回落 0.84 俯冲）：MINCO 轨迹在绕行→回正切换点**加速度突变** → bridge 原样透传加速度前馈（setpoint_raw.acceleration_or_force）→ PX4 高度骤变。**修复（双保险）**：①控制层 `super_to_px4_bridge.cpp` 加速度前馈加一阶低通 `acc_ff = α×新 + (1-α)×旧`（α=acc_ff_smooth，0.3 太钝→**0.5 平衡**）；②轨迹层 real_flight.yaml 主轨迹 `penna_jerk -5e5 → -7.5e5`（MINCO 更平滑；-1e6 太平滑拖慢，折中；backup -1e5→-5e5）。**原理：俯冲=加速度突变，治本走轨迹 jerk、治标走控制低通，两个都做效果最好**。参数经验（1m/s 时代）：max_vel 1.0 配 max_acc 0.6 够用（加速距离 0.85m）；max_omg 2.5→6.0 + max_acc_thr 20→25 治绕行 "Opt failed, Omg or thr or Pos violation"（MINCO 失败=物理约束太紧，先放宽约束而非降速度）
50. **FAPP 动态避障算法（arXiv 2312.08743，港科大 Lu Peng 团队）+ 融合可行性**（2026-08-14 调研）：动态杂乱环境 UAV 避障系统，4 模块——①**点云分割静态/动态** ②自适应运动估计（协方差适应，预测多动态物）③高效轨迹优化 ④自适应重规划（轨迹无解兜底=已实现为坑47 AVOID_WAIT）。**与当前项目融合的最大价值=①动态点云分割**：当前帧(cloud_raw) vs 静态累积图(cloud_slam) 差分 → 动态物单独剔除、不进静态地图 → 残留/误判/卡死全解（FAPP 思想在 Python 桥接 cloud_raw_lidar_to_odom.py 即可实现，不碰 C++）。**ESDF 不建议开**：从占用图派生（残留同源，对残留无帮助）+ 3D 全图更新 CPU 45%→80%+ 风险（Orin NX 8GB 边缘）。**刷新频率评估**：cloud_raw_odom 6.5Hz 地图 + 10Hz replan → 检测延迟 0.2-0.3s = 1m/s 下 0.3m 提前量，对慢走的人勉强够、快跑的人不够；瓶颈是桥接降采样耗时（49152→13000 点），优化 numpy 或增大降采样体素可到 10Hz。**调研方法：Bing 网页/RSS 被中文广告污染（返回 139 邮箱/知乎内容），学术算法直接用 arXiv API**：`http://export.arxiv.org/api/query?search_query=all:%22FAPP%22+AND+all:obstacle&max_results=5`（Atom XML，PowerShell `[xml]$x=$r.Content` 解析 entry 的 title/summary）
51. **⭐ FAPP 阶段1 实施：动态点云分割（cloud_dynamic_filter.py，时间一致性滑动窗口）**（2026-08-14 晚实施）：数据验证确认 cloud_slam 基本纯静态（人走进差分 462 体素、人走后残留仅 1.5%）→ 方案A（当前帧 vs 静态背景）可行。脚本：订阅 cloud_raw → TF(Time(0)) 到 odom → 地面过滤(z<0.1) → 体素化(VOX=0.2) → 查背景 → 输出 `/odin1/cloud_static`(进地图) + `/odin1/cloud_dynamic`(可视化)。脚本在 `task_run_scripts/scripts/cloud_dynamic_filter.py`；接入地图=real_flight.yaml `cloud_topic: /odin1/cloud_static`
    - **v1 naive 差分（当前帧 vs cloud_slam 体素集）过度分割**：dynamic 7537 点 86% 散布——cloud_slam 只含"被扫过多次"的静态点，单次扫到的边角/远处静态点全被误分动态
    - **v2 背景累积(A1) 失败——鸡生蛋缺陷**：只把"被判 static 的点"并入背景，但边角静态点永远被判 dynamic → 永远不累积 → 84% 误分依旧。背景更新必须让**所有当前帧点**参与（带遗忘），不能只累积 static 结果
    - **⭐ v3 时间一致性滑动窗口（正解）**：维护体素计数表 `coord → (count, last_time)`，每帧所有点 count++（上限 20），`FORGET_T` 未命中即遗忘；`count >= CONFIRM_N → static，< → dynamic`；动态候选再过密度聚类（scipy cKDTree，0.35m 内 ≥3 点保留，人=成块/噪声=散点）。基线正常：static 23295 / dynamic 8
    - **⭐ CONFIRM_N 参数敏感性（核心调优）**：5（0.5s）→ 人站 0.5s 就转 static 进地图（残留问题回归）；20（2s）→ 仍短；**60（6s）+ FORGET_T 8s → 穿行场景合理**：穿行的人停留 <6s 全程 dynamic 不进地图，只有站 6s+ 的静止物转 static（等同静态障碍，合理）。⚠️ FORGET_T 必须 > CONFIRM 时长，否则 count 中途被遗忘永远到不了阈值
    - 判读坑：分割结果的"人形判断"要用**变换后 odom 系坐标**（飞机不在原点时 lidar 系范围会误判）；人确实在 cloud_raw 但 static/dynamic 都没有 → 检查过滤链（CONFIRM 过短已转 static / 坐标系判断错）
    - **⭐ 最终结论：纯时间一致性分割对 odin dToF 点云不可靠 → FAPP 阶段1 暂停**（2026-08-14 晚实测）：dToF 点云每帧有约 2 万点**间歇性闪烁/抖动**（同一个物理点帧间在相邻体素间跳动）→ CONFIRM_N=60 时静态物（墙）体素 count 也到不了阈值 → 大量静态点被误判 dynamic（实测 static 150 / dynamic 23318 反转）；VOX 0.2→0.4 部分改善（static 20540）但闪烁点仍 ~2 万误分。**根本矛盾：CONFIRM_N 小→人站 0.5s 转 static（残留回归）；CONFIRM_N 大→静态物也到不了（闪烁）**。要区分"站住的人"和"静止的箱子"必须做**运动检测**（体素位置随时间变化/速度一致性，FAPP 论文方法），纯计数不行。**处置：暂停动态分割，地图继续用 cloud_slam + 全向光线（已实测残留改善 90%），优先排查"多次穿行卡死"根因（坑48 终点判定嫌疑）**。如重启 FAPP 需：①先用运动检测思路（每体素记录最近位置序列，位置漂移=动态）②或依赖 odin SDK 自带的 SLAM 动态过滤能力
    - 详细数据见 references/odin-super-dynamic-avoidance.md 第11节
52. **⭐ "多次穿行后卡死"真凶 = fsm_node 崩溃（corridor_generator.cpp:113 未捕获异常），终点判定只是嫌疑已被排除**（2026-08-14 实锤，坑48 注释终点判定后实测仍卡死 → 卡死时现场抓堆栈）：现象=FOLLOW_TRAJ + Using FULL GLOBAL 一直刷（轨迹在生成）但飞机不动。**现场抓取（关键！）**：让用户"卡死时保持系统运行别关"，立即查 ①`ps aux | grep fsm_node`（进程没了=崩溃）②`rostopic hz /planning/pos_cmd`（no new messages=没人发指令）③`tail -60 real_flight.log`——**日志尾部出现 `Source ".../xxx.cpp", line N` + `#N` 堆栈帧 = 崩溃堆栈铁证**（进程已死，不是普通 WARN）。**崩溃链**：fsm.cpp:77 `callReplanOnce` → super_planner.cpp:191 ReplanOnce → generateExpTraj → :845 `SearchPolytopeOnPath` → **corridor_generator.cpp:113 `throw std::runtime_error("seed line too long")`**（A* 绕行路径相邻两点距离 > seed_line_max_length_×1.5=1.8m，异常未被 try-catch → terminate → 进程死）→ 无 pos_cmd → bridge 超时悬停 = 视觉"卡死"。**修复（已实施, fsm_node 16:16）**：①**长段插值**——SearchPolytopeOnPath 生成 seed_lines 时相邻点距离 > seed_line_max_length_ 则 `n_split=ceil(len/max)` 中间插点（每段 ≤max，走廊正常生成，绕行可用）；②移除 throw（原崩溃点）。**"防崩溃只是不报错"的教训**：throw→return false 只保命（失败→AVOID_WAIT 等待恢复），要真正绕行必须插值治本（A* 长段 → 走廊永远失败 → 反复等）。**排查纪律**：此前多轮"卡死"排查都因事后查日志（进程已退、状态丢失）白折腾——**卡死类问题必须当场抓现场**，先确认进程在不在，再看日志尾部是否崩溃堆栈。配套：⚠️ **不要调 p_miss 0.49→0.6**（曾误调导致全屏障碍，p_miss 是"清除目标概率"必须 < p_free 0.4999，见坑53）——残留加速用加密全向光线（坑46）

53. **⭐ p_miss 是"清除目标概率"不是"清除强度"——必须 < p_free(0.4999)，调大=清除失效=全屏障碍**（2026-08-14 实锤，曾误调 0.49→0.6 全屏障碍块）：ROG-Map 概率更新：命中点 → 概率向 p_hit(0.70) 更新；光线穿过 → 概率向 **p_miss** 更新（不是"衰减强度"！）。判定阈值：p_free=0.4999（概率 < 则自由）、p_occ=0.85（> 则障碍）。**p_miss=0.49(<0.4999) → 光线多次穿过概率降到自由 ✅ 清除正常；p_miss=0.6(>0.4999) → 概率停在 0.6 永远不自由 → 之前清掉的格子全变回"非自由" → 全屏障碍块**。想加快清除：调**小** p_miss（如 0.3-0.4，但会误清障碍边缘，慎调）或**加密全向光线**（坑46 step_h 10→5），不是调大。铁律：**改概率参数前先确认它落在判定阈值（p_free/p_occ）的哪一侧**；怀疑清除问题时先 `grep -A12 'raycasting:'` 看 enable + p_miss/p_free 相对关系（坑17）

54. **绿线（/global_path_marker）断开又恢复 = fallback 绕行的正常表现，不是 bug**（2026-08-14，用户问"绿色轨迹为什么断开又完整出现"）：绿线随 guide_path 走——人挡路 → fallback 分支 `guide_path.clear()` → 绿线消失/变绕行线（A* 路径断续）；人走开 → 恢复 `Using FULL GLOBAL PATH` → guide_path=原始航迹 → 绿线完整出现。**断开=正在绕行、恢复=回到循迹，是链路正常工作的可视化信号**（配合坑48 的"Using FULL idx=0 不是 bug"）。判读：绿线断开期间日志应有 `Global path blocked by obstacle, fallback to A* replan`；人走后 `Using FULL GLOBAL PATH` 恢复 = 一切正常

55. **推送纪律：`git add -A` 会把 .bak 备份文件提交进仓库**（2026-08-14 两次发生：`.bak_pmiss06`/`.bak_confirm`/`.bak_seedfix` 混入提交，次轮又专门清理）：调试期每改一处就 `cp xxx.bak_<tag>`，一天累积 46 个 .bak，`git add -A` 全提交 → 仓库被备份文件污染。**提交前先清**：`find ~/catkin_ws/src/{SUPER,task_run_scripts} -name '*.bak*' -type f -delete` 再 `git add -A`（.bak 只保留 1 份"已知正常"（如 .bak_odintopic 恢复法，坑44），调试中间备份随手清）。提交后用 `git status --short` 确认只剩真实改动（M/A/D），避免把实验脚本（cloud_dynamic_filter.py 等）误当正式代码推——实验性脚本提交时在 commit message 标注"实验/待验证"

56. **⭐ 撞墙根因：frontend_in_known_free=false → 未知区域当自由 → 雷达范围外的墙扫不到=未知 → A* 直穿撞墙**（2026-08-14 实机严重事故，用户"没有全局地图+雷达范围小，没扫描到周围墙壁，直接撞到墙"）：
   - **雷达实际探测范围仅 ~4.5m**（cloud_slam 点云 x 边界 3.8~4.3）——墙在 5m+ 扫不到 → 地图"未知"
   - super_planner.cpp:1330 `int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE)` —— **`frontend_in_known_free: false`（默认）→ A* 把未知当自由（UNKNOWN_AS_FREE）→ 直飞进未知区撞墙**
   - **修复：`frontend_in_known_free: true`**（A* 只搜"雷达确认自由"的区域；前方未知 → 无路可走 → 停车不撞——宁停勿撞，这是最高优先级的安全兜底）
   - 代价：雷达范围外不能自主飞（需先扫到区域）；测试时飞机在未知边缘停下 = 安全行为，不是故障
   - **⚠️ 副作用：打开后"轨迹乱跳"**（2026-08-14 实测，现象：绿线/轨迹每轮 replan 变化剧烈、日志 `SearchPolytopeOnPath for new path failed ×96`、用户观感"A* 一直在规划尝试"）：A* 只在"已知自由区"搜，而 dToF 点云每帧闪烁 → **"已知/未知边界"每帧抖动** → A* 结果每帧不同（某帧找到路径、下帧失败截断、再下帧绕行）→ 轨迹反复切换。静态测试（障碍固定+飞机不动）时边界稳定、乱跳不明显；人穿行/飞机移动时加剧。**要根治需稳定边界=全局地图**（墙预先标进地图，不依赖实时扫描）；无全局地图时 frontend=true 只适合"静止障碍短测"，常规飞行保持 false 等全局地图
   - 判读：撞墙/飞进未知区时查 `grep frontend_in_known_free config/real_flight.yaml`；点云范围用 python 读 cloud_slam 的 x/y/z 边界确认雷达实际感知半径

57. **防摩擦（碰撞）参数组合：min dis 0.1 → 0.6m**（2026-08-14 用户报"和障碍物发生了摩擦"）：日志 `[CIRI] min dis to obstacle is only: 0.1` = MINCO 轨迹离障碍仅 0.1m → 飞机旋翼半径 ~0.2m → 摩擦。修复（real_flight.yaml 两处）：`inflation_step: 2→3`（0.4→0.6m，A* 路径离障碍更远）+ `corridor_bound_dis: 0.39→0.6`（走廊加宽 → MINCO 轨迹约束离障碍更远）。原理：轨迹离障碍 0.6m - 飞机半径 0.2m = 0.4m 安全余量。⚠️ 与坑39 对照：corridor_bound_dis 语义是"搜索盒"，但实测调大后走廊更宽、MINCO 更贴中心，轨迹确实离障碍更远。代价：膨胀加大 → 窄空间绕行可能扩不出走廊（Opt failed），折中 inflation 2.5。**⚠️ 2026-08-14 晚用户决策：防撞墙/防摩擦改动全部回退**（frontend_in_known_free 回 false、inflation_step 回 2、corridor_bound_dis 回 0.39、终点判定回 0.1）——后续**加全局地图约束飞行范围**（把墙/边界放进地图，飞行被限制在已知安全区）来解决撞墙，届时再配 frontend_in_known_free=true + 终点判定 0.3（坑58）。回退法：`cp real_flight.yaml.bak_safe real_flight.yaml`（改 inflation/corridor 前的完整版）+ `cp super_planner.cpp.bak_goal03 super_planner.cpp`（终点判定 0.1 版）→ 重新编译确认二进制含 0.1m 标记（`strings fsm_node | grep -c "dist=%.3fm < 0.1m"`）
   - **绕行方向偏好：A* 总从固定方向先搜（邻居扩展顺序，通常先左）**（2026-08-14 实测，用户"总是从左边找路但有墙"）：frontend=false 时左边墙=未知=可走 → 走左边撞墙；开 frontend=true 后改走右边 → 但膨胀不足（inflation 2=0.4m < 桨 0.3m+余量）→ 贴障碍撞障碍物。**"撞墙(方向/未知) vs 撞障碍物(距离/膨胀) 是两个独立维度**：未知区用 frontend_in_known_free 管，贴障碍用 inflation_step+corridor_bound_dis 管——只修一个另一个还会撞。**对用户解释"航点/路径跳变"**：传入的 /global_path（51 点直线，latch）固定不变；跳变的是 FSM 内部 guide_path（每轮 replan 从"最近点→终点"动态截取 + fallback 换成 A* 绕行路径）——RViz 里看到"线在跳"要先确认显示的是 /global_path 还是 guide_path 可视化

58. **⭐ 终点"1 点残局"乱转：guide_path 只剩 1 点（idx=total-1）**（2026-08-14 晚，现象：航迹末端日志 `Using FULL GLOBAL PATH, idx=50 total=1 pts` + `GeneratePolytopeFromLine failed` 循环 + 飞机原地乱转）：飞机飞到航迹最后一点（51 点航迹 idx=50=终点）但距终点 >0.1m（终点判定 0.1m 未触发）→ guide_path 注入块（super_planner.cpp:683-692 `sub_path=[global_path[nearest_idx]]` 直到终点，nearest_idx=最后一点时只剩 1 点）→ 每轮 PlanFromRest 生成"当前位置→终点"超短轨迹 + 1 点无法生成走廊 → 原地微调循环 = 视觉"乱转"。**与 frontend_in_known_free 无关**（循迹时走 Using FULL 分支不用 A*，该开关只影响 fallback 绕行搜索；但它会让 fallback A* 在未知区搜不到 → 更多规划失败 → 观感更乱）。**修复：终点判定阈值 0.1→0.3m**（super_planner.cpp:337 `dist_goal < 0.3`，到终点 0.3m 内即悬停，不留"1 点残局"）。判读：日志 `idx=50 total=1`（total=1 是铁证）= 飞机已到航迹末端、终点判定没触发 → 放宽阈值或直接判定到达。注意：`idx=0 total=N` 不是问题（飞机在起点时最近点=0，正常）；只有 `total=1` 才是残局。⚠️ 用户最终决定"不更改，后面加全局地图"——终点判定已还原 0.1（fsm_node 18:01），加全局地图时再一并开 0.3

59. **⭐ 系统没有"飞机半径/体积"参数——碰撞安全全靠膨胀层间接实现**（2026-08-14 用户问"是不是可以调一下飞机半径"后查证）：yaml 与代码里都没有 vehicle_radius/body_radius；唯一的体积概念是 corridor_generator 的 `robot_r=0.25m`（坑39），且**只影响 z 方向**（`box_min.z() += robot_r_; box_max.z() -= robot_r_`），水平方向轨迹可贴障碍。这就是"min dis 0.1m 还撞"的结构性原因：MINCO 轨迹是"点"路径，飞机有 ~0.3m 桨半径，膨胀层是唯一水平安全余量来源。**两条路**：①改代码加 `vehicle_radius` 参数（膨胀=半径+余量，A*/走廊/碰撞检查统一用——精细但需改 corridor_generator/astar 多处）；②配置等效（inflation_step=3 即 0.6m 膨胀 ≈ 半径0.3+余量0.3，但室内窄空间会堵死绕行）。**"膨胀防撞 vs 绕行空间"是室内小空间(2.5m 航迹)的硬矛盾，实测数据**：inflation_step=3 → fallback 48 次全失败（SearchPolytopeOnPath failed ×48 + GeneratePolytopeFromLine failed ×135）→ 无绕行轨迹 → 乱转；inflation_step=2 → 绕行空间够（历史 122 次 SUCCESS）但贴障碍（min dis 0.1）。折中 inflation 2.5 或 corridor_bound_dis 加大。**结论：正式方案 = 用户已定的全局地图（约束飞行范围 + 稳定已知/未知边界，配合 frontend_in_known_free=true 防撞墙）+ 届时加 vehicle_radius 参数统一水平安全余量**。排查碰撞问题顺序：①日志 `min dis to obstacle`（轨迹实际离障碍多远）②飞机桨半径（物理尺寸）③膨胀/走廊配置——三者对齐才能保证不碰

60. **⭐ 飞机半径纳入防撞：robot_r（CIRI 走廊）是唯一体积概念 + penna_pos 守走廊**（2026-08-14 用户报桨半径 23cm，要求"把飞机半径考虑在内不要撞障碍物"）：系统无 vehicle_radius 参数（坑59），碰撞安全链=**CIRI 走廊生成（corridor_generator.cpp:34-45 构造传 robot_r → ciri_->setupParams(robot_r)）→ CIRI 检查 seed line 离障碍 < robot_r → 不可行 FAILED → 走廊失败 → 无轨迹**。MINCO 轨迹本身**没有碰撞检查**（只有 backup 有 magnitude 检查），完全靠 vPolytopes 走廊约束保证安全——所以安全距离=走廊边界=robot_r。**调参**：`robot_r: 0.25→0.35`（桨 0.23+余量 0.12）+ 主轨迹 `penna_pos: 5e6→1e7`（MINCO 更严格守走廊不穿透）。⚠️ 注意 robot_r **只影响 z 方向**（`box_min.z() += robot_r_`，坑39），水平方向安全靠膨胀层（inflation_step）——两个维度都要管。判读：日志 `[CIRI] min dis to obstacle is only: X` + `robot_r: 0.35`（打印在 FAILED 后的 cout 里）——X < robot_r 即走廊扩不出（安全拒绝）；X 远大于 robot_r 却还撞 = 膨胀/走廊水平方向没生效（查 inflation_step + corridor_bound_dis）

61. **⭐ bridge 前馈禁用排查"起飞后左右摇摆"（use_velocity_ff / use_accel_ff，yaml 开关无需编译）**（2026-08-14 用户报起飞后左右摇摆）：bridge（super_to_px4_bridge.cpp:713-732）向 PX4 发 setpoint = 位置 + 速度前馈(velocity_ff_gain×cmd_vel) + 加速度前馈(acc_ff_smooth 低通后) —— 两个前馈都可能引入抖动：`velocity_ff_gain 0.7`（yaml 注释自述"过大会引入抖动"）+ `acc_ff_smooth 0.5` 低通（我们为防俯冲加的，滞后可能导致位置振荡）。**排查/禁用**：`config/super_to_px4_bridge.yaml` 里 `use_velocity_ff: false` + `use_accel_ff: false`（代码有开关判断，**纯 yaml 改动无需编译**，重启按钮3 生效）→ setpoint 变纯位置模式，PX4 只跟踪位置，无前馈抖动。代价：跟踪精度略降（前馈本用于抗风/精度），0.3m/s 慢速影响小。**判断"摇摆是规划还是控制"**：先 `rostopic echo /mavros/local_position/pose -n 3` 看姿态 orientation 是否稳定（稳定=位置层面摆动，不是姿态失控）→ 再查 bridge 前馈参数 → 禁用前馈重测。⚠️ 若禁用后仍摆 → 不是前馈问题，继续查 PX4 PID（MC_*/ATC_*，见 px4-flight-controller-tuning 技能）/视觉定位抖动/风

62. **⭐ 自主探索 + 全局地图持久化方案（2026-08-15 调研定稿，未实施）**：用户要"给现有 SUPER 架构加入自主探索"，且追问"探索后地图怎么保存（雷达扫不到就消散）"——**核心结论：探索=前沿(Frontier)目标生成器 + 复用现有 FSM 执行；全局地图独立累积+文件保存+下次加载先验，三者闭环才有价值**（rog_map 是滑动局部地图，飞机飞走旧区域消散，探索结果必须另存）。架构=新增 1 个 exploration_node（订阅 `/fsm_node/rog_map/frontier` → DBSCAN 聚类前沿簇 → score=信息增益-α×距离选目标 → 发 goal）+ 开 `frontier_extraction_en: true`（**rog_map 内置实现，无需编译**；prob_map.cpp:148/536 + rog_map_ros1.hpp:185 frontier_pub 话题）+ FSM 分毫不动（避障/AVOID_WAIT 全保留）。**goal 通道：bridge 的 goalCallback 已订阅 `/move_base_simple/goal` → 探索节点直接发 PoseStamped 零对接**（⚠️ 先确认 FSM 实际订阅哪个话题，坑1：/waypoint_goal 走航点通道 vs /move_base_simple/goal 走 click 通道高度被覆盖）。开源调研：**FUEL（HKUST ⭐1457，与 SUPER 同源 FAST-Planner 系，集成成本最低）/ GBPlanner（NTNU ⭐842）/ explore_lite（偏2D）/ OctoMap（⭐2353，.bt 持久化）**。实施路线：短期 rog_map 内置 frontier+自写节点（1天）→ 中期借鉴 FUEL 增量前沿 → 持久化 PCD/.npy → 后期 OctoMap；全局地图接入后配 frontend_in_known_free=true（坑56）+ 终点判定 0.3（坑58）解决撞墙/乱跳/乱转。详细方案（接口确认/架构图/开源对比/机制类比/FUEL 全套执行计划/三层解耦设计）见 `references/autonomous-exploration.md`（第八节=FUEL 五阶段路线+集成策略"只借探索大脑不整替换"；第九节=探索层/规划层/飞控层三层解耦+两标准接口，用户明确"解耦方便替换规划器"+"不能让 SUPER 没有作用"→走方案X=FUEL探索层+SUPER规划层）

41. **endpoint.txt（waypoint_mission 数据源）空格格式必须 ≥4 列**（2026-08-11 实机，现象：触发后 `Mavros triggered.` 出现但**无 `Pub goal`**、飞机不动，日志 `Load data from: .../endpoint.txt` + `Load 0 waypoints.`）：加载器（mission_planner config.hpp:116）对**空格格式要求 `result.size() >= 4`**（x y z switch_dis 四列）——只写 `x y z` 3 列会被跳过 → 0 个航点 → CH10 触发后无目标可发。**修复：endpoint.txt 写 4 列** `f"{x:.6f} {y:.6f} {z:.6f} 0.1
"`；nx_command_server.py RecordPathStop 自动生成逻辑**同样要加第 4 列**（初版只写 3 列踩过）。CSV 格式（逗号 7 列）不受影响。判读：`grep 'Load.*waypoints' real_flight.log` 应为 `Load 1 waypoints`，0 = 格式错

63. **⭐ 目标高度被"地面测试临时参数"压死 → 轨迹入地/贴地（2026-08-19 实机，用户报"SUPER 生成的轨迹在地下"）**：real_flight.yaml 的 `goal_z_min: 0.0 / goal_z_max: 0.3`（yaml 注释明写"地面测试临时改"）没改回来 → fsm.cpp:231 路径A钳制代码（`if (gi_.goal_p.z() < z_min) ... > z_max`）把吸附后目标 z 压到 [0.0, 0.3] → 航点 z=0.75 的目标被压到 0.3。**叠加 `fix_map_origin: [0,0,1.5]`（地图原点抬高 1.5m，rog_map.cpp:36 `robot_state_.p = fix_map_origin`）→ 地图系 z = odom 系 z + 1.5 → goal z=0.3(地图系) = odom 系 -1.2m = 地下**——与用户 RViz 看到的现象完全一致。**铁证判读（两行日志对拍）**：`[MISSION] Pub goal to 2.5 0 0.75`（waypoint_mission 发布值，正常）vs `[Fsm] Get goal at 2.325 0.375 0.300`（FSM 吸附+钳制后值，z 被压）——z 不一致=钳制发生。**git 历史定位正常值**：`git log --oneline -- config/real_flight.yaml` + `git show <commit>:config/real_flight.yaml | grep goal_z`——9fcf976（路径A正常版 0.6/0.825）→ 402a86e 引入 0.0/0.3"地面测试临时改" → d906e20 仍保留（一直没改回来）。**修复**：改回覆盖实际航点高度的范围（如 0.5/1.3 覆盖 guidepoints 0.75 与 real_mission 1.2），备份+diff+重启 SUPER。**附带发现**：`max_acc 0.6→0.2`（git diff 未提交的临时改动）过紧 → MINCO `Opt failed, Omg or thr or Pos violation` 无限循环 + `GeneratePolytopeFromLine failed` + fallback A* 循环——轨迹"不合理"的另一半原因。**教训：yaml 里注释带"临时改/测试用"的参数 = 定时炸弹，飞行前 `grep -E '临时|test|TODO' config/*.yaml` 检查**；排查轨迹高度异常先对拍 Pub goal vs Get goal 的 z，别先怀疑坐标系/地图

64. **⭐ "避障太近" = CIRI min dis < robot_r → 走廊失败 → 反复 AVOID_WAIT → 飞不到终点（2026-08-19 复飞复盘）**：用户报"飞行中离障碍太近"。日志刷屏 `[CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: 0.186` → `GeneratePolytopeFromLine failed`（2000+ 次）→ AVOID_WAIT（80 次）→ 飞机停在半路（实测 (2.05, -1.12, ~0) vs 终点 (2.5, 0, 0.75)）。**机制**：ciri.cpp:102 `if (dis < robot_r_ - 1e-2) return FAILED`——CIRI 检查**每个障碍点到 seed line 的最短距离**，< robot_r(0.35) 拒绝走廊 → 无轨迹 → AVOID_WAIT 停车 → 每轮撞同一障碍无限循环。**0.186m < 桨半径 0.23m = 真会擦桨**。**关键认知：CIRI 量的是到真实障碍的距离，不是膨胀后距离**——A* 路径离障碍 0.186m < 膨胀 0.4m = 该障碍没被膨胀覆盖（新出现/动态）或路径贴障碍走。**修复**：① `inflation_step 2→3`（0.4→0.6m 膨胀）让 A* 路径直接离更远（代价：室内窄空间堵死绕行）② ⚠️ **别动 robot_r**——它是 CIRI 拒绝阈值，加大更易失败（方向反了）③ corridor_bound_dis 0.6 已加宽保持 ④ 动态障碍（人）走开自动恢复；地图残留则起飞前原地转一圈重扫。**判定**：min dis 数值反复相同 = 同一固定障碍反复挡路；<0.2 危险贴桨、<0.35 走廊必失败。另注意**重启 SUPER 必须停止/启动拆两个脚本**（pkill 会匹配脚本自身 argv 自杀 exit 128，见 windows-ssh-automation 技能）；agent 改完 yaml 后**复查文件 mtime+内容**（用户可能自行覆盖，本会话 goal_z 修复被用户 13:20 改回 0.0/0.3）——完整版见 uav-super-navigation 坑 42/43

65. **⭐ "明明右边有空间却不绕行，贴着障碍走" = frontend_in_known_free=true 把未知当障碍限死 A* 搜索空间（2026-08-19 实机）**：用户观察"右边飞高一些有路能绕行，但规划器贴着障碍走"。因果链：frontend=true（防撞墙）→ super_planner.cpp:1305/1330 `UNKNOWN_AS_OCCUPIED` → A* 只搜"已知自由"区，**雷达没扫过的右侧/上方 = 未知 = 被当墙** → 唯一可走的路 = 贴已知障碍的窄通道 → CIRI min dis 0.186 < robot_r → 走廊失败 → AVOID_WAIT 卡死。**min dis 每次完全相同 = A* 确定性搜索永远给同一条贴障碍路径，从未考虑右侧**。与坑36 区分：frontend=false 撞墙（未知当自由），frontend=true 过度保守（未知当墙）——**两个方向都危险，frontend 无全局解，必须配全局地图或"先扫后飞"**。**方案A（不改配置）**：起飞前让雷达扫过右侧（地面手持扫/起飞后转一圈）→ 未知变已知 → frontend=true 时立即可绕。验证：扫完 RViz `/fsm_node/rog_map/unk`（橙色）消失；绕行成功日志 `Global path blocked by obstacle, fallback to A* replan` + `A* fallback path corridor OK`。**方案B**：frontend=false 允许穿未知区，但可能撞墙（坑36），只能临时测试用。**✅ 起飞后自动扫描已实施（2026-08-24 origin/main ccbb463）**：super_to_px4_bridge 新增 `scan_after_takeoff_en / scan_angle_deg / scan_yaw_rate`（默认关，渐进旋转防甩头）——坑66 返航"转向扫描"可直接复用。实现基础（若需自定义）：bridge 的 superCmdCallback（super_to_px4_bridge.cpp:244）**无条件接受任何发布者的 /planning/pos_cmd**（super_cmd_active=true），FSM 在 WAIT_GOAL 不发 pos_cmd（fsm.cpp:133）→ Phase4 起飞悬停后、CH10 触发前是天然空窗期：可在 bridge Phase4→5 之间插入 yaw 旋转扫描循环（位置保持 init_goal、yaw 每步+30°×12 步转 360°），或零改动用手动 yaw 杆转一圈再拨 CH10。**⚠️ 扫描必须 yaw 旋转（位置不动）**，位置动了会偏离轨迹起点触发 guide_path 吸附错位（坑15/34）——完整版见 uav-super-navigation 坑 44/45

66. **⭐ 长距离复飞方案（2026-08-23 定稿，已同步桌面 SUPER.md 第九章，备份 SUPER.md.bak_20260823）**：12.5m 已验证；**地图 = 滑动窗口**（map_sliding_en=true，robocentric 窗口中心随飞机）——判定依据：autonomous-exploration.md 记载 + 用户实测"飞走旧区域消散"（铁证）；**⚠️ 别仅凭 real_flight.yaml 的 map_size[30,30,6]+fix_map_origin 推断"固定原点 30m 硬上限"**（agent 首答误判过，被 references 纠正——判断 ROG-Map 行为以 references 实测记录为准，不以参数表外观为准）。长距离单程（50-100m+）可飞（飞到哪扫到哪，走廊只需前方几米），**返航/往返不行**（身后地图消散=摸黑，须坑62 全局地图）。**fix_map_origin [0,0,1.5] 语义 = 地图 z 窗口中心对准飞行高度层**（地图系 z = odom z + 1.5，窗口覆盖 odom [-0.45,3.45]，地面压窗口底边；改它 = 全屏障碍坑44 / 轨迹入地坑63，红线勿动；改作业高度层须联动 virtual_ground+virtual_ceil 三参数）。三批改动清单（必查5项/适配5项/全局地图）、红线、运营注意、patch 编辑 markdown 代码块边界坑（old_string 取自代码块内部会把闭合符 ``` 推走，需两步修复）详见 references/long-distance-replay.md
   - **返航方案（同会话定稿，2026-08-23）**：返航卡死本质=返航路径变未知（身后消散）→ 走廊/A*(frontend=true)走不动 → AVOID_WAIT 死循环；解=让路径重新已知。按成本：①**往返一条航迹录制**（A→B→A 一次录完=更长的单程，零改动，建议第一步）②预录返航航迹+文件切换 ③**反向发布器**（半天 Python）：global_path_publisher.py 加 `_reverse:=true`（`wps=list(reversed(wps))` 即可，path_msg/marker 自动反向）+ 新建 `endpoint_reverse.txt`（=guidepoints_new.txt 第一行 A 点，空格 4 列坑41）+ UserCmd4 按钮（发布器 _reverse:=true + launch waypoint_data_path:=endpoint_reverse.txt；改 yaml 后必须重启 nx_command_server 坑22）④**航迹先验注入**（治本，1-2 天 C++，=坑62 全局地图轻量先行版，接口复用）：prob_map 加 `markFreeLine(p1,p2,width)`——沿航迹把 **UNKNOWN 格标 FREE、绝不覆盖已观测格**（新障碍实时扫到=OCCUPIED 不被覆盖→避障照常；可加坑46 hit_cnt 思路第二层保护），fsm globalPathCallback 收到航迹时逐段调用，width 1.2m 起步；⑤大窗口伪全局（map_size 覆盖全程，纯配置但 Orin NX 8GB 内存/CPU 未实测）⑥坑62 全局地图（终极）。**⭐ 机制分叉（决定返航方案，待 30-50m 试飞验证）**：走廊=前方 planning_horizon 5m 滚动（坑32 暗示）→ yaw 转向+边飞边扫就能返航，零 C++；走廊=最近点→终点全程（坑30/37 暗示）→ 任意未知段整体失败 → 必须先验（方案④）。判读：试飞日志 `Using FULL GLOBAL idx` 持续推进+failed 少=滚动；failed/AVOID_WAIT 刷屏=全程。**yaw 转向返航（用户想法，方向正确）**：本质=边飞边扫（先转向扫清前方再沿航迹飞）；⚠️ **360° LiDAR 转向本身不增加视野**（转不转都扫周围一圈），收益=启动段已知+滚动推进；⚠️ 坑65：扫描/转向必须**位置不动**（动了偏离轨迹起点→guide_path 吸附错位坑15/34）
   - **CH9 中断返航（✅ 2026-08-24 已实现+真机验证，发布器内嵌方案替代原 return_trigger.py 设计）**：FSM 零改动原理=guide_path 最后一点即飞行方向（发布器切换内容=切换方向）。**实现（global_path_publisher.py 内嵌，无新进程/无按钮改动）**：订阅 `/mavros/rc/in`（⚠️ 类型是 **mavros_msgs/RCIn，不是 sensor_msgs/RCIn**！写错 ImportError）→ 检测 `channels[8]`（CH9）下降沿（与 CH10 语义一致：`last>1500 && now<1500`，先高位再拉低，一直停低位不触发）→ ①`pkill -f 'waypoint_missio[n]'`（字符类防自误杀，停发去程 goal——竞态关键，先杀再切，否则新旧 goal 打架 FSM 乱转）②切反向发布（`wps=reversed(wps)`，path_msg/marker 自动反向，FSM 收新 guide_path 自动重规划返航）③**周期重发返航 goal**（=航迹起点，`/waypoint_goal` latch 双保险）。⚠️ 实现坑：触发条件高位必须 **>1500**（1500>1500=false 不触发；真实 PWM 高位 2000/低位 1000）；**rospy 无 spinOnce**（C++ 概念，Subscriber 回调自动在线程跑，直接删掉）；**平时绝不发 /waypoint_goal**（坑33 门禁，仅 CH9 触发后发）。真机验证（遥控器 18 通道 len≥9 ✅）：`rostopic echo /mavros/rc/in -n 1` 看 channels 长度 + 拨杆看 channels[8]（1050↔1950）；下降沿触发日志 `⚡ CH9 下降沿触发返航!` + `已杀 waypoint_mission` + `已切换为反向发布` + `返航 goal -> /waypoint_goal = (0,0,0.75)` 周期重发。⚠️ **`rostopic info /mavros/rc/in` 显示 Publishers: None = mavros 没跑**（话题残留，遥控器开了信号也进不来——先启动雷达环境再测）；测试脚本含 pkill+启动行会自杀（坑10/64），用纯启动脚本。模拟测试（无遥控器时）：`rostopic pub /mavros/rc/in mavros_msgs/RCIn "channels: [1500,...,2000,...]"` → 1000 触发下降沿（高位必须 2000 不是 1500）。依赖：滚动机制成立→零 C++ 闭环；全程→先方案④。⚠️ **未实测风险（完整飞行测试重点观察）**：FSM 的 `global_last_idx_` 单向推进（防跳段：最近点只能从上次索引之后找）在**切换反向航迹时可能不重置** → 返航最近点从去程旧索引开始 → 取错航迹段（从半路开始折返）。判读：返航后日志 `Using FULL GLOBAL idx` 应从反向航迹开头推进（idx 小→大）；异常则需发布器切换时发新路径信号或 FSM 收到新 /global_path 时重置 last_idx。详见 references/long-distance-replay.md 第九节
   - **git 分支实测 + 切 main 实录（2026-08-23/24）**：机载三仓库初始状态——task_run_scripts **停在 feat/obstacle-to-px4（非 main！）**、SUPER=feat/guide-path-c（**无 main**，origin/HEAD→dev/4uav）、NX_code=feat/4uav-guide（无 main 有 master）。**⭐ 分支新旧判断法**：`git log --oneline A..B`——`HEAD..main` 空 = main 是 HEAD 祖先 = 切本地 main 是回退（会丢 obstacle-to-px4 独有 30+ 提交）；但**本地 main 旧 ≠ origin/main 旧**！`git pull origin main --ff-only` 后 origin/main 已合入全部开发成果（obstacle/路径A/RViz插件，+5651 行）——**判断"切 main 丢不丢工作"必须看 HEAD..origin/main，不是本地 main**。切+拉实操：①快照 `tar czf ~/task_run_scripts_backup_<TS>.tar.gz --exclude='task_run_scripts/log' --exclude='task_run_scripts/.git' -C ~/catkin_ws/src task_run_scripts`（含未跟踪 data_guidepoints）②工作区 `M scripts/NX_code`（子模块 gitlink 漂移）挡 checkout/pull → 先 `git checkout -- scripts/NX_code`（只影响父仓库指针，子模块内容不动）③`git checkout -f main` ④`git pull origin main --ff-only`。⚠️ **pull 快进会覆盖本地未提交的跟踪文件**（data_guidepoints/guidepoints_new.txt、endpoint.txt 被 origin/main 版本替换，本地航迹修改丢失——快照兜底）；未跟踪归档文件（guidepoints_YYYYMMDD*.txt）不受影响。⚠️ **config 结构重构**（ccbb463 起）：real_flight.yaml → mid360_real_flight.yaml + odin_real_flight.yaml（按传感器分文件），launch 分家（launch/fastlivo2/、launch/odin/）——旧路径全部失效（见关键文件地图）。⚠️ NX_code 子模块 pull 后仍漂移，需 `git submodule update --init --recursive` 才同步（写操作，会改按钮配置，先确认）。**开发/飞行前必做 `git branch --show-current`（三仓库）+ rosparam 确认实际生效配置**（坑21/28）；新功能（CH9 等）从 origin/main 最新拉新分支开发，别在 obstacle-to-px4 混。fouruav IP 08-24 实测 = <NET_WLAN_A>.45（hostkey 不变=同机，IP 会变 hostkey 是稳定身份）。**⭐ 2026-08-24 阶段0 执行实录（长距离复飞开发启动）**：①**按钮 launch 回归修复**（origin/main 重构后 real_flight.launch 默认 config_path 指向已删除的 config/real_flight.yaml → 直接启动 FSM 加载失败/用默认参数！）：NX_code/User_define_commands.yaml 三处——按钮1/3 `roslaunch task_run_scripts real_flight.launch` → `roslaunch task_run_scripts fastlivo2/mid360_real_flight.launch`（薄包装 include real_flight.launch+覆盖 config_path+rviz_config，use_waypoint/waypoint_data_path 命令行参数自动透传）；按钮2 pkill 改 `'mid360_real_flight[.]launch'`（字符类防自误杀；原模式 'roslaunch task_run_scripts real_flight.launch' 匹配不到新命令行——中间插了 fastlivo2/mid360_）。**⚠️ 此方案随后被推翻**：实测 `roslaunch task_run_scripts fastlivo2/mid360_real_flight.launch` 必 RLException（**参数含 `/` 被当文件系统相对路径**，坑69 修正版）——**最终按钮1 用纯文件名 `roslaunch task_run_scripts mid360_real_flight.launch`**（roslaunch 递归搜索 launch/ 树自动找到子目录文件，最简正解）；按钮2 pkill 模式 `'real_flight[.]launch'`（子串同时匹配顶层和包装两种命令行）。②**nx_command_server 重启又踩 pkill 自杀**（坑10/64 再犯）："杀旧+启新"写同一 plink -m 脚本 → pkill -f 'nx_command_server[2]\.py' 匹配到脚本自身的 nohup 启动行（nx_command_server2.py 明文在 argv）→ exit 128 自杀。**安全流程：`ss -tlnp | grep 6004` 拿 PID → `kill -9 <PID>` → 单独启动脚本（无 pkill）**，验证 `ps -eo pid,lstart` 新 PID + curl 6004 健康检查 + 日志 profile='mid360'。③**摸底实况**：map_sliding 段结构 = `map_sliding: {enable: true, threshold: 0.3}`（yaml 段名是 map_sliding 不是 map_sliding_en！滑动已开）；**max_vel=0.3/max_acc=0.4/max_omg=2.5/max_acc_thr=20/penna_jerk=-5e5 全低速保守版**（非 SUPER.md 的 1.0 时代值）→ 长距离第一步提速 max_vel 0.3→1.0 + max_acc 0.4→0.6（坑49：1.0 配 0.6，一次只改一组）；scan_after_takeoff_en 在 super_to_px4_bridge.yaml（默认关，scan_angle_deg 90、scan_yaw_rate 5°/s）；guidepoints_new.txt 当前仅 2.5m/51 点（需录 30-50m 新航迹）。④**开发分支已建**：feat/long-distance-replay（从 main=origin/main ccbb463 拉）；四阶段计划=0 修回归→1 提速→2 录航迹试飞判滚动/全程机制→3 按结果（卡停则方案④先验 C++）→4 分批提交（task_run_scripts 侧推 feat/long-distance-replay，SUPER 侧续 feat/guide-path-c）

67. **⭐ 按钮2 "没反应+rviz 残留"双 bug（2026-08-24 实机，用户场景：启动雷达→按钮1 弹 rviz→按钮2 无反应 rviz 依旧运行）**：①**`&&` 断链**：按钮2 原 command `rosservice call /mavros/set_mode ... && sleep 1 && pkill ...`——mavros 不在/切换失败时第一条非 0 → && 链断 → 后续 pkill 全不执行 = "没反应"；②**裸 pkill 模式自杀**：command 由 nx_command_server 以 `bash -c '整段command'` 执行，bash 自身命令行含 'fsm_node' 等模式文本 → `pkill -f 'fsm_node'` 匹配 bash 自身 → 自杀 exit 128 → 后面兜底杀 rviz 的 `pkill -f 'top_down.rviz'` 永远执行不到 = rviz 残留（与坑10/23 同源：任何从 shell 执行的 command 里的裸 pkill 都会这样，不限于 plink -m 脚本）。**修复模板**（已上机，yaml 里 `\"` 转义为 `\\\"`）：`rosservice call /mavros/set_mode '{custom_mode: \"POSCTL\"}' 2>/dev/null || true; sleep 1; pkill -SIGINT -f 'real_flight[.]launch' 2>/dev/null; sleep 2; pkill -f 'fsm_[n]ode' 2>/dev/null; pkill -f 'super_to_px4_bridg[e]' 2>/dev/null; pkill -f 'waypoint_missio[n]' 2>/dev/null; pkill -f 'map_to_worl[d]' 2>/dev/null; pkill -f 'top_down[.]rviz' 2>/dev/null; echo button2_done`——`|| true` 防断链、**全部 pkill 正则字符类**（正则仍匹配目标进程，但不再匹配自身 argv 文本）、末尾 echo 完成标记便于判读。pkill 模式用 `'real_flight[.]launch'`（子串同时匹配 mid360_real_flight.launch 和 real_flight.launch——按钮1/3 可能用不同 launch，必须同时能杀）

68. **按钮1 语义：航点任务 ≠ 复飞（2026-08-24 用户拍板升级）**：按钮1 原 command 只有 `roslaunch ...mid360_real_flight.launch`（**无 global_path_publisher**）→ FSM 无 guide_path → waypoint_mission 读 data/real_mission.txt 逐点发 goal → A*+MINCO 自由规划（**不贴航迹**）；复飞必须 发布器（/global_path）+ endpoint.txt（坑29/方案C）。08-24 用户要求"只改按钮1 不动按钮3"→ 按钮1 升级为复飞（发布器 guidepoints_new.txt + **顶层 real_flight.launch + 显式 config_path:=mid360_real_flight.yaml + rviz_config:=mid360_top_down.rviz** + endpoint.txt——⚠️ 子目录包装 `fastlivo2/mid360_real_flight.launch` 命令行 roslaunch 必 RLException 不可用，见坑69），按钮3 还原原样（**仍有 config_path 回归**——real_flight.launch 默认指向已删的 real_flight.yaml，用户选择暂不修）。⚠️ **用户随后自行把按钮3 改成 "空按钮"**——agent 改完配置必须复查 mtime+内容（用户会自行改机载配置，坑63/64 同款实例）；按钮2 的 pkill 模式必须同时兼容两套 launch（用 'real_flight[.]launch' 子串，见坑67）

69. **⭐ roslaunch 不支持 launch 子目录：`roslaunch pkg subdir/xxx.launch` 必 RLException → 按钮 command 必须顶层 launch + 显式参数**（2026-08-24 实锤，按钮1"按下没反应"的最终根因）：
   - 现象：`roslaunch task_run_scripts fastlivo2/mid360_real_flight.launch` → `RLException: [fastlivo2/mid360_real_flight.launch] is neither a launch file in package [task_run_scripts]`——即使 `rospack find task_run_scripts` 路径正确、`launch/fastlivo2/mid360_real_flight.launch` 文件存在
   - 验证法（安全，只解析不启动）：`roslaunch --nodes task_run_scripts <file> [args]`；实测 `fastlivo2/xxx.launch`、`launch/fastlivo2/xxx.launch` 全 RLException，只有顶层 `roslaunch task_run_scripts real_flight.launch` 成功——**该 roslaunch 环境只认 launch/ 根目录文件，launch/fastlivo2/ 与 launch/odin/ 的"按传感器分目录"包装全部命令行不可用**（origin/main 的分目录设计在此环境失效，只能当模板参考）
   - **修复（按钮 command 一律）**：`roslaunch task_run_scripts real_flight.launch config_path:=/home/fouruav/catkin_ws/src/task_run_scripts/config/mid360_real_flight.yaml rviz_config:=/home/fouruav/catkin_ws/src/task_run_scripts/rviz/mid360_top_down.rviz use_waypoint:=true waypoint_data_path:=.../endpoint.txt`（等价包装行为：显式 config_path+rviz_config；use_waypoint/waypoint_data_path 是 real_flight.launch 自带参数）
   - 判读：real_flight.log 第一行 RLException = launch 解析失败（不是节点崩溃）；ps 里发布器在但 roslaunch/fsm_node 不在 = launch 没起来 → cat real_flight.log 看第一行
   - 顺带：origin/main 重构后 `config/real_flight.yaml` **已不存在**（→ mid360_real_flight.yaml + odin_real_flight.yaml）；launch/real_flight.launch 默认 config_path 仍指向旧 real_flight.yaml = 回归点，任何按钮 command 必须显式 config_path
   - **⚠️ 根因修正（同会话晚些实测，2026-08-24）**：不是\"roslaunch 不支持子目录\"——**含 `/` 的文件名参数一律被当文件系统相对路径**（相对 CWD），不含 `/` 的纯文件名会**递归搜索整个 launch/ 树（含子目录）**。实测：`roslaunch task_run_scripts mid360_real_flight.launch`（纯文件名）**成功**（--nodes 出 5 节点，无参数/带参数都行）——launch/fastlivo2/ 和 launch/odin/ 的包装**命令行可用**，只是必须写纯文件名。**正确按钮 command = `roslaunch task_run_scripts mid360_real_flight.launch use_waypoint:=true ...`**（不是顶层 launch + 显式 config_path——那只是过度方案——⚠️ 该结论已被坑71 推翻：纯文件名 wrapper 不透传 waypoint_data_path（只声明 config_path/rviz_config），必须顶层 real_flight.launch + 全显式参数）。⚠️ **同名冲突**：launch/ 根目录与子目录放同名文件 → `RLException: multiple files named [xxx.launch] in package`（agent 曾误建根目录转发文件踩中，删除即可）——**不要为了\"修复\"在 launch/ 根目录复制同名文件**。验证法始终用 `roslaunch --nodes task_run_scripts <纯文件名> [args]`（只解析不启动）\n\n70. **⭐ \"轨迹（绿线）不显示\" = rviz 配置文件缺 GlobalPathMkr 显示项，数据链路其实全通**（2026-08-24 实机，按钮1 修好后用户报\"按下按钮1 轨迹没有显示\"）：\n   - 现象：发布器正常（log `发布 51 个航点`）、FSM 正常（log `GLOBAL PATH received: 51 pts` + `Wait for goal`）、roslaunch/fsm_node/rviz 进程全在——**唯独画面无绿线**\n   - **判读铁证：`rostopic info /global_path_marker` 看 Subscribers——`None` = RViz 显示项没订阅**（数据链路全通但画面没开\"眼睛\"；坑25 同款：Path 显示项不可靠，必须 MarkerArray + 配置文件加显示项，且**每个 rviz 配置文件都要加**——origin/main 新增的 mid360_top_down.rviz 漏加）\n   - 修复（block 格式 rviz 文件可安全字符串 replace，**必须插在顶层 Displays 列表（4 空格缩进独立条目）或 Enabled:true 的组里**，缩进对齐）：\n   ```yaml
        - Class: rviz/MarkerArray
          Enabled: true
          Marker Topic: /global_path_marker
          Name: GlobalPathMkr
          Namespaces:
            global_path: true
          Queue Size: 100
          Value: true
   ```
   - 验证：python3 yaml.safe_load（block 格式可解析；flow 压缩格式必须字符串插入坑40）→ **重启 SUPER（按钮2→按钮1）才加载新 rviz 配置**（已运行的 rviz 持旧配置）
   - ⚠️ **显示项插进 Enabled:false 的组（如 Frontend 组）会被静默禁用**：显示项存在、YAML 合法、重启后 Subscribers 仍永远 None（2026-08-24 第一次插入 Sensing 组内后依旧无绿线，移到顶层才可能生效）——确认顶层结构：`python3 -c "import yaml; d=yaml.safe_load(open('xxx.rviz')); print([i.get('Name') for i in d['Visualization Manager']['Displays']])"`（rviz 的 Displays 在 Visualization Manager 下，不在根）
   - ⚠️ **一键按钮只能按一次**：重复按 → global_path_publisher 固定名节点 "new node registered with same name" 互相顶替（发布器日志出现 `shutdown request`）→ 后续 roslaunch 被同名节点冲突搅乱 → SUPER 起不来（发布器在、fsm_node 不在、real_flight.log 无新内容）。重启流程 = 按钮2 关干净（ps 确认无 fsm_node 残留）→ 只按一次按钮1
   - 排查顺序：①`rostopic info /global_path_marker`（Subscribers=None=显示项缺失）→ ②grep rviz 配置显示项列表（`grep -n 'Name:\\|global_path' <rviz配置>`）→ ③确认实际加载哪个 rviz 配置（按钮 command 的 rviz_config 参数）
   - 顺带：**"启动雷达"按钮弹的 rviz 是 slam_rviz.rviz（点云视图），SUPER 的 rviz 是 mid360_top_down.rviz——两个 rviz 别混**（用户曾把雷达的 rviz 当按钮1 弹的）

71. **⭐ "轨迹没指向终点"双根因 + 08-14~19 修复在机载 SUPER 全丢（2026-08-24 实机，用户 CH10 后轨迹不指向终点）**：
   - **根因①：wrapper launch 不透传 waypoint_data_path → goal 发成 (0,0,1.2) 而非 endpoint**。按钮 command 用 `roslaunch task_run_scripts mid360_real_flight.launch use_waypoint:=true waypoint_data_path:=...endpoint.txt` 时（无论纯文件名还是带前缀），**fastlivo2/mid360_real_flight.launch 只声明/传递 config_path+rviz_config，命令行 waypoint_data_path 被静默忽略** → waypoint_mission 用默认 `data/real_mission.txt`（内容 `0.0 0.0 1.2 0.5`）→ CH10 后 `[MISSION] Pub goal to 0 0 1.2`。**判读铁证：`rosparam get /waypoint_mission/` 看 data_path 实际值**（应=endpoint.txt，错=real_mission.txt）；再对拍 `Pub goal` vs endpoint.txt 内容。✅ 正解 = 顶层 real_flight.launch + 全显式参数（坑67 修正版 command）
   - **根因②：fsm_node 崩溃**。goal 错 → 主轨迹走廊失败（GeneratePolytopeFromLine failed）→ fallback A* → 绕行长段 → **corridor_generator.cpp:113 `throw "seed line too long"` 未捕获 → terminate → fsm_node 死**（日志尾部 `Source "...corridor_generator.cpp", line 113` + `#N` 堆栈 + `Aborted` = 崩溃铁证，坑52 判读法）→ 无 pos_cmd → "没轨迹"。**因果链：goal 错是根，崩溃是连锁**——先修 goal 再谈崩溃
   - **验证"修复在不在"**：`grep -rn 'AVOID_WAIT\|omni\|acc_ff_smooth\|corridor_small' SUPER/super_planner/src/` + `sed -n '105,115p' .../corridor_generator.cpp`（看 throw 是否还在）+ `git log --oneline dev/4uav`（HEAD=8cafee3 即修复前状态）
   - **✅ 最终结局（2026-08-24 晚，重做 4 项后才发现的真相）**：**08-14 修复其实早就提交并推送到远程 feat/guide-path-c**——f5a1575（AVOID_WAIT+omni+acc_ff+corridor_small）、b70b242（seed line 长段插值防崩溃）、0e36bfc（终点判定 0.1 恢复）、2fbadee（scan_after_takeoff 起飞后扫描）——**本地机载停在 8ab2f2c 只是没拉取远程**（stash 空 + reflog 无记录是"未提交丢失"的表象，实际是"提交了但本地落后"）。重做 4 项 + 编译 + commit a1c62e3 后 **push 被拒**（提示"一个仓库已向该引用进行了推送"）才暴露。**处置：放弃重复实现**——`git rebase --abort`（冲突中）→ `git reset --hard origin/feat/guide-path-c` → 对齐远程官方版 2fbadee → 重编译 → 验证。**⭐ 核心教训：怀疑"修复丢失/代码被回退"时，第一件事 `git fetch` + `git log origin/<branch> -5` + `git log --oneline HEAD..origin/<branch>` 对比远程，别急着从技能记录重做**（重做 4 项 C++ 约 2 小时白费）；"配置有（omni_en: true）≠ 代码有"同样先确认分支状态——config 来自修复后的机载提交、源码停在旧分支 = 两个仓库时间点不同步，都是"没拉取"而非"丢失"。**远程提交才是权威**（当时实机验证过），重复实现细节可能有差异要弃用
   - **编译与二进制验证（2026-08-24 实操）**：增量编译只编相关包 `catkin_make --pkg super_planner rog_map -DROS_EDITION=ROS1 -j4`（Jetson 3-8 分钟 vs 全编 10-20 分钟；fsm_node/bridge 在 super_planner，prob_map/config.hpp 在 rog_map）；nohup 后台编译必须 `nohup bash -c 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && cd ~/catkin_ws && catkin_make ...' > log 2>&1 &`（plink 非交互 shell PATH 无 catkin_make，裸 nohup 报"没有那个文件或目录"）；**fsm_node 217MB 链接比 bridge 慢几分钟**，轮询日志等 `Built target fsm_node` 才算完；**strings 二进制验证**：`strings devel/lib/super_planner/fsm_node | grep -c 'AVOID_WAIT'`（应>0）+ `grep -c 'seed line too long'`（应为 0）——strings 是"修复真进二进制"的铁证（源码对≠二进制新，fsm_node mtime 才是真据）。当前机载 SUPER = feat/guide-path-c @ 2fbadee（AVOID_WAIT/seed 插值/omni/acc_ff_smooth/scan_after_takeoff 全部生效，二进制已验证）
   - **系统能力速查（用户问"静态动态避障/悬停等待/严格循迹都有吧"时的标准回答，2026-08-24）**：静态避障 = ROG-Map 占用格 → A* 前端 → CIRI 走廊硬约束（min dis ≥ robot_r 0.35）→ MINCO；动态避障 = 实时点云更新 → 走廊失败 → fallback A* 绕行（坑54：绿线断开=绕行中）→ 障碍消失 omni 清残留 → 自动恢复循迹；悬停等待 = AVOID_WAIT（失败→publishHoldPose 主动悬停→0.3s 限频重试→自动恢复）；严格循迹 = 方案F（guide_path 注入 + attractor 拉贴 + dead_d=0.05 防斜切 + 直发改兜底）。⚠️ 边界：循迹拐角会切角（MINCO 平滑走弦，圆角航迹影响小；直线段误差 ≤0.1m，坑35）；动态避障是反应式非预测式（雷达 4.5m @1m/s ≈4s 提前量，快跑的人可能来不及）

72. **⭐ 子模块 push 冲突：merge 优先于 rebase（merge 不重写已提交 hash → 父仓库 gitlink 指针保持有效）**（2026-08-24 实操）：NX_code 提交按钮改动 fa05b79 后 push 被拒（提示先与远程变更合并）——远程领先 f1871ec（odin 适配：Radar_Launcher.yaml/odom_to_mavros.py）。处理流程：①`git fetch && git log --oneline HEAD..origin/<分支>` 看远程独有提交 ②`git diff HEAD...origin/<分支> --stat` 看冲突面 ③双方改**不同文件**（远程改 Radar_Launcher.yaml/odom_to_mavros.py，本地改 User_define_commands.yaml）→ 无冲突 → **用 `git merge origin/<分支>` 而非 rebase**：merge 产生 merge commit 但**保留 fa05b79 原 hash** → 父仓库已提交的指针（task_run_scripts a24bec4 记录 fa05b79）**仍然有效**；rebase 会重写 fa05b79 → 指针指向旧 hash = 再次漂移 → 父仓库被迫再更新指针（连锁）。merge 后验证 `git log --oneline | grep -c <本地hash>` ≥1 = 指针稳定。⚠️ **提交纪律（坑55 增强版）**：`git add <指定文件>`（不用 add -A）→ **用户本地改动和 .bak 不混入**（本次明确排除：NX_code start_fastlivo2.sh 用户本地修改、task_run_scripts config/odin_real_flight.yaml mass 2.08→1.958 疑似调参残留）——指定文件 add 可保留 .bak 作回退路，不必删。子模块提交后父仓库显示 `M scripts/NX_code`（gitlink 漂移）→ 是否更新父仓库指针由用户拍板（更新=多一个 chore 提交，未来 submodule update 拉到新版本；不更新=下次 update 拉回旧版按钮改动消失）——本次已更新（a24bec4 含 scripts/NX_code）。提交消息用单引号（plink 链双引号会炸，见 windows-ssh-automation 技能）

73. **CH9 返航发布器已上机（templates 同步更新）**：机载 `scripts/global_path_publisher.py` = CH9 内嵌版（坑66 实现细节 + `~reverse` 参数），templates/global_path_publisher.py 已同步为机载版；旧版（无 CH9）在机载 `.bak_pre_ch9`。改动已提交：NX_code feat/4uav-guide fa05b79+075d7ed（按钮1/2/3）、task_run_scripts feat/long-distance-replay a24bec4（发布器+绿线显示+L形航迹+NX_code指针）均 push 远程

#### 路径 A：外部全局路径引导（2026-08 开发中）

用户目标：给 SUPER 一条全局路径（飞一遍录的轨迹），SUPER 沿路径做局部规划+动态避障。调研结论：SUPER **原生无外部路径输入口**（FSM 只有 goal/waypoint 两个订阅；Astar 只接受起点终点；reference_path 仅日志用）。唯一可行=**改 SUPER 源码注入 guide_path**（generateExpTraj 的局部变量，走廊沿它生成）。注入点与文件见 `references/global-path-guide-injection.md`。**方案对比**：抽稀/拐点提取=对死轨迹删点（用户明确否定）；真全局 A* 节点=要处理地图局部性问题（30×30m 窗口+滑动）；源码注入=沿路径优化且动态避障天然保留（走廊基于实时地图重建，遇障碍失败→fallback 原有 A* 绕行→障碍消失恢复）。用户选定：**飞一遍录轨迹→global_path_publisher.py 发 /global_path→FSM 订阅转 vec_Vec3f→planner 滚动取段（前方 5m）替代 A* 的 guide_path**。

**落地状态（2026-08-11 已完成两版，编译 0 错误）**：
- 新增 `task_run_scripts/scripts/global_path_publisher.py`（读 guidepoints_new.txt → 发 `/global_path`，frame_id=map，latch+周期重发；`_file:=`/`_min_pt_dist`/`_rate` 参数，默认 min_pt_dist=0.3m）
- SUPER 改 4 文件：`fsm_ros1.hpp`（订阅 /global_path + globalPathCallback 转 vec_Vec3f + `nh_.param("global_path/roll_dist", 5.0)`）、`super_planner.h`（成员 global_path_/global_path_valid_/global_roll_dist_/global_used_prev_/global_skip_this_round_ + setGlobalPath/clearGlobalPath）、`super_planner.cpp`（generateExpTraj 的 "Second Part" 注释前注入）、`config/real_flight.yaml`（global_path/roll_dist: 5.0）
- **V1（初版）**：注入块滚动取段只取**前方 5m**（global_roll_dist_），5m 之后走原有 `PathSearch(guide_path.back(), goal, ...)` A* 续搜 → 前 5m 沿轨迹、之后 A* 自由（航点密时基本贴轨迹，稀疏时会抄近路）
- **V2（用户要求"除非必须切换 A* 否则一直沿全局路径"）**：注入块改为**取全程**（最近点→终点），并保证 `guide_path.back()` 贴近目标（`(sub_path.back()-gi_.goal_p).norm() > res*5` 则补终点）→ Second Part 走 `(guide_path.back()-goal).norm() < res*5` 的 **NO NEED 分支，不再 A* 续搜** → 走廊沿整条轨迹生成。同时在走廊生成失败处（`SearchPolytopeOnPath` 返回 false，super_planner.cpp ~756 行）注入 **fallback**：`global_skip_this_round_=true` 本轮跳过全局路径 → 用 `PathSearch(robot_state_.p, goal, planning_horizon)` 绕行 → 重新 SearchPolytopeOnPath；每轮 replan 末尾重置 skip 标志 → **障碍消失后自动恢复沿全局路径**。日志判读：正常 `-- [SUPER] Using FULL GLOBAL PATH, idx=N total=M pts`；障碍 `Global path blocked by obstacle, fallback to A* replan` + `A* fallback path corridor OK`；罕见失败 `A* fallback corridor also failed`（force return）
- **按钮3 command 加 global_path_publisher 启动**（先 nohup rosrun 发布器 → sleep 1 → nohup roslaunch real_flight.launch，都在 NX_code 的 User_define_commands.yaml，改后需重启 nx_command_server 才生效）——**⚠️ 后台命令必须用子括号 `(nohup ... &)` 包裹，见坑#22**
- **⚠️ global_path_publisher 必须不抽稀**（2026-08-11 实测，用户纠正"我传输的是全局路径"）：第一版默认 `min_pt_dist=0.3m` 把 705 点抽稀到只剩 **7 点** → RViz 看不到绿线、FSM 收到 `GLOBAL PATH received: 7 pts`。**全局路径≠航点任务：抽稀（waypoint_downsample_dist）是给 waypoint_mission 减少飞行停顿用的，全局路径要保真（点越密轨迹保真度越高），SUPER 自己会滚动取段+走廊生成处理点密度**。修复：`min_pt_dist` 默认改 `0.0`（0=不抽稀全发）。验证：`rostopic echo /global_path -n 1 --noarr | grep length:` 应为几百点（≈源文件行数-1），不是个位数
- **top_down.rviz 加 GlobalPath 显示**：初版用 `Class: rviz/Path, Topic: /global_path`（绿色粗线），**实测不工作（Subscribers 无 RViz，见坑#25）**；最终方案=发布器**额外发 `/global_path_marker`（MarkerArray LINE_STRIP）**，rviz 加 `Class: rviz/MarkerArray, Topic: /global_path_marker, Namespaces: {global_path: true}` 显示项（格式抄 Waypoints 项，插入在 Goal 显示块后；rviz 配置 yaml 里 Goal 的 Name 和 Topic 分行写，锚点用 `Name: Goal` + 后续 `Value: true` 整块）
- 触发方式不变：`start_trigger_type: 1` 仍 CH10 下降沿
- 测试流程：start_fastlivo2 → 按按钮3（一键发布器+SUPER）→ `grep "GLOBAL PATH" real_flight.log` 应见 `GLOBAL PATH received: N pts, roll_dist=5 m` → 起飞拨杆 → 沿绿线飞 → 纸箱测避障

#### ⭐ 方案 C 最终架构（2026-08-11 实飞后定稿，CH10 门禁 + 全局路径引导）

- **use_waypoint:=false 方案被否决**（坑#29 的第一版修复）：global_path_publisher 直接发 `/waypoint_goal`（终点）**绕过了 CH10 门禁**——用户实测"没按 CH10 飞机自己开始循迹"。原因：CH10 触发逻辑在 waypoint_mission 内部（start_trigger_type=1），发布器直接发 goal 时 FSM 收到即规划，触发门禁消失
- **方案 C 定稿**：①global_path_publisher **移除 /waypoint_goal 发布**（只发 /global_path + /global_path_marker，goal 由 waypoint_mission 独占）；②real_flight.launch 恢复 `use_waypoint:=true`；③新建 `data_guidepoints/endpoint.txt`（**空格格式 `x y z`，只含航迹终点 1 点**）作为 waypoint_mission 的数据源 → CH10 触发 → 发终点 goal → FSM goal=终点 + guide_path=完整轨迹 → 全程沿轨迹飞
- **endpoint.txt 自动更新**（nx_command_server.py RecordPathStop 内）：同步 guidepoints_new.txt 后，读其最后一行 CSV 提取 (x,y,z) 写 endpoint.txt（**空格格式 4 列** `f"{x:.6f} {y:.6f} {z:.6f} 0.1
"`，第 4 列 switch_dis 必须有，见坑#41）——每次录制新航迹终点自动跟随，无需手动维护
- **"为什么要只发终点"架构解释**（用户常问）：FSM 需要 goal（飞去哪）+ guide_path（怎么飞）两个输入；发全部 1617 点→goal 一直变→FSM 反复重规划→与全局路径引导打架→乱转；只发终点→goal 固定→guide_path 全程引导→顺滑。且不能用发布器直接发终点（绕过 CH10），必须经 waypoint_mission
- 日志判读：`Pub goal` 应只有一次（终点）；无 CH10 触发时**不应**出现 `Pub goal`（出现=门禁被绕过，查谁在发 /waypoint_goal）

#### SUPER 规划架构（FPP 一体化，非"先全局后局部"）

用户问"SUPER 怎么规划出来的 / 全局+局部思路"时这样讲（代码实证）：
- **FPP = Front-end Path-finding + Back-end MINCO 优化，每次 `PlanFromRest` 一体完成**，没有独立全局/局部规划器节点（不像 move_base 两层）
- Step1 前端：`astar.cpp` 在 ROG-Map 上 3D A* 搜无碰撞粗路径（`frontend_in_known_free: false`=允许穿未知空间）
- Step2 后端：`exp_traj_optimizer_s4.cpp` 把路径点 → MINCO 多项式轨迹，在凸多面体走廊(Polytopes)内优化，约束 max_vel=2.0/max_acc=4.0/起终点状态 → `/planning_cmd/poly_traj`
- Step3 执行：FOLLOW_TRAJ 每控制周期取轨迹点发 `/planning/pos_cmd`，飞行中每 0.1s `ReplanOnce` 重规划（重新 A*+MINCO）
- "全局感"来自每次重规划都重新全图 A*；"局部感"来自 MINCO 只在当前视野凸走廊内平滑。两者耦合，无独立节点
- 外层配合：waypoint_mission 只负责"下一个航点是谁"（/waypoint_goal），FSM 管"规划还是跟"，SUPER 内部管"怎么飞过去"


---

### 来源技能 `super-planner-replay`

> SUPER 规划器轨迹复飞/航点任务系统（fouruav，MID360S+D435+FAST-LIVO2+PX4）：两套复飞架构（Path Replay 与 SUPER 航点任务）、QGC 用户按钮（UserCmd1/2/3）配置与安全关闭、地面验证全流程、常见坑（话题不匹配/进程误杀/NUL 损坏/同名顶替/RViz 白屏）。适用于在 fouruav 上验证/配置/排查轨迹复飞功能。

#### 系统架构：两套独立复飞机制

| 机制 | 入口 | 通道 | 特点 |
|---|---|---|---|
| **Path Replay** | nx_command_server.py 内置 | UDP 6003 指令 + HTTP 6004 状态 | 纯回放 My_Flight_Path.txt，无避障，走 /mavros/setpoint_position/local |
| **SUPER 航点任务** | UserCmd1/3 → real_flight.launch | fsm_node 规划 → /planning/pos_cmd → bridge → setpoint_raw | 规划器+避障，走 OFFBOARD 轨迹跟踪 |

#### 端口与通道（重要）

- **UDP 6003**：QGC 按钮指令通道（RecordPathStart/Stop/Replay/AbortReplay、UserCmd1/2/3）——**不是 HTTP POST！**
- **HTTP 6004**：状态查询（/check_pose /check_file /check_replay_status /check_camera）
- **TCP 6000**：rviz_full_stream 全屏推流（遥控器画面来源）
- UDP 6002：触控转鼠标

#### QGC 按钮配置（User_define_commands.yaml，改完即生效无需重启服务）

- **按钮1** = 启动 SUPER（默认 real_mission.txt）
- **按钮2** = 关闭 SUPER：先 `rosservice call /mavros/set_mode POSCTL`（**&& 链失败则不杀进程**，用户选择保留此保守逻辑）→ SIGINT roslaunch → 兜底杀 fsm_node/bridge/waypoint/map_to_world
- **按钮3** = 启动 SUPER 复飞：固定加载 `data_guidepoints/guidepoints_new.txt`（始终最新航迹）+ **手动拨杆触发**（start_trigger_type: 1，用户明确要求保留拨杆，不自动触发）

#### 录制归档方案（data_guidepoints，2026-08 落地）

- 目录：`task_run_scripts/data_guidepoints/`（**注意在 task_run_scripts 根，不是 NX_code 下**）
- `RecordPathStart` 每次生成**时间戳文件** `guidepoints_YYYYMMDD_HHMMSS.txt`（永不覆盖，历史归档）
- `RecordPathStop` 同步更新 **`guidepoints_new.txt`**（始终=最新航迹，按钮3 固定加载它）
- 实现：改 nx_command_server.py 的 RecordPathStart/Stop 两处（PATH_FILE 动态化 + 复制 new 文件），改完**必须重启 nx_command_server2.py 服务**（安全方式见坑 11/12）
- 旧 My_Flight_Path.txt 保留为历史，不再被录制覆盖
- **录制终点不要落地面**：降落段（z<0.3m）混入航迹 → 复飞冲向地面 + 终点不明显。录制走完轨迹后**保持高度直接 Stop**（用户已接受此习惯，不靠代码过滤）

#### 地面验证全流程

1. **第一块·传感器链路**：`bash start_fastlivo2.sh false`（NX_code 目录）→ 6 项检查：`rostopic hz /livox/lidar`(10Hz) /`/usb_cam/image_raw`(30Hz) /`/livox/imu_tf`(200Hz) /`/cloud_registered`(10Hz, 最关键) /`/aft_mapped_to_init` + `/mavros/state`(connected)
2. **第二块 A·Path Replay 状态机**：`curl http://127.0.0.1:6004/check_replay_status` → IDLE；UDP 发 RecordPathReplay → RUNNING_WAITING（不 armed 不会飞，安全）→ AbortReplay → IDLE
3. **第二块 B·SUPER 链路**：启动 real_flight.launch → 日志 `Load N waypoints` + `WAYPOINT GOAL ENABLE` → 发 `/waypoint_goal` 测试目标 → 日志 `Get goal at [x,y,z]`（验证高度不被覆盖）→ `/planning/pos_cmd` 有输出
4. **RViz 轨迹验证**：top_down.rviz 显示 Occ 占据地图 + LocalCloud + frontend_path；推流截全屏，遥控器可见。**拨杆后出现轨迹线 = 复飞执行中**

#### 关键坑（全部实测踩过）

1. **话题不匹配（坑1）**：FSM 有两个目标通道——click（/move_base_simple/goal，override_height=true 强制 click_height）和 waypoint（/waypoint_goal，不覆盖高度）。waypoint_mission 的 goal_pub_topic 必须指向 /waypoint_goal，否则航点高度全被覆盖成 click_height。task_run_scripts 版已修复，SUPER 仓库原版未修复。
2. **/planning/state 话题不存在**：FSM 只发布 /planning/pos_cmd、/planning_cmd/poly_traj、fsm/path。WAIT_GOAL 状态只能看日志。
3. **前馈参数在 super_to_px4_bridge.yaml**（use_velocity_ff/use_accel_ff/velocity_ff_gain），不在 real_flight.yaml；且默认全开。
4. **switch_dis 单位=米（3D 直线距离）**：yaml 的 3.0 是全局默认；空格格式文件（x y z switch_dis）第 4 列逐点覆盖；CSV 7 列格式无第 4 列 → 用全局默认 3。
5. **录制文件尾部 NUL 损坏**：RecordPathStop 用 terminate() 杀进程，最后一行写入被打断产生 \x00 → Python CSV 解析报 "line contains NUL"。修复：备份后截断到首个 NUL 偏移前。
6. **`pkill -f 'rviz'` 严重误杀**：会杀 rviz_full_stream2.py（推流=遥控器黑屏）和 SLAM roslaunch（命令行含 "rviz:=false" 字样）。必须精确匹配 `pkill -f 'top_down.rviz'`。
7. **同名节点顶替**：重复启动 real_flight.launch → 日志 "Reason: new node registered with same name" → 旧实例全部退出 → RViz 白屏。**RViz 白屏先查 fsm_node 是否存活 + `rostopic hz /fsm_node/rog_map/occ` 有无数据，别急着查 RViz 配置**。
8. **CH10 触发是下降沿**：条件 = last>1500 && now<1500 且 pitch(ch2)<1200。CH10 一直停在低位(1050)不会触发，必须先拨高再拉低。
9. **bridge 自动起飞悬停（Phase 1-5 状态机）**：切 OFFBOARD → bridge 自动飞到 init_goal(z=1.2) 悬停（30s 超时），与 CH10 拨杆（waypoint_mission）完全独立。真机流程：按按钮3 → 解锁+切 OFFBOARD → 自动起飞悬停 → 拨 CH10 → 沿航迹飞。
10. **pkill/pgrep -f 自杀（plink -m 脚本）**：脚本内容含 `pkill -f 'nx_command_server2.py'` 时，plink -m 把整段脚本塞进 bash -c 命令行，**pkill -f 匹配到自己的命令行 → 把自己杀了**（exit 128 无声中断）。规避：① 正则技巧 `pkill -f 'nx_command_server[2]\.py'`（字符类打断字面匹配）；② 更稳：`ss -tlnp | grep 6004` 拿 PID 再 kill；③ 重启脚本里干脆不含任何 pkill/pgrep 命令，只做启动+验证。
11. **nohup 启动 python 服务必须带 ROS 环境**：`nohup python3 -u nx_command_server2.py` 直接跑会 `ModuleNotFoundError: rospy`。正确：`nohup bash -c 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export DISPLAY=:0 && exec python3 -u nx_command_server2.py' &`。启动后验证：`ss -tlnp | grep 6004` + `curl http://127.0.0.1:6004/check_replay_status`。
12. **路径层级坑**：改 nx_command_server.py 里 `os.path.dirname(os.path.abspath(__file__))` 只到 NX_code/ 一级——归档目录会建在 `NX_code/data_guidepoints/` 而按钮3 找 `task_run_scripts/data_guidepoints/`，两边对不上。拼 task_run_scripts 根要用 `os.path.dirname` 连跳三层（__file__=…/scripts/NX_code/xxx.py）。
13. **抽稀 vs switch_dis 关系**：抽稀间距(0.1m)远小于 switch_dis(3.0m) 时，到达判定会让相邻航点"瞬间连跳"，抽稀白做。0.1m 抽稀建议 switch_dis≈0.3m。本地模拟抽稀（零风险）：Python 复刻 config.hpp 的 CSV 分支逻辑（距上个保留点<阈值跳过）算保留点数。
14. **RViz 航点可视化**：waypoint_mission 发布 `/waypoint_mission/mkr`（钢蓝航点线 + 橙色切换球半径=switch_dis×2 + 黑色编号），**top_down.rviz 默认不含此显示项**，需手动加 MarkerArray Display（改 rviz 配置后重启按钮3 生效）。FSM 目标点 `/fsm_node/visualization/goal` 已配置。

#### 航点文件地图

| 文件 | 用途 | 备注 |
|---|---|---|
| `data_guidepoints/guidepoints_new.txt` | **按钮3 固定加载（始终最新）** | CSV 7列，RecordPathStop 时同步 |
| `data_guidepoints/guidepoints_YYYYMMDD_HHMMSS.txt` | 历史归档（每次录制一份，不覆盖） | 时间戳排序，可按需回溯 |
| `scripts/NX_code/My_Flight_Path.txt` | 历史数据源（不再被覆盖） | 旧 PATH_FILE，保留作参考 |
| `data/real_mission.txt` | 按钮1 默认航点 | 空格 4列 x y z 切换距离 |
| `SUPER/mission_planner/data/*.txt` | 源码副本 | 实际运行不读这里 |
| 旧脚本写死 `~/odin_ws/src/task_run_scripts/waypoints.txt` | record.py / task_node_nav | **目录不存在，已失效** |

#### 用户偏好（必须遵守）

- **保留手动拨杆触发**航点任务（start_trigger_type: 1），按钮只负责启动系统
- **改核心脚本要受控**（2026-08 演变确认）：用户最初说"不要写在脚本中这个逻辑"（指不想为归档需求写独立重命名脚本），但**接受对 nx_command_server.py 的最小侵入修改**（"OK，你先改，我测试"）。前提：先备份 + 改完贴 diff + py_compile 语法检查 + 改完重启服务验证。独立归档脚本方案（重命名思路）被用户否决，改为直接改 RecordPathStart/Stop 生成时间戳文件
- 改任何文件先备份 + 贴 diff 透明汇报；测试由用户执行，agent 盯日志判读
- 指挥官模式：命令一次 1-3 条 + 预期输出

#### 路径 A：全局路径引导（2026-08-11 落地，分支 feat/4uav-guide + SUPER dev/4uav）

**目标**：无人机沿录制航迹（全局路径）飞行 + 动态避障，代替"逐点航点任务"。
- **数据链**：`guidepoints_new.txt` → global_path_publisher.py → `/global_path`(nav_msgs/Path 全量点) + `/global_path_marker`(MarkerArray 绿线, RViz 显示用) + `/waypoint_goal`(终点 PoseStamped)
- **SUPER 源码改动**（3 文件，全部有 .bak_globalpath 备份）：
  - `fsm_ros1.hpp`：订阅 `/global_path` → globalPathCallback → `planner_ptr_->setGlobalPath()`
  - `super_planner.h`：成员 `global_path_/global_path_valid_/global_roll_dist_/global_used_prev_/global_skip_this_round_` + setter
  - `super_planner.cpp` generateExpTraj：guide_path = 全局路径[最近点→终点]（全程不截断）；走廊失败 → fallback A*（global_skip_this_round_ 本轮跳过，下轮自动恢复）
- **纯全局路径模式**：按钮3 加 `use_waypoint:=false`（real_flight.launch 已有该 arg）→ 禁用 waypoint_mission 逐点发 goal，只靠 global_path_publisher 发**终点** goal → 消除"逐点切换与全局路径打架"（表现为飞机乱转）
- **常见失败链（GeneratePolytopeFromLine failed）**：全局路径附近地图被障碍/人占用 → 走廊生成失败 → PlanFromRest failed 循环重试 → bridge 超时切 POS_HOLD。**测试时人必须远离飞机 2m+**，起飞前雷达转一圈清地图
- **沿轨迹程度**：走廊沿全局路径生成 + MINCO 在走廊内平滑（拐角圆滑化，非严格贴折线）；障碍时 A* 绕行（偏离但安全）；**起飞点必须对齐航迹起点**否则先飞向最近点

#### 关键坑（续）

15. **FSM 加载的 real_flight.yaml 是 task_run_scripts 的，不是 SUPER 仓库的**：launch 里 `config_path=$(find task_run_scripts)/config/real_flight.yaml`。改 SUPER/super_planner/config/real_flight.yaml **不生效**！raycasting/地图参数必须改 task_run_scripts 那份。排查时 `ps aux | grep fsm_node | tr ' ' '
' | grep yaml` 确认实际加载路径。
16. **PX4 1.17 参数体系（微空 NxtPX4v2）**：
    - `EKF2_AID_MASK` **已废弃**（读出来=0 但≠禁用视觉）；视觉融合看 `EKF2_EV_CTRL`（=11 正常）
    - **`CP_DIST`/`CP_GO_NO_DATA` 致命组合**：CP_DIST>0（开了避障）+ CP_GO_NO_DATA=0（无数据不可飞）+ obstacle_to_px4 没跑（无 /mavros/obstacle/send 数据）→ PX4 认为全向 1m 内都是障碍 → **POSCTL 切不进/自动回退 HOLD**，报 "No valid global position estimate"。修复：`CP_DIST=-1` 关避障，或先启动 obstacle_to_px4 数据源。完整文档：NX_code/obstacle_warning.md（本会话产出）
    - 判断视觉融合是否正常：**看 .ulg 的 estimator_status_flags `cs_ev_pos=1`**，别信 /mavros/estimator_status 的 pos_horiz_abs（flag 布局不同会误导）
17. **编译必须 `catkin_make -j4 -DROS_EDITION=ROS1`**：livox_ros_driver2 双版本，默认走 ROS2 分支（缺 ament_cmake_auto 报错）
18. **RViz 显示全局路径用 MarkerArray 而非 Path 显示项**：top_down.rviz 的 rviz/Path 显示项不工作（订阅不上），改用 `/global_path_marker`(MarkerArray LINE_STRIP 绿线) 可靠。top_down.rviz 是**压缩 YAML 格式**（flow style），python `yaml.dump` 重写会破坏原显示项——**用 yaml.safe_load 读→改 dict→yaml.dump 写回**才安全
19. **global_path_publisher 同名顶替**：重复启动报 "new node registered with same name" → 旧实例被关。重启前先按 PID 杀干净（pkill -f 会匹配到自己命令行，见坑 10）
20. **按钮3 command 里 `& &&` 卡死**：`nohup cmd & && sleep 1 &&` 会让 nx_command_server 执行超时（timed out）什么都不启动。正确：`(nohup cmd1 &) && sleep 1 && (nohup cmd2 &)` 子括号包裹后台命令

#### 日志分析方法（实飞后判读）

1. 事件时间线：`grep -aE "Mavros triggered|Pub goal|Get goal|FOLLOW_TRAJ|NONEED|POS_HOLD|timeout|OptimizationExp|SUCCESS" log/real_flight.log | tail -25`
2. 飞行轨迹：`grep -a "POS+VEL" log/real_flight.log | tail -20`（x/y/z/vx/vy/vz 是否沿轨迹）
3. 对照模板：启动→起飞（Publishing initial setpoints→OFFBOARD engaged→reached altitude）→触发（Mavros triggered→Pub goal→Get goal）→规划（GenerateExpTrajectory SUCCESS→FOLLOW_TRAJ）→飞行（ReplanOnce succeed + POS+VEL 沿轨迹）→停（NONEED / OptimizationExp failed / SUPER command timeout→POS_HOLD）
4. 停止原因三连：`close to goal`(认为到了) / `OptimizationExpTrajInPolytopes failed`(走廊优化失败) / `SUPER command timeout`(bridge 0.5s 无指令→POS_HOLD 安全兜底)


---
