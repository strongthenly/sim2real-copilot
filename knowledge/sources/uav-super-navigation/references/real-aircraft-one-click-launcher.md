# 实机一键启动脚本：设计铁律与提速清单（fouruav 实测，2026-10-08）

> 场景：给机载电脑做「按一个按钮 → 起全套（SUPER + FUEL + 仲裁器 + RViz）」的启动脚本。
> 用户在 QGC/地面站上按按钮，脚本必须在**几秒内给出反馈**。
> 实测把 20.7 s 压到 3.9 s。这里既是他明确提过的要求，也是反复踩过的坑的汇总。

## 0. 用户的两条硬要求（别再问，照做）

1. **一键启停必须秒级返回**。原话："按下到响应的时间太长"。宁可让 ROS master 这类常驻服务留着，
   也别每次重启等分钟级。**单文件 + 开关**（`--status` / `--with-super`），不要堆一堆脚本。
2. **用户看画面是「屏幕串流」，不是本机显示器**。
   `scripts/NX_code/rviz_full_stream.py`（`import mss`，抓**整屏**再 jpeg 推给地面站）。
   ⇒ **屏幕上最上层的那个窗口 = 他看到的画面**。
   这把"rviz 没有任何显示 / 一片红 / 黑屏"这类报障的排查方向完全改写：
   先查**屏幕上到底哪个窗口在最上层**（`xwininfo -root -tree`），而不是先怀疑 RViz 配置。

## 1. 铁律一：脚本【不 await 任何东西】—— 启动慢的元凶是脚本自己

`roslaunch` 启动本身要 10~20 s（含 RViz 的 Qt/OGRE 初始化），**这个没法快，但脚本不必等它**。
把「逐项 await」全部拆掉，只**并行后台发起 + 立刻返回**，慢的部分交给后台自然就绪。

实测 20.7 s → 3.9 s，逐项清单（照抄即可）：

| 慢在哪（都是"脚本自己在等"✗） | 改成 | 省 |
|---|---|---|
| `rosnode kill` **循环 11 次**（每次一条命令要建连接） | **一条命令杀所有【实际存在】的节点**：先 `rosnode list` 取交集，再 `rosnode kill $KILL` | 3~10 s |
| `rostopic hz X` 做前置检查（**要采 1~2 s 样本**） | **`rostopic list \| grep -qx X`** 快查（只判话题在不在） | 3~8 s |
| `du -sm ~/.ros/log`（**遍历目录**） | **`ls -1 ~/.ros/log \| wc -l`** 数条目（阈值 >150 项就清） | 1~2 s |
| 清理后 `sleep 2` + `sleep 1` | 压到 **`sleep 1`**（别全删——这 1 s 是保证新节点能起来的） | 2 s |
| `sleep 8` 固定等待 | **删掉**（后台发起，不需要等） | 8 s |
| 逐项 `rosnode list` 轮询（8 次 × `timeout 5`） | **删掉**，改由 `--status` 按需查 | 最多 40 s |

**剩余 ~4 s 的构成**（这是地板，别再抠）：`rosnode kill` 一条 ~1~2 s · `sleep 1` · `rostopic list` ~0.5 s ·
3 条 `nohup` 发起 <0.1 s。

**配套**：脚本尾部提示"约 15~20 秒后就绪，确认: `bash ~/xxx.sh --status`"，
`--status` 子命令秒回（只查节点表 + 关键话题发布者 + 关键日志最后一行）。

### 诊断原则（可迁移）
**当"按钮响应慢"时，先审计脚本自身所有阻塞调用**：循环里逐条调子进程、会采样的命令（`hz`）、
遍历大目录（`du`/`find`）、固定 `sleep`。不要先去怀疑守护进程慢——`roslaunch` 的启动时间在
脚本里是可以并行掉的。

## 2. 铁律二：幂等 —— 启动前清同名旧节点

```
症状：关键节点"起来了但行为不对"（例：仲裁器等不到触发信号）
真因：节点被启动了两次，ROS 报
      shutdown request: [/goal_arbiter] Reason: new node registered with same name
      → 先起的被顶掉，而【后起的那个参数可能不对】
      （实测：第一次带 _watch_topic，第二次没带 → 活下来的永远等不到触发）
```
**做法**：启动前 `rosnode kill` 掉本次要起的全部同名节点（见上表第 1 行，一条命令），
再收掉旧 `roslaunch` 父进程（按 `/proc/<pid>/cmdline` 匹配 `real_flight`/`fuel_real_airborne`，
**不要**用 `pkill -f 'roslaunch ...'`——见本技能 `plink -m` 自匹配那条坑）。

## 3. 铁律三：一个话题只能有一个"目标源"

```
症状：拨 CH10 之后没有轨迹；SUPER 日志一直
      [Fsm] Current state: FOLLOW_TRAJ / GLOBAL PATH received: 106 pts
真因：/waypoint_goal 有【两个发布者】：
        /global_path_publisher   ← 固定航点任务，把 fsm 一直牵走
        /goal_arbiter            ← 探索目标
      → 探索目标被固定航点打架
```
**做法**：探索模式下**不启动 `global_path_publisher`**（它只服务固定航点任务）。
`/waypoint_goal` 在探索模式下的发布者应当**只有 `/goal_arbiter`**——这是一条**可在启动后立即断言**的健康检查。

`waypoint_mission`（CH10 触发那个）保持独立：用一份**独立 yaml** 把它的输出改道到
`/waypoint_goal_task`（`sed 's|^goal_pub_topic:.*|goal_pub_topic: "/waypoint_goal_task"|' waypoint.yaml > waypoint_explore.yaml`），
**原 `waypoint.yaml` 一字不动**，于是"飞固定航点"的老用法随时可用。仲裁器 watch `/waypoint_goal_task`
→ 收到即"操作员拨了 CH10"。

## 4. RViz 三类报障（按此顺序查，别跳）

### 4.1 「视角不对」→ `Views.Current.Class`
- 仿真的探索视角是**正俯视**：`Class: rviz/XYOrbit` + `Pitch: 1.5707963267948966`（90°）+ `Distance: 30`
- `ThirdPersonFollower`（Pitch≈0.2）是第三人称斜视，用户会说"不像仿真"
- **抄现成的**：`rviz/mid360_top_down.rviz` 的 `Views` 段就是正的，整段搬过来最稳
- `Target Frame` 用 `<Fixed Frame>`

### 4.2 「RViz 里什么都没有」→ 话题名对不对
- **不要照搬仿真 rviz 配置的话题名**：同一套 FUEL 源码在不同构建里话题名可能不同。
  实测 `exp.rviz` 里一半话题在实机不存在（`/sdf_map/occupancy_inflate` vs 真名
  `/sdf_map/occupancy_local_inflate`；`/sdf_map/unknown`、`/sdf_map/esdf` 干脆没有）。
- **正确源**：`rosnode info /<node>` 的 **Publications** 段（**只看 Subscriptions 会白跑一轮**）。
- **⚠️ 但 `rostopic hz` 不能当"有内容"的证据** —— 见下节 4.2.1，这一条本会话曾把我带反。
- **探索"边界/前沿"话题是哪一个（实测澄清，别再猜）**：

  | 话题 | 类型 | 实况 | 结论 |
  |---|---|---|---|
  | `/heading_planner/frontier` | PointCloud2 | `heading_planner.cpp:594 frontier_pub_.publish(msg)`，由 `setFrontier()` 喂 | ★★★ **这才是"需要探索的边界"**。**探索真正开始后才有**（未开始 → 空，属正常） |
  | `/heading_planner/visib` | PointCloud2 | 所有 `publish` 调用**在源码里被注释掉了**（316/386/558 行）→ **死代码，永远为空** | 从 RViz 配置里删掉，别留着吃 "No message" |
  | `/planning_vis/frontier` | Marker（CUBE_LIST, `ns:"frontier"`） | `planning_visualization.cpp:23`，**规划阶段**才填；**闲置时是 `pts=0` 的空消息但仍在高频发** | 是"前沿**候选视点**"画的一堆小方块，**不是**边界点云 |

### 4.2.1 ★★★ 头号陷阱：`hz` 高 ≠ 有内容（一个话题可以高频发【空】消息）

```
实测：rostopic hz /planning_vis/frontier   →  354~369 Hz     （看起来"在狂发"）
      取一帧看：ns=frontier  type=6(CUBE_LIST)  color=(0,0,0)  pts=0   ← 空消息 ✗
      同理 /sdf_map/occupancy_local_inflate → 20 Hz，但 width=0（空点云）
      同理 rostopic info 显示 Publishers 存在，也不代表消息非空
⇒ 我曾据 hz 判定"前沿可视化有数据"，结论完全反了，白查一轮。
```

**铁律：判"RViz 能不能画出来"，必须取一帧看内容。**
- PointCloud2 → `width * height`（=0 就是空）
- Marker 列表型（CUBE_LIST/POINTS/LINE_LIST/TRIANGLE_LIST）→ `len(msg.points)`
- MarkerArray → `len(msg.markers)`
- Image → `width`/`height`

**直接跑现成脚本，别手打**：`scripts/ros_topic_truth_probe.py`（只读）
```bash
python3 scripts/ros_topic_truth_probe.py --all     # 扫 /sdf_map /planning /heading_planner 等
# → 每个话题标 [有内容]/[空]/[无数据]，并打印点数、marker 类型+颜色、z 范围
```
**另注**：`空 ≠ 坏了`。多个话题（`/heading_planner/frontier`、`/planning_vis/*`）要**功能真正跑起来**
（拨 CH10 / 开始探索）才有内容。所以"配置里留着但暂时空"是有意义的，**别一律删掉** ——
但要在交付说明里写清"哪个要等开始探索才出现"。

### 4.2.2 两个改 `.rviz` 时的坑

- **Marker 显示项若没有 `Topic` 字段，RViz 默认订阅 `/visualization_marker`**。
  实测 `exp.rviz` 的 `frontier` / `AABB` / `visib_constraint` 三项就没有 `Topic` →
  它们看的是 `/visualization_marker`（实机上是 **`/laserMapping`（FAST-LIVO2）在发**，不是 FUEL）。
  ⇒ 排查"这个显示项到底订阅了什么"时**不能假设**，要读配置。
- **批量改 `Enabled` 时【不要用名字前缀匹配】**。
  实测踩坑：脚本想"关掉所有非 `★` 开头的重复项"，结果**把自己刚加的 `★` 项也关了**
  （匹配逻辑写在 topic 上、前缀判断失效）→ 最终没有任何一项指向有数据的话题 → RViz 全空。
  ⇒ **正确做法：先 `Enabled: false` 全部，再按【实测有内容的话题】从模板重建目标项**
  （`copy.deepcopy` 一个同 Class 的现成项再改字段，schema 才对），改完**跑一遍 probe 脚本校验**。
  > `yaml.safe_load` / `safe_dump` 往返 **是安全的** —— 用 `QT_QPA_PLATFORM=offscreen rviz -d` 验过能加载。
  > 真正会咬人的是【逻辑写错】，不是序列化。

- `exp.rviz` 的 `Simulation`/`Estimation`/`Tracking` 三组是仿真专有（`/map_generator/*`、
  `/pcl_render_node/*`、`/uart_odom`、`/Robot_1/*`、`/dynamic/*`）→ 实机只会显示
  "No message received" 并把左侧显示树撑很长 → 应 `Enabled: false`。

### 4.2.3 ★★ 「怎么突然全变灰了」→ `AxisColor` 的默认配色是【黑→白灰度】

- RViz 的 `Color Transformer: AxisColor`（按轴着色）**默认 Min Color = 黑 / Max Color = 白** →
  整片点云呈**灰阶**，不是彩虹色 ✗。
  实测把占屏最多的两块点云设成 AxisColor 后，用户立刻反馈「这下 rviz 怎么都变灰了」。
- ⇒ 想要「按高度彩虹色」必须**显式设 `Min Color` / `Max Color`**；否则就用 `FlatColor` + 明确颜色，
  或干脆保留话题自带的着色方式（`Intensity`）。**不要把 AxisColor 当成"彩色"**。
- 顺带：**一个话题只留一个显示项**。重复项叠着画会同时出现两种配色，是"画面乱"的常见来源。
  改完跑一次 `scripts/ros_topic_truth_probe.py` 核对话题清单，并断言「无重复话题 / 无残留 AxisColor」。

### 4.2.4 ★ RViz **只在启动时读配置** → 别陷入「改配置 ↔ 让用户重按按钮」的循环

- 改完 `.rviz` 必须**重启 RViz**（= 重按按钮）才生效。
- **更省事的路径（用户偏好，优先给他这条）**：让用户**直接在 RViz 里改** ——
  勾选框开关显示项、点色块改色、`Size (m)` 改点径、滚轮缩放、左键拖平移、中键拖旋转、
  左上 `Views` 下拉换视角（有 `TopDownOrtho` 正交俯视）；改完 **`Ctrl+S` / `File → Save Config`**
  存回同一个 `.rviz` → 下次按按钮自动生效 ✓。
- ⚠️ **没保存 = 下次启动回到你写的版本**（用户会以为改了没用）。交付话术里要明确写「改完必须 Ctrl+S」。
- 会话教训：本次为调一个 `.rviz` 来回改了 5~6 轮、每轮都要用户重按按钮 → 很费时。
  **一次给全 + 教会用户自助** 远比反复代改高效。

### 4.3 「黑屏」→ 两类，都不是 RViz 坏了
1. **Fixed Frame 不存在**：RViz 的 `Global Options → Fixed Frame`（如 `world`）不在 TF 树里。
   - 诊断：`rosrun tf tf_echo <A> <B>` 报 `Could not find a connection between 'A' and 'B' because they are not part of the same tree`
     → 坐标系不存在；`rosrun tf tf_monitor | grep -i <frame>` 列出现有帧与父链。
   - 修复：把提供该帧的 static TF 起起来（如 `map → world` 由 `real_flight.launch` 里的
     `static_transform_publisher 0 0 0 0 0 0 map world 1000` 提供）；
     **或** RViz 跑起来后在 Global Options 里把 Fixed Frame 重选一次（RViz 启动早于 TF 就绪时需要）。
2. **RViz 是从【没有 DISPLAY】的会话起的**（典型：agent 用 ssh/plink 代跑脚本）。
   - 症状：`qt.qpa.xcb: could not connect to display` / `Unable to create the rendering window after 100 tries`
     → rviz 进程秒退或黑屏，而【节点表里没有 /rviz】。
   - **这不是飞机的问题**：用户在物理机上按按钮就会正常。
   - **agent 自查配置能否加载（无显示器也可做）**：
     `export QT_QPA_PLATFORM=offscreen; rviz -d <file> 2>&1 | grep -viE "GLX|OGRE|parentWindowHandle|Stereo|OpenGl|Forcing|compiled against|rviz version"`
     → 过滤掉 offscreen 固有的 GLX 噪声后，配置错误会直接打印出来。
   - **跑一次脚本做联调测试后，记得把 agent 自己起的那个坏 rviz 清掉**，
     按 cmdline 精确匹配（`*explore_fuel.rviz*`），**绝不能碰用户的 `slam_rviz.rviz`**。

## 5. 可视化语义速查（用户会问"这一堆是什么"）

> **★ 交付要求（用户偏好，别再等他问）**：每次改完 `.rviz`，**随配置一起给一份
> 「显示项 → 含义」对照表**，并标明**默认开/关**与**"要等到哪一步才出现"**。
> 实测他会连着追问四次（"黄色物块是什么"、"红色的点是什么"、"frontier 是什么"、
> "怎么看探索边界"）——**主动给表**比被问四次省事，也是他判断配置对不对的依据。
> 表里还要写一句"**颜色约定**"（红=待探索边界 / 深灰=已探索 / 黄=未知 / 青=实时点云），
> 否则他会把不同显示项的颜色当成同一个东西。

| 看到的东西 | 是什么 | 怎么处理 |
|---|---|---|
| 飞机附近**一堆黄色小方块** | `/planning_vis/frontier`（`ns:"frontier"`，`type: 6` = **CUBE_LIST**）= FUEL 的**前沿候选视点**，飞机附近视野最好所以最密 | **正常**。嫌乱就把该显示项 `Enabled: false`（RViz 里可随时勾回）。`/planning_vis/visib_constraint` 同理 |
| **红色的点**（大片） | 几乎必然是 `/sdf_map/depth_cloud`（**实测 7.3 万点**）= FUEL 用深度图 raycast 出的**障碍物/地形表面**。红是因为该显示项 `Color Transformer: Intensity`（按强度上色，暖端偏红）；而它常**有两个重复显示项**叠着画，看着更乱 | **不是故障**。要消除困惑：把这几项统一成 `FlatColor` + 固定色（如青/黄/橙），并**关掉重复项**；`depth_cloud` 是"飞机看到的东西"，若只关心探索边界可默认关 |
| 一条**绿色航迹** | `Path(/waypoint_mission)` 或 `/planning/travel_traj`（Marker, ~9 Hz，`traj_server` 发） | 两个都加进配置：`/planning/travel_traj` 才是【真实执行的轨迹】 |
| 地图逐渐长出来 | `/sdf_map/occupancy_all`(7 Hz, 4040 点) · `/sdf_map/occupancy_local`(20 Hz, 3674 点) | 都要加。**注意**：名字最像"局部地图"的 `/sdf_map/occupancy_local_inflate` 实测是 **width=0 空点云** —— 先 probe 再配 |
| **要看的"探索边界"** | `/heading_planner/frontier`（PointCloud2） | 设成**醒目纯红 + 点径 0.15** ；**开始探索后才有**，见 4.2.1 末段 |

**"边界"相关的三类话题别再混**：`/heading_planner/frontier`（点云=边界本身，★要看的）、
`/planning_vis/frontier`（CUBE_LIST=候选视点方块）、`/heading_planner/visib`（死代码，永空）。

## 6. FUEL「到底开始探索了没」怎么判 + 为什么地面上看不到探索边界

### 6.1 触发链路是【三段】，别只测第一段

```
CH10 → waypoint_mission → /waypoint_goal_task
     → goal_arbiter → /move_base_simple/goal                    ← 你手动模拟 CH10 只到这一段
     → waypoint_generator → /waypoint_generator/waypoints       ← ★ 中间段（最容易漏）
     → exploration_node   WAIT_TRIGGER → PLAN_TRAJ
```

- `waypoint_generator` 订阅 `/move_base_simple/goal`，发布 `/waypoint_generator/waypoints`（`nav_msgs/Path`），
  `exploration_node` 订阅后者 —— 这一段**不在** `rosnode info /exploration_node` 的 Subscriptions 里显形为
  `/move_base_simple/goal`，所以只看一个节点会以为链路断了。
- **决定性判据（比看 hz 可靠得多）**：`grep -aE "Triggered|triggerCallback" <fuel.log>`
  → 出现 `Triggered!` + `[triggerCallback]: from WAIT_TRIGGER to PLAN_TRAJ` 才算**真触发** ✓
- 想跳过 CH10 直接验后半段：
  ```bash
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
    "{header:{frame_id:'world'},pose:{position:{x:2.0,y:0.0,z:1.5},orientation:{w:1.0}}}"
  # 同时另开一个终端：rostopic echo -n2 /waypoint_generator/waypoints  ← 看有没有被转发
  ```
- 另外：**仲裁器日志为空 ≠ 仲裁器没工作** —— `nohup python3 x.py > log` 时 Python **stdout 被缓冲**，
  要跑一段时间才落盘（想要即时日志用 `python3 -u`）。别据此判定它挂了 ✗。

### 6.2 ★★★ 飞机在地面上【不动】→ FUEL 必然很快 FINISH → **永远不会出现探索边界**

```
实测：触发成功（Triggered → PLAN_TRAJ）之后，日志立刻变成
   new num: 0, new dormant: 3
   [FSM]: state: FINISH
   finish exploration.
→ /exploration_goal 无数据、/heading_planner/frontier 无数据 → RViz 里当然没有边界 ✓
```

**原因**：FUEL 靠**飞机移动**来扩展地图；不动 → 地图不增长 → 算出的那几个前沿很快被判
`dormant`（休眠/不可达）→ FUEL 认定「探索已完成」→ **不再发布前沿**。

**⇒ 这是算法特性，不是 bug。** 地面上能看到的只有地图点云（`/sdf_map/occupancy_*`）。
**要看到红色探索边界，只有三条路**：
1. **飞起来**（正解）；
2. **拿着飞机走一圈 / 转一下**（D435 看到新画面 → 地图扩展 → 前沿立刻出现）—— 地面可验证的好办法；
3. 在 RViz 里点 `2D Nav Goal` 指一个远处的点。

⚠️ **别把「地面上看不到边界」当故障去查配置/话题** —— 本会话在这上面绕了很久。
这也是同一根因的另一面：最早报的「frontier 一片红 / 不更新」同样是【没有活跃前沿】。

### 6.3 脚本 + 按钮两个「静默失效」

**(a) `set -u` 会让脚本第一步就死** ✗
```
/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: 行 3: ROS_DISTRO：未绑定的变量
```
ROS 的 `setup.bash` 内部会读**尚未定义**的 `$ROS_DISTRO`；脚本里写 `set -u` 就把它变成致命错误，
**在 `source` 那一行直接退出** ← 「按钮点了没反应」的隐形杀手。
⇒ 机载脚本里**不要写 `set -u`**（`set -e` 同理慎用）；要严格模式就在所有 `source` **之后**再开。

**(b) `nx_command_server.py` 的按钮白名单是【硬编码】的**
```python
# 原来 —— 只有 3 条按钮生效，后加的 UserCmd4/UserCmd5 白加 ✗
elif cmd in ["UserCmd1", "UserCmd2", "UserCmd3"]:
# 改成 —— 一劳永逸（分支内部本来就会校验 yaml 里有没有这条）
elif cmd.startswith("UserCmd"):
```
- 改完**必须重启该服务**才生效：`pkill -f nx_command_server2` 后重跑
  `scripts/NX_code/start_qgc_servers.sh`，或重启桌面会话。
- **实际在跑的是 `nx_command_server2.py`，它只是个 wrapper**（`from nx_command_server import ...`）
  → 补丁打在 `nx_command_server.py` 上是对的，但**重启的是 wrapper 那个进程**。
- ⚠️ `~/NX_code/` 下还有一套**旧副本**（3 月版），正在用的是
  `catkin_ws/src/task_run_scripts/scripts/NX_code/` 里那份 —— **别改错文件**。

## 7. 收尾检查清单（每次改完启动脚本都过一遍）

```bash
bash ~/xxx.sh                    # 计时：应 <5 s
bash ~/xxx.sh                    # 连按第二次：仍应 <5 s（幂等）
bash ~/xxx.sh --status           # 秒回，逐节点 OK/缺
rostopic info /waypoint_goal     # 探索模式下发布者【只有】/goal_arbiter
grep -a "watch 模式" ~/catkin_ws/src/task_run_scripts/log/arbiter.log | tail -1
grep -aE "shutdown request|same name" ~/catkin_ws/src/task_run_scripts/log/*.log   # 应为空
```
