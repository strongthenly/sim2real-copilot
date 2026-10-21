# 路径A 方案C/F（全局路径引导 + guide_path 直发 + MINCO 必经点）— 2026-08-12

SUPER 轨迹复飞"沿轨迹 + 避障"的完整演进记录。代码在 SUPER 仓库 `feat/guide-path-c` 分支（**不是** dev/4uav！），task_run_scripts/NX_code 配套在 `feat/4uav-guide`。

## 方案演进（按时间）

1. **路径A（原版）**：注入块把 guide_path 换成全局路径[最近点→终点] + MINCO 走廊优化 → 避障链路存在，但 **MINCO 斜切（不贴轨迹）**。
2. **方案C（guide_path 直发）**：getOneCommandFromTraj 里 committed_guide_path_ 有效时直接线性插值发 pos_cmd（跳过 MINCO）→ **严格循迹（误差≈0）**，但 **避障是空壳**（障碍出现时飞机仍沿旧 committed 轨迹 → 会撞）。
3. **方案F（A+C 融合，当前）**：保留 MINCO，但 attractor 用 guide_path 点 + dead_d=0.05 → MINCO 优化时被"钉"在航迹上；直发仅作 MINCO 失败的兜底。**实测仍斜切（2026-08-13 确认）**：attractor 是软约束，拉不住段内自由。
4. **方案F2（走廊小球硬约束，下一步）**：不靠 attractor 软拉，**把走廊（vPolytopes）退化成 guide_path 点上的小球（±0.05m）+ piece 数=航点数** → MINCO 优化时 points 被**硬约束**在航迹上 → 严格循迹 + 避障保留 + MINCO 作用保留。原理见下文"方案F2 详解"。

## 方案C 实现要点（SUPER，feat/guide-path-c）

- `super_planner.h`：`committed_guide_path_ / committed_guide_stamp_ / committed_guide_valid_ / committed_guide_mtx_` 成员。
- `super_planner.cpp` 注入块（generateExpTraj 开头，~630-680 行）：单向推进找最近点（`global_last_idx_`，从上次索引往后搜，防跳段）→ 取 [最近点→终点] → `sub_path.back() = gi_.goal_p`（终点统一为吸附后 goal，防 MINCO NaN）→ 生成 guide_stamp（沿航点匀速时间戳）→ 存 committed。
- `getOneCommandFromTraj`（~341 行）：方案C = 按 eval_t 在 guide_stamp 上定位 + 线性插值 → pvaj（速度=段平均，加速度=0）；方案F = 改 `bool minco_traj_ok = (total_dur>1e-6) && !traj_finish`，只有 `!minco_traj_ok` 才直发兜底。
- 可视化：自定义红色 LINE_STRIP marker（`/fsm_node/visualization/committed_path`，**frame_id 必须 "map"**）。

## 方案F 实现要点

- `exp_traj_optimizer_s4.cpp`（~461 行）：attractor 改为"guide_path 里离盒子中心最近的点"（遍历 guide_path 找 best_pt）+ `dis = std::min(dis, 0.05)`。
- `exp_traj_optimizer_s4.cpp`（~494 行）：`opt_vars.points.col(j) = opt_vars.guide_path[i]`（MINCO piece 边界=航点）。
- `super_planner.cpp`（~342 行）：直发改兜底。
- 编译：2026-08-12 19:18 fsm_node 编译成功（-j2，0 错误）。

## 关键机制认知（避免再踩）

### MINCO 斜切是架构特性，不是 bug
MINCO = "走廊内最短时间轨迹"（物理最优），只保证在走廊内，不保证贴路径。attractor（waypoint_attractor）代价**只在每段端点（is_waypoint/is_end）生效**（exp_traj_optimizer_s4.cpp ~141 行），段内自由斜切。dead_d 只能收端点，收不住段内。**corridor 加密（0.4→0.25）能改善轮廓但 0.25 会走廊崩**。

### corridor_line_max_length 必须 1.2（不能 0.4/0.25）
`seed line too long` 崩溃：corridor_generator.cpp:113 检查 `(path[first_id]-path[second_id]).norm() > seed_line_max_length_ * 1.5` 就 throw。0.4 → 阈值 0.6m，无障碍直线段 isLineFree 延伸超长 → throw → `what(): seed line too long` → fsm 崩溃。**1.2 → 阈值 1.8m 不崩**。

### 终点吸附 → MINCO NaN 链
FSM `getNearestInfCellNot`（fsm.cpp:63）吸附终点（如 0.75→0.825）→ guide_path 终点(0.75) 与 goal(0.825) 不一致 → MINCO "function value became NaN or Inf" → 终点乱转。解法：`sub_path.back() = gi_.goal_p`（终点统一为吸附后 goal）。

### 吸附偏移保护（fsm.cpp）
先存 `goal_orig = gi_.goal_p`，吸附后算 `(gi_.goal_p - goal_orig).norm()`，**>0.1m 则 ROS_WARN + return**（不设 goal → 悬停保护，不飞向错误吸附点）。

### 闭环航迹（终点=起点）不飞
闭环 goal=起点=飞机当前位置 → "close to goal"(res*3=0.45m) 触发 → NONEED → 原地不动。解法：goal 放圈上某点（如起点前方 0.5m 的轨迹点）或用"动态 goal 前移"。**方案D（改到达判定）风险大**（全局逻辑，影响所有场景）。

### 地面测试（不起飞）
- 飞机 z=0 起点在地图边界 → MINCO 失败（Omg/thr/Pos violation）。临时把 `fsm/goal_z_min/max` 改成 0.0/0.3，**飞行前必须改回 0.6/0.825**。
- backup MINCO 常失败（`Minco backup_traj opt failed`）是**备用轨迹**失败，**不影响主轨迹**（主轨迹 GenerateExpTrajectory SUCCESS 时避障绕行正常，2026-08-13 实测）。

## 编译铁律（新增）
- **`catkin_make -j2 -DROS_EDITION=ROS1`**（不是 -j4！）：Orin NX 8GB 上 -j4 编译 MINCO/lbfgs 大型模板会**内存满载 → 系统卡死 → SSH 握手超时**（22 端口通但登录超时）。-j2 约 10-15 分钟不卡死。
- 系统高负载时 plink 会超时：先 `Test-NetConnection -Port 22` 确认端口通（True=网络 OK），再等编译完成负载降下来，别反复重试。

## RViz 显示坑
- SUPER adapter（ros1_adapter.hpp）可视化 **frame_id 写死 "world"** → RViz 用 map/camera_init 帧看不到。自定义 marker 必须显式 `m.header.frame_id = "map"`。
- **话题带节点名前缀**：`/fsm_node/visualization/committed_path`（不是 /visualization/...）。查 rostopic 别漏前缀。
- top_down.rviz 是"压缩单行式"格式，`yaml.safe_load` 的 Displays=0；字符串插入要保持原缩进。改完要**重启 RViz 加载**（运行中改文件不生效）。
- 多个 RViz 并存（slam_rviz + top_down）时，确认看的是哪个窗口、哪个配置。
- **跟随无人机视角（2026-08-13 尝试后已回退）**：曾把 top_down.rviz 的 `Fixed Frame: world` 改成 `Fixed Frame: imu`（odin 发布的机载系，TF 链 odom→imu）想让视角跟随飞机。**实测结果：地图直接不显示了**（imu 帧依赖 odin 在跑 + 地图/轨迹显示项都是 world 系，切 imu 后无法显示）→ **用户要求改回俯视 → Fixed Frame 改回 world，重启后地图恢复**。**结论：保持 world 俯视，别用 imu 固定帧做跟随**（odin 链路下地图会消失）。**判地图不显示时的顺序**：① `ps aux | grep fsm_node`（real_flight 是否在跑——最常见是没先按"启动雷达"或 real_flight 被停，此时与 Fixed Frame 无关）② `grep -n "Fixed Frame"` 确认配置 ③ 改配置后**必须重启 real_flight/按钮3**（RViz 启动时才读 -d 加载的配置文件，运行中改不生效）。配置文件在 `task_run_scripts/rviz/top_down.rviz`（`grep -n "Fixed Frame"` 572 行附近）。
- **RViz 同名节点互相顶掉（2026-08-13）**：odin1_ros1.launch 的 RViz 节点 `name="rviz"` 与 real_flight.launch 的 RViz 节点 `name="rviz"` 同名 → 按按钮3 后 odin_ros.rviz 被关（日志 `Reason given for shutdown: new node registered with same name`）。修复：real_flight.launch 的 RViz 节点改名 `name="rviz_topdown"`，两界面共存（雷达视图 + 导航视图都要看）。

## 方案F2 详解：走廊小球硬约束（2026-08-13 已实施+编译成功）

**背景**：方案F 实测仍直线飞（飞机位置 x≈0.2 不变、y 直冲终点 -1，idx 从 3 跳到 144 追平）。排查确认方案F 代码已编译进二进制（`strings fsm_node | grep -c best_pt` = 1），**问题不是没编译，是 attractor 强度不够**。

### 为什么 attractor 拉不住（关键机制认知）
```cpp
// exp_traj_optimizer_s4.cpp 优化循环:
// 666行(优化前): gcopter::backwardP(points, vPolytopes, xi)  // points 由 xi 经走廊参数化
// 294行(求值时): gcopter::forwardP(xi, vPolytopes, points)   // points 从 xi 恢复
// 757行: minco.setParameters(points, times)
```
- **points 是优化变量**，被 `vPolytopes`（走廊）参数化——**走廊内任意位置都"合法"**。
- **attractor（waypoint_attractor）是软代价**，只在段端点（is_waypoint/is_end，~141 行）计入成本——段与段之间自由斜切。
- dead_d=0.05 只约束 ~11 个段端点（=盒子数），管不住中间 140 个航点。MINCO 权衡"斜切省的时间 > attractor 代价"时必然斜切。

### 方案F2 核心思路：把航迹写进**硬约束**
```
现在: 走廊 = 大盒子(0.39m) → MINCO 在盒内自由斜切
改成: 走廊 = guide_path 每个点上的小球(半径 0.05~0.1m) + piece 数 = 航点数
→ points 被走廊硬约束在航迹上(违反=不可行, 不是代价!) → 无法斜切
```
- 走廊是 MINCO 的**硬约束**（不可行域），attractor 只是"代价"（可权衡）——**硬约束才能保证贴轨迹**。
- 障碍出现时：guide_path 被 fallback A* 替换 → 绕行路径点照样生成小球 → 绕行轨迹同样硬约束 → 避障保留。
- MINCO 平滑/速度分配作用保留（SUPER 不退化）。

### 实现改动点（2026-08-13 已实施，编译成功 16:05）

**实际实施位置**：`exp_traj_optimizer_s4.cpp` `processCorridorWithGuideTraj`（~473 行，方案F attractor 块之后）。**vPolytopes 不再用大盒子 curIOB**，改为每个 piece 的 attractor（=guide_path 点，方案F 的 best_pt 逻辑已就位）上的**小立方体**（8 顶点，半宽 HF=0.1 → 总宽 0.2m）：
```cpp
const double HF = 0.1;
const Vec3f c = opt_vars.waypoint_attractor.col(i);   // = guide_path 点(方案F 已改)
curIOB.resize(3, 8);   // 8 顶点立方体(±HF 各轴组合)
opt_vars.vPolytopes.push_back(curIOB);
```
**关键认知**：piece 数仍 = 盒子数（sizeCorridor = hPolytopes.size()-1，~11 个）——**不需要 piece 数=航点数**！因为每个 piece 的 vPolytope 已经缩到 guide_path 点的小立方体，**硬约束把 piece 边界钉在航迹上，段与段之间无法斜切**（MINCO 的 points 经 backwardP/forwardP 必须落在 vPolytopes 内）。这就是与 attractor 软约束的本质区别。

编译：`catkin_make -j2 -DROS_EDITION=ROS1` → fsm_node 16:05，0 错误。
判读已编入：`strings fsm_node | grep -c "方案F2"` 或看 vPolytopes 8 顶点逻辑。

**实测结果（2026-08-13 16:15 直行 8m）**：✅ **斜切消失**（飞机 x=8.03 精确贴航迹终点 x，不再直线斜切）！但暴露**终点段问题**（见下）。障碍测试：静态障碍放直线上 → 检测 131 次 `Global path blocked`，**主轨迹 GenerateExpTrajectory SUCCESS ×122（飞机实际绕行成功，用户亲见）**；backup MINCO 全部 FAILED（=无第二保险，不影响主飞行，见下文"重大判读修正"）。若小球 0.1m 半宽导致 MINCO 失败可放宽 0.15~0.2。

### 终点段问题（斜切根治后暴露的下一道坎）— 2026-08-13 实测
**现象**：飞机到终点 x=8 后不悬停，在终点附近**来回摆**（y 从 -1.42 摆到 -0.04），日志循环 `GeneratePolytopeFromLine failed` + `generateBackupTrajectory return [FAILED]`。
**根因链**：飞机到终点 → 最近点 idx=160（最后一段）→ **guide_path 只剩 2 点（idx160→161，间隔 0.05m）** → SearchPolytopeOnPath 的 seed line 太短 → 走廊失败 → 无新轨迹 → 飞机用旧轨迹在终点附近摆。**这不是斜切，是"终点收敛"问题**（guide_path 太短时走廊/backup 都失败）。
**修复方向（已定）**：注入块取段时保证 guide_path 至少 ~1m（若 sub_path < 5 点则 nearest_idx 回退 20 个点重新取段）→ 走廊有足够 seed line。

### 终点判定（用户方案，2026-08-13 已实施+编译 16:28）
用户提出：**飞机位置距终点 0.1m 内 → 判定到达 → 悬停**（根治终点摆）。实施于 super_planner.cpp getOneCommandFromTraj（~336 行）：
```cpp
if (!traj_finish) {
    const double dist_goal = (robot_state_.p - gi_.goal_p).norm();
    if (dist_goal < 0.1) {
        traj_finish = true;   // 停止推进 → 悬停
        ROS_INFO("[SUPER] Goal reached (dist=%.3fm < 0.1m), hold at goal.", dist_goal);
    }
}
```
`robot_state_.p` 和 `gi_.goal_p` 都是类成员，函数内直接可用。编译 `-j2` → fsm_node 16:28，0 错误。

### 动态避障可行性评估（用户问"避开动态障碍物的可能性大吗"）— 2026-08-13
| 环节 | 能力 | 说明 |
|---|---|---|
| 检测动态障碍 | ✅ | 地图 10Hz 更新，人走进去→占用→blocked |
| A* 绕行搜索 | ✅ | fallback 触发（实测 131 次 blocked） |
| **绕行轨迹生成** | ❌ | **backup MINCO 优化失败（当前瓶颈）** |
| 恢复原轨迹 | ✅ | 障碍消失→恢复 |

**结论**：检测可靠，绕行被 backup MINCO 失败卡死。**要动态避障必须先修 backup MINCO 失败**（A* 绕行路径太曲折→MINCO 动力学不可行，或方案F2 小立方体对绕行路径太紧）。修通之前方案F2 不适合真机避障测试。

### ⚠️ 重大判读修正：主轨迹 vs backup 轨迹是两条独立的（2026-08-13 实测确认）
**用户观察"MINCO 失败但最后还是绕行了" → 排查后真相**：
```
同一秒内日志交替出现:
[WARN] generateBackupTrajectory return [FAILED]   ← backup(备用兜底)失败
[INFO] GenerateExpTrajectory SUCCESS              ← 主轨迹成功!
```
**关键认知**：`GenerateExpTrajectory`（主轨迹 exp_traj）和 `generateBackupTrajectory`（备用兜底）是**两条独立轨迹**！
- **飞机执行的是主轨迹**（exp_traj，`cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info)`）：实测 **SUCCESS ×122**
- backup 失败（×233）只是**没有第二道保险**，不影响主飞行
- **因此静态避障实际是成功的**：障碍→blocked→fallback A*→**主轨迹沿 A* 绕行路径生成成功→飞机绕行**（用户亲见绕行）
- **之前"避障失败/绕行无"的判断是看错日志**（把 backup FAILED 当成了主轨迹失败）——判读避障只看 `GenerateExpTrajectory SUCCESS` 数量，backup 失败单独看
- **动态避障可行性因此上调**：主轨迹绕行链路是通的，动态障碍（人走动）检测→A*→主轨迹绕行应同样有效

### ⚠️ 动态避障实测：有绕行趋势但暂停原地（方案F2 小立方体对绕行路径太紧）— 2026-08-13 17:39
**现象**：面对动态障碍（人），飞机先有绕行趋势（前进+侧移），随后**暂停原地不动**。日志：
```
Global path blocked ×96（检测 ✅）
主轨迹 GenerateExpTrajectory SUCCESS ×1 / FAILED ×0（开始成功一次）
backup FAILED ×114 / Opt failed(Omg or thr) ×456
[CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: 0.2046
SUPER command timeout. Switching to position hold.  (timeout_threshold: 0.5)
```
**根因链**：障碍 → blocked → fallback A* 绕行路径 → **方案F2 小立方体（0.2m）沿绕行路径生成，但绕行路径本身贴障碍（min dis 0.204）→ 障碍距离约束 + 小立方体约束冲突 → MINCO 不可行（×456）** → 无新轨迹 → bridge 0.5s 超时 → `position hold`（悬停，安全但不绕）。
**修复方向（方案 A，2026-08-13 已实施+编译 17:53，见下方"方案A 详解"）**：**fallback（绕行）时 vPolytopes 用大盒子（0.39m）给 MINCO 空间，正常循迹时用小立方体**——在 super_planner 的 fallback 分支绕过方案F2 的小立方体修改（`global_skip_this_round_`/A* 路径分支里恢复大盒子 vPolytopes）。备选：缩小 CIRI 障碍距离要求（0.2→0.1，更贴障碍，更危险）。**⚠️ 17:58 实测修正：方案A 只让主轨迹绕行（飞机有侧移趋势），backup 仍全失败（不受 corridor_small_ 管）——真正根因是 A* 路径贴障碍（CIRI min dis 0.04）→ 走廊凸分解失败，已改为 `rog_map inflation_step 2→3`（0.6m 膨胀）修复，见下"绕行暂停真正根因"节**。

### 方案A 详解：corridor_small_ 标志（循迹小立方体 / 绕行大盒子，2026-08-13 已实施+编译成功 17:53）
**改动 3 处**：
1. `exp_traj_optimizer_s4.h`：成员 `bool corridor_small_{true};` + **必须放 public 区**的 setter `setCorridorSmall(bool v)` / `corridorSmall()`。
2. `exp_traj_optimizer_s4.cpp` `processCorridorWithGuideTraj`（~473 行）：`if (corridor_small_)` → 方案F2 小立方体（HF=0.1 八顶点）；`else` → 原始大盒子 `opt_vars.vPolytopes.push_back(curIOB)`。
3. `super_planner.cpp`：正常循迹分支（`global_used_prev_ = true;` 之后，~706 行）`setCorridorSmall(true)`；fallback 分支（`Global path blocked` warn 后，~853 行）`setCorridorSmall(false)`。

**⚠️ 编译坑（实测踩到）**：setter 若写在类内 private 区（默认，`public:` 标记在 ~325 行 `typedef std::shared_ptr<ExpTrajOpt> Ptr` 附近），super_planner.cpp 调用报 `error: 'void traj_opt::ExpTrajOpt::setCorridorSmall(bool)' is private within this context`。**setter 必须放 public 区**；且用 python 替换时注意 old 文本唯一性（`public:` 在 325 行不是 329 行，替换失败会静默删掉 setter——改完 `grep -n setCorridorSmall` 确认两处都在：public 区声明 + 两个调用点）。

**逻辑链**：正常循迹 corridor_small_=true → 小立方体硬约束贴航迹（不斜切）✅ → 障碍出现 fallback → false → 大盒子给 MINCO 绕行空间（不再 Opt failed×456 / 不暂停原地）✅ → 障碍消失恢复 true ✅。
**重要认知**：`SUPER command timeout → position hold` 是 bridge 的安全兜底（`timeout_threshold: 0.5`，super_to_px4_bridge 参数）——**MINCO 失败时飞机不会乱飞，只会停**。这也意味着"绕行失败"的表现是"悬停"而非"撞上"。

### 方案A 实测边界：corridor_small_ 只影响主轨迹，backup 不受管（2026-08-13 17:58）
方案A 编译后实测（直行 8m + 动态障碍）：飞机**有绕行趋势**（x 4.82→7.00 持续前进 + y 从 0 侧移到 -1.5，确实在绕开障碍），但**最终仍 timeout 悬停**。日志：
```
主轨迹 GenerateExpTrajectory SUCCESS ×1 / FAILED ×4
backup FAILED ×117 / Opt failed(Omg or thr) ×476
[CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: 0.044~0.119
GeneratePolytopeFromLine failed ×N
SUPER command timeout → position hold
```
**关键边界认知**：`corridor_small_`（方案A）**只影响主轨迹 exp_traj 的 processCorridorWithGuideTraj**——**backup 轨迹走自己的独立链路**（generateBackupTrajectory → `GeneratePolytopeFromLine` → BackupTrajOpt），**不受 corridor_small_ 影响**。所以 backup 失败（Opt failed ×476）方案A 修不了，必须另修（见下）。判读避障时：飞机 y 侧移 = A* 绕行路径在生效（绕行趋势 ✅），走廊生成失败才是卡点。

### 绕行暂停真正根因：A* 路径贴障碍 → 走廊 CIRI 凸分解失败（inflation_step 修复，2026-08-13 已实施）
**根因链**：障碍 → fallback A* → **A* 贪心最短路径贴着障碍边缘走**（`[CIRI] min dis to obstacle: 0.044~0.119m`）→ backup 的 `GeneratePolytopeFromLine`（corridor_generator.cpp:304）用 `getSeedBBox`（seed ± bound_dis 0.39m 盒子）boxSearch 收集障碍 → **CIRI 凸分解要求障碍距离 ≥ 阈值，贴障碍 0.04m 不满足 → FAILED** → 无 backup 轨迹 → bridge 0.5s 超时（`timeout_threshold: 0.5`）→ position hold（悬停，安全但不绕）。
**日志判读**：`[CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: X` = **绕行路径贴障碍 → 走廊凸分解失败**（不是 MINCO 参数问题，调 max_omg 无用）。
**修复（纯 yaml，无需编译，2026-08-13 已实施未验证）**：
```yaml
rog_map:
  inflation_step: 3   # 2→3: A* 障碍膨胀 0.4m→0.6m → 绕行路径离障碍 ≥0.6m → 走廊凸分解有余量
```
**原理**：A* 搜索用 rog_map 膨胀层判障碍（`inflation_resolution 0.2 × inflation_step` = 膨胀半径），膨胀越大搜出的路径离障碍越远 → GeneratePolytopeFromLine 的 CIRI 分解有余量 → backup/绕行轨迹能生成。0.6m 仍失败可继续加大，或放宽 corridor 生成的障碍距离要求（更危险，贴障碍飞）。

### backup 轨迹失败的代码根因（super_planner.cpp generateBackupTrajectory ~998 行）
3 个失败点（按优先级）：
1. **起点被占用**：`map_ptr_->getNearestCellNot(OCCUPIED, shifted_robot_p, ..., 3.0)` 3m 内无空闲格 → `Local start point is deeply occupied` → FAILED（障碍就在飞机旁时最常见）
2. **走廊生成失败**：`cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)` 失败 → `GeneratePolytopeFromLine failed` → FAILED
3. **MINCO 优化失败**：`Opt failed, Omg or thr or Pos violation`（A* 绕行路径急转弯→动力学不可行）

**backup 优化方案C（2026-08-13 已实施，纯 yaml 参数改动，无需编译）**：放宽物理约束边界——
```yaml
traj_opt:
  boundary:
    max_omg: 6.0        # 2.5 → 6.0 (角速度放宽 2.4 倍)
    max_acc_thr: 25     # 20 → 25 (推力放宽)
```
**原理**：backup 失败日志 `Opt failed, Omg or thr or Pos violation` 的直接原因是 **A* 绕行路径急转弯 → 角速度超 2.5 rad/s → 违反 Omg 约束**。`max_omg/max_acc_thr` 在 `traj_opt/boundary` 段（`Config.h:102-106` 从 `traj_opt/boundary/*` 加载），**主轨迹与 backup 共用**（backup 的 `traj_opt/backup_traj` 段只有 penna_* 惩罚系数，无独立物理约束）。放宽只是"允许更大"不强制，**主轨迹不受损**（正常仍按 max_vel 0.3 慢飞）。改完重启按钮3 生效。**判读**：`grep generateBackupTrajectory log/real_flight.log` 应从 FAILED 变 SUCCESS。
**更根本的 backup 优化方向（未实施，方案 A）**：backup 直接保存 A* 路径点直发（不依赖 MINCO/走廊，两者是失败主因；类似方案C 思路但只用于紧急兜底）→ 主轨迹失败时飞机仍能沿 A* 路径绕行而不是悬停。备选：起点搜索半径 3.0→5.0m（缓解原因1）。

**备选思路（未实施）**：若小立方体+盒子数约束仍不够密，可下采样 guide_path 到 ~40 关键点并让 piece 数=关键点数（需同步 opt_vars.piece_num 与 hPolytopes 构造），比 153 piece 更稳妥。

**判读方案F 是否生效**（不必看时间戳，strings 二进制）：`strings fsm_node | grep -c "minco_traj_ok"`（=1 直发兜底已编入）、`grep -c "best_pt"`（=1 attractor 改动已编入）。**若都已编入仍斜切 → 是 attractor 软约束本质问题，直接上方案F2，别再调 dead_d/corridor 参数**。

## 测试航迹生成（2026-08-13：2m×2m C 形 / 直行 8m）

用户要求"前进2m右移2m后退2m"（比 1.6m×1m 大）——生成 C 形航迹的模板（Python，圆角 R、z=0.75、5cm 间距）：
- 形状：(0,0)→前进→(2,0)→右移→(2,-2)→后退→(0,-2)，终点 (0,-2)
- 直线段 5cm 步进 + 圆弧（每 2° 采样，中心在拐角）；154 点，总路程 7.4m
- **避障测试用直行 8m 最简单**：(0,0)→(8,0)，162 点（5cm 间距），endpoint=(8,0,0.75)
- **直行 12.5m（2026-08-13 最终测试用，当前 guidepoints_new.txt）**：(0,0)→(12.5,0)，251 点（5cm 间距），endpoint=(12.5,0,0.75)——直线越长给避障测试的缓冲区越大，改长度只需改循环上限和 endpoint，格式不变
- endpoint.txt 写 `0.000000 -2.000000 0.750000 0.1`（4 列空格格式含 switch_dis！）

### ⚠️ 圆弧参数错误 → 转弯"跳跃"不平滑（2026-08-13 实测坑）
首版圆弧用了错误角度（`add_arc(2,0,-90,0)` = 从 (2,-0.3) 到 (2.3,0)，位置完全错），圆弧与直线之间出现 **0.44m 缝隙** → RViz 里转弯断连、飞机转弯跳变。**正确圆弧角度**（y 向下坐标系，C 形右转）：
```
转弯1 (前进→右移, 圆心(2,0)): 180°→270°   # (2-R,0) 弯到 (2,-R)
转弯2 (右移→后退, 圆心(2,-2)): 90°→180°   # (2,-2+R) 弯到 (2-R,-2)
```
`add_arc` 必须**包含起点**（`for i in range(n+1)` 从 i=0 开始，不能跳端点）——直线终点 = 圆弧起点要严丝合缝。圆角 R=0.5 比 0.3 更平滑。

### ⚠️ 航点文件必须是 CSV 格式（global_path_publisher 只认这个）
`global_path_publisher.py` 用 `csv.reader` 解析（**逗号分隔 + 表头 `x,y,z,qx,qy,qz,qw`**）。生成脚本若写成**空格分隔**（`f"{x} {y} {z}"`）→ 每行是一个整体元素 → `float(row[0])` ValueError → 全部跳过 → 日志 `文件为空: ...` + `无有效航点, 退出` → 绿线不显示。**生成航点必须写 CSV 格式**（旧备份文件也是逗号+表头）。

### 判读 global_path_publisher 是否在发（2026-08-13 实测两个误判）
1. **`rostopic hz /global_path` 显示 `no new messages` ≠ 没在发**：publisher 用 `latch=True` + 1Hz，**latch 话题只在新订阅者出现时重发**，hz 工具统计不到 → 用 `timeout 5 rostopic echo /global_path -n 1 --noarr | grep frame_id`（能收到 header 就是正常在发）。
2. **nohup/setsid 后台启动的 publisher 日志 0B ≠ 启动失败**：Python stdout 缓冲，`> log 2>&1` 重定向后日志迟迟不刷。判据用进程存在（`ps aux | grep global_path_publisher`）+ `cat /proc/<pid>/wchan`（`do_select` = 正常在 select 等发布循环）+ `rostopic echo` 收到数据。前台 `timeout 5 rosrun ...` 能打印 `发布 162 个航点` 即脚本本身没问题。

### 重叠检查：用线段交叉法，别用点距法
点距法（非相邻点距离 < 阈值）在圆弧采样密时会**大量误报**（圆弧点距 0.026m，隔 3 个点 0.08m < 0.25m 阈值 → 误判 1226 对"重叠"）。**正确做法：检查非相邻线段是否真正交叉**（`seg_cross` 四点叉积符号判断）。C 形等开放路径真实自交 = 0 即形状干净。


- **SUPER 仓库的路径A/C/F 改动提交到 `feat/guide-path-c` 新分支**，dev/4uav 保留不动（用户会指出"SUPER 那个不是分支，应该创建新分支"）。
- task_run_scripts / NX_code 提交到 `feat/4uav-guide`。
- 已提交：SUPER feat/guide-path-c（0ae8f75 方案C、8ab2f2c 方案F）；task_run_scripts feat/4uav-guide（402a86e）。
- 文档 SUPER.md（Drone-Testing-Documentation 仓库 SUPER/ 目录）同步方案演进。

## 测试判读标准（方案F 避障）
```
障碍出现 → 日志 "Global path blocked"（检测到）
→ "GenerateExpTrajectory SUCCESS"（MINCO 绕行轨迹生成=避障通）
❌ 若 "OptimizationExpTrajInPolytopes for new path failed" = MINCO 问题待查
```
- 速度参数已调低（max_vel 0.3 / max_acc 0.6）仍可能 MINCO 失败 → 怀疑坐标系错位（雷达别处启动，地图原点≠测试点）或起点占用。
- 测试前确保：测试点重启雷达（坐标系正确）、人退开 2m+。

## 新机载电脑部署（拉取分支 + 编译）— 2026-08-13
新机器 fouruav@<NET_WLAN_B>.31（hostname=fouruav）部署方案F 全流程，含踩坑。

### 拉取流程（干净切换，不覆盖本地修改）
```bash
# ① 先查状态: git branch --show-current + git status --short（确认有没有本地修改）
# ② SUPER 有本地修改(real_flight.yaml waypoint_goal 参数)时:
git stash push -m "newmachine_local_yaml"     # 暂存(不能直接 checkout, 会报"本地修改将被覆盖")
git checkout feat/guide-path-c && git pull origin feat/guide-path-c
git stash pop                                   # 恢复本地修改(自动合并, 冲突则以远程为准)
# ③ task_run_scripts 切 feat/4uav-guide; NX_code 子模块同样切 feat/4uav-guide
```
- 注意：task_run_scripts 的 `scripts/NX_code` 子模块指针切分支时显示 `M`——子模块单独 checkout 即可。
- fetch 子模块偶发 `fatal: 远程错误: upload-pack: not our ref` = 子模块指针与远程不一致，单独在子模块目录 fetch/checkout 解决。

### livox_ros_driver2 DoubleEcho 编译错（新机器常见）
```
错误: pub_handler.cpp:329 'kLivoxLidarDoubleEchoData' was not declared in this scope
      pub_handler.cpp:463 'LivoxLidarDoubleEchoRawPoint' was not declared
```
**根因**：pub_handler.cpp 是**新版驱动源码**（含双回波 DoubleEcho 处理），但**系统安装的 Livox SDK 头是旧版**（/usr/local/include/livox_lidar_api.h 无 DoubleEcho 定义）→ 版本不匹配。
**诊断**：
```bash
grep -c "DoubleEcho" /usr/local/include/livox_lidar_api.h    # 0=旧SDK
grep -c "DoubleEcho" /usr/local/include/livox_lidar_def.h    # 0=旧SDK
grep -rln "kLivoxLidarDoubleEchoData" ~/catkin_ws/src/livox_ros_driver2/  # 只有 pub_handler.cpp 引用=头缺失
```
**注意**：CMake 明确 "Defining ROS1 / Build system: ROS1" 仍报错——**不是 ROS_EDITION 双版本坑**，是 SDK 头版本问题。
**解法（已实测，推荐回退法）**：用 git 历史回退 pub_handler 到"引入 DoubleEcho 之前"的提交，匹配系统旧 SDK：
```bash
cd ~/catkin_ws/src/livox_ros_driver2
git log -S "DoubleEcho" --oneline -- src/comm/pub_handler.cpp   # 找引入提交(如 4a1def9)
git log --oneline -10                                           # 确认引入提交的前一个(如 13eb05e "support Mid-360s")
# 验证目标提交无 DoubleEcho 且类型与系统 SDK 匹配:
git show 13eb05e:src/comm/pub_handler.cpp | grep -c "DoubleEcho"           # 应为 0
git show 13eb05e:src/comm/pub_handler.cpp | grep -oE "kLivoxLidar[A-Za-z]+" | sort -u
grep -oE "kLivoxLidar[A-Za-z]+" /usr/local/include/livox_lidar_def.h | sort -u  # 系统SDK支持的类型应覆盖
# 回退(先备份):
cp src/comm/pub_handler.cpp src/comm/pub_handler.cpp.bak_avia2
git show 13eb05e:src/comm/pub_handler.cpp > src/comm/pub_handler.cpp
git show 13eb05e:src/comm/pub_handler.h   > src/comm/pub_handler.h
# 同时检查其他文件是否也用 DoubleEcho: grep -rn "DoubleEcho" src/ | grep -v .bak
```
**判据**：驱动源码 git log 有 `Support Avia2 Lidar` / `support Mid-360s` 提交 = 新源码；SDK 头无对应定义 = 旧 SDK。回退目标是"含 Mid-360S 支持但无 DoubleEcho"的提交（13eb05e 正合适）。

**编译产物验证技巧**：fsm_node 时间戳可能滞后/误导（日志停在 "make -j2" 但实际已编译完），**用 strings 检查二进制是否含新代码标记**更可靠：
```bash
strings ~/catkin_ws/devel/lib/super_planner/fsm_node | grep -c "minco_traj_ok"   # 1=方案F已编入
strings ~/catkin_ws/devel/lib/super_planner/fsm_node | grep -c "committed_path"  # 4=方案C/F可视化
```

## odin 传感器切换（导航复用）— 2026-08-13

传感器从 MID360 换成 odin1 时，导航功能**只需改传感器接入层，SUPER/方案F 代码不用动**。原理：SUPER 只依赖点云话题 + 位姿，谁提供的都行。

### 两套链路对比（关键认知）

| 项 | MID360 | odin1 |
|---|---|---|
| 点云 | FAST-LIVO2 输出 `/cloud_registered` | odin 自带 `/odin1/cloud_slam`（也有 /odin1/cloud_raw） |
| 定位 | FAST-LIVO2（需雷达+IMU） | **odin 自带**（/odin1/odometry_highfreq + /odin1/odometry） |
| 位姿→PX4 | FAST-LIVO2→MAVROS | **odom_to_mavros.py**（odin_ros_driver 自带，现成） |
| 整链路启动 | 按钮"启动雷达"→ fastlivo2 | `roslaunch task_run_scripts start_all_odin_apm.launch`（已存在：odin1_ros1.launch + apm.launch + odom_to_mavros.py + mavcmd 初始命令） |

### 需要改的配置（3 处）
1. `task_run_scripts/config/real_flight.yaml` ~160 行：`cloud_topic: "/cloud_registered"` → **`"/odin1/cloud_slam"`**（SUPER 地图输入）
2. 启动链路：用 start_all_odin_apm.launch（**不跑 FAST-LIVO2**）
3. 确认 odom_to_mavros.py 发布到 /mavros/vision_pose/pose（旧坑：odin_to_mavros 缓存位姿 30Hz 死发不查新鲜度）

**不用改**：SUPER 导航代码、global_path_publisher、航迹文件、按钮3、bridge→PX4 链路。

### "启动雷达"按钮 ≠ 一定启动 odin（常见坑）
- 按了"启动雷达"按钮但 RViz 无点云：**先查按钮实际执行什么**。
- **两个 nx_command_server 版本并存（新机器重点）**：
  - 运行中的进程 = `catkin_ws/src/task_run_scripts/scripts/NX_code/nx_command_server2.py`（它 `from nx_command_server import start_command_server`，import 同目录的 catkin_ws 版）
  - **catkin_ws 版（8月13 新版）用 `Radar_Launcher.yaml` profile 机制**：`RADAR_LAUNCHER_YAML = 脚本同目录/Radar_Launcher.yaml`，按钮"启动雷达"读该文件的 `active_profile` 决定启动 odin 还是 mid360（优先级：环境变量 `LIYUMEN_RADAR_PROFILE` > yaml active_profile > 默认 "odin"）。odin profile 里有 launch_with_cloud（recorddata=1）/launch_without_cloud 两条启动命令 + cleanup pkill 规则。
  - 旧版 `~/NX_code/nx_command_server.py`（3月17）才是 `cmd == "Odin"/"Odin-MindCloud"/"KillLiDAR"` 分发 + 用 `~/odin_ws/devel/setup.bash` 启动——**没在跑**，别被它误导。
- **改 Radar_Launcher.yaml 后必须重启 nx_command_server**（配置在 server 启动时加载，运行中改不生效）：`pkill -f nx_command_server2.py; sleep 2; nohup python3 -u <路径>/nx_command_server2.py > ~/.cache/liyumen/log/nx_srv.log 2>&1 &`。
- **按钮没触发 odin 的判据**：`~/.cache/liyumen/log/` 空（无 odin_launch.log）= 按钮根本没执行 odin 启动（配置对但 server 旧/按钮 cmd 不匹配）。
- 平板/投影的按钮列表可能来自**旧机器缓存**——新机器 yaml 里可能根本没有"启动雷达"按钮（只有 SUPER航点任务/停止SUPER/按钮3 三个）。核对 `rostopic list | grep odin` 是否有话题 + `ps aux | grep odin` 是否有进程，先确认驱动真的在跑。
- 排查顺序：`rostopic list`（话题存在？）→ `rostopic hz`（有数据？）→ `ps`（进程在？）→ 按钮定义（启动的是什么？）。

## 按钮命令 `&&` 短路坑（UserCmd2 停止不生效）— 2026-08-13

**现象**：按按钮2（停止SUPER：`rosservice call set_mode POSCTL && sleep 1 && pkill ...`）后 fsm_node/waypoint_mission 还活着。

**根因**：命令链用 `&&` 串联。MAVROS 没跑时 `/mavros/set_mode` 服务不存在 → `rosservice call` 报 `ERROR: Service [/mavros/set_mode] is not available` → **`&&` 短路 → 后面的 pkill 全部跳过**。日志能看到 `收到 QGC 指令: UserCmd2` + `ERROR: Service ... not available`，但杀进程从未执行。

**修复**：把安全步骤与清理步骤解耦——`&&` 改 `;`（无论切定点是否成功都继续杀进程）：
```yaml
# UserCmd2 command 改为:
rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}' ; sleep 1 ; pkill -SIGINT -f 'roslaunch task_run_scripts real_flight.launch' ; ...
```
注意：切定点仍是真机飞行时的安全前置（先稳住再杀），只是"尝试失败也要继续清理"。

**判读**：server 日志 `收到 QGC 指令: <cmd>` 后紧跟 `ERROR: Service ...` = 命令链第一环失败被短路；`ps aux | grep fsm_node` 仍存活 = 清理没执行。

## odin 多进程抢 USB（LIBUSB_ERROR_BUSY → 无点云）— 2026-08-13

**现象**：`rostopic list` 有 /odin1/cloud_raw 等话题，但 `rostopic hz` 全部 `no new messages`，RViz 无点云/图像。

**根因**：**两个 host_sdk_sample 进程同时抢同一个 USB 设备**（如：我 SSH 手动启动了一次 + 你按投影"启动雷达"又启动一次）。第二个进程日志：
```
[ERROR][lib_usb.cpp:open_device:759]: Failed to claim interface 0: LIBUSB_ERROR_BUSY
[ERROR][lib_usb.cpp:event_monitor_routine:163]: detected device 2207:0019 attach, but open fail.
```
话题是旧进程残留注册的（存在但无数据），新进程拿不到 USB → 整个链路无数据。

**诊断**：
```bash
ps aux | grep host_sdk_sample | grep -v grep     # 期望只有 1 个; 多个=抢USB
# 确认 USB 设备在: lsusb | grep 2207   (2207:0019 = odin)
# 日志是二进制(含\0)时用 strings 提取: strings odin_launch.log | grep -iE "claim|busy"
```

**修复**：杀掉**所有** odin 进程 → **只启动一次**（手动或按钮二选一，不要都做）：
```bash
# 用 PID 杀(不要 pkill -f, 会匹配 plink 自身):
pgrep -f host_sdk_sample | head -5   # 拿 PID
kill -9 <pids>
# 然后只启动一次:
roslaunch odin_ros_driver odin1_ros1.launch    # 或只按一次投影"启动雷达"
```
**启动后验证**：`ps aux | grep host_sdk_sample | wc -l` = 1，`rostopic hz /odin1/cloud_raw` ≈ 10Hz。

## 投影三服务架构 + 屏幕流被杀导致投影消失 — 2026-08-13

**现象**：投影界面"没有了"，但 nx_command_server 进程和 6004/6003 端口都正常。

**架构（3 个服务，缺一不可）**：
| 服务 | 进程 | 作用 |
|---|---|---|
| 触控 UI | `nx_touch_server2.py`（端口 6002） | 投影按钮界面 |
| 指令服务 | `nx_command_server2.py`（UDP 6003 + HTTP 6004） | 接收 QGC 指令执行 |
| **屏幕流** | `rviz_full_stream2.py`（端口 6000） | **截图推流给遥控器/投影（关键！）** |

**根因**：误杀 `rviz_full_stream2.py`（如清理 rviz 时 `pkill -f rviz` 连带杀掉）→ 屏幕流断 → 投影黑屏。触控/指令服务还活着，但画面没了。

**修复**（重启屏幕流）：
```bash
export DISPLAY=:0
source /opt/ros/noetic/setup.bash && source /home/fouruav/catkin_ws/devel/setup.bash
nohup python3 -u ~/catkin_ws/src/task_run_scripts/scripts/NX_code/rviz_full_stream2.py > ~/.cache/liyumen/log/full_stream.log 2>&1 &
# 日志确认: "看门狗守护神已启动" + "截图线程启动 monitors[1] 1920x1080"
```
**教训**：`pkill -f rviz` / `pkill -f stream` 会连屏幕流一起杀——清理 rviz 用 `pgrep -f 'rviz -d'`（只匹配 RViz 显示进程），别用宽泛的 `rviz`。

## plink + pkill -f 匹配自身（exit 128）

**现象**：用 plink 执行 `pkill -f nx_command_server2.py` 时退出码 128（`Unable to open connection` 或命令没跑完），因为 **pkill -f 的模式匹配到了 plink 命令行本身**（命令行里包含该字符串）。

**修复**：杀进程用**精确 PID**（先 `ps aux | grep <name> | grep -v grep` 拿 PID 再 `kill <pid>`），或接受 exit 128 后单独验证目标进程已死（`ps aux | grep <name> | grep -v grep` 空=已杀），再起新进程。重启 nx_command_server 的完整顺序：kill 旧 → 验证空 → `nohup python3 -u <path>/nx_command_server2.py > ~/.cache/liyumen/log/nx_srv.log 2>&1 &` → `tail` 日志确认 `雷达配置已加载: profile='odin'`。

## odin 多进程抢 USB 的根本修复：cleanup 配置必须覆盖 host_sdk_sample — 2026-08-13

**用户疑问**："我每次都按关闭雷达，为什么还会发生多个进程抢设备？"

**根因（两阶段清理都有漏洞）**：
1. **第一阶段进程组追踪**（`_kill_tracked_radar_groups`）只杀 server 自己记录的进程组——**手动/SSH 启动的 odin 不在记录里 → 杀不到**。
2. **第二阶段 pkill 回退**（Radar_Launcher.yaml odin cleanup 的 pkill_patterns）原本只有 `px4_and_others.launch` / `odin1_ros1.launch` / `roslaunch mavros`——**全是 roslaunch 外壳名**！真正占 USB 的 `host_sdk_sample` 是 roslaunch 的子进程，**roslaunch 被杀/退出后 host_sdk_sample 变孤儿（PPID=1）继续占 USB，pkill 碰不到** → 下次启动新 odin 抢 USB → LIBUSB_ERROR_BUSY。

**修复（Radar_Launcher.yaml 的 odin cleanup）**：pkill_patterns 加实际驱动进程名：
```yaml
    cleanup:
      pkill_patterns:
        - "px4_and_others.launch"
        - "odin1_ros1.launch"
        - "host_sdk_sample"          # ← 关键! 实际占USB的驱动进程
        - "odom_to_mavros"           # ← 2026-08-13 补: 防止桥接进程残留打架
        - "cloud_reprojection_node"
        - "image_overlay_node"
        - "pcd2depth_node"
        - "roslaunch mavros"
```
改完**重启 nx_command_server**（配置启动时加载）。以后按"关闭雷达"能真正杀干净 host_sdk_sample。

## odin 启动后 PX4 NOT READY：缺 odom_to_mavros 桥 — 2026-08-13

**现象**：odin 驱动起来了、点云正常（/odin1/cloud_raw ~10Hz），但 QGC/遥控器起飞状态仍是 NOT READY。

**排查链（按序）**：
```bash
ps aux | grep odom_to_mavros                 # ① 桥接进程在吗?
rostopic hz /mavros/vision_pose/pose         # ② PX4收到视觉位姿吗?
rostopic hz /odin1/odometry                  # ③ odin 定位数据源正常吗?
rostopic echo /mavros/estimator_status -n 1  # ④ EKF 融合状态
```
**根因**：**odom_to_mavros.py 没启动** → /mavros/vision_pose/pose 无数据 → EKF `pos_horiz_abs=False / velocity_horiz=False` → PX4 无全局位置 → NOT READY。**注意：Radar_Launcher.yaml 的 odin profile 启动命令只有 `odin1_ros1.launch + px4_and_others.launch`，不含 odom_to_mavros！**（start_all_odin_apm.launch 里有，但按钮链路没有——这是个配置缺口）

**修复**（启动桥接）：
```bash
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
nohup python3 ~/catkin_ws/src/task_run_scripts/scripts/NX_code/odom_to_mavros.py > ~/.cache/liyumen/log/odom2mav.log 2>&1 &
# 验证: rostopic hz /mavros/vision_pose/pose ≈ 100Hz(1-11ms)
```
**EKF 判读**（启动后等几十秒）：
```
✅ velocity_horiz=True / pos_horiz_rel=True / pos_vert_abs=True = 视觉融合已生效
⚠️ pos_horiz_abs=False = 无 GPS/绝对位置源，但 PX4 通常允许用相对位置+视觉解锁（以 QGC 提示为准）
```
**参数确认**（PX4 1.17）：`EKF2_EV_CTRL=11`（视觉位置+速度融合，AID_MASK 已废弃）、`CP_DIST=0`（避障关，不会 hold 锁死）。查参数：`rosservice call /mavros/param/get "param_id: 'EKF2_EV_CTRL'"`。

### 一键 Ready to Fly 的根治：把 odom_to_mavros 加进 Radar_Launcher.yaml 启动命令

odom_to_mavros 手动启动只是临时解。**根治 = 改 Radar_Launcher.yaml 两个 command**（launch_with_cloud + launch_without_cloud），在 px4_and_others 之后追加：
```yaml
        roslaunch task_run_scripts px4_and_others.launch
        > /home/fouruav/.cache/liyumen/log/px4_others_launch.log 2>&1 &
        sleep 12 &&
        python3 /home/fouruav/catkin_ws/src/task_run_scripts/scripts/NX_code/odom_to_mavros.py
        > /home/fouruav/.cache/liyumen/log/odom2mav.log 2>&1
```
（`sleep 12` 等 odin 驱动起来再桥接；两处 command 都要改，改完重启 nx_command_server）。以后按"启动雷达" = 雷达驱动 + MAVROS + 视觉桥全自动 → EKF 融合 → POSCTL Ready to Fly。

### READY→NOT READY（反向坑：多个 odom_to_mavros 打架）— 2026-08-13 实测
**症状**：启动雷达后先是 READY，过一会变 NOT READY（EKF pos_horiz_rel/velocity_horiz 全 False）。
**根因**：多个 odom_to_mavros 实例同时发 /mavros/vision_pose/pose → 频率飙升（正常 ~10Hz，打架时**数千 Hz**）→ PX4 EKF 判定视觉数据不可信 → 融合丢失 → NOT READY。常见来源：手动起过 odom_to_mavros + 按钮又自动起（按钮已集成后**勿再手动起**），或重复按"启动雷达"。
**判读**：`rostopic hz /mavros/vision_pose/pose` 数千 Hz + `ps aux | grep -c odom_to_mavros` >1。
**修复**：杀光全部 odom_to_mavros + host_sdk_sample（PID 精确杀），**只按一次"启动雷达"**；cleanup 的 pkill_patterns 已加 `odom_to_mavros`（"关闭雷达"能一起清）。
**铁律**：按钮已自动带 odom_to_mavros，**绝不要手动再起**（否则必然打架）。
**启动源去重（2026-08-13 排查确认）**：odom_to_mavros 共有 3 个潜在启动源——① `px4_and_others.launch:8` 的 odom_to_mavros_node ② `Radar_Launcher.yaml` 启动命令里的 python3 ③ `start_all_odin_apm.launch:6`。**关键事实：px4_and_others.launch 里的 odom_to_mavros_node 是 `-->` 注释掉的（不生效）**，所以真正起作用的只有 Radar_Launcher.yaml 里的 python3 启动——**保持它作为唯一启动源**，不要再往 px4_and_others.launch 加（否则又双份）。判断是否重复：`ps aux | grep -c odom_to_mavros` 应为 1。

**但唯一启动源仍不够（2026-08-13 实测）**：一次按钮"启动雷达"就能产生 **4 个 odom_to_mavros**（bash -c 命令链被多次 fork，`ps -eo pid,ppid,cmd | grep odom_to_mavros` 显示 140115(bash)→140116/140117(同一条完整命令链的 bash)+140436(实际 python3)）。**最终兜底 = 给 odom_to_mavros.py 加单实例锁**。

### odom_to_mavros 单实例锁（pid 文件法，2026-08-13 已实施+验证）
**坑：用 rospy master 查节点名做锁会失败**——`rospy.init_node('odom_to_vision_pose', anonymous=True)` 生成带 PID 后缀的节点名（`odom_to_vision_pose_142035_...`），且两个进程同时启动时都还没注册，getSystemState 查不到对方 → 锁不住。**正确做法：pid 文件锁**（插在 import 区后、rospy.init 前）：
```python
_LOCK = '/tmp/odom_to_mavros.lock'
try:
    if os.path.exists(_LOCK):
        with open(_LOCK) as _lf:
            _old_pid = int(_lf.read().strip() or 0)
        try:
            os.kill(_old_pid, 0)  # 进程存在?
            print(f"[odom_to_mavros] 已有实例 PID={_old_pid} 在运行, 本实例退出", flush=True)
            sys.exit(0)
        except (ProcessLookupError, ValueError):
            pass  # 旧进程已死, 继续
    with open(_LOCK, 'w') as _lf:
        _lf.write(str(os.getpid()))
except Exception as _e:
    print(f"[odom_to_mavros] 锁检查异常({_e}), 继续运行", flush=True)
```
**验证**：起第二个实例 → 日志 `已有实例 PID=xxx 在运行, 本实例退出` + `ps aux | grep odom_to_mavros.py | wc -l` = 1。改完无需编译（python 脚本）。**注意**：kill 旧实例时要把 `/tmp/odom_to_mavros.lock` 一起 `rm`，否则旧锁文件残留会误判。**清理重复进程用 `ps aux | grep <name> | grep -v grep | awk '{print $2}'` 拿 PID 再 `kill -9`，别用 pkill -f**（匹配 plink 自身 → exit 128）。

### ⚠️ 进程计数陷阱：grep 会把 bash -c 包装误算成目标进程（2026-08-13 实测）
`ps aux | grep odom_to_mavros` 显示 4 个，但其中 3 个是 `bash -c "source ... && python3 odom_to_mavros.py ..."` 的**命令链包装**（命令行包含该字符串被 grep 命中），真正跑的是 1 个 python3。**精确计数用 `ps aux | grep "python3.*odom_to_mavros" | grep -v grep`**；或 `cat /tmp/odom_to_mavros.lock` 看锁文件 PID 对应哪个进程（锁文件 PID = 真正的实例）。判断"是否重复"前先区分 bash 包装 vs 实际 python3 进程。

### ⚠️ NOT READY 第三根因：map frame 不存在 → odom_to_mavros lookup_transform 失败（2026-08-13 实测）
**症状**：进程全正常（host_sdk=1、odom_to_mavros=1、odin odometry ~14Hz、vision_pose 在发），EKF 一度 fusion True，但启动后仍 NOT READY。
**根因**：odom_to_mavros.py 的 `odom_callback` 里 `tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(0))` —— **map frame 不存在时抛 LookupException → vision_pose 发不出 → EKF 无视觉 → NOT READY**。map frame 由 **px4_and_others.launch 的 `odom_to_map_broadcaster`**（`static_transform_publisher "0 0 0 0 0 0 map odom 100"`）发布——px4_and_others 没跑/没加载该节点时 map 就不存在。
**排查**：
```bash
rosrun tf tf_echo map odom 2>&1 | head -3    # "map does not exist" = 根因!
ps aux | grep static_transform_publisher | grep -v grep  # map 发布者进程
ps aux | grep px4_and_others | grep -v grep              # 上层 launch 在跑吗
timeout 3 rostopic echo /tf_static -n 1 --noarr | grep -E "frame_id|child"  # map→odom 静态TF
tail -10 ~/.cache/liyumen/log/odom2mav.log | grep -iE "TF|not available"    # odom_to_mavros 异常日志
```
**判据**：`odom2mav.log` 里 `TF map->odom not available yet` 刷屏 + `tf_echo map odom` 报 map 不存在 = 此根因。**修复方向**：确保 px4_and_others.launch 随"启动雷达"正常拉起（含 odom_to_map_broadcaster）；或 odom_to_mavros 对 TF 失败做降级（直接发 odom 系位姿，不强制转 map）。
**完整 NOT READY 根因清单（四选一，按序查）**：① 缺 odom_to_mavros 进程（手动没起/按钮没带）② 多个 odom_to_mavros 打架（vision_pose 数千 Hz）③ map frame 不存在（px4_and_others 没跑）④ **单实例但 vision_pose 超频（odin 400Hz 全转发 → 500-1000Hz，PX4 EKF 拒收）**。排查顺序：`ps odom_to_mavros` → `rostopic hz vision_pose` → `tf_echo map odom`。

### ⚠️ NOT READY 第四根因：vision_pose 超频（单实例也翻车）→ 限速 20Hz（2026-08-13 实测+修复）
**症状**：odom_to_mavros 单实例（锁生效）、host_sdk=1、EKF 一度 fusion True，但启动后仍反复 NOT READY。`rostopic hz /mavros/vision_pose/pose` = **0.001-0.002s（500-1000Hz！）**。
**根因**：odin 的 `/odin1/odometry_highfreq` 是 **400Hz 高频**话题，odom_to_mavros.py 的 `odom_callback` **全量转发** → vision_pose 500-1000Hz → **PX4 EKF2 对 vision_pose 有频率上限，超频被判定数据异常/不可信 → 融合丢失 → NOT READY**。之前以为是"多实例打架才超频"，实际**单实例 + 400Hz 输入源本身就超频**。
**修复（odom_to_mavros.py 回调限速，已实施）**：`__init__` 加 `self._last_pub = rospy.Time(0)`；发布前：
```python
now = rospy.Time.now()
if (now - self._last_pub).to_sec() >= 0.05:   # 20Hz 上限
    self._last_pub = now
    self.vision_pub.publish(pose_map)
```
**验证**：`rostopic hz /mavros/vision_pose/pose` = 0.049-0.096s（10-20Hz）✅ + EKF velocity_horiz/pos_horiz_rel=True。改完重启 odom_to_mavros（`kill` 旧实例 + `rm /tmp/odom_to_mavros.lock` + nohup 重启）。
**教训**：**任何高刷传感器源（400Hz）经 python 直通转发都会超频**——转发节点必须限速（50ms 节流），不能依赖 PX4 侧容错。正常 vision_pose 应 ≈ odom 源频率（10-20Hz）。

## SUPER 地图不显示（只有轨迹没有地图）排查 — 2026-08-13

**现象**：ready to fly 后按按钮3，top_down.rviz 里有轨迹线（绿线 global_path_marker）但**没有地图**（无点云/占用网格）。

**排查链（按序）**：
```bash
ps aux | grep fsm_node                    # ① real_flight/fsm_node 在跑吗(最常见! 按钮3可能已停)
tail -20 log/real_flight.log              # ② 看是否 "killing on exit" = 被外部关闭(非崩溃)
grep -n cloud_topic config/real_flight.yaml  # ③ 数据源: /cloud_registered(MID360) vs /odin1/cloud_slam(odin)
rostopic info /odin1/cloud_slam           # ④ 订阅者: fsm_node 应订阅; 只有 rviz=SUPER 没接上
timeout 4 rostopic echo /tf -n 1          # ⑤ TF: odin 只发 odom→imu; SUPER 地图可视化 frame_id="world"
```
**三个独立根因（都可能）**：
1. **cloud_topic 还是 MID360 的** `/cloud_registered`（odin 不发布）→ 改 `/odin1/cloud_slam`（已修复）。
2. **fsm_node 根本没在跑**：real_flight.launch 被正常关闭（日志 `[fsm_node-1] killing on exit` + `shutting down processing monitor complete` = **外部关闭不是崩溃**——可能是误按了停止按钮/UserCmd2）。此时所有地图话题都不存在。
3. **TF frame 不匹配（待验证）**：odin 点云 frame_id="odom"，TF 树只有 `odom → imu`；SUPER rog_map 可视化 frame_id="world"（real_flight.yaml ~171 行），需要 odom→world/map 桥接（real_flight.launch 里有 `map_to_world (tf/static_transform_publisher)` 节点——MID360 时是 camera_init→map，odin 时是否适配未验证）。**MID360 与 odin 的 frame 链不同，换传感器后除 cloud_topic 外还要核对 TF 链**。

**判读 real_flight.log**：`process[fsm_node-1]: started with pid [31xxx]` + `killing on exit` = 启动成功但被停止（检查是否误触停止按钮/残留 global_path_publisher 进程仍在发旧数据）。重启按钮3 前可先 `pkill -f global_path_publisher` 清残留实例。

## Offboard 安全前提：bridge 必须先在跑 — 2026-08-13

**用户问"切 Offboard 还有悬停吗"**——答案取决于 bridge 是否在跑：
- **bridge（super_to_px4_bridge）在跑** → 即使无轨迹也发"当前位置 setpoint"（hold）→ 切 Offboard 悬停 ✅
- **bridge 没跑**（real_flight 未启动/被停）→ 无 setpoint 源 → 切 Offboard 后 **COM_OF_LOSS_T（默认 1.0s）触发 "Offboard lost" 失控保护**（降高/降落，危险）❌

**检查**：`ps aux | grep super_to_px4_bridge` + `rostopic hz /mavros/setpoint_raw/local`（有数据=bridge 在发）。**安全流程**：先按按钮3（bridge 起来）→ 确认 setpoint 在发 → 才切 Offboard → 稳定悬停后拨 CH10 触发复飞。**绝不能在 bridge 没跑时切 Offboard**。

## 导航检验完整步骤序列（安全第一，逐级确认）

```
① 解锁 + 起飞: 遥控器解锁 → POSCTL → 推油门悬停 ~0.6m → 确认稳（不飘不移）
② 切 Offboard: 观察 3-5s 稳定悬停（bridge 发当前位置 setpoint）→ 异常立即切回 POSCTL
③ 拨 CH10 触发: 日志 "Mavros triggered" + "Pub goal" + "Get goal"
④ 观察循迹: 沿轨迹走（不斜切/不跑偏）、速度均匀(max_vel 0.3)、转弯平滑
⑤ 静态避障: 凳子/箱子放前进段 → "Global path blocked" → 绕行 → 恢复
⑥ 动态避障: 人持纸箱走到轨迹前方(离飞机 1-2m 外) → 绕行 → 走开 → 恢复
⑦ 回收: 切 POSCTL → 降高落地 → 停止SUPER按钮 → 关闭雷达
安全红线: 飞机异常(飘/抖/乱转)立即遥控器 POSCTL/Position 接管; 人站 2-3m 外; 障碍用长杆放
```

**每步完成确认再进下一步**——尤其②是"桥接悬停"关键验证点（确认 bridge 悬停正常才敢触发复飞）。

## "启动雷达"按钮 = 先关后开（已实现，勿重复排查）— 2026-08-13

新版 `nx_command_server.py` 的 `_handle_radar_launch`（~454 行）**已经实现"先关再开"**：
```
按"启动雷达"(Odin/Odin-MindCloud 分支, ~2327 行) →
  ① 等上一轮关闭线程(bg_kill_lidar)结束(join timeout=120) 防误杀新进程
  ② 检查残留(有则 kill_lidar_processes 完整清理)
  ③ 清理干净才 _launch_and_track_radar 启动新雷达
同时有 bg_start_radar 线程互斥: 上一轮启动未完成则忽略重复指令
```
**结论**：重复按"启动雷达"不会产生多进程（有互斥+先清后启）——之前的多进程事故根因是 **cleanup 配置漏 host_sdk_sample/odom_to_mavros**（已修），**不是启动按钮没做先关后开**。遇到多进程时先查 cleanup 是否覆盖驱动进程名，别改启动逻辑。

## 动态障碍"消失不净"：cloud_slam（累积） vs cloud_raw（当前帧）— 2026-08-13 晚

**用户症状**："障碍块刷新不够快，障碍明明消失了仍被判断为障碍块"。

**根因分析**：地图输入 `cloud_topic=/odin1/cloud_slam` 是 **SLAM 累积/配准点云**（建图语义：人走过的位置点会保留）→ 人走开后点还在 → 障碍残留。动态障碍跟踪应使用**当前帧**点云（人走→当前帧无人的点→raycasting 光线穿过→贝叶斯衰减清除）。

**已试方案（2026-08-13 晚，已回退）**：
```yaml
rog_map:
  ros_callback:
    cloud_topic: "/odin1/cloud_raw"   # 累积SLAM点 → 当前帧原始点云
  raycasting:
    p_miss: 0.6                        # 0.49 → 0.6 清除更快(贝叶斯衰减加速)
```
**结果**：换成 cloud_raw 后**按按钮3 没有地图了**（连带触发"Odom below virtual ground"地图拒绝更新，见下节），用户叫停"回退到之前的吧，先别改"→ 已回退到 `real_flight.yaml.bak_bakopt`（cloud_slam + p_miss 0.49 + inflation_step 2），改动前先把当前版备份为 `.bak_3changes` 以便再试。

**教训**：
- cloud_raw（当前帧）语义上才是动态障碍的正解，但**换点云源会连带触发地图不更新问题**——下次再试必须同时核对：① 启动顺序（先"启动雷达"再按钮3，odin 没跑时 fsm_node 会退）② odom/TF 链 ③ "Odom below virtual ground" 日志。或在 cloud_slam 上先验证 raycasting 是否真在清（p_miss/p_free 贝叶斯衰减应几帧内把消失障碍概率压到 p_free 以下）。
- **未验证改动先回退**：用户对"越改越糟"的容忍度低，多个参数一起试后出问题，回退到最后一个已知良好版本（备份链 .bak_* 按时间保留）。

## "Odom below virtual ground" 刷屏 = 地图拒绝更新（启动顺序 + fix_map_origin 量化）— 2026-08-13 晚

**现象**：按钮3 后无地图，`real_flight.log` 反复刷 `-- [ROGMapCore] Odom below virtual ground, please check map parameter -- .`，`/fsm_node/rog_map/inf_occ` 无数据。

**排查链**：
```bash
ps aux | grep fsm_node                     # ① 在吗? 实测常为 0 = real_flight 已退出(odin 没先启动)
ps aux | grep host_sdk_sample              # ② odin 在吗? 实测常为 0 = 没先按"启动雷达"
timeout 3 rostopic hz /odin1/cloud_slam    # ③ 点云源(odin 没跑=无数据)
grep -a "below virtual" log/real_flight.log # ④ 地图拒绝更新日志
grep -n "fix_map_origin\|virtual_ground" config/real_flight.yaml
```
**两个独立根因**：
1. **启动顺序（最常见）**：按钮3（real_flight）启动时若 odin 还没跑（没先按"启动雷达"），fsm_node 会退出/无地图。**铁律：先"启动雷达"→ 等 20s → 再按钮3**。
2. **virtual_ground 量化**（prob_map.cpp:64-67）：`virtual_ground_height: -0.9` 经 `posToGlobalIndex` 量化成网格 id 再乘 resolution（`cfg_.virtual_ground_height = id_g * 0.15`），实际生效值可能是 **-0.15**（日志 `[ProbMap] virtual_ground_height: -0.15` 与配置 -0.9 不符就是量化痕迹）。结合 `fix_map_origin: [0,0,1.5]`（地图原点 z=1.5），地面 odom z≈0 在地图系坐标 ≈ -1.5，可能低于虚拟地面 → 地图拒绝所有更新。**未修**（用户叫停回退）——下次排查方向：fix_map_origin z 改 0、或 virtual_ground 改更低（-1.8）、或先算清 odom z 在地图系的真实值。
**判读**：`[ProbMap] virtual_ground_height: X` 的 X ≠ 配置值 = 量化；`Odom below virtual ground` 刷屏且 fsm/odin 进程都在 = 地图系 z 判断问题（不是启动顺序问题）。

