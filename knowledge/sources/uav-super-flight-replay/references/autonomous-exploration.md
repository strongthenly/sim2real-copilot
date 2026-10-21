# 自主探索 + 全局地图持久化方案（2026-08-15 调研，未实施）

背景：用户撞墙（坑56 雷达仅扫4.5m）+ 雷达范围外无地图，决定"加全局地图约束飞行范围"。
本文件 = 探索建图 + 全局地图持久化 + 开源算法调研的完整落地方案。

## 一、为什么探索必须配全局地图（用户关键疑问）

- rog_map 是**滑动局部地图**（map_sliding_en=true，原点随飞机移动，滑出边界清除）→ 飞机飞走
  之前扫的区域**消散** → 探索白做。
- **结论：探索 = 建图手段，全局地图 = 保存载体，下次加载先验 = 复用价值，三者闭环才有意义**。

## 二、架构（最小侵入：新增 1 节点 + 开 1 开关）

```
┌─ 现有 rog_map（开 frontier_extraction_en: true）────┐
│  prob_map.cpp 内置前沿提取 → 发布 /fsm_node/rog_map/frontier(点云) │
└──────────────┬───────────────────────────────────────┘
               ▼
┌─ 新增 exploration_node（Python 独立节点）────────────┐
│ ① 订阅 frontier 点云 + odom                          │
│ ② 前沿簇聚类（DBSCAN）                               │
│ ③ 目标选择：score = 簇点数(信息增益) - α×距离(路程代价)，可达性检查 │
│ ④ 发布 goal → 现有通道                               │
│ ⑤ 到达判定（距目标<0.3m）→ 选下一个前沿               │
└──────────────┬───────────────────────────────────────┘
               ▼
┌─ 现有 FSM（不动）────────────────────────────────────┐
│ 订阅 goal → PlanFromRest → MINCO → 飞向前沿 → 扫新区域 → 循环 │
└──────────────────────────────────────────────────────┘
```

**探索循环**：提取前沿(已知↔未知边界) → 聚类 → 选目标(最近+信息量大+可达) → 飞过去
→ 雷达扫开新区域 → 地图扩大 → 新前沿 → 循环 → 无前沿/覆盖达标 → 完成。

**机制类比**（给用户讲）：黑屋子手电筒探索——光斑边缘=前沿，永远朝光斑边缘走，
屋子被一寸寸照亮；墙被扫到后前沿不再延伸向墙（墙=障碍不穿过），探索自然停止。

## 三、⭐ 已确认的对接接口（网络恢复后实测）

- **frontier 提取**：`rog_map` 已有 `frontier_extraction_en`（config.hpp:114 加载，默认 false；
  prob_map.cpp:41/148/300/536 实现 `fcnt_map_->updateFrontierCounter` + `boxSearch(FRONTIER)`；
  rog_map_ros1.hpp:185-189 有 `frontier_pub` 话题发布）。**开开关即可用，无需编译**。
- **goal 通道**：waypoint_mission（ros1_waypoint_planner.hpp:186）发 `/move_base_simple/goal`
  （waypoint.yaml `goal_pub_topic`）；**bridge 的 goalCallback 已订阅该话题** → 探索节点直接
  发 PoseStamped 到 `/move_base_simple/goal` 即可，FSM 自动响应，零改动对接。
  ⚠️ 注意：waypoint.yaml 的 goal_pub_topic 若是 `/waypoint_goal` 则走航点通道（override_height=false），
  若是 `/move_base_simple/goal` 则落 click 通道（override_height=true 高度被覆盖）——坑1。探索节点
  发 goal 前先确认 bridge/FSM 实际订阅哪个话题。
- **与现有机制兼容性（天然契合）**：frontend_in_known_free=true → 前沿就在已知边界，A* 只飞已知区
  到边界扫新区域 → 安全；AVOID_WAIT → 探索途中遇障停车等待；全向光线 → 地图干净；
  robot_r=0.35 → 走廊离障碍远。

## 四、全局地图持久化（解决"探索后消散"）

```
探索中每帧: cloud_slam(odom系) + 位姿 → 全局体素哈希累积（只写不删）
探索完成:   全局地图 → 保存文件（PCD / .npy 障碍格坐标 / OctoMap .bt）
下次任务:   加载文件 → 初始化先验 → 墙/边界已知 → 不撞墙 → 无需重新探索
```

- cloud_slam 已在 odom 系（frame=odom），无需额外变换——全局地图累积简单。
- 实现可复用 cloud_dynamic_filter.py 的体素哈希代码（坑51）。

## 五、开源成熟算法调研（GitHub 实测数据）

| 项目 | 地址 | ⭐ | 适配度 |
|---|---|---|---|
| **FUEL** | HKUST-Aerial-Robotics/FUEL | 1457 | 🥇 **与 SUPER 同源**（同为 FAST-Planner 系 MINCO/走廊），消息/坐标系/风格一致，集成成本最低。论文 FuEL: Fast UAV Exploration using Incremental Frontier Structure and Hierarchical Planning |
| **GBPlanner** | ntnu-arl/gbplanner_ros | 842 | 🥈 图基探索（地下/室内，NTNU），随机采样探索图+信息增益，ROS1 支持 |
| **explore_lite** | robo-friends/m-explore-ros2 | 385 | 🥉 经典前沿探索（1997 算法），偏 2D 小车，3D 无人机要改——可借鉴"前沿提取+目标选择"思路 |
| **OctoMap** | OctoMap/octomap | 2353 | 🏅 标准八叉树占用地图，.bt 文件保存/加载，概率更新——完美持久化载体 |

**调研方法**：GitHub API `https://api.github.com/search/repositories?q=<query>&sort=stars`
（PowerShell `Invoke-RestMethod -Uri ... -Headers @{"User-Agent"="Mozilla/5.0"}`）。

## 六、实施路线（推荐）

```
① 短期(1天): rog_map 内置 frontier_extraction_en=true + 自写轻量 exploration_node
   → 最快验证"探索→建图→不撞墙"闭环（室内 2.5m 很快扫完）
② 中期(2-3周): 借鉴 FUEL 增量前沿结构（信息增益目标选择）
③ 持久化: 全局地图先 PCD/.npy 简单保存，后期 OctoMap .bt
④ 全局地图接入规划: 飞行范围约束（把墙/边界放进地图）→ 配 frontend_in_known_free=true
   + 终点判定 0.3（坑58）→ 解决撞墙/乱跳/乱转
```

## 七、避障保留确认（用户问"避障还在吗"）

探索只换"目标从哪来"（航迹→前沿），FSM 收到 goal 后链路分毫未动：
PlanFromRest → MINCO → 途中遇障碍走廊失败 → fallback A* 绕行 → 绕行失败 → AVOID_WAIT
停车等待 → 障碍消失恢复。探索模式下避障甚至更关键（未知环境障碍更多）。

## 八、⭐ FUEL 全套路线（2026-08-15 用户选定，未实施）

用户明确：**想要解耦方案（方便随后替换规划器），先拉 FUEL 全套跑通，再设计解耦接口**——
"先看全貌，再拆接口"。与"自写轻量节点"的取舍：轻量版(1天)绑定 SUPER 不解耦；FUEL 全套
周期长但先理解完整系统才能设计出正确的接口边界。

**FUEL 内部结构（GitHub API 实测）**：`fuel_planner/` 下：
- `exploration_manager/`（探索管理器=核心：frontier 管理+目标选择+全局探索路径）
- `active_perception/`（主动感知：前沿提取+增量前沿树 FIS，核心创新）
- `path_searching/`（RRT 采样探索图）、`plan_env/`（自己的 SDF 栅格地图）
- `bspline + bspline_opt`（B样条轨迹优化，MINCO 同源）、`plan_manage/`（主节点：订阅点云+odom→输出轨迹）

**集成策略（关键决策）**：FUEL 是完整系统（自带规划器），但绝不整个替换（会丢掉我们调试好的
避障/走廊/防撞墙/AVOID_WAIT）。**只借"探索大脑"**：
- ✅ 用：`active_perception`（前沿树）+ `exploration_manager`（目标选择）——裁剪移植
- ❌ 不用：`plan_env`（改用我们的 rog_map 查询接口）、`bspline/bspline_opt`（用我们的 MINCO）
- 输入：rog_map 地图数据（坐标系/参数已调好）；输出：goal（/move_base_simple/goal）→ SUPER 执行
- 依赖：Eigen(已有) + OSQP(bspline 才需要→可省)

**执行计划（五阶段，总约 1 周）**：
```
A 拉取+编译(0.5-1天): git clone ~/FUEL → 依赖检查(OSQP/nlopt apt) → fuel_planner+uav_simulator
  放入 catkin_ws/src → catkin_make -j2 -DROS_EDITION=ROS1（NX 算力慢, 后台 nohup）
B 仿真跑通(0.5-1天): uav_simulator Gazebo 场景 → FUEL 探索节点+RViz → 观察前沿簇/目标选择/轨迹
  （NX 跑 Gazebo 较重→降画质/headless/借开发机 45）
C 接口分析(0.5天): FUEL 订阅什么(点云/odom/goal) 发布什么(traj/可视化) → 探索管理器与轨迹优化的耦合点
D 解耦设计(0.5天): 定义三层+两接口（见第九节）→ 决策 方案X vs 方案Y
E 集成测试(2-3天): 方案X 实施 → 仿真联调 → 室内真机探索→建图→保存(PCD/OctoMap)→下次加载
```

**风险清单**：NX 跑 Gazebo 性能（降画质/换机）；依赖装不上（apt 源码装）；话题冲突（/fuel_* 命名空间隔离）；
移植工作量大（只移 exploration_manager 不碰 bspline）；坐标系（FUEL odom vs 我们 odom，对齐验证）。

## 九、⭐ 三层解耦架构设计（用户核心需求：方便替换规划器）

```
┌─ 探索层(可替换) ──────────────────────────┐
│ 选择1: FUEL exploration_manager（选定路线）│
│ 选择2: rog_map frontier 轻量节点           │
│ 选择3: 自定义(未来)                       │
└──────────────┬──────────────────────────┘
   接口①: 标准消息输出 "目标点"(geometry_msgs/PoseStamped)
           或 "探索路径"(nav_msgs/Path)
┌─ 规划执行层(可替换) ──────────────────────┐
│ 选择1: SUPER FSM(MINCO+避障)（推荐保留）    │
│ 选择2: FUEL bspline(无避障? 需评估)        │
│ 选择3: 其他规划器                         │
└──────────────┬──────────────────────────┘
   接口②: 输出轨迹/指令（现有 /planning/pos_cmd，bridge→PX4）
┌─ 飞控层(固定) ───────────────────────────┐
│ super_to_px4_bridge → PX4                │
└──────────────────────────────────────────┘
```

- **接口①设计原则**：探索层"不知道"谁在执行——只发标准 goal/路径消息；
  **接口②**：规划层"不知道"探索用啥——只收目标点。
  → 换探索算法只动上层；换规划器只动中层；解耦完成。
- **方案 X（推荐）**：FUEL 探索层 + SUPER 规划层——保留全部避障调试成果；
  **方案 Y**：FUEL 全套（探索+轨迹）+ 飞控桥——需重做避障，放弃 SUPER 成果。
- 用户立场（2026-08-15 明确）："不能让 SUPER 没有作用"→ 默认走方案 X。
- 全局地图（第四节）由探索层一并维护，作为独立第三输出（探索建图→保存→下次加载先验）。

