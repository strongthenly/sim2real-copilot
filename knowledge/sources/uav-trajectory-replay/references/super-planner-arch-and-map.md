# SUPER 规划架构 + ROG-Map 地图机制 + 障碍吸附坑（2026-08-10 源码核对）

## 1. SUPER 规划架构：FPP 一体化（不是"全局规划器+局部规划器"两层）

HKU MaRS SUPER（Science Robotics 2024, "Safety-assured high-speed navigation for MAVs"）。
**每次 PlanFromRest（fsm.cpp:139）同时完成寻路+轨迹优化**，无独立全局规划器节点：

```
fsm.cpp PlanFromRest(goal_p, goal_yaw, new_goal)
  Step1 前端 A* 寻路（src/super_core/astar.cpp，path_search/astar.h）
        · 在 ROG-Map 占据地图上 3D A*，当前位置→目标，找无碰撞粗略路径
        · frontend_in_known_free: false（当前配置=允许穿未知空间）
  Step2 后端 MINCO 优化（src/traj_opt/exp_traj_optimizer_s4.cpp + minco.cpp）
        · 离散路径 → 凸多面体走廊(Polytopes) → 时间分配 + MINCO 多项式轨迹
        · 约束：max_vel=2.0 / max_acc=4.0（real_flight.yaml traj_opt/boundary）
        · 失败日志：GeneratePolytopeFromLine failed / OptimizationExpTrajInPolytopes failed
  Step3 FOLLOW_TRAJ 跟踪：每控制周期取轨迹点发 /planning/pos_cmd；飞行中每 ~0.1s ReplanOnce
```

与 move_base 对比：move_base=全局规划一次+局部跟随；SUPER=**每次重规划都重跑 A*（全局感）+ MINCO 只在当前凸走廊优化（局部感）**，两者耦合执行。
三层配合：waypoint_mission（最外层，只管"下一个目标是谁"）→ FSM 状态机（中层，管"该规划还是该跟"）→ SUPER 规划器（内层，管"怎么飞过去"）。航点 <0.1m 到达（switch_dis）→ waypoint_mission 给下一个 → 重新 A*+MINCO。

## 2. ROG-Map 概率地图：动态避障机制（"忘记"而非"追踪"）

- 点云入口：rog_map_ros1.hpp cloudCallback（97 行）→ 缓存 rc_.pc + rc_.pc_pose → updateCallback 定时调 updateProbMap
- **概率更新（raycasting）**：命中点 p_hit 概率↑，射线穿过 p_miss 概率↓；判定 >p_occ=占用、<p_free=空闲
- 默认参数（config.hpp:185-191）：p_hit=0.70, p_miss=0.70, p_min=0.12, p_max=0.97, p_occ=0.80, p_free=0.30
- **关键：不清除旧障碍的机制**——体素概率被拉低**完全依赖新激光帧的射线穿过该体素**。人走开后若该位置不再被激光扫到（人进盲区/飞机没动/被遮挡），p_miss 永不作用 → **障碍块永久残留**。这是所有射线式概率地图的通病，不是 bug
- 残留加剧因素：use_fov_cut=false（旧区域不主动失效）、fix_map_origin 固定原点、map_size [30,30,6] 局部地图边缘
- **⭐ 双关 = 永不消除（2026-08-11 实机确认，比"残留延迟"更严重）**：查 `real_flight.yaml`（SUPER 仓库 super_planner/config/real_flight.yaml 第 149-186 行）——**`esdf.enable: false` 且 `raycasting.enable: false` 两个都关**！此时点云只做"命中更新"（p_hit 障碍变实），**p_miss 清除更新完全不执行** → 障碍只增不减，人走过的障碍块**永远留在地图里**（不是"过一会儿消失"，是根本不消失）。用户实测："动态障碍并没有被及时去除"。**修复**：`raycasting.enable: false → true`（它同时做命中+清除；esdf 即使开了也是静态场不清动态障碍）。⚠️ 打开后副作用：激光扫不到的角落静态障碍可能被 p_miss 误清（飞前注意）+ 计算量上升。改完重启 SUPER 生效（real_flight.yaml 启动时加载）。**排查优先级**：发现"规划失败 + 动态障碍不消失"，先查这两个 enable，比查 p_free/p_occ 数值更根本
- 对策（按成本排序）：①起飞前原地转一圈/动一动清场（让 p_miss 扫掉人留下的块）；②p_free 0.30→0.45（更快判空闲）；③起飞前重启 SUPER 清图；④use_fov_cut=true（视野外旧数据主动失效，需配 FOV 参数）

## 3. ⭐ getNearestInfCellNot 障碍吸附坑（复飞失败"贴地转桨"深层根因，2026-08-10 实测）

**现象**：waypoint_mission 日志 `Pub goal to -0.127 0.091 0.554`（原始航点正确），但 fsm 日志 `Get goal at -0.075 0.075 0.825`（坐标被改写，x/y/z 全偏，z +0.27m）。

**根因**（fsm.cpp:184-196 setGoalPosiAndYaw）：
```cpp
void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q, bool override_height) {
    auto click_point = p;
    if (override_height && cfg_.click_height > -5) click_point.z() = cfg_.click_height;
    // ⭐ FSM 收到航点后不直接用，而是找"离该点 3m 内最近的空闲格"作为实际目标
    if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)) {
        cout << " -- [Fsm] Get goal at " << gi_.goal_p.transpose();  // 打印的是吸附后的!
    }
}
```
- **日志里 `Get goal at` 打印的是"吸附到最近空闲格后的目标"**，不是原始航点
- 原航点在 ROG-Map 里被判 OCCUPIED（人走过的障碍块残留 / 起飞点拥挤 / 坐标系错位）→ 被吸附到旁边空闲格
- 吸附导致目标高度突变（0.554→0.825）→ 与起飞悬停高度(0.60)、起点状态约束冲突 → **MINCO 优化 NaN/Inf**（日志 `[MINCO] TrajOpt failed, The function value became NaN or Inf`）→ GenerateExpTrajectory failed → 复飞中断 → 无指令 → 落地贴地转桨

**排查铁证链**：触发成功（`Mavros triggered.` + `Pub goal`）→ 但 `Get goal at` 坐标 ≠ `Pub goal` 坐标 → 即吸附发生 → 查地图该点是否占用（RViz 蓝点 vs 灰色障碍格相对位置）
**根治**：清地图残留（见第 2 节对策）+ 起飞点选空旷处 + 起飞前 RViz 核对蓝点（航点）不在障碍格里

## 4. 复飞失败排查时间线模板（两次实机案例）

```
[bridge] Waiting for OFFBOARD → OFFBOARD mode engaged → Current altitude 逐秒爬升 → Drone reached target altitude
  ↑ 这段成功 = 起飞链路正常（与 CH10/规划无关）
[waypoint] -- [MISSION] Mavros triggered. 出现? 
  · 没有 + FSM 一直 WAIT_GOAL = 拨杆问题（CH10 一直低位无下降沿）
  · 有 + Pub goal 出现 = 触发成功，问题在规划层
[fsm] Get goal at 坐标 vs Pub goal 坐标:
  · 不一致 = getNearestInfCellNot 障碍吸附（第 3 节）
  · 一致但 PlanFromRest failed + CIRI "min dis to obstacle only: 0.00000001" = 地图障碍/坐标系错位
  · GenerateExpTrajectory failed + MINCO NaN = 轨迹优化数值崩溃（吸附/边界/时间分配）
```

## 5. 附加：guidepoints_new.txt 最新性验证

- `ls -lt data_guidepoints/` 看 guidepoints_new.txt 与最新时间戳文件 mtime 一致
- `md5sum guidepoints_new.txt <最新时间戳文件>` 哈希一致 = 同步正确
- 实测文件常被用户反复更新（972 点→1856 点→705 点），每次排查前先确认当前版本，别用旧数据判读
