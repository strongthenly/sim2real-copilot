# 论文陪读 + FUEL 算法拆解（研究型任务）

用户说"想从算法层突破、先学习一下别人的算法 / 做我的论文陪读"时，这是一类**研究型任务**（不是运维仿真）。产出物 = 一份**算法拆解笔记**（markdown 落盘给用户反复读），核心是让用户"看懂每个算法的核心洞察 + 局限（局限就是创新切入点）"。

## 论文陪读方法论（可复用：FUEL / EGO-Planner / RACER / 任何探索/规划算法）

1. **拉资料**（都要走 Clash 代理 `127.0.0.1:7899`）：
   - 源码：`git -c http.proxy=... -c https.proxy=... clone --recursive <repo>`；源码 README 里通常有论文 arXiv 链接 + 依赖 + 包结构。
   - 论文：`curl -L --proxy http://127.0.0.1:7899 -o paper.pdf https://arxiv.org/pdf/<id>`（v2 用 `.../<id>v2`）。
   - **查 DOI / 找 OA PDF（用户要"放 N 篇相关最新论文"时，先检索再下载，别直接猜 arXiv 号）**：
     - **CrossRef 查 DOI**：`https://api.crossref.org/works?query.bibliographic=<标题>`（返回 DOI + title；顺带能发现同方向的后续工作）。
     - **Semantic Scholar 查 OA PDF + arXiv**：`https://api.semanticscholar.org/graph/v1/paper/DOI:<doi>?fields=title,openAccessPdf,externalIds,abstract`（⚠️ 会 429 限流，重试或换接口）。
     - **Unpaywall 判 OA**：`https://api.unpaywall.org/v2/<doi>?email=<任意email>`（`is_oa: false` = 付费墙）。
     - **arXiv 搜索**：`http://export.arxiv.org/api/query?search_query=all:"<词>"&sortBy=submittedDate&sortOrder=descending&max_results=N`（返回 `<entry>`，正则提 title/id）。
     - **付费墙（IEEE 等）处理**：`is_oa:false` 且无 arXiv → **换 arXiv 免费的相关论文**（本会话 FUEL 后续 FGIP/EFP 是付费墙，换成 LiDAR 探索的 EPIC arXiv:2410.14203、SHIELD arXiv:2512.23972，反更贴用户 MID360 平台），并把 DOI 记下来供用户用学校订阅自取。
     - **⚠️ 别用 `curl | python -c "json.load(sys.stdin)"`**——curl 输出带 UTF-8 BOM，管道进 python 报 `Unexpected UTF-8 BOM (decode using utf-8-sig)`。**改用 Python `urllib.request`（`ProxyHandler({'http'/'https': 'http://127.0.0.1:7899'})`）直接请求**，避开 PowerShell 管道编码坑。
   - **产出物落点（用户偏好）**：拆解笔记 + 论文 + 源码，用户要**整个放到桌面根目录** `C:\Users\<USER>\Desktop\`（不是 workspace 子目录）。用户说"放桌面上"时 = 把**整个项目文件夹 + 笔记一起**挪到桌面根目录，**别只复制笔记/PDF 两个文件**（本会话第一次只复制了笔记+PDF，用户又重复"放桌面上"，才明白要连源码文件夹一起挪过去）。
2. **提取论文文本**：pymupdf（`import fitz; [p.get_text() for p in fitz.open(pdf)]`）。8 页 RA-L ≈ 41KB / 1000 行，直接 `read_file` 读。**提取脚本用相对路径定位**：`HERE = os.path.dirname(os.path.abspath(__file__))`，再 `fitz.open(os.path.join(HERE, "paper.pdf"))`——别硬编码绝对路径，否则文件夹一移动（本会话就把脚本随文件夹一起挪到桌面）脚本就失效。**验证**：ad-hoc 脚本确认产物含标题/Abstract 关键词 + 非空。
3. **读论文** → 提炼「问题怎么建模 → 核心洞察 → 局限」三层，别背公式。
4. **读源码对照**：**先读数据结构头文件**（`*_data.h` 里的 struct）+ **类接口**（`.h` 的方法签名），**再读主流程函数**（如 `planExploreMotion()`）确认实现即可——**不必通读全部 `.cpp`**（FUEL 有 42 个，只读 5-6 个关键的就够写笔记）。定位关键函数用 `search_files` 搜 `void Class::(func1|func2|...)`。
5. **写拆解笔记**（结构见下），落盘 `算法名_算法拆解笔记.md`。
6. **结尾给"下一步选项"**：精读某模块源码 / 转读另一算法 / 聊论文贡献定位（把选择权交回用户）。

### 拆解笔记结构（已验证有效）
```
0 一句话总结
1 问题本质（探索 = 信息增益 vs 路径成本 的多目标权衡）
2 总览 pipeline（图/数据流）
3 核心创新逐个拆（数据结构 + 大白话 + 源码↔论文对照）
4 源码↔文件对照表
5 对接自己平台的数据流（输入输出怎么映射）
6 创新点切入点（局限 → 改进方向）
```

## FUEL 算法核心（RA-L 2021, arXiv:2010.11561, HKUST Aerial Robotics）
**一句话**：FIS（增量 frontier 结构）让它**快**；层次化规划（全局 TSP + 局部精化 + 最小时间轨迹）让它**路径短、飞得快**。3-8× 快于当时 SOTA。

**层次化 pipeline**：传感器→占据栅格(含 ESDF) → [0]增量更新 FIS → [1]全局 TSP tour → [2]局部视角精化 → [3]最小时间 B-spline 轨迹 → 控制器，地图一更新就重规划。

- **FIS（核心创新1）**：每个 frontier 簇存 `{格子 Ci, 平均位置 pavg, AABB Bi, 候选视角点 VPi, 连接代价 Lcost}`。增量更新用 **broad/narrow phase**（AABB 粗筛淘汰无关簇 + 精确检查）避免每次扫全图；大簇用 **PCA 沿第一主成分递归劈半**（让每个簇对应"有意义的未知区域"）。
- **视角点生成**：以簇中心为原点**圆柱坐标采样** + **yaw 优化**（最大传感器覆盖）+ 筛 top `Nview`(默认15)。连接代价 `tlb = max(路径/vmax, yaw变化/ξ̇max)`（A* 搜路径）。代价**增量**更新（O(k_new·N_cls) 非全量 O(N²)）。
- **全局 tour（核心创新2）**：把"遍历所有簇"建模成 **ATSP**。**开环→闭环归约**：设"回当前视角点代价 = 0"，则闭环最优解必含等代价开环解。加**运动一致性代价 `cc`**（当前速度方向 vs 目标方向夹角）惩罚折返。用 **LKH** 求解。
- **局部精化**：截断 tour（<Rrf 默认5m）内视角点摆成 **DAG**，**Dijkstra** 求最优局部视角序列。
- **轨迹**：基于 **EGO-Planner**（B-spline + ESDF 梯度），但**连时间间隔 ∆tb 一起优化**使总时间最小 → 飞最快。NLopt 求解。

**关键源码对照**（`fuel_planner/` 下）：
| 模块 | 文件 | 干什么 |
|---|---|---|
| 主流程/TSP/精化 | `exploration_manager/src/fast_exploration_manager.cpp` | `planExploreMotion` / `findGlobalTour` / `refineLocalTour` |
| frontier 检测 | `active_perception/src/frontier_finder.cpp` | `searchFrontiers` / `computeFrontiersToVisit` / `getTopViewpointsInfo` |
| 数据结构 | `exploration_manager/include/exploration_manager/expl_data.h` | `ExplorationData`(frontiers/views/tour…) |
| 局部规划 | `plan_manage/src/planner_manager.cpp` + `bspline_opt/src/bspline_optimizer.cpp` | EGO-Planner 管理 + B-spline 优化 |
| ESDF 地图 | `plan_env/src/sdf_map.cpp` + `edt_environment.cpp` | 占据栅格 + ESDF |
| TSP | `utils/lkh_tsp_solver/src/lkh_interface.cpp` | 写 single.tsp/.par 调 LKH |

**依赖（编译 FUEL）**：`nlopt v2.7.1`（源码编译）+ `libarmadillo-dev`；测试于 Ubuntu18.04(Melodic)/20.04(Noetic)。跑 demo：`roslaunch exploration_manager rviz.launch` + `roslaunch exploration_manager exploration.launch`（Rviz 里 2D Nav Goal 触发）。

## 对接本平台（ros1-twin）的数据流
```
FAST-LIVO2 /cloud_registered(点云) ──转换──► 占据栅格+ESDF（要补"点云→栅格"节点）
FAST-LIVO2 /aft_mapped_to_init(位姿) ──映射──► odom + tf（frame camera_init → FUEL world）
FUEL exploration_manager(FIS+TSP) → EGO-Planner 输出轨迹
   → MAVROS offboard（已有 move_forward.py 那套 setpoint 发布/切 OFFBOARD/Arm）
```
**要补的两块**：① 点云→栅格（喂给 FUEL 自带 SDFMap/edt_environment，或写 octomap 桥）；② 位姿→nav_msgs/Odometry + tf。

## 创新点切入点（局限→改进方向，论文贡献从这里长）
FUEL 自认局限：**假设位姿完美**（`assumes perfect state estimation`，没考虑漂移）。其余可挖：
| 局限 | 改进方向 |
|---|---|
| ① 假设位姿完美 | **位姿不确定性感知探索**（定位协方差纳入信息增益） |
| ② frontier 纯几何 | **语义/任务驱动探索** |
| ③ 假设静态 | 动态环境探索 |
| ④ 单机固定 FOV | 多机（→作者后续 RACER） |
| ⑤ ESDF 依赖精确 | 无/稀疏 ESDF 快速规划 |
| ⑥ TSP 用 LKH 黑盒 | 可微/学习式 tour 规划 |

**给二区论文的建议**：平台卖点是"高保真 AirSim + 真 PX4 + 实机复刻 + 话题级一致"；**算法层突破选 ① 或 ②**（跟 FAST-LIVO2 定位/感知强项结合最自然），把 FUEL 当 baseline，不去改它内部。
