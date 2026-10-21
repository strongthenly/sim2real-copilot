# FUEL 接入实战（ROS1 / WSL 环境）

> 实测时间：2026-09。环境：WSL 发行版 `ros1-twin`（Ubuntu 20.04 + ROS **Noetic**）。
> FUEL 论文：*FUEL: Fast UAV Exploration using Incremental Frontier Structure and Hierarchical Planning*，Zhou 等，RA-L 2021，**arXiv:2010.11561**，DOI `10.1109/LRA.2021.3051563`，仓库 `HKUST-Aerial-Robotics/FUEL`。

## 0. 一句话：FUEL 是什么
层次化自主探索框架。两个核心贡献：
1. **FIS**（Frontier Information Structure）——把 frontier 做成**增量维护**的数据结构，而不是每次扫全图（这是它快的根因）。
2. **层次化规划**——全局 TSP 求遍历顺序 → 局部视角精化 → 最小时间 B-spline 轨迹（基于 EGO-Planner，但连时间间隔一起优化）。

## 1. 依赖与编译（实测零 error）

```bash
# ① 查现状：armadillo 一般自带
dpkg -l | grep -i armadillo

# ② nlopt 必须源码装 v2.7.1（走代理）
export http_proxy=http://127.0.0.1:7899 https_proxy=http://127.0.0.1:7899
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git ~/nlopt
cd ~/nlopt && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && make -j$(nproc)
sudo make install && sudo ldconfig      # → /usr/local/lib/libnlopt.so.0.11.1 + nlopt.h
# ⚠️ ldconfig 会报 /usr/lib/wsl/lib/libcuda.so.1 is not a symbolic link —— WSL 无害噪音

# ③ LKH 不用装！lkh_tsp_solver 的 CMakeLists 用 file(GLOB SRCS src/*.c) 把 ~120 个 LKH 源码编进库

# ④ 独立工作空间（别污染既有 twin_ws） + 编译
mkdir -p ~/fuel_ws/src && cd ~/fuel_ws/src
git clone --recursive https://github.com/HKUST-Aerial-Robotics/FUEL.git
cd ~/fuel_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4
```
产物：`devel/lib/exploration_manager/exploration_node`、`devel/lib/plan_manage/{fast_planner_node,traj_server}`、`devel/lib/lkh_tsp_solver/lkh_tsp`。

## 2. 跑自带 demo（独立 master，零干扰）

```bash
export ROS_MASTER_URI=http://127.0.0.1:11321     # 换端口，别占主 master
source /opt/ros/noetic/setup.bash && source ~/fuel_ws/devel/setup.bash
setsid rosmaster --core -p 11321 > /tmp/fuel_master.log 2>&1 &
setsid roslaunch exploration_manager exploration.launch > /tmp/fuel_demo.log 2>&1 &
# 可视化另开：roslaunch exploration_manager rviz.launch

# 触发探索（等价于 RViz 的 2D Nav Goal，不必手点）
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 1.0, z: 1.0}, orientation: {w: 1.0}}}"
```
**成功判据**：`rostopic hz /planning/pos_cmd` ≈ 100Hz；`/state_ukf/odom` 位置**离开起点**；日志出现 `cost mat size final` → `Cost mat / TSP` → `Local tour graph` → `[Kino replan]` → `FSM: PLAN_TRAJ → PUB_TRAJ → EXEC_TRAJ`；frontier 簇数先增后减。
demo 自带：`pcl_render_node`(渲深度) + `map_generator`(office.pcd) + `so3_control` + `quadrotor_simulator_so3`。

**实测性能（本机 i5-12500H）**：`Total 14.4ms`（Cost mat 3.5 / TSP(LKH) 1.9 / Local refine 0.36 / Kino 0.2 / Traj 5.8）。论文 i7-8550U 是 24-36ms。b 可直接进论文性能表。

## 3. 接口清单（读源码核实，接入照这个接）

| 方向 | 话题 | 类型 | 备注 |
|---|---|---|---|
| 输入·位姿 | `/odom_world` | `nav_msgs/Odometry` | FSM 订阅 |
| 输入·点云 | `/map_ros/cloud` | `sensor_msgs/PointCloud2` | 与 `/map_ros/depth` 二选一 |
| 输入·触发 | `/move_base_simple/goal` → `/waypoint_generator/waypoints` | `PoseStamped` | 可编程发 |
| 输出·控制 | `/position_cmd` | `quadrotor_msgs/PositionCommand` | 带 pos/vel/acc/yaw/yaw_dot **前馈** |
| 内部·地图 | `/sdf_map/{occupancy_all,occupancy_local,unknown,esdf}` | PointCloud2 | 可视化/调试用 |
| frame | **`world`** | — | 你的 VIO/LIO 常是别的（`camera_init`），**必须对齐** |

关键源码：`exploration_manager/src/fast_exploration_manager.cpp`（`planExploreMotion` / `findGlobalTour`(TSP) / `refineLocalTour`）、`active_perception/src/frontier_finder.cpp`、`plan_env/src/map_ros.cpp`（订阅话题）、`plan_manage/src/traj_server.cpp`（发 `/position_cmd`）、`exploration_manager/include/.../expl_data.h`（数据结构）、`utils/lkh_tsp_solver/src/lkh_interface.cpp`。

## 4. 接到自己平台的 5 阶段 + 两个桥

```
FAST-LIVO2（已通） ──桥①──► FUEL ──桥②──► MAVROS ──► PX4（已通）
```
- **桥①（输入）**：`/aft_mapped_to_init` → `/odom_world`（+ 发 tf `world→body`）；`/cloud_registered` → `/map_ros/cloud`（坐标从 `camera_init` 对齐到 `world`）。一个 Python 节点。
- **桥②（输出）**：`/position_cmd`(PositionCommand) → **`/mavros/setpoint_raw/local`**(`mavros_msgs/PositionTarget`)。用 `raw/local` 而非 `setpoint_position`，因为 PositionCommand 带速度/加速度前馈，raw 才能完整映射。别忘了 offboard 要**持续发**（停发 PX4 超时会 AUTO.LAND）。

| 阶段 | 内容 | 验证点 |
|---|---|---|
| ① | 装 nlopt v2.7.1（armadillo 自带、LKH 免装） | 依赖就位 |
| ② | 独立 `~/fuel_ws`，`catkin_make` | 25 包 0 error |
| ③ | 跑自带 demo + 编程触发 | 无人机自主探索、`pos_cmd` ~100Hz |
| ④ | 写桥①，喂自己的位姿+点云 | FUEL 的 `/sdf_map/occupancy_all` 出现**自己场景**的建图 |
| ⑤ | 写桥②，闭环真飞控 | 无人机自己飞着探索 |

## 5. 常见坑速查
- `nlopt` 装到 `/usr/local`，catkin 找得到；若 CMake 报找不到 nlopt，确认 `ldconfig` 已跑。
- demo 与在跑链路**共用 master 会互相干扰** → 换 `ROS_MASTER_URI` 端口。
- `MapROS` 的 `frame_id` 默认 `world`；点云/目标的 frame 对不上 → 点云飘、目标乱飞。
- 点云密度 vs ESDF 分辨率要按自己传感器调（`map_ros/*` 参数：`fx fy cx cy`、`depth_filter_*`、`skip_pixel`）。
- 探索结束判据：**没有 frontier 了**（`/heading_planner/frontier` 空）。

## 6. 算法层突破的切入点（FUEL 论文自承 + 可挖的）
| 局限 | 方向 |
|---|---|
| **假设位姿完美**（论文结论段自承） | 位姿不确定性感知的探索（把定位协方差纳入信息增益）← 与 FAST-LIVO2 强项结合最自然 |
| frontier 是纯几何 | 语义/任务驱动探索（给 frontier 加语义权重） |
| 假设静态环境 | 动态环境探索（区分移动物体与静态结构） |
| 单机、固定 FOV | 多机协作（作者后续 RACER）/ 主动感知 |
| 依赖精确 ESDF | 无 ESDF / 稀疏 ESDF 快速规划 |
| TSP 用 LKH（外部黑盒） | 可微/学习式 tour 规划 |

## 7. 怎么找 / 下载 baseline 论文（可复用姿势）

```python
# 统一用 python urllib + 代理（不要用 curl 管道给 PowerShell：会带 UTF-8 BOM 导致 json.load 报 Unexpected UTF-8 BOM）
import urllib.request, json, urllib.parse
opener = urllib.request.build_opener(urllib.request.ProxyHandler(
    {'http': 'http://127.0.0.1:7899', 'https': 'http://127.0.0.1:7899'}))
```
| 目的 | 接口 |
|---|---|
| 按标题/关键词搜 arXiv | `http://export.arxiv.org/api/query?search_query=all:"..."&sortBy=submittedDate&sortOrder=descending` |
| 由 DOI 拿元数据 + OA 链接 | `https://api.semanticscholar.org/graph/v1/paper/DOI:<doi>?fields=title,year,venue,openAccessPdf,externalIds` |
| 按题名反查 DOI | `https://api.crossref.org/works?query.bibliographic=<title>` |
| 判断是否开放获取 | `https://api.unpaywall.org/v2/<doi>?email=<你的邮箱>`（`is_oa` / `best_oa_location`） |
| 下载 arXiv PDF | `https://arxiv.org/pdf/<id>v<N>`（走代理 `curl -sL --proxy ... -o`） |

经验：IEEE TII/RA-L 的很多论文**无 OA、无 arXiv**（如 FGIP `10.1109/TII.2023.3342457`、EFP `10.1109/LRA.2024.3363531`）→ 下不到 PDF，只能记 DOI 让用户走学校订阅。**优先挑 arXiv 有预印本的**（例：EPIC `arXiv:2410.14203`、SHIELD `arXiv:2512.23972` 都是 LiDAR 探索，比付费墙文章更贴 LiDAR 平台）。
论文文本提取：`pymupdf`（`fitz`）逐页 `get_text()`；**提取脚本里的路径写成 `__file__` 相对路径**，文件挪目录后仍可用。
