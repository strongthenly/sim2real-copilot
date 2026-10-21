# 仿真版 RViz（点云 + rog_map + 规划轨迹）与 world↔camera_init TF

2026-09-22 实测。目标：在 `ros1-twin` + AirSim + FAST-LIVO2 仿真里，**一个 RViz 窗口**同时看到
FAST-LIVO2 点云、SUPER 的 rog_map 占据/未知/膨胀地图、以及前端/后端/MINCO 规划轨迹，
与实机 `real_flight.launch` 弹出的界面一致。

---

## 1. 实机那份配置长什么样（`task_run_scripts/rviz/top_down.rviz`）

`real_flight.launch` 里的 RViz 节点：
```xml
<arg name="rviz_config" default="$(find task_run_scripts)/rviz/top_down.rviz"/>
...
<!-- 静态 TF: map -> world (frame_id 不要加前导斜杠，否则与 map/odom 树断开) -->
<node pkg="tf" type="static_transform_publisher" name="map_to_world"
      args="0 0 0 0 0 0 map world 1000"/>
<!-- RViz (实机模式) -->
<node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rviz_config)" output="log"/>
```
> 注意：**实机也只有 `map→world` 这一条静态 TF**，没有 `world↔camera_init`。

`top_down.rviz` 的启用显示项（`Fixed Frame: world`，白底）：

| 组 | 显示项 | 话题 |
|---|---|---|
| Sensing | PointCloud2 `/Sensing/GlobalCloud` | `/odin1/cloud_slam` |
| Sensing | PointCloud2 `/Sensing/Occ` | `/fsm_node/rog_map/occ` |
| Sensing | PointCloud2 `/Sensing/Unknown` | `/fsm_node/rog_map/unk` |
| Sensing | PointCloud2 `/Sensing/InflatedCloud` | `/fsm_node/rog_map/inf_occ` |
| Sensing | Odometry `/Sensing/Odometry` | `/mavros/local_position/odom` |
| Frontend | MarkerArray `/Frontend/{Astar,Goal,GlobalPathMkr}` + Marker `/Frontend/FrontendPath` | — |
| Backend | MarkerArray `/Backend/{ExpTraj,BackupTraj}` + Path `/Backend/Path` | `/fsm_node/fsm/path` |
| TrajOpt | MarkerArray `/TrajOpt/{BackUp,Exp,Yaw}` | — |
| — | Path `/Path` | `/waypoint_mission` |

同目录另有 7 份：`slam_rviz.rviz` / `odin_rviz_me.rviz`（Fixed Frame `odom`）/ `mid360_top_down.rviz` /
`obstacle_test{,_mid360}.rviz` / `real_svc.rviz` / `fast_livo2.rviz`。

---

## 2. 为什么实机那份在仿真里一帧都不显示

两个叠加原因，**只修一个还是空白**：

**① 话题是实机残留**
```
/Sensing/GlobalCloud  →  Topic: /odin1/cloud_slam      ← 仿真里不存在这个话题
```
仿真点云来自 FAST-LIVO2，话题是 **`/cloud_registered`**。

**② Fixed Frame 与点云 frame 之间没有 TF**
```
/cloud_registered 的 header.frame_id = "camera_init"
top_down.rviz      Global Options.Fixed Frame = "world"
⇒ 缺 world→camera_init，RViz 无法变换 ⇒ 静默空白
```

### TF 链实测（别想当然，逐条查过）
```bash
# 谁提供 TF（结果：只有 map→world，没有 world↔camera_init）
grep -rn "static_transform_publisher" ~/super_sim_ws/src/task_run_scripts/launch/*.launch
#   real_flight.launch:34  args="0 0 0 0 0 0 map world 1000"

# fuel_input_bridge 只是改写【它自己发的】消息，不动 FAST-LIVO2 原始消息
grep -n "frame_id\|camera_init\|world" ~/fuel_ws/src/fuel_bridge/scripts/fuel_input_bridge.py
#   WORLD = "world"
#   处理: 把 frame_id 从 camera_init 统一改写成 world（FUEL 内部约定）
#   msg.header.frame_id = WORLD

# twin_bridge 不发任何 TF；各 launch 也没有 world↔camera_init
```
对应地，运行时 `/tf` 只有 `camera_init → aft_mapped`，`/tf_static` 只有
`map→map_ned` / `odom→odom_ned` / `base_link→base_link_frd`。

**结论：`camera_init` 是孤儿帧**，仿真里没有任何节点把它接到 `world` 上。

---

## 3. 修法

### 3.1 补 TF（一条命令，零偏移）
仿真配置 `sim_real_flight.yaml` 里 `fix_map_origin: [0,0,0]` → 两个 frame 实际重合：
```bash
setsid rosrun tf static_transform_publisher 0 0 0 0 0 0 world camera_init 100 \
    > /tmp/tf_world_caminit.log 2>&1 < /dev/null &
```
配套在启动脚本里做幂等判断 + 自检：
```bash
if ! pgrep -f "static_transform_publisher.*world camera_init" > /dev/null 2>&1; then
  setsid rosrun tf static_transform_publisher 0 0 0 0 0 0 world camera_init 100 ... &
  sleep 1
  pgrep -f "static_transform_publisher.*world camera_init" >/dev/null 2>&1 \
    && say "     [OK] 已补 world->camera_init 静态 TF" \
    || say "     [!] TF 未起，看 /tmp/tf_world_caminit.log"
fi
```
> ⚠️ **z 值与 `fix_map_origin` 绑定**：若把它改回 `[0,0,1.5]`（雷达安装高），
> 这条 TF 的 z 必须同步改成 `1.5`，否则点云整体错位 1.5m。

### 3.2 生成仿真版 rviz
从实机版派生：只改**点云话题**与**各点云项的 `Decay Time`**，另降 `Frame Rate`：

| 项 | 实机值 | 仿真值 | 理由 |
|---|---|---|---|
| `/Sensing/GlobalCloud` Topic | `/odin1/cloud_slam` | **`/cloud_registered`** | 仿真点云源 |
| `/Sensing/GlobalCloud` Decay Time | 0 | **1** | 0 = 每帧独立 → 丢帧就闪（坑 L-1） |
| `/Sensing/{Occ,Unknown,InflatedCloud}` Decay Time | 0 | **2** | 同上，地图项也给点累积 |
| Global Options Frame Rate | 30 | **15** | 点云本身才 10 Hz，30 帧纯浪费 |

**Fixed Frame 保持 `world`**（补了 TF 之后就是对的）。

生成脚本：`scripts/make_sim_rviz.py`（可重跑，源配置变更后重新派生）。
**必须用 `yaml.safe_load` → 改 → `yaml.safe_dump(sort_keys=False)`**，不要按行文本替换 ——
RViz 配置里 `Decay Time:` 这种键在多个显示项里重复出现，按块边界定位极易改错项
（本次第一版就把 `GlobalCloud` 的 decay 和 `Occ` 的改了互相覆盖，读回校验才发现）。

### 3.3 启动脚本里起它（坑 N 的落地）
```bash
pkill -x rviz 2>/dev/null      # ★ 先关 FAST-LIVO2 那个，避免两个 RViz 抢渲染 / 同名顶替（坑51）
sleep 1
RVIZ_CFG="$(rospack find task_run_scripts 2>/dev/null)/rviz/sim_top_down.rviz"
[ -f "$RVIZ_CFG" ] && setsid rviz -d "$RVIZ_CFG" > /tmp/rviz_super.log 2>&1 < /dev/null &
sleep 3
pgrep -x rviz >/dev/null 2>&1 && say "[OK] RViz 已起" || say "[!] RViz 未起，看 /tmp/rviz_super.log"
```
`--no-rviz` 开关即跳过这一段（无头跑 / 省 GPU）。脚本里那条 `rospack find` 依赖
`super_sim_ws` 已 source —— 见坑 G（多工作区要手动叠加 `ROS_PACKAGE_PATH`）。

---

## 4. 验证判据

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311

# ① 点云 frame
rostopic echo -n1 /cloud_registered/header/frame_id        # → "camera_init"

# ② TF 通不通（补之前是 "does not exist"）
rosrun tf tf_echo world camera_init                        # → 有平移/旋转输出

# ③ 地图话题有数据（旧 skill 记过：RViz 白屏先查 occ）
timeout 6 rostopic hz /fsm_node/rog_map/occ

# ④ 轨迹话题存在
rostopic list | grep -E "fsm_node/visualization|fsm_node/fsm/path"

# ⑤ 配置自校验（YAML 层面，不依赖 RViz 界面）
python3 - <<'PY'
import yaml
d = yaml.safe_load(open('/home/<USER>/super_sim_ws/src/task_run_scripts/rviz/sim_top_down.rviz'))
vm = d['Visualization Manager']
want = {'GlobalCloud': ('/cloud_registered', 1),
        'Occ': ('/fsm_node/rog_map/occ', 2),
        'Unknown': ('/fsm_node/rog_map/unk', 2),
        'InflatedCloud': ('/fsm_node/rog_map/inf_occ', 2)}
got = {}
def walk(ds):
    for it in ds or []:
        if it.get('Class') == 'rviz/PointCloud2' and it.get('Enabled'):
            got[it.get('Name')] = (it.get('Topic'), it.get('Decay Time'))
        if it.get('Displays') is not None: walk(it['Displays'])
walk(vm.get('Displays'))
print('Fixed Frame:', vm['Global Options'].get('Fixed Frame'))
for k, v in want.items():
    print(('  OK  ' if got.get(k) == v else '  BAD ') + k, got.get(k))
PY
```
预期：Fixed Frame `world`；四项话题/decay 全部符合。

---

## 5. 这套检查已并进 canonical 测试

`~/fuel_ws/src/fuel_bridge/test/smoke_test.sh` 的 **G 段**（10 项）覆盖：
三个脚本语法、`world->camera_init` TF 存在、使用 `sim_top_down.rviz`、
`--print-params` 的 `rviz=1` / `--no-rviz` 的 `rviz=0`、`port_open` 含 `ss -tuln`、
停止脚本含 PCD 归档、以及上面的 rviz YAML 校验。
→ `cd ~/fuel_ws/src && make test` 一把全过（`MAKE_TEST_EXIT=0`）。
