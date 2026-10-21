# ROS：节点都在、话题都在，但【没有数据在流】—— 话题级数据流验证法

> 2026-10-08 在 fouruav 上定位「FUEL 一切看起来正常却永远不工作」时固化。
> 适用于任何 ROS1/ROS2 系统：只要出现**节点起得来、`rostopic list` 有、launch 不报错，
> 但功能就是不动**，先跑这一套，再谈算法/参数/调参。

## 0. 铁律：node up ≠ data flowing

`rosnode list` 里有 · `rostopic list` 里有 · `roslaunch` 不报错 —— 这三件事
**一个都不能证明数据在流**。决定性判据只有两个：

| 判据 | 命令 | 危险信号 |
|---|---|---|
| 有没有人**发** | `rostopic info <t>` | **`Publishers: None`** |
| 有没有**真数据** | `rostopic hz <t>` | 采不到 average |

★ 本次实例：`/map_ros/depth` 话题存在、`exploration_node` 订阅了它、
  launch 里 remap 也写得好好的 —— 但 **`Publishers: None`**。
  于是下游建图永远是空的、探索永远不动，**而全程没有任何报错**。
  （真因见下 §3：仿真里这张深度图是仿真器免费给的，实机没人接。）

## 1. 三步法：从「消费者」的嘴往回查

```bash
# ① 消费者订阅/发布什么 —— 这就是"应该有什么"的权威清单
rosnode info <consumer_node> | sed -n '/Publications/,/Services/p'

# ② 逐个话题看有没有发布者
for t in <话题清单>; do echo "== $t"; rostopic info "$t" | sed -n '1,10p'; done

# ③ 逐个话题看有没有真数据
for t in <话题清单>; do
  printf "%-46s " "$t"
  timeout 5 rostopic hz "$t" 2>&1 | grep -m1 average || echo "(无数据)"
done
```

**判读表**（照这个下结论，别凭感觉）：

| Publishers | hz | 结论 / 下一步 |
|---|---|---|
| `None` | — | **静默杀手**：没人发 → 下游永远空。去查 launch 的 remap/param 指向、以及"谁本该发它" |
| **多个** | 有数 | ★ **第二个隐形杀手**：你以为只有一个发布者，实际有别人在**抢同一个话题**、把下游牵走。见 §0.1 |
| 有 | 有数 | 这一环正常 |
| 有 | 没数 | 发布者活着但不产出 → 继续往**它的**输入查（递归用第 ① 步） |
| — | `Unknown topic` | 话题名不存在。**如果刚改过 remap，这是成功信号**（见下）★ |

★ **反直觉但很有用**：改对 remap 之后，**旧话题名会报 `Unknown topic: /old_name`**。
  那不是错误，恰恰是"remap 生效了"的证据 —— 本次就是靠它确认深度图改接成功。

### 0.1 多个发布者抢同一话题（"只做加法"时最容易踩）

**判据不止"有没有人发"，还要数"有几个在发"**：
```bash
rostopic info <下游的输入话题> | sed -n '/Publishers/,/Subscribers/p'
```
**必须只有你预期的那一个。** 多一个就是有人在抢。

**典型场景（本次实测）**：往既有系统里"只做加法"地接新功能时，
**照抄了原来的启动命令** —— 而那条命令里带的节点**也在往同一个话题发**：
```
新架构：FUEL → /exploration_goal → 仲裁器 ─┐
既有节点：global_path_publisher ───────────┤→ /waypoint_goal → fsm_node
                                            ✗ 两个发布者，fsm 被固定航点牵走
症状：「按了触发但没反应 / 没有出现轨迹」，日志里下游在执行【另一条】任务
```
**⇒ 抄既有启动命令之前，先 `rosnode info <那些节点>` 看它们往哪些话题发**，
  确认不会撞上你新层要用的那个入口；不需要的一律**不要启动**。

### 0.2 启动脚本必须【幂等】

重复执行同一个一键脚本（或用户连按两次按钮）时，**同名节点会被后来者顶掉**：
```
shutdown request: [/goal_arbiter] Reason: new node registered with same name
⇒ 活下来的是第二次起的那个；如果两个实例的参数不同，你会得到一个【用错参数】的实例
  （本次表现为\"watch 模式失效、永远等不到触发\"，但从日志上看节点是\"活着\"的）
```
**⇒ 脚本开头先按精确节点名清一遍旧实例，再起新的**（顺手也解决了连按两次）。
**⇒ 起完之后用 `--status` 子命令回读关键状态**（节点齐不齐 + 关键参数有没有生效），
  不要假设"启动了 = 用对了参数"。

## 2. 把结果铺成一张「消费者视角」对照表

把 launch/配置里引用的**每一个**话题列成表，填两列「有没有发布者 / 有没有数据」，
一次填完再动手。本次就是靠这张表一眼看出 FUEL 只差一个输入，避免了一轮轮猜。

## 3. 传感器输入的「隐藏依赖」：仿真里免费的东西，实机必须显式接上

搬仿真栈到实机时最容易漏的一类：**仿真环境里由仿真器免费提供的传感器输入**。
本次是**同一个根因的两个独立实例**（一个吵一个静）：

| 仿真里谁提供 | 实机缺什么 | 症状 |
|---|---|---|
| `uav_simulator/Utils/waypoint_generator`（一个包）| 整包没拷 | `roslaunch` 起不来（**吵**，好发现）|
| `uav_simulator/pcl_render_node` 渲染的**深度图** | 深度图没人发布 | 一切正常但功能不动（**静**，最难发现）|

**⇒ 规程**

1. **排除任何包之前**，先把所有 launch 里的包引用抓出来逐个验证：
   ```bash
   grep -rhoE 'pkg="[A-Za-z_]+"' <launch 目录> | sort -u          # <node pkg="...">
   grep -rhoE '\$\(find [A-Za-z_]+\)'  <launch 目录> | sort -u    # $(find ...)
   # 逐个: rospack find <pkg>
   ```
   任何一个 `Missing` 都会让 launch 直接失败、或**悄悄少一路输入**。
2. **落地（拷完/编完）之后再跑一遍**同样的检查。
3. **别只盯包** —— 逐节点问「它靠什么传感器输入」，再去实机确认那路输入**存在且有人发布**。
   本次：FUEL 建图靠深度图 → 实机 D435 现成有 → 把 `depth_topic` 直接指过去，
   **一行 remap 解决，不用写任何代码**。
4. **输入格式必须对齐**（不然接上了也不对）：encoding / 单位 / 内参 / frame。
   本次实测正好匹配，可直接抄：D435 `16UC1`(uint16) ↔ FUEL 按 `uint16_t` 读，
   `k_depth_scaling_factor=1000` 正好做毫米→米；`depth_filter_maxdist 5.0` 匹配 D435 量程；
   内参用 `/camera/depth/camera_info` 里的实测值（原配置里那份是仿真相机的，差约 2%）。

## 4. 移植过来的 RViz / 参数文件：话题名必须对【这一版】运行系统核对

从仿真拷来的 `.rviz` / 参数文件，话题名**常常和实机这一版不是同一套**。
本次 `exp.rviz` 里**一半话题在实机上不存在**（真名在别处，例如
`/sdf_map/occupancy_inflate` 实际叫 `/sdf_map/occupancy_local_inflate`；
真正的"前沿"可视化是 `/planning_vis/frontier`，配置里根本没配）。

**⇒ 流程**：
1. 先用 §1 的 `rosnode info <node>` **Publications** 拿到真名清单；
2. 逐个 `rostopic hz` 确认真有数据；
3. 再改配置 —— 存在的接上，**不存在的一律 `Enabled: false`**（否则左侧显示树被
   一堆 "No message received" 撑爆）；
4. 改 `.rviz` 用**文本级替换/插入**，别 `yaml.load` → `yaml.dump` 往返
   （rviz 是 Qt 类型化 YAML 读取器，往返会重排/改类型；而文本改动其余字节不动最安全）。
5. 改完先用 `QT_QPA_PLATFORM=offscreen rviz -d x.rviz` 验一遍能不能加载
   （详见 `ros-startup-and-rviz-diagnostics.md` §5）。
6. **rviz 只在启动时读配置** —— 改完必须重启 rviz 进程，或让操作员重按一次启动按钮。

## 5. 判据要能在【地面】验证，别把验证拖到实飞

上面所有修复本次都在**地面**就验完了（飞机没起飞）：

```bash
rostopic info <输入话题> | sed -n '1,10p'    # Subscribers 里应出现那个消费者 ✓   ← 接上了
rostopic hz  <下游产物话题>                   # 应出数据（本次 /sdf_map/depth_cloud 10Hz）✓
grep -a "<节点自己的健康关键字>" <log>         # 应持续打印（本次 "Frontier t:" 每 5s 一轮）✓
```

★ **别把「功能不动」当成「配置没生效」**：本节点自己打印的健康日志（如计算耗时、
  前沿计数）比看话题更直接，是最快能判断"算法到底跑没跑"的证据。
★ 同理，反过来说：**有些现象是"没动"的正常结果**，不是故障 ——
  例如飞机不移动时地图不增长 ⇒ 前沿全部休眠 ⇒ FSM 判 `FINISH`、可视化停在最后一帧。
  判断这类问题要**先问"这个状态是不是本来就该这样"**，再往下查
  （见 `fuel-exploration-aircraft-deployment.md` §21）。
