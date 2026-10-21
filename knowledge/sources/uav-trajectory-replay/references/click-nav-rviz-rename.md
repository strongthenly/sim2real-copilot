# 点击导航模式 + RViz 同名互杀（2026-08-25，fouruav）

## 背景
用户拍板："智能航线任务" 从"沿录制航迹复飞"改为 **SUPER 点击导航**（RViz 2D Nav Goal 点哪飞哪）。同一天发现并修复了"启动智能航线任务后雷达界面退出"的 RViz 同名节点互杀问题。

## 点击导航模式切换（改 2 个文件，均在 task_run_scripts）

SUPER 原生支持点击导航：fsm_node 订阅 `click_goal_topic`（默认 `/move_base_simple/goal`，与 RViz 2D Nav Goal 工具输出话题一致）→ `setGoalPosiAndYaw()` → `gi_.new_goal=true` → A*+MINCO 自主规划避障。

| 文件 | 改动 | 作用 |
|---|---|---|
| `config/odin_real_flight.yaml` | `waypoint_goal_en: true → false` | 关闭复飞航点订阅（fsm 不再听 /waypoint_goal） |
| `launch/real_flight.launch` | `<arg name="use_waypoint" default="true"/> → false` | 不启动 waypoint_mission 节点 |

保留不动：`click_goal_en: true`（默认已开）、`click_goal_topic: "/move_base_simple/goal"`。

链路：RViz 2D Nav Goal → `/move_base_simple/goal` → fsm（自主规划）→ `/planning/pos_cmd` → super_to_px4_bridge → setpoint → PX4。

复飞（CH9 返航版发布器）保留在 `feat/long-distance-replay` 分支；main 上是点击导航。配置出处：`SUPER/super_planner/config/real_flight.yaml` 模板里有 `click_goal_topic` / `waypoint_goal_en` / `waypoint_goal_topic` 三开关。

验证：`grep -nE "waypoint_goal_en|click_goal_en" config/odin_real_flight.yaml` → 见 `waypoint_goal_en: false`、`click_goal_en: true`。

## RViz 同名节点互杀（雷达界面消失）

**症状**：start_odin.sh 的雷达界面（odin_rviz_me.rviz，name=`rviz`）正常显示；点"智能航线任务"后雷达界面消失，日志：
```
[WARN] Shutdown request received.
[WARN] Reason given for shutdown: [[/rviz] Reason: new node registered with same name]
```

**根因**：`real_flight.launch` 的规划 RViz 节点也叫 `name="rviz"`：
```xml
<node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rviz_config)" output="log"/>
```
ROS master 不允许同名节点共存 → 新 rviz 注册时旧 rviz（雷达界面）被强制 Shutdown。平板推流画面随之断（rviz_full_stream2.py 推的正是被杀的那个 rviz 画面）。注意：规划/避障进程（fsm_node/bridge/waypoint_mission）本身都活着，只有雷达 rviz 死。

**修复**：改名 `name="rviz_super"`，备份 `launch/real_flight.launch.bak_rviziname_<ts>`，XML 校验：
```bash
python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('launch/real_flight.launch'); print('OK')"
```
效果：雷达界面与规划视图两个 RViz 共存。改 launch 文件即时生效（roslaunch 每次读文件），无需重启服务/重编译。

**推流无副作用确认**：`scripts/NX_code/rviz_full_stream.py` 用 `mss` 整屏截图（`sct.grab(monitor)`，monitor 级别，靠 `LIYUMEN_MONITOR_INDEX` 环境变量选屏），**不按节点名/窗口名抓取** → 改规划 rviz 节点名不影响平板推流。

## 顺带发现（未处理）
odin1_ros1.log 刷 `Queue PC2XYZRGBA is full, dropping oldest data`（odin SDK 回调队列满，点云消费不过来）—— 仅丢帧警告不影响功能；若后续画面卡顿再查订阅压力。
