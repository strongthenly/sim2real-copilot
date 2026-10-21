---
name: super-planner-replay
description: SUPER 规划器轨迹复飞/航点任务系统（fouruav，MID360S+D435+FAST-LIVO2+PX4）：两套复飞架构（Path Replay 与 SUPER 航点任务）、QGC 用户按钮（UserCmd1/2/3）配置与安全关闭、地面验证全流程、常见坑（话题不匹配/进程误杀/NUL 损坏/同名顶替/RViz 白屏）。适用于在 fouruav 上验证/配置/排查轨迹复飞功能。
---

# SUPER 规划器轨迹复飞系统（fouruav）

## 触发条件
- 用户提到"轨迹复飞/航点任务/复飞/SUPER 规划器/按钮1/2/3"或要求验证复飞功能
- fouruav 上 real_flight.launch / waypoint_mission / fsm_node 相关排查
- QGC 自定义按钮（User_define_commands.yaml）配置或修改

## 系统架构：两套独立复飞机制

| 机制 | 入口 | 通道 | 特点 |
|---|---|---|---|
| **Path Replay** | nx_command_server.py 内置 | UDP 6003 指令 + HTTP 6004 状态 | 纯回放 My_Flight_Path.txt，无避障，走 /mavros/setpoint_position/local |
| **SUPER 航点任务** | UserCmd1/3 → real_flight.launch | fsm_node 规划 → /planning/pos_cmd → bridge → setpoint_raw | 规划器+避障，走 OFFBOARD 轨迹跟踪 |

## 端口与通道（重要）
- **UDP 6003**：QGC 按钮指令通道（RecordPathStart/Stop/Replay/AbortReplay、UserCmd1/2/3）——**不是 HTTP POST！**
- **HTTP 6004**：状态查询（/check_pose /check_file /check_replay_status /check_camera）
- **TCP 6000**：rviz_full_stream 全屏推流（遥控器画面来源）
- UDP 6002：触控转鼠标

## QGC 按钮配置（User_define_commands.yaml，改完即生效无需重启服务）
- **按钮1** = 启动 SUPER（默认 real_mission.txt）
- **按钮2** = 关闭 SUPER：先 `rosservice call /mavros/set_mode POSCTL`（**&& 链失败则不杀进程**，用户选择保留此保守逻辑）→ SIGINT roslaunch → 兜底杀 fsm_node/bridge/waypoint/map_to_world
- **按钮3** = 启动 SUPER 复飞：固定加载 `data_guidepoints/guidepoints_new.txt`（始终最新航迹）+ **手动拨杆触发**（start_trigger_type: 1，用户明确要求保留拨杆，不自动触发）

## 录制归档方案（data_guidepoints，2026-08 落地）
- 目录：`task_run_scripts/data_guidepoints/`（**注意在 task_run_scripts 根，不是 NX_code 下**）
- `RecordPathStart` 每次生成**时间戳文件** `guidepoints_YYYYMMDD_HHMMSS.txt`（永不覆盖，历史归档）
- `RecordPathStop` 同步更新 **`guidepoints_new.txt`**（始终=最新航迹，按钮3 固定加载它）
- 实现：改 nx_command_server.py 的 RecordPathStart/Stop 两处（PATH_FILE 动态化 + 复制 new 文件），改完**必须重启 nx_command_server2.py 服务**（安全方式见坑 11/12）
- 旧 My_Flight_Path.txt 保留为历史，不再被录制覆盖
- **录制终点不要落地面**：降落段（z<0.3m）混入航迹 → 复飞冲向地面 + 终点不明显。录制走完轨迹后**保持高度直接 Stop**（用户已接受此习惯，不靠代码过滤）

## 地面验证全流程
1. **第一块·传感器链路**：`bash start_fastlivo2.sh false`（NX_code 目录）→ 6 项检查：`rostopic hz /livox/lidar`(10Hz) /`/usb_cam/image_raw`(30Hz) /`/livox/imu_tf`(200Hz) /`/cloud_registered`(10Hz, 最关键) /`/aft_mapped_to_init` + `/mavros/state`(connected)
2. **第二块 A·Path Replay 状态机**：`curl http://127.0.0.1:6004/check_replay_status` → IDLE；UDP 发 RecordPathReplay → RUNNING_WAITING（不 armed 不会飞，安全）→ AbortReplay → IDLE
3. **第二块 B·SUPER 链路**：启动 real_flight.launch → 日志 `Load N waypoints` + `WAYPOINT GOAL ENABLE` → 发 `/waypoint_goal` 测试目标 → 日志 `Get goal at [x,y,z]`（验证高度不被覆盖）→ `/planning/pos_cmd` 有输出
4. **RViz 轨迹验证**：top_down.rviz 显示 Occ 占据地图 + LocalCloud + frontend_path；推流截全屏，遥控器可见。**拨杆后出现轨迹线 = 复飞执行中**

## 关键坑（全部实测踩过）
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

## 航点文件地图
| 文件 | 用途 | 备注 |
|---|---|---|
| `data_guidepoints/guidepoints_new.txt` | **按钮3 固定加载（始终最新）** | CSV 7列，RecordPathStop 时同步 |
| `data_guidepoints/guidepoints_YYYYMMDD_HHMMSS.txt` | 历史归档（每次录制一份，不覆盖） | 时间戳排序，可按需回溯 |
| `scripts/NX_code/My_Flight_Path.txt` | 历史数据源（不再被覆盖） | 旧 PATH_FILE，保留作参考 |
| `data/real_mission.txt` | 按钮1 默认航点 | 空格 4列 x y z 切换距离 |
| `SUPER/mission_planner/data/*.txt` | 源码副本 | 实际运行不读这里 |
| 旧脚本写死 `~/odin_ws/src/task_run_scripts/waypoints.txt` | record.py / task_node_nav | **目录不存在，已失效** |

## 用户偏好（必须遵守）
- **保留手动拨杆触发**航点任务（start_trigger_type: 1），按钮只负责启动系统
- **改核心脚本要受控**（2026-08 演变确认）：用户最初说"不要写在脚本中这个逻辑"（指不想为归档需求写独立重命名脚本），但**接受对 nx_command_server.py 的最小侵入修改**（"OK，你先改，我测试"）。前提：先备份 + 改完贴 diff + py_compile 语法检查 + 改完重启服务验证。独立归档脚本方案（重命名思路）被用户否决，改为直接改 RecordPathStart/Stop 生成时间戳文件
- 改任何文件先备份 + 贴 diff 透明汇报；测试由用户执行，agent 盯日志判读
- 指挥官模式：命令一次 1-3 条 + 预期输出

## 路径 A：全局路径引导（2026-08-11 落地，分支 feat/4uav-guide + SUPER dev/4uav）
**目标**：无人机沿录制航迹（全局路径）飞行 + 动态避障，代替"逐点航点任务"。
- **数据链**：`guidepoints_new.txt` → global_path_publisher.py → `/global_path`(nav_msgs/Path 全量点) + `/global_path_marker`(MarkerArray 绿线, RViz 显示用) + `/waypoint_goal`(终点 PoseStamped)
- **SUPER 源码改动**（3 文件，全部有 .bak_globalpath 备份）：
  - `fsm_ros1.hpp`：订阅 `/global_path` → globalPathCallback → `planner_ptr_->setGlobalPath()`
  - `super_planner.h`：成员 `global_path_/global_path_valid_/global_roll_dist_/global_used_prev_/global_skip_this_round_` + setter
  - `super_planner.cpp` generateExpTraj：guide_path = 全局路径[最近点→终点]（全程不截断）；走廊失败 → fallback A*（global_skip_this_round_ 本轮跳过，下轮自动恢复）
- **纯全局路径模式**：按钮3 加 `use_waypoint:=false`（real_flight.launch 已有该 arg）→ 禁用 waypoint_mission 逐点发 goal，只靠 global_path_publisher 发**终点** goal → 消除"逐点切换与全局路径打架"（表现为飞机乱转）
- **常见失败链（GeneratePolytopeFromLine failed）**：全局路径附近地图被障碍/人占用 → 走廊生成失败 → PlanFromRest failed 循环重试 → bridge 超时切 POS_HOLD。**测试时人必须远离飞机 2m+**，起飞前雷达转一圈清地图
- **沿轨迹程度**：走廊沿全局路径生成 + MINCO 在走廊内平滑（拐角圆滑化，非严格贴折线）；障碍时 A* 绕行（偏离但安全）；**起飞点必须对齐航迹起点**否则先飞向最近点

## 关键坑（续）
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

## 日志分析方法（实飞后判读）
1. 事件时间线：`grep -aE "Mavros triggered|Pub goal|Get goal|FOLLOW_TRAJ|NONEED|POS_HOLD|timeout|OptimizationExp|SUCCESS" log/real_flight.log | tail -25`
2. 飞行轨迹：`grep -a "POS+VEL" log/real_flight.log | tail -20`（x/y/z/vx/vy/vz 是否沿轨迹）
3. 对照模板：启动→起飞（Publishing initial setpoints→OFFBOARD engaged→reached altitude）→触发（Mavros triggered→Pub goal→Get goal）→规划（GenerateExpTrajectory SUCCESS→FOLLOW_TRAJ）→飞行（ReplanOnce succeed + POS+VEL 沿轨迹）→停（NONEED / OptimizationExp failed / SUPER command timeout→POS_HOLD）
4. 停止原因三连：`close to goal`(认为到了) / `OptimizationExpTrajInPolytopes failed`(走廊优化失败) / `SUPER command timeout`(bridge 0.5s 无指令→POS_HOLD 安全兜底)

## 参考
- references/replay-system-audit.md —— 代码级审计：话题映射表、bridge 状态机、waypoint.yaml 注释翻译、抽稀验证（1038→83@0.1m）
- references/global-path-guide-mode.md —— 路径A 全局路径引导：代码改动明细、注入逻辑、fallback、纯全局路径模式、故障链
- NX_code/obstacle_warning.md —— PX4 避障参数（CP_DIST/CP_GO_NO_DATA）无数据源导致 POSCTL 锁死的完整排障记录
