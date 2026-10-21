# SUPER 复飞系统地面验证记录（2026-08-10）

fouruav（Orin NX，<NET_WLAN_A>.45，用户 fouruav/密码 <SSH_PASSWORD>）上 SUPER 复飞系统完整排查记录。

## 环境指纹
- host key: SHA256:<ONBOARD_HOSTKEY_FP>
- plink 连接：`& "$env:TEMP\plink.exe" -ssh -batch -hostkey "SHA256:yRJ..." -pw <SSH_PASSWORD> fouruav@<NET_WLAN_A>.45 -m C:\path\script.sh`
- 复杂/含中文命令一律走 `-m 脚本文件`（PowerShell 引号嵌套会炸）

## 第一块：传感器链路验证结果（全部通过）
- /mavros/state: connected=True, mode=POSCTL
- /livox/lidar: 9.98Hz；/livox/imu_tf: 191.8Hz
- /cloud_registered: 10.04Hz（SUPER 地图输入）
- /aft_mapped_to_init: 9.99Hz（FAST-LIVO2 里程计）
- D435 实际话题是 `/camera/color/image_raw`（不是 /usb_cam/image_raw！后者无数据，但无链路依赖它，不影响）
- check_pose 返回 ERR_Z_LOW 是地面安全闸正常（z<0.4m 拒绝起飞）

## 第二块A：Path Replay 状态机（通过）
- check_replay_status: IDLE → RecordPathReplay → RUNNING_WAITING → AbortReplay → IDLE
- 判据：daemon 进程存在 + /tmp/path_replay_daemon.py 生成 + 结束后 mode 回 POSCTL

## 第二块B：SUPER 链路（通过）
- 启动：`nohup roslaunch task_run_scripts real_flight.launch waypoint_data_path:=<文件> > /tmp/wpt_test.log 2>&1 &`
- 日志预期：`-- [MISSION] Load 83 waypoints.`、`-- [Fsm] WAYPOINT GOAL ENABLE: /waypoint_goal`
- 发目标验证坑1修复：`rostopic pub -1 /waypoint_goal geometry_msgs/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 0.0, z: 1.2}, orientation: {w: 1.0}}}"` → 日志 `Get goal at [1.0, 0.0, 1.2]`

## 录制 NUL 损坏修复
```python
raw = open(path, 'rb').read()
nul_idx = raw.find(b'\x00')
if nul_idx > 0:
    clean = raw[:nul_idx]
    if not clean.endswith(b'\n'):
        last_nl = clean.rfind(b'\n')
        if last_nl > 0: clean = clean[:last_nl+1]
    open(path, 'wb').write(clean)
```
备份：My_Flight_Path.txt.bak_nul

## 抽稀实测
- 1038 点轨迹，waypoint_downsample_dist: 0.1 → Load 83 waypoints；模拟 0.5 → 14 点
- CSV 格式无逐点 switch_dis（用全局 3.0）；空格格式 `x y z switch_dis` 逐点生效（real_mission.txt 第4列=0.5）

## 按钮2 修复 diff（User_define_commands.yaml）
```diff
-  ... pkill -f 'rviz' 2>/dev/null"
+  ... pkill -f 'map_to_world' 2>/dev/null; pkill -f 'top_down.rviz' 2>/dev/null"
```
按钮2 完整语义：先 `rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}'`（&& 链——切失败则不杀，用户选择保留此保守行为）→ SIGINT roslaunch → 兜底杀 SUPER 专属进程。推流/SLAM/MAVROS 不受影响。

## 时间戳归档改造（2026-08-10 完成，第二版）
- nx_command_server.py RecordPathStart 分支：PATH_FILE 动态指向 `data_guidepoints/guidepoints_%Y%m%d_%H%M%S.txt`
- **第二版（最终）**：RecordPathStop 分支同步复制 PATH_FILE → `data_guidepoints/guidepoints_new.txt`（固定文件名始终最新）；按钮3 command 固定 `waypoint_data_path:=.../data_guidepoints/guidepoints_new.txt`（不再用 ls -t）
- 备份：nx_command_server.py.bak_newfile、User_define_commands.yaml.bak_newfile（中间版 .bak_guidepoints 已清）
- 重启服务必须 source ROS 环境（见 SKILL.md 坑#4）
- 目录路径坑：`os.path.dirname(os.path.abspath(__file__))` 拼 data_guidepoints 会落到 NX_code/ 下（`__file__` 在 NX_code 里），需 dirname 套三层回 task_run_scripts 根。判读：看服务日志 `[RecordPathStart] 新航迹文件: <绝对路径>`

## 真机复飞测试（2026-08-10 晚）：起飞悬停成功但复飞中断
现象：切 OFFBOARD → bridge 自动起飞到 0.60m（日志 `Drone reached target altitude of 0.60 meters`）→ 拨 CH10 触发成功（`Mavros triggered.` + `Pub goal to -0.127 0.091 0.554`）→ 第一段轨迹真的飞了（`FOLLOW_TRAJ` + `ReplanOnce succeed`×N）→ 第二段 `[MINCO] TrajOpt failed, The function value became NaN or Inf` 无限重试 → `SUPER command timeout` → 回 WAIT_GOAL → 无指令落地贴地转桨。

根因：FSM `setGoalPosiAndYaw`（fsm.cpp:191）调 `getNearestInfCellNot(OCCUPIED, click_point, gi_.goal_p, 3.0)`——目标点在地图被判占用时被吸附到 3m 内最近空闲格。实测 waypoint_mission 发布 (-0.127, 0.091, 0.554)，FSM 收到 (-0.075, 0.075, 0.825)——三处高度不一致（航点 0.55 / 起飞 0.60 / 吸附后 0.825）→ MINCO 短轨迹优化 NaN。

判读方法：对比日志 `Pub goal to`（发布值）vs `Get goal at`（FSM 吸附后值）。未解决前**起飞前必查 RViz**：1 号蓝点是否落在灰色障碍格里、飞机模型与地图是否对齐（区分真实障碍 vs SLAM/飞控坐标系错位）。

## 排查陷阱（本会话踩过）
1. plink -m 脚本内 pkill/pgrep -f 会匹配到脚本自身命令行（bash -c 整段脚本作为 argv）→ 自杀 exit 128。规避：正则字符类技巧 `nx_command_server[2]\\.py`（模式字符串 ≠ 目标字符串字面量）
2. 前台 roslaunch 输出在终端、nohup 输出在重定向文件——判读前先 `ps -eo pid,lstart,args | grep waypoint_mission` 确认看的是哪个实例，或 `rosparam get /waypoint_mission/` 直接查参数服务器
3. 同名节点顶替（"new node registered with same name"）→ 旧实例全退、RViz 白屏——同时间只跑一个 real_flight.launch
4. RViz 白屏根因链：fsm_node 死 → /fsm_node/rog_map/occ 无数据 → 只有白底网格。先查进程再查话题
5. **"没触发"误判**：grep 触发只看 tail -30 会漏掉日志中部的 `Mavros triggered.`——先 `grep -a 'Mavros triggered.' real_flight.log` 确认，别急着下"没触发"结论（本会话因此误判过一次）
