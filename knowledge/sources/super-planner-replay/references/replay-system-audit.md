# SUPER 复飞系统代码级审计（2026-08-10 实测）

## 1. 话题/配置映射表（实测核对）

| 环节 | 配置值 | 文件位置 |
|---|---|---|
| waypoint_mission 发布 | goal_pub_topic: `/waypoint_goal`（task_run_scripts 版已修复） | `task_run_scripts/config/waypoint.yaml` |
| FSM click 通道 | click_goal_topic: `/move_base_simple/goal`（override_height=true） | `super_planner/config/real_flight.yaml` |
| FSM waypoint 通道 | waypoint_goal_topic: `/waypoint_goal`（override_height=false） | 同上 |
| ROG-Map 点云输入 | rog_map.ros_callback.cloud_topic: `/cloud_registered`（FAST-LIVO2 版已适配） | 同上（task_run_scripts 版） |
| ROG-Map 里程计 | odom_topic: `/mavros/local_position/odom`（飞控自定位，非 SLAM） | 同上 |
| 前馈参数 | use_velocity_ff/use_accel_ff/velocity_ff_gain（默认全开 gain 0.7） | `super_planner/config/super_to_px4_bridge.yaml` |
| 起飞高度 | init_goal/z: 1.2 | bridge yaml |
| 航点切换 | switch_dis: 3.0（全局默认，米）；文件第4列逐点覆盖 | waypoint.yaml + real_mission.txt |

## 2. bridge 状态机（super_to_px4_bridge.cpp，Phase 1-5）

```
Phase 1: 预发布位置指令（PX4 要求切 OFFBOARD 前有指令流）→ 持续发 init_goal
Phase 2: 自动解锁+切 OFFBOARD（auto_arm/auto_offboard=false 时跳过，手动）
Phase 3: 等待 OFFBOARD 生效（持续发 init_goal）
Phase 4: 🚁 自动起飞悬停——发 init_goal 直到 |z-1.2|<0.1m，30s 超时继续
Phase 5: 主循环——super_cmd_active 且未超时则跟踪 /planning/pos_cmd
         超时/RC丢失/越界/MAVROS断连 → 位置保持或降落
```
- 关键：起飞悬停由 bridge 自动做，**与 CH10 拨杆无关**（CH10 只管 waypoint_mission 何时发航点）

## 3. CH10 触发逻辑（ros1_waypoint_planner.hpp）

```cpp
// start_trigger_type: 0=RViz点击 / 1=RC ch10 / 2=定时
bool pitch_up = msg->channels[1] < 1200;          // CH2(pitch) < 1200 才允许
if (last_ch_10 > 1500 && ch_10 < 1500) {          // 下降沿触发
    triggered = true; waypoint_counter = 0;
}
```
**CH10 停在低位不会触发**——必须"高位→低位"拨动一次。

## 4. waypoint.yaml 参数注释翻译

- `start_trigger_type: 1` = RC ch10 触发（2=启动后 start_program_delay 秒自动触发）
- `cmd_type: 1` = 航点模式（0=路径）
- `switch_dis: 3.0` = 距航点 3m 内算到达（米，文件第4列覆盖）
- `publish_dt: 99999` = 每个航点只发布一次
- `waypoint_downsample_dist` = 抽稀阈值（0=不抽稀；CSV 格式生效）

## 5. 航点文件格式

- **空格格式**（real_mission.txt）：`x y z switch_dis`，每行第4列=该航点切换距离（逐点独立）
- **CSV 格式**（My_Flight_Path.txt）：`x,y,z,qx,qy,qz,qw` 7列，含四元数；无第4列→用全局 switch_dis；支持表头行跳过；waypoint_downsample_dist 在此格式生效

## 6. 抽稀验证结果（实测）

- 0.1m 抽稀：1038 → **83 点**（点间距 ~0.1m 均匀）
- 模拟 0.5m：1038 → 14 点
- 日志证据：`Load 83 waypoints.` + 逐点 `Waypoint N at (x,y,z) Switch dis = 3`
- 加载验证：`roslaunch task_run_scripts real_flight.launch waypoint_data_path:=/path/to/file.txt`

## 7. NUL 损坏修复脚本（录制文件尾部 \x00）

```bash
python3 - <<'EOF'
raw = open(PATH, 'rb').read()
nul = raw.find(b'\x00')
if nul > 0:
    clean = raw[:nul]
    if not clean.endswith(b'\n'):
        nl = clean.rfind(b'\n')
        if nl > 0: clean = clean[:nl+1]
    open(PATH, 'wb').write(clean)
    print(f"截断 {len(raw)} -> {len(clean)} 字节")
EOF
```
先备份再修。根因：RecordPathStop 用 terminate() 杀录制进程，最后一行写一半。

## 8. 按钮2 修复对比（防误杀）

```diff
- pkill -f 'rviz' 2>/dev/null
+ pkill -f 'map_to_world' 2>/dev/null; pkill -f 'top_down.rviz' 2>/dev/null
```
`pkill -f 'rviz'` 会命中：rviz_full_stream2.py（推流，遥控器黑屏）、roslaunch mapping_MID360.launch（命令行含 "rviz:=false"）。
修复后按钮2 只杀 SUPER 本体（fsm_node/bridge/waypoint_mission/map_to_world/top_down.rviz），不碰推流/SLAM/MAVROS。

## 9. RViz 白屏诊断链

1. 先查 `ps aux | grep fsm_node` 是否存活（**同名顶替**会导致 launch 全死：日志 "Reason: new node registered with same name"）
2. 再查 `rostopic hz /fsm_node/rog_map/occ` 有无数据（无数据=地图空=白屏）
3. 确认 `xwininfo -root -tree` 里 top_down.rviz 窗口在最上层（推流截全屏，多个 RViz 会叠窗口）
4. 最后才查 Fixed Frame/Views 配置

## 10. 环境事实

- fouruav X server :0 可用（Xorg tty2），推流 rviz_full_stream2 常驻（6000 端口，72% CPU）
- start_fastlivo2.sh 在 `task_run_scripts/scripts/NX_code/`（不在 home 下），用 DISPLAY=:0
- nx_command_server2.py 只是包装器（11行），真实逻辑在 nx_command_server.py（2576行）
- 用户遥控器 = <NET_RC>.100（nx_ping_QGC.py 硬编码），项目名"<PROJECT_CODENAME>X8"；前端弹窗在遥控器端，fouruav 上无前端代码
