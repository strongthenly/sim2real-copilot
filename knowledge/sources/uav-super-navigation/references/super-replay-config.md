# SUPER 复飞配置细节（fouruav 实测，2026-08-10）

## 数据链路总览
```
MID360S → msg_MID360s.launch → /livox/lidar + /livox/imu_tf
D435 → rs_d435_camera.launch → /camera/color/image_raw（注意不是 /usb_cam/image_raw！）
FAST-LIVO2 (mapping_MID360.launch) → /cloud_registered（点云, ~10Hz）
                                    └→ /aft_mapped_to_init（里程计, ~10Hz）
PX4 → px4.launch → MAVROS (/mavros/*, 飞控自定位)
```
- SUPER 的 rog_map `cloud_topic: /cloud_registered`（task_run_scripts 版已适配）
- SUPER 的 `odom_topic: /mavros/local_position/odom`（**用飞控自定位，不是 SLAM odom**；odom_to_mavros 转发在 px4_and_others.launch 里被注释）
- FAST-LIVO2 视觉里程计未接入（img_topic 指向 /usb_cam/image_raw 但 D435 实际发 /camera/color/image_raw）——不影响 LiDAR+IMU 模式

## 航点文件格式（config.hpp LoadWaypoint 实测）
- **空格格式** `x y z switch_dis`：第4列=该点切换距离（米），逐点独立。real_mission.txt 用 0.5
- **CSV 格式** `x,y,z,qx,qy,qz,qw`（7列）：无切换距离列 → 用 yaml 全局 `switch_dis`（3.0）
- CSV 表头（含字母行）自动跳过
- `waypoint_downsample_dist` 抽稀只在 CSV 分支生效；0=不抽稀
- 抽稀实测：1038 点 @0.1m → 83 点；@0.5m 模拟 → 14 点

## waypoint_mission 触发逻辑（ros1_waypoint_planner.hpp）
```cpp
start_trigger_type: 0 = RViz /goal 点击; 1 = RC CH10 下降沿; 2 = 定时(start_program_delay 秒后)
// CH10 触发条件（代码 68-80 行）：
last_ch_10 > 1500 && ch_10 < 1500   // 下降沿，先拨高再拉低
pitch_up = msg->channels[1] < 1200  // 附加条件：CH2 < 1200
// 切换判断（49-55 行）：3D 欧氏距离 < switch_dis_vec[id]（文件第4列优先，无则 yaml 3.0）
```

## bridge 状态机（super_to_px4_bridge.cpp）
```
Phase 1: 预发布 init_goal 指令流（PX4 要求切 OFFBOARD 前已有指令流）
Phase 2: auto_arm / auto_offboard（默认 false=手动解锁+切模式）
Phase 3: 等 OFFBOARD 生效（期间持续发 init_goal）
Phase 4: 🚁 自动起飞到 init_goal z=1.2m（|当前z - 1.2| < 0.1m 或 30s 超时继续）
Phase 5: 主循环：super_active = super_cmd_active && (now-last_super_cmd) < timeout_threshold(0.5s)
         RC丢失/MAVROS断连 → 位置保持; geofence 越界 → 位置保持+降落
         降落: /bridge/land → land_speed 0.3 匀速下降 → land_height 0.15 调 MAVROS land → auto_disarm
```
- `auto_arm: false, auto_offboard: false`（手动，bridge yaml）
- type_mask 不设 FORCE 标志（PX4 offboard 位置控制不支持 force 字段，会报错）

## 按钮配置（User_define_commands.yaml 最终版）
- UserCmd1: 启动 real_flight.launch（默认航点）→ nohup roslaunch ... > log/real_flight.log 2>&1 &
- UserCmd2: `rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}' && sleep 1 && pkill -SIGINT -f 'roslaunch task_run_scripts real_flight.launch' && sleep 2 && pkill -f 'fsm_node'; pkill -f 'super_to_px4_bridge'; pkill -f 'waypoint_mission'; pkill -f 'map_to_world'; pkill -f 'top_down.rviz'`
  - ⚠️ `&&` 短路：set_mode 失败则杀进程部分不执行（用户选定保留此保守逻辑）
- UserCmd3: 同 UserCmd1 但 `waypoint_data_path:=.../My_Flight_Path.txt`（SUPER 复飞）
- 修改即生效（nx_command_server.py 2554 行每次重读 yaml，无需重启服务）

## UDP 6003 指令集（nx_command_server.py 2338-2560 行）
RecordPathStart / RecordPathStop / RecordPathReplay / AbortReplay / RecordBag:topics / StopBag / KillLiDAR / StartVideo / StopVideo / TakePhoto / UserCmd1-3

## HTTP 6004 查询接口
- /check_replay_status → IDLE / RUNNING_WAITING / RUNNING_FLYING / DONE_SUCCESS / DONE_ABORTED / ERROR
- /check_pose → OK / ERR_Z_LOW(高度<0.4m) / ERR_Z_HIGH(>1.6m) / ERR_DIST(偏离原点>0.2m)
- /check_file → EXISTS / EMPTY
- 另有 /check_camera、/radar/status、/file_transfer/*、/stream/*

## Path Replay daemon 流程（代码级）
1. 读 PATH_FILE（默认 My_Flight_Path.txt，环境变量 LIYUMEN_PATH_FILE 可改）
2. 悬停首点高度 Z0，等 OFFBOARD+armed（状态 WAITING）
3. 起飞确认（|z-Z0|<0.1m 或 15s 超时）→ FLYING
4. 40Hz 逐点发 /mavros/setpoint_position/local（含四元数）
5. 结束切 POSCTL，exit 0；中途模式≠OFFBOARD → exit 2(ABORTED)

## 录制细节
- RecordPathStart → /tmp/path_recorder_daemon.py，40Hz 写 PATH_FILE（覆盖旧文件）
- **已知缺陷**：RecordPathStop 用 terminate() 杀进程，文件尾部可能残留 NUL 字节 → 需清理：
  `python3 -c` 截断到首个 `\x00` 偏移（本会话 147456→147258 字节，去掉 198 字节 NUL）
- 表头 `x,y,z,qx,qy,qz,qw` 在录制时写入

## 环境备注
- 遥控器/QGC = <NET_RC>.100（nx_ping_QGC.py 里 QGC_IP），弹窗等前端逻辑在遥控器端，fouruav 只有指令通道
- RViz 显示：top_down.rviz 配了 /fsm_node/rog_map/occ、/cloud_registered、frontend_path 等；slam_rviz.rviz 是 FAST-LIVO2 的
- 推流：rviz_full_stream2.py（6000端口全屏截图），按钮2 的 pkill -f 'rviz' 会误杀它——已改为精确匹配
