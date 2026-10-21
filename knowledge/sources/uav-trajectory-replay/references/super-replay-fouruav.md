# fouruav SUPER 复飞会话档案（2026-08-10）

## 会话背景
用户从"读 SUPER 仓库 markdown 验证轨迹复飞"开始，最终落到配置 QGC 按钮（按钮3=SUPER复飞、按钮2=安全关闭）。关键转折：用户实际系统是 MID-360S+D435+FAST-LIVO2（不是 SUPER 仓库默认的 odin），且复飞配置全在 task_run_scripts 定制副本。

## 按钮最终配置（User_define_commands.yaml）

```yaml
UserCmd1:  # SUPER航点任务（启动默认航点 real_mission.txt）
  command: "source /opt/ros/noetic/setup.bash && source /home/fouruav/catkin_ws/devel/setup.bash && mkdir -p /home/fouruav/catkin_ws/src/task_run_scripts/log && nohup roslaunch task_run_scripts real_flight.launch > /home/fouruav/catkin_ws/src/task_run_scripts/log/real_flight.log 2>&1 &"

UserCmd2:  # 停止SUPER（先定点再杀进程）
  command: "rosservice call /mavros/set_mode '{custom_mode: \"POSCTL\"}' && sleep 1 && pkill -SIGINT -f 'roslaunch task_run_scripts real_flight.launch' && sleep 2 && pkill -f 'fsm_node' 2>/dev/null; pkill -f 'super_to_px4_bridge' 2>/dev/null; pkill -f 'waypoint_mission' 2>/dev/null; pkill -f 'map_to_world' 2>/dev/null; pkill -f 'top_down.rviz' 2>/dev/null"

UserCmd3:  # 按钮3 = 启动 SUPER 复飞（加载录制航迹）
  command: "source /opt/ros/noetic/setup.bash && source /home/fouruav/catkin_ws/devel/setup.bash && mkdir -p /home/fouruav/catkin_ws/src/task_run_scripts/log && nohup roslaunch task_run_scripts real_flight.launch waypoint_data_path:=/home/fouruav/catkin_ws/src/task_run_scripts/scripts/NX_code/My_Flight_Path.txt > /home/fouruav/catkin_ws/src/task_run_scripts/log/real_flight.log 2>&1 &"
```

### 按钮2 修改 diff（关键修复）
```diff
- pkill -f 'rviz' 2>/dev/null
+ pkill -f 'map_to_world' 2>/dev/null; pkill -f 'top_down.rviz' 2>/dev/null
```
原因：`pkill -f 'rviz'` 模糊匹配会误杀：
- `rviz_full_stream2.py`（推流服务，遥控器画面源）→ 黑屏
- `roslaunch ... mapping_MID360.launch ... rviz:=false`（SLAM 启动命令含 "rviz" 字样）→ SLAM 被干
- `rviz -d slam_rviz.rviz`（FAST-LIVO2 可视化）→ 非致命但被关

### 按钮3 修改 diff
```diff
- command: "echo '这是测试按钮3的输出' > /home/nv/Downloads/Liyumen/user_cmd3_test.log"
+ command: "source ... && nohup roslaunch task_run_scripts real_flight.launch waypoint_data_path:=.../My_Flight_Path.txt > .../real_flight.log 2>&1 &"
```

### waypoint.yaml 变更
- 曾临时把 `start_trigger_type: 1` 改成 `2`（自动触发），用户明确要求恢复 1（手动拨杆），已改回
- 验证期 `waypoint_downsample_dist: 0.1`（用户测试时的值），生产用多少由用户定

## 启动链路细节（start_fastlivo2.sh false）
D435(rs_d435_camera.launch) → Livox(msg_MID360s.launch) → IMU_TF(mid360_imu_tf.launch) → RViz(提前) → FAST-LIVO2(mapping_MID360.launch pcd_save:=false rviz:=false config_name:=MID360_Fly.yaml) → px4_and_others.launch（px4.launch + send_initial_mav_command + realman + map→odom/map→camera_init TF）

FAST-LIVO2 发布话题（LIVMapper.cpp）：
- `/cloud_registered`（PointCloud2，10Hz）← SUPER 的 cloud_topic
- `/aft_mapped_to_init`（Odometry，10Hz）← SLAM 里程计（SUPER 未用，odom_topic 仍是 /mavros/local_position/odom）
- `/path`、`/Laser_map`、`/dyn_obj*`

## RViz 白屏排查实录
1. 症状：按钮3 后 RViz 一片白
2. 第一层：top_down.rviz 窗口在最上层（xwininfo 确认 0x2c0000e），推流没截错
3. 第二层：`/fsm_node/rog_map/occ` `no new messages` → 地图无数据
4. 根因：fsm_node 进程死了，real_flight.log 显示 "all processes on machine have died" + `Reason: new node registered with same name`（重复启动顶替）
5. 修复后（重新按按钮3，无重复启动）：fsm_node 稳定存活，occ 话题 2Hz 持续输出，RViz 正常显示地图

## 关键文件路径速查
| 文件 | 路径 |
|---|---|
| 按钮配置 | ~/catkin_ws/src/task_run_scripts/scripts/NX_code/User_define_commands.yaml |
| 复飞主服务 | ~/catkin_ws/src/task_run_scripts/scripts/NX_code/nx_command_server.py（v2 是 11 行包装器） |
| 录制轨迹 | ~/catkin_ws/src/task_run_scripts/scripts/NX_code/My_Flight_Path.txt |
| SUPER 实机配置 | ~/catkin_ws/src/task_run_scripts/config/real_flight.yaml / waypoint.yaml / super_to_px4_bridge.yaml |
| 航点数据 | ~/catkin_ws/src/task_run_scripts/data/real_mission.txt |
| SUPER RViz | ~/catkin_ws/src/task_run_scripts/rviz/top_down.rviz |
| 启动脚本 | ~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_fastlivo2.sh |
| 遥控器 | <NET_RC>.100（QGC，"<PROJECT_CODENAME>X8"，nx_ping_QGC.py 可查） |

## 备份（回滚点）
- User_define_commands.yaml.bak_btn3（按钮3 改造前）
- waypoint.yaml.bak_auto（start_trigger_type=2 的临时版）
- My_Flight_Path.txt.bak_nul（NUL 清理前）
- config/waypoint.yaml.bak（抽稀测试前）
