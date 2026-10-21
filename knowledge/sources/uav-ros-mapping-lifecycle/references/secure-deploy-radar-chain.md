# NX_code 安全部署版（Cython）— 雷达启动链路 / 配置面 / 合并流程

实测：fouruav（Orin NX）2026-08-21，IP <NET_WLAN_A>.47（hostkey <ONBOARD_HOSTKEY_FP>… 不变 = 稳定身份）。部署形态已从源码版改为 Cython 安全打包版。产出物：`桌面\标书相关\NX_code_视频文案_遥控器雷达按钮与配置合并.md`（视频文案，含口播旁白+画面建议）。

## 部署形态
- `scripts/NX_code/`：核心 .py 全部编译为 `*.cpython-38-aarch64-linux-gnu.so`，部署机无源码、无 .git（`git status` 报"不是 git 仓库"）。
- 包装器：`nx_command_server2.py` / `nx_touch_server2.py` / `rviz_full_stream2.py`（约 5 行，import .so）；`*_launcher.py` 同义。
- 自启：`~/.config/autostart/qgc_servers.desktop` → `start_qgc_servers.sh`（roscore + 三大微服务：nx_touch_server2 UDP 6002、nx_command_server2 UDP 6003 + HTTP 6004、rviz_full_stream2 TCP 6000；日志 `log/<时间戳>/`，软链 `log/latest`，保留最近 10 份）。
- ⚠️ autostart 里的 `odin_stream.desktop` 已改名 `.back`；NX_code 目录里的 `odin_stream.desktop` 仍指向不存在的 `/home/fouruav/odin_ws/`（旧路径，现为 catkin_ws）——拷进 autostart 会启动失败。

## 雷达启动链路（遥控器/平板 QGC 按钮）
QGC 按钮 → UDP 6003 JSON → nx_command_server2.py → .so：
1. 启动时读 `Radar_Launcher.yaml`，日志行：`📡 雷达配置已加载: profile='mid360'`、`✅ 小鲤鱼指令接收通道已开启 (UDP 6003)`、`✅ NX 综合监测微服务已开启 (HTTP 6004 - file-transfer)`。
2. 按 `active_profile`（odin|mid360；环境变量 `LIYUMEN_RADAR_PROFILE` 覆盖）取 profile。
3. 执行 `launch_with_cloud.command`（=录数据建图）或 `launch_without_cloud.command`（=仅飞行）。
4. mid360 → `bash start_fastlivo2.sh true|false`；odin → `roslaunch odin_ros_driver odin1_ros1.launch` + `px4_and_others.launch`（recorddata 由 `config/control_command.yaml` 控制）。
5. 关闭按钮走同 yaml `cleanup`（`pkill_patterns` + `rosnode_patterns`，新加进程必须同步加进清理列表）。

.so 内状态机：`global_radar_start_message / global_radar_start_result / global_radar_start_time / global_radar_starting`；`_build_lidar_launch_cmd`、`SpaceHandler._send_json`（UDP 应答 JSON）。

## start_fastlivo2.sh（6 组节点按序，SIGINT 优雅关闭）
rs_d435_camera.launch（D435）→ msg_MID360s.launch（livox 驱动）→ mid360_imu_tf.launch（IMU→TF）→ rviz 提前弹窗（slam_rviz.rviz）→ mapping_MID360.launch（`pcd_save:=true|false`、`config_name:=MID360_Fly*.yaml`、`rviz:=false`）→ px4_and_others.launch（px4.launch + send_initial_mav_command + realman.launch + map→odom / map→camera_init TF）。
日志 `log/fastlivo2_<时间戳>_<true|false>/`，保留最近 10 份。

## 配置面（部署机可直接改的明文，改 yaml/launch/sh 无需重编译）
| 文件 | 作用 |
|---|---|
| scripts/NX_code/Radar_Launcher.yaml | 雷达类型（active_profile）/启动命令/recorddata/清理规则 |
| scripts/NX_code/User_define_commands.yaml | UserCmd1/2/3 按钮（name 界面显示、command 机载执行） |
| config/MID360_Fly.yaml | 飞行定位（dense_map_en=false, point_filter_num=3, max_iterations=3, voxel_size=1.0） |
| config/MID360_Fly_Lidar.yaml | 高质量建图（true/1/5/0.5，算力约 2 倍） |
| launch/mapping_MID360.launch | pcd_save / config_name / rviz 参数 |
| scripts/NX_code/start_fastlivo2.sh | $1 true/false（录不录数据）、$2 lidar/fly 手动覆盖 |

改 User_define_commands.yaml / Radar_Launcher.yaml 后重启服务（⚠️ 别用 `pkill -f start_qgc_servers.sh`——脚本可能已死、服务是孤儿、且 GNOME autostart 会自动复活，见 SKILL.md"服务进程排障"节）：先 `mv ~/.config/autostart/qgc_servers.desktop ~/.config/autostart/qgc_servers.desktop.off` 防复活 → 按精确进程名 pkill -9（方括号技巧）→ `bash start_qgc_servers.sh` 重启 → 移回 desktop（或重启桌面会话自动拉起）。

## 改代码/合并流程（源码在开发机）
1. 开发机：`git checkout master && git pull` → 开分支 → 改 → commit → push。
2. 合并到 master（PR 或直接 merge）。README 注明 `.github/workflows/build-deploy.yml`：push master 自动 `build_secure.sh`（Cython 编译）→ 发布 deploy 分支。
3. 部署：NX 上拉 deploy 分支，或开发机 `bash build_secure.sh deploy` 后 scp `deploy/` 到 `scripts/NX_code/`。
4. 重启服务生效。
坑：编译期绝对路径（源码里搜 `fouruav`）与部署路径不一致会起不来；autostart 只放有效的 .desktop。

## 实测发现的 3 个问题（视频彩蛋/排障线索）
1. `odin_stream.desktop` 指向 `/home/fouruav/odin_ws/`（不存在，现为 catkin_ws）——拷进 autostart 会失败。
2. UserCmd3 占位：command 写死 `/home/nv/Downloads/...`（本机用户是 fouruav）——执行必失败。
3. catkin_ws/src 无 `odin_ros_driver` 包——切 odin profile 前必须先部署该驱动。

## 侦察技巧（无源码时）
```bash
strings nx_command_server.cpython-38-aarch64-linux-gnu.so | grep -iE 'Radar_Launcher|UserCmd|6003|LIYUMEN|global_radar|launch_with'
```
→ 挖出配置文件名、环境变量（LIYUMEN_RADAR_PROFILE / LIYUMEN_WORKSPACE_SETUP / LIYUMEN_ROS_SETUP / LIYUMEN_LOG_DIR / LIYUMEN_PX4_LAUNCH / LIYUMEN_IMAGE_TOPIC / LIYUMEN_RECORDDATA_YAML / LIYUMEN_USB_ROOT 等）、UDP/HTTP 端口、状态机字段、HTTP 路由（如 file-transfer）。

## 实测进程清单（服务正常时）
`pgrep -af 'nx_command|nx_touch|rviz_full|roscore'` 应看到：roscore + nx_touch_server2.py + nx_command_server2.py + rviz_full_stream2.py（主进程 + `--worker` 子进程）。
