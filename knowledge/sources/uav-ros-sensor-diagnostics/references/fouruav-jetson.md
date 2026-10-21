# fouruav Jetson 机载电脑 — 具体配置备忘

## 连接信息
- 主机名/用户：`fouruav`（注意：是 f-o-u-r-u-a-v，用户名 = 主机名，别打成 fouuav）
- 密码：`<SSH_PASSWORD>`
- Windows 侧登录方式：plink.exe（见 `windows-ssh-password-auth` 技能）
- 系统：Ubuntu 20.04 (focal)，内核 `5.10.216-tegra` (aarch64, Jetson NX 级)

## 软件栈 (ROS1 noetic, catkin_ws)
- `~/catkin_ws/src/`：FAST_LIO, FAST-LIVO2, livox_ros_driver2, mavlink_udp_bridge, slam_to_mavros, imu_tf, task_run_scripts（自定义 launch/scripts）, cv_bridge, rpg_vikit-master, SUPER
- 一键启动：`~/catkin_ws/start_fastlivo2.sh`（依次起 Livox driver → imu_tf → FAST-LIVO2 → MAVROS → RealMan monitor，Ctrl+C 全部停止）
- 注意：`start_fastlivo2.sh` 的日志在 `~/.ros/log/`，**roslaunch 会警告 log 目录 >1GB，可 `rosclean purge -y`**

## 飞控 (PX4)
- 串口：`/dev/ttyTHS0` @ **921600**（px4.launch / apm.launch 的 `fcu_url`）
- MAVROS 启动：`source ~/catkin_ws/devel/setup.bash && roslaunch task_run_scripts px4.launch`
- 确认连接标志：`CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`（VID/PID 1b8c:0036）
- ttyTHS3 也被打开过（用途未确认）；ttyTHS4 闲置
- 也有 apm.launch（ArduPilot 用，gcs_url 指向 <NET_LAN2>.27:14550）

## 雷达 (Livox MID360)
- **雷达自身 IP：`<NET_LAN2>.5`（用户确认，铁证：ping 通时 ttl=255、RTT 0.8–2.6ms）**
- 主机网卡 IP：eth1 用 **`<NET_LAN2>.2`**（之前误配成 <NET_LAN2>.5 与雷达冲突，导致雷达被"劫持"、SDK 找不到设备——修复：`sudo ip addr del <NET_LAN2>.5/24 dev eth1 && sudo ip addr add <NET_LAN2>.2/24 dev eth1`）
- 配置文件：`~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json`（host_net_info = 对象格式，含 cmd_data_ip 等 4 个主机 IP 字段；lidar_configs[].ip = 雷达 IP）
- **配置文件里的雷达 IP 都是旧的**：MID360_config.json 写 .196、MID360s_config.json 写 .175——两个都不对，真值是 .5。改配置前先 ping 验证
- `MID360s_config.json`（task_run_scripts/config/）的 host_net_info 是**数组 + host_ip 格式**，SDK 不认 → 报 `Params check failed, all livox lidars config is empty`；msg_MID360s.launch 的 `user_config_path` 指向它
- 网卡：eth1 = USB 转接网卡（雷达口，MAC 86:b1:ce...）；eth0 = 板载网口连地面站 <NET_RC>.100
- 雷达驱动单测：
  ```bash
  source ~/catkin_ws/devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360s.launch
  ```
  - 驱动日志：`~/.ros/log/*/livox_lidar_publisher2*.log`（带时间戳，`latest/` 可能失效）
  - 数据验证：`rostopic hz /livox/lidar --window 20`（正常 ~10 Hz）
- 状态（2026-08-01 会话末）：**雷达已确认在线**（IP 冲突解除后 ping <NET_LAN2>.5 通，ttl=255）；配置改正（主机 2.2 / 雷达 2.5）后待启动驱动验证点云。备查：`mixed_HAP_MID360_config.json`（主机 <NET_LIDAR>.5，雷达 <NET_LIDAR>.100/1.12，另一套混合配置）
- FAST-LIVO2 参数：`task_run_scripts/config/MID360_Fly_Lidar.yaml`（lidar_type 1，相机内参 fx 893.68 / fy 892.146 / cx 382.965 / cy 301.193）

## launch 文件（task_run_scripts/launch/）
px4.launch, apm.launch, node.launch, px4_and_others.launch, start_all_odin_apm.launch, mapping_MID360.launch, mid360_imu_tf.launch, msg_MID360s.launch, tf_to_mavros.launch, realman.launch, rs_d435_camera.launch

## 事故记录
- 2026-08-01 16:57：launch/ 下 10 个文件被批量清空成 0 字节（疑似 XML 粘贴进 shell 触发 `>` 重定向）；通过 `git checkout -- launch/` 恢复（task_run_scripts 是 git 仓库）。px4.launch 曾用手工重建（1231 字节）备用。
- 经验：遇到 launch 文件报 XML 语法错/空文件，先 `ls -la` 看是否 0 字节，git 仓库优先 `git status` + `git checkout --` 恢复。
