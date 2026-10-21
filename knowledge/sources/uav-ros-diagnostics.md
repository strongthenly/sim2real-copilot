---
name: uav-ros-diagnostics
description: 诊断 ROS 无人机机载电脑（Jetson NX 等）的飞控与雷达连接——PX4/ArduPilot 飞控(MAVROS)、Livox 激光雷达(MID360 等)的连通性排查与点云验证。触发词：飞控、雷达、PX4、mavros、Livox、MID360、点云、rostopic hz、Jetson、UAV。
---

# UAV ROS 系统连接诊断（飞控 + 雷达）

远程诊断无人机机载电脑（常见：Jetson NX + ROS noetic + PX4 飞控 + Livox 雷达）的传感器连接。核心原则：**分层验证**——网络层 → 设备层 → SDK/驱动层 → ROS 话题层，每层都有独立判据，不要跳过。

## 工作流程

### 0. 接入机载电脑（Windows 主机）
Windows 上交互式 SSH 密码登录常失败（PTY 崩溃 0xC000013A / 无法读密码）。**用 plink.exe 非交互登录**，多行命令用 base64 传输。详见 `references/windows-ssh-password.md`。

### 1. 飞控验证（PX4/ArduPilot）
1. 串口节点：`ls -l /dev/ttyTHS* /dev/ttyS*`（Jetson 飞控通常接 UART `ttyTHS0`，**不是** USB，别只查 ttyACM/ttyUSB）
2. 波特率测试：`timeout 3 bash -c 'stty -F /dev/ttyTHS0 921600 raw -echo; cat /dev/ttyTHS0' | xxd | head` —— 有 `fd`/`fe` 开头字节流 = MAVLink 在发
3. 启动 MAVROS：`roslaunch task_run_scripts px4.launch`（launch 文件在具体包里，**必须带包名**）
4. 判据：日志 `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot` = 飞控通；`rostopic echo /mavros/state -n 1` 看 `connected: True`
5. 用户若在 `dialout` 组可直接读串口，无需 sudo；`dmesg` 需 sudo

### 2. Livox 雷达验证（重点，坑最多）
1. **网络层**：雷达有固定 IP（如 <NET_LAN2>.5）。`ping` 通 ≠ 连上，但先要通。**ttl 判据**：ping 通但 `ttl=64` + 延迟 <0.1ms = ping 的是本机自己（**IP 冲突**，主机抢占了雷达 IP）；`ttl=255` + 延迟 ~1ms = 雷达本体
2. **身份确认**：`curl -s https://api.macvendors.com/<mac前缀>` 查 MAC 厂商（Livox = "DJI BAIWANG"）；`ip neigh show dev ethX` 看对端 MAC
3. **设备广播**：雷达每秒发 430 字节广播（UDP 56200→56201），内含 SN、型号、固件、**上位机 IP 配置**。用 `scripts/livox_broadcast_probe.py` 抓取解析——这是定位连接问题的金矿
4. **SDK 连接判据**：驱动日志 `Init livox lidars succ` **只代表 SDK 初始化成功，不代表连上雷达**。真正的判据是 `rostopic list | grep livox` 有没有话题、`rostopic hz /livox/lidar` 有没有 10Hz
5. **广播码 = 设备 SN**：launch 默认 `bd_list=100000000000000` 常与真实设备不符；但 **ROS1 版驱动根本不读 bd_list 参数**（只有 ROS2 读 `cmdline_input_bd_code`），传了也白传，SDK 靠 auto-connect
6. **host_net_info 必须匹配雷达期望的上位机 IP**：雷达只会把点云推给它配置里记录的 host IP（从广播包 TLV 可读出，如 <NET_LIDAR>.5）。主机网卡必须拥有该 IP（`ip addr add <NET_LIDAR>.5/24 dev ethX`）
7. **跨网段陷阱**：雷达 IP <NET_LAN2>.5 + 上位机 <NET_LIDAR>.5（不同网段）时，雷达数据要经**它配置的网关 <NET_LAN2>.1** 转发——主机网卡需补上该网关 IP，否则雷达数据永远发不出去（雷达会持续发 "who-has <NET_LAN2>.1" ARP）
8. **配置格式**：SDK2 的 config JSON 有对象格式（`cmd_data_ip`）和数组格式（`host_ip`+`multicast_ip`）两种，驱动用对象格式；`lidar_configs[].ip` 填雷达 IP。细节见 `references/livox-mid360-connectivity.md`

### 3. 常见坑速查
- **RViz 同名节点互杀（2026-08-25 实战，界面退出类问题首选检查）**：启动 real_flight.launch（智能航线任务）后雷达界面/平板画面退出。日志签名：`[WARN] Shutdown request received. / Reason given for shutdown: [[/rviz] Reason: new node registered with same name]`。根因：雷达界面（start_odin.sh 起的 odin rviz）和 real_flight.launch 的规划 rviz **同名 `name="rviz"`**，ROS 注册同名新节点强制杀旧节点。修复：real_flight.launch 里 rviz 改名 `name="rviz_super"`（两窗口共存）。**推流无副作用**：rviz_full_stream2.py 用 mss 整屏截图（monitor 级），不依赖节点名/窗口名。改 launch 下次 roslaunch 即生效，无需重启/编译。排查步骤：`ps aux | grep "[r]viz"` 看是否有两个 rviz 进程参数不同但同名，再 `grep -n 'name="rviz"'` 所有 launch
- **make 增量编译：二进制时间戳旧 ≠ 没编译到最新**：catkin_make 判定重编用**文件 mtime**（源码比二进制新才重编），不是 git 提交时间。git 提交 8-19 但 fsm_node 二进制 8-14 完全可能已是最新（8-19 只改了 bridge 文件）。验证：`find ~/catkin_ws/src/SUPER/super_planner -name "*.cpp" -newer ~/catkin_ws/devel/lib/super_planner/fsm_node` 空输出 = 无需重编；要强制重编 `touch <源码>` 再 catkin_make
- **子模块 .git 是文件不是目录**：`[ -d "$d/.git" ]` 判定 git 仓库会误判子模块（SUPER/task_run_scripts 的 .git 是 gitlink 指针文件）。正确判定 `git -C "$d" rev-parse --git-dir >/dev/null 2>&1`
- bag 数据后处理（点云话题检测、bag→PCD/LAS、非标准/自研 SDK bag 顺序解析、稠密彩色地图、RGB-D 深度图导出）走 **bag-pointcloud-processing** 技能（Windows 无需 ROS，含 odin SDK bag 格式细节与 calib 参数）
- `roslaunch <file>.launch` 不带包名 → "neither a launch file in package"：必须 `roslaunch <pkg> <file>.launch`
- `rostopic hz` 报 "Cannot load message class" → 环境变量问题：`source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash`；验证 `python3 -c "from livox_ros_driver2.msg import CustomMsg"`
- `~/.ros/log/latest/` 在进程停掉后失效，日志文件名带时间戳，用 `ls -t ~/.ros/log/*/<node>*.log | head -1`
- 改 launch/config 文件前**先备份**（`cp x x.bak`），用 `python3 -c` 改 JSON 比 heredoc 覆盖更体面且不易破坏格式
- launch 目录里 0 字节文件 = 被误清空，若该目录是 git 仓库用 `git checkout -- <dir>` 恢复
- 一个网卡多 IP 时，广播/数据可能从错误源 IP 发出，用 tcpdump 确认实际源地址

## 用户协作偏好（指挥官模式）
用户常以"指挥官模式"协作：**命令由用户亲手在远端执行**，agent 负责给命令、判读输出、做决策。每条消息给 1-3 条命令 + 预期输出说明，等用户贴结果再判读。被问"只要确定 X"时给**明确结论**（✅/❌ + 证据表），不要继续无休止排查。

## 定点模式位置漂移排查（POSCTL 一直朝一个方向偏移，2026-08-18 fouruav 实战）

症状："定点模式飞行时飞机一直朝一个方向偏移"。机载 ROS 日志只能证明"FAST-LIO→tf_to_mavros→/mavros/vision_pose/pose→mavros→PX4"链路通不通，**偏移根因在 PX4 侧 EKF，最终判定必须靠 .ulg**（QGC 下载到桌面 log_360/log_odin 走 px4-ulog-analysis 技能）。机载侧排查流程：

1. **先查机器重启时间**：`uptime`——刚重启过的话 ~/.ros/log 里全是短会话，飞行会话靠时间戳+launch 组成定位
2. **定位飞行会话**：`ls -ltd ~/.ros/log/*/` 按时间排；飞行会话 = 同时含 6 个 launch：roscore + imu_tf/mid360_imu_tf + livox_ros_driver2/msg_MID360 + slam_to_mavros/tf_to_mavros + slam_to_mavros/px4.launch + fast_lio/mapping_mid360.launch（只有 laserMapping+rviz 的 = 纯建图测试不是飞行）
3. **会话内四件套判读**：
   - master.log：`+PUB [/mavros/vision_pose/pose] /tf_to_mavros` + `+SUB` by /mavros = 链路注册齐全
   - laserMapping stdout：有 `IMU Initial Done` = FAST-LIO 初始化成功；只有 "Multi thread started / p_pre->lidar_type 1 / file opened" 三行 = 雷达数据没进来，失败
   - rosout：`CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot` = 飞控通；`CON: Lost connection, HEARTBEAT timed out` = 中途断连
   - SIGINT 后保存的 scans.pcd 大小/时间（如 352MB/3分钟）= 确实建了图
4. ⚠️ **camera_init 警告 = 启动时序正常现象**：`"camera_init" passed to lookupTransform argument target_frame does not exist` 出现在 tf_to_mavros 先于 FAST-LIO 发布 TF 的启动段（每个会话 6-9 次），`IMU Initial Done` 后即消失 = vision_pose 正常发布；只有持续到飞行全程才算异常
5. **埋雷**：tf_to_mavros.launch 写 `target_frame_id=/map, source_frame_id=/base_link`，但节点实际用源码默认值 `/camera_init`→`/body`（launch 参数不生效）；FAST-LIO 恰好发 camera_init→body 所以链路通，参数是死的，建议对齐或删掉
6. **快速旁证**：tf_to_mavros 每秒覆写 `~/share/drone_pose.txt`（X/Y/Z 三行），可看飞行末态视觉位姿
7. **偏移根因判别（2026-08-18 log 01_57_39，两轮误判教训）**：**先排除机械故障再谈视觉 yaw**——实锤案例真凶是桨不平衡/机械振动（电机对角饱和 M3/M4→0 + 振动分段 RMS 放大 + imbalanced_prop 全程非零 + 姿态环 tq_z 0.55 打满拉不住、yaw 指令方向与实测相反），视觉 yaw 卡错 18° 只是振动导致 FAST-LIO 退化的**症状**。曾被误判"视觉 yaw 漂移污染 EKF"和"撞障碍物"，都被用户反证推翻（"之前没这问题/另一架同配置没事/附近没有障碍物"= 最高优先级证据）。**机载侧无法定位此类问题，直接要 .ulg 走 px4-ulog-analysis 技能**，一键脚本 `scripts/posctl_drift_probe.py`（三航向源对拍+电机分配+振动分段），正确案例 `references/posctl-drift-mechanical-case.md`，误判教训存档 `references/visual-yaw-drift-diagnosis.md`

## 定点模式 not ready（切不进 POSCTL，QGC 提示 position estimate invalid，2026-08-20 fouruav 实战）
QGC 定点 not ready = PX4 EKF 无有效位置估计。机载侧排查顺序（mavros 断连是最大暗坑）：
1. **先查 mavros↔飞控连接**：`rostopic echo /mavros/state -n 1`。`connected: False` = 串口断连——此时 /mavros/estimator_status 无消息、`rosparam get /mavros/param/EKF2_AID_MASK` 报 not set（参数未同步），任何定位判断无从谈起，先修串口
2. 视觉里程计是否进 PX4：`rostopic hz /mavros/odometry/in --window 20`（预期 ~20Hz；0=链路断；400Hz=没限速 EKF 拒收）。**fouruav(2026-08 重构 FAST-LIVO2) 的链路是 laserMapping 直接发 /mavros/vision_pose/pose（10Hz）**，节点列表里可以没有 odom_to_mavros/tf_to_mavros，别按旧拓扑等 odom 话题
3. EKF 融合参数：`rosparam get /mavros/param/EKF2_AID_MASK`（vision position 位=8；PX4 1.17 可能已改 EKF2_EV_CTRL，见 px4-ulog-analysis 技能）
- 涉及"该下载哪个固件/飞控什么型号"：读 .ulg 头 ver_hw 定硬件（正则法+ver_hw→固件映射表见 px4-ulog-analysis 技能）

## QGC 连不上（多网卡抢广播路由，2026-09-11 fouruav 实战）
症状：机载 ROS 栈全起、`rostopic echo /mavros/state` 显示 `connected: True`（飞控通），但 QGC（PC/平板）就是**搜不到机、连不上**。
根因：mavros `gcs_url=udp-b://@` 把 MAVLink 广播到 `255.255.255.255:14550`；内核按默认路由 **metric 最小优先** 选出口网卡。机载有两张网卡两条默认路由——eth0（雷达网 / LiDAR，`via <NET_LIDAR>.1 metric 20100`，网关其实死的/ARP FAILED）优先于 wlan0（WiFi，QGC 在这边，`via <NET_LAN2>.1 metric 20600`）⇒ 广播全发进雷达网段(192.168.1.x)，QGC 所在 WiFi 网段永远收不到。
**一条命令定性**：`ip route get 255.255.255.255` → 看 `dev` 是不是 wlan0（不是 = 中招）。
⚠️ **别白改 gcs_url**：`udp-b://@<ip>` 会被 mavros 强制改回受限广播——启动日志必现 `udp1: Remote address: 255.255.255.255:14550`，指定目标地址无效（试过 <NET_LAN2>.255，失败）。所以要修的是路由，不是 URL。
治本（去掉 eth0 多余的默认路由，让广播走 wlan0）：
```bash
sudo ip route del default via <NET_LIDAR>.1 dev eth0              # 立即生效，不重启、不影响雷达(<NET_LIDAR>.0/24 直连路由保留；网关本是死的)
sudo nmcli con mod "Wired connection 1" ipv4.never-default yes   # 永久写入 NM 配置，重启后依然无此路由
ip route get 255.255.255.255   # 期望 dev wlan0
```
mavros 持续发广播，路由一变几秒内即通，**无需重启 mavros**。备用（无 sudo，但依赖 QGC 端 IP 固定）：gcs_url 改单播 `udp://@<QGC_IP>:14550`。
诊断套路：① `ip route` 看有无多条 default 及 metric；`nmcli con show <conn> | grep -iE "ipv4|route"`（本机无 netplan/interfaces/dhcpcd 文件时，路由/网关全来自 NM connection profile，`ipv4.method=manual`+`ipv4.gateway` 就会造出默认路由）。② QGC 端确认在监听：Windows `Get-NetUDPEndpoint -LocalPort 14550` 应显示 `0.0.0.0:14550`；防火墙放行规则 `Get-NetFirewallApplicationFilter | ? {$_.Program -like "*QGround*"}` 必须匹配**实际运行的 exe 路径**（本例 `E:\QGroundControl\bin\QGroundControl.exe`，大小写不敏感）。③ 别用 `Test-Connection` 判断，用 plink 连机载实测。

## fouruav 连接速查
当前 IP：**<NET_WLAN_A>.50**（2026-08-25 实测；DHCP 常变，历史：<NET_RC>.14 → <NET_WLAN_A>.47 → <NET_WLAN_A>.50，均同机），用户 fouruav，密码 <SSH_PASSWORD>，hostkey `SHA256:<ONBOARD_HOSTKEY_FP>`（指纹跨 IP 不变，认指纹不认 IP）。**<NET_LAB>.111（hostkey `SHA256:<OTHER_HOST_HOSTKEY_FP>`）是另一台机器，勿混用**。换 IP 后先 `ssh -v -o ConnectTimeout=10 -o BatchMode=yes user@IP 2>&1 | Select-String "Server host key"` 重抓指纹再用 plink 连（全流程见 windows-ssh-automation 技能）。
