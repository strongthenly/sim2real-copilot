# 机载诊断（飞控 / 雷达 / 图传 / Windows 远程接入）

> 领域 `diagnostics` ｜ 由 4 个原始技能合并而成 ｜ 生成于 2026-10-21

**这一层解决什么**：故障时绝大多数时间花在"判断哪一层坏了"。这一层给的是分层判据、防误判事实，以及从 Windows 远程操作的确定做法。

**建议读法**：A 分层验证原则 → B Livox 雷达判据 → C 飞控/MAVROS → D 图传与数传 → E Windows 远程接入

**包含的原始技能**：`uav-ros-diagnostics`、`uav-drone-diagnostics`、`uav-ros-sensor-diagnostics`、`livox-lidar-diagnostics`


---

### 来源技能 `uav-ros-diagnostics`

> 诊断 ROS 无人机机载电脑（Jetson NX 等）的飞控与雷达连接——PX4/ArduPilot 飞控(MAVROS)、Livox 激光雷达(MID360 等)的连通性排查与点云验证。触发词：飞控、雷达、PX4、mavros、Livox、MID360、点云、rostopic hz、Jetson、UAV。

远程诊断无人机机载电脑（常见：Jetson NX + ROS noetic + PX4 飞控 + Livox 雷达）的传感器连接。核心原则：**分层验证**——网络层 → 设备层 → SDK/驱动层 → ROS 话题层，每层都有独立判据，不要跳过。

**工作流程**

#### 0. 接入机载电脑（Windows 主机）

Windows 上交互式 SSH 密码登录常失败（PTY 崩溃 0xC000013A / 无法读密码）。**用 plink.exe 非交互登录**，多行命令用 base64 传输。详见 `references/windows-ssh-password.md`。

#### 1. 飞控验证（PX4/ArduPilot）

1. 串口节点：`ls -l /dev/ttyTHS* /dev/ttyS*`（Jetson 飞控通常接 UART `ttyTHS0`，**不是** USB，别只查 ttyACM/ttyUSB）
2. 波特率测试：`timeout 3 bash -c 'stty -F /dev/ttyTHS0 921600 raw -echo; cat /dev/ttyTHS0' | xxd | head` —— 有 `fd`/`fe` 开头字节流 = MAVLink 在发
3. 启动 MAVROS：`roslaunch task_run_scripts px4.launch`（launch 文件在具体包里，**必须带包名**）
4. 判据：日志 `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot` = 飞控通；`rostopic echo /mavros/state -n 1` 看 `connected: True`
5. 用户若在 `dialout` 组可直接读串口，无需 sudo；`dmesg` 需 sudo

#### 2. Livox 雷达验证（重点，坑最多）

1. **网络层**：雷达有固定 IP（如 <NET_LAN2>.5）。`ping` 通 ≠ 连上，但先要通。**ttl 判据**：ping 通但 `ttl=64` + 延迟 <0.1ms = ping 的是本机自己（**IP 冲突**，主机抢占了雷达 IP）；`ttl=255` + 延迟 ~1ms = 雷达本体
2. **身份确认**：`curl -s https://api.macvendors.com/<mac前缀>` 查 MAC 厂商（Livox = "DJI BAIWANG"）；`ip neigh show dev ethX` 看对端 MAC
3. **设备广播**：雷达每秒发 430 字节广播（UDP 56200→56201），内含 SN、型号、固件、**上位机 IP 配置**。用 `scripts/livox_broadcast_probe.py` 抓取解析——这是定位连接问题的金矿
4. **SDK 连接判据**：驱动日志 `Init livox lidars succ` **只代表 SDK 初始化成功，不代表连上雷达**。真正的判据是 `rostopic list | grep livox` 有没有话题、`rostopic hz /livox/lidar` 有没有 10Hz
5. **广播码 = 设备 SN**：launch 默认 `bd_list=100000000000000` 常与真实设备不符；但 **ROS1 版驱动根本不读 bd_list 参数**（只有 ROS2 读 `cmdline_input_bd_code`），传了也白传，SDK 靠 auto-connect
6. **host_net_info 必须匹配雷达期望的上位机 IP**：雷达只会把点云推给它配置里记录的 host IP（从广播包 TLV 可读出，如 <NET_LIDAR>.5）。主机网卡必须拥有该 IP（`ip addr add <NET_LIDAR>.5/24 dev ethX`）
7. **跨网段陷阱**：雷达 IP <NET_LAN2>.5 + 上位机 <NET_LIDAR>.5（不同网段）时，雷达数据要经**它配置的网关 <NET_LAN2>.1** 转发——主机网卡需补上该网关 IP，否则雷达数据永远发不出去（雷达会持续发 "who-has <NET_LAN2>.1" ARP）
8. **配置格式**：SDK2 的 config JSON 有对象格式（`cmd_data_ip`）和数组格式（`host_ip`+`multicast_ip`）两种，驱动用对象格式；`lidar_configs[].ip` 填雷达 IP。细节见 `references/livox-mid360-connectivity.md`

#### 3. 常见坑速查

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

#### 用户协作偏好（指挥官模式）

用户常以"指挥官模式"协作：**命令由用户亲手在远端执行**，agent 负责给命令、判读输出、做决策。每条消息给 1-3 条命令 + 预期输出说明，等用户贴结果再判读。被问"只要确定 X"时给**明确结论**（✅/❌ + 证据表），不要继续无休止排查。

#### 定点模式位置漂移排查（POSCTL 一直朝一个方向偏移，2026-08-18 fouruav 实战）

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

#### 定点模式 not ready（切不进 POSCTL，QGC 提示 position estimate invalid，2026-08-20 fouruav 实战）

QGC 定点 not ready = PX4 EKF 无有效位置估计。机载侧排查顺序（mavros 断连是最大暗坑）：
1. **先查 mavros↔飞控连接**：`rostopic echo /mavros/state -n 1`。`connected: False` = 串口断连——此时 /mavros/estimator_status 无消息、`rosparam get /mavros/param/EKF2_AID_MASK` 报 not set（参数未同步），任何定位判断无从谈起，先修串口
2. 视觉里程计是否进 PX4：`rostopic hz /mavros/odometry/in --window 20`（预期 ~20Hz；0=链路断；400Hz=没限速 EKF 拒收）。**fouruav(2026-08 重构 FAST-LIVO2) 的链路是 laserMapping 直接发 /mavros/vision_pose/pose（10Hz）**，节点列表里可以没有 odom_to_mavros/tf_to_mavros，别按旧拓扑等 odom 话题
3. EKF 融合参数：`rosparam get /mavros/param/EKF2_AID_MASK`（vision position 位=8；PX4 1.17 可能已改 EKF2_EV_CTRL，见 px4-ulog-analysis 技能）
- 涉及"该下载哪个固件/飞控什么型号"：读 .ulg 头 ver_hw 定硬件（正则法+ver_hw→固件映射表见 px4-ulog-analysis 技能）

#### QGC 连不上（多网卡抢广播路由，2026-09-11 fouruav 实战）

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

#### fouruav 连接速查

当前 IP：**<NET_WLAN_A>.50**（2026-08-25 实测；DHCP 常变，历史：<NET_RC>.14 → <NET_WLAN_A>.47 → <NET_WLAN_A>.50，均同机），用户 fouruav，密码 <SSH_PASSWORD>，hostkey `SHA256:<ONBOARD_HOSTKEY_FP>`（指纹跨 IP 不变，认指纹不认 IP）。**<NET_LAB>.111（hostkey `SHA256:<OTHER_HOST_HOSTKEY_FP>`）是另一台机器，勿混用**。换 IP 后先 `ssh -v -o ConnectTimeout=10 -o BatchMode=yes user@IP 2>&1 | Select-String "Server host key"` 重抓指纹再用 plink 连（全流程见 windows-ssh-automation 技能）。


---

### 来源技能 `uav-drone-diagnostics`

> Diagnose a UAV onboard computer (Jetson NX, ROS noetic) sensor links — PX4/ArduPilot flight controller over UART via MAVROS, Livox MID360 LiDAR over Ethernet — including password-SSH from a Windows host (plink). Use when the FCU or LiDAR shows no data, or when accessing the drone remotely from Windows.

#### 0. Remote access from Windows (password auth)

Windows OpenSSH crashes (exit 0xC000013A, zero output) when it needs an interactive password prompt under PTY, and refuses stdin passwords. Don't fight it — use PuTTY's plink.exe with `-pw`:

```powershell

curl.exe -L -o "$env:TEMP\plink.exe" "https://the.earth.li/~sgtatham/putty/latest/w64/plink.exe"

& "$env:TEMP\plink.exe" -ssh -batch -hostkey "SHA256:<fp>" -pw <password> user@host "hostname; whoami"
```

Quoting hell (PowerShell → cmd → bash): write the remote script to a local file, base64 it, pipe through bash:
```powershell
$content = Get-Content local.sh -Raw
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($content))
cmd /c "`"$env:TEMP\plink.exe`" -ssh -batch -hostkey `"SHA256:<fp>`" -pw <pw> user@host `"echo $b64 | base64 -d | bash`""
```
Pitfalls: SSH_ASKPASS with a .bat does NOT work on Windows OpenSSH. The `&` call operator can trip Hermes' backgrounding detector — wrap in `cmd /c`. Heredocs inside the piped script break — use single-line python `-c` instead. `sudo` inside remote scripts needs a TTY, so either `echo <pw> | sudo -S <cmd>` or have the user run the sudo step in their own terminal. Full recipes: `references/windows-plink-ssh.md`.

#### 1. Flight controller (PX4/ArduPilot)

- Find the port: `ls -l /dev/ttyTHS* /dev/ttyS*` — on Jetson the FCU is usually ttyTHS0; a recently-touched node (recent mtime) = actively opened.
- Config: `~/catkin_ws/src/<pkg>/launch/px4.launch` / `apm.launch` → `fcu_url=/dev/ttyTHS0:921600`.
- Hardware test bypassing ROS — **baud matters**: plain `cat` at default 115200 shows nothing. Use:
```bash
timeout 3 bash -c 'stty -F /dev/ttyTHS0 921600 raw -echo; cat /dev/ttyTHS0' | xxd | head -20
```
`fd ...` bytes = MAVLink v2 flowing. Nothing at both 921600 and 115200 = no power / wrong wire / FCU off.
- ROS-level: startup log `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot` = success; `rostopic echo /mavros/state -n 1` → `connected: True`.
- PX4 vs ArduPilot: separate launch files exist (px4.launch / apm.launch); pick per FCU. `start_fastlivo2.sh` bundles driver + IMU TF + FAST-LIVO2 + MAVROS.

#### 2. Livox LiDAR over Ethernet (layered, network first)

1. `ip addr` — Jetson NIC must hold a subnet-adjacent address to the radar. Livox JSON has `host_net_info` (host IP, 4 fields) + `lidar_configs[].ip` (radar IP). Same /24, NOT the same address.
2. `ping <radar_ip>` — **read the TTL**: ttl=64 & ~0.1 ms = you pinged yourself (IP conflict!); ttl=255 & ~1-2 ms = real embedded device. This TTL trick caught a self-ping in a real session.
3. `ip neigh show dev <iface>` — radar MAC OUI `9c:5a:8a` = DJI BAIWANG (Livox).
4. `rostopic hz /livox/lidar --window 10` → ~10 Hz = connected. **"Init livox lidars succ" ≠ device connected** — the topic publishes only after full connection.
5. No topic → watch the wire. Radar broadcasts device info 56200→255.255.255.255:56201 (1 Hz, ~430 B); discovery handshake on UDP 56000 (SDK sends 24 B, radar replies 48 B within ~1 ms). Radar alive = ARP replies + 430 B broadcast every second → problem is host config, not hardware.
6. Parse the 56201 broadcast (python socket bound to '' :56201) — contains SN, `DevType:Mid-360`, firmware, and TLV fields holding the radar's **configured host IP** (`c0 a8 01 05` = <NET_LIDAR>.5 in the session). If the radar expects a host IP the Jetson doesn't hold, no data flows — add that IP to the NIC and restart the driver.
Full port map, TLV decode and the IP-conflict case study: `references/livox-mid360-debugging.md`.

#### 3. QGC 图传/投屏服务栈（rviz_full_stream / nx_command / nx_touch）故障排查

> **⚠️ 先分辨是哪一代部署，两代故障形态完全不同：**
> - **老（≤2026-08）：GNOME autostart + 脚本拉起**，典型故障 = roscore 起不来 → worker 崩循环 → 6000 不监听。
>   见本节下面 + `references/qgc-stream-services.md`。
> - **新（2026-09 起）：systemd user 单元 + 推流鉴权**，典型故障 = **服务全在跑、6000 在听、ping 也通，
>   但遥控器一直显示「等待视频信号接入」** → 卡在 `AUTH:` 握手。**别在 roscore / 网络里瞎找。**
>   ⇒ 完整判读表、`HTTP/0.9` 误判、`is_auth_enabled()` 的 SECRET 陷阱、
>   `restart_qgc_services.sh` 杀完不启动的事故：**`references/qgc-stream-auth-and-systemd.md`**
>
> **新体系的三个速查事实：**
> 1. 服务归 **`aircore.target`** 下四个单元（`aircore-roscore/stream/command/touch`）。
>    重启只用 `systemctl --user restart aircore-stream.service`，**别用 `restart_qgc_services.sh`**（它硬编码的
>    `catkin_ws/.../NX_code/start_qgc_servers.sh` 路径可能已不存在 → 杀完 systemd 服务后起不来，6000 直接没了）。
> 2. 进程归属哪个单元要读 `/proc/<pid>/cgroup`，**不能看 PPID**（只显示 `user@1000.service`）。
>    环境变量同理：`sudo tr '\0' '
' < /proc/<pid>/environ`（在 `~/.bashrc`/`/etc/environment` 里 grep 不到，
>    它们来自单元的 `EnvironmentFile=`）。
> 3. **6000 不是 HTTP**：帧格式 `<4字节小端长度><JPEG>`。`curl` 报 `Received HTTP/0.9 when not allowed`
>    = **端口通、协议非 HTTP**，不要再拿 curl 判活。
> 4. **★ 最强判据：前台跑 + `Ctrl+C` 看回溯栈停在哪个函数** —— 停在 `conn.recv(1024)`
>    （`_recv_auth_challenge` 内）= 遥控器连上了但**一个字都不发** ⇒ App 不做握手；
>    停在 `accept()` = 根本没连进来 ⇒ 网络层。**先按 Ctrl+C 再决定改不改鉴权**
>    （见 `references/qgc-stream-auth-and-systemd.md` §3.3.1）。

**症状**：遥控器虚拟投屏连不上/图传黑屏。老体系下服务栈由 `start_qgc_servers.sh`（GNOME autostart `~/.config/autostart/qgc_servers.desktop`，仅登录时跑一次）拉起：roscore + 3 python 服务。端口：**TCP 6000**（图传 worker 等遥控器连接）、UDP 6002（触控）、UDP 6003 + TCP 6004（QGC 指令桥）、11311（roscore）、8000（nginx webdav）。`rviz_full_stream2.py` = supervisor/worker 双进程：supervisor 忽略 SIGINT/SIGTERM，拉起 `--worker` 子进程并监控心跳 `/dev/shm/rviz_stream_heartbeat.txt`（>4s 超时 kill -9、死后 2s 重启）；worker 监听 6000，遥控器连上后 mss 截屏（虚拟屏 DP-1 1920x1080）+ JPEG 推流，QGC IP 写 `/dev/shm/qgc_ip.txt`。设置 `~/.config/liyumen/stream_settings.json`（分辨率必须在白名单 {(640,360),(960,540),(1280,720),(1600,900),(1920,1080)}）。日志：`~/catkin_ws/src/task_run_scripts/log/latest/`。

**排查顺序（图传挂了先查 roscore，别在流代码里瞎找）**：
1. `ss -tlnp | grep 6000`：supervisor 在但 6000 不在 = worker 崩循环（`log/latest/rviz_full_stream2.log` 每 2 秒一条"图传进程崩溃"）。
2. **worker 异常被 `except Exception as e: sys.exit(1)` 吞掉，日志永无 traceback**——前台跑 `python3 -u rviz_full_stream2.py --worker` 也可能静默 exit(1)。要拿真异常用 traceback 脚本（见 references/qgc-stream-services.md）。
3. 看 `log/latest/roscore.log`：worker `rospy.init_node` 依赖 roscore，**roscore 挂了 worker 必崩**。roscore 崩的典型根因 = `rosversion` 解析坏 XML：`ParseError: not well-formed (invalid token)` → `RLException: Cannot load command parameter [rosversion]`。
4. 定位坏 package.xml：`find /home/fouruav/catkin_ws/src /opt/ros/noetic/share -name package.xml` 逐个 `python3 -c "import xml.etree.ElementTree as ET; ET.parse(...)"` 验证。
5. **最常见的坏法：git rebase/merge 冲突标记残留在 package.xml**（`<<<<<<<`/`=======`/`>>>>>>>` 非合法 XML；`git status -s` 显示 `UU`）。根因链：冲突标记 → XML 崩 → rosversion 崩 → roscore 起不来 → worker 崩循环 → TCP 6000 不监听 → 投屏连不上。
6. 修复：备份（`cp package.xml package.xml.bak.$(date +%Y%m%d_%H%M%S)`）→ 删冲突标记行**保留两侧内容**（合并语义：两边改动都要）→ `git add` → `git -c core.editor=true rebase --continue`（防编辑器卡死）→ 验证 `source devel/setup.bash && rosversion roslaunch` 有版本号输出。
7. 重启整套（autostart 不自动重拉，必须手动）：
   ```bash
   pkill -TERM -f start_qgc_servers.sh; pkill -TERM -f "nx_touch_server2|nx_command_server2"; sleep 2
   pkill -9 -f "rviz_full_stream2|nx_touch_server2|nx_command_server2|start_qgc_servers"; pkill -9 -f "rosmaster|rosout"; sleep 2
   pgrep -af "nx_|rviz_full_stream|rosmaster|roscore|start_qgc" || echo CLEAN   # 零残留才拉新
   cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code && nohup bash start_qgc_servers.sh >/dev/null 2>&1 & disown
   sleep 25; ss -tlnp | grep -E "6000|6004|11311"   # roscore 启动 >20s（.ros/log 大时 disk usage 检查慢）
   ```
   验证：6000/6004/11311 都 LISTEN；心跳 age<2s；日志出现 `等待遥控器连接 (端口 6000)` 且无崩溃行。

**坑（全部实测踩过）**：
- **旧 supervisor 忽略 SIGTERM**（`signal.SIG_IGN`），且 `pkill -f start_qgc_servers.sh` 匹配不到它（命令行不含该串）→ 必须 `pkill -9 -f rviz_full_stream2`。杀不干净 = 新旧 supervisor 抢 6000 端口，新 worker bind 失败继续崩循环。重启前必须 pgrep 确认零残留。
- worker 前台跑静默退出 ≠ 没崩，是异常被吞；用 traceback 脚本抓真异常（见 references/qgc-stream-services.md）。
- `.ros/log` 超 1GB 时 roscore 启动警告且变慢（`source /opt/ros/noetic/setup.bash && rosclean purge -y` 可清，不影响运行）。
- 完整案例实录（命令输出、git 前后状态、重启验证）：`references/qgc-stream-services.md`

#### 4. FAST-LIVO2 PCD 导出与优雅退出（建图结束 PCD 损坏问题）

- **🚨 当前状态（2026-08-07 会话末）：全部本地修复已被 git 拉取覆盖（用户主动"放弃所有修改"）**。用户执行 4 仓库 `git fetch && git reset --hard origin/<分支>` 到 GitHub 最新（NX_code `f7c22a8` / FAST-LIVO2 `ab247ad` / task_run_scripts `c1639ad` / SUPER `6be0fb8` / catkin_ws 根 `5a7c5ec`），`~/rollback_snapshots/` 快照目录也已不存在。**下述方案 B / v2 / optkill / hazard2 等修复全部不再生效**（回到上游 SIGTERM 强杀行为），FAST-LIVO2 源码已更新需重新 `catkin_make -j2` 编译。若用户要求重做修复，本条目的历史记录（含实施 diff）就是施工图；动手前先确认 git 现状（`git status -s` + `git log --oneline -1`），别假设修复还在。仓库是**嵌套子模块**（catkin_ws 根 → src/task_run_scripts → scripts/NX_code），reset 要逐层执行，子模块指针用 `git submodule update --init --recursive` 同步；SUPER 子模块曾因本地 `real_flight.yaml` 修改挡住 checkout，先 `git checkout -- <file>` 再 reset。
- **⚠️ 飞行安全配置（2026-08-07 用户实测反馈，安全级）**：`MID360_Fly_Lidar.yaml`（高质量建图）在**树林等点云密集场景**导致 FAST-LIVO2 延迟过高 → PX4 定点模式严重漂移 → 迫降。建图模式算力≈飞行模式 2 倍：point_filter_num=1(不抽稀)/dense_map_en=true(全量点)/lio max_iterations=5/voxel_size=0.5/max_points_num=50/half_map_size=100 vs 飞行模式 point_filter_num=3/dense_map_en=false/max_iterations=3/voxel_size=1.0/max_points_num=20/half_map_size=50。**飞行任务用 `MID360_Fly.yaml`（低算力保实时），专门建图（不飞/慢扫）才用 Fly_Lidar**。折中档 = 保 dense_map_en=true（PCD 质量）但 point_filter_num=2、max_iterations=3、voxel_size=0.8、max_points_num=30；用日志 `LIO Mapping Time` 表量化验证（ICP 行 <50ms 安全、>100ms 危险）。用户已把 MODE 分支改为 lidar 也用 Fly.yaml（飞行安全优先）——**这是有意取舍，别再当 bug 报**。
- **PCD 无 RGB 颜色 → CloudCompare 全蓝**：PCD 头部 `FIELDS x y z intensity normal_x normal_y normal_z curvature`（无 rgb 字段），CloudCompare 默认显示蓝色。方案：① 临时看——CloudCompare 里 `Edit → Scalar fields → Set current` 选 intensity 或 z，再 `Set color → From scalar field`；② 一键转彩色——`python3 ~/catkin_ws/src/task_run_scripts/scripts/colorize_pcd.py <in.pcd>` 生成 `<名>_color.pcd`（FIELDS x y z rgb，按高度 Z jet 色图）；③ 照片级颜色需相机投影（img_en=1 时 savePCD 存 laserCloudWorldRGB）或 3DGS 重建——纯雷达 XYZI 数据变不出真实颜色。着色脚本本体在技能 `scripts/colorize_pcd.py`。
- **half_map_size（local_map）**：局部地图内存裁剪半径（米），配合 `map_sliding_en`+`sliding_thresh`（默认 8m）——移动超过阈值后清掉当前位置 ±half_map_size 之外的体素（`clearMemOutOfMap`）。**当前两配置 map_sliding_en=false → 该参数不生效**（地图无限累积，长时间建图内存持续涨）；开启后 PCD 仍完整（`pcl_wait_save` 独立累积不受裁剪影响）。建图 100m / 飞行 50m 的差异即内存-算力取舍。
- PCD 机制：`LIVMapper.cpp` 的 `savePCD()` 只在 `run()` 的 `while(ros::ok())` 主循环退出后调用一次。`pcd_save/interval=-1`（`task_run_scripts/config/MID360_Fly_Lidar.yaml`）→ 所有帧合并写 `FAST-LIVO2/Log/pcd/all_raw_points.pcd`（几 GB），写盘耗时几十秒。
- **roscpp 只响应 SIGINT（Ctrl+C），不响应 SIGTERM** → `kill -TERM` 直接杀进程，savePCD 根本没机会执行或写到一半。
- 损坏链条：停止流程（`nx_command_server.py` `_kill_tracked_radar_groups()`：SIGTERM→sleep 2.0→SIGKILL；`kill_lidar_processes()`：pkill -SIGTERM→1.5s→pkill -9；`start_fastlivo2.sh` cleanup()：SIGTERM→最多3s→SIGKILL）把正在写几 GB PCD 的进程强杀。
- **⚠️ 首版修复曾整体回退（2026-08-05 中午）——以用户现场验收为准**：首版修复在功能上成功（多轮实测 PCD 完整差 0 B、零残留），但用户现场使用后判定**关闭等待 30~90s 观感不可接受**（"每次启动关闭都要好久，总以为是卡住了"），要求回退到修复前版本。回退 = 3 文件 `cp` 回 `.bak.20260804_123240`（start_fastlivo2.sh / nx_command_server.py / mapping_MID360.launch），修复版存档为 `.fix.20260805_1640` 便于反悔。
- **✅ 方案 B 已落地并生效（2026-08-05 下午，备份 `.bak.planB.20260805_171029`，服务已重启加载）**：用户最终拍板的三优先级——**① 进程零残留 = 绝对前提（一票否决）② PCD 保存完整 ③ 启动/关闭响应灵敏**。方案 B = **异步关闭 + SIGINT 保存 + PCD 时间戳 rename**，两处实现：
  - `start_fastlivo2.sh` cleanup()：先 `kill -INT "$FASTLIVO_PID"` + `pkill -INT -f fastlivo_mapping` 触发 savePCD → 轮询 `pgrep -f fastlivo_mapping` 等它写完（最长 120s）→ `mv all_raw_points.pcd` 为 `$(date +%Y%m%d_%H%M%S).pcd` → 再 TERM 其他进程 → SIGKILL 兜底。
  - `nx_command_server.py`：`_kill_tracked_radar_groups()` SIGTERM→SIGINT、sleep(2.0)→轮询等进程组消失（90s）；pkill 兜底 SIGTERM→SIGINT、1.5s→15s；**尾部新增 rename**（`all_raw_points.pcd` → `time.strftime("%Y%m%d_%H%M%S")+".pcd"`，同秒冲突加 `_1/_2` 后缀防覆盖，`os.rename` 原子操作）；**KillLiDAR 处理异步化**——`threading.Thread(target=kill_lidar_processes, name="bg_kill_lidar", daemon=True).start()`，主线程毫秒级回复"⏳ 关闭指令已提交"，用 `any(t.name == "bg_kill_lidar" and t.is_alive() ...)` 防重复指令。
  - **效果**：点关闭立即响应（异步）；PCD 完整（SIGINT + 不限时等到写完）；每次建图一个独立时间戳文件（**用户明确要求：PCD 命名要可读时间戳 `20260805_170123.pcd`，不覆盖历史**）；零残留（SIGKILL + pkill -9 + rosnode cleanup 兜底）。"关闭后立即启动"会等上一轮收尾，需要 `⏳ 已等 X 秒` 进度提示 + 可选"强制启动丢弃旧 PCD"逃生口。
- **用户拒绝过的方案（别再提）**：① interval>0 分片（产生大量需手工合并的中间文件）；② 给保存设硬上限（"超过30秒PCD白录报废"——大 PCD 必报废，缺陷致命）；③ 先等旧进程写完再开新（嫌慢）；④ launch 加 `timeout="120"`（roslaunch 报 `unknown <node> attribute 'timeout'` 并忽略——15s 默认是节点**启动**超时非关闭超时）。
- **用户明确的三优先级（2026-08-05，做关停流程设计的准则）**：**① 进程零残留 = 绝对前提（"这个现象绝对不能够存在，即使保存不上也不能残留节点"）② PCD 保存完整 ③ 启动/关闭响应灵敏**。注意顺序：零残留是**一票否决项**，PCD 完整性次之，响应灵敏排最后（用户说"其次启动和关闭雷达相应灵敏"）。别用"响应快"去换零残留——宁可多等几秒也要清干净。
- **方案演进史（2026-08-05，设计过程，最终已落地为上面的方案 B）**：用户先否了"有界等待 15~25s"（"那岂不是说超过30秒，PCD直接白录，报废了"——**给保存设硬上限 = 大 PCD 必报废**，这个缺陷是致命的）；随后否了"先等旧进程写完再开新"（嫌慢）；最后提出"中间缓存区"构想。**收敛设计（异步关闭）**：`KillLiDAR` 处理从同步 `kill_lidar_processes()` 改为丢进 `threading.Thread(daemon=True)` 后台执行（文件已有成熟先例：`threading.Thread(..., daemon=True).start()` 用于 file-transfer/zip 任务，且已 `import threading`）→ 主线程毫秒级回复"已提交关闭"；后台线程 SIGINT → **不设保存上限，等到写完成** → 兜底清理 → 零残留。用户接受的前提是："关闭后进程还会活几十秒写 PCD"**不是残留**——区别 = 残留是没人管的孤儿（无限期），后台保存是有人盯着、到点必走（写完自退/超时强杀/启动前兜底清理三重保险）。"关闭后立即启动"会等后台收尾，但要有 `⏳ 等待上一轮 PCD 保存完成，已等 X 秒...` 进度输出 + 可选"强制启动（丢弃旧 PCD）"逃生口，默认等完（PCD 完整优先于启动快）。
- **写盘速度真相（先测再设计缓存，别过度设计）**：`img_en=0`（无相机）时 savePCD 走最简路径——**一次 `writeBinary`，不做降采样**，419MB 写 NVMe 固态理论仅 2~5s。用户感知的"90s 慢"大部分是轮询等待 + roslaunch 生命周期 + 清理流程的固定开销，不是写文件本身。设计任何"缓存区/异步转换"前，先实测 SIGINT→PCD 写完的真实耗时（在日志打 `⏳ 开始保存` / `✅ 完成（耗时 X 秒）` 两行时间戳）；若 ≤10s，异步方案就够，缓存区是最后手段（需改 C++ 两段式 + 重编译 + 转换脚本）。
- **中间缓存区构想（用户提出，评估过可行性）**："快照 + 异步转换"——关闭时 fastlivo 把内存点云快速 dump 到 `/tmp` 缓存（秒级）→ 进程立即退出（释放雷达+资源）→ 后台低优先级转换器把缓存转成标准 PCD。障碍：① **PCD 文件冲突**——新旧进程最终都写同一个 `all_raw_points.pcd` 必坏，解法是每次启动用独立文件（`Log/pcd/<时间戳>/all_raw_points.pcd`，顺带解决"新图覆盖旧图无备份"）；② **雷达独占**——保存阶段不需要雷达（可提前杀 livox driver 释放），但新建图需要雷达，故旧进程必须快速退出；③ **Jetson NX 算力竞争**——旧进程写几 GB 抢 CPU/内存，新建图实时性下降甚至 OOM，这是硬件限制绕不开，双跑时要在启动时明确警告用户。
- **PCD 路径（写死位置）**：`CMakeLists.txt:40 add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")` → `~/catkin_ws/src/FAST-LIVO2/Log/pcd/all_raw_points.pcd`。单文件覆盖式：新图直接覆盖旧图，**旧图无备份**（要留档需在覆盖前自行拷贝）。同目录还有 `lidar_poses.txt` 轨迹文件。
- **回退/改代码的运行生效陷阱（踩过）**：文件回退 ≠ 运行生效。运行中的 `nx_command_server2.py`（由 `start_qgc_servers.sh` 于 16:14 拉起，PID 3526）内存里仍是启动时的代码——**Python 进程加载的是启动那一刻的文件内容**。要真正回退必须重启服务：`pkill -f nx_command_server2.py; pkill -f start_qgc_servers.sh; bash scripts/NX_code/start_qgc_servers.sh`（重启会短暂断开 QGC/遥控器连接，先确认无任务）。任何"改文件→重启服务→验证"三步都要把重启列为显式步骤。
- **✅ 修复方案核心（2026-08-04 首版，核心思想被方案 B 继承）**：保持 interval=-1（用户拒绝 interval>0——会产生大量需手工合并的中间文件）；对建图进程发 **SIGINT**（kill -INT / os.killpg SIGINT / pkill -SIGINT）触发优雅退出，然后**轮询等待 `fastlivo_mapping` 节点退出（90~120s）**再 SIGKILL 兜底（轮询对象是节点进程而非 roslaunch PID——roslaunch 收到 SIGINT 后可能先退，子节点仍在写 PCD）。
- **launch 不可改（已踩坑并回滚）**：不要给 laserMapping 节点加 `timeout="120"`——roslaunch 报 `WARNING: unknown <node> attribute 'timeout'` 并忽略该属性。roslaunch 的 15 秒默认 timeout 是节点**启动**超时（启动后须注册到 master），**不是关闭超时**，不会在写 PCD 中途 SIGKILL 节点。SIGINT+轮询修复单独就够（实测 205MB/640万点 PCD 差 0 B）。若在日志看到这条 WARNING，说明有人（或旧改动）加过该属性，删掉即可。
- **开新一轮建图会自动先杀旧进程**：`nx_command_server.py:461` 在 `_launch_and_track_radar()` 前无条件 `kill_lidar_processes()`（"先关闭旧雷达"）→ 建图中直接开新一轮 = 强制中断旧图（保存为截至当前快照，未建完部分丢失），且启动阻塞最多 90s 等旧进程写完 PCD。正确流程：先停干净 → 确认 PCD 完好 → 再开新一轮。
- **二次启动卡根因（2026-08-05 方案 B 后用户反馈）**：启动路径 `_handle_radar_launch()` 里启动前**同步**执行 `kill_lidar_processes()`——若上一轮异步关闭线程（`bg_kill_lidar`）还在后台写 PCD（最长 90s 窗口），启动的同步清理会撞上它干等，且第二层 `sleep(15)` 是**固定等待，不管 PCD 大小、不管还有没有进程**（用户确认过：15s 每个 pcd 不管大小都要等——是的）。修复（2026-08-06 已全部实施，备份 `.bak.startasync.*`）：① **启动也异步化**——UDP 指令循环里 `Odin`/`Odin-MindCloud` 调用改为 `threading.Thread(target=_handle_radar_launch, name="bg_start_radar", daemon=True).start()`（主循环立即回复"⏳ 启动指令已提交"，防重复指令用 `any(t.name == "bg_start_radar" and t.is_alive())`）；② 启动路径检测 `bg_kill_lidar` 线程 `is_alive()`，在跑则 `join(timeout=120)` 等它完成（避免误杀新进程，见 v2 致命坑）；③ join 完成后**跳过重复完整清理**——`waited_prev` 标志 + 快速 `pgrep` 验证（有残留才 `kill_lidar_processes()`，无残留直接启动，省 15~90s）；④ 第二层 `sleep(15)` 改动态检测——`for _ in range(15): pgrep -f "fastlivo_mapping|livox_ros_driver2|laserMapping"` 无进程立即 break（**零牺牲**：正常省 15s，有漏网进程时该等还等）。**直接删 15s 会牺牲 SSH 手动启动/孤儿进程的 PCD**（不在进程组追踪列表、第一层管不到，靠第二层 pkill 兜底给保存窗口）——要去掉只能动态检测，不能硬删。**⚠️ 启动/关闭处理绝不能同步阻塞 UDP 指令循环**（`while True: sock.recvfrom` 6003 端口单线程）——join(120s) 或同步 kill_lidar_processes 一阻塞，所有后续指令（关闭/心跳/启动）全部无响应，界面表现为"卡死"。
- **物理约束（方案 B 后用户追问能否关了马上开、PCD 后台跑、启动不受影响）**：雷达独占 + 同一 `all_raw_points.pcd` 文件冲突 = 旧进程没死透就开新的在任何方案下都不安全。能做到：关闭立即响应 ✅、雷达立即释放（保存阶段不需要雷达，可提前杀 livox driver）✅、PCD 后台写完 ✅；**不能保证**：新进程建图完全不受影响——Jetson NX 上旧进程写几 GB 抢 CPU，新建图实时性下降甚至 OOM。中间缓存区（用户构想：快照 dump 秒级 + 后台低优先级转换）是唯一接近完全不受影响的路线，但要改 C++ 两段式 + 重编译 + 转换脚本，且 PCD 文件冲突要靠独立时间戳目录解决。
- 完成提示（2026-08-04 第二轮，备份 `.bak.notify.*`）：等建图进程退出后检查 `Log/pcd/all_raw_points.pcd`（`du -h` 取大小、`grep -a -m1 '^POINTS'` 取点数——PCD 头部是 ASCII 即使二进制文件也能 grep）→ 打印 ✅"可以安全启动新一轮建图"/⚠️；`start_fastlivo2.sh` 与 `nx_command_server.py` 两处都有。
- **进程组陷阱（实测踩坑）**：roslaunch 的子节点（fastlivo_mapping、livox_ros_driver2_node 等）各有**自己的 PGID**（ps 可见 `pgid=pid` 自成组），`killpg(roslaunch组, SIGINT)` 实际只到 bash/roslaunch，节点靠 roslaunch 转发 SIGINT 才退。所以：① 兜底要 `pkill -INT -f fastlivo_mapping` 双保险；② 轮询"是否写完"看**节点进程名**（pgrep fastlivo_mapping），不能只看 roslaunch 死没死。
- **PCD 文件小 ≠ 保存 bug——先确认建图实际时长再下结论（2026-08-06 实测排查法）**：用户反馈"最新 PCD 只有 6.6MB，与采集路径完全不一致"，但逐项核对发现**建图实际只跑了 ~8 秒**（保存逻辑完全正常）。三证据交叉验证法：① `lidar_poses.txt` 首尾行时间戳差 = 建图真实时长（`head -1` vs `tail -1`，10Hz 雷达 × 行数 ≈ 秒数）；② `grep -c 'LIO Mapping Time' fastlivo.log` 表格数 × ~0.1s ≈ 时长；③ 日志 mtime - 启动时间。另注意日志开头几条 `[ LIO ]: No point!!!` = 启动初期雷达短暂无数据（几秒内恢复），属正常不算故障（看 `Raw feature num` 行出现即恢复）。排查顺序：先算"建图实际跑了几秒"，再谈 PCD 大小是否合理（~10Hz × 每帧 ~2.7 万点 × 32B ≈ 8MB/s 量级）。**教训：PCD 大小异常先查建图时长，别急着怀疑保存逻辑**。
- **🚨 PCD 点数骤减的第二类根因：配置模式被切错（2026-08-06 实测，别归因环境）**：症状 = 建图时长正常（92s，进程活了 97s 未被误杀）但 PCD 只有 60MB（188 万点，每帧 ~4200 点）。取证链：`grep -a 'Raw feature' fastlivo.log | sed 's/.*Raw feature num: //;s/,.*//'` 算每帧平均 → 全程**稳定** ~4200（注意"稳定偏低"≠环境波动，环境空旷是逐帧起伏如 2681→4387→2524）；再核对运行时参数 `grep -a '/publish/dense_map_en\|/preprocess/point_filter_num' fastlivo.log` = `False` + `3`，**完全匹配 `MID360_Fly.yaml`**（飞行模式：dense_map_en=false 降采样 feats_down_body + point_filter_num=3 抽稀），而配置文件 `MID360_Fly_Lidar.yaml` 是 `true`+`1`（建图模式全量点）。**根因：`start_fastlivo2.sh` 第 25 行 `if [ "$WITH_CLOUD" = "true" ]` 分支的 `CONFIG_NAME` 被 git 提交写错成 `MID360_Fly.yaml`**（注释还写着"🗺️ 高质量建图"，值却错了），一行改回 `MID360_Fly_Lidar.yaml` 即修复。排查要点：**PCD 点数异常先查 `start_fastlivo2.sh` 的模式选择逻辑（`sed -n '20,35p'`），别假设 git 提交是对的**；`git diff HEAD` 无输出 = bug 已在提交里，不是本次会话改的。PCD 攒的是 `dense_map_en ? feats_undistort : feats_down_body`（LIVMapper.cpp:442），dense_map_en=false 时每帧点少 5~10 倍。
- **QGC 按钮 → 命令映射**：地面站日志 `log/YYYYMMDD_HHMMSS/nx_command_server2.log` 里 `收到 QGC 指令: Odin-MindCloud` = 启动建图、`KillLiDAR` = 停止。`nx_command_server2.py` 只是 317 字节包装（`from nx_command_server import start_command_server`），**真逻辑在 `nx_command_server.py`**——改代码认准后者，别被 server2 文件名带偏。启动流程 `_handle_radar_launch()`（约 440-470 行）：先 `kill_lidar_processes()` 再 `_launch_and_track_radar()`。
- **UX 教训（用户误判"关闭没用"）**：PCD 保存 30~90s 期间**零输出** → 用户以为按钮坏了，又点一次启动 → 两次清理叠加、时序混乱。正确做法：收到停止指令立即打 `⏳ 正在保存 PCD，请等待...`，保存完打 `✅ 完成`。另外 `nx_command_server.py` 打印行仍显示 `-> SIGTERM 进程组` 但实际发的是 SIGINT——**纯显示 bug**（print 字符串没同步改），别被日志误导，看 `os.killpg(pgid, signal.SIGINT)` 的代码才是真值。
- 逐文件逐行根因图与改法（含实施 diff）：`references/fast-livo2-pcd-export.md`。
- 关停方案演进、异步关闭设计、缓存区可行性评估：`references/fast-livo2-shutdown-design.md`。
- **v2 落地后的精简速查（C++ 时间戳命名代码、异步误杀修复、PCD 大小异常取证顺序、服务重启命令）**：`references/fast-livo2-pcd-lifecycle.md`。
- **回退保障机制（用户明确要求"我要保证我有一个后路"，2026-08-06 建立）**：任何重大修改（改 C++、重构关停逻辑）实施后，主动为用户建立 `~/rollback_snapshots/` 快照目录 + 一键回退脚本。三档快照：`planB/`（脚本层 SIGINT+rename 方案，4 文件含 LIVMapper 原始版）、`original/`（8月2日最原始版，2 文件）、`current/`（当前版含全部修复）。回退命令 `bash ~/rollback_snapshots/rollback.sh planB|original|current`，脚本自动提示重编译（`catkin_make -DCATKIN_WHITELIST_PACKAGES=fast_livo -j2`）和重启服务步骤。**用户心理**："后路"= 随时能回到已知可用状态，不是要现在回退——建好保障机制后继续推进新方案即可。实施大改动前先确认快照齐全，并明确告诉用户"你的后路在哪、一条命令怎么回"。
- **🚨 二次启动慢的真瓶颈诊断（2026-08-07，推翻"缓存区必要"假设）**：用户反馈二次启动 20~30s 太久，原计划上中间缓存区（改 C++）。**先诊断再动工**，证据链推翻前提：① PCD 341MB 的 mtime = KillLiDAR 指令**同一秒**（`stat -c '%y'` vs server 日志指令时间戳）→ **写盘几乎瞬间完成**（NVMe 上 writeBinary 几秒），"等 PCD 写完"根本不是瓶颈；② 真正耗时 = 关闭线程**串行等待链**：`_kill_tracked_radar_groups()` 轮询等**整个进程组**消失（最长 90s，rviz/livox 拖后腿）→ pkill → 动态等待 15s → rosnode 清理，而启动线程 `join` 等它**全部**跑完。**结论：不需要缓存区、不需要改 C++、不需要重编译**——只要把关闭线程改成"只等 `fastlivo_mapping` 进程退出（=PCD 写完，几秒）→ 立即 SIGKILL 其他所有"，二次启动即可从 20~30s 降到 3~8s，纯 Python 半小时工作量。**✅ 已实施（2026-08-07，备份 `.bak.optkill.20260807_100904`，服务重启生效）**：`_kill_tracked_radar_groups()` 的 90s 进程组轮询改为 `for _ in range(60): pgrep -f fastlivo_mapping` 无则 break；动态等待 pgrep 模式从 `fastlivo_mapping|livox_ros_driver2|laserMapping` 瘦身为只 `fastlivo_mapping`（livox 不再拖满 15s）。**✅ 隐患2双保险已实施（备份 `.bak.hazard2.20260807_110038`）**：killpg SIGINT 后补 `pkill -INT -f fastlivo_mapping`（防 roslaunch 转发失败 PCD 丢），并顺带修正打印 `-> SIGTERM 进程组` → `-> SIGINT 进程组`（纯显示 bug）。两次改动零编译纯 Python。**方法论：先量化每段耗时（mtime/指令时间戳差值、日志阶段时间），确认瓶颈在哪一步，再选方案——缓存区/改 C++ 是最后手段，别默认需要**。诊断命令：`stat -c '%y' Log/pcd/*.pcd` 对比 server 日志 `收到 QGC 指令` 时间戳；`grep -a '收到 QGC 指令\|SIGKILL 进程组\|清理完毕' log/*/nx_command_server2.log` 重建时间线。**零残留验证**：关闭后 `pgrep -af 'fastlivo|livox|laserMapping|roslaunch|imu_tf|rviz' | grep -v pgrep` 只应剩常驻 `rviz_full_stream2.py`（图传服务，不能杀）。
- **用户会问"大概需要多长时间"**（大工程开工前）：给分阶段时间表（改 C++ / 写转换器 / 集成 / 重编译 / 联调测试各多少分钟）+ 总计 + 说明哪些阶段是等待时间（编译后台跑）、哪些需要用户配合（实测）。可提供分期选项（一次性做完 / 先做代码后联调 / 只做备用模块）。
- **用户提出的"任务队列"设计（缓存区方案的增强，评估过可行性）**：连续快速"关→开→关→开"会产生多个缓存文件待转换。转换器按**文件名时间戳 FIFO** 排队：`.raw`(待转) → `.converting`(转换中) → 转完移到正式目录，失败标记 `.failed` 不阻塞队列，重启后扫描续转。澄清过的物理事实：缓存区方案下旧进程快照后**立即退出**（内存+雷达释放），所以"两个建图进程同时占内存 OOM"不会发生——是"旧图收尾写盘与新图建图重叠"，不是两个完整建图并行（雷达仍独占）。

- **✅ v2 已实施（2026-08-06，最终版）**：① `nx_command_server.py` 第二层 `sleep(15)` 已改为**动态检测**——`for _ in range(15): pgrep -f "fastlivo_mapping|livox_ros_driver2|laserMapping"`，无进程立即 break（备份 `.bak.v2.20260806_212058`）；② **PCD 时间戳命名已下沉到 C++ 源头**——`LIVMapper.h` 新增成员 `std::string save_session_id_;` + `#include <ctime>`，`LIVMapper.cpp` 构造函数用 `strftime(buf, "%Y%m%d_%H%M%S", tm_)` 生成会话 ID，`savePCD()` 文件名改为 `"Log/pcd/" + save_session_id_ + ".pcd"`（降采样为 `_downsampled.pcd`），备份 `.bak.v2cpp.20260806_212118`，需 `catkin_make -j2` 重编译生效（Jetson 约 10~20 分钟，`-j2` 防过热）；改 C++ 后脚本侧 rename 变成冗余可删。**C++ 头文件锚点陷阱**：`img_en` 实际声明是 `int img_en = 1, imu_int_frame = 3;`（不是 `bool img_en;`）——改头文件前先 `grep` 真实成员格式，别用猜的锚点（首次 patch 就因锚点猜错而 AssertionError）。
- **🚨 v2 关键 bug：异步关闭线程会误杀新启动的进程（2026-08-06 发现并修复，PCD 只存 8 秒的真凶）**：症状 = 建图 2 分钟但 PCD 只有 6.6MB（~8 秒数据）。排查三证据：`lidar_poses.txt` 首尾时间戳差 8s、日志 mtime 21:27:50 - 启动 21:27:37 = 进程只活了 13s、日志里 feature 行正常 1500 行（建图确实跑了 2 分钟）。根因 = **上一轮的 `bg_kill_lidar` 异步清理线程没有"代际"概念**——用户等不及上轮收尾就点启动，新进程起来后，上轮还在跑的清理线程的 `pkill -SIGINT/-9 -f fastlivo` 模式匹配到了新进程，把它 13 秒后误杀。修复（3 层保护，备份 `.bak.join3.20260806_213629`）：① **启动路径 join 等待**——`_handle_radar_launch()` 里 `kill_lidar_processes()` 前，遍历 `threading.enumerate()` 找 `name == "bg_kill_lidar" and is_alive()` 的线程 `_t.join(timeout=120)`，打印 "⏳ 等待上一轮关闭收尾完成，避免误杀新进程..."；② **`kill_lidar_processes()` 加互斥锁**——`_kill_lidar_lock = threading.Lock()`，`acquire(blocking=False)` 失败则打印"已有清理流程在运行，跳过"并 return；③ 原函数体改名 `_kill_lidar_processes_impl()` 由新函数包一层 `try/finally` 释放锁。**Python 缩进 patch 陷阱**：直接在函数体内加 `try:` 需要整体重缩进整个函数体（易错）；正确做法是"包装函数"——原函数改名 `_xxx_impl`，新同名函数只做加锁+调用+finally 释放，零重缩进。修复后启动路径若检测到上轮收尾还在，会**等它清完再启动**（界面不卡、有提示），新进程不再被误杀。**排查此类"数据量异常小"问题的顺序**：先确认建图实际时长（pose 时间戳差/日志 mtime），再怀疑保存逻辑——本例保存逻辑从头到尾正常，是进程被误杀。

#### Pitfalls (all hit in real sessions)

- **🔴 「ping 不通」不等于「物理链路断开」——先看 `ip -s link show` 和 `arp`。** 本次实测：`ping <NET_RC>.99` 100% 丢包，
  我据此断言「网线没插/对端没上电」，**结论错了**。同一时刻 `ip -s link show eth0` 显示 `LOWER_UP` + RX/TX 都有包 +
  `errors/dropped=0`，`arp -a` 里还躺着同网段的 `.100` 带 MAC —— 链路完全正常，只是 **`.99` 这个 IP 根本不存在**
  （`arp` 显示 `<incomplete>`）。**判物理层的三件套：`LOWER_UP` 标志 + errors/dropped 为 0 + arp 里有活 MAC。**
- **🔴 「端口在监听」不等于「服务正常」，也不等于「接口不能用」——先搞清楚协议。** 本次用 `curl http://ip:6000/` 拿到
  `HTTP/0.9` + `HTTP 000`，我先判成「服务虽监听但不响应、推流进程有 bug」，**又错了一次**。真相是 6000 跑的是
  自定义二进制协议（`<4字节长度><JPEG>`）+ `AUTH:` 握手，curl 只是**拒收非 HTTP 响应**。
  **判据：`curl` 报 `Received HTTP/0.9` ⇒ TCP 已连上、服务活着；真正的"不通"表现为 `Failed to connect`/`Connection refused`/超时。**
- **🔴 定位「某进程由谁托管」必须读 `/proc/<pid>/cgroup`，不能看 PPID。** 看 PPID 只会得到 `systemd --user`（PID 1554），
  而 `systemctl status $(cat /proc/<pid>/cgroup | grep -o '[^/]*\.service' | head -1)` 这种写法会抓空、退化成
  「列出 `user@1000.service` 整棵 cgroup 树」，让人误以为所有服务都在同一个单元里。**正确写法：
  `cat /proc/<pid>/cgroup`（或 `--user` 下加 `systemctl --user status <unit>`）。**
- **🔴 用户级 `journalctl --user -u <svc>` 可能报 `No journal files were found`**（journald 未持久化）。
  此时**别放弃日志**——`systemctl --user stop` 后**前台直接跑**那个进程（带上 `/proc/<pid>/environ` 里的环境变量），
  握手/崩溃日志就会直接打到终端。这比"没有日志所以查不动"有效得多。
- **FAST-LIVO2 实际加载的配置在 `task_run_scripts/config/`**（launch 里 `$(find task_run_scripts)/config/$(arg config_name)`），**不是** `FAST-LIVO2/config/` 下的同名 yaml（那份 pcd_save 段是注释掉的旧拷贝，会误导）。改参数前先确认 launch 指向。
- plink 远程命令里 `find -exec cat {} \;` 的 `\;` 会被 PowerShell 吞掉报 ParseException → 改用 `find ... -print` 拿到路径后单独 `cat`。
- roslaunch by filename only (`roslaunch px4.launch`) → "no element found". Must be `roslaunch <pkg> <file>.launch`; the launch may live in a different package than the driver (e.g. msg_MID360s.launch is in `task_run_scripts`, NOT `livox_ros_driver2`).
- Livox config JSON: `host_net_info` must be an **object** with `cmd_data_ip`/`push_msg_ip`/`point_data_ip`/`imu_data_ip` — an array with `host_ip` makes the SDK report "all livox lidars config is empty".
- `rostopic hz` → "Cannot load message class ... Are your messages built?" = env issue: `source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash` first (PYTHONPATH needs devel/dist-packages). Verify with `find ~/catkin_ws/devel -name "*CustomMsg*"`.
- Launch files suddenly 0 bytes (bulk): accidental `>` redirect from pasted XML wipes whole directories at once (identical mtime). Restore with `git checkout -- launch/` — the workspace is a git repo; check git BEFORE hand-rebuilding files.
- Editing remote config cleanly: use python3 `json.load`/`json.dump` (targeted field edits, stays valid JSON) over whole-file heredocs; back up first (`cp x x.bak`).
- Don't ping your own freshly-added IP to "verify the radar" — verify against the radar's actual IP from the config.

#### Verification

- FCU: `/mavros/state` connected:True
- LiDAR: `rostopic hz /livox/lidar` ≈ 10 Hz
- Both topics visible in `rostopic list`
- FAST-LIVO2 PCD 完整性: `python3 ~/catkin_ws/src/task_run_scripts/scripts/check_pcd.py`（逐字节校验，差 0 B = 完整；脚本同款在 `scripts/check_pcd.py`）
- **停止后节点残留（用户一票否决项）**: `timeout 10 rosnode list` 应只剩常驻节点（本机为 `/full_screen_streamer`、`/nx_qgc_bridge_daemon_*`、`/rosout`——视频流/QGC 桥，不算残留）；`pgrep -af 'fastlivo|livox|rviz|laserMapping|mavros'` 应为空。出现 laserMapping/livox/imu_tf 节点或 fastlivo 进程 = 残留，必须修。反复启动/关闭测试（用户的标准验收流程）以残留检测为首要目标、PCD 完整性与关闭顺序为次要。一键验收脚本 `check_run.sh` 曾装在 NX（`~/catkin_ws/src/task_run_scripts/scripts/`）但**已按用户要求删除**（2026-08-05："把之前做测试的一些脚本清除一下，最原始的备份要留一份"）——验收时用手动三连（rosnode list + pgrep + check_pcd.py）或从技能 `scripts/check_run.sh` 重新部署；注意排除常驻视频流进程 rviz_full_stream2 避免误报。清理原则：只留 `.bak.20260804_123240` 原始备份 + `check_pcd.py` 工具，中间备份（notify/fix/planB/revert 版）和测试脚本一律删。
- **rosnode list 里的幽灵节点 ≠ 真残留**：被 SIGKILL 的节点来不及向 master 注销，`rosnode list` 会残留 `/rviz_<epoch>` 之类带时间戳的名字，但 `pgrep` 已无该进程——无害，`rosnode cleanup` 可清。判残留以**进程是否存在**为准（pgrep 空白=干净），别被 rosnode list 的幽灵名字吓到。
- **ps 显示 PPID=systemd（PID 1835）≠ 有服务自动拉起**：孤儿进程会被 systemd --user 收养，父进程显示 systemd 只是"真父进程已死被收养"，不是看护服务。判定"谁拉起的"要看 **PGID**（同一 PGID = 同一批启动）和启动时间（`ps -o lstart`），并与 nx_command_server2.log 的 `收到 QGC 指令` 时间戳对照——"rviz 自己又启动"通常是 QGC 又发了一次 Odin-MindCloud 启动指令，不是 watchdog。


---

### 来源技能 `uav-ros-sensor-diagnostics`

> Diagnose flight controller (PX4/ArduPilot via MAVROS serial) and LiDAR (Livox MID360 via Ethernet) connections on a ROS companion computer (Jetson NX). Covers serial recon, radar subnet config, launch-file inspection, MAVROS verification, and git recovery of zeroed files.

#### Workflow preference (指挥官模式 — commander mode)

This user prefers: **agent performs the initial connection, then hands command batches to the user, who runs them on the remote terminal and pastes output back**. Verify each batch's output before issuing the next. Keep commands read-only until the picture is clear. Do NOT run remote commands yourself if the user said they will operate.

#### Phase 1 — Recon (read-only)

```bash
ls -l /dev/ttyTHS* /dev/ttyS* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null   # serial devices (Jetson UART = ttyTHS*)
ip -4 addr show | grep -E "^[0-9]+:|inet "                            # NICs + subnets
ls ~/catkin_ws/src/                                                    # packages reveal the stack
cat ~/catkin_ws/src/*/launch/*.launch 2>/dev/null | head -100          # launch files = the system's "manual"
ps aux | grep -iE "livox|mavlink|px4|ros" | grep -v grep               # what is already running
```
- **Jetson UART ports are `/dev/ttyTHS*`** — the flight controller usually lives there, NOT on ttyACM/ttyUSB
- Package names tell the story: `mavros`, `livox_ros_driver2`, `FAST_LIO/FAST-LIVO2`, `slam_to_mavros` → PX4 + Livox + LIO-SLAM stack
- Note NIC MAC prefixes: onboard Jetson NIC has NVIDIA OUI (3c:6d:66...), USB-adapter NICs often have locally-administered MACs (86:...)

#### Phase 2 — Flight controller (MAVROS over serial)

- Find FCU port + baud in launch files: `fcu_url` e.g. `/dev/ttyTHS0:921600`
- **Raw serial test MUST set the baud rate** — plain `cat` uses 115200 default and shows nothing even when the FC is alive:
  ```bash
  timeout 3 bash -c 'stty -F /dev/ttyTHS0 921600 raw -echo; cat /dev/ttyTHS0' | xxd | head -20
  ```
  Expect MAVLink v2 frames `fd ...` (v1 = `fe ...`). No output → retry with 115200; still nothing → FC unpowered / cable wrong / wrong port.
- Start MAVROS: `source ~/catkin_ws/devel/setup.bash && roslaunch <pkg> <file>.launch`
  - **Success markers**: `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot` + `MAVROS started. MY ID 1.240, TARGET ID 1.1` + `IMU: ... detected!` + `RC_CHANNELS message detected!`
  - `CMD: Unexpected command 520` is a benign PX4 reply, not an error
- Verify topics: `rostopic echo /mavros/state -n 1` (connected/armed/mode), `rostopic hz /mavros/imu/data --window 10` (~50 Hz normal)

#### Phase 3 — LiDAR (Livox MID360 over Ethernet)

- Read driver config JSON: `host_net_info.cmd_data_ip` = the **host** IP the NIC must carry; `lidar_configs[].ip` = the **lidar's** IP
- Add the required subnet to the NIC the radar cable is in (temporary, non-destructive):
  ```bash
  sudo ip addr add <NET_LAN2>.5/24 dev eth1
  ```
- **Ping the LIDAR's IP, not the host IP** — pinging your own host IP always succeeds (false positive!)
- **IP-conflict trap (radar unreachable root cause)**: if the host NIC is given the SAME IP as the radar's fixed IP, the radar is effectively hijacked — the host answers for it, Livox SDK finds no device. `ping <that IP>` returns ~0.08 ms with `ttl=64` (host NIC replying for itself). The REAL radar answers with `ttl=255` and 1–3 ms RTT. Diagnosis: ask the user what the radar's IP is / check `ttl` and RTT. Fix: move the HOST to a different address on the same subnet and let the radar keep its IP:
  ```bash
  sudo ip addr del <NET_LAN2>.5/24 dev eth1 && sudo ip addr add <NET_LAN2>.2/24 dev eth1
  ```
- Interpret results: `目标主机不可达` (Dest Unreachable) from your own IP = ARP unanswered → radar unpowered / wrong port / wrong IP; timeout = packets dropped. `ip neigh show` reveals who actually answered.
- **SDK init success ≠ radar connected**: `Init livox lidars succ` only means the SDK started. Look in the driver log (`~/.ros/log/*/livox_lidar_publisher2*.log`, path carries a timestamp; `latest/` may be stale) for device SN / `online` / `connect timeout`.
- **`Params check failed, all livox lidars config is empty` = wrong JSON schema**: Livox driver2 SDK requires `host_net_info` as an OBJECT with `cmd_data_ip`/`push_msg_ip`/`point_data_ip`/`imu_data_ip` keys. A file written with an ARRAY of `{"host_ip": ...}` objects parses as "no config". Compare against a known-good file; the launch file's `user_config_path` param decides which JSON is loaded (`grep user_config_path <launch>`).
- **Multiple config files may disagree on the radar IP** (e.g. `MID360_config.json` says .196, `MID360s_config.json` says .175). User-stated device IP is ground truth — verify by ping TTL before editing.
- Top hardware suspects for unreachable MID360: power (needs 12–24 V), cable/port, IP drift. Subnet sweep to find a live IP: `for i in $(seq 1 254); do ping -c 1 -W 0.2 192.168.2.$i & done; wait`

#### Config-edit hygiene (user preference: 更体面)

User dislikes whole-file heredoc overwrites for config tweaks. Preferred dignified workflow:
1. Backup: `cp cfg.json cfg.json.bak`
2. Structured edit with python3 (only touch target fields, keep formatting valid):
   ```bash
   python3 -c "
   import json
   p='cfg.json'; d=json.load(open(p))
   for k in ['cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip']:
       d['MID360']['host_net_info'][k]='<NET_LAN2>.2'
   d['lidar_configs'][0]['ip']='<NET_LAN2>.5'
   json.dump(d,open(p,'w'),indent=2); print('OK')"
   ```
3. Validate: `python3 -m json.tool cfg.json | grep '"ip"'` (also proves JSON is legal)
Manual `nano` editing is acceptable; heredoc overwrite only as last resort.

#### Pitfalls

- `roslaunch px4.launch` (bare filename) fails with `Invalid roslaunch XML syntax: no element found` — always `roslaunch <package> <file>.launch` after sourcing `devel/setup.bash`.
- **Zeroed launch files**: if several `.launch` files are 0 bytes with the same mtime, they were bulk-overwritten (e.g. XML pasted into a shell hit a `>` redirect). Find all victims: `ls -la <dir>/ | grep " 0 "`. Recovery: `git checkout -- launch/` if the package is a git repo (check `git status` first).
- `Cannot load message class for [livox_ros_driver/CustomMsg]` → package messages not built: `find ~/catkin_ws/devel -name "*CustomMsg*"`; empty result = rebuild (`catkin_make` in catkin_ws).
- Hand-reconstructing wiped files is error-prone (heredoc paste can pick up trailing junk → `junk after document element`). Prefer git restore; manual rebuild only as fallback.
- On Jetson, `dmesg` needs sudo; user in `dialout` group can access serial without sudo.

See `references/fouruav-jetson.md` for the concrete layout of one such machine.


---

### 来源技能 `livox-lidar-diagnostics`

> Diagnose why a Livox LiDAR (MID360 / HAP / Avia) produces no point cloud on a ROS companion computer (Jetson NX, etc.). Covers network-level health checks, SDK broadcast discovery handshake, the dev_type-mismatch silent drop, config format fixes (old object vs new array+host_ip+lidar_ip), and IP/gateway conflicts. Use whenever livox_ros_driver2 initializes ("Init livox lidars succ") but never publishes /livox/lidar.

#### Goal: fast verdict first

Users typically want "is the radar alive" answered quickly, THEN point cloud. Give the hardware verdict first (it's often the quick win), then debug the SDK side. Radar hardware is usually fine — the broken part is host-side config.

#### 1. Confirm radar is alive (network level)

```bash
ping -c 3 <radar_ip>                       # ttl=255 + real ms latency = embedded device, not a PC
ip neigh show dev <iface>                  # which MAC answered
curl -s https://api.macvendors.com/<mac>   # "DJI BAIWANG" (大疆百旺) = Livox confirmed
```
Radar broadcasts device info every second to `255.255.255.255:56201` (430-byte UDP). Listen with a python socket (bind `('',56201)`, no sudo needed) and regex ASCII strings out: you get SN, `DevType:Mid-360`, `FmVer`, `BuildTime` — instant identity proof.

#### 2. SDK discovery handshake

SDK broadcasts 24B on UDP 56000; radar replies 48B within ~1ms. If you see this exchange in tcpdump (`udp port 56000`), the radar is fully functional — the problem is SDK-side device matching. Note: 48B response is only sent AFTER a 24B trigger; the radar's periodic 430B push on 56201 is independent.

#### 3. The silent drop: dev_type mismatch

`device_manager.cpp` drops a discovered device when `type_lidars_cfg_map_.find(detection_data->dev_type) == end()`. Symptom: logs show "Init livox lidars succ", no livox topics ever appear, no error message. This is the #1 cause of "SDK sees radar but never connects".
- Radar firmware may report `dev_type=0x23 (35)` while SDK enum has `kLivoxLidarTypeMid360 = 9` (older SDK vs newer firmware).
- Parse the 48B DetectionData reply to read dev_type: after the 24B SdkPacket header — data[0]=ret_code, data[1]=dev_type, data[2..17]=SN, data[18..21]=lidar_ip, data[22..23]=cmd_port.

#### 4. Fix: switch config to NEW format (array + lidar_ip)

Old object format (`"host_net_info": {"cmd_data_ip": ...}`) keys the device map by dev_type → fails on mismatch. The NEW array format registers the radar into `custom_lidars_cfg_map_` keyed by **IP**, bypassing dev_type matching entirely:
```json
"MID360": {
  "lidar_net_info": { "cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500 },
  "host_net_info": [
    { "host_ip": "<host_ip>", "lidar_ip": ["<radar_ip>"], "cmd_data_port": 56101, "push_msg_port": 56201, "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501 }
  ]
},
"lidar_configs": [ { "ip": "<radar_ip>", "pcl_data_type": 1, "pattern_mode": 0, "extrinsic_parameter": {...} } ]
```
Success signal in driver log: `Lidar ip:<radar_ip> ... unicast is enabled` (was `Device type:9 ...` before). Keep `lidar_configs` for the ROS layer.

#### 5. IP / gateway conflicts

- **Host must NOT hold the radar's own IP.** If radar = <NET_LAN2>.5, host uses a different IP in the same subnet. (Pinging the radar's IP from a host that also owns that IP gives ttl=64/0.08ms — local loopback, NOT the radar. Radar replies show ttl=255.)
- Radar may be configured with a **cross-subnet host + gateway** (e.g. radar <NET_LAN2>.5, gw <NET_LAN2>.1, host <NET_LIDAR>.5). The host NIC must carry BOTH the gateway IP and the host IP:
```bash
sudo ip addr add <NET_LAN2>.1/24 dev eth1
sudo ip addr add <NET_LIDAR>.5/24 dev eth1
```
Radar's configured host IPs are readable from its broadcast TLV keys 0x0006 (point-data host IP:port) / 0x0007 (IMU host IP:port).

#### 6. ROS1 driver ignores broadcast-code args

livox_ros_driver2 **ROS1** build reads only `user_config_path`; `bd_list` / `cmdline_str` args are parsed by the **ROS2** build only (`cmdline_input_bd_code`). Don't burn time passing broadcast codes to ROS1. (Livox broadcast code == device SN, e.g. `ARMCP540030617`.)

#### 7. After the IP-match fix: `Send command failed ... command_id:257`

`kCommandIDLidarGetInternalInfo (0x0101)` failing with `get cmd handler failed` = SDK's `CreateCommandHandler` doesn't know the firmware's dev_type (still 35). SDK is too old for the firmware → upgrade Livox-SDK2, rebuild, reinstall to /usr/local, relink the driver.

#### 7b. Compile-time version mismatch: DoubleEcho symbols not declared (new-machine deployment)

```
pub_handler.cpp:329: error: 'kLivoxLidarDoubleEchoData' was not declared in this scope
pub_handler.cpp:463: error: 'LivoxLidarDoubleEchoRawPoint' was not declared in this scope
```
Root cause: **driver source is NEWER than the installed SDK headers**. New livox_ros_driver2 commits (`Support Avia2 Lidar`, `support Mid-360s`) added double-echo (DoubleEcho) handling in pub_handler.cpp, but the SDK headers installed at `/usr/local/include/livox_lidar_api.h` / `livox_lidar_def.h` are old (no `DoubleEcho` types). This is **NOT** the ROS1/ROS2 `-DROS_EDITION` trap — cmake logs show "Defining ROS1" fine; the missing definitions are in the SDK headers.
Diagnosis:
```bash
grep -c "DoubleEcho" /usr/local/include/livox_lidar_api.h /usr/local/include/livox_lidar_def.h   # 0 = old SDK
grep -rln "kLivoxLidarDoubleEchoData" ~/catkin_ws/src/livox_ros_driver2/                          # only pub_handler.cpp references it
```
Fix (2 options): A. upgrade/reinstall Livox-SDK2 (rebuild + `make install` to /usr/local) to match driver source; B. revert pub_handler.cpp to the pre-DoubleEcho version (ask whether MID-360S needs double-echo). Check `git log --oneline -3` in livox_ros_driver2 — recent Avia2/Mid-360s commits ⇒ new source that needs the matching SDK.

#### Pitfalls

- "SDK init success" ≠ device connected. `Init livox lidars succ` only means config parsed and sockets opened.
- ping-OK ≠ data flow. Ping proves L2/L3 only; SDK still needs dev_type or IP registration to accept the device.
- Multiple IPs on one NIC (e.g. 144.14 / 2.2 / 1.5 / 2.1 on eth1) are normal in this drone setup — don't remove them while debugging.
- sudo over SSH: `echo PASS | sudo -S cmd` (see windows-ssh-automation skill).
- `rostopic hz` errors like `Cannot load message class for [livox_ros_driver/CustomMsg]` are env-var issues (forgot `source devel/setup.bash`) — verify `PYTHONPATH` contains `devel/lib/python3/dist-packages` before suspecting the message build.


---
