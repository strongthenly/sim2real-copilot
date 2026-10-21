# ROS1(Noetic) 侧数字孪生链路：启动时序 / MAVROS offboard / FOV 盲区

场景：Windows 跑 **AirSim(UE4.27 Blocks)**，WSL 发行版 `ros1-twin`(Ubuntu 20.04 + ROS Noetic) 跑
**PX4 v1.11.3 + twin_bridge + FAST-LIVO2 + MAVROS + RViz**。完整链路 2026-09 实测跑通并推送
`github.com/<GITHUB_USER>/<PRIVATE_ROS1_WS_REPO>`（twin_bridge / fastlivo_config / scripts / control / tools / docs）。

数据流：`AirSim 传感器 → twin_bridge(/twin/*) → FAST-LIVO2(/aft_mapped_to_init + /cloud_registered) → 规划器 → MAVROS(offboard) → PX4 → 无人机`

---

## 1. 启动时序（最容易误判成"坏了"的地方）

AirSim settings 用 **`PX4Multirotor + LockStep + SteppableClock`** ⇒ **必须先有 PX4 连上 TCP 4560，仿真才开始跑**。
链路一断，AirSim 日志回到 `Waiting for TCP connection on port 4560`，此时**传感器零数据**（桥自然也没数据）。

一键脚本的正确顺序与等待判据：

| 步 | 动作 | 判据（轮询，别死等固定秒数） |
|---|---|---|
| 1 | 清理 ROS 侧（**不动 PX4 进程**） | `pgrep -f 'rosmaster\|roslaunch'` 归零 |
| 2 | PX4 已在跑就复用，别无条件重启 | `pgrep -f "bin/px4"` |
| 3 | 起 roscore 并等 master 真就绪 | `timeout 5 rosnode list` 成功 |
| 4 | **等 PX4↔AirSim 建链** | `ss -tn \| grep ':4560'` |
| 5 | 启桥 → 等 /twin 出数据 | `rostopic echo -n1 /twin/lidar/points_preview/header` |
| 6 | FAST-LIVO2 + RViz | `rostopic hz /aft_mapped_to_init` |

实测时间：4560 建链 ~3s；**桥从启动到出数据 ~30s**（LiDAR 初始化 + 仿真步进），不要 12s 就判失败。

**前提**：启动前 Windows 侧 **Blocks 必须处于 Play**。收工脚本另存（清理 fastlivo/rviz/桥/PX4/roscore）。

### 1.1 长等待循环必须打进度；且不要无脑重启 roscore
- **roscore 在这类机器上可能要 2 分钟以上才就绪**（实测 2m21s；已排除 DNS——`getfqdn` 0.028s、
  反向解析 0.033s 都很快，主因是**和 PX4 的 `make` 抢 CPU**）。若等待循环**不打印任何进度**，
  界面静默两分钟，用户会以为"卡死了"⇒ **每个长等待循环都按周期输出一次**（如每 10 次探测打印
  `... 仍在等 master（100s）`）。
- **能复用就别重启 master**：清理阶段只杀 `roslaunch` + 节点进程，**保留 `rosmaster`/`roscore`**；
  脚本开头先 `timeout 6 rosnode list` 探测，成功就直接复用（省掉那 2 分钟）。run_id 冲突的成因是残留
  **roslaunch**、不是 roscore 本身，所以不必杀 roscore。

## 2. roslaunch 报 run_id 冲突

```
RLException: run_id on parameter server does not match declared run_id: 91926ee2-... vs 949f8ef8-...
```
= 有**残留 roslaunch** 占着 master 的 `/run_id`（例如之前跑过 `roslaunch mavros` 没退干净）。
**修法**：启动前清理**所有** roslaunch，而不是只清理目标那一个：

```bash
pkill -f roslaunch; pkill -f rosout; pkill -f rosmaster; pkill -f roscore
for i in $(seq 1 10); do pgrep -f "rosmaster|roslaunch" >/dev/null || break; sleep 1; done
# 再起干净的 roscore，并等 rosnode list 成功
```

## 3. MAVROS 接入（ROS1 + AirSim 栈的 PX4）

两个必过门槛（缺一个就连不上）：

1. **GeographicLib geoid 数据** —— 缺则 `[FATAL] UAS: GeographicLib exception: File not readable
   /usr/share/GeographicLib/geoids/egm96-5.pgm` 秒退。装法：`apt install geographiclib-tools` + 跑
   `install_geographiclib_datasets.sh`；**更快的路子是从另一个已装好的发行版直接拷**
   （文件 + `.pgm.aux.xml` + `.wld` 三个，`/usr/share/GeographicLib/geoids/`，用 C 盘中转）。
2. **专用 MAVLink 端口** —— 默认 onboard 14540 被占，MAVROS 报
   `FCU: DeviceError:udp:bind: Address already in use`。在 PX4 控制台另开一条：

```
pxh> mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard
```
```bash
roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610
```

判据：`rostopic echo -n1 /mavros/state` → `connected: True`。
（ROS2/Gazebo 栈上的同类做法见 SKILL.md 里的 14600/14601 条目。）

## 4. offboard 控制脚本骨架（ROS1 MAVROS）

位置 setpoint 走 `/mavros/setpoint_position/local`（PoseStamped）。必备顺序：

```python
# 1) 预发 setpoint ≥2s（PX4 切 OFFBOARD 的前置条件）
for _ in range(60): pub.publish(sp); rate.sleep()
# 2) 切模式 + 解锁
rospy.wait_for_service('/mavros/set_mode'); rospy.wait_for_service('/mavros/cmd/arming')
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)      # custom_mode='OFFBOARD'
arming   = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) # arming(True)
# 3) 20Hz 持续发 setpoint（位置控制）
```

- **转弯 = 改 setpoint 的四元数 yaw**（位置不变 → PX4 原地转向）：
  `qz=sin(yaw/2), qw=cos(yaw/2)`，平滑插值 `yaw0→yaw1`。
- 多段机动（前进/右转/再前进）就是"沿 yaw 方向递增位置 + 转向段"拼接，实测 40m 直线高度/航向稳定、
  前进10m→右转90°→前进20m 两段严格正交。
- **判读定点质量要用"稳态段"**：把起飞爬升（z:0→5）算进漂移会得到 5m 的假告警；只统计到达目标后的样本
  （如 `z > target-0.5`），实测稳态漂移 <5cm。
- ⚠️ **脚本停止发 setpoint 后**，PX4 offboard 超时 → 依 `COM_OBL_ACT` 执行 **AUTO.LAND**（脚本一结束无人机就降落，
  这是安全行为不是故障）。要它留在空中悬停，得让脚本持续运行或显式切 POSCTL/AUTO.LOITER。
- 想验证"用 LIVO 定位"的定点，才需要把 `/mavros/vision_pose/pose` 接给 PX4 EKF；否则用的是 PX4/AirSim 自身位置。

## 5. LiDAR FOV 盲区几何（"相机看得到、点云没有"的真相）

**症状**：相机（FOV 大）前方明显有障碍，LiDAR 点云里却没有 → 多半**不是 bug，是 FOV 几何盲区**。

MID360 真实垂直 FOV 是 **`-7° ~ +52°`**（向下只有 7°）。飞高 h 时，**地面/低处障碍的盲区半径 ≈ `h / tan(7°)`**：

| 飞行高度 | 盲区半径 | 实测 |
|---|---|---|
| 3 m | ≈ 24 m | 点云最近点 **23.09 m**（完全吻合） |
| 0.4 m | ≈ 3 m | 落地后最近点降到 1.84 m |

⇒ **移动无人机（前进 2m/40m）不改变盲区**（盲区只由高度和 FOV 决定）。

出路（按保真度取舍）：① 飞低；② 放宽 AirSim `settings.json` 的 `VerticalFOVLower`（如 -7 → -25，**偏离实机**，
适合跑探索时用）；③ 给 LiDAR 加 `Pitch` 倾角。**改 settings.json 必须重启 Blocks 生效。**
建议保留两份配置（实机复刻版 / 仿真探索版）并说明差异，免得论文的"高保真"说法被自己打破。

## 6. 交付物清单（一键脚本 + 控制脚本 + 工具）

- `twin_livo_rviz.sh`（启动：PX4+桥+FAST-LIVO2+RViz，**v3 起含完整时序等待**）/ `twin_livo_stop.sh`（收工）
- 控制：`hover_verify.py`（定点+稳态漂移）、`move_forward.py`、`move_forward_cont.py`、`maneuver.py`（多段机动）
- 工具：`cloud_analyze.py`（点云统计：点数/xyz 范围/**俯仰角分布**/水平距离——判 FOV 盲区就用它）
- 配置：`fastlivo_config/TwinSim*.yaml`（仿真 `/twin/*` + lidar_type 4）、`TwinReal*.yaml`（实机 bag）
- 写脚本的自我检查：**文件名 + 内容里出现的每个词，会不会被自己的 `pkill -f` 命中**（坑详见 `fast-livo2-mapping`）。
  本会话就因脚本名含 `rviz` 被 `pkill -f rviz` 自杀过一次。

## 7. 成果入库（Windows → GitHub）

- 仓库 `github.com/<GITHUB_USER>/<PRIVATE_ROS1_WS_REPO>`：只放**自己的产物**（桥/配置/脚本/工具/文档），**不推第三方源码**
  （FAST-LIVO2 / livox_ros_driver / vikit 等）；按功能分子目录（`twin_bridge/` `fastlivo_config/` `scripts/`
  `control/` `tools/` `docs/`），README 里写清每个文件对应工作区里的哪个路径。
- **推送走代理**：国内直连会 `Failed to connect to github.com:443 after 21104 ms` →
  `git -c http.proxy=http://127.0.0.1:7899 -c https.proxy=http://127.0.0.1:7899 push origin main`。
  （WSL 侧的 git 没有 Windows 的凭据助手 → `git ls-remote` 会卡住；校验远程要么在 Windows 侧跑，
  要么先 `export GIT_TERMINAL_PROMPT=0` 让它快速失败而不是挂住。）
- **含 Linux 脚本的仓库必须加 `.gitattributes`**（`* text=auto eol=lf`）：否则 Windows `core.autocrlf=true`
  会让 `git status` 反复报 phantom `M`，且带 CRLF 的 `.sh` 在 WSL 里执行会出错。
- ad-hoc 校验（这类仓库没有测试套件）：对已提交文件跑 `ast.parse`(py) / `bash -n`(sh) / `yaml.safe_load` /
  XML parse + **幂等性**（重跑收集脚本后 `git status --porcelain` 应为空）+ 用 `git ls-files -z` 遍历
  （直接 `git ls-files` 会把中文文件名转义成 `\345\...`，导致"空文件"假告警）。

---

## 8. FAST-LIVO2 接入细节（两套数据源 + 三个配置坑）

两条数据源各配一份（本会话产物 `fastlivo_config/TwinSim*.yaml` / `TwinReal*.yaml`）：

| 数据源 | 话题 | `lidar_type` | 关键要求 |
|---|---|---|---|
| 仿真桥（AirSim） | `/twin/lidar/points_preview`(PointCloud2) | **4**（L515 分支） | 点云必须带 `rgb` 字段 |
| 实机 bag / 实机 | `/livox/lidar`(CustomMsg) | **1**（Avia） | 需要逐点时间戳 |

### 8.1 相机配置即使 `img_en: 0` 也必须加载
只 load 算法配置会让 FAST-LIVO2 秒崩：
```
[ERROR] Cannot find value for parameter: laserMapping/cam_model, will try again.
terminate called after throwing an instance of 'std::runtime_error'
  what():  Camera model not correctly specified.
```
⇒ 相机配置要**单独 load 到节点命名空间**：
```bash
rosparam load fastlivo_config/TwinSim.yaml
rosparam load fastlivo_config/TwinSim_camera.yaml /laserMapping   # cam_model/fx/fy/cx/cy/d0..d3
rosrun fast_livo fastlivo_mapping __name:=laserMapping
```

### 8.2 点云必须有 `rgb` 字段（lidar_type 4 走 L515 分支）
`Preprocess::l515_handler` 用 `pcl::fromROSMsg(*msg, pcl::PointCloud<pcl::PointXYZRGB>)`
⇒ **XYZ-only 点云会失败**；其它 PointCloud2 分支（velodyne/oust64/robosense）都要 `intensity`，同样喂不了纯 XYZ。
⇒ 修法：**改桥**，把 `create_cloud_xyz32` 换成带 `x,y,z,rgb` 四字段的 `create_cloud`（rgb 填 0.0）：
```python
fields = [PointField('x',0,FLOAT32,1), PointField('y',4,FLOAT32,1),
          PointField('z',8,FLOAT32,1), PointField('rgb',12,FLOAT32,1)]
pts = [[float(q[0]), float(q[1]), float(q[2]), 0.0] for q in p]
```
**改完必须重启桥**，否则 FAST-LIVO2 仍报缺 `rgb`。

### 8.3 实机 bag 的 Livox 消息类型：md5 相同就能"借壳"
实机 bag 里是 **`livox_ros_driver2/CustomMsg`**，twin_ws 装的是 `livox_ros_driver`(v1) →
`rosbag play` 报 unknown message type。但两者 **md5 完全相同**
（`e4d6829bdfe657cb6c21a746c86b21a6`）⇒ **建一个"只含 msg 定义"的最小 `livox_ros_driver2` 包**
（`package.xml` + `CMakeLists.txt`：`add_message_files(DIRECTORY msg FILES CustomMsg.msg CustomPoint.msg)`
+ 从 v1 拷 `.msg`），`catkin_make --pkg livox_ros_driver2` 后 `rosmsg md5 livox_ros_driver2/CustomMsg`
应与 bag 一致，`rosbag play` 即可直接回放（ROS1 连接判据是 md5 匹配，v2 发布者能被 v1 订阅者接收）。
⚠️ `package.xml` 的 `<maintainer email=...>` 必须是**合法邮箱**（`twin@local` 会被 catkin 拒绝）。

### 8.4 坐标系约定（自洽 ≠ bug）
FAST-LIVO2 的 `camera_init` 是**雷达 FLU** 系，MAVROS/PX4 是 **ENU** ⇒ 差一个标准旋转。
实测无人机沿 ENU 的 **+x（东）** 飞 40m，FAST-LIVO2 地图沿 **-y** 延伸 —— **自洽，不是错**。
接规划器时要么统一到一边，要么在接口处做该旋转。

---

## 9. WSLg 图形窗口排查（"RViz 打不开窗口"）

**症状**：`rviz` 进程在跑、rosout 里有 `OpenGL device: D3D12 (NVIDIA ...)` / `OpenGl version: 3.x`
（GL 初始化成功），但用户说"提示 warning、窗口打不开"。

**排查方法（实测有效）**：WSLg 把**所有** Linux 应用窗口都渲染在 **`msrdc.exe`** 这一个 Windows 进程里
⇒ `Get-Process | Where MainWindowTitle` **找不到 rviz 窗口**（标题挂在 msrdc 上）。
要用 Win32 `EnumWindows` 枚举 **msrdc 的顶层窗口**（脚本：`scripts/wslg-window-dump.ps1`）：
```
proc=msrdc pid=13916 min=False rect=(-21,-21)-(1688,961) size=1709x982
     title=[WARN:COPY MODE] fast_livo2.rviz - RViz (ros1-twin)      ← 窗口其实开着，还全屏
proc=msrdc pid=13916 min=False rect=(659,371)-(1043,557) size=384x186
     title=[WARN:COPY MODE] ROS 1 End-of-Life is May 31st, 2025     ← 用户说的那个 "warning"
```
两个症状当场对上：
1. **"提示 warning"** = **ROS 1 End-of-Life 弹框**（Noetic 停服 2025-05-31 后 roscore 会弹），
   与 RViz 无关，点掉即可。
2. **"窗口打不开"** = 窗口其实**全屏且未最小化**，只是被别的窗口（Hermes/UE4）盖住
   ⇒ 让用户 `Alt+Tab` 或先最小化其它窗口确认。

**`[WARN:COPY MODE]` 前缀 = WSLg 退化到软件合成**（没用 GPU 合成）→ 渲染慢/黑屏/花屏。
常见成因：`~/.wslconfig` 指定了**自定义内核**（本机是
`kernel=C:/ProgramData/NVIDIA Corporation/SDKs/JetPack_6.2.2_Linux/vmlinux`，Jetson 的内核，
缺 WSLg 的 GPU 合成支持）⇒ **注释掉 `kernel=` 行 → `wsl --shutdown` → 重启**通常即恢复。
（`msrdc.exe /wslg /silent ... "C:\Program Files\WSL\wslg.rdp"` 在跑 = WSLg 合成器本身是活的。）

**写 `.ps1` 的坑**：PowerShell 5.1 按 ANSI 读脚本文件，`write_file` 写出的 **UTF-8(无 BOM) 中文会乱码并破坏引号**
⇒ 报 `The string is missing the terminator: "` / `Missing closing '}'`。
**给 PowerShell 5.1 的 `.ps1` 一律只写英文。**
