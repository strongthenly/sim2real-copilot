# 一键启动链路排障：AirSim + PX4 + twin_bridge + FAST-LIVO2 + MAVROS

实战时间 2026-09-12（`ros1-twin`，Ubuntu 20.04 / ROS Noetic）。按 **现象 → 查什么 → 怎么修** 组织。
（相关：SKILL.md 的「Windows↔WSL 调用与运维坑」与「AirSim 数字孪生 + FAST-LIVO2」两节。）

## 症状 → 根因 速查

| 现象 | 最可能根因 | 去哪节 |
|---|---|---|
| 脚本"卡住"不动、界面无输出 | ① 等待循环不打印进度 ② roscore 冷启动慢 | ③ |
| 桥进程在，`/twin/*` 一直无数据 | PX4↔AirSim 的 TCP4560 断链（LockStep） | ① |
| `roslaunch` 起不来，报 `run_id ... does not match` | 残留 roslaunch 占着 master 的 `/run_id` | ② |
| MAVROS 节点秒退 | 缺 GeographicLib geoid / 14540 被占 | ⑤ |
| `sudo shutdown now` 报 systemd 错 | WSL 没有 systemd，不是故障 | ④ |
| RViz “打不开窗口” / Alt+Tab 切不过去 | WSLg 退化成 COPY MODE 软件合成（**窗口其实存在**） | ⑥ |
| 校验脚本误报“文件为空/被改” | 中文文件名被 git 转义 / CRLF 假象 | 见 `shared-repo-collaboration` |

---

## ① 桥起来但 `/twin` 没数据 → 先查 PX4↔AirSim 的 TCP4560

**机制**：`settings.json` 里 `ClockType:"SteppableClock"` + `LockStep:true` → **仿真时间靠 PX4 步进**，
AirSim **必须等 PX4 连上 TCP4560 才开始跑**。链路一断，AirSim 就停在等待行，**传感器一个数据都不出**
（桥会在那儿空转，看起来"启动了但收不到"）。

**Windows 侧直接证据** —— 读 `E:\AirSim\Unreal\Environments\Blocks\Saved\Logs\Blocks.log`：
```
LogTemp: Connected to SITL over TCP.            ← 正常：PX4 连上过
LogTemp: received first heartbeat
LogTemp: Got GPS lock
LogTemp: Got GPS Home Location
[06:40:14] LogTemp: Exception sending messages to vehicle
           MavLinkConnectionImpl: Error sending message on connection 'hil',
           details: TcpClientPort socket send failed with error: 10053   ← 10053=WSAECONNABORTED，对端断了
[06:40:14] LogTemp: Disconnecting mavlink vehicle
[06:40:14] LogTemp: Waiting for TCP connection on port 4560, local IP 127.0.0.1   ← 回落成等待
```

**WSL 侧判据**：`ss -tn | grep ':4560'` —— **有 established 才算连上**（只是"有监听"不算）；
没有就说明 Blocks 没在 Play，或 PX4 没起来。

**顺序要求（一键脚本必须这样）**：
1. 确认 **Blocks 处于 Play**（AirSim 已 `Waiting for TCP connection on port 4560`）
2. 起 PX4（tmux 保 tty），**轮询等 `ss -tn | grep ':4560'`** 出现
3. **再**起桥，**轮询等 `/twin` 出数据**
> **桥从启动到首帧数据约需 30s**（等 LiDAR 初始化 + 仿真步进）——别等 10s 就判失败。
> AirSim 会**自动等待重连**（回到等待行后，PX4 再连上即恢复），所以修 PX4 那一侧就够了，**不用重启 UE**。

**⚠️ 不要无条件杀正在跑的 PX4**：早期版本脚本 `[1/7]` 里写了 `tmux kill-session -t px4`，
于是每次启动都重启 PX4 → ① 要多等几十秒 ② 容易把已建立的 4560 链路打断 → 桥必然"没数据"。
判据用 `pgrep -f "bin/px4"`，**已在跑就复用**。

---

## ② `RLException: run_id on parameter server does not match declared run_id`

```
RLException: run_id on parameter server does not match declared run_id:
  91926ee2-ae75-11f1-857c-9c2dcd120631 vs 949f8ef8-ae75-11f1-bdf7-9c2dcd120631
The traceback for the exception was written to the log file
```
**根因**：**有残留的 roslaunch 还占着 master 的 `/run_id`**（最常见是以前跑过的
`roslaunch mavros px4.launch ...` 没退干净）。只清理"目标那一个" roslaunch 不够。

**修**（启动前清理**所有** roslaunch，而不是只清目标）：
```bash
pkill -f fastlivo_mapping; pkill -x rviz; pkill -f bridge.py
pkill -f roslaunch          # ← 关键：全部 roslaunch
pkill -f rosout; pkill -f rosmaster; pkill -f roscore
for i in $(seq 1 10); do
  pgrep -f "rosmaster|roslaunch" >/dev/null 2>&1 || break
  sleep 1
done
echo "残留: $(pgrep -f 'rosmaster|roslaunch' | wc -l) 个"   # 期望 0
```
> ⚠️ `pkill -f roslaunch` 里的模式**不能**是脚本自己文件名/内容会命中的词——否则自杀（见 SKILL.md 的 pkill 自杀坑）。

---

## ③ `[3/7] 启动 roscore ...` 静默 2 分钟 ≠ 卡死

**实测现象**：roscore 进程已起了 2 分 21 秒，但其子进程 `rosmaster` 才跑了 6 秒；
而此时脚本**其实已经往下跑了**（子进程里已有 `roslaunch twin_bridge` + `fastlivo_mapping`）。
用户看到的是"停在 [3/7] 不动"，实际是**等待循环不打印进度 + roscore 冷启动极慢**。

**先把 DNS 排除掉**（本机实测，不是元凶）：
```
socket.getfqdn()          → LY.localdomain   0.028s
socket.gethostbyaddr()    → localhost        0.033s
/etc/hosts 有 127.0.1.1 LY.localdomain LY ；/etc/nsswitch.conf: hosts: files dns
```

**真因（2026-09-12 二次实测，定论）：`roscore` 这个包装脚本本身慢，能绕开！**

逐个计时把元凶钉死：
```
roscore   → 140 秒   ✗ 唯一瓶颈
rosmaster --core -p 11311 → 2 秒   ✓   （roscore 内部真正干活的就是它）
rosout    → 0 秒                    ✓
ROS_IP=127.0.0.1 也设了 → 无效（不是主机名/多网卡解析）
getfqdn / gethostbyaddr → 几十 ms    （DNS 早已排除）
```
CPU 被同脚本 `make px4_sitl` 抢只是**次因**（会让它更慢），**不是主因** —— 单独跑 roscore 照样 140s。

**根治：绕开 `roscore`，直接起 `rosmaster` + `rosout`**（★ 本会话最大提速，140s → 6s）：
```bash
source /opt/ros/noetic/setup.bash
setsid rosmaster --core -p 11311 > /tmp/rosmaster.log 2>&1 < /dev/null &
for i in $(seq 1 30); do timeout 2 rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosparam set /rosdistro noetic 2>/dev/null      # roscore 原本会设这两个参数
rosparam set /rosversion 1.17.4 2>/dev/null
setsid rosrun rosout rosout __name:=rosout > /tmp/rosout.log 2>&1 < /dev/null &
sleep 2
```
**实测**：master **4 秒**就绪 → `roslaunch twin_bridge` 完全正常（**桥 1 秒就出数据**）→
**全量启动（master+桥+FAST-LIVO2+RViz）6 秒**；紧接着再跑一次 **0 秒**。
> `rosmaster` 直接起时 `/run_id` 未设，`roslaunch` 会自己设 —— 所以**仍需遵守 §② 的"先清干净残留 roslaunch"**。

**次优修法（不想改启动方式时）**：
1. **master 还在就复用**，**不要无脑重启 roscore**：
   ```bash
   if timeout 6 rosnode list >/dev/null 2>&1; then
     echo "✅ master 已在跑，直接复用（省 2 分钟）"
   else
     setsid roscore > /tmp/roscore.log 2>&1 < /dev/null &
     ...轮询等待...
   fi
   ```
2. 等待循环**必须打印进度**：`[ $((i % 10)) -eq 0 ] && echo "   ... 仍在等 master（${i}0s）"`
   （否则用户看到的是"假死"，会来报"卡住了"）
3. 想更彻底避开慢启动：**先起 ROS master 再起 PX4**（避免 `make` 抢 CPU）。

**判"到底卡没卡"**（别只看屏幕）：
```bash
pgrep -af "twin_livo_rviz"                       # 脚本还在不在
ps -o pid,stat,etime,cmd -p <roscore_pid>        # roscore 跑了多久
ps --ppid <脚本pid> -o pid,stat,etime,cmd        # 子进程走到哪一步（最有用）
```
子进程里已出现 `roslaunch twin_bridge` / `fastlivo_mapping` = **那个"卡住"的步骤早就过了**。

**另附**：roscore 起来后 /tmp/roscore.log 可能是 0 字节（roscore 自己的输出很少），
**别拿"日志为空"当"启动失败"判据**——判据是 `rosnode list` 能否成功。

---

## ④ WSL 里 `sudo shutdown now` 一定失败

```
System has not been booted with systemd as init system (PID 1). Can't operate.
Failed to connect to bus: Host is down
Failed to talk to init daemon.
```
= **WSL 没有 systemd**（PID 1 不是 init），`shutdown`/`systemctl` 都不可用，**不是故障**。

**正确关法（在 Windows PowerShell 里）**：
```powershell
wsl --terminate ros1-twin      # 只关一个发行版
wsl --shutdown                 # 关掉所有 WSL 发行版（连带释放 vmmem）
```
**收尾顺序**：先 `bash ~/twin_livo_stop.sh` 停 ROS 侧 → 再 `wsl --terminate <发行版>` →
最后在 Windows 手动关 AirSim(Blocks) / QGC。

---

## ⑤ MAVROS 节点秒退（两个 FATAL，速查；细节见 SKILL.md 定点验证节）

| FATAL | 修 |
|---|---|
| `UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm` | 装 geoid 数据：`sudo geographiclib-get-geoids egm96-5`；**同机其它发行版已有就直接拷**（`lyuav` → `ros1-twin`，经 `/mnt/c` 中转 `egm96-5.pgm`+`.aux.xml`+`.wld`） |
| `FCU: DeviceError:udp:bind: Address already in use` + `Bind address: 0.0.0.0:14540` | 14540 被占 → **给 PX4 开专用 MAVLink 专线**：`tmux send-keys -t px4 "mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard" Enter`，再 `roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610` |

`rostopic echo -n 1 /mavros/state` 报 `topic does not appear to be published yet` =
**MAVROS 根本没启动**（不是"连不上"）——先确认它起来了再查连接。

---

## ⑥ "RViz 打不开窗口" / Alt+Tab 切不过去 → WSLg 合成坏了（窗口其实存在）

**用户原话**："rviz 启动后提示 warning，打不开可视化窗口" / "后台有两个 rviz 界面，但按下 alt+tab 完全打不开界面"。

**别先怀疑 RViz 或配置** —— 先取证：

1. **RViz 进程在不在 + GL 有没有初始化成功**（看 ROS 日志，不在 `/tmp/rviz.log`）：
   ```bash
   LATEST=$(ls -dt ~/.ros/log/*/ | head -1); grep -iE "rviz|OpenGL" "$LATEST/rosout.log" | tail -5
   ```
   本机实测（说明 RViz 一切正常，只是窗口没被画出来）：
   ```
   INFO /rviz_...: Stereo is NOT SUPPORTED
   INFO /rviz_...: OpenGL device: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)
   INFO /rviz_...: OpenGl version: 3.1 (GLSL 1.4)
   ```
   > ⚠️ RViz(Qt) 的 stdout/stderr **常常是空的**（`>/tmp/rviz.log` 0 字节不代表失败）；它的日志走 **rosout**。
2. **枚举 WSLg 合成器 `msrdc` 的顶层窗口**证明窗口到底存不存在 —— 用 `scripts/enum_wslg_windows.ps1`
   （Win32 `EnumWindows`+`GetWindowRect`+`IsIconic`；`Get-Process | MainWindowTitle` 查不到 WSLg 窗口，
   因为所有 WSLg 应用的窗口都挂在 **msrdc** 这一个进程上）。本机实测输出：
   ```
   Primary screen: 1707x960
   msrdc pid=13916 min=False rect=(-21,-21)-(1688,961) size=1709x982
         title=[WARN:COPY MODE] fast_livo2.rviz - RViz (ros1-twin)      ← RViz 窗口在，全屏、未最小化
   msrdc pid=13916 min=False rect=(659,371)-(1043,557) size=384x186
         title=[WARN:COPY MODE] ROS 1 End-of-Life is May 31st, 2025      ← 用户说的\"warning\"其实是它
   ```
   → 结论：窗口**存在且可见**，只是被 z-order 压在 Hermes/UE4 下面 + WSLg 合成异常。

**判读三条**：
- WSLg 窗口标题前缀 **`[WARN:COPY MODE]`** = WSLg **没走 GPU 合成、退化成 CPU 拷贝**（渲染慢/花屏/切不过去）。
  正常应为无此前缀。
- `ROS 1 End-of-Life is May 31st, 2025` 是 **ROS Noetic 停服提示框**（本身也是个 WSLg 窗口），
  **与 RViz 无关**，用户极易当成"RViz 报错"——讲清这条能省一轮排查。
- 先让用户 **Alt+Tab / 最小化 Hermes·UE4** 确认窗口本体在不在；"在但画不出"才去查 WSLg。

**可疑根因 & 修**：本机 `.wslconfig` 指定了**自定义内核**（`kernel=C:/ProgramData/NVIDIA Corporation/SDKs/JetPack_6.2.2_Linux/vmlinux`，
即 Jetson JetPack 内核），可能缺 WSLg 的 GPU 合成支持 → COPY MODE。
```powershell
wsl --shutdown                       # ① 先让 WSLg 重新初始化（最常用、最简单）
# 仍 COPY MODE → ② 注释掉 .wslconfig 里的 kernel= 行，用 WSL 默认内核，再 wsl --shutdown
```
诊断辅助（Windows 侧）：`Get-Process | ? { $_.Name -match "wsl|msrdc|wayland|weston|Xway" }` ——
**`msrdc.exe /wslg /silent /v:... "C:\\Program Files\\WSL\\wslg.rdp"` 在跑 = WSLg 管线是活的**；
`/mnt/wslg`、`/tmp/.X11-unix/X0`、`DISPLAY=:0` 存在也都是正常态，**别拿它们当故障证据**。

> ⚠️ **写 Windows 侧的 `.ps1` 诊断脚本不要夹中文** —— PowerShell 5.1 对 UTF-8 无 BOM 文件按 ANSI 读，
> 中文被拆坏后报 `The string is missing the terminator: ".` / `Missing closing '}'`。
> 本会话连踩两次，**改全英文立刻通过**。中文只放在 `.sh`（WSL 侧）里。

---


## 判"全绿"的标准输出（一键脚本结尾应能看到）

```
[1/7] 残留检查: 0 个
[2/7] ✅ PX4 已在跑（不重启） / -> 已启动 PX4 (tmux: px4)
[3/7] ✅ master 已在跑，直接复用   或   ✅ master 就绪（第 N 次探测）
[4/7] ✅ 4560 已连接（第 3s）
[5/7] ✅ /twin 有数据（第 3~33s）
[7/7] /laserMapping + /twin_sensor_bridge + /rviz
      /aft_mapped_to_init  ~9-10 Hz
```
`/aft_mapped_to_init` 有频率 = **整条链路（传感器→定位）通了**。
