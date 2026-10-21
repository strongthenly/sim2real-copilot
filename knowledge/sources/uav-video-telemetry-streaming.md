---
name: uav-video-telemetry-streaming
description: 在无人机机载电脑（Jetson NX/Orin）上搭建/诊断「网线透传」——图传（RTSP，相机或屏幕画面）与数传（MAVLink→UDP 14550，QGC）。覆盖 D435/USB/CSI 相机到 RTSP 的完整 GStreamer 管线（含 Jetson 硬件编码）、GstRtspServer Python 绑定的 6 个致命坑、遥控器"等待视频信号接入"的私有协议 vs 标准 RTSP 判别、mavlink-router 配置。触发词：图传、数传、RTSP、rtsp://、14550、QGC 连不上、遥控器看不到画面、等待视频信号接入、mediamtx、推流、透传。
---

# UAV 图传 + 数传 网线透传

## When to use
用户要在机载电脑（Jetson NX/Orin + Ubuntu 20.04）上实现或排查：
- **图传**：相机/屏幕画面 → `rtsp://<ip>:8554/live`，电脑或遥控器拉流
- **数传**：飞控 MAVLink → UDP 14550，QGC 连上
- 遥控器显示「等待视频信号接入」、QGC 无心跳、网线直连不通

## 先分清三种「图传」，别答错方向

| 形态 | 端口/协议 | 客户端要求 | 判别方法 |
|---|---|---|---|
| **标准 RTSP** | `tcp:8554` + `rtsp://` | 任意播放器（VLC/ffplay/ffprobe） | `ffprobe -rtsp_transport tcp -i rtsp://...` 能出 SDP |
| **私有推流协议**（AirCore/Liyumen 系） | `tcp:6000` 帧=`<4字节LE长度><JPEG>` | 必须厂家的 App | `curl` 报 `Received HTTP/0.9 when not allowed`；`ffprobe` 直接失败 |
| **MJPEG over HTTP** | `tcp:8000`、`multipart/x-mixed-replace` | 浏览器/支持 MJPEG 的客户端 | 浏览器直接打开能看 |

**关键判别命令**（连上但拿不到画面的第一步）：
```bash
printf 'OPTIONS rtsp://127.0.0.1:8554/live RTSP/1.0\r\nCSeq: 1\r\n\r\n' | timeout 5 nc 127.0.0.1 8554
# 200 OK + Server: GStreamer RTSP server  → 是真 RTSP
# 无响应 / 返回 AUTH:xxx 裸字符串        → 是私有协议
```

## 图传：D435 → RTSP 完整可跑管线（Jetson 实测）

**用户机器上通常已有全部依赖**：`libgstrtspserver-1.0` + `gir1.2-gst-rtsp-server-1.0` 已装，
`test-launch` 测试程序没装也不需要 —— **用 Python 自写服务，零额外安装**。
（`apt install gstreamer1.0-rtsp` 常被 `ros-noetic-cv-bridge → libopencv-dev` 断链卡住，**不要去修那个依赖**。）

管线（640x480@15fps 硬件编码，实测 CPU 占用极低）：
```
v4l2src device=/dev/video4 !
  video/x-raw,format=YUY2,width=640,height=480,framerate=15/1 !
  videoconvert ! video/x-raw,format=I420 !
  nvvidconv ! video/x-raw(memory:NVMM),format=I420 !
  nvv4l2h264enc bitrate=1200000 iframeinterval=30 insert-sps-pps=true !
  rtph264pay name=pay0 pt=96 config-interval=1
```
软件编码备选（无 NVENC 时）：把后三级换成 `x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 key-int-max=30`。

可直接部署的服务模板见 `scripts/d435_rtsp_server.py`（systemd user service 托管，
文件头注释写全了下面 6 个坑）。`v4l2-ctl --list-devices` 确认哪个 `/dev/videoN` 是 RGB
（D435 常是 video4；video0=深度 Z16、video2=红外）。

### 六个致命坑（每一个都曾让 DESCRIBE 返回 400）

1. **`format=YUYV` 是错的，必须写 `format=YUY2`。**
   D435 经 `uvcvideo` 报的 caps 是 `YUY2`。写 `YUYV` → `无法将 v4l2src0 链接到 videoconvert0`，
   在 RTSP 场景下表现为 **`DESCRIBE` 返回 400 Bad Request，看不到任何真实原因**。
   验证真实 caps：`gst-launch-1.0 -v v4l2src device=/dev/video4 num-buffers=10 ! fakesink`
2. **必须用 `GstRtspServer.RTSPServer.new()`，不要用 `RTSPServer()`。**
   后者在本环境能 `set_service`/`attach` 成功、端口也 LISTEN，但 DESCRIBE 恒 400。
3. **`set_launch()` 的管线字符串不能带外层括号 `( ... )`。** 只给裸描述。
4. **`rtph264pay` 必须命名为 `pay0`**（`name=pay0`），否则 factory 找不到 payloader。
5. **`nvvidconv` 不接受 `YUY2`/`RGB`**，必须先 `videoconvert` 到 `I420` 再进 `nvvidconv`。
6. **`v4l2src` 不要加 `io-mode=2`** —— D435 的 UVC 驱动会拒，导致连接失败。

### 验证链（三步都要过，缺一不可）
```bash
# 1) 管线本身能不能建（最快的定位手段）
gst-launch-1.0 -q <PIPELINE> num-buffers=20 ! fakesink     # 看是否打印 NvVideo: NVENC
# 2) RTSP SDP
ffprobe -hide_banner -rtsp_transport tcp -i rtsp://127.0.0.1:8554/live
#    期望：Video: h264 (Constrained Baseline), yuv420p, 640x480, 14.99 fps
# 3) 真解码（端口监听 ≠ 能出图）
ffmpeg -hide_banner -loglevel error -rtsp_transport tcp -i rtsp://127.0.0.1:8554/live -t 5 -f null -
```

## 数传：MAVLink → UDP 14550

先确认飞控**物理上接到哪**，否则配了也没数据：
```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttyTHS* 2>/dev/null
ps aux | grep -Ei 'mavros|mavlink|px4' | grep -v grep
```
- 无 `/dev/ttyACM*` → 飞控 USB 没插（最常见）
- 有 `/dev/ttyTHS*` 但无进程占用 → 可能走 UART 但驱动没起，用 `stty` 定波特率后 `cat` 验证（见 `uav-ros-sensor-diagnostics`）
- 串口读取要**显式设波特率**，默认 115200 读不到 921600 的数据

`mavlink-router` 配置（`/etc/mavlink-router/main.conf`）：
```ini
[General]
TcpServerPort=0
ReportStats=false
MavlinkDialect=common

[UdpEndpoint qgc]
Mode = Normal
Address = <NET_RC>.100   # 地面电脑 IP
Port = 14550

[UartEndpoint fcu]
Device = /dev/ttyACM0
Baud = 57600
```
QGC 侧：`Application Settings → Comm Links → Add → Type: UDP → Listening Port: 14550`。

## 遥控器「等待视频信号接入」的定位树

看到这句提示，按顺序排：

1. **目标 IP 是不是本机？** `ip -4 addr show`。常见机载 lan 口就是 `<NET_RC>.14/24`。
2. **端口有没有人听？** `ss -lntup | grep <port>`。
3. **对端在不在这个网段？** `arp -a | grep 192.168.144`。
   有记录=链路通；`<incomplete>`=IP 不存在；**整段无记录=二层都不通**。
4. **物理链路**：`ip -s link show lan1` 看 `LOWER_UP`（有载波）与 errors/dropped 是否 0。
5. **协议握手**：见上面「先分清三种图传」。若是私有 6000 端口协议，
   服务端 `recv()` 阻塞等 `AUTH:` 就是客户端不会握手 —— 用前台运行看栈：
   ```bash
   python3 -u <stream_script>     # Ctrl+C 时的 traceback 直接暴露阻塞在哪个 recv
   ```
6. **鉴权开关**（AirCore 系）：`LIYUMEN_STREAM_TOKEN=on` 要求一次性 token（6004 签发→6000 出示）；
   且 `LIYUMEN_SECRET` 一设，`is_auth_enabled()` 恒 True，**只关 TOKEN 不够，两者都要关**
   才会退化成裸连。改 `auth.env` 前先备份，并注意该文件被 command/touch 服务共用。

## 诊断纪律（本次会话踩过的判断失误）

**`400 Bad Request` / `000` 这类"连上了但不对"的信号，不要先归因到网络或防火墙。**
先证明"服务端到底期望客户端发什么"：

1. 用 `nc`/`printf` 发一个最小合法请求，看服务端真实回什么（`OPTIONS` 能 200 就说明协议栈好的）
2. **把管线/单元单独跑一遍**（`gst-launch-1.0 ... ! fakesink`）—— 服务里失败的构件单独测往往是秒出结果
3. **停掉托管服务、前台手动跑**，Ctrl+C 的 traceback 直接暴露阻塞点

**`curl` 返回 `000` 不等于"服务坏了"** —— 它只说明"对端不是 HTTP"。
先确认协议类型，再谈连通性。

**端口 LISTEN 不等于服务可用。** 必须走完 管线能建 → SDP 能出 → 真解码 三步。

**用户说"重启视频流服务"时，先查清服务是谁在托管**（systemd user unit / 自启脚本 / 手工 nohup），
再决定命令。贸然跑一个 `restart_*.sh` 可能先停掉 systemd 服务、再用已失效的旧路径启动失败，
把原本能跑的服务搞停。

## 网络侧通用事实
- 机载常配多网口：`lan1`=`<NET_RC>.14/24`（设备/遥控器侧）、`lan2`=`<NET_LAN2>.5/24`、WiFi DHCP。
  用 `ip route` 确认 `<NET_RC>.0/24 dev lan1` 存在。
- **⚠️ 网口名因机而异，别把 `lan1/lan2` 当固定事实写进配置或文档**（2026-10 实测两台同型 Orin NX）：
  一台是 `lan1` + `lan2` + `wlP1p1s0`，另一台是 **`eth0`（=<NET_RC>.14/24）+ `wlan0`，且没有第二个有线口**。
  `<NET_RC>.14/24` 这个**地址**是稳定的（遥控器/透传侧约定），但**接口名、是否有 lan2、WiFi 名
  都必须先 `ip -4 addr show` 读出来**，再写命令/脚本/交付文档。写错接口名会让整条透传链路看起来"没配好"。
- **不要用 `ping` 判断服务是否活着** —— 防火墙常拦 ICMP。用 `Test-NetConnection -Port 22`（Windows）或 `nc -z`。
- Windows 侧有线网卡显示 `Disconnected` + IP 是 `169.254.x` = 网线没插好（APIPA），不是软件问题。

## 相关技能
- `references/aircore-private-stream-protocol.md`：AirCore/Liyumen 6000 端口私有推流协议完整诊断实录
  （源码级握手分支、token 开关语义、drop-in 安全改法、`restart_qgc_services.sh` 旧路径坑、端口对照表）
- `references/windows-plink-remote-ops.md`：从 Windows 用 plink/pscp 非交互运维机载电脑
  （`-m` 脚本须 LF+纯 ASCII、`cat >` 会写出 0 字节必须用 pscp、后台进程与 `systemctl --user` 的退出码坑、
  journalctl --user 不可用时的替代、sudo 受限时的零依赖替代命令）
- `uav-ros-sensor-diagnostics`：飞控串口/LiDAR 排查、`stty` 波特率验证、指挥官模式偏好
- `technical-diagram-svg`：给方案出黑白功能实现图
