# 数传（QGC UDP 14550）+ 图传（RTSP）经网线透传 —— 标准架构与部署

> 适用：用户要用**网线**把机载电脑接到地面电脑，在一台电脑上同时看
> QGC（飞控数据）和视频画面，且不接受私有协议（如 AirCore 的 6000/AUTH 握手）。
> 与 `qgc-stream-auth-and-systemd.md` 的关系：那篇是**私有协议**排障，
> 这篇是**换成标准协议**的正解 —— 当遥控器 App 不支持私有握手时，走这条路。

---

## 0. 先分清架构：谁在提供 RTSP

用户常说的"遥控器通过网线连电脑实现数传图传透传"，实际有三种形态，
**必须先问清是哪一种，否则配错对象**：

| 形态 | 服务跑在哪 | 网线两端 | 说明 |
|---|---|---|---|
| **A. 机载直出（推荐）** | 机载电脑 | 机载 eth0 ↔ 电脑网口 | 机载同时提供 MAVLink + RTSP，**电脑直接连，不经过遥控器** |
| **B. 遥控器透传** | 遥控器/图传模块 | 遥控器网口 ↔ 电脑 | 模块内部把射频收到的流重新暴露成 RTSP |
| **C. 私有协议** | 机载（如 AirCore 6000） | 机载 ↔ 电脑 | **不是 RTSP**，需厂家 App 配套（见另一篇 reference） |

判据：**看遥控器里填的地址是谁的 IP。**
- 填的是机载 IP（如 `<NET_RC>.14:6000`）→ 属 A/C 类，机载是服务端
- 填的是遥控器自己的 IP → 属 B 类

> 关键认知：**遥控器只是个带屏幕的终端，它本身不"做"数传图传。**
> 真正对外提供服务的是机载电脑（或模块）。所以"让电脑能看"这件事，
> 应当直接在机载电脑上做标准服务，绕开遥控器的私有协议。

---

## 1. 目标架构

```
┌─────────────┐  UART/USB   ┌──────────────────┐   以太网    ┌────────────┐
│  飞控 PX4   │──MAVLink───▶│  机载电脑         │──RJ45─────▶│  地面电脑   │
│ (TELEM2)    │             │  mavlink-router  │             │            │
└─────────────┘             │    → UDP 14550   │             │ QGC :14550 │
                            │                  │             │            │
┌─────────────┐  USB/CSI    │  mediamtx        │             │ VLC/ffplay │
│  相机/屏幕   │──视频──────▶│    → RTSP :8554  │             │ rtsp://... │
└─────────────┘             │  eth0 <NET_RC>.14/24         └────────────┘
                            └──────────────────┘
        ▲ 遥控器摇杆 ──射频──▶ 飞控 RC IN（独立链路，不走网线）
```

**两个软件各解决一件事：**
- `mavlink-router`：串口 MAVLink → UDP 14550（QGC 只认网络）
- `mediamtx`：视频源 → H.264 → RTSP :8554（VLC 只认 RTSP）

---

## 2. 网络层（先通这个，再谈服务）

```bash
# 机载侧
ip addr show eth0                    # 期望 <NET_RC>.14/24
# 地面电脑侧（Windows）
# 静态 IP <NET_RC>.100/255.255.255.0，网关留空
ping <NET_RC>.14
```

判物理链路（**别只看 ping**）：
```bash
ip -s link show eth0     # LOWER_UP + errors/dropped=0
arp -a | grep 192.168.144   # 有 MAC=对端活着；<incomplete>=该 IP 不存在
```

⚠️ **网段必须一致**。常见坑：机载接 WiFi 拿了 `192.168.2.x`（DHCP），
而网线侧是 `192.168.144.x` —— 此时 `ping <NET_RC>.14` 能通（走本地路由），
但遥控器/电脑从别的口访问会不通。**要确认服务监听在哪个网卡上**（`ss -lntup` 看 `0.0.0.0` vs 具体 IP）。

---

## 3. 数传：mavlink-router

```bash
sudo apt install -y mavlink-router
```

`/etc/mavlink-router/main.conf`：
```ini
[General]
TcpServerPort=0
ReportStats=false
MavlinkDialect=common

[UdpEndpoint qgc]
Mode = Normal
Address = <NET_RC>.100      # 地面电脑 IP
Port = 14550

[UartEndpoint fcu]
Device = /dev/ttyACM0          # 或 /dev/ttyTHS0
Baud = 57600                   # 与飞控 TELEM 口一致
```

```bash
sudo systemctl enable --now mavlink-router
```

**QGC 侧**：Application Settings → Comm Links → Add：
```
Type: UDP
Listening Port: 14550
```
（`Address` 留空 = 监听所有来源）

**端口冲突提醒**：若机载上已有别的进程往 14550 广播（如 AirCore 的推流服务），
电脑同时开两个监听会抢包。选一个。

---

## 4. 图传：mediamtx（RTSP server）

```bash
# Jetson/aarch64
wget https://github.com/bluenviron/mediamtx/releases/download/v1.9.3/mediamtx_v1.9.3_linux_arm64v8.tar.gz
tar -xzf mediamtx_v1.9.3_linux_arm64v8.tar.gz -C /opt/mediamtx
```

`mediamtx.yml` 关键段（按图传源三选一）：

```yaml
paths:
  live:
    # ① USB 摄像头
    runOnInit: ffmpeg -f v4l2 -i /dev/video0 -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp rtsp://localhost:8554/live
    runOnInitRestart: yes

    # ② Jetson CSI 相机（硬件编码，最省算力）
    # runOnInit: gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! nvv4l2h264enc bitrate=4000000 ! h264parse ! rtspclientsink location=rtsp://localhost:8554/live

    # ③ 抓屏幕（复现原来 rviz_full_stream 的效果）
    # runOnInit: ffmpeg -f x11grab -video_size 1920x1080 -framerate 25 -i :0.0 -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp rtsp://localhost:8554/live
```

**播放侧**：
```powershell
ffplay -rtsp_transport tcp rtsp://<NET_RC>.14:8554/live
# 或 VLC：媒体 → 打开网络串流
```

**`-rtsp_transport tcp` 很重要** —— 网线直连下 UDP 拉流容易丢包花屏，改 TCP 立刻稳。

---

## 5. 遥控器操纵（不走网线）

网线只用于**地面看数据/看画面**，不参与飞控操纵：
- 遥控器 → 机载接收机 → 飞控 RC IN（射频）
- 或遥控器 → 4G → 机载数传 → 飞控 TELEM 口（MAVLink RC override）

**别指望网线传摇杆**，除非模块明确支持"以太网 RC 透传"。

---

## 6. 最小验证（两条都通 = 功能成立）

```bash
# 数传
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print('waiting...'); m.wait_heartbeat(); print('HEARTBEAT OK', m.target_system)
"

# 图传
ffprobe -rtsp_transport tcp rtsp://<NET_RC>.14:8554/live
```

---

## 7. 坑

- **Jetson 无硬件编码时 CPU 爆**：`libx264 -preset ultrafast` 在 NX 上 1080p25 约吃 1-2 核；
  能上 `nvv4l2h264enc` 就用硬编。
- **mediamtx 版本要匹配架构**：`arm64v8`（Jetson Orin/NX aarch64），别下 amd64。
- **防火墙**：Ubuntu 若装了 ufw 要放行 8554/tcp + 14550/udp。
  实测这台机载**没有 ufw**（`sudo ufw status` → 找不到命令），iptables INPUT 为 ACCEPT。
- **mavlink-router 与 MAVROS 抢串口**：两者不能同时打开同一个 `/dev/ttyACM0`。
  要 MAVROS 就让它独占，再单独起个 MAVLink 转发；或只留一个。
- **写框图时**：黑白框线风格、箭头精确落在边框上，见技能 `technical-diagram-svg`。
