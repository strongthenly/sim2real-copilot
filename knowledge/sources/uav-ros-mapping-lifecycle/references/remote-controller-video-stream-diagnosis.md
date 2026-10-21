# 遥控器「虚拟投影 / 图传看不到」诊断（fouruav 推流 6000）

触发：遥控器（平板 QGC / 一体地面站）显示 **「等待视频信号接入」**，地址形如 `<NET_RC>.14:6000`；
用户说「遥控器没有虚拟投影」「图传黑屏」「推流没画面」。

## 0. 先认清这条链路是什么（90% 的误判源于此）

`<NET_RC>.14:6000` **不是 RTSP，也不是 MJPEG/HTTP**。它是 AirCore 的**私有二进制 TCP 协议**：

| 端口 | 协议 | 用途 |
|---|---|---|
| `6000/tcp` | 私有二进制 | 推流主端口；帧格式 `<4字节小端长度><JPEG>`；**连上后必须先发 `AUTH:<token>
`** |
| `6004/tcp` | HTTP | `GET /stream/token` 签发一次性 token（绑客户端 IP、TTL 60s、一次性） |
| `6004/udp` | 文本 | 状态/统计回报（`LIYUMEN_QGC_UDP_PORT`） |
| `8000/tcp` | HTTP | nginx 默认站点（静态页 + webdav），**和图传无关** |

进程：`rviz_full_stream2.py --worker`（父进程 `supervisor()` 守护，崩溃无限重启）+ `nx_command_server2.py`（6004 签发 token）。
服务端 IP 是 **`eth0` = `<NET_RC>.14/24`**，与 `wlan0`（如 <NET_LAN2>.110）双网段并存——别把 wlan IP 当推流地址。

### 关键判读：`curl` 报 `HTTP/0.9` = 服务是好的

```
curl -sS -m 3 -o /dev/null -w "HTTP %{http_code}\n" http://<NET_RC>.14:6000/
→ curl: (1) Received HTTP/0.9 when not allowed      HTTP 000
```
**这不是故障。** 含义是「TCP 连上了，对端发回的不是 HTTP 响应」——curl 发 `GET / HTTP/1.1`，
服务端把它当 AUTH 挑战，回的是裸 `AUTH:...` / 二进制帧。
正确结论是 **「端口活着、协议不是 HTTP」**，不要再顺着 `000` 去查网络/防火墙。

## 1. 分层排查顺序（不要跳步）

### ① 服务端在不在

```bash
sudo ss -lntup | grep -E ':600[0-9]'          # 6000 有 python3 监听 = 推流在跑
ip -4 addr show eth0                          # 应 = <NET_RC>.14/24
ps aux | grep -E 'rviz_full_stream2|supervisor_launcher' | grep -v grep
```

### ② 物理链路（`ip -s link` 比 ping 更早定性）

```bash
ip -s link show eth0
```
- `state UP` **且** `LOWER_UP`、RX/TX 在涨、`errors/dropped` 全 0 → **链路通，别再查网线**
- 只有 `UP` 没有 `LOWER_UP` → 对端没插/没上电

### ③ 对端到底是谁（arp 是金证据）

```bash
ping -c 3 -W 1 <NET_RC>.99 ; arp -a | grep 192.168.144
```
- `<incomplete>` = **该 IP 根本不存在**（连 ARP 都没人应答，不是防火墙）
- 出现 `(<NET_RC>.100) 位于 84:fc:14:... 在 eth0` = **对面活着的是 .100**
  （`.100` 常是图传模块/中继盒；遥控器可能挂在它后面做 NAT → NX 看不到遥控器的 ARP，
  此时「连不上 .14:6000」的根因在 `.100` 的转发，不在 NX）
- ★**别默认用户报的 IP 就是设备 IP**：本次 `.99` 完全不存在，实活地址是 `.100`。

### ④ 防火墙（Jetson 上通常本来就没装）

```bash
sudo ufw status verbose 2>/dev/null || echo "no ufw"
sudo iptables -L INPUT -n | head
```
`policy ACCEPT` + 无规则 = 不拦。`ufw: 找不到命令`是精简镜像的正常现象。

### ⑤ 协议/鉴权握手（**真正的高发根因**）

```bash
sudo tr '\0' '\n' < /proc/$(pgrep -f 'rviz_full_stream2.py --worker')/environ | grep -Ei 'LIYUMEN|STREAM|AUTH|TOKEN|DISPLAY'
grep -rn -Ei 'LIYUMEN_STREAM_TOKEN|LIYUMEN_STREAM_AUTH|LIYUMEN_STREAM_PORT' \
  ~/AirCore/*.sh ~/.bashrc ~/.profile /etc/environment 2>/dev/null
```

开关语义（`nx_stream_token.py` / `rviz_full_stream.py`）：

| 变量 | 默认 | 打开后行为 |
|---|---|---|
| `LIYUMEN_STREAM_TOKEN` | **off** | 握手要求 6004 签发的一次性 token（跨进程存 `/dev/shm/liyumen_stream_tokens.json`，IP 绑定 + TTL 60s + 一次性） |
| 静态鉴权 `verify_token("stream", ...)` | — | 要求 `AUTH:<静态token>`，失败回 `AUTH:FAIL:static_auth
` |
| `LIYUMEN_STREAM_PORT` | `6000` | 改端口 |
| `LIYUMEN_STREAM_BACKLOG` | 8（下限 5） | listen backlog |

失败码：`disabled / unknown / expired / ip_mismatch / reused / static_auth`。

**结论模板**：遥控器 App 只做「裸连 6000 收图、不做 AUTH 握手」时，会永远停在「等待视频信号接入」。
- 解法 A（最快）：关鉴权——`LIYUMEN_STREAM_TOKEN` 留空/off，重启推流
- 解法 B：遥控器端实现「6004 取 token → 6000 发 `AUTH:<token>`」（需 App 支持）

## 2. 启动脚本：先确认用的是哪一份（常见大坑）

机器上存在**两份同名 `start_qgc_servers.sh`**：

| 路径 | 状态 |
|---|---|
| `~/NX_code/start_qgc_servers.sh` | **旧版(4/28)，推流行被注释掉** `#nohup python3 ... rviz_full_stream.py` → 跑它永远出不了图 |
| `~/AirCore/start_qgc_servers.sh` | 正式版(9/21)：supervisor 拉起推流 + pkill 预检 + PID 防重 + `--stop` + 日志按 `log/YYYYMMDD_HHMMSS/` 分目录 + roscore 轮询就绪 + `xhost +SI:localuser:$USER` |

判定痕迹：正式版日志写 `$LIYUMEN_CATKIN_WS/src/task_run_scripts/log/latest/`；
该目录为空/不存在 → 跑的是 `NX_code` 旧脚本。重启用 `~/AirCore/restart_qgc_services.sh`。
（旧脚本还会 `nohup roscore`，与 AirCore supervisor 版并存会抢端口，先清现场再重启。）

## 3. 一次给全的自检命令组

```bash
sudo ss -lntup | grep -E ':600[0-9]'
ip -s link show eth0
arp -a | grep 192.168.144
sudo tr '\0' '\n' < /proc/$(pgrep -f 'rviz_full_stream2.py --worker')/environ | grep -Ei 'LIYUMEN|STREAM|AUTH|TOKEN'
ls -lt ~/catkin_ws/src/task_run_scripts/log/ | head -3
```

## 4. 本次实战结论（2026-10）

- NX `eth0` = `<NET_RC>.14/24`；`wlan0` = `<NET_LAN2>.110/24`（双网段）
- `:6000` 由 `rviz_full_stream2.py --worker` 正常监听；iptables 全 ACCEPT
- `curl :6000` 报 `HTTP/0.9` → 协议非 HTTP，非故障（曾据此误判为「服务不响应」，已纠正）
- arp 里 `.99` = `<incomplete>`（不存在），**活的是 `.100`（84:fc:14:…，疑图传/中继模块）**
- 待验证：`/proc/<pid>/environ` 里 `LIYUMEN_STREAM_TOKEN` / 静态鉴权是否开启
  → 「遥控器连上却拿不到图」的头号嫌疑
