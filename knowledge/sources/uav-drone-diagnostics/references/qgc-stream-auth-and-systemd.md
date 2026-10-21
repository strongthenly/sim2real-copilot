# 图传推流「遥控器一直等视频信号」：systemd 托管 + 鉴权握手排查（2026-10-19 实测）

> 前一篇 `qgc-stream-services.md` 记的是 **GNOME autostart + 崩溃循环** 时代的故障。
> 2026-09 之后机载升级为 **systemd user 单元托管 + 推流鉴权**，故障形态完全变了。
> 症状长这样：**服务全在跑、6000 也在监听、链路 ping 得通 —— 遥控器却一直显示
> 「等待视频信号接入：<NET_RC>.14:6000」。** 这篇专治这一类。

---

## 0. 先认清：现在有两套启动体系，互相打架

| 体系 | 路径 | 状态 |
|---|---|---|
| **systemd user（当前正式）** | `~/.config/systemd/user/aircore-*.service` + `aircore.target` | ✅ 开机自启，四服务：`aircore-roscore` / `aircore-stream` / `aircore-command` / `aircore-touch` |
| 旧脚本 | `restart_qgc_services.sh` → `start_qgc_servers.sh` | ⚠️ 路径可能已失效，**会先杀 systemd 服务再起不来** |

### 🔴 头号事故：`restart_qgc_services.sh` 会「杀完不启动」

实测输出：

```
[16:22:11] 端口检查：TCP 6000/6004 已释放 ...      ← 已经把 stream 杀了
[16:22:21] ⚠️ 端口未完全就绪                       ← 6000 只剩 0 个监听
```

`/tmp/restart_qgc_services.log` 里的真因：

```
nohup: 无法运行命令 '/home/fouruav/catkin_ws/src/task_run_scripts/scripts/NX_code/start_qgc_servers.sh':
没有那个文件或目录
```

**旧脚本里硬编码的是旧路径，而实际的 NX_code 已挪到 `/home/fouruav/NX_code/`。**
于是 `pkill` 把 systemd 的 `aircore-stream.service` 停掉（日志里
`Process: 8455 ... (code=killed, signal=INT)`），再用不存在的路径去拉 → **推流彻底没了**。

**⇒ 铁律：在这台机器上重启服务只用 systemd，别用旧脚本。**

```bash
systemctl --user restart aircore.target       # 全套
systemctl --user restart aircore-stream.service   # 只推流
systemctl --user status aircore.target
systemctl --user list-units --all | grep -i aircore
```

诊断「谁在管这个进程」的可靠办法（不要看 PPID）：

```bash
cat /proc/<pid>/cgroup | grep -o '[^/]*\.service'
# → 1:name=systemd:/user.slice/user-1000.slice/user@1000.service/aircore-stream.service
```

⚠️ `ps -o ppid` 只能看到 `1554`（= `user@1000.service`），**看不出具体是哪个单元**；
`systemctl status $(cat /proc/<pid>/cgroup | grep -o '[^/]*\.service' | head -1)`
会抓到一个空的匹配（`.service` 前面没名字）而落到 `user@1000.service` 整棵树上 —— 会误读成
「所有 aircore 都在同一个服务里」。用上面那条 `cgroup | grep` 才对。

---

## 1. 推流端口与协议（别再当 HTTP/RTSP 去测）

| 端口 | 协议 | 用途 |
|---|---|---|
| **6000/tcp** | **自定义二进制：`<4字节小端长度><JPEG>`** | 推流主通道 |
| 6004/tcp | HTTP | 指令桥 / **签发一次性推流 token** |
| 6004/udp | 自定义 | 状态回报（stats/QGC 心跳） |
| 6002/udp · 6003/udp | 自定义 | 触控 / QGC 指令 |
| 11311 | ROS master | |
| 8000 | HTTP | nginx 默认站 + webdav |

**帧格式源码事实**（`rviz_full_stream.py`）：

```python
header = struct.pack('<I', len(jpeg_bytes))
self.latest_frame = header + jpeg_bytes
# ...
self.client_conn.sendall(frame_to_send)
```

### 🔑 `curl` 报 `HTTP/0.9` ≠ 服务坏了

```
curl: (1) Received HTTP/0.9 when not allowed
HTTP 000
```

这条**极易误判成「端口不通 / 服务异常」**。真相是：curl **TCP 连上了**，对端回了非 HTTP 的裸字节，
curl 在 HTTP/0.9 模式下拒收 → 返回 000。

**判读口诀**：
- 收到 `HTTP/0.9` → **端口是通的、服务是活的**，只是协议不是 HTTP。别再拿 curl 判活。
- 真正不通的 curl 表现是 `Failed to connect` / `Connection refused` / 超时，不是 0.9。

### 🔴 本会话连错两次的完整教训（同一个坑别踩第三遍）

| 我看到的证据 | 我当时的结论 | ❌ 真相 |
|---|---|---|
| `ping .99` 100% 丢包 | 「网线没插 / 对端没上电」 | `ip -s link show` 有 `LOWER_UP`、errors=0、`arp` 里有 `.100` 带 MAC → **链路好，只是 `.99` 这个 IP 不存在** |
| `curl :6000` → `HTTP/0.9` + `000` | 「服务虽监听但不响应，推流有 bug」 | TCP 连上了、服务活着，6000 是**自定义二进制协议**，curl 拒收而已 |

**共同病根：拿单一信号（ping / curl / ss）就去断言层级别。**
正确姿势是先答「**我现在验证的是哪一层**」，再去取该层专有的证据：

| 想证明 | 该看的（不是 ping） |
|---|---|
| 物理链路通 | `ip -s link show <if>`：`LOWER_UP` + `errors/dropped=0` |
| 对端真的有这个 IP | `arp -a`：有 MAC = 活；`<incomplete>` = 该 IP 不存在 |
| 端口有服务 | `ss -lntup` 有监听 |
| 服务**真的在服务** | `Ctrl+C` 回溯栈（§3.3.1），不是 curl |

---

## 2. 鉴权握手链（「等待视频信号接入」的真凶）

### 2.1 两种模式，由环境变量决定

源码 `rviz_full_stream.py` 的 `wait_for_client()`：

```python
if not is_ip_allowed(addr[0]):                    # LIYUMEN_ALLOWED_IPS，空=不限
    conn.sendall(b"AUTH:DENIED\n"); conn.close()

if _stream_token_enabled():                        # ① LIYUMEN_STREAM_TOKEN=on
    challenge = self._recv_auth_challenge(conn)    #    等客户端发 "AUTH:<一次性token>\n"
    auth_ok, reason = _consume_stream_token(presented, addr[0])
    if not (challenge.startswith("AUTH:") and auth_ok):
        conn.sendall(("AUTH:FAIL:%s\n" % reason).encode()); conn.close()

elif is_auth_enabled():                            # ② LIYUMEN_SECRET 非空 ⇒ 恒为 True
    challenge = self._recv_auth_challenge(conn)
    auth_ok = challenge.startswith("AUTH:") and verify_token("stream", presented)
    if not auth_ok:
        conn.sendall(b"AUTH:FAIL:static_auth\n"); conn.close()

# 两关都过 → 才开始推 JPEG 帧
```

### 2.2 ★ `is_auth_enabled()` 的坑：只要 SECRET 存在就是 True

```python
def get_secret():
    secret = os.environ.get("LIYUMEN_SECRET", "").strip()
    return secret or None

def is_auth_enabled():
    return get_secret() is not None      # ← SECRET 设了就是 True
```

**⇒ 只把 `LIYUMEN_STREAM_TOKEN` 关掉是不够的。** 会退到 `elif` 分支继续要求
`AUTH:<HMAC-SHA256(secret,"stream")>`。要「裸连直接收图」必须**两个一起关**
（清空 `LIYUMEN_SECRET` 或注释掉）。

⚠️ 但 `LIYUMEN_SECRET` 是**全局共享密钥**，command / touch / HTTP 鉴权都用它 ——
关掉它会连带削弱其它服务。**动它之前必须先把 §3 的日志判读做完**，确认遥控器真的不做握手。

### 2.3 一次性 token 流程（遥控器 App 若支持，就该这么走）

```
遥控器 ──①──▶ 6004/tcp   GET /stream/token     （绑定来源 IP，TTL 默认 60s，一次性）
遥控器 ◀──②── NX 签发
遥控器 ──③──▶ 6000/tcp   发 "AUTH:<token>\n"
NX     ◀──④── 校验通过 → 开始推流
```

跨进程存储：`/dev/shm/liyumen_stream_tokens.json`（flock 原子读写；tmpfs 断电即清）
开关：`LIYUMEN_STREAM_TOKEN` ∈ {on,true,1,yes}；默认 off。

**`reason` 值直接指向根因**（日志里 `一次性token握手失败 [ip] reason=xxx`）：

| reason | 含义 | 该修什么 |
|---|---|---|
| `unknown` | token 不在库 / 已被消费 | 6004 签发链路断了，或遥控器没去取 |
| `expired` | 超 TTL | 调 `LIYUMEN_STREAM_TOKEN_TTL` |
| `ip_mismatch` | 源 IP ≠ 签发时 IP | **中间设备在做 NAT**（如 `.100` 网关），修网络而不是关鉴权 |
| `disabled` | token 模式其实是关的 | 走的是静态分支 |
| `static_auth` | 静态 HMAC 不匹配 | 遥控器有握手能力，配对 secret 即可 |

---

## 3. 排查顺序（照这个走，别乱试）

### 3.1 先看服务端是不是真在「等」

```bash
# 谁在监听 6000
sudo ss -lntup | grep -E ':600[0-9]'
# 服务归属哪个 systemd 单元
cat /proc/<pid>/cgroup | grep -o '[^/]*\.service'
# 环境变量（决定性）
sudo tr '\0' '\n' < /proc/<pid>/environ | grep -Ei 'LIYUMEN|STREAM|AUTH|TOKEN|DISPLAY'
```

必看这几项：
- `LIYUMEN_STREAM_TOKEN` → on = 一次性模式
- `LIYUMEN_SECRET` 非空 → `is_auth_enabled()` = True
- `LIYUMEN_ALLOWED_IPS` 空 → 不限来源
- `LIYUMEN_BIND_ADDR` 空 → 默认 0.0.0.0

### 3.2 网络层先排干净（避免把链路问题当鉴权问题）

```bash
ip -4 addr show | grep 'inet '         # 目标 IP 是否真在本机某网卡上
ip -s link show eth0                    # LOWER_UP + errors/dropped=0 = 物理链路 OK
arp -a | grep <网段>                    # 谁真的活着（<incomplete> = 该 IP 不存在）
```

**实测教训**：`LOWER_UP` + RX/TX 都有包 + `arp` 里出现 `.100` 有 MAC
⇒ 链路是通的，`ping .99` 全丢只是 **`.99` 这个 IP 根本不存在**，不是"网线没插"。
别据一个 ping 不通就断言物理断链。

### 3.3 拿握手的真日志（这是唯一能定性的证据）

```bash
journalctl --user -u aircore-stream.service -f
```

⚠️ **可能报 `No journal files were found`**（用户级 journald 未持久化）。
此时改用**前台直接跑**，日志必打到终端：

```bash
systemctl --user stop aircore-stream.service
cd ~/AirCore
export DISPLAY=:0
export LIYUMEN_STREAM_TOKEN=on
export LIYUMEN_SECRET=<从 /proc/<pid>/environ 抄>
python3 -u rviz_full_stream2.py --worker
# 让遥控器连一次，看终端输出；Ctrl+C 后 systemctl --user start 恢复
```

判读表：

| 终端输出 | 真正的问题 |
|---|---|
| `一次性token握手失败 [ip] reason=unknown` | 遥控器没去 6004 取 token，或 token 已烧毁 |
| `... reason=ip_mismatch` | 中间设备 NAT |
| `推流握手失败 [ip]` + `AUTH:FAIL:static_auth` | 遥控器有握手能力，secret 不对 |
| `拒绝非白名单推流客户端 [ip]` | ALLOWED_IPS 配错了 |
| 反复 `跳过死连接` | 遥控器连上就断，协议不匹配 |
| **完全无输出** | 连接根本没到 6000 → 网络层 |

### 3.3.1 ★ 最强判据：`Ctrl+C` 的回溯栈告诉你「它当时在等什么」

**这是本次唯一真正定性的证据，比任何日志都可靠**（因为用户级 journal 可能是空的、
服务日志可能被吞）。做法：前台跑 `rviz_full_stream2.py --worker`，**让遥控器连一次**，
然后按 `Ctrl+C`。看 traceback 停在哪个函数：

```
File "rviz_full_stream.py", line 726, in run
    self.wait_for_client()
File "rviz_full_stream.py", line 531, in wait_for_client
    challenge = self._recv_auth_challenge(conn)      ← 关键
File "rviz_full_stream.py", line 406, in _recv_auth_challenge
    challenge = conn.recv(1024).decode(...)
KeyboardInterrupt
```

**判读口诀 —— 看 `KeyboardInterrupt` 落在哪一行：**

| 回溯停在 | 说明 | 结论 |
|---|---|---|
| `conn.recv(1024)`（`_recv_auth_challenge` 内） | 已被 `accept()` 接受，正在**阻塞等 `AUTH:` 那行** | **遥控器连上了 6000 但一个字都不发** → App 不做握手 |
| `server_socket.accept()`（`wait_for_client` 内） | 根本没客户端连进来 | 网络层 / 遥控器侧问题 |
| `sendall` 附近 | 握手过了，卡在发帧 | 推流方向问题（对端不收 / 窗口满） |

**两种「等 AUTH」要分清，结论完全相反：**
- **等 AUTH 且遥控器不发** → App 不支持握手 → 只能关鉴权（§4）。
- **等 AUTH 而日志里出现 `reason=...`** → App 有握手能力但校验失败 → 修 token/NAT，
  **不要关鉴权**。

⇒ **先按 `Ctrl+C` 看栈，再决定改不改鉴权。** 盲目关鉴权会掩盖真正的 NAT/版本问题。

⚠️ 前提是**只用单个遥控器连**。多个客户端并发时 6000 是"单客户端接管"语义
（`_close_client_conn("takeover")`），回溯位置会被其它连接干扰。

### 3.4 没有 tcpdump 时怎么看有没有包到

```bash
# 遥控器连接时实时看
watch -n 0.5 'ss -tn "( sport = :6000 or dport = :6000 )"'
```

- 出现 `ESTABLISHED` → 流量到了，问题在应用层（鉴权）
- 完全没有 → 连接没建立，问题在网络层 / 遥控器侧

（`tcpdump` 在这台机上未安装；需要时 `sudo apt-get install -y tcpdump`）

---

## 4. 关鉴权这条路（仅在确认遥控器不做握手时才走）

**先备份，逐处改，贴 diff，留一键回退**（用户硬规矩）：

```bash
# 1. 备份
cp ~/.local/state/aircore-source-20260920/auth.env \
   ~/.local/state/aircore-source-20260920/auth.env.bak-$(date +%Y%m%d_%H%M%S)

# 2. 两行都注释掉（只关一个没用，见 §2.2）
#    LIYUMEN_STREAM_TOKEN=on   → # LIYUMEN_STREAM_TOKEN=on
#    LIYUMEN_SECRET=...        → # LIYUMEN_SECRET=...

# 3. 重启
systemctl --user restart aircore-stream.service

# 4. 回退
mv auth.env.bak-<ts> auth.env && systemctl --user restart aircore-stream.service
```

env 文件位置由单元文件 `EnvironmentFile=` 指定，实测：
`/home/fouruav/.local/state/aircore-source-20260920/auth.env`

**别用 `grep` 在 `~/.bashrc` `/etc/environment` `*.sh` 里找这些变量** —— 实测全找不到，
因为它们是 systemd `EnvironmentFile=` 注入的。要确认来源就 `systemctl --user cat <unit>`。

---

## 5. 源码速查：文件都在 `~/AirCore/`

`~/AirCore` 是指向 `AirCore-releases/<commit>/` 的软链。

| 文件 | 职责 |
|---|---|
| `rviz_full_stream.py` | 主体：`FullScreenStreamer` + `supervisor()`，6000 握手/推流全在这 |
| `rviz_full_stream2.py` | 44 行 wrapper：`--worker` → `FullScreenStreamer().run()`；否则 `supervisor()` |
| `nx_stream_token.py` | 一次性 token 签发/校验（`/dev/shm/liyumen_stream_tokens.json`） |
| `nx_stream_settings.py` | 分辨率/质量/fps 白名单，`~/.config/liyumen/stream_settings.json` |
| `nx_security.py` | `get_bind_addr` / `is_ip_allowed` / `is_auth_enabled` / `make_token` / `verify_token` |
| `restart_qgc_services.sh` | ⚠️ 旧体系，路径可能已失效（见 §0） |

---

## 6. 一句话判据表

| 现象 | 结论 |
|---|---|
| `curl` 报 `HTTP/0.9` | **端口通**，协议非 HTTP，别当故障 |
| 6000 无监听 | stream 服务死了；查 `systemctl --user status aircore-stream` |
| 服务在跑 + 6000 在听 + ping 通 + 遥控器还是「等待信号」 | **卡在鉴权握手**，去看 §3.3 日志 |
| `arp` 有 MAC 但 `ping` 不通 | 那个 IP 不存在，**不是**链路断（看 `LOWER_UP` + errors=0） |
| `restart_qgc_services.sh` 报「端口未完全就绪 期望2 实际1」 | 旧脚本杀完没启动成功，用 systemd 重启 |
