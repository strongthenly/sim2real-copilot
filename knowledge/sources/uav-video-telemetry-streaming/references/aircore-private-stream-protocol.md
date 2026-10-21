# AirCore / Liyumen 私有推流协议（6000 端口）诊断实录

> 场景：遥控器显示「等待视频信号接入：<NET_RC>.14:6000」，
> 机载电脑上推流进程明明在跑、端口也在听，但遥控器永远出不来图。

## 协议结构（从源码读出，`rviz_full_stream.py`）

```
tcp:6000   推流主端口    帧格式 = <4字节小端长度><JPEG数据>   ← 裸二进制，绝不是 HTTP
tcp:6004   token 签发    GET /stream/token
udp:6004   状态回报      qgc_udp_port
```

握手分支（`wait_for_client()`）：

```python
if _stream_token_enabled():                  # LIYUMEN_STREAM_TOKEN=on
    challenge = self._recv_auth_challenge(conn)     # 阻塞等 "AUTH:<token>\n"
    auth_ok, reason = _consume_stream_token(presented, addr[0])
    if not ok:
        conn.sendall(b"AUTH:FAIL:%s\n" % reason)
        conn.close(); continue

elif is_auth_enabled():                      # get_secret() is not None
    challenge = self._recv_auth_challenge(conn)
    auth_ok = challenge.startswith("AUTH:") and verify_token("stream", token)
    ...

# 两个都关 → 直接进推流循环，裸连即可收图
```

一次性 token 存储外化到 `/dev/shm/liyumen_stream_tokens.json`（flock 跨进程），
TTL 默认 60s、绑定客户端 IP。所以 `ip_mismatch` 往往意味着中间有 NAT。

## 三个决定性观测量

| 观测量 | 命令 | 读法 |
|---|---|---|
| 客户端连上后发不发数据 | 前台跑 `python3 -u rviz_full_stream2.py --worker`，让客户端连一次，然后 Ctrl+C | **traceback 停在 `_recv_auth_challenge` 的 `conn.recv()` → 客户端连上了但一句话不发**（App 不会握手）。这是最快的判别，比翻日志可靠得多 |
| 是不是 HTTP | `curl -sS -m 3 -o /dev/null -w "%{http_code}
" http://<ip>:6000/` | `curl: (1) Received HTTP/0.9 when not allowed` + `000` = 服务在响应但协议非 HTTP |
| 鉴权开关实际值 | `sudo tr '\0' '
' < /proc/<pid>/environ \| grep -E 'LIYUMEN'` | `LIYUMEN_STREAM_TOKEN=on` → 一次性 token 模式；`LIYUMEN_SECRET=<hex>` 存在 → `is_auth_enabled()` **恒 True** |

## 关键推论（容易踩错）

**只关 `LIYUMEN_STREAM_TOKEN` 不够。** 因为 `is_auth_enabled()` 的判定是：

```python
def is_auth_enabled():
    return get_secret() is not None      # 只判 LIYUMEN_SECRET 是否存在
```

只要 `LIYUMEN_SECRET` 有值，关掉 token 后会掉进 `elif` 静态鉴权分支，
客户端仍必须发 `AUTH:<HMAC-SHA256(secret, "stream")>`。**要让客户端裸连，两个都要关。**

## 安全开关的正确改法（只影响推流，不动其他服务）

`auth.env` 被 `aircore-command.service` / `aircore-touch.service` 共用，
直接改它会把指令和触控的鉴权一起关掉。**用 drop-in 只覆盖推流服务**：

```bash
mkdir -p ~/.config/systemd/user/aircore-stream.service.d
cat > ~/.config/systemd/user/aircore-stream.service.d/20-no-auth.conf <<'EOF'
[Service]
Environment=LIYUMEN_STREAM_TOKEN=off
Environment=LIYUMEN_SECRET=
EOF
systemctl --user daemon-reload
systemctl --user restart aircore-stream.service
```

先备份再改：

```bash
cp ~/.local/state/aircore-source-*/auth.env{,.bak-$(date +%Y%m%d_%H%M%S)}
```

## 服务拓扑（AirCore 2026-09 版）

`aircore.target` 下四个 user service：
`aircore-roscore` / `aircore-stream` / `aircore-command` / `aircore-touch`

正确操作是 `systemctl --user restart aircore.target`。

**坑：`restart_qgc_services.sh` 里写死了旧路径**
`~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_qgc_servers.sh`
—— 该路径在部分机器上不存在。它会**先 stop 掉 systemd 服务、再用旧路径启动失败**，
结果是推流彻底没起来、6000 端口无监听。日志在 `/tmp/restart_qgc_services.log`。

**坑：`~/NX_code/start_qgc_servers.sh`（旧版）的推流行被注释掉了**
```bash
#nohup python3 $BASE_DIR/rviz_full_stream.py > /dev/null  2>&1 &
```
跑这个脚本永远不会起推流。正式版是 `~/AirCore/start_qgc_servers.sh`（带 supervisor、
端口预检、PID 防重、日志分目录）。**判断哪份是正式版：看文件 mtime + 行数 + 是否有 supervisor_launcher。**

## 端口对照（免得搞混）

| 端口 | 用途 | 出现场景 |
|---|---|---|
| 6000/tcp | 私有推流（自定义帧格式） | AirCore 系 |
| 6004/tcp | token 签发 / HTTP 指令 | AirCore 系 |
| 6002,6003/udp | 状态回报 / 触控 | AirCore 系 |
| 8000/tcp | nginx 静态站点 + webdav | 同机常驻 |
| 8554/tcp | **标准 RTSP** | 本文主技能搭建的服务 |
| 14550/udp | **MAVLink → QGC** | mavlink-router / mavros |
