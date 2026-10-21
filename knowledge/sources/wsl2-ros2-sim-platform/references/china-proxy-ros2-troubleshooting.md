# 中国网络 × WSL2 下搭建 ROS2/Gazebo 的排障决策流

本文件是实测结论，帮助快速定位问题归属。所有探测都要在 **WSL 内部** 进行（不是 Windows 宿主侧），因为 WSL 与宿主走不同网络栈。

## 1. 先归因：502 / 镜像 / 路径，谁的问题？

**方法**（在 WSL 里跑，经 Clash 代理）：
```bash
# 测正确路径（Ubuntu 代号 noble，不是 jazzy！）
curl -x http://127.0.0.1:7890 -sIL -o /dev/null -w "%{http_code}\n" \
  --max-time 15 https://packages.ros.org/ros2/ubuntu/dists/noble/Release
```

**判读矩阵**：

| 现象 | 结论 | 处理 |
|---|---|---|
| `dists/jazzy/...` 全 404，`dists/noble/...` 200 | **路径用错了代号** | 改用 `noble`（Ubuntu 代号）|
| 元数据（小文件）200，大 deb 成片 `502 Bad Gateway` | **Clash 节点对大量下载不稳** | 换更快更稳的节点（看下载速度，非 ping）+ `-o Acquire::Retries=10` |
| 下载只有 ~17 kB/s | **节点带宽太差** | 换节点，别硬重试 |
| `raw.githubusercontent.com` 直连超时 | GitHub raw 在 WSL 直连不通 | 下载 key 时带 `-x http://127.0.0.1:7890` |
| OSRF 源 `packages.osrfoundation.org/gazebo/...` 200 | Gazebo 走 OSRF 更稳 | 优先用 OSRF 装 gz-harmonic |
| `127.0.0.1:7890` TCP 通、但代理请求 000/超时 | WSL 还没共享 Windows 网络 | 检查 `networkingMode=mirrored` 是否生效（`ip route` 应无 172.25 网段）|

## 2. DNS 污染 vs 域名被墙

若 `packages.ros.org` 直连 000，先分清是 DNS 还是连接：
```powershell
# Windows 侧
Resolve-DnsName packages.ros.org              # 系统解析
Invoke-WebRequest -Uri "https://dns.alidns.com/resolve?name=packages.ros.org&type=A" -UseBasicParsing  # DoH 对照
```
若系统解析与 DoH 一致（都指向 `ftp.osuosl.org`，如 64.50.x/140.211.x），**不是 DNS 污染，是到 OSL 机房的路由不通** → 必须走代理。

## 3. 代理连接路径确认

- WSL 是 NAT 模式时，Windows 宿主机从 WSL 看是网关 IP（`ip route | awk '/default via/{print $3}'`，常见 `172.25.16.1`）。若代理只监听 `127.0.0.1`，该 IP 够不到。
- 切到 `networkingMode=mirrored` 后：`127.0.0.1` 在 WSL 与 Windows 互通，`http://127.0.0.1:7890` 即可用。
- 判断 `mirrored` 是否生效：`ip route` 里的 default 网关应不再是 `172.25.x/16` 独立段；`ip addr` 与 Windows 共享地址。

## 4. 为什么国内找不到 ROS2 镜像

实测（2026-09）下列 `ros2/ubuntu/dists/jazzy|noble/Release` 全部 404：
- `mirrors.tuna.tsinghua.edu.cn` / `mirrors.aliyun.com` / `mirrors.ustc.edu.cn` / `mirrors.huaweicloud.com` / `mirrors.cloud.tencent.com` / `repo.huaweicloud.com`

结论：国内**无 ROS2 apt 二进制镜像**，`packages.ros.org` 是唯一官方源且需代理；Ubuntu 的 apt 源可放心换成清华/腾讯镜像（直连通、快）。

## 5. OSRF(Gazebo) 与 ROS 相关的可用端点

- Gazebo apt 源：`https://packages.osrfoundation.org/gazebo/ubuntu-stable`（dists 用 `noble`）
- Gazebo GPG key：`https://packages.osrfoundation.org/gazebo.gpg`（**根路径**，GET 返回 1222B）
- ROS2 源：`http://packages.ros.org/ros2/ubuntu`（dists 用 `noble`）
- ROS2 key：`https://raw.githubusercontent.com/ros/rosdistro/master/ros.key`（需代理）

## 6. 一批实证命令（可复跑）

```bash
# WSL 侧，经 Clash
curl -x http://127.0.0.1:7890 -sL -o /dev/null -w "%{http_code} %{size_download}B\n" --max-time 15 https://packages.osrfoundation.org/gazebo.gpg
curl -x http://127.0.0.1:7890 -sL -o /dev/null -w "%{http_code} %{size_download}B\n" --max-time 15 http://packages.ros.org/ros2/ubuntu/dists/noble/Release
# 对照：代理对已知国外站的连通性
curl -x http://127.0.0.1:7890 -sIL -o /dev/null -w "%{http_code}\n" --max-time 10 https://www.google.com
```
