# 国内网络 + WSL2 + ROS2/Gazebo 源 排障实录

本记录来自一次完整排障（Win11 + WSL2 + Ubuntu 24.04 + ROS2 Jazzy + Gazebo，大陆网络），给出每一步的**判定方法**，未来可直接复用。

## 0. 先确认机器
- `wsl --version`：要新版本（本机 2.7.12，含 WSLg）
- `wsl -l -v`：列出发行版，`*` = 默认发行版
- `Get-CimInstance Win32_VideoController`：确认有 NVIDIA 独显（Gazebo 渲染 / CUDA 直通用）
- `Get-CimInstance Win32_ComputerSystem`：物理内存（本机 15.7GB → WSL 分 12GB）

## 1. DNS 是否污染（不是这一步的锅）
```powershell
Resolve-DnsName packages.ros.org          # 系统解析
# 拿真实IP对照：阿里 DoH
Invoke-WebRequest "https://dns.alidns.com/resolve?name=packages.ros.org&type=A" -UseBasicParsing
```
结论：`packages.ros.org` → CNAME `ftp.osuosl.org` → `64.50.233.100 / 140.211.166.134 / 64.50.236.52`。系统解析与 DoH 一致 = **DNS 未污染**，问题在连接层。

## 2. 定位：OSL 机房大陆直连不通 + Clash 只在 Windows 127.0.0.1
- Windows 宿主 curl `packages.ros.org` = 000（无代理直连）
- WSL 内直连 = 000；WSL 内 GitHub = 200（对照，说明外网路由有通，只是 OSL 被限）
- 虚拟网卡列表**无 TUN/TAP**（Clash 是普通 HTTP/SOCKS 代理模式，非全局 TUN）
- `127.0.0.1:7890` OPEN（= Clash 监听本机）
- WSL 经 `172.25.16.1:7890`（NAT 网关）连 Clash = **超时 000**
- 结论：**WSL2 默认 NAT 模式，够不到仅监听 Windows 127.0.0.1 的 Clash**

## 3. 修复：镜像网络模式
`C:\Users\<user>\.wslconfig`：
```ini
[wsl2]
networkingMode=mirrored
# 保留原有 kernel= / memory / processors / swap 等行
```
> 注意：改 `.wslconfig` 前先 `Copy-Item` 备份；kernel 路径若原为单反斜杠 Windows 路径，会触发 WSL 校验"转义字符无效"，**改成正斜杠**（`C:/ProgramData/...`）可消除。
然后 `wsl --shutdown && wsl -d <name>`。生效特征：`ip route` 网关从 `172.25.x` 变 `172.27.x` / 共享 Windows 地址；`127.0.0.1:7890` TCP OPEN。

## 4. 确认代理链路分情况
经 `127.0.0.1:7890` 代理测（`curl -x http://127.0.0.1:7890 -sIL -o /dev/null -w ...`）：
- google/example/bing/baidu → 200（代理链路正常）
- `http://packages.ros.org/.../dists/jazzy/Release` → 404（**路径用错了**，见下）
- `http://packages.ros.org/.../dists/noble/Release` → 200（**dists 用 noble**）
- `.../Packages.gz` → 200 但大包时可能 502

## 5. ROS2 apt 源正确写法
```bash
sudo curl -x http://127.0.0.1:7890 -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
```
> GPG key 下载必须 `-x` 走代理（WSL 直连 `raw.githubusercontent.com` 不通）。

## 6. apt 走代理 + 重试
`/etc/apt/apt.conf.d/proxy`：
```
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
Acquire::Retries "8";
```
装包：`sudo apt install -y ros-jazzy-desktop -o Acquire::Retries=8`

## 7. 502 Bad Gateway [IP: 127.0.0.1 7890] = Clash 节点问题
反复出现在**大包下载**（deb）而非元数据。修法：**重启 Clash 换健康节点**（通常能救）；仍 502 转 RoboStack/conda（走清华 anaconda 镜像，国内直连）。对照测：`curl -x http://127.0.0.1:7890 http://packages.osrfoundation.org/gazebo/ubuntu-stable/dists/noble/Release` = 200（OSRF 源绕开 packages.ros.org）。

## 8. 源清单 & key
| 用途 | 源 | key |
|---|---|---|
| ROS2 Jazzy | `http://packages.ros.org/ros2/ubuntu` (dists=noble) | `https://raw.githubusercontent.com/ros/rosdistro/master/ros.key` |
| Gazebo Harmonic | `https://packages.osrfoundation.org/gazebo/ubuntu-stable` (dists=noble) | `https://packages.osrfoundation.org/gazebo.gpg`（**根目录**） |
| Ubuntu 系统源 | 清华 `https://mirrors.tuna.tsinghua.edu.cn/ubuntu` | - |

## 9. 验证 ROS2 装好
```bash
source /opt/ros/jazzy/setup.bash && ros2 --version    # 出 0.31.x
```
`ros2: command not found` = 没 source。
