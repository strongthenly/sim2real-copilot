# 本次实测脚本（可复制复用）

> PowerShell 宿主侧跑 WSL 内多行命令时，`bash -lc "多行"` 会因 PowerShell 引号/转义分层解析崩掉（报 `无法将 ip 项识别为 cmdlet`）。
> **正确姿势**：把命令写成 `.sh` 脚本（放到 C:\Users\<u>\ 下），用 `wsl.exe -d <distro> -- bash /mnt/c/Users/<u>/脚本.sh` 执行。

## 1. 连通性诊断探针（判断"代理链路健康" vs "仅目标域名/规则问题"）
```bash
#!/bin/bash
P="http://127.0.0.1:7890"
for u in \
  "https://www.google.com" "https://example.com" \
  "https://www.bing.com" "https://www.baidu.com" \
  "https://raw.githubusercontent.com" \
  "http://packages.ros.org/ros2/ubuntu/dists/noble/Release" \
  "http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-amd64/Packages.gz" ; do
  code=$(curl -x "$P" -sL -o /dev/null -w "%{http_code} %{size_download}B" --max-time 12 "$u" 2>&1)
  echo "  $u -> $code"
done
```
- 前五个**都应 200**（尤其 google/bing/example）= Clash 链路健康。
- 若它们全通、只有 ROS 域名异常 → 是路径(用 noble 非 jazzy)或规则问题，不是网络。

## 2. 看 404 响应体判断"服务器是否真到了"
```bash
curl -x http://127.0.0.1:7890 -sL --max-time 15 "http://packages.ros.org/ros2/ubuntu/dists/noble/Release" | head -c 400
```
- body 出现 `<address>Apache Server at packages.ros.org Port 80</address>` = 真到服务器，只是 URL/路径不对。
- body 为空/000 = 没到（网络/规则问题）。

## 3. 确认镜像模式是否生效（WSL 内）
```bash
ip route | grep default     # NAT: default via 172.25.x / 172.27.x dev eth0；镜像后网段/设备可能变(如 eth2)
timeout 4 bash -c 'exec 3<>/dev/tcp/127.0.0.1/7890 && echo OPEN' || echo NOT-REACHABLE
```

## 4. 一次配完 apt（换清华源 + apt 走代理 + 加 ROS 源）
```bash
#!/bin/bash
set -e
# apt 走 Clash 代理
sudo tee /etc/apt/apt.conf.d/proxy > /dev/null <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
EOF
# 换 Ubuntu 源为清华（兼容 sources.list 与 deb822 的 ubuntu.sources）
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak 2>/dev/null || true
sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak 2>/dev/null || true
sudo sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list 2>/dev/null || true
sudo sed -i 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list 2>/dev/null || true
sudo sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || true
sudo sed -i 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || true
# 加 ROS2 key + 源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "-- proxy:"; cat /etc/apt/apt.conf.d/proxy
echo "-- ros2.list:"; cat /etc/apt/sources.list.d/ros2.list
echo "done. 下一步: sudo apt update"
```

## 5. 内存调整（WSL 只默认拿一半内存）
`C:\Users\<u>\.wslconfig` 追加（保留已有 kernel 行）：
```ini
memory=12GB
processors=10
swap=4GB
```
改完 `wsl --shutdown` 重启；WSL 内 `free -h` 验证 `Mem total` 变 12Gi。
（物理 RAM 15.7GB 时合理；物理内存更小则按需下调。）

## 环境前提（本会话实测）
- Windows 11, WSL 2.7.12 + WSLg 1.0.73, i5-12500H/16GB/RTX3050 4GB。
- 用户 Clash 监听 `127.0.0.1:7890`；Ubuntu 24.04 WSL 发行版名 `lyuav`。
- WSL 分区/发行版名与默认不同时，一律 `wsl -d <名字>` 而不是裸 `wsl`。
