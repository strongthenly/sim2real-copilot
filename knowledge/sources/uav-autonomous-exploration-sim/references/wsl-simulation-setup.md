# 在 Windows(WSL2)上搭建 UAV 仿真平台 —— 诊断与地基

在 Windows 11 上用 WSL2 跑 ROS2 + Gazebo,先做一次只读体检,再建新发行版、调内存。下列命令均在 **PowerShell** 里跑。

## 1. 一次性体检(只读,无副作用)

```powershell
# WSL 版本 / WSLg
wsl --version
wsl --status
# 已装发行版(列出版本号)
wsl -l -v
# 主机 GPU(看有无 NVIDIA 独显)
(Get-CimInstance Win32_VideoController).Name
# 主机物理内存 + CPU
$cs = Get-CimInstance Win32_ComputerSystem
(Get-CimInstance Win32_Processor | Select-Object -First 1).Name
"RAM: $([math]::Round($cs.TotalPhysicalMemory/1GB,1)) GB"
```

读 WSL 输出若乱码,见 `powershell-automation` 技能第 7 条(UTF-16LE)。

## 2. 确认 WSL 内 GPU 直通 + 资源(关键)

```powershell
wsl.exe -d <发行版> -- bash -lc "echo '=== OS ==='; cat /etc/os-release | grep -E 'VERSION_ID|PRETTY_NAME'; echo '=== kernel ==='; uname -r; echo '=== nvidia-smi ==='; nvidia-smi | head -8; echo '=== mem ==='; free -h | head -2; echo '=== disk ==='; df -h / | tail -1"
```

判读要点:
- **`nvidia-smi` 能列出 GPU** = CUDA/GPU 直通成功 → Gazebo 3D 渲染、CUDA(深度学习探索策略)可用。
- `VERSION_ID` 若是很新的 LTS(如 26.04)→ 太新,见第 3 步换 24.04。
- WSL 拿到的内存(`free -h` total)往往远小于物理内存,需用 `.wslconfig` 调大。

## 3. 新建专门仿真的 Ubuntu 24.04 发行版

默认 WSL 发行版常是最新 LTS(如 26.04),ROS2/Gazebo 包适配不成熟,别用它。建一个 24.04:

```powershell
wsl --install -d Ubuntu-24.04 --name uav24
wsl -d uav24 -- bash -lc "cat /etc/os-release | grep PRETTY_NAME"   # 确认是 24.04
```

## 4. 调大 WSL 内存/CPU(编辑 `$env:USERPROFILE\.wslconfig`)

```powershell
notepad "$env:USERPROFILE\.wslconfig"
```

```ini
[wsl2]
memory=12GB
processors=10
swap=4GB
```

保存后 `wsl --shutdown` 重启生效。`memory` 别超过物理内存**减 ~4GB**(给 Windows 留);`processors` 留几个给 Windows。

## 5. 实测样例(本机,可直接对照)

WSL2 2.7.12 + WSLg 1.0.73;GPU 直通 `nvidia-smi` 列出 **RTX 3050 / CUDA 12.3**;物理内存 15.7GB、WSL 默认只拿 7.6GB → 用 `.wslconfig` 调到 12GB;磁盘 ~1TB 空闲;CPU i5-12500H(12核/16线程)。默认发行版为 **Ubuntu 26.04 LTS**(太新,应新建 24.04)。
