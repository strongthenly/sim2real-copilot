# WSL 发行版迁移到非 C 盘 + AirSim/WSL 共存架构

## WSL 发行版迁移到 E 盘（用户偏好：大文件/仿真不占 C 盘）
场景：WSL 发行版（如 `lyuav`）的 `ext4.vhdx` 占满 C 盘（可达 18GB+，因装了 PX4/ros2_ws 等大工程）。用户要迁到 E 盘。

步骤（先导出确认完整，再注销/导入）：
```powershell
# 1) 创建目标目录 + 关闭所有 WSL（会关你当前跑着的仿真）
New-Item -ItemType Directory -Path "E:\wsl" -Force
wsl --shutdown

# 2) 导出（大：18GB 约几分钟，后台跑 + notify_on_complete）
wsl --export lyuav E:\wsl\lyuav.tar
#    tar 大小 ≈ 20-30% 小于 vhdx（因为不含 vhdx 稀疏保留）
#    Warning "bsdtar: ... cannot archive sockets"（如 ./tmp/px4-sock-0）——
#    只是 PX4 的运行时 socket 文件无法归档，非致命，socket 运行时重建，不影响数据完整性。EXIT=0 即成功。

# 3) 从 C 盘注销（删 C 盘原 vhdx，释放空间；tar 是完整备份可回退）
wsl --unregister lyuav

# 4) 导入到 E 盘
wsl --import lyuav E:\wsl\lyuav E:\wsl\lyuav.tar

# 5) 验证
wsl -l -v                 # lyuav 存在
wsl -d lyuav -- ls /home/<USER>   # PX4-Autopilot + ros2_ws 数据在 = 完整
Get-ChildItem "E:\wsl\lyuav" -Filter "*.vhdx"   # ext4.vhdx 在 E 盘
wsl -d lyuav -- bash -lc "whoami"   # import 后默认用户通常仍保留（liyan）
```
- 注意：PowerShell 里 `wsl ... -- bash -lc "echo $HOME"` 的 `$HOME`/`$(...)` 会被 PowerShell 先展开 → 报错。用**脚本文件**（写到 `/mnt/c/Users/<user>/x.sh` 再 `wsl -d <distro> -- bash /mnt/c/.../x.sh`）最稳。
- 保留 `E:\wsl\lyuav.tar` 作为回退备份（约 16GB，E 盘空间充足可留）；确认稳定后可删释放。

## WSL(ROS/算法) + Windows(Unreal/AirSim) 共存架构
- AirSim 基于 **Unreal Engine**，**只能 Windows 本机跑**（WSL 跑不了 Unreal 的 GUI）。这是一个合理的分层架构。
- **协同命门** = `networkingMode=mirrored`：WSL 的 `127.0.0.1` 与 Windows 的 `127.0.0.1` 互通 → WSL 里 `127.0.0.1:<AirSim RPC 端口 41451>` 能连 Windows 上跑的 AirSim。
- **AirSim 官方 ROS bridge**（`airsim_ros_pkgs`，含 ROS2 版）跑在 WSL：通过 RPC 连 Windows AirSim，把 AirSim 的 LiDAR/相机/位姿发布成 ROS 话题，也接收 ROS 命令控制车辆。
- **PX4 留 WSL**（已编译 + MAVROS 通了）：PX4 通过 MAVLink 指挥 AirSim 里的无人机（AirSim 支持 MAVLink 连 PX4）。
- 局限：Windows 需装 Unreal + 构建 AirSim 插件（几十 GB，好 GPU）；`实机完全复刻`需要真实环境/无人机模型/传感器参数素材；3050 mobile(4GB) 跑 Unreal 高保真很吃力。AirSim 仍属"高保真视觉"派，非探索论文主流（主流是 Gazebo+PX4+MAVROS）。
