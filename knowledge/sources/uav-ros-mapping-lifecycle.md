---
name: uav-ros-mapping-lifecycle
description: 运维 Jetson NX 机载电脑上的 FAST-LIVO2 建图生命周期——启动/关闭流程、PCD 保存机制（SIGINT 语义、异步关闭、防误杀）、建图/飞行配置模式选择（树林等重负载场景防漂移迫降）、PCD 完整性验证与着色。Use when FAST-LIVO2 的 PCD 丢失/损坏/点数少、关闭或二次启动慢/卡、进程残留、飞行中定位漂移。
---

# UAV ROS 建图生命周期运维（FAST-LIVO2 on Jetson NX）

## When to use
- FAST-LIVO2 关闭后 PCD 丢失 / 损坏 / 点数异常少（如 2 分钟飞行只有 60MB）
- 关闭建图或二次启动很慢 / 卡死 / 界面无响应
- 关闭后有进程残留（laserMapping / livox / rviz 没清干净）
- 飞行中位姿漂移导致定点模式失控 / 迫降（配置模式问题）
- 需要改建图启动/关闭脚本，或回退相关改动
- 解析/讲解部署版 NX_code 架构（Cython .so 安全打包）、改雷达/按钮配置、合并代码（开发机 git + CI deploy 分支）
- **遥控器「虚拟投影 / 图传黑屏 / 等待视频信号接入 <NET_RC>.14:6000」** → `references/remote-controller-video-stream-diagnosis.md`
  - ★6000 是**私有二进制推流协议**（`<len><JPEG>` + 先发 `AUTH:<token>`），不是 RTSP/MJPEG/HTTP
  - ★`curl :6000` 报 `Received HTTP/0.9 when not allowed` = **端口活着**，不是服务故障，别顺着它查网络
  - ★机器上两份 `start_qgc_servers.sh`：`~/NX_code/` 是旧版且**推流行被注释掉**，正式版在 `~/AirCore/`
  - ★别默认用户报的 IP 就是设备 IP：先 `arp -a | grep 192.168.144`，`<incomplete>` = 该 IP 根本不存在

## 系统架构（fouruav NX，Ubuntu 20.04 + ROS Noetic）
- **部署形态（2026-08-21 实测）**：机载 NX_code 已是 **Cython 安全部署版**——核心 `nx_command_server.py` 源码被编译成 `nx_command_server.cpython-38-aarch64-linux-gnu.so`，部署机**无源码、无 .git**（`git status` 报"不是 git 仓库"）。`nx_command_server2.py` 只是 5 行 import .so 的包装器（`*_launcher.py` 同义）。**改逻辑必须回开发机源码仓库**（README：CI `push master` → `build_secure.sh` 编译 → 发布 deploy 分支 → 拷贝回 NX → 重启服务）。部署机上能改的明文面：`Radar_Launcher.yaml`、`User_define_commands.yaml`、`config/*.yaml`、`launch/*.launch`、`start_fastlivo2.sh`。详见 references/secure-deploy-radar-chain.md。
- **雷达启动链路（遥控器/平板 QGC 按钮）**：QGC 按钮 → UDP 6003 JSON → .so 读 `scripts/NX_code/Radar_Launcher.yaml`（`active_profile: odin|mid360`，环境变量 `LIYUMEN_RADAR_PROFILE` 可覆盖）→ 取 `launch_with_cloud`（true=录数据建图）/`launch_without_cloud`（false=仅飞行）的 `command` → 执行 `bash start_fastlivo2.sh true|false` → 按序拉起 6 组节点（rs_d435 相机 → msg_MID360s 雷达驱动 → mid360_imu_tf → rviz 提前弹窗 → `mapping_MID360.launch`（`pcd_save`+`config_name`）→ `px4_and_others.launch`（px4+realman+TF））。关闭按钮走同 yaml 的 `cleanup` 规则（`pkill_patterns`+`rosnode_patterns`）。.so 内有 `global_radar_start_message/result/time/starting` 异步状态机。
- **用户自定义按钮**：`scripts/NX_code/User_define_commands.yaml` 的 UserCmd1/2/3——平板显示 `name`，机载执行 `command`（当前：UserCmd1=SUPER航点任务、UserCmd2=停止SUPER先切POSCTL、UserCmd3=测试占位）。
- **SSH 手动路径**：`task_run_scripts/scripts/NX_code/start_fastlivo2.sh`（带 cleanup trap）。
- **部署版侦察技巧（无源码时）**：`strings nx_command_server.*.so | grep -iE 'Radar_Launcher|UserCmd|6003|LIYUMEN|global_radar|launch_with'` 可挖出配置面/环境变量/端口/状态机字段。
- **配置**：`task_run_scripts/config/MID360_Fly.yaml`（飞行）/ `MID360_Fly_Lidar.yaml`（建图）。launch 用 `config_name` 参数加载。
- **PCD 输出**：`FAST-LIVO2/Log/pcd/`，路径由 CMakeLists 的 `ROOT_DIR` 编译期写死。
- **服务重启（⚠️ 旧写法已废弃）**：~~`pkill -f start_qgc_servers.sh` + 重新 `bash start_qgc_servers.sh`~~ ——2026-08-21 实测不可靠：脚本进程可能早已死（服务是 nohup 孤儿挂 systemd 名下），且 GNOME autostart 会自动复活服务。正确流程见下"服务进程排障"节（先 mv autostart 为 .off 再按精确进程名杀）。

## 核心机制：PCD 保存的 SIGINT 语义
- **roscpp 只响应 SIGINT（Ctrl+C），不响应 SIGTERM**——`kill -TERM` 会让 fastlivo_mapping 直接暴毙，`savePCD()` 根本没机会执行。
- `savePCD()` 只在 `while(ros::ok())` 主循环退出后执行一次（LIVMapper.cpp 的 run() 末尾），且条件是 `pcd_save_en && interval < 0`。
- **正确关闭信号链**：SIGINT → ros::ok() 变 false → 主循环退出 → savePCD 写盘 → 进程退出。
- **PCD 写盘很快**（NVMe 上 341MB 几乎瞬间），"关闭慢"的瓶颈通常在**清理等待链**（等整个进程组消失、等 livox 优雅退出），不是写盘本身——诊断时先看日志 mtime vs 指令时刻，别误判成写盘慢。

## 关闭/启动流程设计（nx_command_server.py，2026-08 优化后的状态）
1. **KillLiDAR 异步化**：丢进 `bg_kill_lidar` daemon 线程，UDP 主循环立即返回，界面不卡。
2. **启动也异步化**：`bg_start_radar` 线程，主循环永不阻塞（同步 join 会卡死 UDP 循环 → 界面假死）。
3. **防误杀**：启动前 `join(bg_kill_lidar)` 等上一轮关闭线程彻底结束——否则上一轮清理的 `pkill` 会匹配到刚启动的新进程，把它 13 秒内杀掉（PCD 只有 8 秒数据）。
4. **互斥锁**：`_kill_lidar_lock` 保证同一时间只有一个清理流程（装饰器包装 `_kill_lidar_processes_impl`）。
5. **只等关键进程**：关闭时只轮询 `pgrep -f fastlivo_mapping` 退出（= PCD 写完标志，最长 60s），**不等整个进程组**（rviz/livox 不响应 SIGINT 会拖到 90s）→ 二次启动从 20~30s 降到 3~8s。
6. **双保险**：killpg SIGINT 后再 `pkill -INT -f fastlivo_mapping`（防 roslaunch 没转发信号 → PCD 必保存）。
7. **零残留三重保险**：SIGKILL 进程组 + pkill -9 全模式兜底 + rosnode cleanup / 启动前 pgrep 验证。

## 配置模式（飞行安全关键！）
| 参数 | MID360_Fly.yaml（飞行） | MID360_Fly_Lidar.yaml（建图） |
|---|---|---|
| dense_map_en | false（降采样点） | true（全量点云） |
| point_filter_num | 3（抽稀 3 倍） | 1（不抽稀） |
| lio.max_iterations | 3 | 5 |
| lio.voxel_size | 1.0 | 0.5 |
| 算力 | 低（保实时） | ≈2 倍（精度优先） |

- **树林等点云密集场景用 Fly_Lidar 会算力吃满 → 建图延迟高 → PX4 定点模式漂移 → 迫降**。飞行必须用 Fly.yaml；完整地图单独跑建图任务（不飞时慢扫）。
- `start_fastlivo2.sh` 的 `true` 分支决定默认配置；`$2` 参数（lidar/fly）可覆盖。**历史坑**：git 里曾把 `true` 分支误写成 `MID360_Fly.yaml`（注释写"高质量建图"但值错）→ PCD 点数骤减（2 分钟只有 60MB，dense_map_en=false + 抽稀 3 倍）。改配置后对照运行时 `/publish/dense_map_en` 与配置文件是否一致。

## PCD 完整性验证
```bash
python3 task_run_scripts/scripts/check_pcd.py
```
原理：PCD 头部 `POINTS=N` × 每点字节数 + header 长度 = 文件实际大小，差 0 B = 完整。字段 `x y z intensity normal_x normal_y normal_z curvature` = 32B/点；`x y z rgb` = 16B/点。

## PCD 着色（CloudCompare 打开全是蓝色）
- 原因：FAST-LIVO2 输出的 PCD 字段只有 `x y z intensity normal_* curvature`，**没有 RGB** → CloudCompare 默认蓝色。
- 解法：`python3 task_run_scripts/scripts/colorize_pcd.py <in.pcd>` → 生成 `<in>_color.pcd`（XYZI→XYZRGB，按高度 Z jet 色图）。转成 XYZI→XYZRGB 结构体：`struct.pack('<fffI', x, y, z, rgb)`，rgb = (r<<16)|(g<<8)|b。
- 或 CloudCompare 内即时着色：`Scalar field → intensity/z → Set color from scalar field`（不生成新文件）。

## 回退保障
`~/rollback_snapshots/rollback.sh planB|original|current` — 一键恢复到方案B / 最原始版 / 当前版。快照含 nx_command_server.py、start_fastlivo2.sh、LIVMapper.cpp/h。回退到 planB/original 需重编译（`catkin_make -DCATKIN_WHITELIST_PACKAGES=fast_livo -j2`）+ 重启服务。

## 诊断要点（PCD 数据量异常的区分）
- **PCD 时间短**（如 2 分钟飞行只有 8 秒数据）→ 进程被提前杀（异步清理误杀 / 手动强杀）→ 检查进程实际存活时长（日志 mtime vs 启动时刻）。
- **PCD 时间长但文件小**（92 秒只有 60MB）→ 配置错用飞行模式（每帧点数 2-4 千 vs 正常 1 万+）→ 检查 dense_map_en / point_filter_num 运行时值。
- **文件损坏**（头声明点数对不上）→ 写入被 SIGKILL 打断 → 检查关闭是否用了 SIGINT + 等待。

## 服务进程排障（pkill 不起效 ≠ pkill 坏了）
- **症状**：`pkill -f start_qgc_servers.sh` 无任何效果，服务（roscore/三微服务）还在跑。
- **根因（2026-08-21 实测）**：脚本进程**早已不存在**（可能被前一次 pkill/SIGKILL 干掉），而 roscore/微服务是 nohup 子进程，死掉后被 reparent 到 `systemd --user`（`ps -o pid,ppid,cmd` 看 PPID 是 systemd 而非脚本 bash）→ 变成孤儿继续跑。此时 pkill 脚本名无目标，返回码 1，天然"不起效"。
- **判据三连**：① `pgrep -af start_qgc_servers` 只匹配到执行命令的 shell 自己（pkill -f 按完整命令行匹配，**会匹配到正在执行它的 bash -c**——ssh 里跑会自杀掉线）；② 微服务 PPID 是 systemd --user；③ `log/latest/start_qgc_servers.log` 无 "stopping all modules..."（trap 从未执行）。
- **正确停服务**：按**精确进程名**杀微服务，别杀脚本名：
  ```bash
  pkill -f nx_command_server2.py; pkill -f nx_touch_server2.py; pkill -f rviz_full_stream2.py; pkill -f roscore
  ```
  然后 `bash ~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_qgc_servers.sh` 重启（预期：新 `log/<时间戳>/` 出现，start_qgc_servers.log 打印 4 个 pid）。
- **教训**：`pkill -f` 的匹配目标是"当前存在的进程"，先 `pgrep -af` 确认目标存在再杀；pkill 模式用精确进程名（如 `nx_command_server2.py`），不用脚本名。
- **GNOME autostart 自动复活机制（治本关键，2026-08-21 实测）**：`~/.config/autostart/qgc_servers.desktop` 里的应用被 GNOME 会话监控，**进程退出后会自动重新拉起**——表现：杀了服务几秒后新进程 + 新日志目录又出现（实测 16:11:39 自动拉起第二套），遥控器画面不断，误以为 pkill 没作用。**要彻底停服务必须先移走自启入口再杀**：
  ```bash
  mv ~/.config/autostart/qgc_servers.desktop ~/.config/autostart/qgc_servers.desktop.off
  pkill -9 -f 'rviz_full_stream[2]\.py'; pkill -9 -f 'nx_command_server[2]\.py'; pkill -9 -f 'nx_touch_server[2]\.py'; pkill -9 -f 'roscor[e]'
  ```
  恢复：`mv ~/.config/autostart/qgc_servers.desktop.off ~/.config/autostart/qgc_servers.desktop` + `bash .../start_qgc_servers.sh`。
- **pkill -f 自杀坑增强版（远程 plink/bash -c 里必踩）**：`pkill -f xxx` 按完整命令行匹配，**执行命令的 shell（bash -c 整串）里任何位置出现目标子串都会自杀**——包括 grep -E 显示模式、echo 文本、其他命令参数。方括号技巧 `'roscor[e]'` 只能防"pkill 模式自身"，**防不住同命令串里的 grep 模式**（实测：`pkill -f 'roscor[e]'` 因同串 `grep -E '...roscore...'` 含 roscore 而自杀断连，exit 128）。安全姿势：① 命令整串（含 grep 显示模式）都避开目标串；② 或把 pkill 写进 /tmp 脚本再 `bash /tmp/kill.sh` 执行（脚本进程命令行是脚本路径，不匹配任何模式）。
