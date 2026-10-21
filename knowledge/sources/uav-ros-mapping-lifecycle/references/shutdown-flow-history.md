# FAST-LIVO2 关闭/启动流程修复历程（2026-08，fouruav NX）

## 问题链条与对应修复

### 问题 1：PCD 丢失/损坏（最初状态）
- **现象**：关闭建图后 PCD 丢失或写一半损坏。
- **根因**：关闭脚本发 SIGTERM（roscpp 不响应）→ 2~3 秒后 SIGKILL → savePCD() 没机会执行或写一半被杀。
- **修复**：关闭改发 SIGINT + 等待窗口 + SIGKILL 兜底。

### 问题 2：异步关闭线程误杀新进程（PCD 只有 8 秒数据）
- **现象**：飞了 2 分钟，PCD 只有 6.6MB（≈8 秒数据），进程实际只活了 13 秒。
- **根因**：KillLiDAR 异步化后，上一轮 `bg_kill_lidar` 清理线程还在跑，其 `pkill` 匹配到**刚启动的新进程**并杀掉——清理线程没有"代际"概念。
- **修复**：启动路径 `join(bg_kill_lidar)` 等上一轮彻底结束 + `_kill_lidar_lock` 互斥锁。

### 问题 3：PCD 点数少（92 秒只有 60MB）
- **现象**：数据时间对（92 秒连续），但每帧点数只有 2-4 千（正常 1 万+）。
- **根因**：`start_fastlivo2.sh` 第 25 行 `true` 分支误用 `MID360_Fly.yaml`（dense_map_en=false 降采样 + point_filter_num=3 抽稀）——git 提交里带进来的 bug，注释写"高质量建图"但值错。
- **修复**：改回 `MID360_Fly_Lidar.yaml`。

### 问题 4：二次启动慢（20~30 秒）
- **现象**：关闭后立刻启动要等很久。
- **根因**：诊断发现 PCD 写盘其实很快（341MB mtime = KillLiDAR 同一秒），瓶颈在关闭线程的**等待链**：等整个进程组消失（90s，rviz/livox 不响应 SIGINT 拖后腿）+ 动态等待匹配 livox（拖满 15s）。
- **修复**：只等 `fastlivo_mapping` 退出（= PCD 写完标志，最长 60s）+ 动态等待只匹配 fastlivo_mapping → 3~8 秒。

### 问题 5：启动卡死（界面无响应）
- **现象**：点启动后界面完全卡死。
- **根因**：启动路径的 `join(timeout=120)` 同步阻塞在 UDP 指令接收主循环里，期间所有指令（含心跳）都不处理。
- **修复**：启动也异步化（`bg_start_radar` 线程），主循环永不阻塞。

### 问题 6（隐患）：roslaunch 可能不转发 SIGINT
- **风险**：killpg SIGINT 依赖 roslaunch 转发给 fastlivo_mapping，若 roslaunch 异常则 PCD 不保存，等满 60s 被 SIGKILL 全丢。
- **修复**：killpg 后追加 `pkill -INT -f fastlivo_mapping` 双保险（毫秒级，不增加等待）。

## 关键诊断方法
- **区分"数据没采够" vs "没保存好"**：看日志 mtime vs 指令时刻 + pose 文件时间范围（`lidar_poses.txt` 首尾时间戳）判断进程实际存活时长。
- **区分"配置问题" vs "保存 bug"**：`grep '/publish/dense_map_en\|/preprocess/point_filter_num' fastlivo.log` 与配置文件对比。
- **每帧点数参考值**：正常建图模式 ~1 万+ /帧，飞行模式 ~2-4 千 /帧。

## 服务重启要点
- 改 nx_command_server.py 后必须重启 `nx_command_server2.py` 才生效（Python 进程启动时加载代码）。
- 重启：`kill -TERM <start_qgc_servers pid>` → 精确 PID 强杀残留 → `setsid bash start_qgc_servers.sh`（DISPLAY=:0）。
- 强杀残留时**别用 pkill -f 匹配脚本自身内容**（会自杀，exit 128）——用精确 PID。
