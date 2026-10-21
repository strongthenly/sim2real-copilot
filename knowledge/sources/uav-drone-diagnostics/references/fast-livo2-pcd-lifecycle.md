# FAST-LIVO2 PCD 保存与启停生命周期（Jetson NX 实测）

## PCD 保存机制（LIVMapper.cpp）

- `savePCD()` 只在 `while(ros::ok())` 主循环退出后调用**一次**（run() 末尾，约 553 行）。
- **roscpp 只响应 SIGINT（Ctrl+C）**，不响应 SIGTERM。收到 SIGTERM 默认直接终止进程 → `savePCD()` 根本没机会执行；即使执行中，2~3 秒后的 SIGKILL 也会打断写盘 → PCD 丢失/半截损坏。
- 停止流程必须：发 **SIGINT** → 轮询等 fastlivo_mapping 进程自然退出 → 再处理其他进程。
- 配置（task_run_scripts/config/MID360_Fly_Lidar.yaml 的 pcd_save 段）：
  - `interval: -1` = 所有帧合并一个文件（用户偏好，不产生中间文件）
  - `interval: >0` = 每 N 帧落盘一个 `<时间戳>.pcd`（会产生大量中间文件，用户明确拒绝）
  - `img_en: 0`（无相机）→ savePCD 只写 raw（pcl_wait_save_intensity），无 downsampled 文件

## 文件名：从"固定名覆盖"改为"时间戳独立文件"

原状：`std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd"` 固定名，每轮建图互相覆盖（旧图丢失）。

改法（C++ 源头，脚本 rename 为冗余）：
1. `include/LIVMapper.h` 加成员：`std::string save_session_id_;`（放 `int img_en = 1, imu_int_frame = 3;` 附近）
2. 构造函数里（`nh.param<int>("common/img_en"...` 之前）初始化：
   ```cpp
   { char buf[32]; time_t t = time(nullptr); struct tm *tm_ = localtime(&t);
     strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", tm_);
     save_session_id_ = std::string(buf); }
   ```
3. savePCD() 文件名：`ROOT_DIR + "Log/pcd/" + save_session_id_ + ".pcd"`（downsampled 加 `_downsampled` 后缀）
4. 头文件需 `#include <ctime>`（string 已有）
5. 重编译：`cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=fast_livo -j2`（Jetson NX 约 10~20 分钟，后台跑 + notify_on_complete）

## 停止流程设计（nx_command_server.py，三优先级：零残留 > PCD 完整 > 响应灵敏）

- **KillLiDAR 异步化**：`threading.Thread(target=kill_lidar_processes, name="bg_kill_lidar", daemon=True).start()` → 界面立即响应。
- **致命坑（本会话实测踩中）**：后台 kill 线程用 `pkill -f` 模式匹配清理时，会**误杀关闭后新启动的进程**！现象：建图进程只活 13 秒（启动 21:27:37 → 保存 21:27:50），用户飞 2 分钟只存 8 秒数据。修复：
  1. 启动路径（`_handle_radar_launch` 的 "先关闭旧雷达" 处）先 join 上一轮后台线程：
     ```python
     for _t in threading.enumerate():
         if _t.name == "bg_kill_lidar" and _t.is_alive():
             _t.join(timeout=120)
     ```
  2. `kill_lidar_processes()` 加互斥锁（`acquire(blocking=False)`，有清理在跑则跳过），包装成 `_kill_lidar_processes_impl()` + 外层 try/finally release（不要整体缩进函数体——会 IndentationError）。
- SIGINT 替换所有对建图进程的 SIGTERM（killpg / pkill）；等待用轮询（进程组消失即提前退出），不要固定 sleep 白等。
- 保留 SIGKILL 兜底 + pkill -9 + rosnode cleanup → 死保零残留。

## roslaunch 陷阱

- **`<node timeout="120">` 是无效属性**！roslaunch 打印 `unknown <node> attribute 'timeout'` 并忽略。roslaunch 的 15 秒超时是节点**启动**超时，与关闭无关。不要用 launch 属性保护 PCD 写盘。

## PCD 完整性校验（脚本化，见 uav-drone-diagnostics 主 SKILL 或 scripts/check_pcd.py）

- 二进制 PCD：header 从文件头到 `DATA binary` 行结束；`expected = header_len + POINTS × sum(SIZE字段)`；`实际 == 理论` 差 0 B = 完整。
- 点数读取：header 里 `POINTS N` 行。

## 排查"PCD 太小 / 与路径不符"

按序取证（每步一条命令，别瞎猜）：
1. `ls -lat Log/pcd/*.pcd` → 文件大小、时间戳命名
2. `wc -l lidar_poses.txt` + 首尾时间戳（`head -1 | awk '{print $1}'`）→ **实际数据时长**（行数≈秒数 @10Hz）
3. 进程存活时长 = 日志 mtime − 日志里 `process[laserMapping-1]: started with pid` 对应启动时间
4. 日志 `grep -a 'Raw feature' | sed 's/.*Raw feature num: //;s/,.*//' | awk '{sum+=$1;n++} END{print sum/n}'` → 每帧点数：建图模式 ~12000+，飞行模式 ~2000-4000
5. **关键区分两类根因**：
   - **a) 进程被提前杀**（存活 13s vs 用户飞 2min）→ 查异步清理线程误杀新进程（见上"致命坑"）
   - **b) 每帧点数系统性偏少（全程 2000-4000 且稳定）** → **先查配置模式，别归因环境**！核对运行时参数与配置文件：
     ```bash
     grep -a '/publish/dense_map_en\|/preprocess/point_filter_num' <fastlivo.log>
     grep -n 'dense_map_en\|point_filter_num' task_run_scripts/config/MID360_Fly_Lidar.yaml task_run_scripts/config/MID360_Fly.yaml
     ```
     建图模式 = `dense_map_en: true` + `point_filter_num: 1`（全量点）；飞行模式 = `dense_map_en: false` + `point_filter_num: 3`（降采样+抽稀，点数少 5~10 倍）。
   - **配置模式被切错的真实案例（2026-08-06 实测）**：用户飞 2 分钟 PCD 仅 60MB（188 万点，每帧 ~4200）。日志运行时参数 `dense_map_en: False` + `point_filter_num: 3` 完全匹配 `MID360_Fly.yaml`，而用户要的是 `MID360_Fly_Lidar.yaml`。根因：`start_fastlivo2.sh` 第 25 行 `if [ "$WITH_CLOUD" = "true" ]; then CONFIG_NAME="MID360_Fly.yaml"` —— **true 分支被 git 提交写错成飞行配置**（注释仍写"🗺️ 高质量建图"但值是 Fly.yaml），正常应是 `MID360_Fly_Lidar.yaml`。一行改回即可（备份 + `bash -n` 校验）。排查经验：**PCD 点数异常先对"运行时参数 vs 配置文件"，能立刻定位是不是加载错了配置**；grep 前先 `sed -n '20,35p' start_fastlivo2.sh` 看模式选择逻辑，别假设它是对的（git 提交可能带 bug）。
6. 环境空旷/遮挡确实也会导致每帧点数低，但**那是逐帧波动**（如 2681→4387→2524 起伏），与"全程稳定 2000-4000"的配置错误特征不同——用 `awk 'NR%100==1{print NR": "$1}'` 采样分布区分。

## 二次启动等待的实测量化（2026-08-07，配置修复后 + 真瓶颈诊断）

配置修复（建图模式全量点）后 PCD 恢复 341MB，用户反馈二次启动等待 20~30s（体感）。**先诊断再选方案**：

- **关键证据：PCD 341MB 的 mtime = KillLiDAR 指令同一秒**（`stat -c '%y' Log/pcd/*.pcd` vs server 日志 `收到 QGC 指令: KillLiDAR` 时间戳）→ **写盘几乎瞬间完成**（NVMe 上 writeBinary 只要几秒），"等 PCD 写完"不是瓶颈。
- **真瓶颈 = 关闭线程串行等待链**：`_kill_tracked_radar_groups()` 轮询等**整个进程组**消失（最长 90s，rviz/livox 等不响应 SIGINT 的进程拖后腿）→ pkill → 动态等待 15s → rosnode 清理，**全部串行跑完**；启动线程 `join` 等它全部结束才启动。
- 时间线证据：`nx_command_server2.log` 里 `收到 KillLiDAR`（09:48:03）→ `收到 Odin-MindCloud`（09:48:26，用户 23s 后点启动）→ join 等上一轮收尾 → `SIGKILL 进程组` + `跳过重复清理` → 新进程启动。

**结论（推翻早前"无法优化"的假设）**：**不需要缓存区、不需要改 C++、不需要重编译**——只要把关闭线程改成"只等 `fastlivo_mapping` 进程退出（=PCD 写完，几秒）→ 立即 SIGKILL 其他所有（livox/rviz/roslaunch）"，二次启动即可从 20~30s 降到 3~8s，纯 Python 半小时工作量。**方法论：先量化每段耗时（mtime vs 指令时间戳差值、日志阶段时间），确认瓶颈在哪一步再选方案——缓存区/改 C++ 是最后手段，别默认需要**。诊断命令：`stat -c '%y' Log/pcd/*.pcd` 对比 server 日志指令时间戳；`grep -a '收到 QGC 指令\|SIGKILL 进程组\|清理完毕' log/*/nx_command_server2.log` 重建时间线。
- 不要为了快而删 SIGKILL 兜底或设写盘硬上限（大 PCD 必报废，用户一票否决过）

## 配置模式与飞行安全（树林场景迫降事故，2026-08-07）

| 配置 | dense_map_en | point_filter_num | lio.max_iterations | voxel_size | max_points_num | half_map_size | 算力 | 用途 |
|---|---|---|---|---|---|---|---|---|
| MID360_Fly_Lidar.yaml | true（全量点云） | 1（不抽稀） | 5 | 0.5m | 50 | 100m | ~2 倍 | **仅专门建图（不飞）** |
| MID360_Fly.yaml | false（降采样） | 3（抽稀3倍） | 3 | 1.0m | 20 | 50m | 低 | **飞行必须用这个** |

- **树林等复杂场景用 Fly_Lidar → 位姿延迟高（算力 2 倍）→ PX4 定点漂移 → 迫降（真实事故）**。用户已把 `start_fastlivo2.sh` 的 MODE 分支（$2 参数）改为 lidar 也指向 Fly.yaml——飞行安全优先于 PCD 密度，这是用户拍板的安全决策，别当成 bug 改回去。
- 注意：默认路径（`start_fastlivo2.sh true` 无 $2）走第 25 行 CONFIG_NAME——若该路径也用于飞行，需确认是 Fly.yaml；PCD 稀疏是飞行模式的固有代价（dense_map_en=false 攒的是 feats_down_body），要完整地图就单独跑专门建图任务。

## 回退保障机制（用户明确要求"我要保证我有一个后路"，2026-08-06 建立）

任何重大修改实施后，主动建立 `~/rollback_snapshots/` 快照目录 + 一键回退脚本：

```
rollback_snapshots/
├── rollback.sh          # bash rollback.sh planB|original|current
├── planB/               # 方案B：nx_command_server.py + start_fastlivo2.sh + LIVMapper.cpp/.h（原始版）
├── original/            # 8月2日最原始版：nx_command_server.py + start_fastlivo2.sh
└── current/             # 当前版（全部修复）：4 个文件
```

- rollback.sh 核心：`cp $SRC/<file> <目标路径>` 恢复，检测到 LIVMapper 则提示重编译 + 重启服务命令。
- **用户心理**："后路" = 随时能回到已知可用状态，不是要现在回退。建好机制后继续推进新方案即可；实施大改动前先确认快照齐全，并明确告诉用户"后路在哪、一条命令怎么回"。
- 用户会问"大概需要多长时间"：给分阶段时间表（改 C++ / 写转换器 / 集成 / 重编译 / 联调各多少分钟）+ 总计 + 说明哪些是等待时间（编译后台跑）、哪些需用户配合（实测），可提供分期选项。

## 任务队列设计（用户提出的缓存区增强，2026-08-06 评估）

连续快速"关→开→关→开"会产生多个缓存文件待转换，转换器按**文件名时间戳 FIFO** 排队：

```
缓存目录 Log/pcd_cache/：
  20260807_094656.raw  → .converting → 转完移到 Log/pcd/ → 删缓存
  20260807_095201.raw  （排队）
状态：.raw(待转) / .converting(转换中) / .failed(失败不阻塞队列)
恢复：转换器重启后扫描目录，.raw 继续转（断点续转）
```

**澄清过的物理事实**：缓存区方案下旧进程快照后**立即退出**（内存+雷达释放），所以"两个建图进程同时占内存 OOM"不会发生——实际是"旧图收尾写盘与新图建图重叠"，不是两个完整建图并行（雷达仍独占）。用户担心 OOM/雷达独占时，用这个澄清打消顾虑。

## 服务重启（加载脚本改动）

`start_qgc_servers.sh` 管 roscore + nx_touch_server2 + nx_command_server2 + rviz_full_stream2：
- `kill -TERM <start_qgc_servers 父PID>` → 触发脚本 cleanup trap → 等 4s
- 残留用**精确 PID** `kill -9`（不要 pkill -f 模式——命令行含模式串会自杀）
- 重启：`export DISPLAY=:0; setsid bash .../start_qgc_servers.sh > /tmp/qgc_restart.log 2>&1 < /dev/null &`，等 14s 验证
