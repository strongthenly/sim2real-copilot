# FAST-LIVO2 关停流程设计记录（2026-08-05）

用户验收标准（优先级从高到低，做任何关停方案前先复述）：
1. **进程零残留 = 一票否决**（"这个现象绝对不能够存在，即使保存不上也不能残留节点"）
2. PCD 保存完整
3. 启动/关闭响应灵敏（"其次"）

## 时间线（为什么一路从"有界等待"演进到"异步关闭"）

| 方案 | 总耗时 | 用户反应 |
|---|---|---|
| 原始版（SIGTERM→2~3s→SIGKILL） | 2~3s | PCD 必坏（最初的问题） |
| 修复版（SIGINT→等90~120s） | 90~120s | 功能成功但"观感不可接受，总以为是卡住了"→ 回退 |
| 有界等待（SIGINT→15s+8s→KILL） | 15~25s | 被否："超过30秒 PCD 直接白录报废" |
| **方案 B（异步关闭，已落地 2026-08-05 下午，服务已重启生效）** | 界面<1s，后台不限时 | 用户拍板：PCD 要时间戳命名不覆盖历史 |

## 方案 B 实施细节（当前 NX 生效状态，备份 `.bak.planB.20260805_171029`）

### `start_fastlivo2.sh` cleanup()（SSH 手动路径）
```bash
kill -INT "$FASTLIVO_PID"                       # ① SIGINT 触发 savePCD
pkill -INT -f "fastlivo_mapping"                #    双保险（子节点独立 PGID）
for _ in $(seq 1 120); do pgrep -f "fastlivo_mapping" || break; sleep 1; done  # ② 等写完
PCD_DIR="/home/fouruav/catkin_ws/src/FAST-LIVO2/Log/pcd"
if [ -f "$PCD_DIR/all_raw_points.pcd" ]; then
    mv "$PCD_DIR/all_raw_points.pcd" "$PCD_DIR/$(date +%Y%m%d_%H%M%S).pcd"   # ③ rename 时间戳
fi
kill -TERM "$PX4_PID" "$RVIZ_PID" "$IMU_PID" "$LIVOX_PID" "$RS_PID"         # ④ 其余进程
# 短等 → SIGKILL + pkill -KILL fastlivo_mapping 兜底
```

### `nx_command_server.py`（地面站路径，5 处）
1. `_kill_tracked_radar_groups()`：`os.killpg(pgid, signal.SIGTERM)` → `SIGINT`；`sleep(2.0)` → 轮询 `os.killpg(pgid, 0)` 探测进程组消失（最长 90s）
2. `kill_lidar_processes()`：`pkill -SIGTERM` → `pkill -SIGINT`；`sleep(1.5)` → `sleep(15)`
3. 尾部（`_rosnode_cleanup_zombies()` 后、`清理完毕` 打印前）新增 rename：
   ```python
   new_name = time.strftime("%Y%m%d_%H%M%S") + ".pcd"   # 可读时间戳
   # 同秒冲突加 _1/_2 后缀；os.rename 原子操作；文件不存在/空则打 ⚠️
   ```
4. KillLiDAR 异步化（核心）：
   ```python
   elif cmd == "KillLiDAR":
       if not any(t.name == "bg_kill_lidar" and t.is_alive() for t in threading.enumerate()):
           threading.Thread(target=kill_lidar_processes, name="bg_kill_lidar", daemon=True).start()
           print("⏳ 关闭指令已提交，后台保存 PCD 中...", flush=True)
       else:
           print("⏳ 上一轮关闭仍在后台进行中，忽略重复指令", flush=True)
   ```
5. 打印行 `-> SIGTERM 进程组` 未同步改为 SIGINT——纯显示 bug，看代码才是真值

### 关键设计点
- 文件已 `import threading`，有 3 处 `threading.Thread(..., daemon=True).start()` 先例（file-transfer / zip / delete worker）
- **不设保存上限**（用户否决硬上限：大 PCD 会报废）——后台线程等到写完，SIGKILL 只是兜底
- 命名冲突处理：同秒多次 rename 时 `while os.path.exists(new_path)` 加 `_idx` 后缀
- 重启服务生效：`pkill -f nx_command_server2.py` + 精确 PID 杀残留 + `setsid bash start_qgc_servers.sh > /tmp/qgc_restart.log 2>&1 < /dev/null &`（重启后 `ls -td log/20* | head -1` 确认新日志目录，`tail nx_command_server2.log` 看 `已开启 (HTTP 6004)`）

## 残留 vs 后台保存（用户话术，用户问过"这不就是残留吗"）

残留 = 没人管的孤儿进程，无限期存在，下次启动冲突。
后台保存 = 有人盯着、到点必走（写完自退 / 超时强杀 / 启动前兜底清理，三重保险）。
判据以 pgrep 进程存在为准；`rosnode list` 里的 `/rviz_<epoch>` 幽灵名字是 SIGKILL 来不及注销，无害（`rosnode cleanup` 可清）。

## 写盘速度真相（先测再设计）

- `img_en=0`（无相机）→ savePCD 最简路径：一次 `writeBinary`，无降采样，419MB≈2~5s
- 感知的"90s 慢" = 轮询等待 + roslaunch 生命周期 + 清理固定开销，不是写文件
- 实测方法：日志打 `⏳ 开始保存` / `✅ 完成（耗时 X 秒）` 时间戳
- 若实测 ≤10s：异步方案就够，无需缓存区

## 中间缓存区构想（用户提的，评估结论）

"快照 + 异步转换"：dump 到 /tmp 秒级 → 进程立即退出 → 后台转换器写标准 PCD。
- 障碍1 PCD 文件冲突：新旧进程写同一 `all_raw_points.pcd` 必坏 → 每次启动独立文件 `Log/pcd/<时间戳>/`（顺带解决覆盖无备份）
- 障碍2 雷达独占：保存阶段不需要雷达，可提前杀 livox driver；但新建图需要雷达 → 旧进程必须快速退出
- 障碍3 NX 算力竞争：旧写盘抢 CPU/内存 → 新建图掉帧/OOM，硬件限制绕不开 → 双跑时启动明确警告用户
- 代价：改 C++（savePCD 两段式）+ 重编译 + 转换脚本 → 最后手段

## 相关文件位置备忘

- PCD 输出：`CMakeLists.txt:40` `ROOT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/` → `~/catkin_ws/src/FAST-LIVO2/Log/pcd/all_raw_points.pcd`（方案 B 后：关闭时 rename 为 `Log/pcd/YYYYMMDD_HHMMSS.pcd`，不覆盖历史）
- 配置：`task_run_scripts/config/MID360_Fly_Lidar.yaml` 的 `pcd_save:` 段（不是 FAST-LIVO2/config/ 下的同名文件）
- 服务入口：`nx_command_server2.py`（317B 包装）→ `nx_command_server.py` 真逻辑
- 回退存档：`.bak.20260804_123240`（原始）/ `.fix.20260805_1640`（修复版）/ `.bak.planB.20260805_171029`（方案 B 前）

