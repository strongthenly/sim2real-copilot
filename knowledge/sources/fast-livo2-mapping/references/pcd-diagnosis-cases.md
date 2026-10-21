# 实战案例：PCD 数据异常排查（2026-08）

## 案例 A：PCD 只有 6.6MB / 8 秒数据（用户飞了 2 分钟）
**症状**：建图 2 分钟，PCD 只有 6.6MB（21 万点）
**排查链**：
1. `lidar_poses.txt` 只有 80 行，时间范围 1786022862→1786022870 = 正好 8 秒
2. 日志 mtime 21:27:50 vs 启动 21:27:37 → **进程只活了 13 秒**
3. 但 feature 行从 39 行到 1542 行（约 1500 行 ≈ 2.5 分钟建图在跑）
4. **根因**：上一轮 KillLiDAR 的异步清理线程还在跑（pkill 阶段），用户点了启动，新进程 13 秒后被上一轮的 pkill 误杀
5. **修复**：启动路径 join 上一轮 bg_kill_lidar 线程 + kill_lidar_processes 加互斥锁

**教训**：PCD 小先判断"数据没采够"还是"没保存好"——看进程存活时间 vs PCD 点数是否匹配。

## 案例 B：PCD 60MB / 92 秒（时间够但点数少）
**症状**：进程活了 97 秒（正常），pose 925 行连续，但 PCD 只有 188 万点/60MB
**排查链**：
1. 每帧 feature 点数平均 4227（最小 1779）——正常应 12462（8月4日日志）
2. 运行时参数：`/publish/dense_map_en: False`、`/preprocess/point_filter_num: 3`
3. yaml：`MID360_Fly_Lidar.yaml` 里 `dense_map_en: true`、`point_filter_num: 1`
4. **矛盾**：运行时参数匹配 `MID360_Fly.yaml`（飞行模式）！
5. **根因**：start_fastlivo2.sh 第 25 行 `true` 分支误写 `CONFIG_NAME="MID360_Fly.yaml"`（注释写"高质量建图"但值是飞行配置）——git 里带进来的 bug
6. **修复**：改回 `MID360_Fly_Lidar.yaml`

**教训**：PCD 点数骤减优先查运行时参数（日志 PARAMETERS 段），与 yaml 对比；dense_map_en=false 是头号嫌疑。

## 案例 C：二次启动 20~30 秒慢
**症状**：关闭后立刻启动要等很久
**排查链**：
1. 日志：09:48:03 KillLiDAR → 09:48:26 启动 → join 等上一轮收尾
2. PCD mtime = KillLiDAR 同一秒（341MB 写盘几乎瞬间）→ **瓶颈不是写盘**
3. **根因**：关闭线程串行等待链——_kill_tracked_radar_groups 轮询等整个进程组消失（最长 90s，rviz/livox 拖后腿）+ pkill 动态等待 15s（匹配 livox 会等满）+ rosnode 清理
4. **修复**（纯 Python，无需改 C++）：只等 fastlivo_mapping 退出（=PCD 写完标志，最长 60s）→ 立即 SIGKILL 进程组；pkill 动态等待只匹配 fastlivo_mapping
5. 效果：二次启动 3~8 秒

**教训**：二次启动慢先看"关闭线程在等什么"，PCD 写盘在 NVMe 上几乎瞬间，别急着上缓存区/改 C++ 方案。

## 通用排查命令
```bash
# 每帧点数统计
grep -a 'Raw feature' <log> | sed 's/.*Raw feature num: //;s/,.*//' | awk '{sum+=$1;n++} END{print "帧数:",n," 平均:",sum/n}'
# 进程存活 vs PCD
stat -c '%y' <pcd>; grep -a 'process\[laserMapping' <log>
# 运行时参数
grep -a '/publish/dense_map_en\|/preprocess/point_filter_num\|/pcd_save' <log>
```
