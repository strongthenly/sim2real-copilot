# FAST-LIVO2 PCD 导出损坏：根因与修复（fouruav NX, 2026-08）

## 现象
建图结束时常发生建图进程被强制关闭 → `all_raw_points.pcd` 导出不完整/损坏。

## 根因链（3 环节缺一不可）
1. **配置**：`task_run_scripts/config/MID360_Fly_Lidar.yaml` 的 `pcd_save` 段，`interval: -1`（所有帧一个文件）。
   launch（`task_run_scripts/launch/mapping_MID360.launch`）只覆盖 `pcd_save/pcd_save_en`（arg `pcd_save` 默认 false；start_fastlivo2.sh 传 true 时才开），**interval 未覆盖** → 取默认 -1。
2. **代码**：`FAST-LIVO2/src/LIVMapper.cpp` — `savePCD()`（484 行起）在 `run()` 的 `while(ros::ok())`（536 行）退出后才调用（553 行）；roscpp 只处理 SIGINT，SIGTERM 默认动作是终止进程；`writeBinary` 直接写最终文件名，无临时文件保护。
3. **停止流程**（两处都只有 2~3 秒就给 SIGKILL）：
   - `nx_command_server.py:259-276` `_kill_tracked_radar_groups()`: `os.killpg(pgid, SIGTERM)` → `time.sleep(2.0)` → `os.killpg(pgid, SIGKILL)`
   - `nx_command_server.py:350-360` `kill_lidar_processes()`: `pkill -SIGTERM` → `sleep 1.5` → `pkill -9`
   - `start_fastlivo2.sh` cleanup(): `kill -TERM` 全部 → 最多 3 秒轮询 → `kill -KILL`

链条：rosnode kill（优雅）→ savePCD 开始写几 GB 文件 → 2~3 秒后 SIGKILL → 写一半被杀 → 损坏。

## 修复（✅ 已实施 2026-08-04，3 文件 4 处改动）
保持 interval=-1 不动；对建图进程改发 SIGINT + 轮询等待真正退出后再 SIGKILL。实施方式：本地写 python 脚本（备份+精确 replace+assert 命中数+bash -n/py_compile/XML 语法校验），用 `plink -m` 传上去执行（base64 会被 cmd 8191 字符上限截断，见 windows-ssh-automation 技能 step 5b）。

备份：3 个文件各生成 `.bak.20260804_123240` 副本；task_run_scripts 是 git 仓库，`git checkout -- <file>` 也可回滚。

### start_fastlivo2.sh cleanup() 实际改法（116-134 行）
```bash
cleanup() {
	echo "..." 
	kill -INT "$FASTLIVO_PID" 2>/dev/null                       # ① roslaunch 进程发 SIGINT
	pkill -INT -f "fastlivo_mapping" 2>/dev/null                #    双保险：直接对节点进程补发
	for _ in $(seq 1 120); do                                   # ② 轮询等节点退出，最长 120s
		pgrep -f "fastlivo_mapping" >/dev/null 2>&1 || break
		sleep 1
	done                                                        #    等它写完 PCD 自己退出
	kill -TERM "$PX4_PID" "$RVIZ_PID" "$IMU_PID" "$LIVOX_PID" "$RS_PID"  # ③ 其余进程照常 TERM
	for _ in 1 2 3; do ...; done                                # ④ 短等 3 秒
	kill -KILL ... 2>/dev/null                                  # ⑤ SIGKILL 兜底
	pkill -KILL -f "fastlivo_mapping" 2>/dev/null               #    节点残留兜底
}
```
注意：轮询对象是 `fastlivo_mapping` 节点进程（pgrep/pkill -f），不是 roslaunch PID——roslaunch 收到 SIGINT 后可能先退，但子节点还在写 PCD。

### nx_command_server.py 实际改法
- `_kill_tracked_radar_groups()`：`signal.SIGTERM` → `signal.SIGINT`；`time.sleep(2.0)` → 轮询 `os.killpg(pgid, 0)`（signal 0 探测，ProcessLookupError = 组已退），最多 90s。
- `kill_lidar_processes()`：`pkill -SIGTERM` → `pkill -SIGINT`；`sleep 1.5` → `time.sleep(20)`。

### ❌ launch 不要改（早期版本写错了，已回滚 2026-08-04）
早期版本建议给 laserMapping 节点加 `timeout="120"`，**实测无效**：
```xml
<!-- 不要加 timeout！roslaunch 报 WARNING: unknown <node> attribute 'timeout' 并忽略 -->
<node pkg="fast_livo" type="fastlivo_mapping" name="laserMapping" output="screen">
```
**原因**：roslaunch 的 `<node>` 属性没有 `timeout`；日志会出现 `WARNING: [...] unknown <node> attribute 'timeout'`。roslaunch 的 15 秒默认 timeout 是节点**启动**超时（节点启动后必须在 15s 内注册到 master，否则杀），**不是关闭超时**——关闭时 roslaunch 会等子节点写完退出，不会中途 SIGKILL。SIGINT+轮询修复单独就够：实测 205MB/6409634 点 PCD 差 0 B。已把 launch 恢复原样（备份 `.bak.revert.20260804_124731`）。

## 行为变化（要告知用户）
停止后进程不会立刻消失：SIGINT 后建图进程继续跑（最多 120s，实际为写完 PCD 为止，通常几十秒），日志停在"正在关闭 Fast-LIVO2..."。这是故意的，别当卡死。

## 完成提示（✅ 已实施 2026-08-04 第二轮，备份 .bak.notify.20260804_123643）
等建图进程退出后，检查 PCD 是否真正落盘，给用户明确提示"可以安全启动新一轮建图"：
- `start_fastlivo2.sh` cleanup() 轮询结束后：`[ -f "$RAW_PCD" ]` → `du -h` 取大小、`grep -a -m1 '^POINTS' "$RAW_PCD" | awk '{print $2}'` 取点数（**PCD 头部是 ASCII，即使二进制文件也能 grep**）→ echo ✅/⚠️。
- `nx_command_server.py` 轮询循环后：`os.path.exists(raw_pcd)` + `os.path.getsize` → print ✅（地面站日志可见）。

## ⚠️ 开新一轮建图会自动先杀旧进程（2026-08-04 确认）
`nx_command_server.py:461` 在 `_launch_and_track_radar()` 之前无条件调用 `kill_lidar_processes()`（注释原话"先关闭旧雷达"）——所以**建图过程中直接发起新一轮 = 强制中断旧图**：
- 旧图 PCD 会保存为"截至中断时刻的快照"（文件完整，但没建完的部分丢失）。
- 启动新图会阻塞最多 90s（等旧进程写完 PCD），界面像卡死，实为保存。
- 手动 SSH 起的旧图只走 pkill 兜底路径（20s 等待），大场景可能写不完。
- 正确流程：先停干净 → 确认 PCD 完好 → 再开新一轮。

## 其他备忘
- `~/catkin_ws/start_fastlivo2.sh`（nx_command_server.py fallback 路径 :476/:482 引用）**不存在**（verified 2026-08-04），fallback 会失败；正常路径走 `_build_lidar_launch_cmd()`（YAML 驱动）。

## PCD 完整性验证（✅ 实测通过 2026-08-04）
修复后立刻用 8/2 与 8/4 两次真实建图验证。判断标准（不用猜）：
1. **逐字节校验**（最硬）：PCD 二进制头部是 ASCII（`FIELDS/SIZE/TYPE/POINTS/DATA binary`），读到 `DATA` 行结束 = 头长 H；`POINTS=N`、每点 `sum(SIZE)` 字节（本机 32B/点：XYZI+时间等 8 字段）。**理论大小 = H + N×32，与实际文件大小差 0 B = 完整**。工具已固化到 NX：`python3 ~/catkin_ws/src/task_run_scripts/scripts/check_pcd.py`（可带目录参数；本会话 205MB/6409634 点 差 0 B）。脚本本体在技能 `scripts/check_pcd.py`，NX 上那份是同一内容。
2. **savePCD 执行铁证**：fastlivo.log 里有 `Raw point cloud data saved to: .../all_raw_points.pcd with point count: 6409634` → 保存完整跑完；被强杀时这行不会出现。
3. **优雅退出铁证**：roslaunch 日志尾部 `[laserMapping-1] killing on exit` → `shutting down processing monitor... done`（无 SIGKILL 痕迹）。
4. **完成提示**：`fastlivo2_<时间戳>_true/` 目录的启动日志或 nx_command_server 输出出现 ✅。

## 用户否决的方案（不要再推荐）
- `interval: >0`（边建图边落盘带时间戳的 pcd）：会产生大量中间文件，用户不想手工合并。

## 关键路径备忘
- start_fastlivo2.sh 实际路径：`~/catkin_ws/src/task_run_scripts/scripts/NX_code/start_fastlivo2.sh`（**不在** task_run_scripts 根目录）
- 实际加载配置：`$(find task_run_scripts)/config/$(arg config_name)`，默认 `MID360_Fly_Lidar.yaml`（建图）/ `MID360_Fly.yaml`（飞行）。**不是** `FAST-LIVO2/config/` 下同名文件（那份 pcd_save 段整个注释，是误导性旧拷贝）。
- PCD 输出：`FAST-LIVO2/Log/pcd/all_raw_points.pcd` + `all_downsampled_points.pcd`（interval=-1 时）
