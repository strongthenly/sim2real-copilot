---
id: F-候选-2026-10-22-a
title: 改了启动脚本但地面站按钮行为不变——按钮运行的是功能目录里的脚本快照
layer: L3
status: 候选
scope: {airframe: uav2, stack: AirCore 功能目录（工业版）}
evidence: [2026-10-22 12:00 运行目录中的 program-fuel_explore_all.sh 与改前备份逐字节相同]
eval_case:
---

## 症状
已修改并保存 `~/AirCore/fuel_explore_all.sh`（例如把清理列表里的 `/rviz` 改为 `/super_rviz`），点“智能航线任务”后行为仍是旧的：Odin 感知 RViz 在启动约 4 s 后被关掉，FUEL 仍按旧视场运行。

## 先分类
- 改动没保存 / 改错文件？
- 改的文件对，但实际运行的不是它？← 本例

## 根因
地面站按钮由 AirCore 功能服务启动，执行的是功能目录 `/var/lib/liyumen-functions/functions.yaml` 中内嵌的脚本快照（base64 + sha256），每次启动写成 `/var/lib/liyumen-ros-task/<task_id>/program-fuel_explore_all.sh` 再运行。官方保存接口只对教育版开放，工业版改磁盘脚本不会更新快照。

## 判据 / 证据
- 对比运行目录里的 `program-*.sh` 与磁盘脚本（sha256 或 diff）。
- 日志里被 `rosnode kill` 的节点名仍是旧清理表里的名字。

## 修复
用 AirCore 自己的代码重新生成快照（加锁、校验），已封装为 `~/AirCore/refresh_explore_snapshot.sh`：有任务运行时拒绝；先备份功能目录；试算只允许探索功能快照变化；刷新后核对 sha。刷新后地面站可能提示“配置已更新”，刷新功能列表即可。
只有启动脚本本身受影响；它在运行时读取的 launch / yaml / Python 都直接读磁盘。

## 容易误判成
“节点名冲突没修好”或“RViz 自己崩溃”。
