---
id: F-0003
title: 停止建图后 PCD 没有保存
layer: L2
status: 已验证
evidence: [knowledge/00_handbook.md §3 L2, knowledge/domains/KB_slam-mapping.md]
eval_case: evals/cases/diag-003.yaml
---

## 根因
roscpp 只响应 SIGINT，不响应 SIGTERM。用 `kill -TERM` 停止时，`savePCD()` 没有机会执行。
保存只在 `while(ros::ok())` 退出后执行一次，且要求 `pcd_save_en && interval < 0`。

## 修复
用 SIGINT 停止节点。

## 容易误判成
"写盘慢"或"磁盘满"。关闭慢通常是清理等待链慢，先比较日志 mtime 与指令时刻。
