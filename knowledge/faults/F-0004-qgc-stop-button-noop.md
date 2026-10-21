---
id: F-0004
title: QGC 停止按钮无效
layer: L1
status: 已验证
evidence: [knowledge/00_handbook.md §3 L1]
eval_case: evals/cases/diag-004.yaml
---

## 根因
按钮命令用 `&&` 串联，第一环失败即短路，后续 `pkill` 不执行。此外 plink 下 `pkill -f` 会匹配到自身命令行（exit 128）。

## 修复
串联改为 `;`；用精确 PID 或 `rosnode kill` 代替 `pkill -f`。
