---
id: F-0001
title: rostopic hz 正常但消息为空
layer: L3
status: 已验证
evidence: [knowledge/00_handbook.md §3 L3]
eval_case: evals/cases/diag-001.yaml
---

## 症状
雷达话题频率很高（例如 369 Hz），但下游没有点云。

## 根因
话题在高频发布**空消息**（`pts=0`）。

## 判据
`rostopic hz` 不作数，必须取一帧看内容（点数、时间戳）。

## 容易误判成
"雷达工作正常，问题在下游算法"。
