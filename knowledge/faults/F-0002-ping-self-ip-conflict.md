---
id: F-0002
title: ping 雷达能通，其实 ping 的是自己（IP 冲突）
layer: L3
status: 已验证
evidence: [knowledge/00_handbook.md §3 L3]
eval_case: evals/cases/diag-002.yaml
---

## 症状
ping 雷达 IP 正常，但驱动收不到数据。

## 判据
`ttl=64` 且延迟 < 0.1 ms → 回应来自本机（主机占用了雷达 IP）；`ttl=255` 才是雷达本体。

## 修复
修正主机网卡 IP，使其与雷达配置中记录的 host IP 一致且不冲突。
