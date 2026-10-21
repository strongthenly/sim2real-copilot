---
id: GAP-0001
title: MID-360 垂直视场只有 -7°~+52°，正下方存在盲区
layer: L1 / L7 / L8
status: 已验证
report_level: 实飞通过
scope:
  airframe: fouruav
  sensors: [Livox MID-360S]
evidence:
  source:
    - knowledge/00_handbook.md §1「雷达视野实测事实」
verified_at: 待补（见原文实测日期）
revalidate_on: [更换雷达型号, 更改雷达安装角]
---

## 差异
仿真中：若按"360°"或按规格书理想模型建模，传感器看起来无盲区。
实机上：360° 只指水平；垂直视场 -7°~+52°，正下方盲区半径约 `高度 / tan(7°)`。

## 补偿方式
- 仿真侧：传感器模型必须按实测垂直视场配置。
- 规划侧：视点评估要计入下方盲区，否则会选错视点。

## 对 agent 的影响
评估视点、覆盖率、贴地飞行或下视观测时，必须引用本条。
