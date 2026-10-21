---
id: GAP-0003
title: 机载算力预算——桌面仿真复现不了建图配置导致的位姿延迟
layer: L2
status: 已验证
report_level: 实飞通过
scope:
  airframe: fouruav
  compute: Jetson Orin NX
  config: [MID360_Fly.yaml, MID360_Fly_Lidar.yaml]
evidence:
  source:
    - knowledge/00_handbook.md §3 L2「飞行必须用低算力配置」
    - knowledge/domains/KB_slam-mapping.md
verified_at: 待补
revalidate_on: [机载电脑换型, 修改 FAST-LIVO2 配置, 场景密度显著变化]
---

## 差异
仿真中：桌面算力充足，`MID360_Fly_Lidar.yaml`（稠密建图）运行正常。
实机上：该配置约需 2 倍算力，在树林中导致位姿延迟 → 定点漂移 → 迫降。

## 补偿方式
- 飞行一律使用 `MID360_Fly.yaml`（`dense_map_en=false` + 抽稀 3 倍）。
- 仿真侧如需评估算力影响，应限制 CPU 或注入位姿延迟。

## 对 agent 的影响
任何"仿真里定位很稳"的结论，在推断实机表现前都必须核对本条。
