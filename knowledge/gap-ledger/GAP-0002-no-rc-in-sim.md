---
id: GAP-0002
title: 仿真里没有遥控器，状态机不会自动进入规划飞行
layer: L1 / L7
status: 已验证
report_level: 后端联通
scope:
  airframe: fouruav
  config: [real_flight.launch, super_to_px4_bridge]
evidence:
  source:
    - knowledge/00_handbook.md §3 L1「bridge 状态机」
    - knowledge/domains/KB_control-stack.md
verified_at: 待补
revalidate_on: [修改 bridge 状态机, 更换仿真平台]
---

## 差异
实机上：自动起飞悬停后，由 CH10 拨杆或按钮触发进入规划飞行。
仿真中：没有遥控器输入，不设置会出现"有规划轨迹但飞机不动"。

## 补偿方式
- 仿真侧：开启 `auto_arm` + `auto_offboard`。
- 注意：这两个开关**只能用于仿真**，不得带入实机配置。

## 对 agent 的影响
诊断"仿真里飞机不动"时优先检查本条；生成实机配置时必须确认这两项已关闭。
