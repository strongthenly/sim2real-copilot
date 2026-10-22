---
id: GAP-候选-2026-10-22-a
title: Odin1 是前视雷达，不是 360°；按 Mid-360 写的“全向”逻辑会清掉视场外的真实障碍
layer: L1
status: 候选
report_level: 实飞通过
scope:
  airframe: uav2（Orin NX 8 GB + Odin1，无 Mid-360 / D435）
  firmware: 未记录
  sensors: [Odin1]
  config: [odin_real_flight.yaml, fuel_real_airborne.launch FOV_ARGS]
evidence:
  real: [2026-10-22 /odin1/cloud_raw 静态实测, 实飞 RUN 20261022_121913 / 20261022_150803]
  sim: []
  source: []
verified_at: 2026-10-22
revalidate_on: [更换雷达, Odin 固件升级, 雷达安装角变化]
---

## 差异
按 Mid-360 设计时：雷达水平 360°，SUPER 的全向清除光线（omni rays）、FUEL 的视场模型、机头朝向都按“四周都看得见”来写。
Odin1 实机：前视，水平约 ±67–70°，垂直约 ±45°；室内距离中位 2.0 m，最大约 10.9 m。

## 原因
传感器换型，但规划栈里多处隐含“360° 水平视场”假设：
- ROG 全向清除光线在机身侧后方发射，会把雷达根本看不到的真实树干当“空”清掉（与 10-12 撞树同一机理）。
- FUEL 默认按 360° 判断前沿能否被看到。
- yaw_mode=2（朝目标）时飞机可能侧飞/倒飞进看不见的地方。

## 补偿方式
- SUPER：新增 `raycasting/omni_fov_h`（相对机头 ±度，≥180 保持原全向行为），Odin 设 ±60°；`omni_fov_v` 设 ±35°。
- FUEL：视场参数化（fov_h360 / fov_left / fov_right / fov_up / fov_down），Odin 传 ±65° / ±40°。
- SUPER `yaw_mode` 改 1（机头朝飞行方向）。

## 对 agent 的影响
换传感器或在 Odin 机型上调避障、探索参数时必须引用这一条：所有“视场”类参数都要小于雷达实测视场，且要检查是否有代码隐含 360° 假设。
