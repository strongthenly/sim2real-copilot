---
id: PARAM-候选-2026-10-22
title: 2026-10-22 新机部署中确认的参数真实语义（候选）
layer: L1
status: 候选
scope: {airframe: uav2, stack: SUPER + FUEL + goal_arbiter + super_to_px4_bridge}
evidence: [源码阅读 + 实飞 RUN 20261022_121913 / 150803]
---

| 现象 | 真正起作用的参数 | 说明 | 验证程度 |
|---|---|---|---|
| 转向很慢（转 180° 要 18 s） | `super_to_px4_bridge.yaml` 的 `max_yaw_angle_rate`（原 0.1745 rad/s = 10°/s） | SUPER 的 `yaw_dot_max`（2.5 rad/s）只是规划上限，实际转速被桥限幅；用户选 20°/s | 实飞通过（参数生效经 dump-params 确认） |
| 每到一个探索目标机头都转向同一方向 | `goal_arbiter.py` 发 `/waypoint_goal` 时 orientation 恒为 w=1 | SUPER 开着 `click_yaw_en` / `goal_yaw_en`，w=1 被当成“末端朝向 yaw=0” | 离线通过；现改为只在 FUEL 最终目标带 FUEL 视点航向，其余发 NaN（航向自由） |
| Odin 感知 RViz 一启动探索就消失 | 两个 RViz 都叫 `/rviz` + 启动脚本清理表里有 `/rviz` | 规划侧 RViz 改名 `super_rviz`；清理表同步改（还需刷新按钮快照，见 F-候选-2026-10-22-a） | 实飞通过 |
| 产物 CSV 看不懂 / Excel 显示 1.11E+11 | FUEL coverage 由 C++ 直接追加写、无表头；树表方位串全是 0/1 | 加中文表头 + UTF-8 BOM；方位串改 ●○；时间戳改定点 | 离线通过 |
