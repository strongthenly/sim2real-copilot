---
id: F-候选-2026-10-22-c
title: 拨 CH10 后几秒 FUEL 就判“探索完成”，只飞了几米
layer: L1
status: 候选
scope: {airframe: uav2, module: FUEL exploration_node}
evidence: [实飞 RUN 20261022_150803 fuel.log / coverage.csv]
eval_case:
---

## 症状
用户感觉“没飞到 20 m”。实际最远 x ≈ 7 m；FUEL 只给出 1 个目标，约 4 s 后 FINISH，已探明比例仅 46.7%；之后是绕附近的树和返航。

## 先分类
- 区域参数设小了？（否：探索框 0–20 m）
- 视场模型改坏了？（否：12:19 同样配置飞到了 16 m；离线几何模拟也排除）
- FINISH 判定过早？← 本例

## 根因
1. 起飞前在地面等触发约 30 s，前沿数从 27 降到 1（大量被判“无可见视点”）。
2. 触发时飞机离地约 0.75 m，FUEL 地图把起飞点附近地面膨胀成障碍，判起点在碰撞中，4 s 内 373 次“No path to next viewpoint”。
3. 某次重规划时前沿列表恰好为空 → 立刻 FINISH。FINISH 是终态；几秒后前沿已回到 9–21 个，但不会再恢复探索。

## 判据 / 证据
fuel.log 中 “No coverable frontier → FINISH” 之后 `/fuel/frontier_stat` 前沿数回升；coverage.csv 的 FINISH 行已探明比例远低于预期。

## 修复（已编写并编译，尚未实飞）
- FUEL 加宽限期 `fsm/finish_grace`（默认 15 s）：前沿为空时每 0.5 s 重试，持续 15 s 才 FINISH。
- 操作要求：起飞后悬停到 1.2–1.5 m 再拨 CH10（10-08 在 Mid-360 机型上已有同类现象）。

## 容易误判成
“视场改成前视后看不到前沿了”。
