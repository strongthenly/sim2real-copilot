# PX4 1.17 避障参数锁死 POSCTL + 定位诊断（2026-08-11 fouruav 实战）

## 症状
- `rosservice call /mavros/set_mode "{custom_mode: 'POSCTL'}"` 能切，但几秒后自动回退 HOLD
- MAVROS 报 `navigation error: No valid global position estimate`
- QGC 显示 Position 但飞控实际保持原模式（nav_state 全程不变）

## 最终根因（本次确认）
**PX4 碰撞预防参数 `CP_DIST=1`（开启）+ `CP_GO_NO_DATA=0`（无数据方向不可飞）+ `obstacle_to_px4` 数据源未运行** → PX4 认为全向 1m 内都是障碍 → 拒绝位置控制 → 自动回退。

**不是定位问题**（.ulg 证实视觉融合正常）。避障系统在"有参数、无数据"的矛盾配置下把位置控制锁死。

## 排查弯路（勿重走）
1. ❌ 查 `/mavros/estimator_status` 的 `pos_horiz_abs=False / const_pos_mode=True` → **误导**。该 ros 话题 flag 布局与 .ulg 的 `estimator_status_flags` 字段不同，显示 const_pos 但实际视觉融合正常
2. ❌ 查 `EKF2_AID_MASK` → PX4 1.17 已废弃（被 EKF2_EV_CTRL / GPS_CTRL / OF_CTRL 替代），读 0 = 参数不存在
3. ❌ 查视觉话题/坐标系 → 全正常（vision_pose 平滑、MAVROS 订阅、EV_CTRL=11）
4. ✅ 查避障参数 → CP_DIST=1 + CP_GO_NO_DATA=0 + 无数据源 = 锁死根源

## 正确诊断顺序（5 秒先查避障）
```bash
# ① 避障参数（先查这个！）
rosservice call /mavros/param/get "{param_id: 'CP_DIST'}"          # >0 = 避障开了
rosservice call /mavros/param/get "{param_id: 'CP_GO_NO_DATA'}"    # 0 = 无数据不可飞

# ② 数据源
ps aux | grep obstacle_to_px4
rostopic hz /mavros/obstacle/send --window 3

# ③ .ulg 定位真相（别信 ros 话题 flag）
#    estimator_aid_src_ev_pos.fused=1 + time_last_fuse 递增 = 视觉融合正常
#    estimator_status_flags.cs_ev_pos=1 = 视觉辅助启用
#    estimator_aid_src_fake_pos.fused=1 = 在用"假位置"兜底（不健康）
```

## 解决方案
- **不用避障**：`CP_DIST=-1`（负值=关闭，rosservice 或 QGC）
- **要用避障**：先启动 obstacle_to_px4 数据源 + 建议 `CP_GO_NO_DATA=1`

## PX4 避障参数速查
| 参数 | 含义 | 注意 |
|---|---|---|
| CP_DIST | 碰撞预防距离(m) | **负值=关闭**；开启必须配数据源 |
| CP_GO_NO_DATA | 无数据时能否飞 | 0=不可飞(保守,危险)；1=可飞 |
| CP_DELAY | 响应延迟 | 震荡调大 |
| CP_GUIDE_ANG | 引导角 | 阶段5再开 |

## 机上文档
- `task_run_scripts/scripts/NX_code/obstacle_warning.md`（2026-08-11 提交 ecb0846，完整排查链）
