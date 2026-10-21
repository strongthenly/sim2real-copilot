# odin 传感器接入 SUPER 地图 / 动态障碍残留 / 全屏障碍 排查实录（2026-08-13~14）

本文件是 fouruav 换 odin 传感器后，SUPER ROG-Map 地图与动态避障调试的完整知识库。按"问题 → 根因（源码级）→ 解法"组织。

## 1. odin 传感器适配三件套

换传感器只改传感器层，导航核心（方案F2/终点判定/走廊）不动：

| 层 | 改动 | 说明 |
|---|---|---|
| 点云 | `cloud_topic: /odin1/cloud_slam` | odin 自带 SLAM 配准点云，frame=odom，可直接进地图 |
| 位姿桥 | `odom_to_mavros.py` | `/odin1/odometry_highfreq`(400Hz) → `/mavros/vision_pose/pose`，**限速 20Hz + pid 单实例锁** |
| 启动 | Radar_Launcher.yaml `active_profile: "odin"` | cleanup 含 host_sdk_sample/odom_to_mavros；启动链含 `sleep 12 && python3 odom_to_mavros.py` |

odin 话题（`rostopic list | grep odin1`）：
- `/odin1/cloud_raw` — 原始 dToF 点云 49152 点，**frame_id="lidar"**（雷达局部系！）
- `/odin1/cloud_slam` — SLAM 配准点云 ~33000 点，**frame_id="odom"**（累积，含历史点）
- `/odin1/odometry_highfreq` — 400Hz 里程计
- TF 树只有 `odom→imu`（`send_odom_baselink_tf: 1`）

## 2. vision_pose 频率坑（ready→not ready 循环）

**现象**：按"启动雷达"后反复 not ready。
**根因链**：odin odometry 400Hz 全转发 → vision_pose 500-1000Hz → PX4 EKF 拒收/判视觉不可信 → `pos_horiz_abs=False` → NOT READY。另有多个 odom_to_mavros 实例（一次按钮点击 bash 子链多拉）→ vision_pose 6570Hz 打架。

**修复**：
1. **限速**：odom_to_mavros.py 回调里 `if (now - self._last_pub).to_sec() >= 0.05` 才发（20Hz）
2. **pid 单实例锁**：脚本启动时写 pid 文件，已有则退出（比 rosnode 查询可靠，因为 anonymous=True 节点名带 PID 后缀且时序竞争）
3. 验证：`ps aux | grep "python3.*odom_to_mavros" | wc -l` 应=1（**注意：grep 不带 python3.* 前缀会把 bash 包装命令行也算进去**，误报 3-4 个）

**频率上限知识**：PX4 EKF 处理 vision_pose 上限 ~100Hz；1m/s 飞行 20Hz 足够（EKF 内部 IMU 250Hz 内插，vision 只是低频校准）。想改时间戳为原始 msg.header.stamp 有风险：odin 设备时间戳≠主机时间（差数百秒），除非确认 `use_host_ros_time=1`。

## 3. 动态障碍残留（核心问题，源码级）

**现象**：人走过障碍后，地图里障碍块残留，飞机一直以为有障碍 → 绕行/悬停。残留集中在上半部分（空旷方向）。

**根因（prob_map.cpp `raycastProcess`）**：
```cpp
// 点云光线：从飞机向【每个点云点】发射，沿途格子标 free
for (const auto& p : raycasting_cloud) {
    Vec3f raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
    raycast_data_.raycaster.setInput(raycast_start, p);
    while (raycast_data_.raycaster.step(ray_pt)) {
        insertUpdateCandidate(cur_ray_id_g, false);  // free
    }
}
```
**只有"点云里有点的方向"才有光线** → 障碍消失后该方向空旷无点 → 无光线穿过 → 残留不清。

**cloud_slam vs cloud_raw**：
- `cloud_slam`（累积）：人走过的点"留在地图"→ 残留最重。但 frame=odom 直接用。
- `cloud_raw`（当前帧）：人走即无点 → 可清。但 frame=lidar，**必须桥接变换到 odom**（地图不做 frame 变换，直接 `pcl::fromROSMsg` 原样用）。

**全向光线方案（已重新启用且有效，2026-08-14 晚确认）**：在 raycastProcess 点云光线后加全向稀疏光线（config.hpp 加 `raycast_omni_*` 参数 + prob_map.cpp 循环：水平 `omni_step_h`°×垂直 `omni_step_v`°×`omni_fov_v`°×`omni_range` m，遇"当前帧命中格"（`hit_cnt[hash_id]>0`，注意 **hit_cnt 是 `std::vector<uint16_t>` 不是 map，不能用 .find()**，用索引+越界检查）停止，其余标 free）。效果：残留大幅减少（occ 7649→2136 格），只剩小部分。
**历史**：第一轮启用时出现"被障碍包围"视觉 → 回退 `omni_en: false` 待修。**事后经二分确认（见第 4 节）：全屏障碍根因是 fix_map_origin 误改 0，不是 omni**；omni 只清 free 不清 hit，逻辑本身安全。配置基线恢复正常（fix_map_origin 1.5 + virtual_ground -3.0）后重启用 `omni_en: true` 有效。

## 4. 全屏障碍（"飞机被障碍包围"）排查实录

**现象**：RViz 全屏都是障碍块（occ 7622 格散布 x:0~4, y:±4, z:-0.4~1.9）。

**排查链**（二分法）：
1. 点云正常（cloud_raw_odom z:0.1~3.48，飞机周围±1m 只 4%）→ 排除点云变换错乱
2. occ 7649 格但 inf_occ 3753（膨胀层正常）→ 排除膨胀层连片
3. **切回 cloud_slam 二分** → 也全屏 → 排除 cloud_raw 桥接链路，定位到"地图参数/坐标系"
4. **核对 FSM 实际加载值**（`grep "Load param rog_map/"` + `[ProbMap]` 日志）：
   - `fix_map_origin success: [0 0 0]`（我们改成 0，但 yaml 已回退 1.5！）
   - `map_size success: [30 30 8]`、`virtual_ground success: -3`、`virtual_ceil success: 4` —— **全是回退前的旧值**
5. **实锤：FSM 没重启**（配置 13:43 改，FSM 进程还是加载旧值的实例）→ 改配置必须重启按钮3

**fix_map_origin 教训**：`[0,0,1.5]` 抬高地图坐标系 1.5m → 点云 z 相对地图原点 = 实际z - 1.5 → 地面点(z≈0.1)变成"空中障碍"(地图系-1.4) → 全屏散布。**MID360/FAST-LIVO2 时代 1.5 是对的，odin 时代也是 1.5（15:12 那次 cloud_slam 地图 2Hz 正常）**。改 0 雪上加霜。地图坐标系必须与点云坐标系约定一致，改之前先看它为什么是 1.5。

## 5. virtual_ground/ceil 量化 bug

prob_map.cpp 初始化：
```cpp
posToGlobalIndex(cfg_.virtual_ground_height, sc_.virtual_ground_height_id_g);
cfg_.virtual_ground_height = sc_.virtual_ground_height_id_g * cfg_.resolution;  // 回写量化后值
```
`posToGlobalIndex`（ORIGIN_AT_CENTER）：`id = int(res_inv*pos + sign*0.5)`（四舍五入到最近栅格）。

实测量化表（resolution=0.15, res_inv=6.667）：
| yaml 值 | 计算 | 实际 |
|---|---|---|
| -0.9 | int(-6+(-0.5))=int(-6.5) → -6? 但日志 +1 | **+0.15**（→ 地面 z≈0 被拒 "below virtual ground" → 无地图） |
| -3.0 | int(-20.5) → 应-20 | 日志 **-1.95**（被地图边界/clamp 改） |
| 3.0（ceil） | int(20.5) → 应 20 | 日志 **1.95**（map_size z=6 时被钳制） |
| 4.0（ceil） | int(27) → 应 27 | 日志 **2.85** |

**教训**：`[ProbMap] virtual_ground_height: X` 日志是唯一真相，yaml 值仅供参考。地面测试（z≈0）必须先确保量化后 ground < 0；飞行测试（z=0.6）之前没暴露是因为 0.6 > 0.15。

## 6. 地面过滤与降采样（桥接脚本要点）

`scripts/cloud_raw_lidar_to_odom.py` 桥接（cloud_raw → /odin1/cloud_raw_odom）：
```python
trans = tf_buffer.lookup_transform('odom', msg.header.frame_id, rospy.Time(0), ...)  # 必须 Time(0)!
out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
# 1) 地面过滤: z < 0.10 的点剔除（地面点占 19%，否则全被扫成障碍）
# 2) numpy 体素降采样 5cm: 49152 → ~13000 点（否则 FSM CPU 116% → Replan over time）
```
验证：`width` 应 ~13000，`z<0.1` 应 0 点。

## 7. 绕行"暂停原地"与俯冲（MINCO 失败链）

**暂停原地**：A* 绕行路径贴障碍（0.04-0.2m）→ 走廊凸分解要求离障碍≥安全距离 → `GeneratePolytopeFromLine failed` → MINCO `Opt failed`（`The problem is not feasible, min dis to obstacle: 0.04`，日志 456-74458 次）→ 无轨迹 → bridge 0.5s 超时 → position hold。解法：`inflation_step: 3`（0.6m 膨胀）+ 方案A `corridor_small_` 标志（循迹小立方体 0.2m 硬约束贴航迹，fallback 置 false 用大盒子 0.39m 给 MINCO 绕行空间）。

**俯冲**：绕行→回正轨迹切换 → MINCO 加速度突变 → PX4 俯冲（z 0.68→1.02→0.84）。双保险：bridge `acc_ff_smooth: 0.5`（加速度前馈一阶低通 `out = α*new + (1-α)*old`，α=0.5）+ MINCO `penna_jerk` 主轨迹 -7.5e5（原-5e5 的平滑与响应折中）、backup -5e5。

## 8. 日志判读速查

| 日志 | 含义 |
|---|---|
| `Odom below virtual ground` | virtual_ground 量化后 > odom z（地面测试必查） |
| `Map is empty, cannot perform box search` | 地图无数据（点云没进/odom 超时） |
| `The problem is not feasible, min dis to obstacle: 0.04` | A* 路径贴障碍 → 走廊/MINCO 失败 |
| `GeneratePolytopeFromLine failed` | 走廊扩不出（障碍太近/地图太脏） |
| `Replan over time(0.178s)` | FSM 过载（点云太多，需降采样） |
| `SUPER command timeout` | 无轨迹 → bridge 悬停（绕行失败兜底） |
| `Using FULL GLOBAL PATH, idx=N total=M` | 沿全局轨迹正常循迹 |
| `Global path blocked by obstacle, fallback to A*` | 避障绕行中 |
| `A* fallback path corridor OK` | 绕行走廊成功 |

## 9. 调参经验（参数调整汇总，已写入 SUPER.md 第七章）

- **max_vel** 0.3→1.0：长轨迹(≥10m)用 1.0；调速先确认 max_acc 匹配（1.0 配 0.6：加速距离 ~0.85m）
- **max_omg** 2.5→6.0 / **max_acc_thr** 20→25：绕行急转弯需放宽（"Opt failed, Omg or thr or Pos violation" = 物理约束太紧，先放宽约束而非降速）
- **penna_jerk**：越大越平滑（防俯冲）但拖慢响应；主轨迹 -7.5e5 折中
- **inflation_step**：2(0.4m)→3(0.6m) 绕行可行，但膨胀残留更多（动态障碍需配 cloud_raw 当前帧）
- **p_miss** 0.49→0.6：障碍清除更快（2 帧内），但只对"有光线方向"有效
- **corridor_line_max_length** 1.2：线段太短（0.25）会崩 "seed line too long"
- **switch_dis 0.1m 终点判定**：根治终点 idx 只剩 2 点→走廊失败→终点摆
- **goal_z 0/0.3 是地面临时值**，飞行前必须改回 0.6/0.825

## 10. 操作纪律（本会话教训）

- **改 yaml 必重启**：改完必须按钮3 重启（FSM 启动时加载 rosparam），并用 `ps -eo lstart` + `grep "Load param"` 验证生效
- **二分优先**：地图异常先切 cloud_topic 二分（cloud_slam vs cloud_raw_odom），再动参数
- **全屏障碍排查顺序**：①FSM 重启了吗 ②[ProbMap] 实际值 ③fix_map_origin 坐标系 ④二分 cloud_topic
- **每处改动备份 + 贴 diff**（用户硬性要求，可回滚）
- **调参 2-3 轮无效 → 主动回退**（本会话教训）：连续叠加参数后全屏障碍，用户直接要求"还原到正常状态"。**正常锚点 = `.bak_odintopic`**（cloud_slam + fix_map_origin 1.5 + map 30,30,6 + ceil 3 + ground -0.9 + p_miss 0.49 + inflation 2，即全屏障碍前的全部原值）。回退流程：`cp real_flight.yaml.bak_odintopic real_flight.yaml` → 把 cloud_topic 改回 `/odin1/cloud_slam`（该备份是 15:06 的，还是 MID360 的 /cloud_registered）→ 重启按钮3。**每次都先备份当前混乱版**（.bak_confused_HHMM）再覆盖
- **网络不稳**：<NET_WLAN_B>.31 与 <NET_WLAN_A>.50 常超时，重试或换机

## 11. 雷达高度几何遮挡（残留的最后一块拼图，08-14 晚实测）

**用户实测**：地图有障碍残留，**把飞机拿起来（雷达离地 1m+）残留就消失**。

**原理**：raycasting 光线从飞机位置发出 → **雷达越矮（地面测试 0.1-0.3m），激光从低处扫，障碍背后的区域无光线 → 残留**；雷达升高俯视，障碍背后能被看到 → 光线覆盖全 → 残留消失。**这是几何遮挡，不是地图 bug**。

**推论**：
- 飞行作业（雷达离地 0.6m+）残留本来就少；地面测试残留严重是正常现象
- 地面测试垫高雷达（放凳子）更接近飞行状态
- **别为地面测试的残留过度调参**（之前为此折腾 cloud_raw/全向光线，根源其实是测试高度）

**残留剩余小部分的三来源与对策**（omni 启用后）：
1. **光线间隔空隙**：10°×10° 网格在 8m 处相邻光线间距 ≈ 8×tan(10°) ≈ 1.4m → 细障碍（人/杆 0.3-0.5m）漏清 → 加密 `omni_step_h: 10→5`、`omni_step_v: 10→7.5`（**先查 FSM CPU 有余量再加密**，实测 43% 可加）
2. **概率衰减慢**：`p_miss: 0.49→0.6`（清除快 2 倍）
3. **膨胀层残留**：`inflation_step` 缩小（0.4m→0.2m）但影响绕行空间，最后动

**omni 当前参数**（yaml `rog_map/raycasting` 段）：`omni_en: true / omni_range: 8.0 / omni_step_h: 10.0 / omni_step_v: 10.0 / omni_fov_v: 60.0`——加了 `omni_*` 配置后 p_free 那行的注释位置会偏移，无影响。
