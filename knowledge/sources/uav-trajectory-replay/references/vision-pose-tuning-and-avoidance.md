# vision_pose 频率 / 时间戳 / 调参经验 / 避障问题评估（2026-08-14）

## vision_pose 频率上限（odin → PX4 视觉链路）

- **PX4 EKF2 处理上限 ~100Hz**，建议上限 50Hz（超过收益递减 + 噪声相关性风险：同一时刻多观测被当独立观测 → 过度信任 → 漂移）
- **当前 20Hz 对 1m/s 足够**——原理：EKF = IMU 预测(250-500Hz 连续内插) + 视觉修正(低频校准)。20Hz = 每 0.05s 修正一次，1m/s 移动 0.05m，IMU 内插完全覆盖。类比：IMU=秒表(2ms 一跳)，视觉=对表(50ms 校一次)——对表频率不影响秒表精度
- 频率调整：odom_to_mavros.py 限速行 `>= 0.05`(20Hz) → `0.04`(25Hz) / `0.02`(50Hz)。25Hz 是用户当前选择
- **真正影响 1m/s 跟踪的是延迟不是频率**：odom_to_mavros 现在 `pose_map.header.stamp = rospy.Time.now()`（转发时刻当时间戳）→ 引入几十 ms 延迟 → 1m/s 下 50ms=5cm 偏差

## 时间戳改原始（msg.header.stamp）的隐患——先查 use_host_ros_time

| 隐患 | 说明 | 风险 |
|---|---|---|
| 时钟不同步 | odin 消息 stamp 来自**设备时钟**（use_host_ros_time=0）→ 设备时间≠主机时间 → PX4 收到错位时间戳 → EKF 拒绝/错误补偿 | 🔴 高 |
| 时间戳跳变 | 驱动启动时校准时钟 → 前后跳 → EKF 混乱 | 🟡 中 |
| 零/固定时间戳 | 某些驱动 stamp=0/固定值 → PX4 拒收 | 🟡 中 |
| 延迟补偿反向 | PX4 按时间戳算延迟，时间戳错 → 补偿错 → 比 now() 更差 | 🔴 高 |

**决策**：查 `control_command.yaml` 的 `use_host_ros_time`——=1(主机时间) 用原始安全；=0(设备时间) 保留 now()。查不了就**只改频率，时间戳改动等确认同步后再说**。

## 调参经验汇总格式（用户 2026-08-14 明确要求的文档章节）

用户要求 SUPER.md 增加"参数调整经验汇总"，格式为**每个参数带 调整过程 → 现象 → 经验** 的表格 + 分类：

1. **速度类** max_vel/max_acc/max_omg/max_acc_thr：`max_vel` 0.3→1.0(短轨迹用低俗,≥10m 用 1.0)；`max_acc` 4.0→0.6(太激进→MINCO Opt failed)；`max_omg` 2.5→6.0(绕行急转弯 Omg violation)；`max_acc_thr` 20→25(推力余量)
   - 核心经验：**MINCO "Opt failed, Omg or thr or Pos violation" = 物理约束太紧 → 先放宽 omg/thr，不是降速**
2. **平滑类** penna_jerk/acc_ff_smooth：`penna_jerk` 主轨迹 -5e5→-1e6→**-7.5e5 折中**（越大越平滑不俯冲，越小响应快；backup -1e5→-5e5 响应优先）；`acc_ff_smooth` 0.3→0.5（低通系数 α：输出=α×新+(1-α)×旧；0.3 太肉、0.5 平衡）
   - 核心经验：**俯冲=加速度突变 → 两条路：轨迹层 jerk 加大(治本) + 控制层 acc 低通(治标快)，两个都做=双保险**
3. **走廊类** corridor_line_max_length(0.25 崩 seed line too long → 1.2 稳)/corridor_small_(循迹紧小立方体、绕行松大盒子)
4. **地图类** cloud_topic(换传感器必改；cloud_slam 累积点云→人走障碍残留)/inflation_step(2→3→回退2：膨胀=安全vs空间)/p_miss(0.49→0.6→回退：动态清除需配合当前帧点云才有效，只调一个无效)
5. **终点类** switch_dis(0.1m 悬停根治终点摆)/goal_z(飞行前必须改回 0.6/0.825)
6. **视觉桥类** 限速 20Hz(PX4 EKF 拒超频)/单实例锁(任何转发节点都要防多实例)
7. **调参方法论**：现象归类（斜切=轨迹约束、振荡=加速度突变、绕行失败=物理约束或走廊空间、不更新=地图、not ready=视觉链路）→ 一次只改一个 → 日志关键行判读（Opt failed=物理约束、GeneratePolytopeFromLine failed=走廊/障碍距离、The problem is not feasible min dis=路径贴障碍、SUPER command timeout=无轨迹、below virtual ground=地图原点）→ 配置(yaml)优先，不行再改代码(编译)

## 避障问题评估清单（静态 vs 动态，2026-08-14 分析）

**静态避障 ≈ 80% 可用**：
- ✅ 检测(131-135 次 blocked) + 主轨迹绕行(122 次 SUCCESS，飞机沿 y=-1.5 侧绕)
- ⚠️ 问题1：A* 贪心最短路径贴障碍(0.04-0.2m) → 走廊凸分解失败 → MINCO 失败 → bridge 超时悬停（"绕行趋势但暂停原地"）——方案A 大盒子缓解未根治；改进：A* 膨胀加大或路径平滑，但加重障碍残留
- ⚠️ 问题2：backup 轨迹 100% 失败(117-233 次)——主轨迹失败瞬间无第二保险；改进：**backup 直接用 A* 路径直发（不做 MINCO 优化），折线也能飞比悬停强**
- ⚠️ 问题3：绕行恢复 z 俯冲（已修：低通+jerk，待实测）

**动态避障 ≈ 40% 可用**：
- ❌ 问题1（核心）：**cloud_slam 累积点云 → 人走点留 → 假障碍 → 误判悬停/绕行假目标**（动态避障"癌症"）；改进：**cloud_raw 当前帧建图（人走即消，值得重试——之前"无地图"是 fsm 没跑+virtual_ground 误判，非 cloud_raw 本身问题）** 或 raycasting 清除加强
- ⚠️ 问题2：检测延迟（点云 10Hz+replan 10Hz → 0.2-0.5s；1m/s 下 0.2-0.5m 提前量，慢走的人够、跑过来的人可能来不及）
- ⚠️ 问题3：动态障碍绕行不稳定（人移动时每轮 A* 路径变 → 来回摆/犹豫）

**优先级**：① 动态障碍残留(cloud_raw) ② backup 直发 A* ③ 绕行贴障碍(inflation) ④ 检测延迟 ⑤ 绕行抖动
