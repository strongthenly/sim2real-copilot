# SUPER 参数调整经验汇总（2026-08-13~14 实测）

> 每个参数记录"问题 → 调整 → 效果"链路。调参先试配置(yaml，快)，不行再改代码(编译，准)。一次只改一个参数。

## 1. 速度类（traj_opt/boundary）

| 参数 | 调整 | 现象 | 经验 |
|---|---|---|---|
| max_vel | 0.3 → 1.0 | 0.3 太慢(12.5m 42s)；1.0 流畅 | 短轨迹(<5m)低速，长轨迹(≥10m)1.0；调速先确认 max_acc 匹配 |
| max_acc | 4.0 → 0.6 | 4.0 太激进 → MINCO "Opt failed"(动力学不可行) | 1.0m/s 配 0.6m/s²，加速距离约 0.85m 足够 |
| max_omg | 2.5 → 6.0 | 2.5 绕行急转弯 → "Omg violation" → backup 失败 | 绕行路径急转弯，角速度约束必须放宽 |
| max_acc_thr | 20 → 25 | 20 推力余量不足 → "thr violation" | 推力约束放宽给绕行留余量 |

**核心**：日志 `Opt failed, Omg or thr or Pos violation` = 物理约束太紧 → 先放宽对应约束（omg/thr），**不要**先降速度。

## 2. 平滑类（penna_jerk / acc_ff_smooth）

| 参数 | 调整 | 现象 | 经验 |
|---|---|---|---|
| penna_jerk(主轨迹) | -5e5 → -1e6 → **-7.5e5** | -1e6 太平滑可能拖慢优化；折中 | 负数=最小化jerk；绝对值越大越平滑(不俯冲)但优化越慢/可能失败 |
| penna_jerk(backup) | -1e5 → -5e5 | backup 绕行也需平滑 | backup 紧急，权重比主轨迹低(响应优先) |
| acc_ff_smooth(bridge) | 无 → 0.3 → **0.5** | 0.3 太肉响应慢；0.5 平衡 | 加速度前馈低通系数 0.3~0.7；越大响应越快越易振荡 |

**核心**：俯冲/振荡 = 加速度突变 → 两条路：①轨迹层 penna_jerk 加大(治本) ②控制层 bridge 加速度低通(治标快)——**两个都做**效果最好（双保险）。
低通公式：`输出 = α×新值 + (1-α)×旧值`。

## 3. 走廊类

| 参数 | 调整 | 现象 | 经验 |
|---|---|---|---|
| corridor_line_max_length | 0.25 → 0.4 → 1.2 | 0.25 崩("seed line too long" 抛异常)；1.2 稳定 | 走廊线段太短 → 生成器崩溃；循迹用 1.2 最稳 |
| corridor_small_(代码) | 无 → true/false | true 循迹贴航迹；false 绕行给空间 | **循迹紧(小立方体0.2m硬约束)，绕行松(大盒子0.39m)** 最佳策略 |

## 4. 地图类

| 参数 | 调整 | 现象 | 经验 |
|---|---|---|---|
| cloud_topic | /cloud_registered → /odin1/cloud_slam | odin 建图正常 | 换传感器必须改；cloud_slam 累积点云会"人走障碍残留" |
| inflation_step | 2 → 3 → 回退2 → **又改3** | 3 时 A* 路径离障碍远(绕行可行)但膨胀残留多 | 膨胀 = 安全 vs 空间；**配 cloud_raw(当前帧清除快) 一起用互补** |
| p_miss | 0.49 → 0.6 → 回退0.49 | 0.6 清除快但可能误清 | 清除频率需配合"当前帧点云"才有效；累积点云下无效 |

**核心**：动态障碍刷新 = 点云类型(当前帧) + 清除概率(p_miss) + 膨胀残留 三者配合，**只调一个无效**。

## 5. 终点/触发类

| 参数 | 调整 | 现象 | 经验 |
|---|---|---|---|
| 终点判定(代码) | 无 → 距goal<0.1m | 之前 idx 2点 → 走廊失败 → 终点摆 | 终点判定 = 0.1m 悬停，根治终点振荡 |
| goal_z_min/max | 0.0/0.3(地面) | 地面测试 z 低 | 飞行前必须改回 0.6/0.825 |

## 6. 视觉桥类（odin 专用，odom_to_mavros.py）

| 项 | 调整 | 现象 | 经验 |
|---|---|---|---|
| 发布限速 | 无 → 20Hz(50ms) | 500-1000Hz → PX4 EKF 拒收 → not ready | EKF 有 vision_pose 频率上限，限速 ~20Hz；**频率≠定位精度**(IMU 250Hz 内插，20Hz 视觉修正对 1m/s 足够) |
| 单实例锁 | 无 → pid 文件锁 | 多实例 → vision_pose 6570Hz 打架 | 任何转发/桥接节点都要防多实例(pid 锁最可靠) |
| 时间戳 | rospy.Time.now() | 引入延迟(几十ms→1m/s 5cm 偏差) | 改 msg.header.stamp 前**先确认 use_host_ros_time=1**（主机时间）；=0(设备时间) 千万别用原始时间戳 |

## 7. 调参方法论

```
① 现象归类:
   斜切 = 轨迹约束(vPolytopes/attractor)
   振荡/俯冲 = 加速度突变(轨迹 jerk + 控制低通)
   绕行失败 = 物理约束(omg/thr) 或 走廊空间(大小盒子/inflation)
   不更新/残留 = 地图(点云类型/p_miss/膨胀)
   not ready = 视觉链路(频率/多实例)
② 日志关键行:
   "Opt failed, Omg or thr or Pos violation" → 物理约束
   "GeneratePolytopeFromLine failed" → 走廊/障碍距离
   "The problem is not feasible, min dis" → A* 路径贴障碍
   "SUPER command timeout" → 无轨迹(bridge 悬停)
   "below virtual ground" → 地图原点/高度配置
③ 先配置后代码；一次一个参数
```
