---
name: px4-ugv-rover
description: PX4 无人车/UGV（Rover）方案设计与选型：差速/舵机/麦克纳姆三类底盘取舍、专用固件编译（make *_rover）、模式递进配置（Manual→Acro→Stabilized→Position→Auto）、电机驱动选型（VESC 无感 FOC / RoboClaw / PWM 电调）、无编码器闭环方案（GPS 当速度计）、无人车作无人机载体的起降协同。触发词：无人车、UGV、Rover、麦克纳姆轮/麦轮、差速车、舵机转向车、PX4 驱动电机、要不要编码器、无人机载体。
---

# PX4 无人车（Rover / UGV）方案

## 触发场景
- 用户问 PX4 驱动无人车/UGV（差速、舵机转向、麦克纳姆轮三类底盘）
- 无人车作为无人机载体 / 起降平台 / 移动地面站
- 电机驱动选型（无刷 vs 有刷）、"要不要编码器"、"怎么闭环"

## 核心认知：两个闭环，别混为一谈（回答"PX4 怎么闭环"的第一步）
- **电机层闭环**（电流/转速环）在**电机驱动器**里做（RoboClaw/VESC/电调），PX4 只发命令（UART 指令或 PWM 脉宽）
- **整车层闭环**（速度环/偏航环）在 **PX4 rover 模块**里做，PX4 收反馈→算命令→发下去
- 大白话类比：电机层=司机的脚（精细控制油门），整车层=司机看路开车；编码器=车轮计数器

## Rover 支持现状（官方 main 分支，2026-08 核验）
- 三类底盘：**Differential（差速）/ Ackermann（舵机转向）/ Mecanum（麦克纳姆）**——均 Experimental，文档标注 planned v1.16+
- ⚠️ **必须刷专用固件**：`make px4_fmu-v6x_rover`（把 `_default` 后缀换成 `_rover`）。**无 release 固件，QGC 选不到**，需自己编译（Ubuntu toolchain）
- airframe：Generic Rover Differential `SYS_AUTOSTART=50000`、Generic Rover Ackermann `=51000`、Generic Rover Mecanum `=52000`
- Gazebo 有差速/舵机 rover 仿真，先仿真验证再上真车

## 模式递进（逐级解锁，每级依赖上一级配置好，QGC 里配）
```
Manual(开环前馈) → Acro(偏航率闭环) → Stabilized(偏航闭环) → Position(速度闭环+纯追踪) → Auto(航点任务)
```
- 反馈源分析（**不加编码器的可行性依据**）：
  - 前进速度 = GPS 地面速度（户外够用，RTK 更好）；偏航率 = **IMU 陀螺仪**（飞控自带）；航向 = 磁力计/GPS 航向（自带）
  - **只有"前进速度"需要外部反馈源，GPS 就是现成速度计** → 户外无编码器完全可行
- Manual 模式本身是前馈开环（油门→线性映射电机命令，`RO_MAX_THR_SPEED` 标定），一个反馈都不需要
- 关键参数：`RD_WHEEL_TRACK`（轮距）、`RO_MAX_THR_SPEED`（满油门实测速度填进去=前馈标定）、`RO_ACCEL_LIM`/`RO_DECEL_LIM`、`RO_YAW_RATE_LIM`、`RO_YAW_RATE_P/I`、`RO_YAW_P`、`RO_SPEED_P/I`、`PP_LOOKAHD_GAIN/MIN/MAX`（pure pursuit 前瞻距离）；麦轮用 `RM_WHEEL_TRACK`、`RM_MAX_THR_YAW_R`
- 调参方法同多旋翼：实测法（满油门看地面速度）+ log 判读（plot setpoint vs measured，消息 `RoverVelocityStatus`/`RoverRateStatus`/`RoverAttitudeStatus`）

## 电机驱动选型（"要不要编码器"答案表）
| 驱动器 | 电机 | 闭环方式 | 要编码器吗 |
|---|---|---|---|
| **VESC**（DroneCAN FOC） | 无刷 | 无感 FOC（BEMF 反电动势观测器） | ❌ 不需要 |
| RoboClaw 2x15A（官方 Aion R1） | 有刷 | 速度/位置环**要编码器**（`RBCLW_COUNTS_REV`=每圈计数）；可配开环 duty 模式 | ⚠️ 速度闭环要，开环不要 |
| 普通 PWM 车模电调 | 有刷 | 开环为主（占空比→电压） | ❌ 不需要 |
| 带霍尔无刷电机 | 无刷 | 有感 FOC | ✅ 霍尔内置≠外加编码器 |
- **必须加编码器的场景**：室内/无 GPS、低速高精度停车（起降平台对齐）、打滑检测/里程计融合
- VESC 无感 FOC 软肋：极低速 BEMF 太弱→起步抖/低速扭矩虚；解法=**选带霍尔的无刷电机（有感 FOC），霍尔是内置的，不算外加编码器**

## 麦轮（Mecanum）选型结论：无刷完胜
- 麦轮全向运动 = **4 轮转速的精确合成**（前进同速/斜行各轮不同/原地转组合方向）→ 任何一轮转速偏差方向就歪（数学决定，调参救不了）→ **每轮必须闭环** → 无刷+VESC；有刷开环 4 轮不可能一致，必歪
- 类比：差速=2 人抬桌子，歪了能纠；麦轮=4 人抬桌子，一人步子错立刻斜
- 有刷电刷在频繁横移/换向工况磨损快；麦轮辊子效率低+有刷效率低双重损耗；载重时更明显
- 推荐：**4×带霍尔无刷 + 4×VESC（有感 FOC）+ DroneCAN 总线**（PX4 原生支持，一条 CAN 搞定 4 个驱动器）
- 麦轮局限：硬辊子吃地面平整度，草地/土路/碎石打滑空转——户外下野地的载体场景不适合麦轮

## 底盘选型（无人机载体视角）
- **差速**：首选——结构简单（2 电机）、原地转向、PX4 支持最完善、停车位姿准
- **麦轮**：停车对齐最方便（可横移），贵、挑地面
- **舵机转向（Ackermann）**：像汽车有转弯半径，停车位姿难对准，不适合起降平台
- 载体协同三档：① 车停稳当可移动停机坪（先做，最简单）→ ② 车上放 RTK 基站（移动基准，车随无人机转场）→ ③ 车顶 apriltag 视觉对准降落 / 磁吸机械锁（自动回收、移动中起降）
- offboard 复用：MAVROS `/mavros/setpoint_position/global` 发 GPS 航点，PX4 内部 pure pursuit 自动追踪——用户已有四旋翼 offboard/航点代码框架可直接移植

## 阿克曼（Ackermann）补充（2026-08 核验）
- 配置参数：`RA_WHEEL_BASE`（轴距）、`RA_MAX_STR_ANG`（最大转向角）、`RA_STR_RATE_LIM`（转向速率）；速度环同差速（`RO_SPEED_LIM`/`RO_MAX_THR_SPEED`/`RO_SPEED_P/I`，反馈 GPS 地面速度）
- **转向=舵机 PWM 开环 + IMU 航向闭环（`RO_YAW_P`）纠偏，无需转向角传感器**——又一个"省传感器"点
- ⚠️ **高速急转翻车**：小车轮子会离地。`RO_YAW_RATE_LIM` 官方调法=满油门+满转向跑到轮子离地为止，取不翻车的最大值；车顶载无人机抬高重心→翻车阈值更低→限速再降一档
- 阿克曼适合户外（橡胶轮）但不适合起降平台对齐（不能原地转，要倒车入库式调头）；隐藏优势=可直接买 RC 越野车架改装（现成转向机构+悬挂），但 RC 车自带的航模电调是**无感开环**，闭速度环仍需 VESC

## 电机选型经验值（2kg 级车身，2026-08 方案）
- 无刷减速电机看**输出端规格**：300~400rpm / 扭矩 ≥0.3N·m（按 4kg 载重余量，别按裸车重卡线）；**别只看电机本体 KV**（减速比/电压耦合，绕人）
- 带霍尔电机=8 线（3 相 + 3 霍尔 + 5V + GND），只有 3 线=无感
- ⚠️ **VESC 4.12 原版无霍尔接口**（部分国产兼容板也没有）——要带霍尔电机必须确认驱动器有 hall 输入口（选 VESC 6 规格或国产带霍尔版），且必须支持 DroneCAN
- 组装调试顺序：单轮桌面测试（VESC Tool 设有感 FOC，验证霍尔线序）→ 4×VESC 设不同 Node ID 挂 CAN 菊花链 → 刷 `_rover` 固件 → QGC Actuators 映射逐轮验证转向 → Manual→逐级解锁
- 麦轮采购：必须 **2A+2B 配对**（左旋/右旋辊子对角装），装错整车斜走；铝毂+PU 辊子，别买尼龙辊子

## 导航避障栈（ROS 层，2026-08）
- **PX4 rover Auto 模式=pure pursuit 跟航点，无避障**；避障在机载电脑 ROS 层：move_base（A* 全局 + TEB 局部）→ MAVROS cmd_vel → PX4 offboard
- 感知与规划分层：视觉避障/点云=感知层（"哪里有障碍"），TEB=规划层（"怎么绕"），串用不是二选一；D435/MID360 检测结果喂 costmap 障碍层给 TEB
- TEB=Timed Elastic Band：图优化带时间间隔的轨迹（时间最优+动力学+运动学约束），阿克曼有专门版本；参数 80+，调参费劲

## 坑
- 不刷 `_rover` 固件，默认固件里没有 rover 模块（QGC 也选不到 airframe）
- 老版本（v1.13 及以前）rover 支持简陋；新架构（v1.15 重构）目前只有 main 分支可用
- 多旋翼 PID 经验别照搬：rover 只有速度环+偏航环两层（无姿态环），调参简单得多
- RoboClaw：每控制器唯一地址（默认 128，`RBCLW_ADDRESS`），PX4 **不支持一车挂多个 RoboClaw**（地址参数只有一个）
- PX4 麦轮 Position 模式速度闭环分纵向+横向两路（速度=速度矢量模长），两路共用同一套 RO_SPEED_* 调参

## 参考
- `references/px4-rover-official-docs.md` —— 官方文档路径、参数清单、Aion R1/RoboClaw 接线细节
- 抓 PX4 官方文档的方法（jsdelivr CDN + GitHub API，raw.githubusercontent 被墙）见 px4-hardware-peripherals 技能
