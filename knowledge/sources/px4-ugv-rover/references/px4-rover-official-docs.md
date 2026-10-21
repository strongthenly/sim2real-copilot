# PX4 Rover 官方文档细节（2026-08 抓取核验，main 分支）

## 文档路径（jsdelivr 抓取用，raw.githubusercontent 被墙）
- `https://cdn.jsdelivr.net/gh/PX4/PX4-user_guide@main/en/frames_rover/{index,differential,ackermann,mecanum,aion_r1,rover_position_control}.md`
- `https://cdn.jsdelivr.net/gh/PX4/PX4-user_guide@main/en/config_rover/{differential,ackermann,mecanum}.md`
- `https://cdn.jsdelivr.net/gh/PX4/PX4-user_guide@main/en/flight_modes_rover/`（驱动模式）
- 列目录用 GitHub API：`api.github.com/repos/PX4/PX4-user_guide/contents/en/frames_rover`（可访问）

## 刷 Rover 专用固件（frames_rover/index.md #flashing-the-rover-build）
- "Rovers use a custom build that must be flashed instead of the default PX4 build"
- **没有 release 固件，QGC 里选不到**，必须从 main 分支自己编译：`make px4_fmu-v6x_rover`（`_default` → `_rover`）
- 也可在板级配置 `default.px4board` 里手动启用 rover 模块进默认固件

## Airframe（QGC Airframe 页，选不上就设 SYS_AUTOSTART）
- Generic Rover Differential = `50000`
- Generic Rover Mecanum = `52000`
- （Ackermann 编号未核验，用时抓 airframe_reference 确认）

## 差速车配置参数（config_rover/differential.md，按模式递进顺序）
- 基础：`RD_WHEEL_TRACK`（轮距 m，右轮到左轮中心距）
- Manual：`RO_MAX_THR_SPEED`（满油门实测地面速度，前馈标定）、`RO_ACCEL_LIM`/`RO_DECEL_LIM`（满油门加速实测：极速/到达时间）
- Acro：`RO_YAW_RATE_LIM`（deg/s）、`RD_MAX_THR_YAW_R`（偏航率前馈，取满油门前进速度的一半起步）、`RO_YAW_RATE_P/I`（前馈调好后再调闭环）、`RO_YAW_ACCEL_LIM`/`RO_YAW_DECEL_LIM`
- Stabilized：`RO_YAW_P`（从 1 起步；偏航与偏航率环串级，积分只放偏航率环 `RO_YAW_RATE_I`）
- Position：`RO_SPEED_LIM`、`RO_SPEED_P/I`、`PP_LOOKAHD_GAIN`（小=激进易振荡，起步 1）、`PP_LOOKAHD_MIN`/`PP_LOOKAHD_MAX`（低速振荡升 MIN，高速走不直降 MAX）
- Auto：`RO_DECEL_LIM`+`RO_JERK_LIM` 配对调（到航点平滑减速）；`RD_TRANS_DRV_TRN`/`RD_TRANS_TRN_DRV` 状态机阈值（减速触发角）

## 麦轮特有（config_rover/mecanum.md）
- `RM_WHEEL_TRACK`、`RM_MAX_THR_YAW_R`（对应差速的 RD_*）
- **Position 模式速度控制分纵向+横向两路**，速度=速度矢量模长，两路共用同一套 `RO_SPEED_*` 参数

## log 调参消息（判读 setpoint vs measured）
- `RoverVelocityStatus`：`adjusted_speed_body_x_setpoint` / `measured_speed_body_x`
- `RoverRateStatus`：`adjusted_yaw_rate_setpoint` / `measured_yaw_rate`
- `RoverAttitudeStatus`：`adjusted_yaw_setpoint` / `measured_yaw`
- 方法：跑对应模式 → 停车 disarm → plot 对比 → 前馈参数（MAX_THR_*）调不齐时调对应 P，静态误差升 I

## Aion R1 官方车（frames_rover/aion_r1.md）—— RoboClaw 接线参考
- 配置：Pixhawk + RoboClaw 2x15A（UART 串口，接 GPS2 或 TELEM1）+ Auterion Skynode 机载电脑
- PX4 参数：`RBCLW_SER_CFG`（串口）、`RBCLW_COUNTS_REV`（编码器每圈计数，测试板默认 1200）、`RBCLW_ADDRESS`（总线地址，默认 128）
- **PX4 不支持一车多个 RoboClaw**（RBCLW_ADDRESS 只有一个参数）
- QGC Actuators 里选 RoboClaw 驱动，通道分配数值参考 RoboClaw 手册 "Compatibility Commands / Packet Serial"
- 官方当前仅验证了 MANUAL 模式（该文档撰写时）

## 仿真
- Gazebo 提供 ackermann 和 differential rover 仿真（frames_rover/index.md #simulation）

## 场景结论备忘（源自用户咨询，已沉淀进 SKILL.md）
- 户外 GPS 场景无编码器可行：整车速度环用 GPS 速度，偏航环/偏航率环用 IMU+磁力计
- 麦轮=4 轮精确合成→无刷+VESC 必需；无感 FOC 低速弱→带霍尔有感 FOC
