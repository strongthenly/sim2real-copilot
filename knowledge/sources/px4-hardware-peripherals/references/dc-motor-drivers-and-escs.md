# PX4 直流电机驱动与 ESC 选型知识库（2026-08 调研）

## 问题：PX4 集成直流电机驱动是哪一个型号？

### 答案 1：RoboClaw 2x15A（Basicmicro）——固件内置驱动，官方验证型号
- PX4 固件源码：`src/drivers/roboclaw`（UART Packet Serial 模式，启动命令 `roboclaw start <UART设备> <波特率>`）
- 官方验证平台：Aion Robotics R1 UGV（PX4 v1.15+），文档原话 "improve driver support for Roboclaw Motor Controllers, **such as the RoboClaw 2x15A**"
- 能力：双通道 15A/路；除驱动电机外**回读轮编码器**，发布到 `wheel_encoders` uORB 话题（差速小车里程计直接可用）
- 关键参数：
  - `RBCLW_SER_CFG`：挂载串口（如 GPS2/TELEM1）
  - `RBCLW_ADDRESS`：总线地址，默认 128
  - `RBCLW_COUNTS_REV`：编码器计数/圈（R1 官方值 1200）
- 限制：PX4 单车只支持 1 个 RoboClaw（地址参数唯一）

### 答案 2：VESC（Vedder ESC）——开源 FOC 电机控制器，DroneCAN 接入
- 官网 vesc-project.com，开源软硬件，PX4 专门文档页 en/peripherals/vesc.md
- 经 CAN 总线（DroneCAN）接入 Pixhawk，支持有刷/无刷直流电机（电动滑板/小车/机器人）
- 硬件型号：VESC 6 系列、VESC 75/300（vesc-project.com/Hardware 在售）
- VESC tool 配置要点：App=No App、VESC ID 从 1 起（注意 UAVCAN ESC Index 从 0 起，恒差 1）、CAN_BAUD_500K、CAN Mode=UAVCAN、飞控侧 UAVCAN_ENABLE=3

### DroneCAN ESC 生态（无刷为主）
Zubax Orel 20/21（Sapog 固件）、Holybro Kotleta 20、Zubax Telega、Vertiq（大功率模块）、RaccoonLab Cyphal/DroneCAN PWM nodes

### 选型口径
- 轮式底盘/履带车 + 要里程计反馈 → RoboClaw 2x15A（固件驱动最成熟）
- 通用 FOC 电机控制 + 已有 CAN 总线 → VESC
- 多旋翼无刷电机 → ESC 电调（DShot/PWM/DroneCAN），**不属于**"直流电机驱动"范畴

## 出处（官方文档页）
- https://docs.px4.io/main/en/frames_rover/aion_r1.html（RoboClaw 接线/参数/官方验证）
- https://docs.px4.io/main/en/peripherals/vesc.html（VESC DroneCAN 配置表）
- https://docs.px4.io/main/en/dronecan/escs.html（DroneCAN ESC 列表）
- https://docs.px4.io/main/en/peripherals/pwm_escs_and_servo.html（PWM 电调通用接法，Pixhawk AUX 轨限 1A 不带载）
- https://docs.px4.io/main/en/modules/modules_driver.html（roboclaw 驱动说明）

## 抓取这些页面用的原始命令（本机已验证）
```
curl.exe -s -L "https://cdn.jsdelivr.net/gh/PX4/PX4-user_guide@main/en/frames_rover/aion_r1.md" -o aion_r1.md
curl.exe -s -L "https://cdn.jsdelivr.net/gh/PX4/PX4-user_guide@main/en/peripherals/vesc.md" -o vesc.md
curl.exe -s -L "https://api.github.com/repos/PX4/PX4-Autopilot/contents/src/drivers" | Select-String '"name"'
```
