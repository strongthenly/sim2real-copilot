---
name: px4-hardware-peripherals
description: 解答 PX4 外设/硬件选型问题（直流电机驱动、ESC、传感器、CAN/DroneCAN）并给出官方权威依据；含受限网络下抓取 PX4 官方文档的方法（jsdelivr CDN + GitHub API 替代被墙的 raw.githubusercontent）。适用：标书技术方案、无人机/机器人选型。
---

# PX4 硬件外设选型与官方文档检索

## 触发条件
- 用户问 PX4 支持的电机驱动/ESC/传感器/CAN 外设型号（如"PX4 集成直流电机驱动是哪个型号"）
- 写标书/技术方案需要 PX4 外设的官方权威依据（回答要能给出文档出处）
- 需要确认 PX4 固件是否内置某驱动模块（查 src/drivers）

## 抓取 PX4 官方文档（本机网络环境验证有效）

坑：**docs.px4.io 是 VitePress SPA**——curl 抓 HTML 只有 JS 外壳（约 1MB 全是 hash map），正文不在里面。
坑：**raw.githubusercontent.com 连接被重置**（curl exit 56），不能直接抓 raw 文件。

有效方法（按优先级）：
1. **jsdelivr CDN 抓 markdown 源文件**（首选，正文即 markdown）：
   ```
   curl.exe -s -L "https://cdn.jsdelivr.net/gh/PX4/PX4-user_guide@main/en/peripherals/vesc.md" -o vesc.md
   ```
   路径规则：`https://cdn.jsdelivr.net/gh/{owner}/{repo}@{branch}/{path}`
2. **GitHub API 列目录**（api.github.com 可访问，返回 JSON 里 Select-String '"name"' 提取）：
   ```
   curl.exe -s -L "https://api.github.com/repos/PX4/PX4-user_guide/contents/en/peripherals" | Select-String '"name"'
   ```
   固件源码同理：`/repos/PX4/PX4-Autopilot/contents/src/drivers` 确认驱动是否内置（roboclaw、dshot、px4io、uavcan、tap_esc 等）。

## PX4 文档结构地图（en/ 下）
- `peripherals/`：ESC/电机——esc_motors.md、pwm_escs_and_servo.md、dshot.md、oneshot.md、vesc.md（直流电机 DroneCAN）、uavcan_escs.md（已重定向到 dronecan/escs.md）
- `frames_rover/`：小车——differential.md、mecanum.md、ackermann.md、aion_r1.md（RoboClaw 官方验证页）
- `dronecan/`：DroneCAN ESC 列表——escs.md、sapog.md、zubax_orel.md、holybro_kotleta.md
- `config_rover/`、`flight_modes_rover/`：rover 配置与模式
- `modules/modules_driver.md`：固件内置驱动清单（搜 roboclaw/vesc 驱动说明+参数）

## 已知结论摘要（详见 references/dc-motor-drivers-and-escs.md）
- PX4 集成直流电机驱动 = **RoboClaw 2x15A**（固件 src/drivers/roboclaw，UART，官方 Aion R1 验证）或 **VESC**（DroneCAN FOC）
- 无刷电机走 ESC：DShot/PWM/DroneCAN（Zubax Orel、Holybro Kotleta、Vertiq）
- **无人车/UGV 的电机驱动选型与闭环架构**（VESC 无感 FOC 无编码器方案、RoboClaw 编码器要求、麦轮/差速选型、`make *_rover` 专用固件）→ 见 **px4-ugv-rover** 技能

## 坑
- 旧版文档路径（en/robotics/rover/）在 main 分支已不存在，rover 移到 en/frames_rover/；找不到页面先列目录再抓，别硬猜路径
- markdown 里 `<Redirect to="...">` 表示页面已移动，按目标路径抓
- 回答选型问题时先区分：**有刷直流电机**（RoboClaw/VESC/车用电调）vs **无刷电机**（ESC 电调）——用户常混用"电机驱动"一词，混淆会答错
- 抓回来的 markdown 是英文原文，给用户答案时翻译成中文并附官方出处链接
