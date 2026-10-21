---
name: px4-ulog-analysis
description: 分析 PX4 飞控 .ulg 飞行日志（调参效果评估/振荡故障初查）：pyulog 解析、姿态/角速度跟踪误差、FFT 振荡判定（区分打杆 vs 真振荡）、输出饱和、积分项、陀螺振动、起降段检查，输出"表格+每项文字点评+图表"报告。用户给 .ulg 文件或问"调参效果/日志分析/飞得稳不稳/有没有振荡"时使用。一键脚本 scripts/analyze_ulog.py。
---

# PX4 ULog 飞行日志分析

## 触发场景
- 用户给出 .ulg 文件（PX4 飞行日志，QGC 默认存 SD:/log/log_*.ulg）
- 调 PID 后验证效果（本技能主打场景，用户正在调角速度环 P）
- 飞行异常初查：振荡/高频抖/跟手差/输出饱和/落地弹跳

## 导出/获取 ulg（用户问「怎么导出 ulg 日志」时）
- ulg 存在飞控 SD 卡 `/log/` 目录（QGC 下载时看到的就是它）
- 方式一 MAVLink FTP（在线下载，推荐）：NX 上 `pip install pymavlink`，`mavftp.MAVFTP(m).list_directory('/log')` + `.download(src,dst)`；或 MAVSDK 的 `Ftp` 插件（`ftp.list_directory("/log")`/`ftp.download(...)`）。**飞控 USB 直连 NX 得独立 /dev/ttyACM0 通道，不影响 MAVROS**；MAVROS 已占 TELEM 串口时不能直接复用，需停 MAVROS 或走 USB。pymavlink 版本不同 MAVFTP 方法名可能差（`ls` vs `list_directory`），先 `dir(MAVFTP)` 确认
- 方式二 QGC：MAVROS 配 `gcs_url` UDP 转发到地面站后，QGC Analyze Tools → 日志下载（图形化最省事）
- 方式三 SD 卡直读 / USB 大容量存储（bootloader 下 SD 枚举成 U 盘，或拔卡读卡器）
- 下载后拖进桌面 ulg_report.exe 或 QGC Flight Review 验证
- **自动探测串口+波特率**（写一键脚本时用，别让小白填参数）：USB 串口（/dev/ttyACM*、/dev/ttyUSB*）波特率 CDC 忽略、先试 57600 再 921600；TELEM 串口（/dev/ttyTHS*、/dev/ttyS*）试 57600→921600→115200→460800；每个组合 `mavutil.mavlink_connection(port, baud=baud)` + `wait_heartbeat(timeout=2)`，收到心跳即命中
- **一键脚本 `scripts/get_latest_ulog.sh`**（小白版）：自动装 pymavlink + 自动探测串口/波特率 + 递归找最新 .ulg 下载到 ~/ulog/，输出【成功/失败】+ 排查清单（飞控没上电/USB 充电线/MAVROS 占用串口）。fouruav 上 `bash get_latest_ulog.sh`；Windows 端另有双击版 `一键下载日志.bat`（plink 自动连 fouruav 执行，含 pscp 传脚本）。给非技术采集员时配傻瓜化：一键脚本 + 失败排查清单，不解释原理

## 快速开始
1. 确认文件存在与大小（~3MB ≈ 1-2 分钟飞行）
2. 安装依赖：`pip install pyulog numpy plotly matplotlib -i https://pypi.tuna.tsinghua.edu.cn/simple`
3. **首选：交互式 HTML 报告**（用户点名要"可视化强+文字专业"，必须走这条）：
   `python C:\Users\<USER>\Desktop\ulg_report.py <log.ulg> [多个文件]`（或把 .ulg 拖到脚本上）
   输出 `<名>_report.html`（自包含单文件 ~7.5MB，Plotly 交互：悬停数值/缩放/图例开关），8 节内容 + 每节自动分级点评 + 末尾综合建议。架构/阈值/重建要点见 references/ulg-report-tool.md
4. 需要纯控制台统计/深挖原始数据时用技能自带脚本：`python <skill_dir>/scripts/analyze_ulog.py <log.ulg> [outdir]`（5 张 PNG + 控制台统计）

## 日志身份识别（ver_hw/固件版本/哪台机器，2026-08-20 实战）
- **用途**：确认 .ulg 来自哪个飞控/哪台机器、固件版本，回答"该下载哪个固件""这是哪台机的日志"类问题
- **方法（原始字节正则，比严格解析稳）**：QGC 下载的 .ulg 头可能不是标准 ULog v0（本机实测 version=1、B 消息 framing 非标，自写严格解析器两次失败），但 info 字段（如 `char[14] ver_hw` 后紧邻 ASCII 值）用正则扫原始字节即可拿到：
  `python <skill_dir>/scripts/ulg_header_probe.py <log.ulg>`（提取 ver_hw/ver_sw/sys_mcu/sys_name/sys_uuid）
- **ver_hw → PX4 release 固件资产映射（v1.17.0 实测）**：
  | ver_hw | sys_mcu | 固件资产 |
  |---|---|---|
  | HKUST_NXT_DUAL | STM32H7[4\|5]xxx | `hkust_nxt-dual_default.px4`（boards/hkust/nxt-dual） |
  | PX4_FMU_V5 | STM32F76xxx | `px4_fmu-v5_default.px4`（Pixhawk 4） |
- **别信用户记忆的型号名**：用户报"微空 nxtpx4v2"，PX4 官方 boards 目录无此名（hkust 下只有 nxt-dual/nxt-v1）——以 ver_hw 为准。拉 release 资产列表：`Invoke-RestMethod https://api.github.com/repos/PX4/PX4-Autopilot/releases/tags/vX.Y.Z`
- **⚠️ 不同日志可能来自不同机器**：桌面 01_57_39.ulg（08-18）ver_hw=PX4_FMU_V5，而 fouruav 的 log_360/log_odin（08-17）是 HKUST_NXT_DUAL——引用案例结论前先对 ver_hw 确认日志出自哪台
- 刷固件会清参数：先 QGC 导出 .params；优先用 QGC 自动刷（识别板子），手动下载用 release 资产直链

## 判读基准（核心价值）
| 指标 | 优秀 | 警戒 |
|---|---|---|
| 角速度误差 RMS | <0.5°/s（实测 P=0.45 时 0.14） | >2°/s → 内环 P 不足 |
| 姿态误差 RMS（Stabilize 打杆） | 1-2° 正常 | >5° → 跟手差/环太软 |
| 4-8Hz 频带能量占比(悬停段) | <15% 安全（实测 1.7%） | >15% 需关注, >30% 且 rate RMS>1 = 真振荡 |
| 扭矩饱和率（\|T\|>0.9 占比） | ≈0 | 高 → 打杆过猛/电机余量不足 |
| 电机巡航输出 | <0.8 | 顶 1.0 → 推力余量不足 |
| 角速度积分项 vs 限幅(±0.3) | 远小于限幅 | 贴限幅 → 重心偏/配平差 |
| 陀螺 30-80Hz 能量占比 | <10% | 峰频率×60=电机 RPM，检查动平衡 |
| 加速度振动 RMS（水平） | <50mg(0.5 m/s²) | >100mg → 桨/电机振动大 |
| 单芯电压（最低电压/S 数） | ≥3.7V 健康 | 3.5-3.7 警戒, <3.5 深度放电 |

## 报告格式（用户偏好，必须遵守）
- **首选交互式 HTML 报告**（ulg_report.py）：Plotly 图表（悬停看数值/缩放/图例开关）+ 每节自动分级点评，浏览器渲染中文无乱码问题
- 手写报告时：**表格 + 每项指标文字点评（好坏+原因+改进建议）**，不能只给数字和图表
- 点评要专业（闭环相位滞后/增益裕度/功率谱/执行机构余量等术语），关键处给大白话注脚或类比（开车等日常场景）
- 图表用 `MEDIA:` 嵌入（PNG 存日志同目录 analysis_<name>/）；matplotlib 图**用英文标签**（中文渲染乱码）
- 结论先行：第一行给"调参成功/失败"的明确结论；被问"有没有振荡"必须给确定判定（如"非振荡，是打杆操作频带"），不要含糊
- 等待必须逐步进度反馈（工具已打印 [1/4]~[4/4]）

## 用户指标偏好（已明确要求，2026-08）
- **电流数据不可信**：用户电流计数据不可靠，报告/分析一律不用 `current_a`、平均电流、功耗(电流×电压)——电池分析只用电压维度（起始/最低电压、压降速率、单芯电压、cell_count）
- **不要预估飞行时间**：用户明确要求移除剩余时间估算（SOC 速率法/电压外推都不要）
- **调参文档/建议里不写电池相关内容**（2026-08-17 明确要求：写飞控调参参数手册时点名删掉"电池别飞到底"守则，"不需要电池相关的描述"）——报告第 7 节电源状态可保留，但面向用户的调参文档/口头建议少提电池
- **图表操作方式**：`dragmode='pan'`（左键按住拖动=平移、滚轮=缩放），不要默认的框选缩放
- 指标要克制：只给可信度高的指标，宁可少给也不给误导性数据

## 阈值来源可信度（用户问过"依据是什么"，如实分层回答）
- 硬标准（有据可查）：单芯电压 3.7/3.5V（锂聚合物行业标准）、水平振动 50/100mg（ArduPilot 官方振动指南）、30-80Hz 频带（电机基频物理常识）
- 用户文档标准：延迟 30/50/70ms、姿态 100ms（来自用户 B2飞控调参.md）
- 经验启发式（无权威出处，用用户实测好日志校准过，阈值写在脚本里可改）：角速度 RMS 0.5/1.5/3 档、4-8Hz 能量占比 15%/30%、饱和率 5%、积分项 80% 限幅、稳态静差 0.8°/到位率 70%——回答时必须如实说明这是经验档位而非标准

## 工作流细节
1. **概览**：时长、topic 列表与频率、MC_*/MPC_* 参数快照（对照 PX4 默认值找出用户调了什么）
2. **模式判定**：vehicle_control_mode flags（见 Pitfalls）→ 决定哪些环在线（Stabilize 只有姿态+角速度环，没有位置/速度环）
3. **飞行段**：由 actuator_armed / vehicle_land_detected 自动切出，避开起降瞬态
4. **跟踪误差**：四元数转欧拉角（setpoint 插值到 actual 时间轴），统计 RMS/峰值；按角速度阈值 30°/s(=0.52 rad/s) 分悬停/机动段
5. **振荡检查**：rate 误差 FFT + 4-8Hz 频带能量占比（见 Pitfalls 判定法）
6. **饱和/积分/振动**：扭矩与电机输出饱和率、积分项贴限幅否、陀螺 30-80Hz 频带能量与峰频率
7. **起降段**：起飞/落地瞬间 rate 误差峰值（落地 15-20°/s 瞬态=触地正常）
8. 汇总报告（见上）

## Pitfalls

### pyulog 新版 API（v1.x，必看）
- 构造：`ULog(path, disable_str_exceptions=True)`；旧参数 `message_name_filter_out` 已改名 `message_name_filter_list`
- `ulog.data_list[i].data` 是 **dict[str, numpy.ndarray]**（不是结构化数组），**没有 `.timestamps` 属性**；取时间戳用 `d.data['timestamp']`
- 字段名带方括号：`'q[0]'`、`'xyz[0]'`、`'control[0]'`、`'actuator_saturation[0]'`、`'arguments[0]'`
- event 话题字段名是 `'id'`（不是 `event_id`），另有 `'log_levels'`
- **event 话题的事件翻译（2026-08 实机）**：`id` 是 PX4 事件编号（如 16856624/27825120 这种大数），**需查对应固件的 event_messages.h 才能翻译成文字**；`arguments[0..N]` 是事件参数（如模式切换请求的 custom_mode 值）。`id=0` 是"无事件"直接跳过；大数 id 无法从 pyulog 直接解析出含义——可打印 id+arguments 时间线供对照（如模式请求时刻的 arg 变化），要文字含义需拉固件源码。时间戳同样有 2^64 垃圾值陷阱，先过滤
- 关键话题字段：vehicle_attitude=`q[0..3]`(四元数)；vehicle_attitude_setpoint=`q_d[0..3]`；vehicle_rates_setpoint=`roll/pitch/yaw`(**rad/s**)；vehicle_angular_velocity=`xyz[0..2]`(rad/s)；rate_ctrl_status=`rollspeed_integ/pitchspeed_integ/yawspeed_integ`
- **微空 nxtpx4v2 定制固件 PX4 v1.17 字段名差异（2026-08 实机）**：`sensor_baro` **没有 `altitude` 字段**（只有 `pressure/temperature`），气压高度需自己换算 `alt=44330*(1-(p/101325)^0.1903)`（QNH 按 `SENS_BARO_QNH`）；`manual_control_setpoint` 油门字段是 **`throttle`**（不是 `z`）；电机输出在 `actuator_motors.control[0..]`（不是 actuator_controls）；`estimator_status_flags` 是 `cs_*` bool 字段（`cs_fake_hgt`/`cs_inertial_dead_reckoning`/`cs_ev_pos`）。**写脚本前先 `python -c` 打印目标话题 `list(d.data.keys())` 确认字段**，避免 KeyError 反复崩（本类实机踩过 3 次）
- **uint64 时间戳陷阱**：个别话题（event/land_detected）首样本时间戳可能是 2^64 附近垃圾值，减法会溢出成 1.8e19 s——所有时间运算前先过滤 `ts >= start_timestamp`，或直接用 `(t>=FLY0)&(t<=FLY1)` 掩码天然排除
- **`vehicle_land_detected` 着陆检测三坑（log_144 实战，2026-08）**：
  1. 日志开头 landed=1 是开机初始态（t≈0 启动默认值）
  2. **起飞前可能持续 landed=1 数秒**（地面准备阶段）——取"第一个 landed 事件"会把地面段当飞行段 → 飞行段变 0.02s → 所有指标空数组崩溃（"zero-size array to reduction operation maximum"）
  3. 取"最后一个 landed=1 样本"又会在着陆后把地面停留段计入飞行段
  正解：**取起飞(armed+1s)后最后一次 0→1 跳变**（`np.diff(np.concatenate([[0.0], landv]))>0` 找上升沿），无跳变（日志中途停止）才兜底最后一个样本
- **悬停段选取必须限制在飞行段中部**：滑动 5s 窗口选角速度能量最小段时，若不先掩掉飞行段外数据，会选到着陆后静止段/地面段，振荡判定失真
- **Plotly HTML 报告里大量 "NaN" 字符串**是 plotly.js 库源码（isNaN）自带，非数据问题，别误判为数据异常
- **`ulog.initial_parameters` / `changed_parameters` 是 dict {name: value}**（pyulog v1.x），不是 list——`for p in ps` 遍历到的是 key 字符串，取 p[0] 得到首字符、p[1] 越界/短路。用 `for n, v in ps.items()` 或 `ps.get('MC_PITCH_P')`（log_52 踩过）
- **`np.interp` 不支持二维数组**：`np.interp(t_att, t_r, w)` 对 (n,3) 数组报 "object too deep for desired array"——逐列插值 `np.column_stack([np.interp(t, t_r, w[:, i]) for i in range(3)])`
- **uint64 时间戳垃圾值会废掉频率计算**：`np.diff(ts)` 出现巨值 → median 巨大 → 频率显示 0.0 Hz；且 `np.isfinite()` 对 int 数组不过滤（int 恒 True）——频率别纠结，直接用 n/时长估算（如 vehicle_angular_velocity 4155/84s≈50Hz 是 SD 记录率，非控制率）
- **话题字段名必须逐个确认**：本固件 `sensor_gyro` 无 `xyz[0..2]`（KeyError），而 vehicle_rates_setpoint 有 roll/pitch/yaw——技能已强调"先打印 list(d.data.keys())"，log_52 又踩一次，别跳过这步

### 模式判定
- 用 `vehicle_control_mode` 的 `flag_*_enabled` 判断模式：manual+attitude+rates=Stabilize；manual+altitude=定高；**不要硬编码 nav_state 数字**（版本间枚举不一致）
- `estimator_aid_src_fake_pos/fake_hgt` 在订阅列表出现 = 无有效 GPS/气压计 → 位置/速度控制不可用，只能手动模式；报告里要提醒用户飞定点前先查传感器源

### 模式/定位诊断（2026-08 实战：POSCTL 切不进去 + "No valid global position estimate"）
- **判断"模式切换是否被接受"**：看 `vehicle_status.nav_state` 全程是否变化。用户报"切 POSCTL 但过一会回 HOLD"，真相常是 **nav_state 全程没变 = 切换请求从未被接受**（QGC/遥控器显示的是"想要"的模式，飞控实际保持原模式）。辅证：`vehicle_control_mode` 仅 `flag_control_manual/attitude/rates=1`、无 position/velocity 环 = 位置控制从未上线
- **event 时间线定位"模式切换请求"（2026-08 实机补充）**：`event` 话题按 `id`+`arguments[0]` 排时间线——模式请求时刻会出现 event（如 t=10.1s 时 `arg0=234` 类 custom_mode 值），对照 `nav_state` 是否响应（变了=接受，没变=拒绝）。**event id 是大数需查固件 event_messages.h，但 arguments[0] 的数值变化本身就是"有模式请求发生"的信号**——即使翻译不了 id，也能判断"请求发出但被拒"
- **nav_state 枚举（PX4 v1.14+，仅供快速参考）**：0=MANUAL 1=ALTCTL 2=POSCTL 3=AUTO_MISSION 4=AUTO_LOITER 5=AUTO_RTL 6=AUTO_LAND 8=AUTO_READY 9=AUTO_TAKEOFF 12=AUTO_FOLLOW_TARGET 14=AUTO_PRECLAND 15=ORBIT 16/17=VTOL_T/O 24=AUTO_LOITER_RC。**15 是 ORBIT 不是 HOLD**；微空定制固件可能自定义枚举——用 `vehicle_status.valid_nav_states_mask` 位集合反查最可靠（`mask & (1<<bit)` 输出所有允许的模式）
- **EKF 视觉融合状态**：`estimator_aid_src_ev_pos.fused` 连续=1 且 `time_last_fuse` 递增 = 视觉位置被融合；`estimator_status_flags.cs_ev_pos/cs_ev_hgt/cs_ev_yaw`=1 = 视觉辅助启用；`cs_constant_pos`=1 = const_pos 恒定位置模式（无外部位置源兜底）
- **⚠️ `/mavros/estimator_status` 的 status_flag 布局不可靠**：ros 话题 flag 位定义与 .ulg 内 `estimator_status_flags` 字段不同——实测 ros 侧显示 `const_pos_mode=True`（误判 EKF 无位置），.ulg 显示 `cs_ev_pos=1 fused=1`（视觉融合全程正常）。**定位诊断以 .ulg 为准，ros 话题仅参考**
- **fake_pos 陷阱**：`estimator_aid_src_fake_pos.fused=1` = EKF 在用"假位置"（无真实位置源时的虚拟位置），与视觉位置并存是不健康状态
- **⚠️ fake 话题存在 ≠ 在用假位置（ulg_report.py 2026-08-17 修复实锤，log_73）**：微空固件 `estimator_aid_src_fake_pos/fake_hgt` 话题常存在（开机瞬间 1 个样本 fused=1），但 `estimator_status_flags.cs_fake_pos/cs_fake_hgt` 全程 0% = 从未使用假位置（log_73 视觉定位 ev_pos/ev_hgt/ev_yaw 融合 100%、定点 19 分钟位置锁定 1-7cm 却被旧逻辑误报"使用替代源"）。**判定"是否在用假源"以 cs_fake_* 标志为准（>50% on 才算），不能看话题是否存在**。视觉是否工作看 `estimator_aid_src_ev_*.fused` + `cs_ev_*` 标志
- **"No valid global position estimate"** = 无 GPS 全球位置（经纬度）。AUTO.* 任务模式需要 global position；**POSCTL 只需 local position**（视觉/IMU 可提供）——无 GPS 时切 AUTO.POSCTL 被拒是预期行为，切 POSCTL 是否可行取决于固件
- **PX4 1.17 参数换代**：`EKF2_AID_MASK`/`EKF2_NOAID_MASK` 在 1.17 可能已不存在（param/get 报 Unknown 或 QGC 搜不到），视觉融合由 `EKF2_EV_CTRL` 控制（11=位置+高度+偏航）。**排查前先 param/get 确认参数存在**，别拿废弃参数名查半天

### 定点模式持续偏移诊断（POSCTL 朝一个方向漂，2026-08-18 实锤：先排除机械故障，再谈视觉 yaw）
⚠️ **2026-08-20 复核：01_57_39.ulg 的 ver_hw=PX4_FMU_V5（STM32F76xxx=Pixhawk 4），而 fouruav 8-17 日志（log_360/log_odin）ver_hw=HKUST_NXT_DUAL——本案例硬件与 fouruav 现役飞控不一致，可能出自另一台机器或换过飞控；引用结论前先对 ver_hw 确认机器**。
用户报"定点模式飞行时飞机一直朝一个方向偏移"。**机载 ROS 日志只能证明 vision_pose 链路通不通（见 uav-ros-diagnostics 技能），偏移根因必须从 .ulg 判别**。
- **⚠️ 第一步先确认日志真是定点**：`vehicle_control_mode.flag_control_position_enabled` 全程=1 才是定点。用户给的"定点偏移"日志可能混入手动日志（log 01_58_35 实际 manual=1 pos=0 = Stabilize 自稳，位置环从未上线，白分析一轮）——**位置环没在线就没有"定点偏移"可言**
- **⚠️ 最大坑：别急着判"视觉 yaw 漂移污染 EKF"（本案例两轮误判被用户反证推翻）**：
  - 视觉 yaw 卡错 18°（静止时视觉 76.8° vs 实际 58.2°）+ heading innovation>1 被拒 + yaw 扭矩 0.55 饱和，**看着像视觉 yaw 源头漂移，实际是机械振动导致 FAST-LIO 配准退化的症状**
  - **"位置 innovation<1 + heading 被拒"只能证明"EKF 认为位置融合健康、yaw 测量可疑"，无法区分"视觉源头漂"还是"机身真在剧烈振动"** —— 两者 heading 被拒表现相同
  - **三源同步大转 ≠ 一定是 EKF 接受视觉漂移**：机身物理真转时三源当然同步
- **机械故障实锤四件套（先查这些）**：
  1. **电机对角饱和**：悬停四电机均衡（0.33-0.40）→ 0.1s 内 M3/M4 被打到 0.00、M1/M2 拉高 0.6+ = 姿态环拼死抵抗无法控制的外部扰动（log 01_57_39 9.44s 突变）
  2. **姿态环打满拉不住**：tq_z 0.55（限幅 0.2 的 3 倍）+ yaw 速率指令与实测**方向相反**（指令 +89°/s 实测 -83°/s）= 外部力矩远超控制力；若是估计/控制带偏，运动应跟随指令方向
  3. **振动分段放大**：vehicle_acceleration 各轴 RMS 分段统计（起飞前 0.19 → 起飞后 1.14 → 2.92），gyro 峰值 123°/s，停桨后归零
  4. **imbalanced_prop_metric 全程非零**（27/27，t=0 起）＝ PX4 桨不平衡检测全程报警
- **加速度冲击 ≠ 撞击**：az/ay 突变（-15.6/-9.7）像撞障碍物，但用户确认无障碍物 → 重查振动分段+电机分配，区分"单次撞击" vs "振动爆发"
- **用户反证 = 最高优先级证据**："之前没这问题"+"另一架同配置没这问题"+"附近没有障碍物"出现任何一个，都要重查数据而非坚持结论——用户对现场的把控是日志之外最硬的信息
- **若四项全排除才是视觉 yaw 漂移**：视觉 yaw（`estimator_visual_odometry_aligned` q→欧拉）独漂/领先 + 实际姿态不动 = 纯视觉假漂移；三表对拍（视觉/`vehicle_attitude`/`vehicle_local_position.heading`）+ `estimator_innovation_test_ratios` heading>1 vs ev_hpos<1 定坏轴；处置 = 切自稳最安全、起飞后悬停 10-20s 等 SLAM 收敛
- 正确完整案例（证据链+教训）：`references/posctl-drift-mechanical-case.md`；误判教训存档：`references/visual-yaw-drift-diagnosis.md`（旧结论已推翻）；一键探针（含电机/振动判定）：`scripts/posctl_drift_probe.py <log.ulg>`

### 避障参数锁死 POSCTL（2026-08 fouruav 最终根因，伪装成定位问题）
- **现象**：切 POSCTL 能切但几秒后自动回退 HOLD，MAVROS 报 "No valid global position estimate"——**不一定是定位问题！** 本次排查 4 步弯路（EKF flag / AID_MASK / 视觉话题 / 坐标系全正常）后才定位到避障
- **根因**：`CP_DIST=1`（碰撞预防开启）+ `CP_GO_NO_DATA=0`（无数据方向不可飞）+ `obstacle_to_px4` 数据源未运行 → PX4 认为全向 1m 内都是障碍 → 拒绝位置控制 → 自动回退
- **速查命令**：`rosservice call /mavros/param/get "{param_id: 'CP_DIST'}"`（>0=开了避障）；`ps aux | grep obstacle_to_px4` + `rostopic hz /mavros/obstacle/send`（数据源有无）；参数文件里同时看 `CP_GO_NO_DATA`（0=无数据不可飞，1=可飞）
- **用户给 QGC 导出参数文件（`*.param`，格式 `参数名,值` 每行）时**：用 search_files 正则一次提取关键参数，比逐个 rosservice 快得多——`^(COM_FLTMODE|COM_FAIL|COM_RC|CP_|EKF2_EV|MPC_POS|RC_MAP_MODE|GPS_|CBRK_IO)`。判读优先级：**CP_DIST/CP_GO_NO_DATA（避障锁死）> COM_FLTMODE1-6（遥控器模式开关映射，如 =2 是 POSCTL 档）> EKF2_EV_CTRL（视觉融合）> COM_RC_LOSS_T（RC 丢失超时）**
- **正解**：不用避障就 `CP_DIST=-1`（负值=关闭）；要用避障必须配齐数据源且建议 `CP_GO_NO_DATA=1`
- 详细排查文档已存 fouruav：`task_run_scripts/scripts/NX_code/obstacle_warning.md`（2026-08-11 提交）

### 高度锁定/自稳突然爬升诊断（2026-08-15 实机："高度一直 103.8m" + "自稳突然上升"）
- **"高度一直 X m"（如 103.8）首要分辨：QGC 显示的是绝对海拔(AMSL)还是相对高度**。非整数大值通常是 AMSL≈场地海拔；AMSL 钉死不动 = **`EKF2_BARO_CTRL=0`（气压计融合被关）**——EKF 只在开机时用 baro 初始化一次绝对高度，之后永不更新（相对高度 local z 仍会变，别拿它否定）
- **高度源实锤三件套**：① `estimator_aid_src_fake_hgt.fused` 密集=1（假高度源兜底=无真实高度源）；② `cs_fake_hgt` 全程 1；③ `cs_inertial_dead_reckoning` 全程 1（纯惯性推算）。有 `estimator_aid_src_baro_hgt` 但 fused=0 = baro 融合被关
- **`EKF2_HGT_REF=3`（首选视觉）+ 无任何 `estimator_aid_src_ev_*` 话题 = 空配置**：视觉根本没进 EKF（MAVROS vision 没发/没连），别被参数骗；EV_CTRL 只是"想融合"，是否真融合以 aid_src 话题 fused 为准
- **判真高以 baro 原始压力换算高度为准**（标准大气公式），EKF 高度仅参考。姿态大角度（俯仰/横滚>30°）时纯惯性 EKF 高度/vz 被加速度积分骗飞（实机：z 虚标 15m、vz 8.7m/s，baro 实际只动 3m）
- **自稳模式突然爬升 = 悬停油门错配**：自稳油门直通，中位杆输出≈`MPC_MANTHR_MIN+(1-MPC_MANTHR_MIN)*0.5`；若实际悬停油门（`hover_thrust_estimate.hover_thrust`，如 0.31）远小于中位输出，杆回中位持续爬升（vz 可达 -3.9m/s）。判读：找 vz≈0 时的手动油门杆值 = 悬停点，对比 `MPC_THR_HOVER`
- **修复（治本）**：`EKF2_BARO_CTRL=1` + `EKF2_HGT_REF=0`（恢复气压计高度源）+ `MPC_THR_HOVER`≈hover_thrust 实测值，改完重启飞控
- 完整案例（参数证据表+时间线+判读法）：`references/height-source-diagnosis.md`；一键探查脚本：`scripts/altitude_source_probe.py <log.ulg>`

### 振荡判定（本技能最重要思维）
- **FFT 原始峰幅度会骗人**：人打杆的操作频率集中在 1-5Hz，手动飞行时 3-5Hz 一定有峰，幅度比频带均值高 2-4 倍≠振荡
- 真振荡特征：悬停时持续存在 + 窄尖峰 + rate 误差 RMS 大（>2°/s）
- 打杆特征：峰随机动出现 + 绝对幅度极小（rate 误差 RMS <0.5°/s 时任何"峰"都是噪声）+ 4-8Hz 频带能量占比 <2%
- 判定公式优先用**频带能量占比**而非峰值幅度

### 角速度尖刺定性（2026-08-17 log_82 实战：用户报"roll 尖刺特别多"实为 3 个事件）
- **检测法**：相邻样本跳变 >50°/s（50Hz 记录）找索引，间隔 ≤2 合并成事件，统计尖峰值/宽度/时间分布；事件少但用户觉得"多"时降阈值（>15°/s）复查——**落地衰减振荡（0.5s 内来回 ±30°/s）看起来像一堆尖刺，实为一次事件**
- **交叉定性四件套**：① 尖刺时刻杆量（回中=非打杆）② 油门轨迹（猛收 = 下降/落地；log_82 油门 -0.34→-1.0）③ land_detected 前后跳变（32.3s landed=1 确认落地）④ 三轴是否同时尖刺（roll/pitch/yaw 同刻 = 机械冲击特征，非单轴控制问题）
- **判定**：杆回中 + 收油门 + 三轴同时尖刺 + 0.5s 内角速度收敛 = **起飞/落地机械冲击，正常**（不是控制振荡、不是传感器故障）；间隔无规律 = 事件性冲击；间隔规律 ≈ 电机转速相关 = 机械不平衡
- **振动异常看轴间不对称**：加速度振动 X/Y/Z 某轴明显大（log_82 Y=1.02 m/s² vs X=0.29，3.5 倍）= 桨/电机/机架机械不平衡（优先查桨动平衡），**不是 PID 问题**；振动大 → 陀螺噪声经 D 项放大成尖刺，**修好振动前别加角速度增益**

### PowerShell 环境（Windows 主机）
- `curl` 是 Invoke-WebRequest 别名 → 必须用 `curl.exe`
- `python -c "多行带引号代码"` 在 PowerShell 必炸（引号转义地狱）→ **用 write_file 写临时 .py 再执行**
- 输出较长时看尾部用 `Select-Object -Last N`（前面可能是参数表）
- 搜索引擎对国内小众硬件基本无效（本类任务别浪费时间搜板子资料，直接按固件分析）

### 飞行段检测 4 坑（ulg_report.py 2026-08-15 实战：log_52 报告"实飞 2s"实际 79s，全指标作废）
- **坑4（实锤）：land 话题可能比 actuator_armed 多记录零点几秒**——落地上升沿 t 可能 > t1_arm+0.5，用 `t1_arm+0.5` 做上升沿窗口上界会**误杀真落地沿** → 兜底误取"开机初始态 landed=1 段"当飞行段（报告实飞 2s 实际 79s），延迟/静差/到位率全废
- **修复**：上升沿窗口上界改用 land 话题自身末尾 `t_ld[-1]`；**起点用起飞下降沿(1->0)+0.2s** 剔除解锁后地面准备段（log_52 地面 3.3s）；全程 landed=1（从未起飞）→ 无飞行段（fly=None），不再误报
- **验证法**：报告"实飞段"数字 vs `vehicle_local_position.z` 是否飞到 -1m 以上——不一致即飞行段检测出错，先查 land 时间戳/窗口

### 延迟测量 3 坑（ulg_report.py 2026-08-15 实战：log_52 pitch 互相关 168ms vs Flight Review 20ms）
- **坑1：互相关作用在全日志范围**（含地面段/悬停微动段）→ 必须传飞行段窗口 t_win 截断
- **坑2：活动段掩码(actv)计算了但没应用**——互相关仍用全时段信号，活动段只用于样本数检查
- **坑3（本质）：互相关测的是"整体相位滞后"不是"起步延迟"**——对打杆柔和的日志系统性高估（log_52 pitch 84~147ms 而 Flight Review 起步仅 ~20ms）。方法可信度：参数正常的日志（log_148）互相关 21~42ms ≈ Flight Review 20ms ✓；钝系统（P低D高）高估。**判定法：此指标仅用于轴间相对对比（哪轴明显更钝），绝对值不直接对应手感延迟**
- **报告措辞铁律**：不许写"延迟偏大→加 RATE_D"——这正是 D 越加越大越钝的元凶建议（log_148→log_52 期间 D 0.002→0.01 就是这么加出来的）。滞后大的轴：RATE_P 低则加 P(+0.03~0.05)、RATE_D 高则减 D(-0.002~0.003)

### 打杆阶跃响应分析（ulg_report.py 2026-08-17 新增：角速度+角度的真实响应延迟）
- **功能**：报告第 3 节新增"打杆阶跃响应"子块——表格（每轴 阶跃数/角速度跟期望/超调/角度跟期望/90%到位）+ 6 面板交互图（每轴 左角速度 右角度，期望vs实际，虚线=阶跃时刻）
- **主指标 = "期望→实际"**：期望角速度开始变→实际开始变（内环跟随延迟，正常 20-40ms，与 Flight Review 同量级）。**绝不用 stick(manual_control_setpoint) 定位打杆时刻**——该话题常只有 5Hz 采样，50% 插值仍偏差 ±100ms，测"打杆→动"根本不准（log_52 实测全负值）
- **检测法**：扫描期望角速度脉冲（|值|>8°/s 持续≥80ms），合并间隔<1.5s，按幅值取前3；**只统计"基线静止(|期望|<3°/s) + 上升沿陡(0.1s 变化>5°/s)"的干净阶跃**——否则会把持续压杆机动段（期望角速度长时间>8°/s）误当阶跃，基线污染导致 first_cross 全乱
- **穿越检测**：1kHz 插值 + 首次连续 3 点过 10% 幅值阈值 + 线性插值求时刻；起点从窗口开头(w0=脉冲起点-0.25s)找，不依赖 stick
- **角度**：期望角度阶跃幅值(脉冲后 0.6~1.0s 稳态-基线)≥0.8° 才算有目标角；att_90 相对期望角度阶跃时刻
- **判读校准**：log_148(参数好) roll 27ms/pitch 29ms ≈ Flight Review 20ms ✓；log_52(机动飞行无阶跃) pitch 无数据=诚实结果，提示做阶跃测试
- **⚠️ 阶跃法 vs 互相关法两个"延迟"必须分开判读（log_69 实战，2026-08-17）**：阶跃"跟期望"= 快速打杆真实响应（20-40ms 达标，Flight Review 同量级）；互相关"整体相位滞后"= 全时段统计（含悬停微动、滤波固有滞后、积分项低频贡献）——**互相关值偏高 ≠ 打杆延迟高**，用户说"延迟要压到 30ms 内"先看阶跃法。实测：pitch 阶跃 25ms（达标）但互相关 42ms，两指标量级关系随参数变化，不可互换
- **某轴互相关滞后明显大于另一轴（角速度参数已对称）的排查顺序（log_69 实锤：pitch 42ms vs roll 21ms）**：① 姿态环 P 不对称（MC_PITCH_P 4.2 vs MC_ROLL_P 5.3——姿态环是串级带宽瓶颈，两轴角速度参数相同而滞后差一倍，先查这个）② 角速度 I 极低（I=0.009=默认 0.15 的 1/20 → 压杆长期误差补不上 → 低频滞后大，建议回 0.05+）③ DGyro 滤波固有滞后（IMU_DGYRO_CUTOFF=20Hz 在 3-5Hz 引入 5-8ms 固定相位滞后，**PID 调不掉只能放宽滤波**，FFT 确认振动尖刺频率后放宽，如尖刺 48Hz 时放宽到 25Hz 安全）——**不要靠继续加角速度 P/D 压滞后**（P=0.48 已默认 3 倍）
- **负超调（欠跟随）判读**：实际角速度峰值/期望峰值 <1 = 过度阻尼（D 高，log_67 roll D=0.016 → -30%）或滤波衰减；加大 P 效果有限，优先降 D 或放宽 DGyro
- **坑**：off-by-one（dstick[s]=|stick[s+1]-stick[s]|，阶跃在 stick[s]→stick[s+1]）；列表含 None 时 np.mean 崩溃（过滤后再算）

### 互相关延迟成分诊断：阈值扫描法（2026-08-17 log_70 实战，判断"42ms 是不是真延迟"）
- **症状**：某轴互相关滞后稳定偏大（如 pitch 42ms），多轮调参（P/I/D/滤波全动过）**纹丝不动**，而阶跃法该轴已达标（25-32ms）——互相关含慢变段/小幅修正段贡献，可能掩盖真实响应
- **诊断法**：对同一轴做**不同活动阈值**（期望角速度变化率 top 30%/15%/8%/4%/2%）的互相关（桌面 `log\lag_vs_threshold.py <log.ulg>`）：
  - 所有阈值结果一致 = 真实系统延迟（log_148 roll/pitch 全程 21-42ms）
  - **快速段(top≤8%)降到与另一轴相同 = "中等操纵滞后" = 姿态环 P 不足，角速度环本身无问题**（log_70 pitch 42ms→21ms=roll 完全一致；roll 全程 21ms 因姿态环 P=6.3 高，pitch 姿态环 P=4.7 低）
- **机理**：小幅打杆时姿态环 P 低 → 期望角速度小 → 内环跟踪依赖 I/滤波 → 滞后大；猛打杆误差大 → 姿态环 P 直接给大指令 → 内环 P 主导 → 滞后小
- **判定铁律**：调参目标/手感延迟以阶跃法（跟期望）为准；互相关仅轴间对比。某轴互相关持续偏高时先跑阈值扫描定性，别直接动角速度环参数

### 积分项"反作用"验证法（2026-08-17 log_70 实战）
- 用户怀疑角速度环 I 反作用时，查 `rate_ctrl_status.rollspeed_integ/pitchspeed_integ/yawspeed_integ`（rad/s，限幅≈±0.3）+ 角速度误差(期望-实际)，写脚本算四项：
  - **幅度 vs 限幅**：±0.01 vs ±0.3（3%）= I 几乎没出力，P 主导一切
  - **与误差相关系数**：≈0（-0.06/+0.01）= 无相关，正常；显著负相关且幅度大 = 真反作用
  - **符号一致率 ≈50% 不是反作用证据**——误差已被 P 压到噪声级（±0.1°/s）时符号本来就随机
  - **过零次数**：悬停段频繁过零（8 次/120s 内）仍属小幅度待命，贴限幅累积/快速振荡才是 I 过大
- **I 管稳态误差不管动态滞后**：I 从 0.009→0.05（5 倍）积分项幅度几乎不变 → 动态滞后归因姿态环 P，别为滞后调 I

### 振动污染"假振荡"判定（2026-08-17 log_87 实战：用户报"角速度响应很多振荡"）
- **真控制振荡 vs 振动污染的分水岭 = 角速度误差 RMS**：悬停段 4-8Hz 谱峰 + **RMS<1°/s = 振动污染假振荡**（机身振动大 → 陀螺噪声 → D 项放大成角速度上的"振荡"）；RMS>1°/s + 窄尖峰 = 真控制振荡（P 大/D 不足，降角速度 P）
- **机械振动实锤指标**：水平振动 >100mg 异常 + **加速度某轴明显大**（log_87 Y=1.47 m/s² vs X=0.16，9 倍 = 桨不平衡/电机轴承，优先查桨动平衡）；多日志对比持续恶化（104→150mg）= 机械问题在加重，不是 PID
- **参数缓解顺序（机械修好前）**：① `IMU_DGYRO_CUTOFF` 收紧（20→15，直接挡振动进 D 放大通道，最有效）② 降 RATE_D（D 是振动放大器，0.005→0.003）③ **姿态环 P 别降**（振动在 7.1Hz+，姿态环带宽 2-3Hz 够不着，降 P 只损失跟手性+静差回来）
- **治本**：桨动平衡（贴胶带）> 电机轴承 > 减震泡棉 > 机架螺丝；"振动大时调再多 PID 都白费"

### 短日志/地面测试日志分析（2026-08-11 实机）
- **12s 地面日志也能分析**：ulu_report.py 能生成报告，但无飞行段（未解锁/未起飞）时振荡/跟踪误差等指标会空——**别只看报告结论，直接写 pyulog 脚本深挖**：nav_state 时间线（模式切换是否被接受）、estimator_status_flags（视觉融合状态）、event id+arguments（模式请求时刻）、vehicle_control_mode flags（哪些环在线）
- **实飞段 <5s 的日志：延迟/误差数字只能当方向性参考**（互相关样本太少）。判读原则：延迟数字 + 参数证据（D 是否远超默认）+ 稳态静差三者互相印证才下结论（log_52 实飞仅 2.4s：D=默认 3 倍 + 静差 1.23° + 延迟 195/293ms 三者一致 → 过度阻尼判断可靠）
- **先问一句"有没有解锁/起飞"再决定深挖方向**：地面测试日志的"模式"主题（nav_state 不变=切换没被接受）与飞行日志的"稳定性"主题（振荡/跟踪）完全是两类分析
- 事件 id 大数翻译：`arguments[0]` 常含模式请求的 custom_mode 值（如切 POSCTL 请求），id 本身需查固件 event_messages.h（见 Pitfalls event 节）

## 支持文件
- `scripts/analyze_ulog.py` — 一键分析脚本：自动检测飞行段/模式、输出跟踪误差统计+振荡判定+饱和/振动检查+5 张图（控制台输出）
- `scripts/altitude_source_probe.py` — 高度源诊断脚本：baro 原始高度（压力换算）vs EKF z/vz 时间线、estimator_aid_src_* 各源 fused 统计、cs_* 标志、模式/油门/姿态/悬停推力——"高度一直 X""自稳突然爬升""高度跳变"类问题直接跑
- `references/height-source-diagnosis.md` — 高度锁定+自稳爬升完整案例（log_41，2026-08-15）：参数证据表、时间线、可复用判读规则、修复命令
- `references/px4-pid-architecture.md` — PX4 串级 PID 架构（4 环结构、各参数作用、模式差异、调参顺序、症状→对策表）
- `references/ulg-report-tool.md` — 桌面 ulg_report.py 交互式 HTML 报告工具：用法、8 节结构、Plotly 单文件内嵌技术、4 级点评阈值表、已修 bug 清单、判读案例
- `references/b2-tuning-guide.md` — 用户 B2 调参文档方案解析：滤波截止频率作用（DGyro 为什么更小/FFT 选尖刺/与 D 增益配合）、延迟 30/50/70ms、到位→加P，及与 ulg_report 指标映射
- `references/overdamping-delay-diagnosis.md` — 过度阻尼延迟诊断完整案例（log_52，2026-08-15）：D=默认 3 倍 → 延迟 195/293ms，参数证据表、三层机制、可复用判读规则、修复步骤
- `references/posctl-drift-mechanical-case.md` — 定点持续偏移完整案例（log 01_57_39，2026-08-18）：**机械故障（桨不平衡/振动）为真凶**、视觉 yaw 漂只是症状的证据链、两轮误判（视觉yaw漂/撞障碍）被用户反证推翻的教训、可复用判读规则
- `references/visual-yaw-drift-diagnosis.md` — ⚠️ **误判教训存档（旧结论"视觉 yaw 漂移污染 EKF"已被推翻）**：为何错、正确判定看 posctl-drift-mechanical-case.md
- `scripts/posctl_drift_probe.py` — 定点偏移专项探查脚本：模式确认、三航向源对拍、**电机分配对角饱和**、**振动分段 RMS**、imbalanced_prop、EKF reset、摇杆——"定点模式一直朝一个方向偏移"类问题直接跑（机械 vs 视觉判别）
- `scripts/ulg_header_probe.py` — 日志身份识别脚本：正则扫原始字节提取 ver_hw/ver_sw/sys_mcu/sys_name/sys_uuid（QGC 下载的 version=1 ulog 严格解析会失败，此法稳）——"哪个固件/哪台机器"类问题直接跑

## ulg_report.py 打包 exe（用户要求时执行，2026-08-17 流程验证）
- 用户改完 py 后要求"打包覆盖 exe"：`PyInstaller` 已装（6.21）。命令（在临时目录构建避免污染桌面）：
  `python -m PyInstaller --onefile --noconsole --name ulg_report --distpath "$env:TEMP\ulg_build" C:\Users\<USER>\Desktop\ulg_report.py`
  脚本第 37-41 行已处理 `--noconsole` 下 stdout=None 的崩溃（print 重定向到 devnull），必须带 `-w/--noconsole`（GUI 弹窗选文件模式）
- 流程：**先备份旧 exe**（`ulg_report.exe.bak_YYYYMMDD`）→ 打包 → Copy 到桌面覆盖 → **运行验证**（`& ulg_report.exe <log.ulg>` 用命令行参数走非 GUI 路径，等 3s 后检查报告文件时间戳/内容——新功能标志如"打杆阶跃响应"或"整体相位滞后"是否在 HTML 里）→ 清理 `$env:TEMP\ulg_build`
- **验证陷阱**：某日志无阶跃块 ≠ 打包失败——先跑 py 对比 `step_resp`（None=日志本身无干净阶跃，正常）；exe 与 py 行为一致即打包成功
