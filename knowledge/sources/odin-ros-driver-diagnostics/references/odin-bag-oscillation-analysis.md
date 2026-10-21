# odin bag 振荡/数据质量分析（模仿 odin_bag_report 模板 → PDF）

## 触发条件
用户给 odin 采集 bag（含 `/odin1/imu` + `/odin1/odometry`），要求"模仿 odin_bag_report（交控方）.pdf 模板分析"。模板 = 图页(截图/图表) + 报告文本页，报告共 6 节 + 飞控摘要。

## 环境与工具（Windows 本机，无需 ROS）
- Python 包：`rosbags`（读 bag）、numpy、scipy、matplotlib、markdown、pypdf
- 22GB 级 bag 顺序扫描提取 ~4-7 分钟 → 必须 `terminal(background=true, notify_on_complete=true)` 跑，别前台等
- 桌面工作区：`C:\Users\<USER>\Desktop\bag_tools\`（用户工具目录，脚本留这里可复用）
- 分析脚本：`bag_tools\_extract_odin_bag.py`（提取）+ `bag_tools\_analyze_odin_bag_v2.py`（分析+出报告/图，v2 版含摇杆证据与机动判读）+ `bag_tools\_extract_rc.py`（摇杆/RC/vision_pose 证据提取）——已有可用版本，新 bag 只需改 BAG 路径和 BAG_START_NS

## 提取要点（_extract_odin_bag.py）
- `AnyReader([bag])` 新版 API **不接受 typestore 参数**（旧示例代码里的 `get_typestore` 已过时）
- 只订阅需要的话题（`reader.messages(connections=[...])`），点云/图像大消息跳过，避免反序列化 22GB
- `/odin1/imu`：`sensor_msgs/Imu`，~400Hz，112941 条/280s 量级
- `/odin1/odometry`：`nav_msgs/Odometry`，~10Hz；`/odin1/odometry_highfreq` ~400Hz（备用）
- `/mavros/state`：**mode 字段是字符串**，不能进 float64 数组 → 单独存 txt（时间\t模式）
- `/mavros/estimator_status`：**字段名带 `_status_flag` 后缀**（`pos_horiz_rel_status_flag`、`velocity_horiz_status_flag`、`attitude_status_flag` 等），不是 flags 数组！用 `getattr(msg, name, 0)` 但名字必须带后缀，否则**静默全 0 → 假结论"EKF 全无效"**（本会话实际踩过）

## 模板指标定义（逆向自 odin_new_report.md，_analyze_odin_bag.py 已实现）
1. **IMU 数据质量**：消息数/时长=平均采样率；中位间隔；丢帧=dt>2×中位、重复=dt<0.5×中位；acc mean/std；gyro std；偏置警告=|mean|>0.01 rad/s 的轴
2. **振荡检测**：gyro 水平分量 sqrt(gx²+gy²) 的 1s RMS 滑窗（cumsum 快速法），阈值 0.0150 rad/s；合并 <1s 毛刺；输出各段 [t0-t1]、总时长占比、振荡/平稳段 RMS 及幅度比
3. **频谱**：**按轴分析**（gx/gy/gz 各算 welch PSD 取主导轴），不要用合成 sqrt(gx²+gy²+gz²)——全波整流效应会把 1Hz 翻成 2Hz（本会话实测）；主峰用**格点 argmax**（抛物线插值在平缓谱上会出伪影，实测 1.48 vs 0.49Hz）；平稳段"无显著峰"判定 = 峰值 < 10×中位数 **或** < 0.1×振荡段峰值
4. **高频振动**：acc 水平合成 welch，5-20Hz 频带积分能量（np.trapezoid）；>20Hz 找 find_peaks 取 top3（电机频率参考）；振荡/平稳段 5-20Hz 能量比 >10 → 控制振荡驱动电机调速（非机械共振）
5. **Odometry**：频率、位置范围、平稳/振荡段水平位置 std（sqrt(std(x)²+std(y)²)）、振荡段速度谱主峰（0.05-10Hz）、roll/pitch std+max
6. **结论**：规则化（IMU 正常/burst/异常三档 + 振荡特征 + 模板建议措辞）
7. **飞控摘要**（模板没有，对交控方有价值）：mavros/state 模式段 + estimator_status 关键标志的有效时间段统计

## 坑清单（全部实测踩过）
- **odom 振荡标记映射**：必须按时间戳比较（`any(a<=t<=b for a,b in segments)`），**不能**用 IMU 索引截断 `osc_seg[:len(odom)]`——IMU 400Hz 与 odom 10Hz 时间轴错位 → 平稳/振荡段全错 + std=NaN
- numpy 2.x：`np.trapz` 已改名 `np.trapezoid`
- scipy.welch：`nperseg = min(nperseg, len(x))` 自适应，否则平稳段只有 ~1s（402 点）时报 `noverlap >= nperseg` 错误
- 平稳段可能极短（全程振荡时 <2s），其 PSD 主峰（如 32Hz 电机频率）不可当姿态主频——用相对振荡段的幅度判定
- IMU 丢帧/重复计数高 ≠ 传感器故障：odin 驱动**批量发布**，dt 分布呈 burst（~17% 间隔 <0.5×中位，0-1ms 成对到达）。判定逻辑：burst_pct>10% → 报告"burst 非均匀采样"而非"异常"
- gyro mean 高可能来自真实转动（非偏置），措辞写"角速度均值偏移"较稳妥，但模板用"恒定角速度偏置"可保留
- 合成数据冒烟测试：std 对阶跃边缘单点 outlier 敏感（201 点中 1 个 -2.6m 点 → std 0.002→0.18），真实数据无此问题，测试时注意别误判为 bug

## 图表（matplotlib 中文）
- 先 `font_manager.fontManager.addfont(r"C:\Windows\Fonts\msyh.ttc")` + `rcParams["font.sans-serif"]=["Microsoft YaHei","SimHei"]`、`axes.unicode_minus=False`，否则中文乱码
- 6 面板：gyro 水平分量(振荡段红底)、acc 水平、gyro PSD 三曲线、acc PSD(标 5-20Hz)、roll/pitch、x-y 轨迹(平稳绿/振荡红)

## md + 图 → PDF（模板样式：图页 + 文本页）
1. `bag_tools\_md_to_html.py`：图 base64 内嵌 `<img>` + `markdown.markdown(md, extensions=["fenced_code","sane_lists"])` + 中文 CSS（Microsoft YaHei，10.5pt）→ 单 HTML
2. Edge headless 打印：**PowerShell 里 `&` 调用符会被 Hermes 终端误判为后台标记** → 必须用 `Start-Process -FilePath $edge -ArgumentList '--headless','--disable-gpu','--no-pdf-header-footer','--print-to-pdf=<路径>','file:///...' -Wait -NoNewWindow`
3. 验证：pypdf 读页数 + 每页 extract_text 检查结尾"报告: ...md"在最后一页

## ★ 判读铁律：机动 vs 故障振荡（08-19 用户纠正，本类任务最重要）
**1s RMS 滑窗只能测"运动活跃度"，区分不了手动机动和故障振荡**。08-08 bag 曾误判"全程振荡 100%、疑似定位源退化"，用户纠正：**飞机悬停时左右前后摇摆（手动操纵）是正常的，不是振荡**。给交控方的报告误报故障是严重错误。

自动检测出"振荡段"后**必须**用证据链定性，按硬度排序：
1. **摇杆 manual_control/control**：x(前后)/y(左右) 过零次数（280s 内 10-30 次=在持续打杆）、最大幅度（>0.5=大幅打杆）、打杆占比（|x|>0.15 或 |y|>0.15 的时间 >10%=飞行员在操纵）——最硬证据
2. **RC in 原始 PWM**：多通道 std>30 PWM 且同时活动 = 手动操纵
3. **位置轨迹形态**：280s 内往返 15+19 次、范围 2-3m、速度>0.3m/s 占 13%、最大 1.8m/s = 在来回平移（机动），非原地抖
4. **姿态-加速度一致性**：ax vs -g·sin(pitch)、ay vs g·sin(roll)（0.5s 滑动平均后算相关）r>0.4 = 有真实平动加速度（机动）
5. **vision_pose**：30Hz 且 dt std<2ms = 定位流畅无跳变（死值是 timer 级恒定，跳变是 dt 剧变）
6. **频谱主峰 0.2-0.8Hz 低频段** = 位置环悬停修正/操纵频段，非高频振荡
7. **EKF 全程有效**（pos_horiz_rel/velocity_horiz/attitude 100%）= 飞控有有效视觉源（排除"飞控拒视觉数据"）

故障振荡应满足：摇杆静止 + 位置原地抖/大漂移 + 定位跳变 + 姿态高频摆。无法证实故障时报告一律写"**运动活跃段**"而非"振荡段"。

## 判读经验（08-08 bag 实例）
- 该 bag 实测为**正常悬停+平移机动**：全程 POSCTL 手动、摇杆过零 10/20 次、位置往返 15/19 次、姿态-加速度 r=0.45-0.58、vision_pose 30Hz 稳定、EKF 全程有效 → 结论"属正常飞行行为，无需针对振荡处理"
- 若真无摇杆（纯自主飞行 bag）：主频低(0.5Hz 附近) + 位置 std 大 + 速度谱 0.09Hz 慢漂 → 才指向**定位源不稳定**（视觉里程计抖动），而非控制环高频振荡
- 对比模板 bag（1.00Hz、6.7 deg/s、单段 167s、采样干净 0/0）做差异表呈现给交控方
- 模板 bag 若场景同为悬停机动，其"振荡"判定同样需复核（该 bag 无 mavros 摇杆话题，无法验证）
