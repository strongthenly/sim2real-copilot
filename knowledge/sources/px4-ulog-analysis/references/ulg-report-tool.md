# ulg_report.py — 交互式 HTML 报告工具（桌面）

## 位置与用法
- 路径：`C:\Users\<USER>\Desktop\ulg_report.py`（用户交付模式同 pcd_report.py：脚本放桌面 + 自包含 HTML 报告）
- 依赖：`pyulog` `numpy` `plotly`（本机 Python 3.12 已装；换机器 `pip install pyulog plotly -i https://pypi.tuna.tsinghua.edu.cn/simple`）
- **GUI 模式（双击/exe 无参数）**：弹窗选 .ulg（askopenfilenames 可多选）→ 弹窗选保存文件夹（默认定位上次记忆目录）→ 进度窗 → 完成后自动浏览器打开最后一份报告。记忆文件 `ulg_report_config.json`（exe 同目录，`_base_dir()` 用 sys.executable 目录防 frozen 丢失）
- 命令行：`python ulg_report.py xxx.ulg [-o 输出目录]`；无 -o 用记忆目录，没有则日志同目录
- 输出：`<名>_report.html` 自包含单文件（plotly.js 内嵌），~7.5MB，离线可开
- 进度打印：GUI 模式为 ttk 进度窗（Progress 类同 pcd_report.py），命令行文本进度条（用户对静默等待敏感，必须逐步反馈）

## exe 打包（已交付桌面）
- `C:\Users\<USER>\Desktop\ulg_report.exe`（51.4MB onefile **--noconsole 无终端窗口**，双击弹窗即用）
- 打包命令：`pyinstaller --onefile --noconsole --name ulg_report --collect-all plotly --distpath <桌面> --workpath <tmp> --specpath <tmp> ulg_report.py`
- **坑 1（GBK 编码）**：Windows 中文控制台 stdout 用 GBK，print 里 `✓`(U+2713) 会 UnicodeEncodeError 崩溃——print 禁用特殊符号；脚本开头加 `sys.stdout.reconfigure(errors='replace')` 兜底
- **坑 2（--noconsole 后 stdout=None）**：windowed 模式下 `sys.stdout/stderr` 是 None，任何 print 都崩——脚本开头必须 `if sys.stdout is None: sys.stdout = open(os.devnull,'w')`（Progress._out 也要判 None）
- **坑 3**：--noconsole 后用户看不到输出——GUI 模式必须用 `tkinter.messagebox.showerror/showinfo` 报告失败/完成；onefile 首次启动慢 1-2s 正常；杀软可能误报
- 重打包后必须实测：带参数跑（验证 stdout 兜底 exit=0 + 报告生成）+ 无参数启动 7s 存活（验证 GUI 弹窗）

## 新增指标：延迟测量（互相关）
- `time_lag(t_sp, sp, t_act, act)`：互相关求期望/实际时间滞后。**只取期望信号变化率 top 30% 的活跃段**（打杆激励段延迟才可测），搜索范围 ±0.5s，返回 ms
- **坑：setpoint 时间戳是控制发布时刻、angular_velocity 是 IMU 采样时刻，有固有对齐偏差 → 小延迟轴的符号会翻转（roll 可能 -20ms）→ 报告/点评一律用绝对值**
- 分级（B2 调参标准）：<40ms 优秀 / <60 一般 / ≥60 偏大（加 RATE_D）；姿态环 <100ms 正常
- yaw 延迟大属常态（偏航惯量大、PX4 默认 YAWRATE_D=0），点评给特殊分支，不建议贸然给 yaw 加 D

## 新增指标：B2 方案三件套（结合 B2飞控调参.md，2026-08）
1. **稳态静差/到位率**（对应 B2"能否到达同一值→加P"）：期望姿态变化率<1°/s 的稳定段，算平均偏差（有符号）+ 到位率(|err|<0.5° 占比)。|mean_err|>0.8° 或 reach<70% → C 级提示加 MC_*_P(+1.0)。剔除了打杆瞬态，专看"到没到位"
2. **滤波 vs 尖刺对照**（对应 B2 第一步 FFT 选频率）：gyro FFT 找 20-150Hz 最强尖刺，对照 IMU_GYRO_CUTOFF——尖刺<cutoff → 噪声未被滤除(降 cutoff 或动平衡)；尖刺>cutoff → 已滤除，可考虑放宽滤波档位。实测 log_148：尖刺 59Hz > cutoff 40Hz → 建议放宽 50/60Hz 档
3. **调参参数快照**：第 3 节表格显示 MC_*RATE_P/I/D、MC_*_P、IMU_*_CUTOFF 当前值 vs PX4 常见默认（0.15/0.15/0.003、6.5、40/30），已调整的标黄
- 坑：f-string 里中文引号必须用「」不能裸 "（会断 f-string）；pyulog 参数值是字符串要 float() 转换

## 报告 8 节结构
1. **概览仪表盘**：彩色分级卡片（时长/实飞段/导航模式/控制环/角速度跟踪 RMS 徽章/姿态 RMS/扭矩饱和率/振动 mg/电压 v0→vmin）
2. **飞行概况**：模式时间线（nav_state 中文名）、控制环启用（flag→中文映射）、导航源警告（fake_pos/fake_hgt 替代源）、电池
3. **姿态跟踪**：3 轴 SP vs ACTUAL + 误差填充图 + RMS/均值/峰值表
4. **角速度跟踪**（调参核心）：同上 + 悬停/机动分段误差 + 机动占比 + 全程角速度峰值
5. **频谱与振荡检测**：rate 误差 FFT（红=最平稳 5s 悬停段，区分打杆 vs 真振荡）+ 陀螺振动谱 0-80Hz
6. **执行机构与饱和**：扭矩（限幅 ±1.0 虚线）/电机输出/积分项范围 + 饱和率
7. **机身振动**：30-80Hz 能量占比 + 谱峰频率→电机 RPM + 水平加速度 mg
8. **综合结论与建议**：关键结论表 + 按优先级排行动建议（机动量小→补阶跃打杆测试；fake 源→先验证 GPS 等）

## Plotly 单文件内嵌技术（重建工具关键）
- 第一张图：`fig.to_html(full_html=False, include_plotlyjs='inline', config={'displaylogo': False, 'scrollZoom': True})`；后续图 `include_plotlyjs=False`（JS 只内嵌一次，省 3.6MB×N）
- **交互方式（用户偏好）**：`fig.update_layout(dragmode='pan')` 统一在 fig_to_html 里加——左键按住拖动=平移、滚轮=缩放（scrollZoom=True）；config 里 `modeBarButtonsToRemove=['lasso2d','select2d']` 去掉框选按钮。报告 legend 文字同步写"按住左键拖动=平移, 滚轮=缩放"
- 交互：`hovermode='x unified'`；最底行 `update_xaxes(rangeslider_visible=True)`；飞行段 `fig.add_vrect(x0, x1, fillcolor='rgba(22,163,74,0.05)', line_width=0)` 浅绿底纹
- 中文：浏览器渲染无乱码（只有 matplotlib 才需英文标签）
- 四元数→欧拉角 + wrap180 + `np.interp` 对齐（与主 SKILL 相同）

## 自动点评引擎分级阈值（4 级：A 优秀/B 良好/C 需关注/D 异常）
| 指标 | A | B | C | D |
|---|---|---|---|---|
| 角速度跟踪 RMS (°/s) | <0.5 | <1.5 | <3 | ≥3 |
| 姿态跟踪 RMS (°，手动) | <1.5 | <2 | <5 | ≥5 |
| 悬停段 4-8Hz 带能量占比 | — | <15% | 15-30% | >30% 且 rate RMS>1 |
| 振动 30-80Hz 能量占比 | <10% | <25% | — | ≥25% |
| 水平加速度振动 | <50mg | <100mg | — | ≥100mg |
| 扭矩/电机饱和率 | — | <5% | ≥5% 警告 | — |
| 角速度积分项 vs 限幅 ±0.3 | <0.24 | — | ≥0.24 配平问题 | 顶限幅 |
| 单芯电压（vmin/S 数） | ≥3.7V | 3.5-3.7V 警戒 | — | <3.5V 深度放电 |

## 分析逻辑细节
- **飞行段检测**：actuator_armed armed=1 区间 ±0.5s 缓冲；着陆时刻 = **起飞(armed+1s)后最后一次 0→1 跳变**（见"4 个 bug"第 4 条，三坑正解）
- **悬停段**：飞行段内滑动 5s 窗口选角速度能量最小（必须先用掩码排除飞行段外数据，否则选到着陆后静止段）
- **电池 S 数估算**：`for s in (6,5,4,3): if v/s > 3.2: return s`（默认 6）
- **in_fly 作用域陷阱**：analyze() 内 lambda 需在函数级初始化（默认全 False）再在飞行段检测后覆盖，否则后面执行机构/振动/电池部分拿不到掩码 → 空数组崩溃
- **电机通道**：actuator_motors control[0..11] 只取 std>1e-6 的通道（跳过恒 0 的）
- **nav_state 映射**（PX4 新版）：0 手动/1 定高/2 定点/3 任务/4 悬停/5 返航/6 特技/7 机载/8 起飞/9 降落/10 跟随/11 精准降落/12 环绕/13 VTOL 起飞/14 未知/**15 外部 External**（本机日志实测 nav_state=15）

## 本会话修掉的 4 个 bug（重建勿漏）
1. in_fly 作用域（见上）
2. landed 开机态（见上）
3. 电池评估用**飞行段最低电压 vmin** 而非段末电压——段末会漏掉落地瞬间的电压低谷（实测 23.4V vs 21.97V），单芯 3.9V 判断过乐观；改用 vmin 后正确触发 3.66V 警戒提示
4. **landed 上升沿检测**（log_144 实战）：landed=1 可能从开机持续数秒（地面准备）——取第一个 landed 会把地面段当飞行段→所有指标空数组崩溃（"zero-size array to reduction operation maximum"）；取最后一个样本又会在着陆后含地面停留段。**正解：取起飞(armed+1s)后最后一次 0→1 跳变**（landv_f diff），无跳变（日志中途停止）才兜底最后一个样本

## 指标移除记录（用户明确要求，勿重新引入）
1. **电流相关指标全部移除**（2026-08）：用户电流计数据不可信——`current_a`、平均电流 avg_i、平均功耗 avg_p(电流×电压) 已从 analyze/battery_comment/第7节表格删除。电池分析只用电压维度（v0/vmin/drop_rate/cells/单芯电压）
2. **预估飞行时间移除**（2026-08）：SOC 速率法（t_empty/t_safe）和电压外推（t_v_est）都已删除——用户不要剩余时间估算。概览卡"预计可飞"、第 7 节 SOC 行、battery_comment 剩余时间评语、综合建议 t_safe 提示全部清掉
- 教训：新增指标前先确认数据源可信度（电流计质量未知时不要引入依赖电流的指标）；用户明确否决的指标类别（剩余时间）不要再提

## 判读案例（log_148 实测，P=0.45/0.505 调参飞行）
rate RMS 0.14°/s（优秀）· 4-8Hz 带 1.7%（无振荡，5.2Hz 峰为打杆操作频带）· 扭矩饱和 0% · 积分项 ≤0.057（配平好）· 30-80Hz 16.7% 峰 59Hz≈3516RPM（电机基频）· 单芯 3.66V（警戒）→ 结论：调参成功，建议补阶跃打杆测试（快速打杆至 30-50° 保持 2s 回中）检验极限。
