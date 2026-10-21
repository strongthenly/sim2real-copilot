# 点云质量评估（建图验收的尺子）

> 领域 `pcd-quality` ｜ 由 4 个原始技能合并而成 ｜ 生成于 2026-10-21

**这一层解决什么**：没有量化判据时"看着还行"会把飞点、重影、分层当成正常，后续所有结论都建在坏数据上。

**建议读法**：A 三层判断框架 → B 阈值与判读表 → C 工具与报告

**包含的原始技能**：`pointcloud-quality-assessment`、`pointcloud-quality-analysis`、`pcd-quality-analysis`、`point-cloud-quality-assessment`


---

### 来源技能 `pointcloud-quality-assessment`

> 评估点云地图/建图质量好坏（LiDAR SLAM 结果判断）：自动化分析脚本（numpy 直接解析 PCD，无需 open3d）+ 密度/范围/地面/重影阈值判读表 + 稀疏/漂移/分层问题诊断树。适用 FAST-LIVO2、odin、任何 bag 导出的 PCD。

#### 评估框架（三层，缺一不可）

1. **全局一致性**：回环是否闭合、首尾重叠是否对齐、z 轴有没有漂移（最致命）
2. **局部细节**：密度、边缘锐度、地面连续性
3. **量化指标**：轨迹误差（evo ATE/RPE，有真值才做）、时间戳丢帧检查

#### 自动化分析（首选路径）

直接跑 `scripts/analyze_pcd.py`（numpy+scipy+matplotlib，无需 open3d）：
```powershell
python analyze_pcd.py <map.pcd> [out_dir]
```
输出：范围统计、密度（NN 距离分位数 + 按距原点距离衰减）、地面平面拟合残差、z 直方图峰值、地面网格 z-spread（重影检测）、4 张图（XY/XZ 密度热图、高程图、z 直方图、地面粗糙图）。全程 <1 分钟（724 万点）。

#### 一键报告工具（推荐 GUI 路径）

`scripts/pcd_report.py` — 成熟版：**双击运行 → tkinter 弹窗选 .pcd → 自动生成 `<同名>_report.html`**（自包含：3 图 base64 内嵌 + 文字分析 + 6 项检查清单 + 全部表格，无中间文件，拷走即用）：
```powershell
python pcd_report.py <map.pcd>     # 或直接双击，弹窗选文件
```
报告结构：总体判定横幅 → 文字分析（自然语言点评每项指标+改进建议）→ 检查清单 → 关键指标 → 范围/密度衰减/分层/峰表 → 3 张标注图 → 解读教学。判据阈值同下表（重影逻辑见陷阱节，必须用修正版）。

#### 判读阈值表（从实战标定）

| 指标 | 正常 | 异常 | 含义 |
|------|------|------|------|
| z 跨度 | <20~30m | >50m | 高度漂移或扫到高楼（侧视图确认） |
| 地面点比例（z<zmin+0.6m）| 10~30% | <1% | 地面缺失/断裂，严重问题 |
| NN 中位（0-5m 处）| 1~2cm | >5cm | MID360 物理正常 ≈1.5cm |
| NN 中位（整体）| 2~4cm | >6cm | 整体偏稀，近距覆盖不足 |
| z 直方图峰数 | 单层=2 峰（地面+天花）| 4+ 峰 | 多层楼 或 垂直重影分层 |
| 地面网格 z-spread | <8cm | >8cm 的 cell 比例高 | 重影/粗糙（**仅室内纯平面有效**；街道场景必然大，见陷阱节） |
| 距离衰减 | 越远越稀 | — | **物理正常**（NN≈距离×0.0035 rad，角分辨率 0.2°），不是缺陷 |

#### 稀疏点云诊断树

1. **近距（如 0.55m 尺度）还稀疏 = 数据源头缺失**，不是正常现象（该距离 NN 应 ≈0.2cm）。先查 3 条命令：
   - `rostopic hz /laser_map` — 地图话题频率稳定？
   - `rostopic hz /livox/lidar` — 原始应稳定 10Hz
   - `rostopic echo -n1 /livox/lidar | grep -c "x:"` — 单帧应 >2 万点
2. 快速通过区域 → 重采补扫（慢速 + 多角度）
3. 下采样参数太大 → 调 FAST-LIVO2 config：`grep -rn "voxel\|downsample\|filter" ~/catkin_ws/src/FAST-LIVO2/config/`，voxel 降到 0.02~0.05m
4. 丢帧（hz 大跳变）→ 数据废了，重采
5. 反射率低 / FOV 盲区（MID360 垂直 -7°~+52°，头顶和脚下物理盲区）→ 接受或改装法，调参无效
6. **禁止插值补洞**：稀疏空洞补不了，插值/上采样是造假，会误导后续测量和定位。宁可保留空洞真相

#### 陷阱

- 局部细节清晰 ≠ 图好：必须看全局（回环、z 漂移），否则被"表面好图"骗
- 丢帧检查不能省：图看着行但丢帧 30%，后续处理全受影响
- open3d 常未安装：numpy 结构化 dtype 解析 PCD binary 足够（8 字段 float32 = 每点 32 字节，DATA binary 从头部偏移起读）
- **地面检测会被单个飞点骗**：`z < zmin+0.6m` 若 zmin 是异常飞点（如 -7m），地面点数=0；应结合 z 直方图主峰判断真实地面高度再选带
- 0.55m 尺度判断时先区分"点间距变大（正常渲染）" vs "真空洞（问题）"
- MID360 距离衰减要按物理预期判读（10m 处 ≈3.5cm 间距），别把物理稀疏当病态
- **重影检测必须防街道误报**（真实踩坑：7.24M 点街道图被 ±2m 窗口 + 10% 峰高阈值误判"不合格"）："地面+路沿+车底盘"天然多峰。正确逻辑：最低主峰 ±0.5m、1cm bin、峰高 ≥30% max、相邻峰间隔 5~40cm **且双峰半宽都 <20cm** 才算重影。z-spread 网格法只适用室内纯平面地板
- **z 范围大 ≠ 漂移**：扫到真实高楼（z 到 82m）是物理事实，先问采集环境再判异常；远点聚团=真实远处建筑（如 10m 网格内聚成 109 团），散点=噪点
- 飞点阈值（NN 距离法）：最近邻 >1m 占比 <0.5% 干净、>2% 噪声明显；最近邻 >2m 占比 <0.1% 很好
- matplotlib 默认字体无中文字形：图内标题一律英文（否则标题变方块+告警刷屏），HTML 正文才用中文
- Python 3 `%` 格式化：`'%d' % float` 报错；大模板占位符与参数元组错位时报 `TypeError: must be real number, not str`——先数清占位符再对参数（本会话 meta 行 4 占位符只给 2 参数导致全线错位）
- numpy 2.x 已移除 `np.core.records` → 用 `np.rec.fromarrays`（读取 ascii PCD 时）
- PowerShell 写长 Python 脚本：`@'...'@` here-string + Set-Content 落盘再跑；`python -c` 多层引号转义地狱 → 改写成临时 patch 脚本文件再执行

#### 关联

- bag→PCD 提取/转换：见 rosbag-pointcloud-processing 等 bag 系列技能
- 环境背景：MID360（角分辨率 0.2°、FOV -7°~+52°、40m 量程）、FAST-LIVO2、fouruav（Orin NX）、odin（留形科技）


---

### 来源技能 `pointcloud-quality-analysis`

> 判断点云/SLAM 建图质量好坏并出报告。读 PCD 文件做密度、飞点、覆盖、垂直重影、高度分层自动检查，生成三张标注图 + 自包含 HTML 报告（含中文文字分析），可打包 exe 交付。触发词：点云建图怎么样/好坏、PCD 分析、点云质量、重影检测、建图评估、sparse/稀疏判断。

#### 何时用

- 用户给 PCD/bag 点云问"建图怎么样/好不好"、判断稀疏、检查漂移/重影
- 需要量化评估密度、噪点、覆盖、垂直结构
- 要交付"一键出报告"工具（脚本或 exe）

#### 用户偏好（必须遵守）

- **单一完整输出文件**：HTML 报告内嵌 base64 图，不要散落中间 png
- 报告必须含**中文文字分析**（自然语言逐项点评+改进建议），不能只有表格
- 最终交付倾向 exe（用户明确要过）；exe 放桌面，源码也留桌面
- 技术解释要大白话+类比，逐层讲清"为什么"

#### 判定框架（三层）

1. **全局一致性**：回环重合、高程图颜色撕裂/马赛克、双窄峰重影
2. **局部细节**：最近邻密度、边缘锐度、飞点
3. **量化指标**：NN 距离分位数、覆盖占用率、峰宽、时间戳连续性（丢帧）

#### 分析流水线（numpy+scipy+matplotlib，无需 open3d）

1. 读 PCD binary：解析头部 FIELDS/SIZE/TYPE → `np.dtype([(f,'<f4')...])`，`np.fromfile(path, dtype=dt, offset=头部偏移)`；ascii 用 `np.genfromtxt(skip_header=行数)`
2. 统计：NaN/Inf、xyz 范围、到中心距离 p99/p99.9
3. 密度：采样 ≤30 万点 `cKDTree.query(k=2)`，最近邻分位数；按距原点 0-5/5-10/10-20/20-40/40+m 分桶（对照物理衰减：MID360 角分辨率 0.2°→点间距≈距离×0.0035）
4. 飞点：最近邻>1m 占比（<0.5% 干净）
5. Z 直方图峰：`scipy.signal.find_peaks(height=2%max, distance=0.5/binwidth)`，算每峰半宽
6. 垂直重影：主峰 ±0.5m 细扫（1cm bin），双**窄**峰(<20cm 半宽)间隔 5-40cm 才算
7. 覆盖：主体范围(5-95 分位) 1m 网格占用率（`np.add.at` 计数）
8. 三张图：XY+XZ 密度(log)、高程图(0.5m cell 均值 z, clip 1-99%)、Z 直方图标注峰

#### 关键判定阈值

- 近处(0-5m) NN 中位：<2cm 优秀 / <5cm 良好 / <10cm 偏稀
- 飞点(NN>1m)：<0.5% 干净
- 覆盖占用率：>50% 完整 / <30% 严重不足
- 地面峰半宽：<25cm 平整；**室外街道 60cm 属正常**，别误报

#### 三张图解读口诀（教用户时用）

- 密度图 = 扫没扫全（覆盖/空洞/拉丝）
- 高程图 = 对不对得上（全局一致/漂移/重影）
- Z 直方图 = 高度层正不正常（缺层/垂直重影）

#### 坑（全部实踩过）

- **MID360 垂直 FOV -7°~+52°**，无人机装法正下方是盲区 → 地面点 0% 是物理正常，不是建图坏；z 到 80m+ 可能是真实高楼——**先问用户环境再判异常**
- 街道场景 z-spread 大是"地面+车+树冠"混叠，**不是重影**；重影只认"同层双窄峰"
- numpy 2.x 没有 `np.core` → 用 `np.rec.fromarrays`
- Python `%` 格式化：`%d` 收 float 会崩；**大 HTML 模板占位符数量必须逐一对位**（本会话崩了 3 次，症状都是 `TypeError: must be real number, not str`）
- `--windowed` 打包后 print 报 "Exception ignored" 是无害的（无控制台），报告照常生成
- 清华 pip 镜像 403 时换阿里云：`pip install pyinstaller -i https://mirrors.aliyun.com/pypi/simple/`

#### 工具与部署

- `scripts/pcd_report.py` 一键脚本：tkinter 选文件 → 分析 → 生成 HTML 报告并浏览器打开；支持命令行传参/拖文件
- 已部署：`C:\Users\<USER>\Desktop\pcd_report.py` + `PCD点云分析.exe`（PyInstaller `--onefile --windowed`，65.8MB）
- 打包命令：`python -m PyInstaller --onefile --windowed --name PCD点云分析 --clean --noconfirm pcd_report.py`
- 打包后必须验证两个场景：①带参数跑（报告重新生成）②无参数双击（进程存活=tkinter 弹窗正常，用 Start-Process + Get-Process 检测，4 秒后进程在即 OK）

#### 验证

- 报告 ~700KB 自包含，含 3 张 base64 图 + 10 个章节
- 已知良好基准值见 `references/street-scene-baseline.md`


---

### 来源技能 `pcd-quality-analysis`

> 点云建图质量分析（.pcd 文件体检）：密度/飞点/重影/覆盖/垂直结构自动判定 + 三张标注图 + 自包含 HTML 报告，可打包 exe。适用 LiDAR SLAM（MID360/FAST-LIVO2/odin）出图验收与任何 PCD 质量评估。

#### 三张图三层判定框架（先教用户建立直觉）

| 图 | 层面 | 回答的问题 | 抓的病 |
|----|------|-----------|--------|
| 密度图（XY 俯视 + XZ 侧视，log） | 覆盖 | 扫没扫全 | 空洞、拉丝、漏扫 |
| 高程图（每格平均 z，terrain 色带） | 全局一致性 | 对不对得上 | 漂移、扭曲、重影、拼图碎裂 |
| Z 直方图（标注峰+半宽） | 垂直结构 | 高度层正不正常 | 缺层、垂直重影、层宽异常 |

判读要点：
- 高程图颜色应"跟着地形走"：街道连续同色、楼是清晰色块、交界锐利；交界糊成渐变带=墙斜/重影；整图马赛克=全局漂移实锤
- Z 直方图一个峰=一个层；**同一物理层出现两个相邻窄峰（间隔 10~30cm）= 垂直重影实锤**
- 街道场景 z 峰天然宽（0.5~2.6m 连片=车/灌木/行人混叠），**别把宽峰当漂移**

#### 关键指标与阈值（自动判定表）

| 指标 | 好 | 警告 | 差 |
|------|-----|------|-----|
| 点数 | ≥100万 | ≥20万 | <20万 |
| NaN/Inf | 0 | - | >0 |
| 飞点（最近邻>1m 比例） | <0.5% | <2% | ≥2% |
| 近处密度（0-5m 最近邻中位） | <2cm | <10cm | ≥10cm |
| 覆盖（主体范围 1m 网格占用率） | >50% | >30% | ≤30% |
| 地面主峰半宽 | <25cm | <60cm | ≥60cm（室外街道正常，室内才警告） |

#### 物理背景（MID360 等固态雷达）

- 点间距 ≈ 距离 × 0.0035（0.2° 角分辨率）→ 1m≈3.5mm，10m≈35mm。**越远越稀是物理必然**：密度必须按距离分桶（0-5/5-10/10-20/20-40/40+ m）对比，远区骤减才异常
- 垂直 FOV -7°~+52°：正上方/正下方是物理盲区（无人机装法决定地面扫不扫得到）——"近尺度还稀疏"先查盲区/快速通过，再查参数；近处点稀疏=源头就缺，调参只还原被扔的点，造不出没扫到的点
- 82m 高楼层 + 地面点 0.0% + 无飞点 = 可能完全正常（真实高楼 + FOV 盲区），先问用户场景再下结论

#### 重影检测正确姿势（2026-08 修正，防误报）

- ❌ 错：主峰 ±2m 找双峰 → 街道场景把"地面+路沿+车底盘"误判重影（曾把合格图判成"不合格"，用户因此要求修逻辑）
- ✅ 对：主峰 ±0.5m 细扫（1cm bin）+ 峰高阈值 30% + **双峰都要窄（半宽<20cm）**才判重影
- 只有"地面层自身分裂成两个近距窄峰"才是真重影

#### 报告工具（用户桌面已有，2026-08 交付）

- 源码：`C:\Users\<USER>\Desktop\pcd_report.py`（numpy/scipy/matplotlib）
- exe：`C:\Users\<USER>\Desktop\PCD点云分析.exe`（PyInstaller 单文件；双击弹窗选文件，或拖 .pcd 到图标）
- 报告：与 PCD 同目录 `<名>_report.html`，**自包含**（3 图 base64 内嵌，一个文件可拷走）
- 结构：文字分析段落（人话点评每项指标+改进建议，FAIL/WARN 汇总）→ 检查清单表（通过/注意/失败）→ 关键指标 → 范围/密度/分层/峰表 → 三张图+解读教学
- **用户偏好：报告必须带自然语言文字分析，不只表格数字；单一完整输出文件，拒绝中间文件**

#### PyInstaller 打包配方（Windows，已踩坑）

```powershell
pip install pyinstaller -i https://mirrors.aliyun.com/pypi/simple/   # 清华镜像 403 时阿里云可
python -m PyInstaller --onefile --windowed --name 中文名 --clean --noconfirm script.py

```
- `--windowed`（GUI 无黑框）下 print 报无害 `Exception ignored`，功能不受影响
- tkinter 弹窗必须 `--windowed`；脚本保留 sys.argv 支持拖文件
- 打包后必须验证两个场景：①带参数跑完整流程（查报告时间戳）②无参数启动查进程是否存活（tkinter 是否拉起、是否闪退）
- onefile 首次双击解压慢 2-3s 属正常

#### 脚本开发踩过的坑（写 PCD 分析脚本时复用）

- **% 格式化模板**：HTML 模板占位符数量和参数元组必须一一对应（曾因 meta 行 4 个占位符只给 2 个参数→全部错位→`%.1f` 拿到 str 崩溃）；`%d` 不接受 float（先 int()）；占位符数量用正则 `%(\d*\.?\d*[sdf%])` 对拍
- numpy 2.x：`np.core.records` 已移除 → 用 `np.rec.fromarrays`
- 大点云全量 KDTree 太慢 → 采样 10-30 万点算最近邻（固定随机种子可复现）
- 1m 网格 z 分布统计：argsort 分组 + np.searchsorted 切片逐组算分位，别对每个 cell 布尔掩码（O(N·U) 太慢）
- 高程图先限幅到 1-99 分位再画，否则高楼把街道细节压没

#### 报告生成流程（照做）

1. 解析 PCD 头（FIELDS/POINTS/DATA 行，binary 用 offset + np.fromfile 按字段读）
2. 分析：NaN → 范围 → 采样 KDTree 最近邻（全距+分桶）→ 飞点 → z 分层 → 峰+半宽 → 重影（上述正确姿势）→ 覆盖
3. 判表（上表阈值）→ 文字分析（每项指标人话 + 改进建议，FAIL/WARN 汇总）
4. 画 3 图 → base64 内嵌 HTML → webbrowser 打开
5. 用户要 exe 时按上面配方打包


---

### 来源技能 `point-cloud-quality-assessment`

> 点云建图质量评估（PCD 文件/建图结果验收）：三层判断框架（覆盖/全局一致/垂直结构）、量化指标（密度/飞点/重影/覆盖）、Z 直方图峰分析、rviz 观察技巧；配套自动分析脚本（HTML 报告 + 可打包 EXE）。适用于 FAST-LIVO2/MID360、odin 等 LiDAR 建图结果好坏判断。

#### 三层判断框架（先给结论再给证据）

1. **覆盖**：扫没扫全 → 密度图（俯视 XY + 侧视 XZ）
2. **全局一致性**：对不对得上 → 高程图（每格平均高度着色，看颜色撕裂/马赛克）
3. **垂直结构**：高度层正不正常 → Z 直方图（每峰=一层）

三张图分工口诀：密度图看"扫没扫全"，高程图看"对不对得上"，直方图看"高度层正不正常"。三张全过=图可信；任一不过，按对应方向排查。

#### 量化指标与阈值（脚本自动算）

- 数据完整性：NaN/Inf 数；点数 <20 万偏少，>100 万良好
- 飞点/孤立点：采样 20-30 万点 KDTree 最近邻（k=2），NN>1m 占比 <0.5% 干净、>2% 噪点多；NN>2m 约 0.06% 为极佳
- 密度：0-5m 最近邻中位 <2cm 优秀（MID360 物理极限 ~1.3cm），<5cm 良好，>10cm 过稀
- 距离衰减：0-5/5-10/10-20/20-40m 中位应递增（角分辨率物理规律），骤减=扫描过快/反射弱
- 覆盖：主体范围（5-95 分位）1m 网格占用率 >50% 好，<30% 严重不足
- 垂直重影：最低主峰 ±0.5m 内 1cm bin 细扫，双峰间隔 5-40cm 且两峰半宽均 <20cm = 真重影
- 地面层厚度：主峰半宽 <25cm 平整，>60cm 注意（街道场景混入车/灌木属正常）

#### 关键坑（实测踩过）

- **室外街道场景重影检测会误报**：1m 网格内同时有地面+车+树冠，z 分布天然宽。重影检测必须限制在主峰 ±0.5m + 峰高阈值 30% + 双窄峰（半宽<20cm）条件，否则街道被误判"重影实锤"
- **z 范围大 ≠ 漂移**：扫到真实高楼时 z 可到 80m+。先问用户环境（室内/室外/高楼）再判"z 异常"，别急着下漂移结论
- **rviz 拉近到 0.55m 看稀疏**：先分清物理稀疏（MID360 角分辨率 0.2°，点间距=距离×0.0035，越远越稀正常）vs 病态稀疏（丢帧/下采样狠/盲区）。无人机正装雷达正下方 -7° 以下是 FOV 盲区，近处地面扫不到=物理正常
- **rosout.log 被 mavros 刷屏**：fouruav 上 mavros 的 timesync RTT WARN 每 10s 一条，grep 日志要 `| grep -v mavros` 或直接查节点专属 log 文件

#### 分析工具（已交付用户桌面）

- 脚本：`C:\Users\<USER>\Desktop\pcd_report.py` —— 双击弹窗选 PCD 或命令行传参，输出自包含 HTML 报告（3 图 base64 内嵌 + 文字分析 + 检查清单 + 表格），存到 PCD 同目录 `<名>_report.html`
- EXE：`C:\Users\<USER>\Desktop\PCD点云分析.exe`（PyInstaller onefile windowed，65.8MB，目标机无需 Python）
- 用户偏好：**单一完整输出文件**（HTML 自包含），不要中间文件/需后处理方案
- 验证 EXE 三连：①带参数跑一遍（报告 mtime 更新）②无参数 Start-Process 后 4s Get-Process 查存活（tkinter 弹窗不闪退）③检查报告内嵌图数=3

#### 重新打包 EXE（改脚本后）

```powershell
pip install pyinstaller -i https://mirrors.aliyun.com/pypi/simple/
python -m PyInstaller --onefile --windowed --name PCD点云分析 --clean --noconfirm C:\Users\<USER>\Desktop\pcd_report.py
Copy-Item dist\PCD点云分析.exe C:\Users\<USER>\Desktop\ -Force
```
注意：清华镜像 403 时换阿里云；`--windowed` 下 print 报 "Exception ignored" 无害（无控制台 stdout），功能照常

#### PyInstaller 打包坑（实测）

- **HTML 模板 % 占位符错位**：占位符数量必须与参数一一对应，`%d` 拿到 str 报 "must be real number, not str"。调试：regex 数模板占位符（`%(?:\d*\.?\d*[sdf])`）vs 参数列表逐个对位
- numpy 2.x：`np.core.records` 已移除 → 用 `np.rec.fromarrays`
- PowerShell 改 Python 文件用 here-string 写 patch 脚本再执行，别用 `python -c` 内联（引号地狱）
- 复杂远程 bash（SSH 诊断）用 plink `-m 本地脚本文件`，绕开引号问题（见 ssh-remote-linux-ops 技能）

#### 分析脚本核心逻辑（重建参考）

- 解析 PCD header（FIELDS/POINTS/DATA binary 字节偏移），`np.fromfile(dtype=结构化, offset=off)`；binary 每点 32 字节（8 字段 float32）
- `np.add.at` 做 0.5m/1m 网格统计（高程图均值、覆盖占用率）
- `scipy.signal.find_peaks` 找 Z 直方图峰，半宽用 while 循环找半高处
- 采样 KDTree 最近邻（`scipy.spatial.cKDTree` query k=2 排除自身）
- 完整可运行脚本见 scripts/analyze_pcd.py（纯数值版，无 GUI 依赖）


---
