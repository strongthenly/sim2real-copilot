---
name: point-cloud-quality-assessment
description: 点云建图质量评估（PCD 文件/建图结果验收）：三层判断框架（覆盖/全局一致/垂直结构）、量化指标（密度/飞点/重影/覆盖）、Z 直方图峰分析、rviz 观察技巧；配套自动分析脚本（HTML 报告 + 可打包 EXE）。适用于 FAST-LIVO2/MID360、odin 等 LiDAR 建图结果好坏判断。
---

# 点云建图质量评估（PCD）

## 触发条件
- 用户给 .pcd 点云文件问"这个建图怎么样/好不好/怎么判断"
- 需要量化评估（点数、密度、飞点、重影、覆盖）而非只靠肉眼看 rviz
- 用户报告 rviz 里某尺度（如 0.55m）点云稀疏、有重影、怀疑建图质量

## 三层判断框架（先给结论再给证据）
1. **覆盖**：扫没扫全 → 密度图（俯视 XY + 侧视 XZ）
2. **全局一致性**：对不对得上 → 高程图（每格平均高度着色，看颜色撕裂/马赛克）
3. **垂直结构**：高度层正不正常 → Z 直方图（每峰=一层）

三张图分工口诀：密度图看"扫没扫全"，高程图看"对不对得上"，直方图看"高度层正不正常"。三张全过=图可信；任一不过，按对应方向排查。

## 量化指标与阈值（脚本自动算）
- 数据完整性：NaN/Inf 数；点数 <20 万偏少，>100 万良好
- 飞点/孤立点：采样 20-30 万点 KDTree 最近邻（k=2），NN>1m 占比 <0.5% 干净、>2% 噪点多；NN>2m 约 0.06% 为极佳
- 密度：0-5m 最近邻中位 <2cm 优秀（MID360 物理极限 ~1.3cm），<5cm 良好，>10cm 过稀
- 距离衰减：0-5/5-10/10-20/20-40m 中位应递增（角分辨率物理规律），骤减=扫描过快/反射弱
- 覆盖：主体范围（5-95 分位）1m 网格占用率 >50% 好，<30% 严重不足
- 垂直重影：最低主峰 ±0.5m 内 1cm bin 细扫，双峰间隔 5-40cm 且两峰半宽均 <20cm = 真重影
- 地面层厚度：主峰半宽 <25cm 平整，>60cm 注意（街道场景混入车/灌木属正常）

## 关键坑（实测踩过）
- **室外街道场景重影检测会误报**：1m 网格内同时有地面+车+树冠，z 分布天然宽。重影检测必须限制在主峰 ±0.5m + 峰高阈值 30% + 双窄峰（半宽<20cm）条件，否则街道被误判"重影实锤"
- **z 范围大 ≠ 漂移**：扫到真实高楼时 z 可到 80m+。先问用户环境（室内/室外/高楼）再判"z 异常"，别急着下漂移结论
- **rviz 拉近到 0.55m 看稀疏**：先分清物理稀疏（MID360 角分辨率 0.2°，点间距=距离×0.0035，越远越稀正常）vs 病态稀疏（丢帧/下采样狠/盲区）。无人机正装雷达正下方 -7° 以下是 FOV 盲区，近处地面扫不到=物理正常
- **rosout.log 被 mavros 刷屏**：fouruav 上 mavros 的 timesync RTT WARN 每 10s 一条，grep 日志要 `| grep -v mavros` 或直接查节点专属 log 文件

## 分析工具（已交付用户桌面）
- 脚本：`C:\Users\<USER>\Desktop\pcd_report.py` —— 双击弹窗选 PCD 或命令行传参，输出自包含 HTML 报告（3 图 base64 内嵌 + 文字分析 + 检查清单 + 表格），存到 PCD 同目录 `<名>_report.html`
- EXE：`C:\Users\<USER>\Desktop\PCD点云分析.exe`（PyInstaller onefile windowed，65.8MB，目标机无需 Python）
- 用户偏好：**单一完整输出文件**（HTML 自包含），不要中间文件/需后处理方案
- 验证 EXE 三连：①带参数跑一遍（报告 mtime 更新）②无参数 Start-Process 后 4s Get-Process 查存活（tkinter 弹窗不闪退）③检查报告内嵌图数=3

## 重新打包 EXE（改脚本后）
```powershell
pip install pyinstaller -i https://mirrors.aliyun.com/pypi/simple/
python -m PyInstaller --onefile --windowed --name PCD点云分析 --clean --noconfirm C:\Users\<USER>\Desktop\pcd_report.py
Copy-Item dist\PCD点云分析.exe C:\Users\<USER>\Desktop\ -Force
```
注意：清华镜像 403 时换阿里云；`--windowed` 下 print 报 "Exception ignored" 无害（无控制台 stdout），功能照常

## PyInstaller 打包坑（实测）
- **HTML 模板 % 占位符错位**：占位符数量必须与参数一一对应，`%d` 拿到 str 报 "must be real number, not str"。调试：regex 数模板占位符（`%(?:\d*\.?\d*[sdf])`）vs 参数列表逐个对位
- numpy 2.x：`np.core.records` 已移除 → 用 `np.rec.fromarrays`
- PowerShell 改 Python 文件用 here-string 写 patch 脚本再执行，别用 `python -c` 内联（引号地狱）
- 复杂远程 bash（SSH 诊断）用 plink `-m 本地脚本文件`，绕开引号问题（见 ssh-remote-linux-ops 技能）

## 分析脚本核心逻辑（重建参考）
- 解析 PCD header（FIELDS/POINTS/DATA binary 字节偏移），`np.fromfile(dtype=结构化, offset=off)`；binary 每点 32 字节（8 字段 float32）
- `np.add.at` 做 0.5m/1m 网格统计（高程图均值、覆盖占用率）
- `scipy.signal.find_peaks` 找 Z 直方图峰，半宽用 while 循环找半高处
- 采样 KDTree 最近邻（`scipy.spatial.cKDTree` query k=2 排除自身）
- 完整可运行脚本见 scripts/analyze_pcd.py（纯数值版，无 GUI 依赖）

## 参考
- scripts/analyze_pcd.py —— 核心分析脚本（读 PCD → 密度/飞点/峰/重影/覆盖全指标），无 tkinter/HTML 依赖，任何机器可跑
