---
name: pointcloud-quality-analysis
description: 判断点云/SLAM 建图质量好坏并出报告。读 PCD 文件做密度、飞点、覆盖、垂直重影、高度分层自动检查，生成三张标注图 + 自包含 HTML 报告（含中文文字分析），可打包 exe 交付。触发词：点云建图怎么样/好坏、PCD 分析、点云质量、重影检测、建图评估、sparse/稀疏判断。
---

# 点云建图质量分析 (PCD Quality Analysis)

## 何时用
- 用户给 PCD/bag 点云问"建图怎么样/好不好"、判断稀疏、检查漂移/重影
- 需要量化评估密度、噪点、覆盖、垂直结构
- 要交付"一键出报告"工具（脚本或 exe）

## 用户偏好（必须遵守）
- **单一完整输出文件**：HTML 报告内嵌 base64 图，不要散落中间 png
- 报告必须含**中文文字分析**（自然语言逐项点评+改进建议），不能只有表格
- 最终交付倾向 exe（用户明确要过）；exe 放桌面，源码也留桌面
- 技术解释要大白话+类比，逐层讲清"为什么"

## 判定框架（三层）
1. **全局一致性**：回环重合、高程图颜色撕裂/马赛克、双窄峰重影
2. **局部细节**：最近邻密度、边缘锐度、飞点
3. **量化指标**：NN 距离分位数、覆盖占用率、峰宽、时间戳连续性（丢帧）

## 分析流水线（numpy+scipy+matplotlib，无需 open3d）
1. 读 PCD binary：解析头部 FIELDS/SIZE/TYPE → `np.dtype([(f,'<f4')...])`，`np.fromfile(path, dtype=dt, offset=头部偏移)`；ascii 用 `np.genfromtxt(skip_header=行数)`
2. 统计：NaN/Inf、xyz 范围、到中心距离 p99/p99.9
3. 密度：采样 ≤30 万点 `cKDTree.query(k=2)`，最近邻分位数；按距原点 0-5/5-10/10-20/20-40/40+m 分桶（对照物理衰减：MID360 角分辨率 0.2°→点间距≈距离×0.0035）
4. 飞点：最近邻>1m 占比（<0.5% 干净）
5. Z 直方图峰：`scipy.signal.find_peaks(height=2%max, distance=0.5/binwidth)`，算每峰半宽
6. 垂直重影：主峰 ±0.5m 细扫（1cm bin），双**窄**峰(<20cm 半宽)间隔 5-40cm 才算
7. 覆盖：主体范围(5-95 分位) 1m 网格占用率（`np.add.at` 计数）
8. 三张图：XY+XZ 密度(log)、高程图(0.5m cell 均值 z, clip 1-99%)、Z 直方图标注峰

## 关键判定阈值
- 近处(0-5m) NN 中位：<2cm 优秀 / <5cm 良好 / <10cm 偏稀
- 飞点(NN>1m)：<0.5% 干净
- 覆盖占用率：>50% 完整 / <30% 严重不足
- 地面峰半宽：<25cm 平整；**室外街道 60cm 属正常**，别误报

## 三张图解读口诀（教用户时用）
- 密度图 = 扫没扫全（覆盖/空洞/拉丝）
- 高程图 = 对不对得上（全局一致/漂移/重影）
- Z 直方图 = 高度层正不正常（缺层/垂直重影）

## 坑（全部实踩过）
- **MID360 垂直 FOV -7°~+52°**，无人机装法正下方是盲区 → 地面点 0% 是物理正常，不是建图坏；z 到 80m+ 可能是真实高楼——**先问用户环境再判异常**
- 街道场景 z-spread 大是"地面+车+树冠"混叠，**不是重影**；重影只认"同层双窄峰"
- numpy 2.x 没有 `np.core` → 用 `np.rec.fromarrays`
- Python `%` 格式化：`%d` 收 float 会崩；**大 HTML 模板占位符数量必须逐一对位**（本会话崩了 3 次，症状都是 `TypeError: must be real number, not str`）
- `--windowed` 打包后 print 报 "Exception ignored" 是无害的（无控制台），报告照常生成
- 清华 pip 镜像 403 时换阿里云：`pip install pyinstaller -i https://mirrors.aliyun.com/pypi/simple/`

## 工具与部署
- `scripts/pcd_report.py` 一键脚本：tkinter 选文件 → 分析 → 生成 HTML 报告并浏览器打开；支持命令行传参/拖文件
- 已部署：`C:\Users\<USER>\Desktop\pcd_report.py` + `PCD点云分析.exe`（PyInstaller `--onefile --windowed`，65.8MB）
- 打包命令：`python -m PyInstaller --onefile --windowed --name PCD点云分析 --clean --noconfirm pcd_report.py`
- 打包后必须验证两个场景：①带参数跑（报告重新生成）②无参数双击（进程存活=tkinter 弹窗正常，用 Start-Process + Get-Process 检测，4 秒后进程在即 OK）

## 验证
- 报告 ~700KB 自包含，含 3 张 base64 图 + 10 个章节
- 已知良好基准值见 `references/street-scene-baseline.md`
