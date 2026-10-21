# 松树林世界 v4/v5：树木计数、人工增补、起降坪压平、原地更新资产

配套：点云→UE4 的**管线全貌与 12 个基础坑**在技能 `pointcloud-to-ue4-airsim-world`；
本文是同一条线的**迭代期经验**（用户提"加密度 / 加粗壮大树 / 改起降坪尺寸"这类需求时会用到）。

## 1) ⭐ 树密度的**交叉验证**（比单一算法可靠得多）

用户要"增加树的密度"时，**不要直接调灵敏参数就交** —— 先做独立第二种估计。

**方法 A（冠层局部极大）**：`CHM = 平滑(grid_max - ground)` → `maximum_filter` 找局部极大 →
按高度降序非极大抑制 → 冠层像素按最近峰 Voronoi 分配 → `crown_r = sqrt(area/π)`。
参数扫描实测（4.6 ha 成熟松林）：

| win | h_min | 棵数 | 冠幅中位 |
|---|---|---|---|
| 5 | 3.0 | 174 | 3.66 m |
| 4 | 2.0 | 211 | 3.52 m |
| **3** | **2.0** | **230** | **3.43 m** |

→ **峰值法有上限（本例 ~230）**，再敏感就开始把大冠的次峰算成独立树 ✗

**方法 B（树干聚类，独立且更硬）**：取 `AGL ∈ [0.5, 3.0]` 的点（树干在该高度**彼此分离**）→
投到 **0.25 m** 网格成二值图 → `binary_closing(3×3, iterations=1)` → `ndimage.label` →
按 bbox 过滤（`max(h,w) > 1.6 m` 判灌木丛/连片，`< 2 格` 判噪点）：

```
本例: 39,005 体素 → 460 连通域 → 剔除 过小158 / 过大115 → 保留 187
      树干等效直径 中位 0.63 m (10%~90%: 0.49~0.85)   ← 松树胸径，可作合理性佐证
```
⚠️ `closing iterations=2` 会把邻近树干并掉（187 → 96）✗ —— 用 **iterations=1** ✓

**结论表述法**：两个数字都给出来 ——
"两种独立方法给出 **187~230**，真实棵数在此区间；**230 已达实测可信上限，
再密只能靠插值合成 = 编造，所以停在这里**" ✓
（用户明确反对合成数据冒充真实结果；**主动停在实测边界比凑数更受认可**）

## 2) 人工增补的树：**选位规则容易写反**（我写反过）

用户要"加 10 棵比较粗壮的树"：
- ✗ **错误规则**：`离最近树最远` 优先 → 选出来全在**林子外的空地**（离最近树 50~73 m），
  视觉上孤零零杵在旷野里 ✗
- ✓ **正确规则**：在**林内空隙**找 —— `6.5 m ≤ 离最近树 ≤ 13 m` **且** `20 m 内 ≥ 6 棵`，
  彼此间隔 ≥20 m、离起降坪 ≥15 m ✓
- **"粗壮"用缩放实现**：`XY 缩放 = Z 缩放 × 1.4`（干径 0.36 m → 1.33 m，约 3.7 倍）
- **必须可识别 + 必须写进交付说明**：actor 标签 `bigtree_XX`，文档注明
  "这 10 棵是**人工添加**的，点云里没有这么粗的树" ✓

## 3) 起降坪：把地形局部压平

```python
t5 = terr.copy()
t5[pj-R:pj+R+1, pi-R:pi+R+1] = zmed            # zmed = 该区块高程中值
for k in (1, 2):                                # 边缘 k 格过渡，避免硬台阶
    np.clip(子区域, zmed - k*0.3, zmed + k*0.3, out=子区域)
```

- **⚠️ 必须从【原始平滑地形】重算**，不要在上一版已压平的数组上再压 ✗（否则旧坪残留）
- **坪尺寸与网格对齐**：1 m 网格上做 **2×2 m** 坪 → 占 4 个格，**坪中心落在网格角点**
  （`XMIN+pi, YMIN+pj`）而不是格中心 ✓ 想清楚再算
- **视觉平台**：项目自带 `/Game/Geometry/Meshes/1M_Cube`（原生 100×100×100 cm）缩放
  `(size/nc[0], size/nc[1], 厚/nc[2])`，`location.z = 坪面 - 厚/2` 让**顶面**对齐坪面 ✓
  改完**读回 actor bounds 校验**顶面/尺寸（本例逐位一致 ✓）
- **选坪**：`slope[cells].max() <= 0.05` + 离最近树心 ≥8 m，且**优先靠近旧坪**
  （本次距旧坪仅 0.71 m → 其他东西都不用动）✓
- **AirSim 生成点**换算：`NED_X = UE_Y`、`NED_Y = UE_X`、`NED_Z = -UE_Z`（米），取坪中心 +1.5 m ✓

## 4) ⭐⭐ 改局部：**原地更新资产**，不要新建关卡

本次"起降坪 5×5 → 2×2"**没有新建关卡**，四步搞定：

```
1. 重算坪 → 生成新地形数组
2. OBJ **写到同一路径**（terrain_v4.obj）→ 同一个 import 脚本 + replace_existing=True 重导
   → 关卡里 terrain actor 引用的是同一资产 → **自动更新** ✓
3. 只调受影响的那个 actor（起降坪平台的位置/缩放/标签）→ save_current_level
4. 只改 settings.json 的生成点
```
树的列表、树干碰撞网格、材质**全不用动** ✓
**关卡版本不要一直堆**（v1→v2→v3→v4 已是上限；"只改局部"一律原地改）✓
改完照旧 `-game` 启动验证 + 确认日志里新网格被重新构建（`Built static mesh [...] terrain_v4`）✓

## 5) 交付文档写两份：机器可读 + 面向人

- `ue_assets_v*/generated/scene_metadata.json`（机器）：`ue_transform.units/mapping`、
  `georeference_status: "local coordinates only; source LAS has no CRS/VLR"`、
  树数/高度/冠幅统计、`runtime_design`（视觉树 / 树碰撞 / 地形碰撞各用什么）
- `PineForestV4_场景说明.md`（面向人）：**限制表**（真北/EPSG 未知、原点地理含义未知，
  **尺度可信 / 相对位置可信 / 绝对定位与朝向不可信**）+ **已知偏离清单**
  （人工增补的树、人工压平的坪、树高裁剪上限、非均匀缩放限制、未用 HISM、隐藏碰撞体、宏纹理未接入）
- 每次"只改局部"追加一行**变更记录**（改了什么 / 为何用原地更新而非新关卡）✓

## 6) UE 编辑器位置 + 为什么用户"找不到"

```
UE4Editor.exe / UE4Editor-Cmd.exe   E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\
项目                                E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject
```
用户报"找不到 UE 编辑器"时先查这两点（本例都中招）：
1. **桌面快捷方式没带项目**（`Arguments` 为空）→ 打开后停在**"选项目"界面** ✗
2. **`.uproject` 关联的是 `UnrealVersionSelector.exe`**，它依赖
   `C:\Program Files\Epic Games\Launcher\...`；**Epic Games Launcher 未装 → 双击 `.uproject` 无反应** ✗
   ```powershell
   cmd /c "assoc .uproject"            # Unreal.ProjectFile
   cmd /c "ftype Unreal.ProjectFile"   # -> UnrealVersionSelector.exe
   Test-Path "C:\Program Files (x86)\Epic Games\Launcher\Portal\Binaries\Win64\EpicGamesLauncher.exe"
   ```

**✅ 修法：给用户一个双击文件（不装 Launcher 也能进）** —— `.cmd`、**纯 ASCII、无 BOM**：
```bat
@echo off
chcp 65001 >nul
title UE4 Editor
start "" "E:\Program Files\Epic Games\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe" "E:\AirSim\Unreal\Environments\Blocks\Blocks.uproject" /Game/PineForestV4/PineForestV4 -nosound
exit
```
带 `/Game/<关卡路径>`（且**不带** `-game`）= 编辑器**直达该关卡** → 用户只管点 ▶ Play ✓
`.cmd` 的编码坑（中文路径必乱码、`-replace` 拼路径事故）见
`powershell-automation` 的 `references/cmd-launcher-encoding.md` ✓

## 7) 每版交付前的数值自洽校验

- `关卡 actor 数 = 环境 + 地形 + 隐藏碰撞体 + 起降坪 + 树数`（本例 9+1+1+1+240 = 252 ✓）
- `Σπ·crown_r²` ≈ 冠层像素数（单木分割自洽）
- 每版 OBJ 重新解析 `v`/`f` 行：**法线朝上占比 = 100%**、坐标范围与预期逐位对比
- 平台/坪：读回 actor bounds 与期望值逐位对比
- 启动：`Game class is 'AirSimGameMode'` + 41451 监听 + 日志无 Error/Fatal
