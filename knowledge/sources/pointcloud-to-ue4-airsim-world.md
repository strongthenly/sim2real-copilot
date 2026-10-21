---
name: pointcloud-to-ue4-airsim-world
description: 把真实 LiDAR 点云（.las/.laz 等）做成 AirSim/UE4 可飞、可撞的仿真世界——完整管线：LAS 解析→地面滤波→AGL 分层→体素→贪心合并碰撞盒→OBJ 导出→UE4 Python 自动导入/建关卡→启动验证。当用户说"想在 XX 点云地图里跑探索/仿真"、"把建图结果（PCD/LAS）搬进 UE4/AirSim"、要给无人机做真实场景仿真时使用。含 8 个实测坑（OBJ 坐标约定、commandlet vs 编辑器、GameMode、-nosound 等），每个都带验证过的修法。
---

# 点云 → UE4/AirSim 可飞世界

**核心难点**：AirSim/UE4 的物理世界是**碰撞网格**，点云本身不能碰撞。必须"点云 → 几何"。

## 何时用

- 用户有一份真实 LiDAR 扫描（`.las`/`.laz`/`.pcd`），想在**这个场景里**跑无人机仿真/探索算法
- 要把 SLAM 建图结果（PCD/LAS）变成 AirSim 的仿真场景
- 需要**真实的碰撞几何**（撞树/撞墙有反应），而不是只有视觉

## 关键设计决策（先想清楚再动手）

| 决策 | 做法 |
|---|---|
| 碰撞体用什么 | **飞行高度带内的体素合并盒**（无人机撞不到树冠，取 AGL 0.3~H 即可，H 由用户飞行高度定） |
| 地形用什么 | 地面滤波后的**高度场网格**（单张 OBJ 网格 + trimesh 碰撞），不要碰 UE4 Landscape（Python 很难自动建） |
| 碰撞怎么开 | 网格 `body_setup.collision_trace_flag = CTF_USE_COMPLEX_AS_SIMPLE`（三角面直接当碰撞体，无需生成凸包） |
| 关卡怎么建 | **从目标项目里一个"AirSim 已跑通"的关卡派生**（打开→只删 StaticMeshActor→放自己的网格→另存） |

## 步骤

### 1. 摸清点云（先看数据，别急着转几何）

LAS 头**手解二进制**即可，不依赖 laspy：

```python
import struct
f=open(p,'rb'); f.seek(24); vmaj,vmin=f.read(2)
f.seek(94); hsize=struct.unpack('<H',f.read(2))[0]
f.seek(96); off=struct.unpack('<I',f.read(4))[0]
f.seek(104); fmt=struct.unpack('B',f.read(1))[0]
f.seek(105); rlen=struct.unpack('<H',f.read(2))[0]
f.seek(107); npts=struct.unpack('<I',f.read(4))[0]
f.seek(131); sx,sy,sz=struct.unpack('<3d',f.read(24))
f.seek(155); ox,oy,oz=struct.unpack('<3d',f.read(24))
f.seek(179); maxx,minx,maxy,miny,maxz,minz=struct.unpack('<6d',f.read(48))
```
点格式 3（34 B/点）：X,Y,Z(int32) | intensity(u16) | flags(u8) | cls(u8) | scanangle(i8) | user(u8) | srcid(i16) | gpstime(f8) | R,G,B(u16) ✓ **有 RGB 时可用**
格式 0/1 是 20/28 B 且无颜色。

**分块读**（`np.fromfile` + 结构化 dtype + `np.memmap`），几千万点 30 秒内搞定。必看：
- **Z 直方图** → 判地形范围 vs 树冠层
- **XY 每 1 m 格的 count/min/max** → 覆盖率（手持扫描常只有 20~30% 覆盖，是"采集带"不是矩形！）
- **0.5 m 体素去重计数** → 决定几何规模（几千万点常只剩几十万体素）

### 2. 地面滤波（决定一切）

每格 `min_z` 当地面有 **穿透噪点** 和 **树冠遮挡** 两个坑：

```python
g = grid_min
med = ndimage.median_filter(g, 5)
g = np.where(g < med - 1.0, med, g)          # 去低于局部中值 1m 的穿透点
for r in (3,5,5):
    g = ndimage.grey_opening(g, size=r)       # 开：削掉窄高假地形
    g = ndimage.grey_closing(g, size=r)       # 关：填窄低坑
ground = ndimage.gaussian_filter(g, 2.0)
# 无数据格用最近有效值填充
idx = ndimage.distance_transform_edt(~valid, return_indices=True)
gf = np.where(valid, ground, np.nan)[tuple(idx)]
```

然后 `agl = z - ground[cell]`（**必须用局部地面**，不能用全局分位！用全局分位会把"飞行高度带"算成"最低洼地带"，数量差一个量级）。

### 3. 体素 → 碰撞盒（贪心合并，注意别写错）

- 取 `AGL ∈ [0.3, H]`（H = 用户飞行高度 + 余量）
- **邻域支撑过滤**去孤点：26 邻域内至少 N 个邻居
- 合并：**一维 XYZ 游程**（先 X 游程，再 Y，再 Z）→ 通常压 3~5 倍
  ⚠️ 合并的 bookkeeping 极易写错导致**丢盒**（表现为量出来的包围盒跨度比预期小）→ 写完**一定要量 OBJ 的坐标范围跟预期对比**
- 宁可少合并也别丢盒；几十万三角面对 UE4 是小事

### 4. ⭐ OBJ 导出：UE4 的坐标约定（**全部实测确认**）

UE4 的 OBJ 导入行为（UE 4.27 实测）：

| 项 | 实测行为 | 对策 |
|---|---|---|
| 单位 | **数值 1:1 当 cm，不缩放** | OBJ 里坐标 **×100**（米→厘米） |
| Y 轴 | **镜像**（右手系→左手系） | OBJ 里写 **Y = -y** 抵消；地形网格同时**翻转面绕序** |
| X / Z | 1:1 直通 | 不动 |

**校准方法（别靠猜）**：导一个已知几何的探针 OBJ（10 m 立方体 + 三方向标记），在引擎里用 `actor.get_actor_bounds(False)` 量出来，与 OBJ 里的期望值对比 ✓

导出后**务必**用 Python 再解析一遍 OBJ 的 `v` 行，确认范围符合预期（这一步抓到过我两个 bug）。

### 5. UE4 自动化：**两种模式必须分清**（这里是最大的坑）

| 模式 | 命令 | 能做什么 | 不能做什么 |
|---|---|---|---|
| **commandlet** | `UE4Editor-Cmd.exe proj.uproject -run=pythonscript -script=x.py` | **导入资产**（`import_asset_tasks`）✓ | **spawn actor / 建关卡 → 必崩**（临时世界 `/Temp/Untitled_0`，`EXCEPTION_ACCESS_VIOLATION`） |
| **编辑器** | `UE4Editor.exe proj.uproject -ExecutePythonScript=x.py` | spawn actor、建/存关卡、量 bounds ✓ | 会弹窗口；`quit_editor()` 会 **断言崩溃** |

**分工**：commandlet 只做导入；编辑器做关卡。编辑器脚本**结尾不要调 `quit_editor()`** —— 存完盘后由外部 `Stop-Process` 杀掉；用日志标记轮询判断脚本是否跑完：

```powershell
$pr = Start-Process -FilePath $ue -ArgumentList @($proj,"-ExecutePythonScript=$s","-nosplash","-nosound","-log","-abslog=$log") -NoNewWindow -PassThru
for ($i=0;$i -lt 90;$i++){ Start-Sleep 3; if ((Test-Path $log) -and (Select-String $log -Pattern "LVL DONE" -Quiet)){break}; if($pr.HasExited){break} }
Start-Sleep 5; Get-Process | Where-Object {$_.ProcessName -match 'UE4Editor'} | Stop-Process -Force
```

前置：`.uproject` 的 `Plugins` 段要有 `PythonScriptPlugin` + `EditorScriptingUtilities`（`Enabled: true`）。

导入 OBJ 的正确选项：
```python
opt = unreal.FbxImportUI()
opt.set_editor_property("is_obj_import", True)
opt.set_editor_property("import_mesh", True)
opt.set_editor_property("import_materials", False)
opt.set_editor_property("import_rigid_mesh", True)
opt.set_editor_property("import_as_skeletal", False)
task.set_editor_property("options", opt)
task.set_editor_property("automated", True)
task.set_editor_property("replace_existing", True)
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
```

### 6. ⭐⭐ 建关卡：**必须从"AirSim 已跑通的关卡"派生**

**症状**：关卡能加载、网格能构建，但 **AirSim RPC 41451 永不监听**。
**根因**：AirSim 的 SimMode/RPC 由 GameMode 拉起，**关卡 World Settings 里的 `GameMode Override` 必须是 `AirSimGameMode`**。`new_level` 建的关卡没有它。
**判定**：日志里 `LogLoad: Game class is 'AirSimGameMode'` ✓ vs `'BlocksGameMode'` ✗

**正确做法**：
```python
unreal.EditorLevelLibrary.load_level(SOURCE)              # 项目里已知能跑 AirSim 的关卡
for a in list(unreal.EditorLevelLibrary.get_all_level_actors()):
    if a.get_class().get_name() == "StaticMeshActor":      # ★ 只删静态网格！
        unreal.EditorLevelLibrary.destroy_actor(a)          # 保留天空/光照/雾/后处理/PlayerStart
# ... 放自己的网格 ...
unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/X/NewLevel")   # 另存为新关卡
```
**⚠️ 千万别 `destroy_actor` 全部 actor** —— 会删掉光照/天空/以及 AirSim 相关的 actor。

**⚠️ 坑中之坑**：`EditorLevelLibrary.new_level(p)` 对**已存在**的关卡返回 **False 且不新建**，此时你 spawn 的东西会落进**当时打开的那个关卡**（通常是 `DefaultEngine.ini` 的 `EditorStartupMap`）→ 污染无关关卡 ✗。**必须先判存在**：
```python
if unreal.EditorAssetLibrary.does_asset_exist(LEVEL):
    unreal.EditorLevelLibrary.load_level(LEVEL)
    for a in list(...get_all_level_actors()): destroy_actor(a)
else:
    unreal.EditorLevelLibrary.new_level(LEVEL)
# 并且每次打完印 get_editor_world().get_path_name() 确认操作的是对的关卡！
```

### 7. 启动：`-nosound` 是必需的

命令行起 UE4 时**音频线程会死循环**：
```
LogAudio: Display: Waited 800 ms for audio thread.   ← 每帧刷屏，帧号 38→323 用了 11 分钟 ≈ 0.4 fps
```
→ 世界几乎不 tick → AirSim 初始化完不成 → RPC 不监听。

**命令**：
```powershell
$ue = "…\UE4Editor.exe"; $proj = "…\Blocks.uproject"
Start-Process -FilePath $ue -ArgumentList @($proj, "/Game/PineForest/PineForest", "-game", "-windowed", "-ResX=1280", "-ResY=720", "-nosound", "-log", "-abslog=$log") -PassThru
```
`-game` 模式**直接进关卡，不用手工点 ▶ Play** ✓
**判据**：20 秒内 `Test-NetConnection 127.0.0.1 -Port 41451` = True，日志有 `Game class is 'AirSimGameMode'` + `Waiting for mavlink vehicle...`

### 8. AirSim 生成点（settings.json）

`%USERPROFILE%\Documents\AirSim\settings.json` 的车辆段里 `X/Y/Z` 缺省 = UE4 原点。
AirSim 用 **NED**（X=北 Y=东 Z=下，单位 m），与 UE4 的换算：
```
NED_X = UE_Y ,  NED_Y = UE_X ,  NED_Z = -UE_Z
```
放起飞点前先查该点地形高程 + 周围净空（水平 3 m 内无碰撞盒），离地 1.5 m 起。
**改前必须备份** `settings.json.bak_before_xxx`。

## 坑清单（全部实测踩过）

| # | 坑 | 现象 | 修法 |
|---|---|---|---|
| 1 | OBJ 单位被当 cm | 世界小了 100 倍 | OBJ 坐标 ×100 |
| 2 | OBJ Y 轴被镜像 | 与点云坐标系不一致 | OBJ 写 Y=-y + 翻面绕序 |
| 3 | commandlet 里 spawn actor | `EXCEPTION_ACCESS_VIOLATION` | 导入用 commandlet，建关卡用 `-ExecutePythonScript` |
| 4 | `quit_editor()` | `Assertion failed: IsValid() SharedPointer.h:879` | 不调它，外部杀进程（存盘已完成，不影响） |
| 5 | `new_level` 对已存在关卡 | 返回 False 不新建 → **污染当时打开的关卡** | 先 `does_asset_exist` 判断；每次打印当前关卡路径 |
| 6 | 缺 GameMode Override | 关卡能加载但 AirSim RPC 不监听 | 从能跑 AirSim 的关卡派生 |
| 7 | 无 `-nosound` | 音频线程死循环 → 0.4 fps → AirSim 起不来 | 命令行加 `-nosound` |
| 8 | 全量 `destroy_actor` 源关卡 | 天空/光照/SimHUD 被删 | **只删 StaticMeshActor** |
| 9 | 体素合并 bookkeeping 写错 | 丢盒（量出的包围盒跨度偏小） | 导出后量 OBJ 范围与预期对比 |
| 10 | 路径含中文 | 日志里显示 `1225???` | 脚本能跑；但**资产/OBJ 放 ASCII 路径**更稳 |
| 11 | 全局分位当"地面" | 飞行高度带体素数算错一个量级 | 用**逐格局部地面**算 AGL |
| 12 | **用 `Test-NetConnection` 探测 4560** | AirSim 的 TCP server 接受任何连接 → 日志出现**假的** `Connected to SITL over TCP` → 之后永远 `not receiving any messages from HIL` ✗ | **绝不要探测 4560**；判据只用 **41451**（AirSim RPC）。探过之后**必须重启 UE4** 才能正常接 PX4 |

## ⭐ 正确的启动顺序（顺序错了就卡死）

```
1. 起 UE4 树林关卡（-game -nosound）
2. 等 20~30 s，只检查 41451（别碰 4560！）
3. 再起 WSL 里的 PX4 SITL —— 由 PX4 主动连 AirSim 的 4560
4. 判据：UE4 日志 "Connected to SITL over TCP" + "Ground control connected over UDP"
     且 /clock 开始步进
```
**若先探过 4560**：AirSim 把探测当成 SITL 并卡在等 HIL → **重启 UE4** 再来。

## 验证清单（每步都要过）

1. LAS 头：点数/范围/格式/RGB 打印出来
2. 地面滤波后：地形高程范围合理（别是 56 m 的"树"）
3. 体素分层：AGL 剖面有物理意义（树冠层在哪一层最密）
4. 导出 OBJ 后：**重新解析 v 行**，范围与预期逐位一致
5. 导入后：`verts(lod0)` 与 OBJ 顶点数一致性检查；`collision_trace_flag` = CTF_USE_COMPLEX_AS_SIMPLE
6. 建关卡后：`get_editor_world().get_path_name()` = 目标关卡；`.umap` **时间戳变新**（尺寸小是正常的，关卡只存 actor 引用）
7. 启动后：41451 监听 + 日志 `Game class is 'AirSimGameMode'` + `Waiting for mavlink vehicle...`
