# UE4 侧执行：OBJ 约定 / 自动化建关卡 / PBR 材质 / 场景后期修改

配套 `references/pointcloud-map-to-simulation-world.md`（那份讲**方案选择与分析**），本文讲**在 UE4 里真正把它做出来**的实测细节。

---

## ⚠️ 1. OBJ 导入的坐标约定（UE 4.27 实测，靠代码量出来而不是猜）

| 项 | 实测行为 | 对策 |
|---|---|---|
| 单位 | **数值 1:1 当 cm，不缩放** | OBJ 里坐标 **×100**（米→厘米）。忘了就差 100 倍（一个 252 m 的地形变成 2.5 m） |
| Y 轴 | **镜像**（右手系→左手系） | OBJ 里写 **`Y = -y`** 抵消 |
| X / Z | 1:1 直通 | 不动 |
| 面绕序 | **导入器自己已经翻过一次** | **绝对不要手动翻绕序** ⚠️ 见下 |

### ⭐ 绝不能手动翻面绕序（我踩过，症状极像"地图做坏了"）

早期笔记写过"为抵消 Y 镜像要同时翻绕序"——**这是错的**：

```
v1：预镜像 Y + 手动翻绕序  ->  地形 127,008 个面 100% 法线朝下
                              -> 背面剔除 -> 地面完全看不见
                              -> 现象：「一堆悬空的方块（杂乱）」+「按 ▶ Play 直接掉入虚空」
v2：预镜像 Y + 不翻绕序    ->  法线朝上 92,112 / 朝下 0  ✓
```

**判定方法（写代码算，别靠眼睛）**：
```python
V = np.array([...]); F = np.array([...])          # 解析 OBJ 的 v / f 行
n = np.cross(V[F[:,1]] - V[F[:,0]], V[F[:,2]] - V[F[:,0]])
print("朝上 %d / 朝下 %d" % ((n[:,2] > 0).sum(), (n[:,2] < 0).sum()))
```
- **地形网格**：必须 100% 朝上
- **竖直圆柱（树干）**：这个统计**无意义**（侧面法线水平，正负由数值误差决定）→ 直接给双面材质
- **双保险**：所有材质设 `two_sided = True`，法线朝向问题彻底免疫

### 校准探针（第一次接新项目时做一次）
导一个已知几何的探针 OBJ（10 m 立方体 + 三方向标记），在引擎里用 `actor.get_actor_bounds(False)` 量出来与期望对比。
**实测确认的例子**：OBJ 里 X[-108.74..143.26] Y[-84.88..167.12] → UE4 里 Y[-8488..16712]，即 `UE_Y = +y*100`（预镜像已被抵消）✓

### 另一条：**不要给数据网格 padding 成 2 的幂**
v1 把 203×229 的真实高程网格用 `np.clip` 复制边缘凑成 253×253 → **四周多出一大片复制出来的假地形**（也是"杂乱"来源）。
**做 OBJ 静态网格不需要 2 的幂**（那是 UE4 Landscape 的要求）→ 直接用真实尺寸。

---

## 2. 自动化：两种模式必须分清

| 模式 | 命令 | 能做什么 | 不能做什么 |
|---|---|---|---|
| **commandlet** | `UE4Editor-Cmd.exe proj.uproject -run=pythonscript -script=x.py` | **导入资产** ✓ | **spawn actor / 建关卡 → 必崩** `EXCEPTION_ACCESS_VIOLATION`（临时世界 `/Temp/Untitled_0`） |
| **编辑器** | `UE4Editor.exe proj.uproject -ExecutePythonScript=x.py` | spawn actor、建/存关卡、量 bounds ✓ | 会弹窗；**`quit_editor()` 会断言崩溃** `IsValid() SharedPointer.h:879` |

**分工**：commandlet 只做导入；编辑器做关卡。
编辑器脚本**结尾不要调 `quit_editor()`** —— 存完盘由外部杀进程；用日志标记轮询：
```powershell
$pr = Start-Process -FilePath $ue -ArgumentList @($proj,"-ExecutePythonScript=$s","-nosplash","-nosound","-log","-abslog=$log") -NoNewWindow -PassThru
for ($i=0;$i -lt 90;$i++){ Start-Sleep 3; if((Test-Path $log) -and (Select-String $log -Pattern "DONE" -Quiet)){break}; if($pr.HasExited){break} }
Start-Sleep 5; Get-Process | Where-Object {$_.ProcessName -match 'UE4Editor'} | Stop-Process -Force
```
**长任务用 `background=true` + `notify_on_complete=true`**（建关卡实测 15 s ~ 数分钟）。

前置：`.uproject` 的 `Plugins` 段要有 `PythonScriptPlugin` + `EditorScriptingUtilities`（`Enabled: true`）。
**UE4 的 `print()` 会吞掉多参数** → 用 `unreal.log("[TAG] " + str(x))` 或单字符串 `print`，否则日志里只剩 `[TAG]` 没内容 ✗

### 建关卡：必须从"AirSim 已跑通的关卡"派生
**症状**：关卡能加载、网格能构建，但 **AirSim RPC 41451 永不监听**。
**根因**：AirSim 的 SimMode/RPC 由 GameMode 拉起，**关卡 World Settings 的 `GameMode Override` 必须是 `AirSimGameMode`**；`new_level` 建的空关卡没有它。
**判定**：日志 `LogLoad: Game class is 'AirSimGameMode'` ✓ vs `'BlocksGameMode'` ✗

```python
unreal.EditorLevelLibrary.load_level(SOURCE)          # 项目里已知能跑 AirSim 的关卡
for a in list(unreal.EditorLevelLibrary.get_all_level_actors()):
    if a.get_class().get_name() == "StaticMeshActor":  # ★ 只删静态网格！
        unreal.EditorLevelLibrary.destroy_actor(a)      # 保留天空/光照/雾/后处理/PlayerStart
# ... 放自己的网格 ...
unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/X/NewLevel")
```
⚠️ **别 `destroy_actor` 全部 actor** —— 会删掉光照/天空/以及 AirSim 相关 actor。
⚠️ **`new_level` 对已存在关卡返回 False 且不新建** → spawn 的东西会落进**当时打开的那个关卡**（`DefaultEngine.ini` 的 `EditorStartupMap`）→ **污染无关关卡**。先 `does_asset_exist` 判断，并且**每次都打印 `get_editor_world().get_path_name()` 确认操作对象**。（我污染过用户的 FUEL 关卡，事后要写脚本删回并留底。）

### 启动：`-nosound` 是必需的
命令行起 UE4 时**音频线程死循环**：`LogAudio: Display: Waited 800 ms for audio thread.` 每帧刷屏 → 帧号 38→323 用了 11 分钟 ≈ 0.4 fps → 世界几乎不 tick → **AirSim 初始化完不成、RPC 不监听**。
```powershell
Start-Process -FilePath $ue -ArgumentList @($proj,"/Game/X/Level","-game","-windowed","-ResX=1280","-ResY=720","-nosound","-log","-abslog=$log") -PassThru
```
`-game` 模式**直接进关卡** ✓（本项目的 `Blocks.uproject` 没有打包 exe，以前只能让用户手工点 ▶ Play；`-game` 绕开了这个手动前置）
**判据**：15~30 s 内 **41451 LISTENING** + 日志 `Game class is 'AirSimGameMode'` + `Waiting for TCP connection on port 4560`。
**首次加载慢**：三棵 15 万顶点的树首次构建各约 20 s（`LoadMap` 68 s）；有 DDC 缓存后降到 5~9 s。

---

## 3. ⚠️ 硬限制：**UE 4.27 Python 不能动态创建组件 → HISM 做不了**

实测（`dir()` 探针）：
```
has HierarchicalInstancedStaticMeshComponent  True      # 类存在
has HierarchicalInstancedStaticMeshActor      False     # 但 Actor 类不存在
Actor 方法里没有 add_component_by_class / add_instance_component
EditorLevelLibrary 只有 spawn_actor_from_class / spawn_actor_from_object
```
→ **无法**用 Python 往关卡里加 HISM/ISM 组件。**替代**：
- **N 个独立 StaticMeshActor**（实测 240 个完全可用；静态网格渲染器按 mesh+材质合批）
- 真要 HISM 就**在编辑器里手工放一个**，Python 之后用 `get_component_by_class` 拿它再加实例

**别在交付文档里写"HISM instances"却实际用独立 actor** —— 分开写清楚（本次原设计文档写了 HISM，实际做不到，已如实标注）。

---

## 4. PBR 材质（Python 全自动）

### 纹理参数：三件事必须设，否则材质一定不对
```python
t.set_editor_property("srgb", ...)                   # 颜色贴图 True，数据贴图 False
t.set_editor_property("compression_settings", ...)   # 法线必须 TC_NORMALMAP
t.set_editor_property("flip_green_channel", True)    # ★ GL 法线 -> DX
unreal.EditorAssetLibrary.save_asset(path, only_if_is_dirty=False)
```
| 贴图 | sRGB | compression |
|---|---|---|
| `*_diff_*` / `*_basecolor_*` | **True** | `TC_DEFAULT` |
| `*_nor_dx_*`（DirectX 法线） | False | `TC_NORMALMAP`（**不翻绿通道**） |
| `*_nor_gl_*`（OpenGL 法线） | False | `TC_NORMALMAP` + **`flip_green_channel=True`** |
| `*_rough_*` / `*_ao_*` | False | `TC_DEFAULT` |
| `*_alpha_*` / `*_mask_*` | False | `TC_GRAYSCALE` |

Poly Haven 命名：`nor_dx` 可直接用；`nor_gl` **必须翻绿通道**。弄错的表现是"光照方向怪、凹凸反了"，很难一眼看出 → **导入后立刻设**。

### 建材质
```python
mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(name, path, unreal.Material,
                                                             unreal.MaterialFactoryNew())
coord = create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate, -900, 0)
coord.set_editor_property("u_tiling", 100.0)     # ★ 1k 贴图铺 200+ m 必须平铺，否则糊成一片
coord.set_editor_property("v_tiling", 100.0)
s = create_material_expression(mat, unreal.MaterialExpressionTextureSample, -600, yy)
s.set_editor_property("texture", tex)
connect_material_expressions(coord, "", s, "Coordinates")      # 引脚名 "Coordinates"
connect_material_property(s, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
# 法线 "RGB"->MP_NORMAL ; 粗糙/AO "R"->MP_ROUGHNESS/MP_AMBIENT_OCCLUSION ; 遮罩 "R"->MP_OPACITY_MASK
```
**树叶/针叶的正确配法**（缺一个都不对）：
```python
mat.set_editor_property("two_sided", True)
mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)                         # alpha 镂空
mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_TWO_SIDED_FOLIAGE)  # 双面叶
```
⚠️ **不要 `MaterialEditingLibrary.recompile_material(mat)`** —— 每个材质触发一次着色器编译，**第 2 个就卡死**（实测等 180 s 无响应）。建完统一 `save_asset`，引擎在关卡加载时自会编译。去掉后 3 个材质 + 建关卡 **15 秒**完成 ✓

### 网格挂材质
```python
mesh.set_material(0, mat)          # 存进网格资产（两个槽的树要 slot0=Bark, slot1=Twig）
comp.set_material(0, mat)          # 也设到 actor 组件上
```
漏设 → 显示默认灰网格材质，看着像"没贴图"。

---

## 5. 碰撞：看得见的是树，撞得到的是树干

真实扫描树有 15 万顶点 → **不能**给整棵树做 trimesh 碰撞（性能炸）。
**做法**：另用"实测树位/树高/冠幅"生成一个**只有树干锥台**的网格，放进关卡但**隐藏渲染**，用它的 trimesh 做碰撞：
```python
comp.set_visibility(False, True)      # 或 comp.set_editor_property("visible", False)
# 碰撞仍在！CTF_USE_COMPLEX_AS_SIMPLE 照常生效
```
好处：
- 飞机在 1~5 m 高度**只能撞树干**，可从枝叶下穿行（符合真实：树冠枝叶不挡多旋翼）
- 碰撞体面数极小（240 棵树 ≈ 8.6 千面）
- **树干半径必须与视觉树等比匹配**：`r_base = 0.0155 * H * xy_bias`（Poly Haven 松树苗 高 1.30 m / 干径 ~4 cm，等比放大后 20 m 树 ≈ 干径 0.62 m，与真实松树比例吻合）
- **必须在交付说明里写明"树干碰撞体是隐藏的"**，否则用户困惑"看不见却撞到"

地形/树干都设 `body_setup.collision_trace_flag = CTF_USE_COMPLEX_AS_SIMPLE`（三角面直接当碰撞体，无需生成凸包）。

---

## 6. 场景后期修改：**就地更新资产，不要新建关卡**

用户提"改起降坪大小 / 换地形 / 挪树"这类**局部修改**时，别再来一轮 `save_map` 建 vN+1 关卡（关卡版本爆炸；用户偏好"只留最新版本"）。

```
1. 用新版数据重写【同一个】OBJ 路径（如 terrain_v4.obj）
2. commandlet 重导该 OBJ（replace_existing=True）-> 同一资产被覆盖
3. 关卡里的 actor 引用自动指向新数据（下次 LoadMap 生效）——【不用重建关卡】✓
4. 只有需要移动/缩放的 actor（如起降坪平台）才开一次编辑器脚本改，然后 save_current_level()
5. settings.json 起点同步更新
```
⚠️ **重算必须在【原始地形】上做**：从"已经压平过"的数组再算，上一次的人工压平就永远抹不掉（换位置/改尺寸都会带着旧痕迹）→ **每次都从原始 `ground_1m.npy` 重新平滑**。

---

## 7. 起降坪（平坦起飞区）

用户说"要一个 X×Y 米平坦区、起点在中心"时：

**选址**：`np.gradient` 算坡度 → 区内最大坡度 ≤ 5~6%；离最近树心 ≥ 8 m；**优先靠近上一版位置**（少动别的东西）。
**压平**：区内赋同一高程（区内中值）；**外圈 2~3 格按 0.25~0.3 m/格做过渡**，否则一圈硬台阶。

**⭐ 偶数尺寸在 1 m 网格上的对齐（容易错）**：
- 格心定义 `cell(j,i)` 中心 = `(XMIN+i+0.5, YMIN+j+0.5)`
- **2×2 m 坪的中心落在网格角点**（不是格心）→ 格用 `i ∈ {pi-1, pi}, j ∈ {pj-1, pj}`，**中心 = `(XMIN+pi, YMIN+pj)`**
- 写完后**必须打印"坪覆盖范围"核对**是否正好等于尺寸（`x[28.76, 30.76] y[12.62, 14.62]` = 2×2 m ✓）

**视觉平台**：项目自带 `/Game/Geometry/Meshes/1M_Cube`（**原生 100×100×100 cm**，用 `get_bounds()` 确认），缩放即可 —— 2×2 m 且厚 10 cm → `scale=(2.0, 2.0, 0.1)`；`z_center = 坪面 − 厚度/2` 让**顶面对齐坪面**（实测 200×200×10 cm、顶面 -874.0 cm 与期望逐位一致 ✓）；单独建灰色材质；actor 标签写清尺寸（`takeoff_pad_2x2m`）。

**AirSim 起点**（`settings.json` → `Vehicles.PX4.X/Y/Z`）：
```
NED_X = UE_Y = y_m ,  NED_Y = UE_X = x_m ,  NED_Z = -(z_ue_m + 1.5)
```
实测坪中心 (29.76, 13.62, -8.74) m → `X=13.62, Y=29.76, Z=7.24` ✓
**改前备份** `settings.json.bak_before_<what>_<ts>`，并在回复里给回退命令。

**交付时主动提示**：多旋翼 `robot_r` 约 0.35 m，起飞有横向漂移 —— **2×2 m 偏小**，主动问用户要不要放到 3×3。

---

## 8. 树的密度：两种独立方法交叉验证，且不许编造

**① 冠层局部极大法**（`scipy` 就够）
```python
chm = gaussian_filter(np.maximum(grid_max - ground, 0), 1.0)     # 冠层高度模型
mxf = maximum_filter(chm, size=win, mode="constant")             # win=3 -> 最密
peaks = (chm > h_min) & (chm >= mxf - 1e-4)
# 按高度降序做非极大抑制（剔除间距 < min_dist 的）
# 冠幅：distance_transform_edt(~peaks, return_indices=True) 把冠层像素分给最近峰
#       -> 每棵树的冠层像素数 -> crown_r = sqrt(area/pi)
```
实测扫描（同一片成熟松林）：

| win | h_min | 棵数 | 冠幅中位 |
|---|---|---|---|
| 5 | 3.0 | 174 | 3.66 m |
| 4 | 2.0 | 211 | 3.52 m |
| **3** | **2.0** | **230** | **3.43 m** |

**② 树干聚类法**（更硬、与①完全独立）—— 树干在 AGL 0.5~3 m 彼此分离，**连通域个数 = 棵数**
```python
m = (agl >= 0.5) & (agl <= 3.0)                     # 用体素，不用原始点
# 投到 0.25 m 网格 -> binary_closing(3x3, 1 次) -> ndimage.label(8-连通)
# 剔除：像素数 < 2（噪点） 或 单块 bbox 跨度 > 1.6 m（灌木丛，不是单棵独干）
```
实测：`39,005 体素 → 460 连通域 → 保留 187`，**树干等效直径中位 0.63 m（0.49~0.85）** ← 松树胸径，合理 ✓

**结论用法**：真实棵数在**两法区间内**（本项目 187 ~ 230）。
- 用户要"增加密度"→ 在可信上限内取上限，并**把两法数字都报给用户**
- 想超过上限只能靠插值合成 = **编造数据** → 明确拒绝并说明（用户明确反对合成数据冒充实测）

---

## 9. 人工添加的元素（如"加 10 棵粗壮树"）

**选位：要【林内空隙】，不是"离树最远"**
```
✗ 错误：按"离最近树最远"降序 -> 选出来的全是林子外的空地（离最近树 50~73 m），很突兀
✓ 正确：约束 6.5 m <= 离最近树 <= 13 m  且  20 m 内 >= 6 棵
          再按"周围树多"降序 + 彼此 >= 20 m 散布
```
实测 10 处：离最近树 6.7~7.3 m、20 m 内 14~24 棵 ✓ 都在林子里 ✓

**尺寸与缩放**：树高 26~32 m、冠幅 6~8 m（"粗壮"主要体现在**树干直径**）。
视觉树按实测**树高/冠幅分别缩放**，并用 `clamp(scale_xy, 0.6*scale_z, 1.6*scale_z)` 限制非均匀缩放幅度，避免放大后树被压扁；粗壮树额外 `xy_bias=1.4` → 实测干径 1.33 m（普通树 0.36 m，约 3.7 倍）。**树干碰撞网格同步更新**（见 §5）。

**诚实标注**（用户明确要求）：actor 用可识别命名 `bigtree_XX`；交付说明里**写明是人工添加的**，并维护"已知偏离"清单。

---

## 10. 交付前数值自洽校验（每次都要算）

- **关卡 actor 数 = 环境 actor + 地形 + 隐藏碰撞体 + 起降坪 + 树数**（例：9+1+1+1+240 = 252 ✓ 恰好对上说明无误操作）
- `Σπ·冠幅半径²` ≈ 冠层像素数
- 地形 OBJ 法线朝上占比 = 100%
- 平台实测尺寸/顶面 Z 与期望**逐位一致**
- 起降坪"坪覆盖范围"打印值 = 用户要求的尺寸
- 启动后 `Game class is 'AirSimGameMode'` + 41451 监听 + 日志无 Error/Fatal
- 交付说明里写清：**未知 EPSG/真北**（源 LAS 无 CRS 时尺度可信、相对位置可信、绝对定位与朝向不可信）

## 11. 路径与编码的坑（会让人查半天）

- **含中文的路径**在 UE 日志里显示 `1225???`：脚本本身能跑，但**资产/OBJ 放 ASCII 路径**更稳
- **`unreal.log()` 优于 `print()`**（`print` 多参数会被日志吞掉）
- **Windows 双击启动器（.cmd）**：内容必须 **100% ASCII**（cmd 按 ANSI/GBK 解析批处理，`chcp 65001` 救不了）→ 中文只放**文件名**，路径用 `%~dp0` 或纯 ASCII 绝对路径；Python 自己调 `chcp.com 65001` 兜底。
  详见 `powershell-automation` 的 `references/cmd-launcher-encoding.md`（含"别用 `-replace` 拼路径"那条把我坑过的实例）。
