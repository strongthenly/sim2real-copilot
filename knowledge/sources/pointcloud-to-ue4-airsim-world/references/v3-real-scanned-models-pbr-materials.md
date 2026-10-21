# v3：真实扫描模型（Poly Haven CC0）+ PBR 材质 + 隐藏碰撞体

v2 是程序化圆锥树，v3 换成**真实扫描的松树 FBX（含 PBR 贴图）**，并用**隐藏的实测树干网格**做碰撞。

## 1) 纹理参数：三件事必须设，否则材质一定不对

```python
t.set_editor_property("srgb", ...)                      # 颜色贴图 True，数据贴图 False
t.set_editor_property("compression_settings", ...)      # 法线必须 TC_NORMALMAP
t.set_editor_property("flip_green_channel", True)       # ★ GL 法线 -> DX
EditorAssetLibrary.save_asset(path, only_if_is_dirty=False)
```

| 贴图类型 | sRGB | compression |
|---|---|---|
| `*_diff_*` / `*_basecolor_*` | **True** | `TC_DEFAULT` |
| `*_nor_dx_*`（DirectX 法线） | False | `TC_NORMALMAP`（**不翻绿通道**） |
| `*_nor_gl_*`（OpenGL 法线） | False | `TC_NORMALMAP` + **`flip_green_channel=True`** |
| `*_rough_*` / `*_ao_*` | False | `TC_DEFAULT` |
| `*_alpha_*` / `*_mask_*` | False | `TC_GRAYSCALE` |

**Polly Haven 的贴图命名**：`nor_dx` = UE 能直接用；`nor_gl` = 必须翻绿通道。**弄错的表现是"光照方向怪、凹凸感反了"**，很难一眼看出，所以导入后立刻设。

## 2) 用 Python 建 PBR 材质（TextureSample + UV 平铺）

```python
mat = AssetTools.create_asset(name, path, unreal.Material, unreal.MaterialFactoryNew())

coord = create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate, -900, 0)
coord.set_editor_property("u_tiling", 100.0)      # ★ 1k 贴图铺 200+ m 必须平铺，否则糊成一片
coord.set_editor_property("v_tiling", 100.0)

s = create_material_expression(mat, unreal.MaterialExpressionTextureSample, -600, yy)
s.set_editor_property("texture", tex)
connect_material_expressions(coord, "", s, "Coordinates")     # 引脚名是 "Coordinates"
connect_material_property(s, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
# 法线: "RGB" -> MP_NORMAL ; 粗糙度/AO: "R" -> MP_ROUGHNESS / MP_AMBIENT_OCCLUSION
# 透明度遮罩: "R" -> MP_OPACITY_MASK
```
`connect_material_property(expr, 输出引脚名, MP_xxx)` 的**输出引脚名**：`"RGB"`/`"R"`/`"A"`/`""` ✓

**树叶/针叶材质的正确配法**（缺一个都不对）：
```python
mat.set_editor_property("two_sided", True)
mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)                       # 用 alpha 镂空
mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_TWO_SIDED_FOLIAGE) # 双面叶
# alpha 贴图 "R" -> MP_OPACITY_MASK
```

**⚠️ 不要 `MaterialEditingLibrary.recompile_material(mat)`** —— 每个材质触发一次着色器编译，第 2 个就卡死（实测卡 180 s 无响应）。建完统一 `save_asset`，引擎在关卡加载时自会编译。

## 3) ⚠️ 硬限制：**UE 4.27 Python 不能动态创建组件 → HISM 做不了**

实测（`dir()` 探针）：
```
has HierarchicalInstancedStaticMeshComponent  True      # 类存在
has HierarchicalInstancedStaticMeshActor      False     # 但 Actor 类不存在 ✗
Actor 方法里没有 add_component_by_class / add_instance_component   ✗
EditorLevelLibrary 只有 spawn_actor_from_class / spawn_actor_from_object  ✗ 无创建组件接口
```
→ **无法**用 Python 往关卡里加 HISM/ISM 组件。**替代**：
- **N 个独立 StaticMeshActor**（实测 137 个完全可用；静态网格渲染器按 mesh+材质合批）✓
- 需要 HISM 就**在编辑器里手工放一个**，Python 之后用 `get_component_by_class` 拿到它再加实例 ✓

**别在交付文档里写"HISM instances"却实际用独立 actor** —— 分开写清楚。

## 4) 真实模型按实测尺寸摆放：先读原生 bounds 再算缩放

```python
b = mesh.get_bounds()                       # StaticMesh.get_bounds() ✓ 存在
native_cm = (b.box_extent.x*2, b.box_extent.y*2, b.box_extent.z*2)
# 关键判断：mesh 局部原点是否在【底部中心】？
#   bounds.origin.z ≈ box_extent.z  ->  底部在 z≈0，摆放时 location.z 直接设地面高度即可 ✓
#   否则要先加偏移
scale_z  = target_height_m * 100 / native_cm[2]
scale_xy = target_crown_radius_m * 100 / (native_cm[0]/2)
scale_xy = clamp(scale_xy, scale_z*0.6, scale_z*1.6)   # ★ 限制非均匀缩放幅度，否则树被压扁/拉尖
```
FBX 导入时 `static_mesh_import_data.combine_meshes = False` → FBX 里的多个变体会拆成**独立资产** ✓
（这句话能拿到"3 个真实变体"，避免整片林子重复一个轮廓）

LOD 批量生成：
```python
settings = [unreal.EditorScriptingMeshReductionSettings(percent_triangles=p, screen_size=s)
            for p, s in ((1.00,1.00),(0.25,0.58),(0.08,0.24),(0.025,0.07))]
opts = unreal.EditorScriptingMeshReductionOptions(auto_compute_lod_screen_size=False,
                                                  reduction_settings=settings)
unreal.EditorStaticMeshLibrary.set_lods(mesh, opts)     # 返回 LOD 数
```

## 5) 隐藏碰撞体模式（"看得见的是树，撞得到的是树干"）

真实树的网格有 15 万顶点 → **不能**给整棵树做 trimesh 碰撞（性能炸）。
**做法**：另生成一个**只有树干锥台**的网格，放进关卡但**隐藏渲染**，用它的 trimesh 做碰撞：
```python
comp.set_visibility(False, True)     # 或 comp.set_editor_property("visible", False)
# 碰撞仍在！CTF_USE_COMPLEX_AS_SIMPLE 照常生效
```
好处：
- 飞机在 1~5 m 高度**只能撞树干**，可从枝叶下穿行（符合真实：树冠枝叶不挡多旋翼）✓
- 碰撞体面数极小（137 棵树 ≈ 5 千面）✓
- **必须在交付说明里写明"树干碰撞体是隐藏的"**，否则用户会困惑"看不见却撞到"

## 6) 交付文档必须记录的限制

源点云**没有 CRS/VLR** 时，元数据里要写清：
```json
"ue_transform": {
  "units": "centimetres",
  "mapping": "UE_X=src_X*100; UE_Y=-src_Y*100; UE_Z=src_Z*100",
  "georeference_status": "local coordinates only; source LAS has no CRS/VLR" }
```
并额外给一份**面向人的**说明（md）：**尺度可信、相对位置可信、绝对定位与朝向不可信**（未知 EPSG / 真北未知）→ 可用于算法验证，不可与真实地图/GNSS/实飞航点做绝对对齐。

## 7) 数字自洽校验（每次交付前算一遍）

- `关卡 actor 数 = 环境 actor + 地形 + 隐藏碰撞体 + 树数`（例：9+1+1+137 = 148 ✓）
- `Σπ·冠幅半径²` ≈ 冠层像素数（单木分割自洽）
- 地形 OBJ 法线朝上占比 = 100%
- 每棵树的材质槽都设了（`slot0 ↔ Bark`, `slot1 ↔ Twig`），否则是默认灰材质
- 启动后 `Game class is 'AirSimGameMode'` + 41451 监听 + 日志无 Error/Fatal
