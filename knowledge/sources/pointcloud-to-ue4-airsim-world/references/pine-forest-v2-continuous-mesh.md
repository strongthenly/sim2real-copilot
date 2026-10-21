# v2：从"杂乱体素"升级到"连续地形 + 真实感树木"

v1 用 0.5 m 体素方块堆场景 → 用户反馈**太杂乱**、**按 Play 直接掉入虚空**。v2 的做法与踩到的三个坑：

## 坑 A（最致命）：**面绕序不要手动翻 —— UE4 已经翻过**

v1 里为了抵消 UE4 的 Y 镜像，我在 OBJ 里**手动翻转了面绕序**。结果 UE4 里
地形 **127,008 个面 100% 法线朝下** → 背面剔除 → **地面完全看不见**
→ 表现为"一堆悬空方块（杂乱）"+"视觉上掉入虚空" ✗

**判定方法（写代码算，别靠眼睛）**：
```python
V = np.array([...]); F = np.array([...])
n = np.cross(V[F[:,1]] - V[F[:,0]], V[F[:,2]] - V[F[:,0]])
print("朝上 %d / 朝下 %d" % ((n[:,2] > 0).sum(), (n[:,2] < 0).sum()))
```
**结论**：UE4 的 OBJ 导入**自己会处理镜像导致的绕序反转** →
- **不要手动翻绕序**（保持 `(a,c,b)/(b,c,d)` 这种原始网格顺序）
- 同时**给材质设 `two_sided = True`** 兜底 → 法线朝向问题彻底免疫 ✓✓

## 坑 B：**不要把数据网格 padding 成 2 的幂**

v1 把 203×229 的真实高程网格用 `clip` 复制边缘凑成 253×253 → **四周多出一大片复制出来的假地形**，也是"杂乱"的来源 ✗

**做 OBJ 静态网格时不需要 2 的幂**（那是 UE4 Landscape 的要求）。**直接用真实尺寸** ✓

## 坑 C：**材质创建不要调 `recompile_material`**

```python
m = asset_tools.create_asset(nm, path, unreal.Material, unreal.MaterialFactoryNew())
e1 = MaterialEditingLibrary.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -400, 0)
e1.set_editor_property("constant", unreal.LinearColor(r, g, b, 1.0))
MaterialEditingLibrary.connect_material_property(e1, "", unreal.MaterialProperty.MP_BASE_COLOR)
e2 = MaterialEditingLibrary.create_material_expression(m, unreal.MaterialExpressionConstant, -400, 200)
e2.set_editor_property("r", 1.0)                       # 粗糙度
MaterialEditingLibrary.connect_material_property(e2, "", unreal.MaterialProperty.MP_ROUGHNESS)
m.set_editor_property("two_sided", True)               # ★ 双面
# ★★ 不要 MaterialEditingLibrary.recompile_material(m) —— 每调一次触发着色器编译，
#     第 2 个材质就卡死（实测等了 180 s 无响应）。材质在关卡加载/保存时引擎自会编译
EditorAssetLibrary.save_asset(m.get_path_name())        # 全部建完再统一 save
```
去掉 `recompile_material` 后 3 个材质 + 建关卡 **15 秒**完成（原来 180 s+ 卡死）✓

## 从点云到真实感树木：单木分割 → 程序化建模

**单木分割**（scipy 就够，不用 skimage）：
```python
chm = gaussian_filter(np.maximum(grid_max - ground, 0), 1.2)   # 冠层高度模型
canopy = chm > H_MIN                                            # 3 m
peaks = canopy & (chm >= maximum_filter(chm, size=WIN) - 1e-4)  # 局部极大
# 按高度降序做非极大抑制（剔除间距 < MIN_DIST 的）
# 冠幅：把冠层像素用 distance_transform_edt(~peaks, return_indices=True) 分配给最近峰
#       -> 每棵树的冠幅像素数 -> crown_r = sqrt(area/pi)   （等效半径）
```
**自洽性校验（很重要）**：`Σπ·crown_r²` 应 ≈ 冠层像素总数。若不接近，说明分割参数不对 ✓

实测这片成熟松林：137 棵 / 冠幅中位 8.7 m / 间距 8~10 m / 119 棵每公顷 —— **自洽 ✓ 是真实密度**（不要为了"看起来密"而编树，那属于合成数据）

**程序化松树**（每棵约 130 三角面即可，效果远好于体素）：
- **树干**：12 面锥台，底径 0.30 m → 顶径 0.13 m，地面 → 0.35×树高，带底盖
- **树冠**：3 层塔形圆锥叠加（松树），各层 `(起点比例, 底半径系数, 终点比例, 顶半径系数)`
  ```
  (0.28, 1.00, 0.60, 0.52)
  (0.48, 0.82, 0.80, 0.38)
  (0.68, 0.58, 1.00, 0.00)   # 顶层收成尖
  ```
  最底层另加一个极薄锥台做**底盖**，避免从下往上看穿
- **自然感**：圆周顶点加 ±4~6% 随机半径扰动、树高 ±10%、顶点加微小抖动，**固定随机种子**保证可复现
- **裁剪异常值**：`H = min(H, 35.0)`（点云地面估计偏低会造出 42 m 的假高树）

**碰撞策略**（无人机飞 0~5 m）：
- 地形：整个网格 `CTF_USE_COMPLEX_AS_SIMPLE`（trimesh）
- 树干：锥台自身 trimesh（12 面很便宜）
- 树冠：也给 trimesh（树冠底约在 0.28×树高 ≈ 6.7 m，飞 1.5 m 时撞不到；但飞 5 m 就会撞到低枝，符合真实）
- 树冠底面以上的部分天然不会挡 1.5 m 高度的飞行 → **可以从树冠下穿行**，树干是唯一低空障碍

## 关卡派生（沿用 v1 的正确做法）

```python
unreal.EditorLevelLibrary.load_level("/Game/FUEL/FUEL_RandomRoom")   # 已知能跑 AirSim 的关卡
for a in list(get_all_level_actors()):
    if a.get_class().get_name() == "StaticMeshActor":                # ★ 只删静态网格
        destroy_actor(a)
# 放新网格 + 各自 set_material(0, mat)（网格资产和组件都要设）
unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/XXX/YYY")
```
**另存为新关卡**，源关卡保持不动 ✓

## 交付前必须做的数值校验

1. OBJ 的 `v` 行范围与预期逐位对比
2. **法线朝向统计**（朝上/朝下）—— 朝下 = 地面不可见，必崩
3. `verts(lod0)` 与预期顶点数一致
4. `Σπ·crown_r² ≈ 冠层像素数`（单木分割自洽）
5. 网格的 `body_setup.collision_trace_flag` = `CTF_USE_COMPLEX_AS_SIMPLE`
6. 每棵树的材质都 `set_material(0, mat)` 成功（否则是默认灰网格材质，看着像"没贴图"）
