# 实机复刻（数字孪生）：外形 + 传感器 + 数据流

目标：AirSim 里的无人机**外形像实机、传感器/数据流与实机一致**，而飞行仍由**真 PX4** 控制。
（实机参照：LiyumenX8 + MID360(S) LiDAR + D435 相机 + NxtPX4v2 飞控 + Jetson NX。）

## 一、外形复刻（FBX → Unreal → BP_FlyingPawn）
只换**视觉 mesh**，不改动力学 —— 飞行物理仍是 PX4（这正是要的：外形像实机、控制是真 PX4）。

**素材格式（优先级）**
1. **`.fbx`（首选）**——Unreal 原生最支持，几何+材质+分部件一体。
2. `.obj` / `.gltf/.glb` —— 次选（材质另配）。
3. CAD（STEP/IGES/SolidWorks）**不能直接进 Unreal** → 先导出 FBX/OBJ，或用 UE **DataSmith** 插件导 CAD。

**导出/导入要求**
- 真实尺寸（按实机轴距/机身），**单位 mm 或 m**，FBX 导出选 **cm**（UE 内部 1 unit = 1cm）。
- **Y-up**（导入 Unreal 自动转 Z-up / 勾 `Convert Scene`）。
- **面数适中**：几十万面以上在 RTX3050 4GB 上会卡 → 需要减面 / LOD。
- 材质/贴图随附（PNG/JPG 或 FBX 内嵌）。
- 分部件（机身 + 4 臂 + 4 旋翼）更好 —— 以后可让旋翼转。

**UE4.27 导入 + 换 mesh 步骤（GUI）**
1. 停 Play（`Esc`）→ Content Browser 新建文件夹 → 点 **Import** → 选 `.fbx`。
2. FBX Import Options：勾 **`Combine Meshes`** + **`Import Materials`** + **`Convert Scene`**；`Import Uniform Scale` 先 **1.0**（后校准）→ **Import All** → 得到 StaticMesh。
3. **⚠️ 关键坑：AirSim 插件内容在 UE 里默认不显示，直接找是找不到 `BP_FlyingPawn` 的**（AirSim 官方 `docs/working_with_plugin_contents.md` 原话：**"Plugin contents are not shown in Unreal projects by default."**）。定位法（任选）：
   - **最省事 = Content Browser 顶部搜索框直接输 `BP_FlyingPawn` 回车**（实测：勾 `Show Plugin Content` 后左侧仍看不到文件夹，但**搜索框能搜到** → 用这个）；
   - 或 **`View Options`（Content Browser 右下角齿轮图标）→ 勾 `Show Plugin Content`**（勾上后目录树出现 `AirSim Content`）；
   - 或展开左侧 **Sources 面板底部的 `Plugins`** 节点。
   文件实体在 `E:\AirSim\Unreal\Plugins\AirSim\Content\Blueprints\BP_FlyingPawn.uasset`（可直接确认存在）。→ 双击打开蓝图。
4. 左侧 **Components 组件树** → 找 mesh 组件（名可能是 `Body`/`Mesh`/`QuadRotor`/`StaticMeshComponent`，**BP_FlyingPawn 结构因版本而异**；找不到就把组件树截图）→ 右侧 **Details → Static Mesh** 换成导入的 X8 模型。
5. 调 **Scale**（按实机轴距校准缩放）+ 位置归中（旋翼大致对齐原 4 旋翼位）→ **Compile + Save**。
6. 回主编辑器 Play → 无人机变实机外形。

**注**：UE 项目默认**没启用 PythonScriptPlugin** → 不能用 UE Python 脚本自动导入，需在编辑器里手动 Import（或先改 `.uproject` 启用 Python 插件）。

## 二、传感器一致性（与实机对齐）
| 层面 | 实机 | AirSim 能否复刻 |
|---|---|---|
| 雷达 | MID360（Livox） | ⚠️ **难点**：AirSim 原生 LiDAR 是**机械重复扫描**，MID360 是**非重复（花瓣）扫描** —— 原生不支持精确复刻，只能**近似**（点云形态有差异）或写 custom sensor / 用 Livox 官方仿真插件 |
| 相机 | D435（RGBD） | ✅ AirSim 可配 RGB + Depth 相机（FOV/分辨率/频率按 D435 参数近似） |
| IMU | NxtPX4v2 上的 IMU | ✅ AirSim 可配 IMU（噪声/频率按 datasheet），PX4 用 AirSim 的 sensor（HIL） |
| 数据流 | 雷达/相机 → ROS 话题 → FAST-LIVO2 | ✅ 把 AirSim 传感器桥到**与实机同名话题**（如 `/livox/lidar`、`/camera/depth`、IMU）→ 算法**不改**即可跑 |

## 三、分层实施路线（从易到难）
1. **外形**：导入 X8 FBX → 换 `BP_FlyingPawn` mesh（立竿见影）。
2. **相机**：AirSim 配 Depth+RGB（D435 近似）。
3. **IMU**：AirSim 配 IMU（按实机噪声）+ PX4 用。
4. **雷达**：AirSim LiDAR 近似 MID360（非重复扫描是最大障碍，后续 custom）。
5. **数据流**：AirSim 传感器 → ROS2 → **与实机同名话题** → FAST-LIVO2 等直接跑。

## 四、需要向用户采集的信息（才能配准）
1. 实机型号/尺寸（轴距、机身）+ 3D 模型（FBX/CAD）。
2. 雷达型号（MID360/MID360S？）+ 安装位姿。
3. 相机型号（D435？）+ 安装位姿。
4. **实机的 ROS 话题名**（雷达点云/图像/IMU 各叫什么）。
5. 实机跑的算法（FAST-LIVO2？）——对齐它的输入。
