# 点云 → 深度图 / RGB-D（三种方案实测对比）

## 方案对比

| 方案 | 覆盖率 | 形状准确度 | 需要什么 |
|---|---|---|---|
| A. 球面 range image | 稀疏（单帧 1.9%） | 雷达视角 | 无（仅点云） |
| B. 单帧雷达 → 相机投影 | 1.9% | 稀疏、细结构变形 | K + Tcl 外参 |
| C. **SLAM 地图投影**（推荐） | **40.8%** | ✅ 与彩图对齐 | K + Tcl + odometry 位姿 |

C 方案原理：`p_cam = Kcl @ inv(T_odom_k) @ p_slam`——把 cloud_slam 全局地图（odom 系，覆盖整个飞行区域）投影到第 k 帧相机视角。**解决了单帧雷达的两个致命问题**：视场外物体无深度（如图像左边相机看到但雷达没扫到）、细长结构（电线杆）形状变形。

## 深度图生成要点

- 投影：`p = ph @ T_cam.T`，`z = p[:,2]`，`ok = z > 0.05`，`u = round(p[0]/z)`, `v = round(p[1]/z)`
- **像素取最近深度必须 `depth = np.full((H,W), np.inf)` + `np.minimum.at(flat_idx, ...)`**——初始 0 会被 min 吞掉（本次会话最大坑）
- 扁平索引：`np.minimum.at(depth.ravel(), nv*W+nu, zz)`
- 输出：16bit PNG（单位 mm，`depth*1000`）、npy 数组
- 可视化：turbo 伪彩色（红近蓝远），`norm = clip(depth/MAX_M, 0, 1)`，MAX_M≈25m

## 深度补洞（可视化用，稀疏深度→稠密）

1. 最近有效值填充：`idx = ndimage.distance_transform_edt(~mask, return_indices=True)`; `filled = depth[tuple(idx)]`
2. 引导滤波平滑（保边缘）：以彩图灰度为引导图，`scipy.ndimage.uniform_filter` 实现 guided filter（r≈6, eps≈1e-4）
3. 有效像素保留原值，补洞区域用滤波结果
4. **补洞值是估计值**：可视化/浏览用；定量分析用原始 16bit
5. 提示用户：补洞后覆盖 100% 但形状在有效数据极稀疏时仍会变形——根治靠方案 C

## RGB-D 对导出（rgbd_export.py）

- 图像（`/odin1/image/compressed`，JPEG）与点云帧**一一对应**（同 header，seq 相同，dt~1e-10s）——按序或按时间戳最近匹配都行
- 每组输出：去畸变彩图（inv_map remap，与深度对齐）+ 16bit 深度 + 原始鱼眼图 + 预览
- 抽稀参数 `--every N`（每 N 帧取 1 组）控制体量
- 时间戳：ROS1 `sec u32 + nsec u32`（`sec + nsec/1e9` 秒）；bag 内是录制后相对秒数（如 296.45s）

## 时间对齐注意

- 图像/点云/odometry 用同一 Header 格式解析（seq 4B + stamp 8B + frame_id string）
- Odometry 解析：跳过 frame_id **和 child_frame_id** 两个 string 后才是 pose（pos 3×f64 + quat 4×f64）
- 位姿四元数顺序 (x,y,z,w)，转旋转矩阵标准公式
