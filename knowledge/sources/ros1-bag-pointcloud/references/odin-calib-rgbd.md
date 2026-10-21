# odin 相机标定 + RGB-D 深度图导出

## 标定参数来源

fouruav 机载电脑（SSH：plink，fouruav/密码 <SSH_PASSWORD>）：
- `~/odin_ws/src/odin_ros_driver/config/calib.yaml` — 官方标定（设备 O1-N090100105）
- `~/odin_ws/src/odin_ros_driver/include/polynomial_camera.hpp` — FishPoly 相机模型源码
- `~/odin_ws/src/odin_ros_driver/src/pointcloud_depth_converter.cpp` — 官方点云→深度算法
- 另有 `~/nvid_ws/src/ros_lidar_saver/config/fusion_params.yaml`（用户自写融合配置，含 lidar_to_camera + 内参，与官方略有差异）

## 官方标定值（calib.yaml）

```
cam_model: FishPoly          # 多项式鱼眼（非针孔！）
image: 1600 x 1296
A11 = 730.15941250505863   A12 = -0.38404991252279618   A22 = 730.06326545815682
u0  = 786.58292415336745   v0  = 647.19096056101932
k2 = 1.4580e-2  k3 = -8.6729e-2  k4 = 1.6441e-1  k5 = -1.9743e-1  k6 = 1.1604e-1  k7 = -2.8092e-2
maxIncidentAngle = 120      # 超广角
Tcl_0 (雷达→相机 4x4):
[ 0.00186, -0.99994, -0.01105,  0.03118
 -0.00663,  0.01104, -0.99992,  0.0121
  0.99998,  0.00193, -0.00661, -0.03035
  0,        0,        0,        1      ]
```

## FishPoly 投影（world2cam，来自 polynomial_camera.hpp）

```python
r = sqrt(x² + y²)                    # 相机系归一化坐标
theta = acos(z / norm)               # 或 atan(r) 视角
thetad = theta + k2 θ² + k3 θ³ + k4 θ⁴ + k5 θ⁵ + k6 θ⁶ + k7 θ⁷
scaling = thetad / r
xd = x * scaling; yd = y * scaling
px = xd*fx + yd*skew + cx;  py = yd*fy + cy
```

## 官方 RGB-D 算法（pointcloud_depth_converter.cpp）

SDK 的 pcd2depth_node 做法（用 K 内参针孔投影，不用畸变多项式投影点云）：
1. `Kcl = K4x4 × Tcl`（K4x4 = diag(A11,A12,u0; 0,A22,v0; 0,0,1) 扩成 4x4）
2. 点变换：`pc = Kcl @ [x,y,z,1]^T`
3. 过滤 `z <= 0.05`，`u = round(pc[0]/z)`, `v = round(pc[1]/z)`，界内写入 `depth[v,u] = z`（米）
4. 3×3 邻域填充空洞（同深度）→ resize 到 1600×1296 → Sobel 梯度阈值 0.75 去边缘伪影
5. 彩图用 inv_map（每个像素反推鱼眼坐标）remap 去畸变 → 与深度图对齐

**深度图写入的 numpy 陷阱**：每像素取最近点要用 `np.full(inf)` + `np.minimum.at(flat_idx, ...)`，
0 初始化会被 `min(0, z) = 0` 吞掉导致深度图全 0。

## 图像与点云对齐

- `/odin1/image/compressed`（JPEG 1600×1296）与 `/odin1/cloud_raw` **同 header、同 seq、帧数相同**
  → 第 i 帧 ↔ 第 i 帧（或按时间戳最近匹配，dt<0.1s）
- 图像帧率约 1Hz（334 帧/6 分钟），点云 10Hz——但 odin 里两者同频发布

## RGB-D 导出产物（rgbd_export.py）

每组：`*_color_<stamp>.jpg`（去畸变对齐彩图）+ `*_depth_<stamp>.png`（16bit mm）+
`*_raw_<stamp>.jpg`（原始鱼眼）+ `*_preview_<stamp>.png`（三图对比）。
单帧深度图有效像素约 2.5%（雷达 4.9 万点 + 相机 120°FOV 只覆盖前方）——稀疏是物理事实。
