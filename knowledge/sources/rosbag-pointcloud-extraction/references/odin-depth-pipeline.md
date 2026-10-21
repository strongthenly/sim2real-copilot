# odin 深度图 / RGB-D 生成管线（数学细节）

来源：odin SDK 源码（polynomial_camera.hpp / pointcloud_depth_converter.cpp）+ calib.yaml 官方标定，Windows 上用 numpy 重写。

## 1. 标定参数（calib.yaml，设备 O1-N090100105）
```
cam_model: FishPoly        # 多项式鱼眼（非针孔！）
image: 1600×1296
A11=730.15941250505863  A12=-0.38404991252279618  A22=730.06326545815682
u0=786.58292415336745   v0=647.19096056101932
k2=1.4579599839177437e-02  k3=-8.6729039634919721e-02  k4=1.6441206565323623e-01
k5=-1.9743481946801402e-01 k6=1.1604312461490942e-01  k7=-2.8092352949327107e-02
maxIncidentAngle: 120      # 120° 超广角
Tcl_0（雷达→相机 4×4，行主序）:
[0.00186,-0.99994,-0.01105,0.03118;
 -0.00663,0.01104,-0.99992,0.0121;
 0.99998,0.00193,-0.00661,-0.03035;
 0,0,0,1]
```

## 2. 点云 → 深度图（SLAM 地图投影，覆盖 ~40.8%）
```
K  = [[A11, A12, u0],[0, A22, v0],[0,0,1]]
K4 = eye(4); K4[:3,:3] = K
Kcl = K4 @ Tcl                      # 内参×外参合并 4×4
# 每帧: T_odom = 该帧 odometry 位姿(4×4, frame_id=odom/child=odin1_base_link)
T_cam = Kcl @ inv(T_odom)           # slam地图(odom系) → 当前帧相机归一化坐标
pc = slam_homo @ T_cam.T            # (N,4), slam_homo 预计算 float32 (N,4)
z = pc[:,2]; ok = z > 0.05
u = round(pc[ok,0]/z[ok]); v = round(pc[ok,1]/z[ok])
# 有效: 0<=u<1600, 0<=v<1296; depth[v,u] = z(米)
# 3×3 邻域填充: np.minimum.at(flat, nv*W+nu, zz), flat 必须 inf 初始化(0会被min吞)
```
注意：Kcl@inv(T_odom) 中 z 分量 = Tcl 第三行 @ inv(T_odom) @ p ≈ 雷达系 x（Tcl 第三行≈[1,0,0,-0.03]），所以相机 z 正 = 雷达前方。

## 3. 鱼眼图 → 针孔图（去畸变，与深度对齐）
每像素 (u,v) 预计算逆映射（1600×1296 一次，存 map_x/map_y float32）：
```
y = (v - v0) / A22
x = (u - u0 - A12*y) / A11
r = sqrt(x²+y²); theta = atan(r)
theta_d = theta + k2θ² + k3θ³ + k4θ⁴ + k5θ⁵ + k6θ⁶ + k7θ⁷
xd = x*(theta_d/r); yd = y*(theta_d/r)
map_x[u,v] = xd*A11 + A12*yd + u0
map_y[u,v] = yd*A22 + v0
```
重采样：`scipy.ndimage.map_coordinates(rgb, [map_y, map_x], order=1)`（双线性，3 通道逐通道）。这是鱼眼"拉直"成普通透视，使深度（针孔投影）与彩图像素级对齐。

## 4. 深度补洞（可视化用，估计值非测量）
1. `idx = ndimage.distance_transform_edt(~mask, return_indices=True)` → 最近有效值填充
2. guided filter（引导=彩图灰度，r=6, eps=1e-4）：uniform_filter 盒式滤波实现 `a=cov_Ip/(var_I+eps), b=mean_p-a*mean_I, q=mean_a*I+mean_b`
3. 有效像素保持原值，补洞区域用滤波结果；渲染时补洞区亮度×0.8 以示区分
- 补洞形状对细长物（电线杆）仍可能变形——用户投诉过，根本解法是 SLAM 地图投影（真数据密度高）

## 5. 单帧雷达投影（rgbd_export，仅测试）
同样 Kcl 但直接对单帧 cloud_raw 点投影：覆盖仅 1.9%（4.9万点/207万像素 + 相机120°FOV只覆盖前方）——用户视为不可用，别作为交付方案。

## 6. RGB-D 输出约定
- depth_raw.png：16bit PNG，单位 **mm**（`clip(depth*1000, 0, 65535).astype(uint16)`），Pillow `mode='I;16'`
- depth.png：turbo 伪彩色 8bit（0~25m 归一化，红近蓝远）
- color.jpg：去畸变后彩图（quality=92），与深度像素级对齐
- 帧文件夹以时间戳命名（`f'{st:.6f}'`），st = sec + nsec/1e9
