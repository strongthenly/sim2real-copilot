# odin 系统（留形科技）bag 专项

## 环境事实

- **SDK 位置**：fouruav（Jetson NX）`~/odin_ws/src/odin_ros_driver/`（launch_ROS1/odin1_ros1.launch 起 host_sdk_sample；自带 pcd2depth_node/cloud_reprojection_node/image_overlay_node）
- **标定文件**：`~/odin_ws/src/odin_ros_driver/config/calib.yaml`（设备 O1-N090100105）
- 用户融合配置：`~/nvid_ws/src/ros_lidar_saver/config/fusion_params.yaml`（lidar_to_camera 矩阵，与 calib 略不同）
- 登录：plink.exe -pw <SSH_PASSWORD> fouruav@fouruav（hostkey SHA256:<ONBOARD_HOSTKEY_FP>）

## odin bag 特征（非标准格式）

- 文件头字段规范（index_pos/chunk_count/conn_count 正常），但 **chunk header 字段值为裸字节**：compression 裸 "none"（无 u32 长度前缀）、size 为 4 字节 u32
- 连接记录仅 `conn/op/topic` 三字段，无 type/md5sum/message_definition
- 消息体是标准 ROS1 序列化（可手工解析）
- 图像(/odin1/image/compressed)与点云(/odin1/cloud_raw) **帧数相等、同 seq、同 stamp**（dt ~1e-10s，一一对应）——按序配对即可，无需复杂同步
- stamp 是 ROS1 标准 sec u32 + nsec u32（读成 u64 会错 ~13 年量级）

## 话题表

| 话题 | 类型 | 内容 |
|---|---|---|
| /odin1/cloud_raw | PointCloud2 | 原始雷达帧 49152 点/帧，fields: x y z intensity(u8) confidence(u16) offset_time，frame=odin1_base_link |
| /odin1/cloud_slam | PointCloud2 | SLAM 地图帧（odom 系），fields: x y z rgb（**rgb 是 float32 打包**，PCL 风格），非累积式（每帧独立 ~1.3 万点），带真彩色 |
| /odin1/odometry | nav_msgs/Odometry | frame=odom child=odin1_base_link，位姿=雷达→odom |
| /odin1/image/compressed | CompressedImage | 1600×1296 JPEG 鱼眼图 |
| /odin1/imu、/mavros/*、/tf、/tf_static | | 辅助话题（/tf 只有 odom→map 单位变换，无相机外参） |

## 相机标定（calib.yaml，FishPoly 鱼眼模型）

```
cam_model: FishPoly    image_width: 1600    image_height: 1296    maxIncidentAngle: 120
A11: 730.15941250505863   A12: -0.38404991252279618   A22: 730.06326545815682
u0: 786.58292415336745    v0: 647.19096056101932
k2: 1.4579599839177437e-02  k3: -8.6729039634919721e-02  k4: 1.6441206565323623e-01
k5: -1.9743481946801402e-01  k6: 1.1604312461490942e-01  k7: -2.8092352949327107e-02
p1: 0.  p2: 0.
```

**外参 Tcl_0（雷达→相机 4×4，行主序）**：
```
[ 0.00186, -0.99994, -0.01105,  0.03118,
 -0.00663,  0.01104, -0.99992,  0.0121,
  0.99998,  0.00193, -0.00661, -0.03035,
  0,        0,        0,        1      ]
```

## SDK pcd2depth 官方算法（pointcloud_depth_converter.cpp 复现）

1. `K_4x4 = diag(A11, A22, 1, 1)` + A12 于 (0,1)；`Kcl = K_4x4 @ Tcl`
2. 点变换 `p' = Kcl @ [x,y,z,1]^T`；`z<=0` 丢弃；`u=round(p'[0]/p'[2])`, `v=round(p'[1]/p'[2])`
3. `depth[v,u] = z`（相机系深度，米）+ **3×3 邻域填充**（空像素填该点 z）
4. 后处理：resize 到 1600×1296（最近邻）+ Sobel 梯度 >0.75 的边缘像素置 0（去伪影）
5. 彩图对齐：逆畸变映射 `inv_map` remap（θ_d=θ+k2θ²+...+k7θ⁷，x_distorted=x·θ_d/r）→ 针孔视角彩图与深度图对齐

## RGB-D 导出约定

- 每组输出：`<bag>_color_<stamp>.jpg`（去畸变对齐彩图）、`<bag>_depth_<stamp>.png`（**uint16 mm**，1600×1296）、`<bag>_raw_<stamp>.jpg`（原始鱼眼）、`<bag>_preview_<stamp>.png`
- 单帧深度图有效像素 ~2%（雷达 4.9 万点 + 相机 120°FOV 只覆盖前方）是正常的
- 批量导出用 `--every N` 抽稀（图像 10Hz，如每 5 帧取 1）

## 已知坑

- 图像/点云都是 10Hz 帧率（0.0975s 间隔），不是 1Hz/2Hz——验证抽稀是否生效要看文件 stamp 间隔 = N×0.0975s
- cloud_slam 每帧点数波动（6000~20000）是"每帧独立地图"，merge 全部帧 + 0.03m voxel 去重才是完整地图
