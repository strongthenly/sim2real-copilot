# odin 系统自定义 bag 格式与标定（2026-07 实测）

## bag 结构（顺序解析器需要处理的特性）

文件头：`#ROSBAG V2.0
` + `u32 header_len` + header + `u32 data_len` + data（**data 部分必须跳过**，否则后续记录错位）。header 字段 = 连续 `u32 len + "key=value"`（无分隔符），字段含 `op=0x03`、`chunk_count`(u32)、`conn_count`(u32)、`index_pos`(u64)。

文件级记录：chunk（op=0x05）、index data（op=0x04）。**chunk header 的字段值是非标准序列化**：
- `compression` 是裸字符串（`b'none'`，无 u32 长度前缀）
- `size` 是 **4 字节 u32**（标准是 8 字节 u64）——读取时按 `len(sv)>=8 ? '<Q' : '<I'` 自适应
- `op` 是裸 1 字节

chunk 内部记录（连接/消息）格式与文件级相同：`u32 hlen + header + u32 dlen + data`。

**连接记录只有 3 个字段：`conn`(u32)、`op`=0x07、`topic`(裸字符串)**——没有 type/md5sum/message_definition。所以类型必须靠**消息字节嗅探**：
- PointCloud2：header 后 height(u32)≤10，fields 数组可解析且字段名包含 x/y/z
- Livox CustomMsg：header 后 timebase(u64) > 1e15 且 point_num 与数组长度匹配（`abs(arrlen - pn) <= 1`，arrlen 在 pos+13）

该 bag 的话题（10 个连接，全部 ROS1 序列化消息体）：
`/mavros/imu/data`、`/tf_static`、`/odin1/imu`、`/mavros/local_position/pose`、`/tf`、`/odin1/image/compressed`(JPEG 1600×1296)、`/odin1/cloud_raw`(PointCloud2: x y z intensity confidence offset_time，49152 点/帧)、`/odin1/odometry`(nav_msgs/Odometry)、`/odin1/cloud_slam`(PointCloud2: x y z rgb，frame_id=odom，每帧 1.3 万点独立地图，非累积)。

## odin 相机标定（calib.yaml，设备 O1-N090100105，fouruav 上 `~/odin_ws/src/odin_ros_driver/config/calib.yaml`）

- 模型：**FishPoly（多项式鱼眼）**，图像 1600×1296，maxIncidentAngle=120°
- 内参：A11=730.1594, A12=-0.3840, A22=730.0633, u0=786.583, v0=647.191
- 畸变：k2=1.458e-2, k3=-8.673e-2, k4=1.644e-1, k5=-1.974e-1, k6=1.160e-1, k7=-2.809e-2
- 外参 Tcl_0（雷达→相机 4×4，行主序）：
  ```
  [0.00186, -0.99994, -0.01105, 0.03118]
  [-0.00663, 0.01104, -0.99992, 0.0121]
  [0.99998, 0.00193, -0.00661, -0.03035]
  [0, 0, 0, 1]
  ```
  （雷达 x 轴 ≈ 相机 z 轴光轴；第三行旋转 ≈ [1,0,0] → 相机系 z ≈ 雷达系 x）

## SDK 官方点云→深度算法（pointcloud_depth_converter.cpp 复现）

1. `Kcl = K4x4 @ Tcl`，其中 K4x4 = diag(A11, A12→(0,1), A22, 1, 1) + u0/v0
2. 点变换：`p_cam = Kcl @ [x,y,z,1]`，深度 = `z` 分量（相机系深度）
3. 像素：`u = round(p_cam[0]/z)`, `v = round(p_cam[1]/z)`，`z > 0` 才投影
4. 3×3 邻域填充空洞
5. 彩图对齐：用逆畸变映射 remap 把鱼眼图转针孔视角（与深度图视角一致）

## SDK 里可参考的源码（fouruav）

`~/odin_ws/src/odin_ros_driver/`：`include/polynomial_camera.hpp`（FishPoly 投影公式）、`src/pointcloud_depth_converter.cpp`、`src/pcd2depth_ros.cpp`、`src/depth_image_ros_node.cpp`、`src/cloud_reprojector.cpp`。launch 文件 `launch_ROS1/odin1_ros1.launch` 启动 host_sdk_sample + pcd2depth_node + cloud_reprojection_node。
