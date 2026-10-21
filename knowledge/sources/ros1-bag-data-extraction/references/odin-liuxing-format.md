# 留形科技 odin 系统（odin1）数据格式

2026-07 逆向所得。odin = 留形科技无人机/机器人 LiDAR-SLAM 系统，相机为 120° 鱼眼。

## bag 话题结构（全部自定义 writer 非标准格式）
| 话题 | 类型 | 内容 |
|---|---|---|
| `/odin1/cloud_raw` | sensor_msgs/PointCloud2 | 原始雷达帧，49152 点/帧，frame_id=`odin1_base_link`，字段 x y z intensity confidence offset_time（**无 rgb**） |
| `/odin1/cloud_slam` | sensor_msgs/PointCloud2 | SLAM 地图，frame_id=`odom`，字段 x y z rgb。每帧 0.6~2 万点波动（**非累积式**，每帧独立局部地图；全部帧合并+0.03m 去重=完整地图） |
| `/odin1/odometry` | nav_msgs/Odometry | frame_id=`odom` child=`odin1_base_link`。数量与图像帧数相同、同一 header 发布（时间戳差 <0.1s） |
| `/odin1/image/compressed` | sensor_msgs/CompressedImage | 1600×1296 JPEG 鱼眼（120° FOV） |
| `/odin1/odometry_highfreq`、`/odin1/imu`、`/mavros/*` 等 | — | 辅助话题 |

## bag 格式特征（自定义 writer）
- header 字段值裸字符串（无 u32 长度前缀）、chunk size 4B u32、连接记录仅 conn/op/topic
- rosbags 库读 connections=0 → 必须用顺序解析器（见主 SKILL.md）
- bag 常以"同名目录/目录内多 bag"结构存放（如 `odin_data_xxx.bag\odin_data_xxx.bag`），批量时注意

## 标定参数（calib.yaml，SDK 路径 fouruav: `~/odin_ws/src/odin_ros_driver/config/calib.yaml`）
设备 O1-N090100105，相机模型 **FishPoly（多项式鱼眼）**：
- A11=730.1594125, A12=-0.3840499, A22=730.0632655, u0=786.5829, v0=647.1910
- 畸变 k2=0.014580, k3=-0.086729, k4=0.164412, k5=-0.197435, k6=0.116043, k7=-0.028092
- maxIncidentAngle=120（超广角）
- Tcl_0（雷达→相机 4×4，行主序）：
  [0.00186, -0.99994, -0.01105, 0.03118; -0.00663, 0.01104, -0.99992, 0.0121; 0.99998, 0.00193, -0.00661, -0.03035; 0,0,0,1]
- 注意 `fusion_params.yaml`（用户自己的 ros_lidar_saver）里有**另一组** lidar_to_camera 矩阵（数值略不同，可能不同设备/重标定），与官方 calib.yaml 区分使用

## SDK 深度图算法（pointcloud_depth_converter.cpp，复现要点）
1. `Kcl = K4x4 @ Tcl`（内参×外参合并为 4×4）
2. `p' = Kcl @ [x,y,z,1]`；`u=round(p'x/p'z), v=round(p'y/p'z)`；`depth[v,u] = p'z`（相机系 z，米，float32）
3. 3×3 邻域填充空洞 → customResize 到 1600×1296 → Sobel 梯度阈值(0.75)去边缘伪影
4. 彩图对齐：逆畸变映射（鱼眼→针孔）`x=(u-u0-A12*y)/A11, y=(v-v0)/A22, theta=atan(r), thetad=θ+k2θ²+…+k7θ⁷, remap` 后与针孔视角深度图对齐
5. SDK 自带节点：pcd2depth_node、cloud_reprojection_node、image_overlay_node（launch: odin1_ros1.launch）

## 用户已有工作流
- 原有点云产物为 `.olx` / `.bin`（odin 私有格式，无公开文档，需找厂商或 SDK 才能复原）
- 补救路径：bag → PCD/LAS → SLAM 彩色地图（merge cloud_slam 全帧）→ RGB-D（每帧一文件夹）
