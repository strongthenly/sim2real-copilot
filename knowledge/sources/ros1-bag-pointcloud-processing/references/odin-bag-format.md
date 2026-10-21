# odin SDK（留形科技）bag 格式细节

留形科技 odin 系统（自研 SLAM/雷达系统）录制的 ROS1 bag 为**非标准格式**，rosbags 读不到任何连接（`connections=[]`、`message_count=0`）。实测逆向结论（2026-07 会话，bag 5.2GB 样本）。

## 与标准 ROS1 bag v2.0 的差异

| 项 | 标准 | odin SDK |
|---|---|---|
| 文件头 | string 值带 u32 长度前缀 | **裸字符串**（无长度前缀） |
| chunk `size` 字段 | u64 (8B) | **u32 (4B)** |
| chunk `compression` | 带长度前缀的 string | 裸 "none"（4B） |
| 连接记录字段 | conn/op/topic/type/md5sum/message_definition/callerid/latching | **只有 conn/op/topic**（无 type！） |
| 末尾索引段 | 完整，rosbags 可读 | index_pos 有效但 rosbags 解析失败（自定义序列化导致） |

文件头其他字段正常：`chunk_count` u32、`conn_count` u32、`index_pos` u64（指向文件末尾附近）、`op=0x03`。

## 话题结构（样本 bag，10 个连接）

| conn | 话题 | 类型（嗅探） | 数量级 | 备注 |
|---|---|---|---|---|
| 0 | /mavros/imu/data | sensor_msgs/Imu | 1.7万 | |
| 1 | /tf_static | — | 1 | |
| 2 | /odin1/imu | sensor_msgs/Imu | 13.8万 | |
| 3 | /mavros/local_position/pose | — | 2.2万 | |
| 4/8 | /tf | — | 333/342 | **只有 odom→map**（单位变换），无相机外参 |
| 5 | /odin1/image/compressed | JPEG | 343 | **1600×1296** |
| 6 | /odin1/cloud_raw | **sensor_msgs/PointCloud2** | 3549 | frame_id=odin1_base_link，每帧 **49152 点**（MID360 整帧），fields: x y z intensity(u8) confidence(u16) offset_time(f32) |
| 7 | /odin1/odometry | nav_msgs/Odometry | 3549 | frame_id=odom, child=odin1_base_link；帧数与 cloud_raw 相等 → 按序配对 |
| 9 | /odin1/cloud_slam | **sensor_msgs/PointCloud2** | 3548 | frame_id=**odom**，fields: x y z **rgb(f32 打包)**；每帧 0.6~2万点（**非累积**，波动），SLAM 降采样输出 |

## 消息布局（手工解析要点）

- **PointCloud2 rgb 字段**：datatype=7 (float32)，PCL 风格打包 → `view(np.uint32)` 取位模式（R<<16|G<<8|B）
- **nav_msgs/Odometry**：header + child_frame_id + pose(7 f64: pos xyz + quat xyzw) + cov(36 f64) + twist(6 f64) + cov(36 f64)，消息约 719B
- **bag 里没有相机内参/外参**（无 camera_info 话题，tf 无相机 frame）→ 相机视角 RGB-D 深度图需要用户从 SDK 配置提供 fx/fy/cx/cy + T_cam_lidar

## 用户数据背景

- 原始点云成果文件是 `.olx` + `.bin`（odin 私有格式，无公开文档），曾丢失；bag 是唯一数据源
- 补救路径：bag → SLAM 帧 merge 彩色地图（质量最优，替代 .olx/.bin 的最佳产物）
- 数据是草地+电线杆场景（无人机室外飞行），6 分钟/段，多段连续录制（文件名时间戳递增）
