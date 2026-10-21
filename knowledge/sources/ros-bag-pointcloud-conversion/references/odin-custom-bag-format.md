# odin SDK（留形科技）非标准 bag 格式细节

用户环境：无人机/车载 LiDAR 系统，厂商=留形科技（深圳），产品=odin，自研 SDK（bag 内 callerid=`/host_sdk_sample`）。

## 为什么标准工具读不了
- header 字段值为**裸字节**：string 无 u32 长度前缀（如 `compression=none` 直接 4 字节）、chunk `size` 是 4B u32（标准 8B u64）
- 连接记录只有 `conn/op/topic` 三个字段，**无 type/md5sum/message_definition**
- 文件头 `index_pos` 有效、`chunk_count`/`conn_count` 正常，但 rosbags 解析失败 → **静默返回 connections=0**
- 结论：用顺序解析 + 消息字节嗅探（见 SKILL.md），消息体本身是标准 ROS 序列化

## 话题清单（2026-07-27 批次录制）
| 话题 | 类型（嗅探） | 说明 |
|---|---|---|
| /mavros/imu/data | — | PX4 IMU |
| /mavros/local_position/pose | — | PX4 位姿 |
| /odin1/imu | — | 雷达 IMU（~200Hz，帧数约为点云的 40 倍） |
| /odin1/cloud_raw | sensor_msgs/PointCloud2 | **原始点云**，frame_id=odin1_base_link，每帧 49152 点（MID360 全帧），fields: x y z intensity confidence offset_time（intensity=u8, confidence=u16, offset_time=f32，point_step≈20） |
| /odin1/cloud_slam | sensor_msgs/PointCloud2 | **SLAM 地图帧**，frame_id=odom，fields: x y z rgb（**rgb datatype=7 f32 位模式打包**），每帧 ~6000~20000 点**波动（非累积式）**，全部帧 merge 即完整彩色参考 |
| /odin1/odometry | nav_msgs/Odometry | frame_id=odom / child=odin1_base_link，位姿=雷达→odom 变换；帧数与 cloud_raw 相同（一一对应） |
| /odin1/image/compressed | — | 相机图像（RGB 颜色来源） |
| /odin1/odometry, /tf, /tf_static | — | 位姿/坐标变换 |

## 数据规模参考
- 每 bag 时长 1~10 分钟；10Hz 录制；cloud_raw 帧数 = 秒数×10
- bag 大小 0.5~8GB；点云占绝大部分体积（每帧 ~0.93MB）
- 单 bag 全量点云 ≈ 帧数×49152 点（如 7.93GB bag ≈ 2.6 亿点）

## 丢失的原始点云格式：.olx + .bin
- odin 系统的导出/工程格式为 `.olx`（疑似索引/工程）+ `.bin`（二进制点云），**无公开文档**
- 获取格式定义的途径（按优先级）：① odin SDK 中的导出代码/头文件（搜 olx/bin/export/save/pointcloud）② 残留样本逆向（哪怕 1KB）③ 联系留形科技要《点云数据格式说明》
- 补救方案：bag 内数据完整，cloud_slam 即 odin 建图输出（真彩色、odom 系），是 .olx/.bin 的最佳替代

## 转换产物命名约定（本机 bag_tools 工具）
- `batch_convert.py`：每 bag 取 cloud_slam 最后一帧 → `<bag名>_odin1_cloud_slam.pcd`（小文件，稀疏）
- `bag2map_color.py`：odometry 拼接全部 cloud_raw + slam KDTree 着色 → `<bag名>_dense_color.pcd`（完整数据量、彩色；有 odometry 漂移→重影/灰团）
- `merge_slam.py`：cloud_slam 全部帧合并 + 体素去重(0.03m) → `<bag名>_slam_map.pcd`（SLAM 优化、真彩色、无重影，质量最佳）
- 用户验收指标：文件大小（期望≈bag 量级）、着色率（期望 >90%）、无重影/灰团
