---
name: ros1-bag-pointcloud
description: ROS1 bag 点云提取/转换/深度图导出（Windows 本机即可，无需安装 ROS）。兼容标准 bag（rosbags 库）与索引损坏/自定义非标准 bag（内置顺序解析器）。触发词：bag、pcd、点云、LAS、深度图、RGB-D、rosbags、cloud_raw、cloud_slam、odin。
---

# ROS1 bag 点云处理（Windows 无需 ROS）

从 ROS1 bag 提取点云、构建点云地图、导出深度图/彩色图数据对。核心场景：用户只有 bag 文件（原始点云/工程文件丢失），需要补救出点云数据。

## 工具链（Windows 本机）

- Python 3.12（winget 装，`C:\Users\<USER>\AppData\Local\Programs\Python\Python312`，需手动加 PATH）
- 库：`pip install rosbags numpy scipy pillow matplotlib`（清华镜像 `-i https://pypi.tuna.tsinghua.edu.cn/simple`）
- 脚本目录：`C:\Users\<USER>\Desktop\bag_tools\`：
  - `bag2pcd.py` — 通用 bag→PCD（rosbags + 顺序解析双路径，--list/--mode map|frames|merge/--voxel）
  - `batch_convert.py` — 目录下所有 bag → SLAM 地图最后一帧 PCD
  - `bag2map.py` / `bag2map_color.py` — odometry 位姿拼接稠密地图（后者用 cloud_slam 颜色 KNN 着色）
  - `merge_slam.py` / `batch_merge_slam.py` — cloud_slam 全部帧合并（干净彩色地图，推荐）
  - `rgbd_export.py` — 按时间戳对齐导出 彩图+深度图 对（odin 内参内置）
  - `convert_pcd2las.py` — PCD→LAS 1.2（格式3，含 RGB）

## 两条读取路径（自动选择）

1. **标准 bag**：`rosbags.rosbag1.Reader` 读 connections。若 `connections` 为空 → 自动降级。
2. **非标准/索引损坏 bag**：内置顺序解析器（见下）。触发场景：rosbags 打开成功但 0 连接、或自定义 SDK 录制的 bag。

## 顺序解析器要点（非标准 bag 核心方法）

ROS1 bag v2.0 顺序解析，不依赖末尾索引。完整代码见 `bag2pcd.py` 的 `_open_bag_file/read_file_record/read_chunk_record/parse_fields/decompress`。

字节布局（记录 = u32 header_len + header + u32 data_len + data；header 字段 = 循环 `u32 len + "key=value"`，**无分隔符**）：

| 记录 | op | 说明 |
|---|---|---|
| 文件头 | 0x03 | 字段含 chunk_count/conn_count/index_pos |
| chunk | 0x05 | 字段 compression(none/lz4/bz2/zlib) + size；数据区=内部记录流（连接 op=0x07 + 消息 op=0x02） |
| index data | 0x04 | 跳过 |

关键坑（每个都实际踩过）：
- **文件头记录也有 data_len+data 段**，读完 header 必须 `u32 dlen + 跳过 dlen`，否则后续全部错位
- **自定义 bag 的 header 字段值为裸类型**：string 无 u32 长度前缀（`compression=b'none'` 4 字节）、chunk size 是 4 字节 u32（标准是 8 字节 u64）→ 按 `len(v)>=8 ? '<Q' : '<I'` 自适应
- **连接记录可能只有 conn/op/topic 三个字段**（无 type/md5sum/message_definition）→ 消息类型必须**字节嗅探**：
  - PointCloud2：解析 fields 数组，字段名集合含 {x,y,z} 且 3≤nfields≤20
  - Livox CustomMsg：header 后 timebase u64 > 1e15 且 point_num 与 points 数组长度匹配
- **RGB 字段陷阱**：PCL 风格 PointCloud2 的 rgb 是 `datatype=7 (float32)` 打包 → 必须 `.view(np.uint32)` 取位模式，**不能用 `.astype(np.uint32)`**（数值转换会毁掉颜色）
- **时间戳格式**：ROS1 Header 是 `sec u32 + nsec u32`（共 8 字节），不是 u64！错误读法会得到 2017/2031 年的假时间戳。正确：`sec = unpack('<I', raw, 4); nsec = unpack('<I', raw, 8); stamp = sec + nsec/1e9`
- **np.minimum.at 深度写入**：每像素取最近点必须用 `np.full(inf)` 初始化再 minimum.at，**0 初始化会被 min(0, z)=0 吞掉**（depth 全 0 的经典原因）；元组索引 (rows,cols) 用扁平索引 `nv*W+nu` 更稳

## 点云地图构建策略（重要决策）

用户要"干净、彩色、准确"的地图时：
- ❌ **odometry 位姿拼接原始帧**（cloud_raw）：数据量大但**有漂移 → 重影、糊、颜色匹配失败变灰**（KNN 着色时 odometry 坐标与 SLAM 坐标偏差超阈值）
- ✅ **SLAM 地图话题**（如 cloud_slam，frame_id=odom，带 rgb）：SLAM 优化坐标系，帧间精确对齐。**全部帧合并 + 0.03m 体素去重** = 干净彩色地图（着色率 99%+）
- 文件大小解释：bag 大是因为全帧率重复观测同一场景（同一物体被扫几十次）+ 所有话题（imu/图像/位姿）；地图大小由**场景实际表面大小**决定，去重后每 3cm 一个点即完整。用户问"为什么这么小"时用"10 分钟视频 vs 一张全景照片"类比

## RGB-D 深度图导出（odin 系统）

详见 `references/odin-calib-rgbd.md`。要点：内参外参在 fouruav 的 `~/odin_ws/src/odin_ros_driver/config/calib.yaml`；相机模型 FishPoly（多项式鱼眼）；官方算法 `Kcl = K4x4 × Tcl`，`u=round(x/z), v=round(y/z)`，深度=相机系 z；彩图用逆畸变映射 remap 与深度对齐；图像与点云同 header（seq 相同）→ 按序或按时间戳最近匹配都行。

## 用户协作偏好

- 指挥官模式：远程命令用户亲手执行，agent 给命令+判读（1-3 条/次）
- 数据质量优先于体积："干净、彩色、准确"是明确需求
- 用户在意产出能用：交付前用脚本验证（读回点数/坐标范围/着色率）再汇报

## 参考文件

- `references/odin-nonstandard-bag.md` — odin 自定义 bag 的字节布局细节、嗅探逻辑、解析代码片段
- `references/odin-calib-rgbd.md` — odin 内参/外参值、FishPoly 投影公式、RGB-D 导出算法
