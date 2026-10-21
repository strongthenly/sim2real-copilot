---
name: bag-pointcloud-processing
description: 在 Windows（无需安装 ROS）上处理 ROS1 bag 点云数据：bag→PCD/LAS、稠密地图拼接（odometry 位姿）、SLAM 彩色地图合并、RGB-D 深度图导出、自定义格式/索引损坏 bag 的顺序解析。触发词：bag、rosbag、点云、PCD、LAS、深度图、RGB-D、cloud_raw、cloud_slam、pcd2depth、点云转深度。
---

# ROS1 Bag 点云处理（Windows，无需 ROS）

处理 ROS1 bag 里的点云（提取/转换/建图/深度图）。核心资产：`C:\Users\<USER>\Desktop\bag_tools\` 全套脚本（Python 3.12 + rosbags + numpy + scipy + pillow + matplotlib，清华镜像安装）。用户 odin 系统（留形科技）录的 bag 是非标准格式，**rosbags 读不到连接**，必须走内置顺序解析器——这是本技能最大的坑。

## 工具清单（Desktop\bag_tools\）

| 脚本 | 用途 |
|---|---|
| `bag2pcd.py` | bag→PCD，双路径（rosbags 标准 + 顺序解析 fallback），`--list/--mode map|frames|merge/--voxel` |
| `batch_convert.py` | 目录批量 bag→PCD（每个取 SLAM 地图末帧） |
| `bag2map.py` | odometry 位姿拼接 cloud_raw → 稠密地图（x y z intensity） |
| `bag2map_color.py` | 稠密地图 + cloud_slam 颜色 KDTree 传递（x y z rgb，--color-max 0.5） |
| `merge_slam.py` | cloud_slam 全部帧合并 → 干净彩色地图（--voxel 0.03 去重） |
| `batch_merge_slam.py` | 批量 merge_slam |
| `rgbd_export.py` | RGB-D 对导出（彩图+16bit 深度图，按时间戳对齐，--every 抽稀） |
| `convert_pcd2las.py` | PCD→LAS 1.2（格式 3，含 RGB） |
| `view_pcd.py` / `check_color.py` | PCD 预览图 / 着色率检查 |
| `probe2.py` / `sniff.py` / `debug_*.py` | bag 结构探查 / 消息类型嗅探 / 调试 |

## 读取路径 A：rosbags 库（标准 bag）

标准 bag（有完整索引、header 字段规范）用 `rosbags` 库。**0.11.x API 要点**（与直觉不同）：
- `Reader(path)` 构造**不接受 typestore**；`Writer(path)` 同样不接受
- Reader **没有 `deserialize` 方法** → 用 `typestore.deserialize_ros1(raw, msgtype)`（typestore 用 `get_typestore(Stores.ROS1_NOETIC)`）
- Writer 的 `add_connection(topic, msgtype, typestore=ts)`；`write(conn, ts_ns, ...)` 收的是**序列化后的字节**：`ts.serialize_ros1(msg, typename)`
- Livox CustomMsg 需自定义注册：`ts.register(get_types_from_msg(msgdef, 'livox_ros_driver2/msg/...'))`
- Reader 打开连接为空的 bag 时 `close()` 抛 `AssertionError`（`assert self.bio`）——属正常现象，忽略或用顺序解析

## 读取路径 B：内置顺序解析器（非标准/索引损坏 bag）

症状：rosbags 打开成功但 `connections=[]`、`message_count=0`。这类 bag（如 odin SDK 录制）特征：
- header 字段值**裸字符串**（无 u32 长度前缀），`size` 是 4 字节 u32（不是 u64）
- 连接记录只有 `conn/op/topic`，**没有 type/md5sum/message_definition**
- 但 chunk 数据区完整 → 顺序遍历可恢复全部数据

做法（已封装在 bag2pcd.py 的 `_open_bag_file/read_file_record/read_chunk_record/decompress/scan_bag_topics/iter_bag_msgs`，可 import 复用）：
1. 读 `#ROSBAG V2.0
` + u32 header_len + header + **跳过 data_len+data**（文件头记录也有 data 段！）
2. 顺序读记录：`u32 header_len + header(fields) + u32 data_len + data`
3. chunk(op=0x05)：按 `compression` 解压（none/lz4/bz2/zlib，lz4 需 `uncompressed_size=size`），内部再按同格式遍历：连接记录(op=0x07)、消息记录(op=0x02)
4. index data(op=0x04) 直接跳过

**类型嗅探**（连接无 type 时从消息字节判断）：解析 header 后——
- PointCloud2：height(u32)≤10 且 fields 数组字段名含 `x/y/z`（强校验，避免把 Imu/Pose 误判）
- CustomMsg：timebase(u64) > 1e15 且 point_num 与 points 数组长度一致

## 手工解析 ROS1 消息布局（自定义 bag 通用）

- **Header**：`seq u32 + stamp: sec u32 + nsec u32 + frame_id string`——**stamp 是 sec+nsec 两个 u32，不要按 u64 读**（u64 读出的值是 nsec<<32|sec，错 13 年量级）
- PointCloud2：header + height u32 + width u32 + nfields u32 + fields[]（name string + offset u32 + datatype u8 + count u32）+ is_bigendian u8 + point_step u32 + row_step u32 + data_len u32 + data + is_dense u8
- Livox CustomMsg：header + timebase u64 + point_num u32 + lidar_id u8 + points[](每点 19B：offset_time u32 + xyz f32×3 + reflectivity/tag/line u8×3)
- Odometry：header + child_frame_id string + pose(7×f64) + cov(36×f64) + twist(6×f64) + cov(36×f64)
- CompressedImage：header + format string + data(jpeg)

## 输出格式

- **PCD binary**：header 行（VERSION/FIELDS/SIZE/TYPE/COUNT/WIDTH/HEIGHT/POINTS/DATA binary）+ 紧凑 float32。rgb 字段存 `uint32.view(np.float32)`（PCL 约定）
- **LAS 1.2 格式 3**：header 227B（scale=0.001、offset=min 点）、点记录 34B（xyz i32 + intensity u16 + flags + gps f64 + rgb u16×3）。见 convert_pcd2las.py
- 大文件策略：先写占位 header → 流式写数据 → 点数需预先统计（两遍扫描）

## 稠密地图 vs SLAM 地图（用户需求判断）

- **odometry 位姿拼接**（bag2map*）：数据量≈bag（1.7 亿点/6 分钟），但**里程计漂移→重影、糊、颜色错位**。只适合要"大小≈bag/完整密度"时
- **SLAM 地图合并**（merge_slam）：cloud_slam 帧合并 + voxel 去重 → **干净、彩色(99%+)、准确**（SLAM 优化坐标系）。用户最终要"干净彩色准确"时选这个
- 用户需求演变示例：先要"大小和 bag 差不多"→ dense；看到灰色团/重影后改口"要有色/干净彩色准确"→ slam。先问清质量 vs 密度优先级

## RGB-D 深度图导出（鱼眼多项式相机）

见 `references/odin-bag-format.md`（odin 标定参数 + SDK 算法复现）。要点：
- `Kcl = K4x4 @ Tcl`（内参×外参），点变换后 `u=round(x/z), v=round(y/z)`，`depth[v,u]=z`（相机系深度）
- 对齐方式：深度图针孔视角 + **彩图用逆畸变映射 remap**（`inv_map`：θ_d=θ+k2θ²+...+k7θ⁷，`map_coordinates` 双线性）
- 时间对齐：图像与点云 stamp 匹配（bisect 最近）；odin 的图/点云**同 header 一一对应**（dt~1e-10s，帧数相等）

## 陷阱清单（都踩过）

1. **`np.minimum.at` 做每像素最近深度必须用 inf 初始化**——0 初始会让 `min(0, z)=0` 吞掉全部有效值（最隐蔽的 bug）
2. **memmap 元素索引 vs `tf.tell()` 字节偏移**——差 4 倍，读错帧/越界静默返回空，表现为"只导出部分组"
3. **rgb float32 打包**：`.view(np.uint32)` 取位模式，`.astype` 是数值转换（全毁）
4. **scipy cKDTree `distance_upper_bound` 的哨兵索引 = n**（越界 1）：`idx = np.where(valid, idx, 0)` 再按 mask 置默认色
5. 非标准 bag 字段解析：fields 是 `u32 len + "key=value"` 连续存储（无分隔符），按 
 分割必错
6. **PowerShell 调用 plink 时 `&` 被误判为后台化** → 用 `cmd /c "`"路径`" ...args` 包装，多行远程命令用 `plink -m script.sh`
7. JPEG 尺寸从 SOF marker（0xFFC0-0xFFCF）解析
8. matplotlib 中文字体缺 Glyph 警告无害（标题乱码可忽略或用英文标题）

## 用户协作偏好

"指挥官模式"：远程命令由用户亲手执行，agent 给命令+预期输出+判读，每次 1-3 条；Windows 本机操作可直接代劳。给结论时用证据表（✅/❌+数据）。

## 参考

- `references/odin-bag-format.md` — odin 系统 bag 格式、话题表、SDK 标定参数（calib.yaml）、pcd2depth 官方算法细节
