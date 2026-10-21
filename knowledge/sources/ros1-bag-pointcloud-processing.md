---
name: ros1-bag-pointcloud-processing
description: 从 ROS1 bag 提取/转换点云——PCD/LAS/深度图/稠密地图，兼容标准 bag 与自研 SDK 的非标准或索引损坏 bag（Windows 无 ROS 环境，rosbags + 手工顺序解析）。触发词：bag、rosbag、点云、PCD、提取点云、点云地图、稠密地图、深度图、LAS、.olx、cloud_slam、cloud_raw。
---

# ROS1 Bag 点云提取与转换（Windows 无 ROS）

在 Windows（无 ROS）上从 ROS1 bag 提取点云并转 PCD / LAS / 深度图 / 稠密地图。核心难点：**很多自研 SDK（如留形科技 odin）录的 bag 是非标准格式，rosbags 等标准工具读不到任何连接**，需要手工顺序解析。

## 工具链（Windows，一次性搭建）

```powershell
winget install -e --id Python.Python.3.12 --accept-source-agreements --accept-package-agreements --silent
$env:Path = "C:\Users\<USER>\AppData\Local\Programs\Python\Python312\;C:\Users\<USER>\AppData\Local\Programs\Python\Python312\Scripts\;$env:Path"
python -m pip install rosbags numpy matplotlib scipy pillow -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 第一步：判断 bag 类型（决定走哪条路）

```python
from rosbags.rosbag1 import Reader
r = Reader(path)
if r.connections:   # 非空 = 标准 bag
    # 走 rosbags：conn.topic / conn.msgtype / conn.msgcount
else:
    # 非标准/索引损坏 → 顺序解析（见下）
```

**rosbags 0.11 API 坑**（实测）：
- `Reader(path)` 构造**不接受** typestore 参数；`Writer(path)` 也不接受（typestore 传给 `add_connection`）
- `Writer.write(conn, ts, data)` 要的是**序列化后的字节**，不是消息对象：`typestore.serialize_ros1(msg, typename)`
- `Reader` **没有** `deserialize` 方法——反序列化用 `typestore.deserialize_ros1(raw, typename)`
- 打开失败的 bag 上 `close()` 会抛 `AssertionError`（`assert self.bio`）——用 try/except 包住或直接不关
- 标准 bag 中反序列化后提取点云，直接手工解析 raw 字节更快更稳（无需 typestore）

## 非标准 bag：顺序解析器（核心方法）

ROS1 bag v2.0 文件结构（顺序遍历，不依赖末尾索引段）：

```
magic "#ROSBAG V2.0\n" (13B)
文件头记录: u32 hlen + header字段 + u32 dlen + data
  ⚠️ 文件头记录也有 dlen+data 部分，不跳过后面全部错位！（本会话踩过的坑）
数据区: 循环读文件级记录
  op=0x05 chunk: header 含 compression/size；data 是 chunk 内容
  op=0x04 index data: 跳过（末尾索引段，损坏 bag 里不可靠）
```

**header 字段格式**：循环 `(u32 len + "key=value" 字节)`，字段间**无分隔符**。value 是裸字节：
- 标准实现：string 值带 u32 长度前缀（如 compression = 4+4=8B）、u64 8B、u32 4B
- **odin SDK 等自定义实现**：裸字符串无长度前缀、chunk size 是 **4 字节 u32**（不是标准 u64）→ `size = unpack('<Q' if len(v)>=8 else '<I', v)`
- 连接记录可能只有 `conn/op/topic` 三个字段（**没有 type/md5sum/message_definition**）→ 消息类型靠字节嗅探

chunk 内部：同样是 `u32 hlen + header + u32 dlen + data` 的记录序列：
- `op=0x07` 连接记录：header 含 conn(u32)、topic
- `op=0x02` 消息记录：header 含 conn(u32)、time(u64)；data = 消息字节
- 压缩：none/lz4/bz2/zlib（`lz4.block.decompress(data, uncompressed_size=size)`）

完整可复用实现见 `scripts/fallback_bag_reader.py`。

## 消息类型嗅探（无 type 字段时）

从消息字节强校验（宽松条件会把 Imu/Pose/Image 误判成点云——本会话踩过）：
- **PointCloud2**：header 后 `height≤10`，且 fields 数组字段名解析后**包含 x/y/z**（3≤nfields≤20）
- **Livox CustomMsg**：`timebase(u64)>1e15` 且 `point_num` 与 lidar_id 后的数组长度 `abs(arrlen-pn)≤1`

## 手工解析 PointCloud2（比反序列化快且兼容非标准 bag）

布局：header(seq u32 + stamp u64 + frame_id string) + height u32 + width u32 + nfields u32 + fields[]（name string + offset u32 + datatype u8 + count u32）+ is_bigendian u8 + point_step u32 + row_step u32 + data_len u32 + data + is_dense u8。

- 字段类型：`DT = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}`，用 np.dtype + offsets + itemsize=point_step 一次性解析
- **⚠️ rgb 字段坑**：PCL 风格（PointXYZRGB）rgb 是 **float32 打包**（datatype=7），颜色在 float32 的**位模式**里。提取必须 `arr['rgb'].view(np.uint32)`；用 `astype(np.uint32)` 是数值转换，会把所有颜色毁掉（本会话修过这个 bug，症状：全灰/着色率 0%）
- Livox CustomMsg：header + timebase u64 + point_num u32 + lidar_id u8 + points[]（每点 19B：offset_time u32 + xyz f32×3 + reflectivity/tag/line u8×3，无 padding）

## 输出格式

- **PCD binary**：FIELDS x y z [intensity] [rgb]；rgb 写入时 `rgb.view(np.float32)`（位模式直写）
- **LAS 1.2 格式 3**（34B/点，含 intensity + gps_time + rgb）：header 227B（签名 LASF、version 1.2、offset 227、scale 0.001、格式 3、记录长 34）；RGB 8bit→16bit `<<8`；验证时注意字段偏移：格式在 offset 104、点数在 107（不是 94，94 是 header size）
- **球面深度图**（range image，无需相机内参）：`az = degrees(atan2(x,y))%360` 栅格化宽、`el = degrees(atan2(z, hypot(x,y)))` 栅格化高，每像素取最近点深度 mm → 16bit PNG（Pillow `I;16`）。单帧雷达点稀疏（有效像素 1% 级）是正常的，多帧融合才稠密

## 稠密地图拼接（两条路线，质量差异大）

| 路线 | 做法 | 结果 |
|---|---|---|
| odometry 拼接 | 解析 nav_msgs/Odometry（header+child+pose 7×f64）位姿，逐帧变换 cloud_raw 拼接 | 大小≈bag，但**里程计漂移 → 重影、糊、颜色对不上** |
| SLAM 帧 merge | 把 SLAM 全局地图话题（如 cloud_slam，frame_id=odom、含 rgb）全部帧合并 + 体素去重 | **干净/彩色/准确**（SLAM 优化坐标），但点量小（帧本身降采样） |

- 用户需求演进模式：先要"大小和 bag 差不多"（完整数据），后要"干净彩色准确"（质量）——**最终答案通常是 SLAM 帧 merge + voxel 去重（0.02~0.03m）**，别一开始就上 odometry 拼接
- 位姿变换：四元数→旋转矩阵，`p' = R·p + t`
- 帧对齐：cloud_raw 与 odometry 帧数通常相等（同一回调发布），按序配对即可；不相等时按 stamp 二分
- **颜色传递（KDTree）坑**：`scipy.spatial.cKDTree.query(pts, k=1, distance_upper_bound=d)` 对超出距离的点返回**哨兵索引 = n**（越界！），必须 `valid = np.isfinite(dist); colors = ref_rgb[np.where(valid, idx, 0)]; colors[~valid] = 灰色`
- 大文件流式写：先写 header 占位 → 逐帧变换写数据 → 完成后 seek 回头部更新 POINTS/WIDTH（或第一遍先统计总点数）

## 常用脚本位置

完整工具在 `C:\Users\<USER>\Desktop\bag_tools\`：bag2pcd.py（检测+转换）、bag2map.py（odometry 拼接）、bag2map_color.py（+KDTree 着色）、merge_slam.py（SLAM 帧 merge）、convert_pcd2las.py、range_image.py、batch_*.py。

## 相关技能

- `uav-ros-diagnostics` / `livox-lidar-diagnostics`：飞控/雷达连接诊断（本技能专注 bag 数据处理，不重叠）
- odin bag 具体字节细节见 `references/odin-bag-format.md`
