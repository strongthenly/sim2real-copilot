---
name: ros-bag-pointcloud-conversion
description: 从 ROS1 bag 提取点云并转 PCD/LAS/彩色地图（Windows/Linux 通用，无需安装 ROS）。支持标准 bag（rosbags 库）与索引损坏/自研 SDK 非标准 bag（内置顺序解析 + 消息字节嗅探 + 手工反序列化）。触发词：bag、rosbag、点云、PCD、LAS、PLY、提取、转换、CustomMsg、cloud_raw、cloud_slam、稠密地图、彩色点云。
---

# ROS1 bag → 点云（PCD/LAS/彩色地图）

## 适用场景
- 手上有 .bag（可能来自自研 SDK、格式非标准），想提取点云成 PCD/LAS/PLY
- Windows 没有 ROS、不想装；或 bag 索引损坏，rosbags 读不到连接
- 用户典型诉求链：先转"最后一帧地图" → 再要"完整数据量" → 再要"彩色" → 最终要"质量好（无重影/颜色对）"

## 环境与工具链（Windows）
- winget 装 Python：`winget install -e --id Python.Python.3.12 --accept-source-agreements --accept-package-agreements --silent`
  - 装完后当前会话需手动加 PATH：`$env:Path = "C:\Users\<USER>\AppData\Local\Programs\Python\Python312\;...\Scripts\;$env:Path"`
  - 注意：WindowsApps 的 `python.exe` 是商店占位符（假 Python），必须用真 Python 路径
- pip 用清华镜像：`python -m pip install rosbags numpy scipy matplotlib -i https://pypi.tuna.tsinghua.edu.cn/simple`
- 本机工具脚本目录：`C:\Users\<USER>\Desktop\bag_tools\`（bag2pcd.py / batch_convert.py / convert_pcd2las.py / bag2map_color.py / merge_slam.py / batch_merge_slam.py / view_pcd.py / check_color.py）——优先读取复用，别重写

## 两条读取路径（自动选择）
1. **rosbags 标准路径**：`Reader(path)`，有 connections 直接用（`reader.messages(connections=[conn])` yield raw 字节，不要 deserialize，统一用手工解析更快）
2. **顺序解析 fallback**：rosbags 对非标准 bag 会**静默返回 connections=0（无报错）** → 用 `len(reader.connections)==0` 判断切 fallback

## 顺序解析器：ROS1 bag 格式的坑（核心知识）
- **文件头记录也有 data_len + data 部分，必须跳过**（漏掉则文件内所有记录错位，这是最常见的解析 bug）
- header 字段格式：循环 `(u32 len, "key=value")`，**字段间没有分隔符**（不是换行）
- 自研 SDK 的 bag 字段值可能是**裸字节**：string 无 u32 长度前缀、chunk 的 size 是 4B u32（标准 8B u64）→ size 按 `len(v)>=8 ? '<Q' : '<I'` 自适应
- 连接记录可能只有 `conn/op/topic` 三个字段（无 type/md5sum/message_definition）→ 类型靠消息字节嗅探
- chunk 内记录与文件级记录同构（u32 hlen + hdr + u32 dlen + data）；压缩有 none/lz4/bz2/zlib
- 核心函数（parse_fields / read_file_record / read_chunk_record）见 `references/sequential-bag-parser.md`，本机完整实现见 bag_tools/bag2pcd.py

## 消息字节嗅探类型（连接记录无 type 时）
- **PointCloud2**：header 后 height(u32) ≤10 且 width 合理 → 解析 fields 数组，**字段名必须含 x/y/z**（强校验，避免把 Imu/Pose/Image 误判成点云）
- **Livox CustomMsg**：header 后 timebase(u64) > 1e15（纳秒时间戳），point_num 与 points 数组长度（pos+13 处 u32）匹配
- 嗅探太宽松会把所有非点云话题误判为点云 → 必须用上面的强校验

## PointCloud2 手工解析（要点）
布局：header(seq u32 + stamp u64 + frame_id string) + height u32 + width u32 + nfields u32 + fields[](name string + offset u32 + datatype u8 + count u32) + is_bigendian u8 + point_step u32 + row_step u32 + data(u32 len + bytes) + is_dense u8
- datatype 映射：1=i1 2=u1 3=i2 4=u2 5=i4 6=u4 7=f4 8=f8
- **⚠️ rgb 字段 datatype=7(f32) 是 PCL 风格位模式打包 → 必须 `.view(np.uint32)`，`.astype(np.uint32)` 是数值转换会毁掉全部颜色**（真实踩坑：表现为"着色率 0%、全灰"）
- intensity/reflectivity 用 astype(f32)

## Livox CustomMsg 布局
header + timebase u64 + point_num u32 + lidar_id u8 + points[](每点 19B = offset_time u32 + xyz f32×3 + reflectivity u8 + tag u8 + line u8，无 padding)

## PCD 写出
- binary 格式；FIELDS `x y z [intensity] [rgb]`；rgb 用 `uint32.view(np.float32)` 写入（PCL 兼容）
- header 行固定：`# .PCD v0.7` / VERSION / FIELDS / SIZE / TYPE / COUNT / WIDTH / HEIGHT / VIEWPOINT / POINTS / DATA binary

## LAS 1.2 写出（格式 3 = 34B/点 含 rgb+gps）
- header 227B：签名 `LASF`、版本 1.2、点格式 3、点记录长 34、缩放 0.001、offset=各轴 min
- 点记录：xyz int32 + intensity u16 + bit_fields u8(return=1) + classification u8 + scan_angle i8 + user u8 + source u16 + gps f64 + rgb u16×3
- **验证时 header 偏移别记错**：点格式在 offset 104、点数在 107（offset 94 是 header size 字段——曾用错偏移误判格式错误）
- rgb 8bit→u16：`v << 8`

## 地图构建：dense 拼接 vs SLAM 地图（质量权衡，用户会在意）
- **cloud_raw（原始帧）+ odometry 位姿拼接** → 完整数据量（大小≈bag），但 **odometry 有漂移 → 重影、结构糊、颜色错位**
- **cloud_slam（SLAM 地图帧）全部 merge** → SLAM 优化坐标系，精确对齐、真彩色、无重影；但点稀疏（SLAM 降采样，几十~几百万点）
- 用户验收会看：文件大小、着色率、有没有重影/灰团 → 交付时给量化指标
- 用户偏好：要"大小和 bag 差不多"= 完整数据量；"要有色建图"= 彩色优先；质量优先时直接给 slam_map

## 彩色传递（KDTree）
- 参考色点：SLAM 全部帧 merge（含 rgb）；目标：dense 点
- **⚠️ scipy `cKDTree.query(..., distance_upper_bound=X)` 对超距点返回哨兵索引 = n（点数），直接索引会越界 → 必须 `valid = np.isfinite(d); colors = ref_rgb[np.where(valid, idx, 0)]; colors[~valid] = 灰色`**
- 阈值 0.5m 合理：dense 与 slam 坐标系有漂移偏差，太小则大量灰色，太大则颜色错位

## 用户协作偏好
- Windows 本机任务可 AI 直接代劳（装环境/写脚本/跑转换）；远端 fouruav 才走指挥官模式
- 大任务（GB 级 bag、多 bag 批量）用后台 + notify_on_complete，别前台干等
- 交付格式：每个产物给"大小/点数/着色率"指标表

## References
- `references/sequential-bag-parser.md` — 顺序解析核心函数 + 嗅探/解析要点代码
- `references/odin-custom-bag-format.md` — odin SDK（留形科技）非标准 bag 的格式细节与话题清单
