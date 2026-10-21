# odin 自定义 bag 字节布局（非标准，rosbags 读不到）

留形科技 odin 系统（`/odin1/*` 话题，callerid=host_sdk_sample）自研 bag writer 录制的格式。
rosbags 打开显示 connections=0（索引段在但 header 解析错位）；必须用顺序解析器。

## 与标准 ROS1 bag 的差异

| 项 | 标准 | odin 自定义 |
|---|---|---|
| 文件头 | magic + 记录 | 相同（#ROSBAG V2.0），index_pos 有效指向文件末尾 |
| header 字段值 | string 带 u32 长度前缀 | **裸字符串**（compression=b'none' 无前缀） |
| chunk size 字段 | u64 (8B) | **u32 (4B)** → 按 `len(v)>=8 ? '<Q' : '<I'` 读 |
| 连接记录 | conn/op/topic/type/md5sum/message_definition/callerid/latching | **只有 conn/op/topic** |
| 消息体 | 标准 ROS 序列化 | 标准（PointCloud2/CompressedImage/Odometry 均可手工解析） |

## 文件级记录

```
文件头记录: '#ROSBAG V2.0\n' + u32 hlen + header + u32 dlen + data   ← data 段必须跳过！
数据区: 循环 [u32 hlen + header + u32 dlen + data]
  op=0x05 chunk: header {compression, op, size}; data = 压缩记录流
  op=0x04 index data: 跳过（不依赖）
```

header 字段解析（关键）：字段 = 循环 `u32 len + "key=value"`，**字段间无分隔符**（不是 
 分隔）。

## chunk 内部

解压（none/lz4/bz2/zlib，lz4 用 `lz4.block.decompress(data, uncompressed_size=size)`）后同样是
`u32 hlen + header + u32 dlen + data` 记录流：
- op=0x07 连接记录：conn(u32) + topic（裸字符串）
- op=0x02 消息记录：conn(u32) + time + data=消息体

## 类型嗅探（连接记录无 type 时）

```python
def sniff_msgtype(raw):
    pos = parse_ros_header(raw)  # seq u32 + stamp 8B + frame_id string
    # PointCloud2: height≤10 且解析 fields 数组后字段名含 x/y/z
    h = u32@pos; w = u32@pos+4
    if h <= 10 and w < 1e8:
        nf = u32@pos+8; p = pos+12
        names = [解析 nf 个 (u32 len + name + offset u32 + datatype u8 + count u32)]
        if 3 <= nf <= 20 and {x,y,z} <= set(names): return 'sensor_msgs/PointCloud2'
    # CustomMsg: timebase u64 > 1e15 且 point_num 与 points 数组长度一致
    tb = u64@pos; pn = u32@pos+8
    if tb > 1e15 and 0 < pn <= 200000:
        arrlen = u32@pos+13  # lidar_id 后
        if abs(arrlen - pn) <= 1: return 'livox_ros_driver2/CustomMsg'
    return 'unknown'
```

实测话题类型：`/odin1/cloud_raw` 与 `/odin1/cloud_slam` 都是 sensor_msgs/PointCloud2
（cloud_raw 字段 x y z intensity confidence offset_time；cloud_slam 字段 x y z rgb）。
非点云话题（Imu/Pose/CompressedImage/Odometry/TF）会被强校验正确排除。

## 消息布局速查

- **PointCloud2**：header + height u32 + width u32 + nfields u32 + fields[] + is_bigendian u8 +
  point_step u32 + row_step u32 + data_len u32 + data + is_dense u8
- **CompressedImage**：header + format string + data(u8 数组，JPEG 字节；1600×1296)
- **nav_msgs/Odometry**：header + child_frame_id string + pose(7×f64) + cov(36×f64) + twist(6×f64) + cov(36×f64)
- **TFMessage**：u32 count + [header + child string + transform(7×f64)]×count（odin 的 /tf 只有 odom→map 单位变换）

## 时间戳（本会话最大的坑）

ROS1 Header 时间戳 = `sec u32 + nsec u32`（8 字节小端）。**误读成 u64** 会得到
"2017 年/2031 年"的假值且图像与点云相差 13.37 年。正确：

```python
stamp = struct.unpack_from('<I', raw, 4)[0] + struct.unpack_from('<I', raw, 8)[0] / 1e9
```

odin bag 实测：图像与点云同一 header（seq 相同、帧数相同）→ 按序配对或按时间戳最近匹配均可，dt < 0.1s。
