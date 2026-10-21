# ROS1 bag 格式与顺序解析器（可重建核心）

本文档包含：ROS1 bag v2.0 二进制格式细节、非标准（自研 SDK）bag 的特征与差异、可直接复用的核心解析代码（顺序解析 + 消息嗅探 + PointCloud2/CustomMsg 手工解析 + PCD 输出）。

## 1. ROS1 bag v2.0 格式

```
文件 = 文件头记录 + (chunk 记录 | index data 记录)* + 末尾索引段
```

**记录通用格式**（文件级与 chunk 内相同）：
```
u32 header_len
header 字段列表（无分隔符，循环读）
u32 data_len
data 字节
```

**header 字段**：循环 `u32 flen + "key=value"`（flen 含 key=value 全部字节）。**标准格式**的 value 是 ROS 序列化（string 带 u32 长度前缀、u64 8B 小端、u32 4B）；**odin 自研格式**的 value 是裸字节（string 无前缀、size 为 4B u32）。

**文件头记录**：`#ROSBAG V2.0
` + 记录。字段：`op=0x03`、`index_pos`(u64)、`chunk_count`(u32)、`conn_count`(u32)。**⚠️ 文件头记录也有 data_len+data 部分，读完 header 必须跳过**，否则后续全部错位。

**chunk 记录**（op=0x05）：字段 `compression`(none/lz4/bz2/zlib)、`size`(解压后大小，4B 或 8B)。data = 压缩后的记录序列。chunk 内记录：
- 连接记录 op=0x07：字段 `conn`(u32)、`topic`、`type`、`md5sum`、`message_definition`、`callerid`、`latching`（**odin 格式只有 conn/op/topic**）
- 消息记录 op=0x02：字段 `conn`(u32)、`time`(u64)，data = 消息体字节

**index data 记录**（op=0x04）：字段 `ver`、`conn`、`count`。文件末尾的索引段含连接定义——损坏/缺失时 rosbags 读不到连接，但 chunk 数据不受影响。

## 2. 顺序解析器核心代码

```python
import struct

def parse_fields(buf):
    """header 字段: 循环 (u32 len, 'key=value')，无分隔符"""
    fields = {}; pos = 0; n = len(buf)
    while pos < n:
        if pos + 4 > n: break
        flen = struct.unpack_from('<I', buf, pos)[0]; pos += 4
        item = buf[pos:pos + flen]; pos += flen
        if b'=' in item:
            k, v = item.split(b'=', 1); fields[k] = v
    return fields

def read_file_record(f):
    head = f.read(4)
    if len(head) < 4: return None
    hlen = struct.unpack('<I', head)[0]
    hdr = f.read(hlen)
    dl = f.read(4)
    if len(dl) < 4: return None
    dlen = struct.unpack('<I', dl)[0]
    data = f.read(dlen)
    if len(data) < dlen: return None
    return parse_fields(hdr), data

def read_chunk_record(buf, pos):
    if pos + 4 > len(buf): return None, None, pos
    hlen = struct.unpack_from('<I', buf, pos)[0]; pos += 4
    hdr = buf[pos:pos + hlen]; pos += hlen
    if pos + 4 > len(buf): return None, None, pos
    dlen = struct.unpack_from('<I', buf, pos)[0]; pos += 4
    data = buf[pos:pos + dlen]; pos += dlen
    return parse_fields(hdr), data, pos

def decompress(comp, data, size):
    if comp == b'none': return data
    if comp == b'lz4':
        import lz4.block
        return lz4.block.decompress(data, uncompressed_size=size)
    if comp == b'bz2':
        import bz2; return bz2.decompress(data)
    if comp == b'zlib':
        import zlib; return zlib.decompress(data)
    raise ValueError(f'未知压缩: {comp}')

def open_bag_file(path):
    f = open(path, 'rb')
    magic = f.read(13)
    assert magic == b'#ROSBAG V2.0\n', '不是 ROS1 bag V2.0'
    hl = struct.unpack('<I', f.read(4))[0]
    f.read(hl)                                  # header
    dl_b = f.read(4)                            # ⚠️ 必须跳过文件头记录的 data 部分
    if len(dl_b) == 4:
        f.read(struct.unpack('<I', dl_b)[0])
    return f
```

**扫描话题**（返回 {topic: {count, raw0}}）：遍历文件级记录，op==b'\x05' 时解压 chunk，chunk 内收集连接（op=0x07）与消息（op=0x02，按 conn 归属 topic）。消息流式遍历同理，yield 目标 topic 的每条 data。注意 chunk 数据是顺序读，5.2GB 约 4600 chunks 全扫需 1-3 分钟。

## 3. 消息类型嗅探（强校验版）

连接记录可能没有 type 字段，必须从消息字节判定。**弱条件会把 Imu/Pose/Image 误判成点云**：

```python
def parse_ros_header(raw):
    pos = 0
    pos += 4                    # seq
    pos += 8                    # stamp
    flen = struct.unpack_from('<I', raw, pos)[0]
    pos += 4 + flen             # frame_id
    return pos

def sniff_msgtype(raw):
    try:
        pos = parse_ros_header(raw)
        if pos + 20 > len(raw): return 'unknown'
        # PointCloud2: 解析 fields 数组，字段名必须含 x/y/z
        h = struct.unpack_from('<I', raw, pos)[0]
        w = struct.unpack_from('<I', raw, pos + 4)[0]
        if h <= 10 and w < 100_000_000:
            nfields = struct.unpack_from('<I', raw, pos + 8)[0]
            p = pos + 12; names = []; ok = True
            for _ in range(nfields):
                if p + 4 > len(raw): ok = False; break
                flen = struct.unpack_from('<I', raw, p)[0]; p += 4
                if p + flen + 9 > len(raw): ok = False; break
                names.append(raw[p:p + flen].decode(errors='replace'))
                p += flen + 9   # offset u32 + datatype u8 + count u32
            if ok and 3 <= nfields <= 20 and {'x', 'y', 'z'} <= set(names):
                return 'sensor_msgs/PointCloud2'
        # CustomMsg: timebase 巨大 + 点数与数组长度匹配
        tb = struct.unpack_from('<Q', raw, pos)[0]
        pn = struct.unpack_from('<I', raw, pos + 8)[0]
        if tb > 1e15 and 0 < pn <= 200000 and pos + 17 <= len(raw):
            arrlen = struct.unpack_from('<I', raw, pos + 13)[0]  # lidar_id 后是数组长度
            if abs(arrlen - pn) <= 1:
                return 'livox_ros_driver2/CustomMsg'
    except Exception:
        pass
    return 'unknown'
```

## 4. 点云消息手工解析

**PointCloud2** 布局：`header + height u32 + width u32 + nfields u32 + fields[](name string + offset u32 + datatype u8 + count u32) + is_bigendian u8 + point_step u32 + row_step u32 + data_len u32 + data`。datatype 枚举：1=i1, 2=u1, 3=i2, 4=u2, 5=i4, 6=u4, 7=f4, 8=f8。用结构化 dtype 提取：names/formats('<f4' 等)/offsets/itemsize=point_step。xyz 必取；intensity/reflectivity 取第一个存在的；**rgb 字段实测是 datatype=7 (float32)**（PCL 风格，RGB 打包在位模式里）→ 提取用 `arr['rgb'].view(np.uint32)`，**绝不能用 astype**（数值转换毁颜色）；仅当 datatype=6 (u32) 才可 astype。PCD 输出写回 `rgb.view(np.float32)`。

**Livox CustomMsg** 布局：`header + timebase u64 + point_num u32 + lidar_id u8 + points[](u32 count + 每点 19B)`。19B = offset_time u32 + x f32 + y f32 + z f32 + reflectivity u8 + tag u8 + line u8（ROS 序列化无对齐）。dtype: `[('offset_time','<u4'),('x','<f4'),('y','<f4'),('z','<f4'),('reflectivity','u1'),('tag','u1'),('line','u1')]`。

## 5. PCD 输出（binary）

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z [intensity] [rgb]
SIZE 4 4 4 [4] [4]
TYPE F F F [F] [F]
COUNT 1 1 1 [1] [1]
WIDTH <n>
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS <n>
DATA binary
```
binary 数据 = 每点字段 f32 紧凑排列（np.column_stack + tobytes），无 padding。rgb 字段存 float32（PCL/CloudCompare 按位还原颜色）。

## 6. rosbags 库 API（标准 bag 路径）

- `Reader(path)` / `Writer(path)` **不收 typestore 参数**
- `Writer.add_connection(topic, msgtype, typestore=ts)`
- `Writer.write(conn, timestamp_ns, bytes)` —— 需先 `ts.serialize_ros1(msg, typename)` 序列化
- 反序列化：`ts.deserialize_ros1(raw, typename)`（Reader 本身无 deserialize 方法）
- `get_typestore(Stores.ROS1_NOETIC)` + `ts.register(get_types_from_msg(msg_text, typename))` 注册自定义类型（如 Livox）
- 类型字符串为 ROS2 风格：`sensor_msgs/msg/PointCloud2`
- 构造测试 bag：Writer + serialize_ros1，见上

## 7. 实战数据（odin_data_20260727 案例）

- 5.6GB / 4627 chunks / 10 连接；6 分钟数据：`/odin1/cloud_raw` 3549 帧（每帧 49152 点，frame_id=odin1_base_link，字段 x y z intensity confidence offset_time）、`/odin1/cloud_slam` 3548 帧（frame_id=odom，字段 x y z rgb，**非累积式**：每帧 1.3 万点左右独立波动）、`/odin1/odometry` 342 条、`/odin1/imu` 13.8 万条
- 同名目录 7 个 bag 批量（22.5GB）：`cloud_slam` 333~5306 帧，末帧 0.9~1.7 万点（稀疏）；稠密拼接实测：0.54GB bag → 263MB PCD（1637 万点），着色率 78.8%（KDTree 0.5m 阈值）
- 稠密彩色地图已实现：`bag2map_color.py`（odometry 位姿按帧序对齐 + cloud_slam 全帧颜色参考 + 两遍扫描流式写），详见 SKILL.md"稠密彩色地图"节

## 7b. odometry 位姿拼接要点（bag2map_color.py）

- nav_msgs/Odometry 解析：header(seq u32 + stamp u64 + frame_id string) + child_frame_id string + pose 7×f64(位置3+四元数4) + cov 36×f64 + twist 6×f64 + cov 36×f64，实测 719B
- 位姿约定：frame_id=odom / child=odin1_base_link → 变换 `p_odom = R(quat)·p_base + t`，首条 ≈ (0,0,0) 单位四元数
- 对齐：cloud_raw 第 i 帧 ↔ odometry 第 i 条（帧数一致，按序即可）
- 颜色传递：slam 全帧 merge 建 cKDTree，`query(distance_upper_bound=0.5)` → 哨兵 idx=n 越界 1，用 `valid=np.isfinite(d)` 掩码；未覆盖给灰 0x808080

## 8. PCD → LAS（实测要点）

- 手写 LAS 1.2 格式 3（34B/点：xyz i32 + intensity u16 + bit_fields/class/scan_angle/user 4B + source u16 + gps f64 + r/g/b u16×3），header 227B，见 scripts/convert_pcd2las.py
- LAS header 字段偏移（验证时别猜）：签名 LASF @0、版本 @24、header size @94、**点格式 @104、记录长 @106、legacy 点数 @107**、scale @131、offset @155、max/min @179。本会话验证脚本误用 offset 94 读点格式，把 227(header size) 当格式读，误报"格式=227"——是验证脚本的偏移错，不是输出错
- LAS 头合法性核对：签名 LASF + 版本 1.2 + 格式 3 + 记录长 34

## 9. .olx/.bin（留形科技 odin 私有点云格式）已知信息

- odin 系统 = 留形科技自研（bag callerid=/host_sdk_sample 佐证）；原始点云数据 = `.olx` + `.bin` 配套格式，**无公开文档**（网搜只出二手平台 OLX）
- bag 补救路线：① odin SDK 里找导出函数/格式头文件（搜 olx/bin/export/save）② 残留样本二进制逆向 ③ 找厂商要格式文档
- 未定项：.olx 是否工程/索引文件、.bin 每点字节数/字段布局——拿到样本后按本文件第 3-4 节的嗅探/解析思路逆向
