# ROS1 bag 顺序解析核心函数（不依赖末尾索引，兼容非标准 header）

完整实现见本机 `C:\Users\<USER>\Desktop\bag_tools\bag2pcd.py`。以下为可复用的核心。

## header 字段解析（关键：无分隔符，循环 u32 len + "key=value"）
```python
def parse_fields(buf):
    fields = {}
    pos = 0
    n = len(buf)
    while pos < n:
        if pos + 4 > n:
            break
        flen = struct.unpack_from('<I', buf, pos)[0]
        pos += 4
        item = buf[pos:pos + flen]
        pos += flen
        if b'=' in item:
            k, v = item.split(b'=', 1)
            fields[k] = v
    return fields
```

## 文件级记录（含打开文件：magic + 文件头 + 跳过 data）
```python
def _open_bag_file(path):
    f = open(path, 'rb')
    assert f.read(13) == b'#ROSBAG V2.0\n', '不是 ROS1 bag V2.0'
    hl = struct.unpack('<I', f.read(4))[0]
    f.read(hl)                       # 文件头 header
    dl_b = f.read(4)
    if len(dl_b) == 4:               # ★ 文件头记录也有 data 部分，必须跳过
        f.read(struct.unpack('<I', dl_b)[0])
    return f

def read_file_record(f):
    head = f.read(4)
    if len(head) < 4:
        return None
    hlen = struct.unpack('<I', head)[0]
    hdr = f.read(hlen)
    dl = f.read(4)
    if len(dl) < 4:
        return None
    dlen = struct.unpack('<I', dl)[0]
    data = f.read(dlen)
    if len(data) < dlen:
        return None
    return parse_fields(hdr), data
```

## chunk 内记录 + 解压
```python
def read_chunk_record(buf, pos):
    if pos + 4 > len(buf):
        return None, None, pos
    hlen = struct.unpack_from('<I', buf, pos)[0]; pos += 4
    hdr = buf[pos:pos + hlen]; pos += hlen
    if pos + 4 > len(buf):
        return None, None, pos
    dlen = struct.unpack_from('<I', buf, pos)[0]; pos += 4
    data = buf[pos:pos + dlen]; pos += dlen
    return parse_fields(hdr), data, pos

def decompress(comp, data, size):
    if comp == b'none':
        return data
    if comp == b'lz4':
        import lz4.block
        return lz4.block.decompress(data, uncompressed_size=size)
    if comp == b'bz2':
        import bz2; return bz2.decompress(data)
    if comp == b'zlib':
        import zlib; return zlib.decompress(data)
    raise ValueError(f'未知压缩: {comp}')
```

## 主循环（流式扫描，两遍：scan 统计/收参考 + build 写入）
- op=0x05 chunk：解压后遍历内部记录（op=0x07 连接 / op=0x02 消息）
- op=0x04 index data：跳过
- 自定义 bag 的 chunk size 字段可能 4B：`struct.unpack('<Q' if len(sv) >= 8 else '<I', sv)[0]`
- 连接记录字段：`conn`(u32) + `topic`（自定义 bag 可能无 type）

## 类型嗅探（强校验，避免误判）
```python
def sniff_msgtype(raw):
    pos = 0; pos += 4 + 8
    flen = struct.unpack_from('<I', raw, pos)[0]; pos += 4 + flen
    h = struct.unpack_from('<I', raw, pos)[0]        # PointCloud2: height
    w = struct.unpack_from('<I', raw, pos + 4)[0]    # width
    if h <= 10 and w < 100_000_000:
        nf = struct.unpack_from('<I', raw, pos + 8)[0]
        p = pos + 12
        names = []
        ok = True
        for _ in range(nf):
            flen = struct.unpack_from('<I', raw, p)[0]; p += 4
            if p + flen + 9 > len(raw): ok = False; break
            names.append(raw[p:p + flen].decode(errors='replace'))
            p += flen + 9
        if ok and 3 <= nf <= 20 and {'x', 'y', 'z'} <= set(names):   # ★ 必须含 x/y/z
            return 'sensor_msgs/PointCloud2'
    tb = struct.unpack_from('<Q', raw, pos)[0]       # CustomMsg: timebase
    pn = struct.unpack_from('<I', raw, pos + 8)[0]
    if tb > 1e15 and 0 < pn <= 200000 and pos + 17 <= len(raw):
        arrlen = struct.unpack_from('<I', raw, pos + 13)[0]
        if abs(arrlen - pn) <= 1:
            return 'livox_ros_driver2/CustomMsg'
    return 'unknown'
```

## PointCloud2 解析要点
- 字段循环后：`pos += 1`(is_bigendian) + point_step(u32) + row_step(u32) + data_len(u32) + data
- 点数 = `len(data) // point_step`
- **rgb（datatype=7 f32，PCL 位模式打包）必须 `arr['rgb'].view(np.uint32)`；astype 会毁颜色**
- intensity 字段名可能为 intensity/reflectivity/intensities，取第一个存在的

## Odometry 位姿解析（nav_msgs/Odometry，拼接用）
`header + child_frame_id(string) + pose 7×f64(位置3+四元数4) + ...`
frame_id=odom / child=base_link 时，位姿即"雷达→odom"变换：`p_odom = R(quat)@p_base + t`

## LAS 1.2 验证偏移（别记错）
- header size 字段：offset 94（u16=227）
- 点格式：offset 104（u8）
- 点记录长：offset 105（u16）
- 点数：offset 107（u32）
