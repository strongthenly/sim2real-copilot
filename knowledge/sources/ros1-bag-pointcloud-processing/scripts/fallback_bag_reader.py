#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 bag 顺序解析器（兼容标准 bag 与自研 SDK 的非标准/索引损坏 bag，如 odin SDK）。
不依赖 rosbags、不依赖末尾索引段，直接从文件头顺序遍历 chunk。

用法:
    from fallback_bag_reader import scan_topics, iter_msgs, sniff_msgtype, parse_pc2_raw

    topics = scan_topics('xxx.bag')           # {topic: {'count': n, 'raw0': 首条消息}}
    for topic, info in topics.items():
        info['msgtype'] = sniff_msgtype(info['raw0']) if info['raw0'] else 'unknown'
    for raw in iter_msgs('xxx.bag', '/odin1/cloud_slam'):
        xyz, intens, rgb = parse_pc2_raw(raw)
"""
import struct

import numpy as np


def parse_fields(buf):
    """header 字段: 循环 (u32 len, 'key=value')，字段间无分隔符。value 是裸字节。"""
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


def read_chunk_record(buf, pos):
    if pos + 4 > len(buf):
        return None, None, pos
    hlen = struct.unpack_from('<I', buf, pos)[0]
    pos += 4
    hdr = buf[pos:pos + hlen]
    pos += hlen
    if pos + 4 > len(buf):
        return None, None, pos
    dlen = struct.unpack_from('<I', buf, pos)[0]
    pos += 4
    data = buf[pos:pos + dlen]
    pos += dlen
    return parse_fields(hdr), data, pos


def decompress(comp, data, size):
    if comp == b'none':
        return data
    if comp == b'lz4':
        import lz4.block
        return lz4.block.decompress(data, uncompressed_size=size)
    if comp == b'bz2':
        import bz2
        return bz2.decompress(data)
    if comp == b'zlib':
        import zlib
        return zlib.decompress(data)
    raise ValueError(f'未知压缩: {comp}')


def _open_bag(path):
    f = open(path, 'rb')
    if f.read(13) != b'#ROSBAG V2.0\n':
        f.close()
        raise ValueError('不是 ROS1 bag V2.0')
    hl = struct.unpack('<I', f.read(4))[0]
    f.read(hl)
    dl_b = f.read(4)
    if len(dl_b) == 4:
        f.read(struct.unpack('<I', dl_b)[0])  # ⚠️ 文件头记录的 data 必须跳过，否则全部错位
    return f


def scan_topics(path, max_chunks=None, keep_last=False, progress=False):
    """顺序扫描: {topic: {'count': n, 'raw0': 首条消息, 'last_raw': 末条(keep_last=True)}}"""
    topics = {}
    with _open_bag(path) as f:
        conns = {}
        chunk_idx = 0
        while True:
            r = read_file_record(f)
            if r is None:
                break
            fields, data = r
            if fields.get(b'op') != b'\x05':
                continue  # 跳过 index data 等
            chunk_idx += 1
            comp = fields.get(b'compression', b'none')
            sv = fields.get(b'size', b'\x00' * 4)
            # 兼容标准 u64 与自定义 u32 的 size
            size = struct.unpack('<Q' if len(sv) >= 8 else '<I', sv)[0]
            buf = decompress(comp, data, size)
            p = 0
            while True:
                cfields, cdata, p = read_chunk_record(buf, p)
                if cfields is None:
                    break
                cop = cfields.get(b'op')
                if cop == b'\x07':
                    cid = struct.unpack('<I', cfields[b'conn'])[0]
                    conns[cid] = cfields.get(b'topic', b'?').decode(errors='replace')
                    topics.setdefault(conns[cid], {'count': 0, 'raw0': None, 'last_raw': None})
                elif cop == b'\x02':
                    cid = struct.unpack('<I', cfields[b'conn'])[0]
                    t = conns.get(cid)
                    if t:
                        info = topics[t]
                        info['count'] += 1
                        if info['raw0'] is None:
                            info['raw0'] = cdata
                        if keep_last:
                            info['last_raw'] = cdata
            if max_chunks and chunk_idx >= max_chunks:
                break
            if progress and chunk_idx % 500 == 0:
                print(f'  已扫 {chunk_idx} chunk...', flush=True)
    return topics


def iter_msgs(path, topic, progress=False):
    """yield 指定 topic 的每条消息原始字节（流式，全量扫描）"""
    with _open_bag(path) as f:
        conns = {}
        chunk_idx = 0
        n = 0
        while True:
            r = read_file_record(f)
            if r is None:
                break
            fields, data = r
            if fields.get(b'op') != b'\x05':
                continue
            chunk_idx += 1
            comp = fields.get(b'compression', b'none')
            sv = fields.get(b'size', b'\x00' * 4)
            size = struct.unpack('<Q' if len(sv) >= 8 else '<I', sv)[0]
            buf = decompress(comp, data, size)
            p = 0
            while True:
                cfields, cdata, p = read_chunk_record(buf, p)
                if cfields is None:
                    break
                cop = cfields.get(b'op')
                if cop == b'\x07':
                    cid = struct.unpack('<I', cfields[b'conn'])[0]
                    conns[cid] = cfields.get(b'topic', b'?').decode(errors='replace')
                elif cop == b'\x02':
                    cid = struct.unpack('<I', cfields[b'conn'])[0]
                    if conns.get(cid) == topic:
                        n += 1
                        yield cdata
            if progress and chunk_idx % 500 == 0:
                print(f'  已扫 {chunk_idx} chunk ({n} 条)...', flush=True)


def parse_ros_header(raw):
    pos = 0
    pos += 4                    # seq
    pos += 8                    # stamp
    flen = struct.unpack_from('<I', raw, pos)[0]
    pos += 4 + flen             # frame_id
    return pos


def sniff_msgtype(raw):
    """从消息字节强校验判定点云类型。宽松条件会把 Imu/Pose/Image 误判成点云！"""
    try:
        pos = parse_ros_header(raw)
        if pos + 20 > len(raw):
            return 'unknown'
        # PointCloud2: fields 数组字段名必须含 x/y/z
        h = struct.unpack_from('<I', raw, pos)[0]
        w = struct.unpack_from('<I', raw, pos + 4)[0]
        if h <= 10 and w < 100_000_000:
            nfields = struct.unpack_from('<I', raw, pos + 8)[0]
            p = pos + 12
            names = []
            ok = True
            for _ in range(nfields):
                if p + 4 > len(raw):
                    ok = False
                    break
                flen = struct.unpack_from('<I', raw, p)[0]
                p += 4
                if p + flen + 9 > len(raw):
                    ok = False
                    break
                name = raw[p:p + flen].decode(errors='replace')
                p += flen + 9  # offset u32 + datatype u8 + count u32
                names.append(name)
            if ok and 3 <= nfields <= 20 and {'x', 'y', 'z'} <= set(names):
                return 'sensor_msgs/PointCloud2'
        # Livox CustomMsg: timebase 巨大 + 点数与数组长度匹配
        tb = struct.unpack_from('<Q', raw, pos)[0]
        pn = struct.unpack_from('<I', raw, pos + 8)[0]
        if tb > 1e15 and 0 < pn <= 200000 and pos + 17 <= len(raw):
            arrlen = struct.unpack_from('<I', raw, pos + 13)[0]
            if abs(arrlen - pn) <= 1:
                return 'livox_ros_driver2/CustomMsg'
    except Exception:
        pass
    return 'unknown'


DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def parse_pc2_raw(raw):
    """手工解析 sensor_msgs/PointCloud2 -> (xyz Nx3 f32, intensity 或 None, rgb 或 None)"""
    pos = parse_ros_header(raw)
    pos += 8                    # height + width
    nfields = struct.unpack_from('<I', raw, pos)[0]
    pos += 4
    fields = []
    for _ in range(nfields):
        flen = struct.unpack_from('<I', raw, pos)[0]
        pos += 4
        name = raw[pos:pos + flen].decode(errors='replace')
        pos += flen
        offset = struct.unpack_from('<I', raw, pos)[0]
        datatype = raw[pos + 4]
        count = struct.unpack_from('<I', raw, pos + 5)[0]
        pos += 9
        fields.append((name, offset, datatype, count))
    pos += 1                    # is_bigendian
    point_step = struct.unpack_from('<I', raw, pos)[0]
    pos += 8                    # + row_step
    data_len = struct.unpack_from('<I', raw, pos)[0]
    pos += 4
    data = raw[pos:pos + data_len]
    np_fields = [(name, '<' + DT[dt], off) for name, off, dt, cnt in fields if dt in DT]
    dtype = np.dtype({'names': [f[0] for f in np_fields],
                      'formats': [f[1] for f in np_fields],
                      'offsets': [f[2] for f in np_fields],
                      'itemsize': point_step})
    n = len(data) // point_step
    arr = np.frombuffer(data, dtype=dtype, count=n)
    xyz = np.column_stack([arr['x'].astype(np.float32),
                           arr['y'].astype(np.float32),
                           arr['z'].astype(np.float32)])
    intens = None
    for name in ('intensity', 'reflectivity', 'intensities'):
        if name in arr.dtype.names:
            intens = arr[name].astype(np.float32)
            break
    rgb = None
    if 'rgb' in arr.dtype.names:
        r = arr['rgb']
        if r.dtype.kind == 'f':
            # ⚠️ PCL 风格: RGB 打包在 float32 位模式里，必须 view 取位模式（astype 会毁掉颜色）
            rgb = r.view(np.uint32)
        else:
            rgb = r.astype(np.uint32)
    return xyz, intens, rgb


def parse_livox_raw(raw):
    """Livox CustomMsg: header + timebase u64 + point_num u32 + lidar_id u8 + points[](每点19B 无padding)"""
    pos = parse_ros_header(raw)
    pos += 8                    # timebase
    point_num = struct.unpack_from('<I', raw, pos)[0]
    pos += 4
    pos += 1                    # lidar_id
    arr_len = struct.unpack_from('<I', raw, pos)[0]
    pos += 4
    n = min(point_num, arr_len)
    if n == 0:
        return np.zeros((0, 3), np.float32), None
    pts = raw[pos:pos + n * 19]
    dtype = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1')])
    arr = np.frombuffer(pts, dtype=dtype, count=n)
    xyz = np.column_stack([arr['x'], arr['y'], arr['z']])
    return xyz, arr['reflectivity'].astype(np.float32)


if __name__ == '__main__':
    import sys
    path = sys.argv[1]
    topics = scan_topics(path)
    print(f'{path}: {len(topics)} 个话题')
    for t in sorted(topics):
        info = topics[t]
        mt = sniff_msgtype(info['raw0']) if info['raw0'] else 'unknown'
        print(f'  {t:<44} {mt:<34} {info["count"]} 条')
