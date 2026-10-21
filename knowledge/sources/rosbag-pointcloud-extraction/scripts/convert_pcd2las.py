#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCD → LAS 1.2（点格式 3，含 RGB + intensity），无需 laspy
用法: python convert_pcd2las.py <pcd文件或目录> [输出目录]
"""
import os
import struct
import sys

import numpy as np


def read_pcd(path):
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            if line.startswith(b'DATA'):
                break
            header.append(line.decode().strip())
        n = int([l.split()[1] for l in header if l.startswith('POINTS')][0])
        fields = [l.split()[1:] for l in header if l.startswith('FIELDS')][0]
        data = np.frombuffer(f.read(), dtype=np.float32).reshape(n, len(fields))
    return data, fields


def write_las(path, data, fields):
    xyz = data[:, :3].astype(np.float64)
    n = len(xyz)
    if n == 0:
        raise ValueError('空点云')

    # intensity: 字段里有 intensity/reflectivity 就用（0-255 → 0-65535）
    intensity = np.zeros(n, np.uint16)
    for name in ('intensity', 'reflectivity', 'intensities'):
        if name in fields:
            i = fields.index(name)
            intensity = np.clip(data[:, i] * 257, 0, 65535).astype(np.uint16)
            break

    # rgb: PCD rgb 字段是 float32 view of uint32(R<<16|G<<8|B)
    r = np.zeros(n, np.uint16)
    g = np.zeros(n, np.uint16)
    b = np.zeros(n, np.uint16)
    if 'rgb' in fields:
        v = data[:, fields.index('rgb')].view(np.uint32)
        r = ((v >> 16) & 255).astype(np.uint16) << 8
        g = ((v >> 8) & 255).astype(np.uint16) << 8
        b = (v & 255).astype(np.uint16) << 8

    # 坐标缩放: 1mm 精度, offset = 各轴 min
    scale = 0.001
    offx, offy, offz = xyz.min(axis=0)
    xi = np.round((xyz[:, 0] - offx) / scale).astype(np.int32)
    yi = np.round((xyz[:, 1] - offy) / scale).astype(np.int32)
    zi = np.round((xyz[:, 2] - offz) / scale).astype(np.int32)

    # LAS 1.2 头 (227 字节)
    hdr = b'LASF'
    hdr += struct.pack('<HH', 0, 0)                       # file source id, global encoding
    hdr += b'\x00' * 16                                   # project id GUID
    hdr += struct.pack('<BB', 1, 2)                       # version 1.2
    hdr += b'pcd2las'.ljust(32, b'\x00')                  # system identifier
    hdr += b'bag2pcd_tool'.ljust(32, b'\x00')             # generating software
    hdr += struct.pack('<HH', 0, 0)                       # creation day/year
    hdr += struct.pack('<H', 227)                         # header size
    hdr += struct.pack('<I', 227)                         # offset to point data
    hdr += struct.pack('<I', 0)                           # number of VLRs
    hdr += struct.pack('<B', 3)                           # point data format (3=xyz+intensity+gps+rgb)
    hdr += struct.pack('<H', 34)                          # point record length
    hdr += struct.pack('<I', n)                           # legacy number of points
    hdr += struct.pack('<5I', n, 0, 0, 0, 0)              # legacy points by return
    hdr += struct.pack('<3d', scale, scale, scale)        # scale
    hdr += struct.pack('<3d', offx, offy, offz)           # offset
    hdr += struct.pack('<6d', xyz[:, 0].max(), xyz[:, 0].min(),
                       xyz[:, 1].max(), xyz[:, 1].min(),
                       xyz[:, 2].max(), xyz[:, 2].min())  # max/min X/Y/Z
    assert len(hdr) == 227, f'header 长度 {len(hdr)} != 227'

    # 点记录格式 3 (34 字节)
    rec = np.zeros(n, dtype=[('x', '<i4'), ('y', '<i4'), ('z', '<i4'),
                             ('intensity', '<u2'),
                             ('bit_fields', 'u1'), ('classification', 'u1'),
                             ('scan_angle', 'i1'), ('user_data', 'u1'),
                             ('point_source', '<u2'), ('gps_time', '<f8'),
                             ('r', '<u2'), ('g', '<u2'), ('b', '<u2')])
    rec['x'], rec['y'], rec['z'] = xi, yi, zi
    rec['intensity'] = intensity
    rec['bit_fields'] = 1          # return 1 of 1
    rec['gps_time'] = 0
    rec['r'], rec['g'], rec['b'] = r, g, b

    with open(path, 'wb') as f:
        f.write(hdr)
        f.write(rec.tobytes())
    return n


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\<USER>\Desktop\pcd'
    out_dir = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\<USER>\Desktop\las'
    os.makedirs(out_dir, exist_ok=True)

    if os.path.isfile(src):
        files = [src]
    else:
        files = [os.path.join(src, f) for f in sorted(os.listdir(src)) if f.lower().endswith('.pcd')]
    if not files:
        print(f'❌ 没有找到 .pcd 文件: {src}')
        sys.exit(1)

    for p in files:
        try:
            data, fields = read_pcd(p)
            out = os.path.join(out_dir, os.path.splitext(os.path.basename(p))[0] + '.las')
            n = write_las(out, data, fields)
            print(f'✅ {os.path.basename(p)} → {os.path.basename(out)} ({n} 点)')
        except Exception as e:
            print(f'❌ {os.path.basename(p)}: {e}')
    print(f'\n输出目录: {out_dir}')


if __name__ == '__main__':
    main()
