#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
occ_probe.py — 探测 ROG-Map / 任意 PointCloud2 话题在某点附近的占用情况
=====================================================================
用途（fouruav 路径A 走廊生成失败诊断，2026-08-11 实机验证）：
  "GeneratePolytopeFromLine failed" / "The start point in obstacle" /
  "Cannot find continuous corridor on path, overlap only -XXX"
  = 飞机当前位置附近被障碍挤压，走廊盒子撑不出来。
  本脚本统计查询点 radius 米内的占用点数并列出最近点，判断起飞环境。

用法：
  python3 occ_probe.py                        # 默认: /fsm_node/rog_map/occ @ (0,0,0.6) r=1.0
  python3 occ_probe.py /cloud_registered 0 0 0.6 0.5
  python3 occ_probe.py <topic> <x> <y> <z> <radius>

要点：
  - PointCloud2 data 每 point_step(常16) 字节一点：x,y,z 是 float32，
    偏移由 fields 给出（x:0 y:4 z:8 通常），后 4 字节 padding。
  - 判读：起点 1m 内占用点 >50 且最近 <0.5m = 起飞点太挤（人/物体站太近），
    走廊必然失败 → 让人退远 1.5m+ 再测，先别动代码。
"""
import math
import struct
import sys

import rospy
from sensor_msgs.msg import PointCloud2


def probe(msg, cx, cy, cz, radius):
    fields = {f.name: f.offset for f in msg.fields}
    ox = fields.get('x', 0)
    oy = fields.get('y', 4)
    oz = fields.get('z', 8)
    ps = msg.point_step or 16
    data = msg.data
    n_pts = len(data) // ps
    near = []
    for i in range(n_pts):
        base = i * ps
        x, y, z = struct.unpack_from('<fff', data, base + ox)
        d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2)
        if d < radius:
            near.append((d, x, y, z))
    near.sort()
    print(f"[occ_probe] frame={msg.header.frame_id} 总点数={n_pts}")
    print(f"[occ_probe] 查询点 ({cx},{cy},{cz}) 半径 {radius}m 内占用点: {len(near)}")
    for d, x, y, z in near[:15]:
        print(f"   ({x:.2f}, {y:.2f}, {z:.2f})  dist={d:.2f}m")
    if near:
        xs = [p[1] for p in near]; ys = [p[2] for p in near]; zs = [p[3] for p in near]
        print(f"[occ_probe] 占用点范围: x[{min(xs):.2f},{max(xs):.2f}] "
              f"y[{min(ys):.2f},{max(ys):.2f}] z[{min(zs):.2f},{max(zs):.2f}]")
        print("[occ_probe] 判读: 垂直面/对称分布=有人在附近; 密集层=地面/机架残留")
    else:
        print("[occ_probe] 查询点附近干净，走廊失败应另找原因(见 uav-trajectory-replay 坑13)")


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/fsm_node/rog_map/occ'
    cx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    cy = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    cz = float(sys.argv[4]) if len(sys.argv) > 4 else 0.6
    radius = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    rospy.init_node('occ_probe', anonymous=True)
    rospy.loginfo(f"[occ_probe] 等待话题 {topic} ...")
    msg = rospy.wait_for_message(topic, PointCloud2, timeout=10)
    probe(msg, cx, cy, cz, radius)


if __name__ == '__main__':
    main()
