#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树干聚类法数树 —— 独立于"CHM 冠层峰值法"的第二种棵数估计。

为什么需要两种方法
------------------
"冠层高度模型(CHM)局部极大"法容易把一棵大树的几个次峰算成多棵（偏多），
单跑一种方法无法判断"到底有几棵树"。用几何上完全不同的第二法取区间，
才知道可信范围；不要为了凑数在区间外造假。

原理
----
离地高度 AGL 0.5~3.0 m 带内的点，投到 0.25 m 网格做 8-邻接连通域。
该高度**树干彼此分离**（间距数米 >> 树干直径 0.3~0.8 m），故连通块数 ≈ 棵数。
过滤：格数过小 = 噪点；单块跨度 > max-span = 灌木丛 / 相邻树干粘连。

实测锚点（4.59 ha 成熟松林，25% 采集覆盖，7600 万点）
----------------------------------------------------
  closing x1 : 187 棵（等效直径中位 0.63 m，10%~90% = 0.49~0.85 m）  <== 用这个
  closing x2 :  96 棵（过度膨胀把邻近树干并掉，偏少）
  对照 CHM 峰值法：230 棵  ->  真实值 187~230
等效直径落在该树种胸径区间 = 结果可信的旁证。

用法
----
  # 直接读 LAS（分块，几千万点约 30 s，纯 numpy 不依赖 laspy）
  python trunk_cluster_tree_count.py --las forest.las --ground ground_1m.npy --grid-origin -108.24 -84.38

  # 读 Nx3 点云 npy（单位米）
  python trunk_cluster_tree_count.py --points xyz.npy --ground ground_1m.npy --grid-origin -108.24 -84.38

  # 没有地面栅格（只适合起伏 < 2 m 的平地，起伏大会明显偏多）
  python trunk_cluster_tree_count.py --las forest.las

  # 调参
  ... --agl-lo 0.5 --agl-hi 3.0 --cell 0.25 --max-span 1.6 --min-cells 2

--grid-origin 是构建 ground 栅格时用的 XMIN/YMIN（格中心 = XMIN + i + 0.5）⚠️
传错只影响地面采样位置，会让 AGL 偏掉、棵数偏掉；不确定就先跑 CHM 法对比。
"""
import argparse
import struct
import sys

import numpy as np
from scipy import ndimage


# ---------------- LAS 读取（手解二进制头，只取 X/Y/Z） ----------------
def read_las_header(path):
    with open(path, "rb") as f:
        if f.read(4) != b"LASF":
            raise ValueError("不是 LAS 文件（缺 LASF 魔数）")
        f.seek(94); hsize = struct.unpack("<H", f.read(2))[0]
        f.seek(96); off = struct.unpack("<I", f.read(4))[0]
        f.seek(104); fmt = struct.unpack("B", f.read(1))[0]
        f.seek(105); rlen = struct.unpack("<H", f.read(2))[0]
        f.seek(107); n = struct.unpack("<I", f.read(4))[0]
        f.seek(131); sx, sy, sz = struct.unpack("<3d", f.read(24))
        f.seek(155); ox, oy, oz = struct.unpack("<3d", f.read(24))
        f.seek(179); mx, mnx, my, mny, mz, mnz = struct.unpack("<6d", f.read(48))
    if n == 0:                                  # LAS 1.4 用 64 位点数
        with open(path, "rb") as f:
            f.seek(247); n = struct.unpack("<Q", f.read(8))[0]
    return dict(hsize=hsize, offset=off, fmt=fmt, rlen=rlen, n=n,
                scale=(sx, sy, sz), origin=(ox, oy, oz),
                xmin=mnx, xmax=mx, ymin=mny, ymax=my, zmin=mnz, zmax=mz)


def iter_las_xyz(path, chunk=4_000_000):
    h = read_las_header(path)
    # 格式 0/1/2/3/6/7/8 的 X/Y/Z 都在记录前 12 字节（i4 x3）
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<i4"] * 3,
                   "offsets": [0, 4, 8], "itemsize": h["rlen"]})
    sx, sy, sz = h["scale"]; ox, oy, oz = h["origin"]
    with open(path, "rb") as f:
        f.seek(h["offset"])
        left = h["n"]
        while left > 0:
            k = min(chunk, left)
            buf = f.read(k * h["rlen"])
            if len(buf) < k * h["rlen"]:
                break
            a = np.frombuffer(buf, dtype=dt, count=k)
            yield (a["x"].astype(np.float64) * sx + ox,
                   a["y"].astype(np.float64) * sy + oy,
                   a["z"].astype(np.float64) * sz + oz)
            left -= k


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--las", help="LAS 文件（LAZ 需先解压）")
    src.add_argument("--points", help="Nx3 点云 .npy，单位米")
    ap.add_argument("--ground", help="1 m 地面栅格 .npy（可选；缺省用 Z 的 1%% 分位近似）")
    ap.add_argument("--grid-origin", nargs=2, type=float, metavar=("XMIN", "YMIN"),
                    help="构建 --ground 时用的栅格角点原点（格中心 = origin + i + 0.5）")
    ap.add_argument("--agl-lo", type=float, default=0.5)
    ap.add_argument("--agl-hi", type=float, default=3.0)
    ap.add_argument("--cell", type=float, default=0.25, help="聚类网格边长 m")
    ap.add_argument("--max-span", type=float, default=1.6, help="单块跨度上限 m（超过=灌木/粘连）")
    ap.add_argument("--min-cells", type=int, default=2, help="单块最少格数（低于=噪点）")
    a = ap.parse_args()

    # 1) 采集点
    if a.las:
        h = read_las_header(a.las)
        print("LAS: 点 %s  格式 %d  记录长 %d  scale %.4f" % (
            format(h["n"], ","), h["fmt"], h["rlen"], h["scale"][0]))
        print("     范围 X %.2f..%.2f  Y %.2f..%.2f  Z %.2f..%.2f" % (
            h["xmin"], h["xmax"], h["ymin"], h["ymax"], h["zmin"], h["zmax"]))
        Xs, Ys, Zs = [], [], []
        for x, y, z in iter_las_xyz(a.las):
            Xs.append(x); Ys.append(y); Zs.append(z)
        X = np.concatenate(Xs); Y = np.concatenate(Ys); Z = np.concatenate(Zs)
    else:
        P = np.load(a.points)
        if P.ndim != 2 or P.shape[1] < 3:
            sys.exit("--points 需要 Nx3 数组")
        X, Y, Z = P[:, 0], P[:, 1], P[:, 2]
    print("读入 %s 点" % format(len(X), ","))

    # 2) AGL
    if a.ground:
        g = np.load(a.ground)
        GH, GW = g.shape
        if a.grid_origin:
            x0, y0 = a.grid_origin
        else:
            x0, y0 = np.floor(X.min()), np.floor(Y.min())
            print("[!] 未给 --grid-origin，按点云最小角近似对齐地面栅格（可能偏）")
        i = np.clip(np.round(X - x0 - 0.5).astype(int), 0, GW - 1)
        j = np.clip(np.round(Y - y0 - 0.5).astype(int), 0, GH - 1)
        agl = Z - g[j, i]
        print("地面栅格 %d x %d，采样自 origin (%.2f, %.2f)" % (GH, GW, x0, y0))
    else:
        agl = Z - np.percentile(Z, 1.0)
        print("[!] 未提供地面栅格 -> 用 Z 的 1% 分位当近似地面（起伏大会明显偏多）")

    m = (agl >= a.agl_lo) & (agl <= a.agl_hi)
    print("AGL %.1f~%.1f m 带内点: %s (%.2f%%)" % (
        a.agl_lo, a.agl_hi, format(int(m.sum()), ","), 100.0 * m.sum() / max(len(X), 1)))
    if m.sum() == 0:
        sys.exit("带内无点 —— 检查 --grid-origin / AGL 范围 / 点云单位")
    px, py = X[m], Y[m]

    # 3) 网格化 + 连通域
    C = a.cell
    ox, oy = px.min() - C, py.min() - C
    nx = int((px.max() - ox) / C) + 2
    ny = int((py.max() - oy) / C) + 2
    img = np.zeros((ny, nx), bool)
    img[((py - oy) / C).astype(int), ((px - ox) / C).astype(int)] = True
    print("聚类网格 %d x %d (%.2f m/格)  非空格 %s" % (ny, nx, C, format(int(img.sum()), ",")))

    out = {}
    for it in (1, 2):
        im2 = ndimage.binary_closing(img, np.ones((3, 3), bool), iterations=it)
        lab, n = ndimage.label(im2, np.ones((3, 3), int))
        sizes = np.bincount(lab.ravel()); sizes[0] = 0
        objs = ndimage.find_objects(lab)
        kept, tiny, big, areas = 0, 0, 0, []
        for l, sl in enumerate(objs, start=1):
            if l >= len(sizes) or sizes[l] == 0:
                continue
            hh = (sl[0].stop - sl[0].start) * C
            ww = (sl[1].stop - sl[1].start) * C
            if sizes[l] < a.min_cells or max(hh, ww) > a.max_span:
                if max(hh, ww) > a.max_span:
                    big += 1
                else:
                    tiny += 1
                continue
            kept += 1
            areas.append(sizes[l] * C * C)
        areas = np.array(areas) if areas else np.array([1.0])
        d = 2 * np.sqrt(areas / np.pi)
        out[it] = kept
        tag = "   <== 以这个为准" if it == 1 else "   (过度膨胀会并掉邻近树干，偏少)"
        print("  closing x%d: 连通域 %-6d 保留 %-5d (剔除 过小%d/过大%d)  "
              "树干等效直径 中位 %.2f m (10%%~90%%: %.2f~%.2f)%s" % (
                  it, n, kept, tiny, big, np.median(d),
                  np.percentile(d, 10), np.percentile(d, 90), tag))

    print()
    print("==> 棵数估计 (树干聚类法): %d" % out[1])
    print("    请再跑一次 CHM 冠层峰值法，取两法的【区间】作为可信范围；")
    print("    树干等效直径是否落在该树种胸径区间，是结果可信的旁证。")


if __name__ == "__main__":
    main()
