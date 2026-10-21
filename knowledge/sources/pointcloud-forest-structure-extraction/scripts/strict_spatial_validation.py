#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
严格空间验证：判断任意一张"树表/检出表"的每个位置**是否真的落在一根树干上**。
—— 用来交叉验证「锚（真值）可不可信」，也用来验证在线检测的输出。

核心判据（关键在【半径必须收紧】）：
    该位置 水平 R 内、离地 [LO,HI] 的点数 >= MIN_PTS
    R=0.30 m（不是 1.2 m！）：只够覆盖"自己脚下这根"；
      树间距 >= 2 m ⇒ 邻树树干进不来。
    对照实测：锚表(CHM峰值法) 32% 为 0 点、中位 24；而检出表 90% >=20 点、中位 370
      ⇒ 一眼看出【锚是坏的】而不是算法错。

为什么必须做这一步：
    多个结构完全不同的算法给出同一个烂指标时，瓶颈往往在【评估/锚】，不在算法。
    一条判据如果连"真值"自己都要否掉 1/3，它就不是判据 —— 先用它跑锚。

用法:
    python strict_spatial_validation.py --cloud a.las \
        --table anchor.csv --table detections.csv \
        [--radius 0.30] [--lo 1.0] [--hi 2.2] [--min-pts 20] [--half 40]

表格式：CSV，需含 x,y 两列（列名不分大小写）；有 ground_z 列则用真实离地高度，
        没有则用 --gz 指定的常数当整片地面（坡地请务必提供 ground_z）。
输出：每张表的 点数中位 / 达标比例 / 0 点比例 + 逐表结论。无需 open3d / laspy。
"""
import argparse
import csv
import math
import os
import struct
import sys

import numpy as np


# ---------------- LAS 读取（手解头部，零依赖） ----------------
def las_header(fp):
    fp.seek(0)
    h = fp.read(375)
    if h[:4] != b"LASF":
        raise ValueError("不是 LAS 文件")
    return dict(
        fmt=h[104] & 0x3F,
        plen=struct.unpack_from("<H", h, 105)[0],
        n=struct.unpack_from("<I", h, 107)[0],
        scale=struct.unpack_from("<ddd", h, 131),
        off=struct.unpack_from("<ddd", h, 155),
        data_off=struct.unpack_from("<I", h, 96)[0],
    )


def iter_las(path, chunk=2_000_000):
    """yield (x, y, z) 三个 float32 数组。点格式 0/1/2/3 的 X/Y/Z 都在前 12 字节。"""
    with open(path, "rb") as f:
        hi = las_header(f)
        f.seek(hi["data_off"])
        plen = hi["plen"]
        sx, sy, sz = hi["scale"]
        ox, oy, oz = hi["off"]
        left = hi["n"]
        buf = bytearray(plen * chunk)
        view = memoryview(buf)
        while left > 0:
            k = min(chunk, left)
            want = plen * k
            got = f.readinto(view[:want])
            if not got or got < want:
                k = got // plen
                if k <= 0:
                    break
            a = np.frombuffer(buf, dtype=np.uint8, count=plen * k).reshape(k, plen)
            xi = a[:, 0:4].copy().view("<i4").ravel()
            yi = a[:, 4:8].copy().view("<i4").ravel()
            zi = a[:, 8:12].copy().view("<i4").ravel()
            yield (xi * sx + ox).astype(np.float32), \
                  (yi * sy + oy).astype(np.float32), \
                  (zi * sz + oz).astype(np.float32)
            left -= k


def iter_pcd(path, chunk=2_000_000):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline()
            if not line:
                break
            s = line.decode("ascii", "ignore").strip()
            if not s or s.startswith("#"):
                continue
            p = s.split()
            hdr[p[0].upper()] = p[1:]
            if p[0].upper() == "DATA":
                break
        n = int(hdr["POINTS"][0])
        fields = [x.lower() for x in hdr["FIELDS"]]
        sizes = [int(x) for x in hdr["SIZE"]]
        types = [x.upper() for x in hdr["TYPE"]]
        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
        dt = np.dtype([(fields[i], (types[i].lower() + str(sizes[i])))
                       for i in range(len(fields))])
        left = n
        while left > 0:
            k = min(chunk, left)
            raw = np.frombuffer(f.read(dt.itemsize * k), dtype=dt, count=k)
            yield (raw[fields[ix]].astype(np.float32),
                   raw[fields[iy]].astype(np.float32),
                   raw[fields[iz]].astype(np.float32))
            left -= k


def iter_cloud(path):
    low = path.lower()
    if low.endswith(".las") or low.endswith(".laz"):
        if low.endswith(".laz"):
            sys.exit("[X] .laz 是压缩格式，本脚本不支持；请先解压成 .las")
        return iter_las(path)
    return iter_pcd(path)


# ---------------- 读表 ----------------
def load_table(path, half, gz_const=None):
    xs, ys, gz = [], [], []
    with open(path, newline="", encoding="utf-8-sig") as f:
        r = csv.DictReader(f)
        cols = {(c or "").strip().lower(): c for c in (r.fieldnames or [])}
        if "x" not in cols or "y" not in cols:
            sys.exit("[X] %s 缺少 x/y 列（实际列: %s）" % (path, r.fieldnames))
        gcol = cols.get("ground_z")
        for d in r:
            try:
                x = float(d[cols["x"]]); y = float(d[cols["y"]])
            except (TypeError, ValueError):
                continue
            if half and (abs(x) > half or abs(y) > half):
                continue
            z = float(d[gcol]) if gcol else (gz_const if gz_const is not None else 0.0)
            xs.append(x); ys.append(y); gz.append(z)
    return np.asarray(xs, np.float64), np.asarray(ys, np.float64), np.asarray(gz, np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True, help="真值点云 .las/.pcd")
    ap.add_argument("--table", action="append", required=True,
                    help="待验证的表（CSV，需 x/y 列；有 ground_z 更好）。可重复给多张做对照")
    ap.add_argument("--radius", type=float, default=0.30,
                    help="水平判据半径 m。★默认 0.30（不是 1.2！1.2 在密林里无区分力）")
    ap.add_argument("--lo", type=float, default=1.0, help="树干带离地下限 m")
    ap.add_argument("--hi", type=float, default=2.2, help="树干带离地上限 m")
    ap.add_argument("--min-pts", type=int, default=20, help="判定'落在树干上'的最少点数")
    ap.add_argument("--half", type=float, default=0.0, help="只看 |x|,|y|<=half 的点/行（0=不限）")
    ap.add_argument("--gz", type=float, default=None,
                    help="表里没有 ground_z 时，用这个常数当整片地面（坡地请务必给 ground_z 列）")
    a = ap.parse_args()

    if a.radius > 0.6:
        print("⚠️  警告：--radius %.2f m 偏松。半径 >0.6 m 时邻树树干会进来，"
              "判据将失去区分力（密林里几乎任何位置都会'通过'）。" % a.radius)

    tables = []
    for p in a.table:
        if not os.path.isfile(p):
            sys.exit("[X] 表不存在: %s" % p)
        xs, ys, gz = load_table(p, a.half, a.gz)
        tables.append((p, xs, ys, gz))
        print("表 %-46s %d 个位置" % (os.path.basename(p), xs.size))

    # 为每张表统计
    counts = []
    for _, xs, ys, gz in tables:
        counts.append((np.zeros(xs.size, np.int64),
                       np.asarray(np.c_[xs, ys]) if xs.size else np.zeros((0, 2)),
                       gz))

    print("\n扫描点云（一次遍历，同时统计所有表）…")
    total = 0
    for cx, cy, cz in iter_cloud(a.cloud):
        total += cx.size
        for idx, (cnt, xy, gz) in enumerate(counts):
            if xy.shape[0] == 0:
                continue
            if a.half:
                m = (np.abs(cx) <= a.half + 2) & (np.abs(cy) <= a.half + 2)
                if not m.any():
                    continue
                px, py, pz = cx[m], cy[m], cz[m]
            else:
                px, py, pz = cx, cy, cz
            dx = px[:, None] - xy[None, :, 0]
            dy = py[:, None] - xy[None, :, 1]
            d2 = dx * dx + dy * dy
            j = np.argmin(d2, axis=1)
            hit = d2[np.arange(j.size), j] <= a.radius * a.radius
            if not hit.any():
                continue
            jj = j[hit]
            zb = pz[hit] - gz[jj]
            ok = (zb >= a.lo) & (zb <= a.hi)
            if ok.any():
                np.add.at(cnt, jj[ok], 1)
    print("点云共 %s 点\n" % format(total, ","))

    print("=== 结果（判据：水平 %.2f m 内、离地 %.1f~%.1f m，点数 >= %d）==="
          % (a.radius, a.lo, a.hi, a.min_pts))
    print("  %-46s %6s %10s %10s %10s" % ("表", "位置数", "点数中位", "达标%", "0点%"))
    verdict = []
    for (path, xs, ys, gz), (cnt, _, _) in zip(tables, counts):
        if xs.size == 0:
            print("  %-46s %6d  (空)" % (os.path.basename(path), 0))
            continue
        med = int(np.median(cnt))
        good = float((cnt >= a.min_pts).mean())
        zero = float((cnt == 0).mean())
        print("  %-46s %6d %10d %9.0f%% %9.0f%%"
              % (os.path.basename(path), xs.size, med, good * 100, zero * 100))
        verdict.append((os.path.basename(path), good, zero, med))

    print("\n=== 判读 ===")
    for name, good, zero, med in verdict:
        if zero > 0.20:
            print("  ✗ %s: %.0f%% 的位置在点云里【一个点都没有】⇒ 这张表本身不可信，"
                  "不要拿它当锚（很可能是 CHM 峰值法的树冠中心，不是树干位置）" % (name, zero * 100))
        elif good < 0.60:
            print("  ⚠️ %s: 只有 %.0f%% 达标 ⇒ 可疑（是锚则锚坏；是检出则过检/偏移）" % (name, good * 100))
        else:
            print("  ✓ %s: %.0f%% 的位置真的落在一根树干上 ⇒ 可信" % (name, good * 100))
    print("\n提示：把锚表和检出表一起跑 —— 若检出表明显优于锚表，"
          "说明【算法是对的，是锚把它压坏了】。")


if __name__ == "__main__":
    main()
