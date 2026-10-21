#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
las_analyze.py —— 把一份 .las/.laz 点云地图"体检"成可决策的数字：能不能搬进仿真？规模多大？

只用 numpy（不需要 laspy / PDAL / open3d / CloudCompare）。LAS 头是固定二进制布局，
按规范手解即可；点记录按 header 里的 point_data_record_length 建结构化 dtype，支持格式 0~3。

用法:
    python3 las_analyze.py <file.las> [--voxel 0.5] [--cell 1.0] [--out DIR] [--ply] [--tz-min -30] [--tz-max 50]

输出（都是决策依据，不是好看而已）:
  1) 头部：版本 / 生成软件 / 点数 / 比例 / 范围 / 面积 / 密度
  2) 高度直方图 —— 看地形 + 树冠是否可分
  3) 1m 网格统计 + 覆盖率 —— 判断这份数据是"整块矩形"还是"采集带"（手持扫描常见后者！）
  4) 0.5m 体素抽稀总数 —— 决定 UE4 用 ISM 还是合并大盒子
  5) **离地高度(AGL)分带** ★ 最关键：飞机飞行高度带内的体素数 = UE4 碰撞几何的规模
  6) 可选 --ply：写一个按高度上色的 PLY，直接在 CloudCompare/RViz 里看

⚠️ 核心方法论（我踩过，务必照做）:
   AGL 必须用【逐格局部地面】做基准。用全局低分位当"地面"是错的 ——
   地形起伏几十米时，那个数字实际是"最低洼地带"的体素数，会得出完全错误的结论。
"""
import argparse
import os
import struct
import sys
import time

import numpy as np

FMT_EXTRA = {
    0: dict(rgb=False, gps=False),
    1: dict(rgb=False, gps=True),
    2: dict(rgb=True, gps=False),
    3: dict(rgb=True, gps=True),
}


def parse_header(path):
    with open(path, "rb") as f:
        h = f.read(375)
    if h[0:4] != b"LASF":
        raise SystemExit("不是 LAS 文件（签名不是 LASF）")
    vmaj, vmin = h[24], h[25]
    n_pts = struct.unpack_from("<I", h, 107)[0]
    if vmaj == 1 and vmin >= 4 and len(h) >= 375:
        n64 = struct.unpack_from("<Q", h, 247)[0]
        if n64:
            n_pts = n64
    sx, sy, sz = struct.unpack_from("<ddd", h, 131)
    ox, oy, oz = struct.unpack_from("<ddd", h, 155)
    maxx, minx, maxy, miny, maxz, minz = struct.unpack_from("<dddddd", h, 179)
    return dict(
        version="%d.%d" % (vmaj, vmin),
        software=h[58:90].decode("ascii", "replace").strip("\x00 "),
        n_pts=n_pts,
        fmt=h[104],
        reclen=struct.unpack_from("<H", h, 105)[0],
        offset=struct.unpack_from("<I", h, 96)[0],
        scale=(sx, sy, sz),
        offs=(ox, oy, oz),
        xmin=minx, xmax=maxx, ymin=miny, ymax=maxy, zmin=minz, zmax=maxz,
    )


def make_dtype(fmt, reclen):
    """LAS 点格式 0~3 的前 34/28 字节布局（超出部分用 pad 占掉）。"""
    base = [("X", "<i4"), ("Y", "<i4"), ("Z", "<i4"), ("intensity", "<u2"),
            ("flags", "u1"), ("cls", "u1"), ("scanang", "i1"), ("user", "u1"),
            ("psid", "<u2"), ("gpstime", "<f8"), ("R", "<u2"), ("G", "<u2"), ("B", "<u2")]
    offs = [0, 4, 8, 12, 14, 15, 16, 17, 18, 20, 28, 30, 32]
    if fmt in (0, 2):
        base = base[:9]
        offs = offs[:9]
        if fmt == 2:
            base += [("R", "<u2"), ("G", "<u2"), ("B", "<u2")]
            offs += [20, 22, 24]
    elif fmt == 1:
        base = base[:10]
        offs = offs[:10]
    if reclen > max(offs) + 2:
        base.append(("_pad", "u1"))
        offs.append(reclen - 1)
    return np.dtype({"names": [b[0] for b in base], "formats": [b[1] for b in base],
                     "offsets": offs, "itemsize": reclen})


def median_filter2d(a, k):
    """numpy-only 中值滤波（不用 scipy）。边界用边缘复制。"""
    from numpy.lib.stride_tricks import sliding_window_view
    r = k // 2
    pad = np.pad(a, r, mode="edge")
    w = sliding_window_view(pad, (k, k))
    return np.median(w, axis=(-1, -2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("las")
    ap.add_argument("--voxel", type=float, default=0.5, help="体素边长 m（默认 0.5）")
    ap.add_argument("--cell", type=float, default=1.0, help="XY 网格边长 m（默认 1.0）")
    ap.add_argument("--tz-min", type=float, default=-30.0, help="Z 直方图下限")
    ap.add_argument("--tz-max", type=float, default=50.0, help="Z 直方图上限")
    ap.add_argument("--bands", default="1,1.5,2,3,4,6,10", help="AGL 分带上限（逗号分隔）")
    ap.add_argument("--out", default=None, help="输出目录（默认 <las所在目录>/_las_analysis）")
    ap.add_argument("--ply", action="store_true", help="额外写按高度上色的 PLY")
    ap.add_argument("--chunk", type=int, default=2_000_000)
    a = ap.parse_args()

    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.las)), "_las_analysis")
    os.makedirs(out, exist_ok=True)

    H = parse_header(a.las)
    print("=" * 74)
    print("文件        : %s  (%.1f MB)" % (os.path.basename(a.las), os.path.getsize(a.las) / 1e6))
    print("LAS 版本    : %s   生成软件: %s" % (H["version"], H["software"]))
    print("点格式      : %d  记录长 %d B  点数 %s" % (H["fmt"], H["reclen"], format(H["n_pts"], ",")))
    print("比例/偏移   : %s / %s" % (H["scale"], H["offs"]))
    dx, dy, dz = H["xmax"] - H["xmin"], H["ymax"] - H["ymin"], H["zmax"] - H["zmin"]
    area = dx * dy
    print("范围        : X %.1f..%.1f (%.1f m)  Y %.1f..%.1f (%.1f m)  Z %.1f..%.1f (%.1f m)"
          % (H["xmin"], H["xmax"], dx, H["ymin"], H["ymax"], dy, H["zmin"], H["zmax"], dz))
    print("面积/密度   : %.0f m2 = %.2f ha  |  平均 %.1f 点/m2" % (area, area / 1e4, H["n_pts"] / area))

    dt = make_dtype(H["fmt"], H["reclen"])
    sx, sy, sz = H["scale"]
    GW = int(np.ceil(dx / a.cell)) + 1
    GH = int(np.ceil(dy / a.cell)) + 1
    gmin = np.full((GH, GW), np.inf, np.float64)
    gmax = np.full((GH, GW), -np.inf, np.float64)
    gcnt = np.zeros((GH, GW), np.int32)
    zlo, zhi = a.tz_min, a.tz_max
    zhist = np.zeros(256, np.int64)
    clshist = np.zeros(256, np.int64)
    rgb_nz = 0
    vox, zmin_all, zmax_all, xmin_all, xmax_all, ymin_all, ymax_all = [], np.inf, -np.inf, np.inf, -np.inf, np.inf, -np.inf

    t0 = time.time()
    done = 0
    with open(a.las, "rb") as f:
        f.seek(H["offset"])
        while done < H["n_pts"]:
            want = min(a.chunk, H["n_pts"] - done)
            buf = f.read(want * H["reclen"])
            if not buf:
                break
            want = len(buf) // H["reclen"]
            if want == 0:
                break
            r = np.frombuffer(buf[: want * H["reclen"]], dtype=dt, count=want)
            x = r["X"].astype(np.float32) * sx
            y = r["Y"].astype(np.float32) * sy
            z = r["Z"].astype(np.float32) * sz
            gi = np.clip((y - H["ymin"]) / a.cell, 0, GH - 1).astype(np.int32)
            gj = np.clip((x - H["xmin"]) / a.cell, 0, GW - 1).astype(np.int32)
            np.minimum.at(gmin, (gi, gj), z)
            np.maximum.at(gmax, (gi, gj), z)
            np.add.at(gcnt, (gi, gj), 1)
            np.add.at(zhist, np.clip(((z - zlo) / (zhi - zlo) * 255).astype(np.int32), 0, 255), 1)
            np.add.at(clshist, r["cls"].astype(np.int32), 1)
            if "R" in dt.names:
                rgb_nz += int(np.count_nonzero(r["R"] | r["G"] | r["B"]))
            v = a.voxel
            vi = np.floor(x / v).astype(np.int64)
            vj = np.floor(y / v).astype(np.int64)
            vk = np.floor(z / v).astype(np.int64)
            vox.append(np.unique(((vi + 8192) * 16384 + (vj + 8192)) * 16384 + (vk + 8192)))
            zmin_all = min(zmin_all, float(z.min())); zmax_all = max(zmax_all, float(z.max()))
            xmin_all = min(xmin_all, float(x.min())); xmax_all = max(xmax_all, float(x.max()))
            ymin_all = min(ymin_all, float(y.min())); ymax_all = max(ymax_all, float(y.max()))
            done += want
            if done % (a.chunk * 5) == 0:
                print("   ... %s / %s 点  %.0fs" % (format(done, ","), format(H["n_pts"], ","), time.time() - t0), flush=True)

    V = np.unique(np.concatenate(vox))
    print("\n体素 %sm 抽稀: %s 个（抽稀率 %.2f%%）" % (a.voxel, format(len(V), ","), 100.0 * len(V) / H["n_pts"]))

    print("\n=== Z 直方图（每 %.0f m）===" % ((zhi - zlo) / 16))
    step = (zhi - zlo) / 16.0
    zc = (np.arange(256) / 255.0) * (zhi - zlo) + zlo
    for i in range(16):
        m = (zc >= zlo + i * step) & (zc < zlo + (i + 1) * step)
        c = int(zhist[m].sum())
        if c:
            print("  z %+7.1f ~ %+7.1f : %14s  %5.1f%%  %s"
                  % (zlo + i * step, zlo + (i + 1) * step, format(c, ","), 100.0 * c / H["n_pts"],
                     "#" * int(50 * c / H["n_pts"])))

    valid = np.isfinite(gmin)
    cov = 100.0 * valid.sum() / valid.size
    print("\n=== %sm 网格覆盖率: %.1f%%（%s / %s）%s ===" % (
        a.cell, cov, format(int(valid.sum()), ","), format(valid.size, ","),
        "★ 明显不满 = 采集带，不是整块矩形" if cov < 70 else ""))
    gm = gmin[valid]
    for q in (1, 25, 50, 75, 99):
        print("  每格最低点 %2d%% 分位: %+7.2f m" % (q, np.percentile(gm, q)))

    # ---- 地面：逐格最低点 + 迭代中值滤波（只看接近地面的格）----
    fill = float(np.median(gm))
    g = np.where(valid, gmin, fill)
    ground = median_filter2d(g, 7)
    for _ in range(3):
        d = g - ground
        m = valid & (d > -3.0) & (d < 1.0)
        if not m.any():
            break
        ground = median_filter2d(np.where(m, g, float(np.median(g[m]))), 7)
    print("  地形（平滑后）: %.1f ~ %.1f m" % (ground[valid].min(), ground[valid].max()))

    # ---- ★ AGL 分带（逐格局部地面）----
    k = V.astype(np.int64)
    vk = (k % 16384) - 8192; k //= 16384
    vj = (k % 16384) - 8192; k //= 16384
    vi = (k % 16384) - 8192
    x = (vi + 0.5) * a.voxel; y = (vj + 0.5) * a.voxel; z = (vk + 0.5) * a.voxel
    ri = np.clip((y - H["ymin"]) / a.cell, 0, GH - 1).astype(np.int32)
    rj = np.clip((x - H["xmin"]) / a.cell, 0, GW - 1).astype(np.int32)
    agl = z - ground[ri, rj]
    print("\n=== ★ 离地高度 AGL 分带（决定 UE4 碰撞几何规模）===")
    print("  AGL 分位 1/50/99%%: %.1f / %.1f / %.1f m" % tuple(np.percentile(agl, [1, 50, 99])))
    for b in [float(t) for t in a.bands.split(",")]:
        m = (agl >= -0.5) & (agl <= b)
        print("  地面 0 ~ %5.1f m : %10s 个体素  (%.2f%%)" % (b, format(int(m.sum()), ","), 100.0 * m.sum() / len(V)))
    print("  AGL < -0.5 m    : %10s  (地面下噪点/穿透)" % format(int((agl < -0.5).sum()), ","))
    print("""
  判读:
    · 飞行带（如 0~3m）体素数 = 要做成碰撞几何的方块数量。
      几千 ~ 几万个 → UE4 用 Instanced Static Mesh 完全可行；几十万 → 先合并成大盒子。
    · 点云本身【不能做碰撞体】。要"真飞真撞"必须转成网格/体素几何进 UE4 关卡。""")

    np.save(os.path.join(out, "voxel_keys.npy"), V)
    np.save(os.path.join(out, "ground.npy"), ground)
    np.save(os.path.join(out, "grid_cnt.npy"), gcnt)
    print("\n已存中间结果到 %s（voxel_keys / ground / grid_cnt .npy）" % out)

    if a.ply:
        p = os.path.join(out, "voxels_%gm.ply" % a.voxel)
        lo, hi = np.percentile(z, 1), np.percentile(z, 99)
        t = np.clip((z - lo) / (hi - lo + 1e-9), 0, 1)
        rr = (255 * np.clip(1.5 * t - 0.5, 0, 1)).astype(np.uint8)
        gg = (255 * np.clip(1.5 - abs(3 * t - 1.5), 0, 1)).astype(np.uint8)
        bb = (255 * np.clip(1 - 1.5 * t, 0, 1)).astype(np.uint8)
        with open(p, "wb") as f:
            f.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
                     "property float x\nproperty float y\nproperty float z\n"
                     "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
                     % len(x)).encode())
            rec = np.empty(len(x), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                          ("r", "u1"), ("g", "u1"), ("b", "u1")])
            rec["x"], rec["y"], rec["z"] = x.astype(np.float32), y.astype(np.float32), z.astype(np.float32)
            rec["r"], rec["g"], rec["b"] = rr, gg, bb
            f.write(rec.tobytes())
        print("已写 PLY: %s (%.1f MB)" % (p, os.path.getsize(p) / 1e6))

    print("\n总耗时 %.0fs" % (time.time() - t0))


if __name__ == "__main__":
    main()
