# -*- coding: utf-8 -*-
"""
树干覆盖度自检 —— 圆柱面 (θ,z) 展开法
=====================================
树干是圆柱面 -> 参数化 (θ,z)；某格"已覆盖" ⇔ 格内有回波点。
覆盖率 = 已覆盖格 / (θ格数 × z格数)，缺失的 (θ,z) 直接告诉你"该从哪个方向/哪个高度补扫"。

⚠️ 这是【扫描完备度】指标，不是林分结构指标 —— 回答"树干点云闭合了没有"。

输入 : 任意点云(.las / .pcd) + 树木列表 CSV (x,y,ground_z,height,crown_r[,kind])
输出 : coverage_per_tree.csv / coverage_summary.json / coverage_report.png

用法:
  python trunk_coverage_selfcheck.py --cloud X.las --trees trees.csv --out out_dir
  # 同一工具先跑手持 LAS 得参考上限，再跑无人机自己的 PCD 做对比

性能: 7600 万点约 10 s（cKDTree 最近树查询，比逐树筛点快几个数量级）
依赖: numpy / scipy / matplotlib（不需要 laspy / open3d / ROS）

⚠️ 前置条件：**ground_z 必须是对的**。地面错了 -> 树干 z 窗口挂错高度 ->
   会得到大量"零树干点"的假象（实测 230 棵里 116 棵，修地面后降到 24 棵）。
   跑之前先按 SKILL.md 步骤 2 修地面、并用"点云 z 的 5% 分位"当裁判验一遍。
⚠️ 覆盖率是**上界**：同一 (θ,z) 格里的枝叶点也算覆盖。
   要更严的口径就把邻近判据收紧到"径向 r±40%"并加入射角约束。
⚠️ 矮树（<12 m）的"树干"往往只有十几个点 —— 很可能是灌木/被林下遮挡，
   不是好的补扫目标。建议按树高分层报告。
"""
import argparse
import csv
import json
import os
import struct
import time

import numpy as np


# ---------------- 点云读取（手解二进制，不依赖 laspy） ----------------

def _las_header(fp):
    fp.seek(0)
    h = fp.read(375)
    if h[:4] != b"LASF":
        raise ValueError("不是 LAS 文件")
    return dict(
        n=struct.unpack_from("<I", h, 107)[0],
        fmt=h[104] & 0x3F,
        plen=struct.unpack_from("<H", h, 105)[0],
        scale=struct.unpack_from("<ddd", h, 131),
        off=struct.unpack_from("<ddd", h, 155),
        data_off=struct.unpack_from("<I", h, 96)[0],
    )


def iter_las(path, chunk=2_000_000):
    with open(path, "rb") as f:
        hi = _las_header(f)
        f.seek(hi["data_off"])
        plen, (sx, sy, sz), (ox, oy, oz) = hi["plen"], hi["scale"], hi["off"]
        left, buf = hi["n"], bytearray(hi["plen"] * chunk)
        view = memoryview(buf)
        while left > 0:
            k = min(chunk, left)
            want = plen * k
            got = f.readinto(view[:want])
            if not got or got < want:
                k = got // plen
                if k <= 0:
                    break
            a = np.frombuffer(buf, np.uint8, count=plen * k).reshape(k, plen)
            yield ((a[:, 0:4].copy().view("<i4").ravel() * sx + ox).astype(np.float32),
                   (a[:, 4:8].copy().view("<i4").ravel() * sy + oy).astype(np.float32),
                   (a[:, 8:12].copy().view("<i4").ravel() * sz + oz).astype(np.float32))
            left -= k


def iter_pcd(path, chunk=2_000_000):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline()
            if not line:
                break
            s = line.decode("ascii", "ignore").strip()
            if s and not s.startswith("#"):
                p = s.split()
                hdr[p[0].upper()] = p[1:]
                if p[0].upper() == "DATA":
                    break
        n = int(hdr["POINTS"][0])
        fields = [x.lower() for x in hdr["FIELDS"]]
        sizes = [int(x) for x in hdr["SIZE"]]
        types = [x.upper() for x in hdr["TYPE"]]
        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
        if hdr["DATA"][0].lower() == "ascii":
            arr = np.loadtxt(f, dtype=np.float32, usecols=(ix, iy, iz))
            yield arr[:, 0], arr[:, 1], arr[:, 2]
            return
        dt = np.dtype([(fields[i], types[i].lower() + str(sizes[i])) for i in range(len(fields))])
        left = n
        while left > 0:
            k = min(chunk, left)
            raw = np.frombuffer(f.read(dt.itemsize * k), dtype=dt, count=k)
            yield (raw[fields[ix]].astype(np.float32), raw[fields[iy]].astype(np.float32),
                   raw[fields[iz]].astype(np.float32))
            left -= k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--trees", required=True, help="CSV: x,y,ground_z,height,crown_r[,kind]")
    ap.add_argument("--out", default=None)
    ap.add_argument("--theta-bins", type=int, default=36)
    ap.add_argument("--z-bins", type=int, default=20)
    ap.add_argument("--r-max", type=float, default=1.2, help="树干邻近搜索半径(m)")
    ap.add_argument("--trunk-h-frac", type=float, default=0.40, help="树干高度 = 该比例 × 树高")
    ap.add_argument("--trunk-h-max", type=float, default=12.0)
    ap.add_argument("--min-points-per-cell", type=int, default=1,
                    help="格内至少几点才算已覆盖（1=宽松上界, 3=严格）")
    ap.add_argument("--exclude-kinds", default="big", help="逗号分隔，排除人工添加的树")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    OUT = a.out or os.path.join(os.path.dirname(os.path.abspath(a.cloud)), "_coverage")
    os.makedirs(OUT, exist_ok=True)
    TB, ZB = a.theta_bins, a.z_bins
    excl = {s.strip() for s in a.exclude_kinds.split(",") if s.strip()}

    rows = []
    with open(a.trees, newline="") as f:
        for d in csv.DictReader(f):
            if (d.get("kind") or "normal").strip() in excl:
                continue
            # ★ 只收数字列 —— 把字符串列塞进 float 数组会炸
            rows.append(tuple(float(d[k]) for k in ("x", "y", "ground_z", "height", "crown_r")))
    T = np.array(rows, dtype=np.float64)
    nT = len(T)
    tx, ty, tz, th, tcr = (T[:, i] for i in range(5))
    tz1 = tz + np.minimum(a.trunk_h_frac * th, a.trunk_h_max)

    print("=" * 78)
    print("树干覆盖度自检  %d 棵（已排除 kind ∈ %s）" % (nT, excl or "{}"))
    print("  点云 %s (%.1f MB)   网格 θ%d × z%d = %d 格/树   覆盖门槛 >=%d 点" % (
        a.cloud, os.path.getsize(a.cloud) / 1e6, TB, ZB, TB * ZB, a.min_points_per_cell))
    print("=" * 78)

    from scipy.spatial import cKDTree
    kd = cKDTree(np.column_stack([tx, ty]))
    cov = np.zeros((nT, TB, ZB), dtype=np.int32)
    radhist = np.zeros((nT, 60), dtype=np.int32)      # 0~1.5 m, 0.025 m/格
    npts = np.zeros(nT, dtype=np.int64)

    t0 = time.time()
    it = iter_las(a.cloud) if a.cloud.lower().endswith(".las") else iter_pcd(a.cloud)
    tot = kept = 0
    for cx, cy, cz in it:
        tot += len(cx)
        d, ti = kd.query(np.column_stack([cx, cy]), k=1,
                         distance_upper_bound=a.r_max, workers=-1)
        m = np.isfinite(d)
        if not m.any():
            continue
        ti, px, py, pz = ti[m], cx[m], cy[m], cz[m]
        dx, dy = px - tx[ti], py - ty[ti]
        rxy = np.sqrt(dx * dx + dy * dy)
        zz = pz - tz[ti]
        zmax = (tz1 - tz)[ti]
        ok = (zz >= 0.10) & (zz <= zmax) & (rxy > 0.02)
        if not ok.any():
            continue
        ti, rxy, zz, dx, dy = ti[ok], rxy[ok], zz[ok], dx[ok], dy[ok]
        zmax = zmax[ok]                 # ★ 必须一起 mask，漏了会广播形状错
        kept += len(ti)
        np.add.at(radhist, (ti, np.clip((rxy / 0.025).astype(np.int32), 0, 59)), 1)
        npts += np.bincount(ti, minlength=nT)
        tb = np.clip(((np.arctan2(dy, dx) + np.pi) / (2 * np.pi) * TB).astype(np.int32), 0, TB - 1)
        zb = np.clip((zz / zmax * ZB).astype(np.int32), 0, ZB - 1)
        np.add.at(cov, (ti, tb, zb), 1)
    print("  %s 点 -> 树旁有效 %s (%.1f%%)  用时 %.1f s" % (
        format(tot, ','), format(kept, ','), 100.0 * kept / max(tot, 1), time.time() - t0))

    # 树干半径：径向直方图峰值（0.04~0.90 m），没测到的回落经验值
    lo, hi = int(0.04 / 0.025), int(0.90 / 0.025)
    sub = radhist[:, lo:hi]
    pk, amp = sub.argmax(axis=1), sub.max(axis=1)
    r_est = np.where(amp > 20, (lo + pk + 0.5) * 0.025, np.nan)
    fb = np.isnan(r_est)
    r_est[fb] = np.clip(0.0155 * th[fb], 0.08, 1.1)

    covered = (cov >= a.min_points_per_cell).sum(axis=(1, 2))
    cov_pct = 100.0 * covered / (TB * ZB)
    empty = npts == 0
    per_theta = (cov >= a.min_points_per_cell).any(axis=2)
    miss = []
    for i in range(nT):
        if empty[i]:
            miss.append("全覆盖缺失")
            continue
        seg, j = [], 0
        while j < TB:
            if not per_theta[i, j]:
                k = j
                while k < TB and not per_theta[i, k]:
                    k += 1
                seg.append("%.0f~%.0f°" % (-180 + 360 * j / TB, -180 + 360 * k / TB))
                j = k
            else:
                j += 1
        miss.append(",".join(seg) or "-")

    with open(os.path.join(OUT, "coverage_per_tree.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x", "y", "ground_z", "height", "crown_r", "trunk_r_est",
                    "trunk_z_top", "near_points", "cells_covered", "cells_total",
                    "coverage_pct", "missing_theta_sectors"])
        for i in range(nT):
            w.writerow([i, round(tx[i], 2), round(ty[i], 2), round(tz[i], 2), round(th[i], 2),
                        round(tcr[i], 2), round(r_est[i], 3), round(tz1[i] - tz[i], 2),
                        int(npts[i]), int(covered[i]), TB * ZB, round(cov_pct[i], 1), miss[i]])

    nd = int((~empty).sum())
    summ = dict(cloud=os.path.abspath(a.cloud), trees=a.trees, trees_examined=nT,
                cells_per_tree=TB * ZB,
                trunk_z_rule="ground+0.10 ~ ground+min(%.2f*H, %.1f)" % (a.trunk_h_frac, a.trunk_h_max),
                min_points_per_cell=a.min_points_per_cell,
                points_total=int(tot), points_near_trunks=int(kept),
                trees_with_trunk_points=nd, trees_with_zero_trunk_points=int(empty.sum()),
                coverage_pct_mean=round(float(cov_pct[~empty].mean()), 1) if nd else 0.0,
                coverage_pct_median=round(float(np.median(cov_pct[~empty])), 1) if nd else 0.0,
                trees_ge_95pct=int((cov_pct >= 95).sum()),
                trees_ge_80pct=int((cov_pct >= 80).sum()),
                trunk_r_est_median=round(float(np.nanmedian(r_est)), 3),
                caveat="覆盖率是【相对该输入点云】的上界：同格枝叶点也算覆盖；"
                       "另建议按树高分层报告（矮树可能是灌木，不是补扫目标）")
    json.dump(summ, open(os.path.join(OUT, "coverage_summary.json"), "w"),
              indent=2, ensure_ascii=False)

    print("\n【结果】")
    print("  有树干点的树 %d / %d  （零树干点 %d <- 数量很大时先怀疑 ground_z 错了）" % (
        nd, nT, empty.sum()))
    print("  覆盖率 均值/中位 %.1f%% / %.1f%%    完备树(>=95%%) %d 棵" % (
        summ["coverage_pct_mean"], summ["coverage_pct_median"], summ["trees_ge_95pct"]))
    print("\n  按树高分层（⭐ 关键：矮树往往根本没扫到）")
    for lo_, hi_ in ((0, 6), (6, 12), (12, 20), (20, 40)):
        m = (th >= lo_) & (th < hi_) & (~empty)
        if m.sum():
            print("    %2d~%-2d m : %3d 棵  覆盖率中位 %6.1f%%  树干带点数中位 %8d" % (
                lo_, hi_, int(m.sum()), np.median(cov_pct[m]), int(np.median(npts[m]))))
    print("\n  CSV %s\n  JSON %s" % (
        os.path.join(OUT, "coverage_per_tree.csv"), os.path.join(OUT, "coverage_summary.json")))

    if not a.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize=(15, 8))
            ax = fig.add_subplot(2, 3, (1, 2))
            sc = ax.scatter(tx, ty, c=cov_pct, s=28, cmap="RdYlGn", vmin=0, vmax=100,
                            edgecolors="k", linewidths=0.3)
            ax.set_aspect("equal"); ax.grid(alpha=.3)
            ax.set_title("Trunk coverage (plan view)"); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
            plt.colorbar(sc, ax=ax, label="coverage %")
            ax2 = fig.add_subplot(2, 3, 3)
            ax2.hist(cov_pct[~empty], bins=np.arange(0, 105, 5), color="#4477aa", edgecolor="k")
            ax2.axvline(95, color="r", ls="--"); ax2.set_title("distribution")
            ax2.set_xlabel("coverage %"); ax2.set_ylabel("# trees")
            idxs = np.nonzero(~empty)[0]
            if len(idxs):
                order = np.argsort(cov_pct[~empty])
                pick = [idxs[order[0]], idxs[order[len(order) // 3]],
                        idxs[order[2 * len(order) // 3]], idxs[order[-1]]]
                for k, ti_ in enumerate(pick):
                    ax3 = fig.add_subplot(4, 3, 7 + k)
                    ax3.imshow((cov[ti_] >= a.min_points_per_cell).T, aspect="auto",
                               origin="lower", cmap="gray",
                               extent=[-180, 180, 0, float(tz1[ti_] - tz[ti_])], vmin=0, vmax=1)
                    ax3.set_title("#%d cov=%.0f%% H=%.1fm" % (ti_, cov_pct[ti_], th[ti_]), fontsize=8)
                    ax3.set_xlabel("theta (deg)", fontsize=7); ax3.set_ylabel("z (m)", fontsize=7)
                    ax3.tick_params(labelsize=6)
                fig.suptitle("unwrapped cylinder: white=covered, black=missing", fontsize=11)
            fig.tight_layout()
            p = os.path.join(OUT, "coverage_report.png")
            fig.savefig(p, dpi=110)
            print("  图 %s" % p)
        except Exception as e:
            print("  画图失败(不影响数据): %s" % e)
    print("=" * 78)


if __name__ == "__main__":
    main()
