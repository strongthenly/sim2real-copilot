#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD 点云建图质量分析 (numpy-only, no open3d needed)
用法: python analyze_pcd.py <map.pcd> [out_dir]
依赖: numpy scipy matplotlib
输入: binary PCD, fields 至少含 x y z (可含 intensity/normal/curvature)
输出: 终端统计 + out_dir 下 4 张 PNG (密度图/高程图/地面粗糙图/z直方图)
判读阈值见技能 SKILL.md 的判读表。
"""
import sys, os, time
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python analyze_pcd.py <map.pcd> [out_dir]")
    sys.exit(1)
PCD = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(PCD), "_analysis")
os.makedirs(OUT, exist_ok=True)
t0 = time.time()

# ---- parse header ----
with open(PCD, 'rb') as f:
    fields = []
    while True:
        line = f.readline()
        if not line:
            break
        s = line.decode('ascii', errors='replace').strip()
        parts = s.split()
        if not parts:
            continue
        if parts[0] == 'FIELDS':
            fields = parts[1:]
        elif parts[0] == 'POINTS':
            n_points = int(parts[1])
        elif parts[0] == 'DATA':
            off = f.tell()
            break
print("FIELDS:", fields, "POINTS:", n_points)

dt = np.dtype([(f, '<f4') for f in fields])
raw = np.fromfile(PCD, dtype=dt, offset=off)
print("read %d points in %.1fs" % (raw.shape[0], time.time() - t0))

xyz = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float64)
print("NaN/Inf:", int(np.isnan(xyz).sum()), int(np.isinf(xyz).sum()))
xyz = xyz[np.isfinite(xyz).all(axis=1)]
print("valid points:", xyz.shape[0])

# ---- range ----
print("\n== RANGE ==")
for i, ax in enumerate(['x', 'y', 'z']):
    v = xyz[:, i]
    print("%s: min=%.3f max=%.3f span=%.3f mean=%.3f" % (ax, v.min(), v.max(), v.max() - v.min(), v.mean()))
c = xyz.mean(axis=0)
r = np.linalg.norm(xyz - c, axis=1)
print("center: [%.2f %.2f %.2f]" % tuple(c))
print("dist-to-center: max=%.1f p99=%.1f p99.9=%.1f" % (r.max(), np.percentile(r, 99), np.percentile(r, 99.9)))

# ---- density via KDTree on 200k sample ----
rng = np.random.default_rng(42)
idx = rng.choice(xyz.shape[0], size=200000, replace=False)
samp = xyz[idx]
tree = cKDTree(samp)
d, _ = tree.query(samp, k=2)
nn = d[:, 1]
print("\n== DENSITY (200k sample) ==")
print("NN dist: median=%.2f cm p90=%.2f cm p95=%.2f cm p99=%.2f cm" % tuple(np.percentile(nn, [50, 90, 95, 99]) * 100))
do = np.linalg.norm(samp, axis=1)
bins = [0, 5, 10, 20, 40, 1000]
for i in range(len(bins) - 1):
    m = (do >= bins[i]) & (do < bins[i + 1])
    if m.sum() > 0:
        print("  range %4d-%4dm: n=%6d  median NN=%.2f cm" % (bins[i], bins[i + 1], m.sum(), np.median(nn[m]) * 100))

# ---- z histogram peaks ----
hist, edges = np.histogram(xyz[:, 2], bins=200)
pk = []
for i in range(1, len(hist) - 1):
    if hist[i] > hist[i - 1] and hist[i] > hist[i + 1] and hist[i] > hist.max() * 0.02:
        pk.append(round((edges[i] + edges[i + 1]) / 2, 2))
print("\n== Z PEAKS ==\n", pk[:20])
# NOTE: z 直方图主峰才是真实地面高度。zmin 可能是飞点，别直接用 zmin 定地面带。

# ---- ground band via histogram main peak (not zmin!) ----
z_mode = pk[0] if pk else 0.0
print("\n== GROUND (band around z-mode=%.2f m) ==" % z_mode)
zb = xyz[(xyz[:, 2] >= z_mode - 0.3) & (xyz[:, 2] <= z_mode + 0.3)]
print("ground-band pts: %d (%.2f%%)" % (zb.shape[0], 100.0 * zb.shape[0] / xyz.shape[0]))
if zb.shape[0] > 1000:
    g = zb
    gc = g.mean(axis=0)
    _, _, vh = np.linalg.svd(g - gc)
    nrm = vh[2]
    res = np.abs((g - gc) @ nrm)
    print("plane normal: [%.3f %.3f %.3f] (z-comp %.3f)" % (nrm[0], nrm[1], nrm[2], nrm[2]))
    print("residual: mean=%.2f cm p95=%.2f cm p99=%.2f cm" % (res.mean() * 100, np.percentile(res, 95) * 100, np.percentile(res, 99) * 100))

    # ---- ghost check: 1m cells z-spread (p70-p30) ----
    print("\n== GHOST CHECK (ground band, 1m cells) ==")
    c2 = np.floor((zb[:, :2] - zb[:, :2].min(axis=0)) / 1.0).astype(np.int64)
    kk = c2[:, 0] * 1000000 + c2[:, 1]
    u2 = np.unique(kk)
    order = np.argsort(kk, kind='stable')
    sv = zb[order, 2]
    si = kk[order]
    starts = np.searchsorted(si, u2)
    vals30 = np.full(len(u2), np.nan)
    vals70 = np.full(len(u2), np.nan)
    ncell = 0
    for i in range(len(u2)):
        lo = starts[i]
        hi = starts[i + 1] if i + 1 < len(u2) else len(sv)
        if hi - lo >= 20:
            seg = sv[lo:hi]
            vals30[i], vals70[i] = np.percentile(seg, [30, 70])
            ncell += 1
    sp = vals70 - vals30
    good = np.isfinite(sp)
    print("cells(>=20pts): %d" % ncell)
    print("z-spread: median=%.2f cm p90=%.2f cm p99=%.2f cm" % (np.median(sp[good]) * 100, np.percentile(sp[good], 90) * 100, np.percentile(sp[good], 99) * 100))
    print("cells spread>8cm: %d (%.1f%%)" % ((sp[good] > 0.08).sum(), 100.0 * (sp[good] > 0.08).mean()))

# ---- plots ----
plt.rcParams['figure.facecolor'] = 'white'
fig, ax = plt.subplots(1, 2, figsize=(16, 7))
h, xe, ye = np.histogram2d(xyz[:, 0], xyz[:, 1], bins=700)
im = ax[0].imshow(np.log10(h.T + 1), origin='lower', extent=[xe[0], xe[-1], ye[0], ye[-1]], cmap='viridis', aspect='equal')
ax[0].set_title('XY density (top view, log)'); ax[0].set_xlabel('x (m)'); ax[0].set_ylabel('y (m)')
fig.colorbar(im, ax=ax[0], shrink=0.8, label='log10 count')
h, xe, ye = np.histogram2d(xyz[:, 0], xyz[:, 2], bins=700)
im = ax[1].imshow(np.log10(h.T + 1), origin='lower', extent=[xe[0], xe[-1], ye[0], ye[-1]], cmap='viridis', aspect='equal')
ax[1].set_title('XZ side view (log)'); ax[1].set_xlabel('x (m)'); ax[1].set_ylabel('z (m)')
fig.colorbar(im, ax=ax[1], shrink=0.8, label='log10 count')
plt.tight_layout(); plt.savefig(os.path.join(OUT, '1_density_xy_xz.png'), dpi=110); plt.close()

cs = np.floor((xyz[:, :2] - xyz[:, :2].min(axis=0)) / 0.5).astype(np.int64)
nr = int(cs[:, 0].max()) + 1; nc = int(cs[:, 1].max()) + 1
zsum = np.zeros((nr, nc)); zcnt = np.zeros((nr, nc))
np.add.at(zsum, (cs[:, 0], cs[:, 1]), xyz[:, 2])
np.add.at(zcnt, (cs[:, 0], cs[:, 1]), 1.0)
zmean = np.divide(zsum, zcnt, out=np.full_like(zsum, np.nan), where=zcnt > 0)
fig, ax = plt.subplots(figsize=(10, 9))
im = ax.imshow(zmean, origin='lower', cmap='terrain', aspect='equal',
               extent=[xyz[:, 1].min(), xyz[:, 1].max(), xyz[:, 0].min(), xyz[:, 0].max()])
ax.set_title('Elevation map (mean z per 0.5m cell)'); ax.set_xlabel('y (m)'); ax.set_ylabel('x (m)')
fig.colorbar(im, ax=ax, shrink=0.8, label='z (m)')
plt.tight_layout(); plt.savefig(os.path.join(OUT, '2_elevation.png'), dpi=110); plt.close()

if zb.shape[0] > 1000:
    cg = np.floor((zb[:, :2] - zb[:, :2].min(axis=0)) / 1.0).astype(np.int64)
    nr2 = int(cg[:, 0].max()) + 1; nc2 = int(cg[:, 1].max()) + 1
    gz = np.zeros((nr2, nc2)); gc2 = np.zeros((nr2, nc2)); gz2 = np.zeros((nr2, nc2))
    np.add.at(gz, (cg[:, 0], cg[:, 1]), zb[:, 2])
    np.add.at(gc2, (cg[:, 0], cg[:, 1]), 1.0)
    np.add.at(gz2, (cg[:, 0], cg[:, 1]), zb[:, 2] ** 2)
    meanv = np.divide(gz, gc2, out=np.full_like(gz, np.nan), where=gc2 > 0)
    stdv = np.sqrt(np.divide(gz2, gc2, out=np.zeros_like(gz2), where=gc2 > 0) - meanv ** 2)
    stdv[gc2 < 20] = np.nan
    fig, ax = plt.subplots(figsize=(10, 9))
    im = ax.imshow(stdv, origin='lower', cmap='hot', aspect='equal', vmin=0, vmax=0.3,
                   extent=[zb[:, 1].min(), zb[:, 1].max(), zb[:, 0].min(), zb[:, 0].max()])
    ax.set_title('Ground z-std per 1m cell (red = rough/ghost)'); ax.set_xlabel('y (m)'); ax.set_ylabel('x (m)')
    fig.colorbar(im, ax=ax, shrink=0.8, label='z std (m)')
    plt.tight_layout(); plt.savefig(os.path.join(OUT, '3_ground_std.png'), dpi=110); plt.close()

fig, ax = plt.subplots(figsize=(10, 5))
ax.hist(xyz[:, 2], bins=300)
ax.set_xlabel('z (m)'); ax.set_ylabel('count'); ax.set_title('Z distribution')
plt.tight_layout(); plt.savefig(os.path.join(OUT, '4_z_hist.png'), dpi=110); plt.close()

print("\nDONE in %.1fs -> %s" % (time.time() - t0, OUT))
