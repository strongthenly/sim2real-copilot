#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD 点云建图质量核心分析（纯数值版，无 tkinter/HTML 依赖）
用法: python analyze_pcd.py <file.pcd>
输出: 全部量化指标到 stdout，用于快速验收/对比

完整 HTML 报告版（含 3 图 + 文字分析 + 检查清单）已交付用户桌面:
  C:\Users\<USER>\Desktop\pcd_report.py  /  PCD点云分析.exe
"""
import numpy as np, sys, time
from scipy.spatial import cKDTree
from scipy.signal import find_peaks

def load_pcd(path):
    """解析 PCD header + 读 binary 数据（FIELDS/POINTS/DATA binary 字节偏移）"""
    with open(path, 'rb') as f:
        fields, n_points, data_type = [], None, None
        while True:
            line = f.readline()
            if not line: break
            parts = line.decode('ascii', errors='replace').strip().split()
            if not parts: continue
            if parts[0] == 'FIELDS': fields = parts[1:]
            elif parts[0] == 'POINTS': n_points = int(parts[1])
            elif parts[0] == 'DATA':
                data_type = parts[1]
                off = f.tell()
                break
    if data_type != 'binary':
        raise ValueError('仅支持 binary PCD，当前: %s' % data_type)
    dt = np.dtype([(f, '<f4') for f in fields])
    return np.fromfile(path, dtype=dt, offset=off, count=n_points), fields

def analyze(xyz):
    """xyz: (N,3) float64 → 全部质量指标 dict"""
    N = xyz.shape[0]
    res = {}
    res['n'] = N
    res['nan_inf'] = int(np.isnan(xyz).sum()) + int(np.isinf(xyz).sum())
    r = np.linalg.norm(xyz, axis=1)
    res['dist_max'] = float(r.max())
    res['dist_p99'] = float(np.percentile(r, 99))

    # 密度 + 飞点（采样，KDTree k=2 排除自身）
    rng = np.random.default_rng(42)
    nsamp = min(300000, max(100000, N // 10))
    idx = rng.choice(N, size=nsamp, replace=False)
    samp = xyz[idx]
    d, _ = cKDTree(samp).query(samp, k=2)
    nn = d[:, 1]
    res['nn_med'] = float(np.median(nn))
    res['nn_p90'] = float(np.percentile(nn, 90))
    res['nn_p99'] = float(np.percentile(nn, 99))
    res['flyer_1m'] = float((nn > 1.0).mean())      # <0.5% 干净
    res['flyer_2m'] = float((nn > 2.0).mean())      # ~0.06% 极佳

    # 距离分层（物理衰减对照）
    do = np.linalg.norm(samp, axis=1)
    res['nn_by_range'] = []
    for lo, hi in [(0, 5), (5, 10), (10, 20), (20, 40)]:
        m = (do >= lo) & (do < hi)
        if m.sum() > 0:
            res['nn_by_range'].append((lo, hi, float(np.median(nn[m]))))
    res['near_med'] = res['nn_by_range'][0][2] if res['nn_by_range'] else None

    # Z 直方图峰 + 峰宽
    hist, be = np.histogram(xyz[:, 2], bins=300)
    centers = (be[:-1] + be[1:]) / 2
    peaks, _ = find_peaks(hist, height=hist.max() * 0.02, distance=int(0.5 / (be[1] - be[0])))
    res['peaks'] = []
    for p in peaks:
        half = hist[p] / 2
        left = p
        while left > 0 and hist[left] > half: left -= 1
        right = p
        while right < len(hist) - 1 and hist[right] > half: right += 1
        res['peaks'].append((float(centers[p]), float((centers[right] - centers[left]) * 100)))
    res['peaks'].sort()

    # 垂直重影：最低主峰 ±0.5m 细扫，双峰间隔 5-40cm 且都窄(<20cm 半宽)才算
    res['ghost'] = False
    if res['peaks']:
        main_z = res['peaks'][0][0]
        m = (xyz[:, 2] >= main_z - 0.5) & (xyz[:, 2] <= main_z + 0.5)
        if m.sum() > 500:
            h2, be2 = np.histogram(xyz[m, 2], bins=100)
            c2 = (be2[:-1] + be2[1:]) / 2
            pk2, _ = find_peaks(h2, height=h2.max() * 0.3, distance=5)
            for j in range(len(pk2) - 1):
                gap = c2[pk2[j + 1]] - c2[pk2[j]]
                if 0.05 <= gap <= 0.4:
                    w = []
                    for k in (pk2[j], pk2[j + 1]):
                        hpk = h2[k]; left = k
                        while left > 0 and h2[left] > hpk / 2: left -= 1
                        right = k
                        while right < len(h2) - 1 and h2[right] > hpk / 2: right += 1
                        w.append(c2[right] - c2[left])
                    if w[0] < 0.2 and w[1] < 0.2:
                        res['ghost'] = True
                        break

    # 覆盖：主体范围（5-95 分位）1m 网格占用率
    xl, xh = np.percentile(xyz[:, 0], [5, 95])
    yl, yh = np.percentile(xyz[:, 1], [5, 95])
    main = (xyz[:, 0] >= xl) & (xyz[:, 0] <= xh) & (xyz[:, 1] >= yl) & (xyz[:, 1] <= yh)
    cg = np.floor((xyz[main, :2] - np.array([xl, yl])) / 1.0).astype(np.int64)
    occ = len(np.unique(cg[:, 0] * 1000000 + cg[:, 1]))
    total = int((xh - xl) + 1) * int((yh - yl) + 1)
    res['coverage'] = 100.0 * occ / total
    return res

def main():
    t0 = time.time()
    raw, fields = load_pcd(sys.argv[1])
    xyz = np.column_stack([raw[f].astype(np.float64) for f in ['x', 'y', 'z']])
    res = analyze(xyz)
    print('== PCD 质量指标 == %s' % sys.argv[1])
    print('点数: %d  NaN/Inf: %d' % (res['n'], res['nan_inf']))
    print('范围: 到中心 max=%.1fm p99=%.1fm' % (res['dist_max'], res['dist_p99']))
    print('密度(最近邻): med=%.2fcm p90=%.2fcm p99=%.2fcm' % (
        res['nn_med'] * 100, res['nn_p90'] * 100, res['nn_p99'] * 100))
    print('飞点: >1m=%.2f%%  >2m=%.2f%%' % (res['flyer_1m'] * 100, res['flyer_2m'] * 100))
    print('距离衰减:', '  '.join('%d-%dm:%.1fcm' % (lo, hi, v * 100) for lo, hi, v in res['nn_by_range']))
    print('Z 峰(m/半宽cm):', res['peaks'])
    print('垂直重影: %s' % ('检测到!' if res['ghost'] else '无'))
    print('覆盖占用率: %.0f%%' % res['coverage'])
    print('耗时: %.1fs' % (time.time() - t0))

if __name__ == '__main__':
    main()
