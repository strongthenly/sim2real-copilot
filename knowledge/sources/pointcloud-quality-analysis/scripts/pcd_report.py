#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD 点云建图质量分析报告工具 (v1.0)
====================================
用法:
  1) 双击本文件 -> 弹窗选择 .pcd 文件 -> 自动生成 HTML 报告并打开
  2) 命令行:  python pcd_report.py <文件路径>
  3) 打包 exe: python -m PyInstaller --onefile --windowed --name PCD点云分析 --clean --noconfirm pcd_report.py

输出: 与所选 PCD 同目录下的 <文件名>_report.html (自包含, 图全部内嵌, 双击即可用浏览器打开)

依赖: numpy scipy matplotlib  (本机 Python 3.12 已装)
注意: numpy>=2.x 用 np.rec.fromarrays (np.core 已移除)
"""

import numpy as np
import os, sys, io, base64, time, webbrowser
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.signal import find_peaks

try:
    import tkinter as tk
    from tkinter import filedialog
    HAS_TK = True
except Exception:
    HAS_TK = False

T_START = time.time()

# ============ 1. 文件读取 ============
def load_pcd(path):
    with open(path, 'rb') as f:
        fields, n_points, data_type, offset, n_hdr = [], None, None, None, 0
        while True:
            line = f.readline()
            n_hdr += 1
            if not line: break
            s = line.decode('ascii', errors='replace').strip()
            parts = s.split()
            if not parts: continue
            if parts[0] == 'FIELDS': fields = parts[1:]
            elif parts[0] == 'POINTS': n_points = int(parts[1])
            elif parts[0] == 'DATA':
                data_type = parts[1]
                offset = f.tell()
                break
    if data_type is None:
        raise ValueError('无法解析 PCD 头部, 请确认是标准 PCD 文件')
    if data_type == 'binary':
        dt = np.dtype([(f, '<f4') for f in fields])
        raw = np.fromfile(path, dtype=dt, offset=offset, count=n_points)
    elif data_type == 'ascii':
        raw = np.genfromtxt(path, skip_header=n_hdr, dtype=np.float32)
        raw = raw[~np.isnan(raw).any(axis=1)]
        if raw.ndim == 1: raw = raw.reshape(1, -1)
        raw = np.rec.fromarrays([raw[:, i] for i in range(len(fields))], names=fields)
    else:
        raise ValueError('不支持格式 "%s"。binary_compressed 请先用 CloudCompare/PCL 转成 binary' % data_type)
    if raw.shape[0] == 0:
        raise ValueError('文件读取结果为空, 可能格式不符')
    return raw, fields, n_points, data_type

# ============ 2. 分析 ============
def analyze(xyz, n_total):
    res = {}
    N = xyz.shape[0]
    res['n'] = N
    res['nan_inf'] = int(np.isnan(xyz).sum()) + int(np.isinf(xyz).sum())

    # 范围
    res['range'] = {}
    for i, ax in enumerate(['x', 'y', 'z']):
        v = xyz[:, i]
        res['range'][ax] = dict(min=float(v.min()), max=float(v.max()),
                                span=float(v.max() - v.min()), mean=float(v.mean()))
    r = np.linalg.norm(xyz, axis=1)
    res['dist'] = dict(max=float(r.max()), p99=float(np.percentile(r, 99)))

    # 密度 (采样)
    rng = np.random.default_rng(42)
    nsamp = min(300000, max(100000, N // 10))
    idx = rng.choice(N, size=nsamp, replace=False)
    samp = xyz[idx]
    tree = cKDTree(samp)
    d, _ = tree.query(samp, k=2)
    nn = d[:, 1]
    res['nn_all'] = dict(med=float(np.median(nn)), p90=float(np.percentile(nn, 90)), p99=float(np.percentile(nn, 99)))
    res['flyer_05'] = float((nn > 0.5).mean())
    res['flyer_10'] = float((nn > 1.0).mean())
    res['flyer_20'] = float((nn > 2.0).mean())

    do = np.linalg.norm(samp, axis=1)
    bins = [0, 5, 10, 20, 40, 1e9]
    res['nn_by_range'] = []
    for i in range(len(bins) - 1):
        m = (do >= bins[i]) & (do < bins[i + 1])
        if m.sum() > 0:
            res['nn_by_range'].append(dict(lo=bins[i], hi=int(min(bins[i+1], 999)),
                                           n=int(m.sum()), med=float(np.median(nn[m]))))
    # 近处密度(0-5m)
    near = [b for b in res['nn_by_range'] if b['lo'] == 0]
    res['near_med'] = near[0]['med'] if near else None

    # z 分层
    edges = [-100, -5, 0, 5, 10, 20, 30, 50, 100]
    res['layers'] = []
    for i in range(len(edges) - 1):
        m = (xyz[:, 2] >= edges[i]) & (xyz[:, 2] < edges[i + 1])
        res['layers'].append(dict(lo=edges[i], hi=edges[i + 1], n=int(m.sum()),
                                  pct=100.0 * m.sum() / N))

    # z 直方图峰
    hbins = 300
    hist, be = np.histogram(xyz[:, 2], bins=hbins)
    centers = (be[:-1] + be[1:]) / 2
    bw = be[1] - be[0]
    peaks, _ = find_peaks(hist, height=hist.max() * 0.02, distance=int(0.5 / bw))
    res['peaks'] = []
    for p in peaks:
        half = hist[p] / 2
        left = p
        while left > 0 and hist[left] > half: left -= 1
        right = p
        while right < len(hist) - 1 and hist[right] > half: right += 1
        res['peaks'].append(dict(z=float(centers[p]), width=float((centers[right] - centers[left]) * 100)))
    res['peaks'].sort(key=lambda k: k['z'])

    # 垂直重影检测: 最低主峰 ±0.5m 细扫 (1cm bin)
    # 只有"地面层自身分裂成两个近距窄峰"才算重影, 避免把街道的车/路沿误判
    ghost = False
    if res['peaks']:
        main_z = res['peaks'][0]['z']
        m = (xyz[:, 2] >= main_z - 0.5) & (xyz[:, 2] <= main_z + 0.5)
        if m.sum() > 500:
            h2, be2 = np.histogram(xyz[m, 2], bins=100)
            c2 = (be2[:-1] + be2[1:]) / 2
            pk2, _ = find_peaks(h2, height=h2.max() * 0.3, distance=5)
            if len(pk2) >= 2:
                # 检查相邻峰对: 间隔 5~40cm 且两者都窄(<20cm 半宽)才是重影特征
                for j in range(len(pk2) - 1):
                    gap = c2[pk2[j + 1]] - c2[pk2[j]]
                    if 0.05 <= gap <= 0.4:
                        hpk = h2[pk2[j]]
                        left = pk2[j]
                        while left > 0 and h2[left] > hpk / 2: left -= 1
                        right = pk2[j]
                        while right < len(h2) - 1 and h2[right] > hpk / 2: right += 1
                        w1 = (c2[right] - c2[left])
                        hpk2 = h2[pk2[j + 1]]
                        left2 = pk2[j + 1]
                        while left2 > 0 and h2[left2] > hpk2 / 2: left2 -= 1
                        right2 = pk2[j + 1]
                        while right2 < len(h2) - 1 and h2[right2] > hpk2 / 2: right2 += 1
                        w2 = (c2[right2] - c2[left2])
                        if w1 < 0.2 and w2 < 0.2:
                            ghost = True
                            break
    res['ghost'] = ghost

    # 覆盖: 主体范围 (5-95 分位) 1m 网格占用率
    xl, xh = np.percentile(xyz[:, 0], [5, 95])
    yl, yh = np.percentile(xyz[:, 1], [5, 95])
    main = (xyz[:, 0] >= xl) & (xyz[:, 0] <= xh) & (xyz[:, 1] >= yl) & (xyz[:, 1] <= yh)
    cg = np.floor((xyz[main, :2] - np.array([xl, yl])) / 1.0).astype(np.int64)
    kk = cg[:, 0] * 1000000 + cg[:, 1]
    occ = len(np.unique(kk))
    total_cells = int((xh - xl) / 1.0 + 1) * int((yh - yl) / 1.0 + 1)
    res['coverage'] = 100.0 * occ / total_cells
    return res

# ============ 3. 绘图 (返回 base64) ============
def fig2b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode('ascii')

def make_plots(xyz, res):
    imgs = {}
    # 图1: XY + XZ 密度
    fig, ax = plt.subplots(1, 2, figsize=(15, 6.5))
    h, xe, ye = np.histogram2d(xyz[:, 0], xyz[:, 1], bins=600)
    im = ax[0].imshow(np.log10(h.T + 1), origin='lower', extent=[xe[0], xe[-1], ye[0], ye[-1]],
                      cmap='viridis', aspect='equal')
    ax[0].set_title('Top view XY density (log)'); ax[0].set_xlabel('x (m)'); ax[0].set_ylabel('y (m)')
    fig.colorbar(im, ax=ax[0], shrink=0.85, label='log10 count')
    h, xe, ye = np.histogram2d(xyz[:, 0], xyz[:, 2], bins=600)
    im = ax[1].imshow(np.log10(h.T + 1), origin='lower', extent=[xe[0], xe[-1], ye[0], ye[-1]],
                      cmap='viridis', aspect='equal')
    ax[1].set_title('Side view XZ density (log)'); ax[1].set_xlabel('x (m)'); ax[1].set_ylabel('z (m)')
    fig.colorbar(im, ax=ax[1], shrink=0.85, label='log10 count')
    fig.tight_layout()
    imgs['density'] = fig2b64(fig)

    # 图2: 高程图
    cs = np.floor((xyz[:, :2] - xyz[:, :2].min(axis=0)) / 0.5).astype(np.int64)
    nr = int(cs[:, 0].max()) + 1; nc = int(cs[:, 1].max()) + 1
    zsum = np.zeros((nr, nc)); zcnt = np.zeros((nr, nc))
    np.add.at(zsum, (cs[:, 0], cs[:, 1]), xyz[:, 2])
    np.add.at(zcnt, (cs[:, 0], cs[:, 1]), 1.0)
    zmean = np.divide(zsum, zcnt, out=np.full_like(zsum, np.nan), where=zcnt > 0)
    zm_lo, zm_hi = np.nanpercentile(zmean, [1, 99])
    fig, ax = plt.subplots(figsize=(10.5, 9))
    im = ax.imshow(zmean, origin='lower', cmap='terrain', aspect='equal', vmin=zm_lo, vmax=zm_hi,
                   extent=[xyz[:, 1].min(), xyz[:, 1].max(), xyz[:, 0].min(), xyz[:, 0].max()])
    ax.set_title('Elevation map (mean z per 0.5m cell, clipped 1-99%)')
    ax.set_xlabel('y (m)'); ax.set_ylabel('x (m)')
    fig.colorbar(im, ax=ax, shrink=0.85, label='z (m)')
    fig.tight_layout()
    imgs['elev'] = fig2b64(fig)

    # 图3: Z 直方图标注
    fig, ax = plt.subplots(figsize=(13, 5.5))
    n, bins, _ = ax.hist(xyz[:, 2], bins=300, color='#4C72B0', alpha=0.9)
    centers = (bins[:-1] + bins[1:]) / 2
    peaks, _ = find_peaks(n, height=n.max() * 0.02, distance=int(0.5 / (bins[1] - bins[0])))
    for p in peaks:
        ax.axvline(centers[p], color='red', ls='--', lw=0.8, alpha=0.7)
        ax.text(centers[p], n[p] + n.max() * 0.01, '%.1fm' % centers[p], ha='center',
                fontsize=9, color='red', fontweight='bold')
    ax.set_xlabel('z height (m)'); ax.set_ylabel('point count')
    ax.set_title('Z distribution with annotated peaks (each peak = one layer)')
    fig.tight_layout()
    imgs['zhist'] = fig2b64(fig)
    return imgs

# ============ 4. 判定 ============
def judge(res):
    checks = []
    def add(name, level, detail):
        checks.append(dict(name=name, level=level, detail=detail))
    # 数据完整性
    if res['nan_inf'] > 0:
        add('数据完整性', 'FAIL', '发现 %d 个 NaN/Inf 点' % res['nan_inf'])
    elif res['n'] >= 1000000:
        add('数据完整性', 'PASS', '%.1f 万点, 无 NaN/Inf' % (res['n'] / 10000.0))
    elif res['n'] >= 200000:
        add('数据完整性', 'WARN', '点数偏少: %d (建议 >100万)' % res['n'])
    else:
        add('数据完整性', 'FAIL', '点数过少: %d' % res['n'])
    # 飞点
    fp = res['flyer_10'] * 100
    if fp < 0.5: add('飞点/孤立点', 'PASS', '最近邻>1m 仅 %.2f%%' % fp)
    elif fp < 2: add('飞点/孤立点', 'WARN', '孤立点 %.2f%% (建议<0.5%%)' % fp)
    else: add('飞点/孤立点', 'FAIL', '孤立点 %.2f%% 过多, 噪点明显' % fp)
    # 近处密度
    if res['near_med'] is not None:
        nm = res['near_med'] * 100
        if nm < 2: add('近处密度(0-5m)', 'PASS', '最近邻中位 %.1fcm (优秀)' % nm)
        elif nm < 5: add('近处密度(0-5m)', 'PASS', '最近邻中位 %.1fcm' % nm)
        elif nm < 10: add('近处密度(0-5m)', 'WARN', '最近邻中位 %.1fcm 偏稀' % nm)
        else: add('近处密度(0-5m)', 'FAIL', '最近邻中位 %.1fcm 过稀' % nm)
    # 覆盖
    if res['coverage'] > 50: add('平面覆盖(1m网格)', 'PASS', '占用率 %.0f%%' % res['coverage'])
    elif res['coverage'] > 30: add('平面覆盖(1m网格)', 'WARN', '占用率 %.0f%% 有空洞' % res['coverage'])
    else: add('平面覆盖(1m网格)', 'FAIL', '占用率 %.0f%% 覆盖严重不足' % res['coverage'])
    # 垂直重影
    if res['ghost']: add('垂直重影', 'FAIL', '最低层附近检测到双峰 (z 方向重影)')
    else: add('垂直重影', 'PASS', '未检测到双峰')
    # 地面层宽度
    if res['peaks']:
        w = res['peaks'][0]['width']
        if w < 25: add('地面层厚度(主峰)', 'PASS', '主峰半宽 %.0fcm, 平整' % w)
        elif w < 60: add('地面层厚度(主峰)', 'WARN', '主峰半宽 %.0fcm, 略厚' % w)
        else: add('地面层厚度(主峰)', 'WARN', '主峰半宽 %.0fcm, 偏厚(可能混入车/灌木或漂移)' % w)
    # 总体
    n_fail = sum(1 for c in checks if c['level'] == 'FAIL')
    n_warn = sum(1 for c in checks if c['level'] == 'WARN')
    if n_fail: overall = ('FAIL', '不合格 - %d 项严重问题' % n_fail)
    elif n_warn: overall = ('WARN', '基本合格 - %d 项需注意' % n_warn)
    else: overall = ('PASS', '合格 - 各项指标正常')
    return checks, overall

# ============ 4.5 文字分析 ============
def generate_narrative(res, checks):
    P = []
    n = res['n']
    P.append('本文件共 %d 个点。' % n)
    if res['nan_inf'] > 0:
        P.append('<b>数据完整性:</b> 存在 %d 个无效点(NaN/Inf)。这类点会在后续配准/测量中产生错误, 建议先剔除。' % res['nan_inf'])
    else:
        P.append('<b>数据完整性:</b> 全部坐标有效, 无 NaN/Inf, 文件结构健康。')
    nm = res['near_med']
    if nm is not None:
        cm = nm * 100
        if cm < 2:
            P.append('<b>密度:</b> 近处(0-5m)点间距中位 %.1fcm, 达到雷达物理极限, 近景细节充分。' % cm)
        elif cm < 5:
            P.append('<b>密度:</b> 近处点间距中位 %.1fcm, 密度良好。' % cm)
        elif cm < 10:
            P.append('<b>密度:</b> 近处点间距中位 %.1fcm, 偏稀。若采集时移动较快, 建议放慢速度或对重点区域补扫。' % cm)
        else:
            P.append('<b>密度:</b> 近处点间距中位 %.1fcm, 明显过稀, 覆盖严重不足, 建议重新采集。' % cm)
    if res['nn_by_range']:
        trend = ' -> '.join('%.1fcm' % (b['med'] * 100) for b in res['nn_by_range'])
        P.append('<b>距离衰减:</b> 点间距随距离变化: %s。越远越稀是雷达角分辨率的物理规律; 若远区骤减, 可能因扫描过快或环境反射弱。' % trend)
    fp = res['flyer_10'] * 100
    if fp < 0.5:
        P.append('<b>飞点:</b> 孤立点占比 %.2f%%, 非常干净, 无明显噪点或动态物体污染。' % fp)
    elif fp < 2:
        P.append('<b>飞点:</b> 孤立点占比 %.2f%%, 少量噪声, 可能来自行人/车辆等动态物体或玻璃反光, 可用半径滤波剔除。' % fp)
    else:
        P.append('<b>飞点:</b> 孤立点占比 %.2f%%, 噪声明显, 强烈建议滤波, 并检查是否有动态物体干扰。' % fp)
    cov = res['coverage']
    if cov > 50:
        P.append('<b>覆盖:</b> 主体区域 1m 网格占用率 %.0f%%, 覆盖完整, 无明显漏扫空洞。' % cov)
    elif cov > 30:
        P.append('<b>覆盖:</b> 主体区域网格占用率 %.0f%%, 存在部分空洞, 建议回看密度图确认空洞位置并补扫。' % cov)
    else:
        P.append('<b>覆盖:</b> 网格占用率仅 %.0f%%, 覆盖严重不足, 存在大面积空白, 不建议用于测量/定位。' % cov)
    if res['peaks']:
        w = res['peaks'][0]['width']
        if w < 25:
            P.append('<b>垂直结构:</b> 主层(地面)峰半宽 %.0fcm, 层位稳定, 无垂直漂移迹象。' % w)
        else:
            P.append('<b>垂直结构:</b> 主层峰半宽 %.0fcm, 偏厚。室外场景可能混入车辆/灌木; 室内则提示 z 方向不稳定。' % w)
    if res['ghost']:
        P.append('<b style="color:#cf222e">垂直重影:</b> 主层附近检测到双峰, 同一平面被建到两个高度(z 方向漂移)。常见原因: 长走廊退化、IMU 初始化不充分、回环未闭合。建议重新采集, 启动后静止 3-5 秒。')
    else:
        P.append('<b>垂直重影:</b> 未检测到双峰, 无 z 方向分层迹象。')
    fails = [c for c in checks if c['level'] == 'FAIL']
    warns = [c for c in checks if c['level'] == 'WARN']
    if fails:
        P.append('<b>改进建议:</b> 存在 %d 项严重问题(%s), 建议优先排查数据源与采集方式, 必要时重新采集。' % (len(fails), '、'.join(c['name'] for c in fails)))
    elif warns:
        P.append('<b>改进建议:</b> 存在 %d 项需注意(%s), 可按上文对应方向优化。' % (len(warns), '、'.join(c['name'] for c in warns)))
    else:
        P.append('<b>改进建议:</b> 各项指标正常, 此图可直接用于后续处理。')
    return ''.join('<p style="margin:6px 0;line-height:1.9">%s</p>' % t for t in P)

# ============ 5. HTML 报告 ============
def html_report(path, fields, n_total, data_type, res, imgs, checks, overall, elapsed, narrative):
    base = os.path.basename(path)
    rows_range = ''.join(
        '<tr><td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td></tr>' %
        (ax, res['range'][ax]['min'], res['range'][ax]['max'], res['range'][ax]['span'], res['range'][ax]['mean'])
        for ax in ['x', 'y', 'z'])
    rows_layers = ''.join(
        '<tr><td>%4.0f ~ %4.0f m</td><td>%d</td><td>%.1f%%</td></tr>' %
        (L['lo'], L['hi'], L['n'], L['pct']) for L in res['layers'] if L['n'] > 0)
    rows_peaks = ''.join(
        '<tr><td>%.2f m</td><td>%.0f cm</td></tr>' % (P['z'], P['width']) for P in res['peaks'])
    rows_nn = ''.join(
        '<tr><td>%s</td><td>%s</td><td>%.2f cm</td></tr>' %
        ('%d~%dm' % (b['lo'], b['hi']), b['n'], b['med'] * 100) for b in res['nn_by_range'])
    chk_html = ''
    for c in checks:
        if c['level'] == 'PASS': icon, color = '&#10003; 通过', '#1a7f37'
        elif c['level'] == 'WARN': icon, color = '&#9888; 注意', '#b08800'
        else: icon, color = '&#10007; 失败', '#cf222e'
        chk_html += '<tr><td style="color:%s;font-weight:bold">%s</td><td>%s</td><td>%s</td></tr>' % (
            color, icon, c['name'], c['detail'])
    ov_color = {'PASS': '#1a7f37', 'WARN': '#b08800', 'FAIL': '#cf222e'}[overall[0]]
    ov_bg = {'PASS': '#e6f4ea', 'WARN': '#fff8e1', 'FAIL': '#fdecea'}[overall[0]]

    html = '''<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<title>PCD 建图质量报告 - %s</title>
<style>
body{font-family:"Microsoft YaHei",sans-serif;margin:24px auto;max-width:1100px;color:#222;background:#fafafa}
h1{font-size:22px}h2{font-size:17px;border-left:4px solid #4C72B0;padding-left:8px;margin-top:28px}
.banner{border-radius:8px;padding:14px 18px;font-size:18px;font-weight:bold;background:%s;color:%s;margin:16px 0}
.meta{color:#666;font-size:13px;line-height:1.7}
table{border-collapse:collapse;width:100%%;background:#fff;font-size:13px;margin:8px 0}
th,td{border:1px solid #ddd;padding:6px 10px;text-align:left}
th{background:#f0f4f8}
img{max-width:100%%;border:1px solid #ccc;border-radius:6px;margin:6px 0}
.caption{color:#555;font-size:13px;margin-bottom:10px}
.warnbox{background:#fff8e1;border-left:4px solid #b08800;padding:10px 14px;font-size:13px;border-radius:4px;margin:10px 0}
.footer{color:#999;font-size:12px;margin-top:30px;border-top:1px solid #ddd;padding-top:8px}
</style></head><body>
<h1>PCD 建图质量分析报告</h1>
<div class="meta">
文件: <b>%s</b><br>
字段: %s | 格式: %s | 声明点数: %s<br>
有效点: %s 个 | 其中无效点(NaN/Inf): %s 个 | 分析耗时: %.1f 秒
</div>
<div class="banner">总体判定: %s</div>
<h2>文字分析</h2>
%s
<h2>检查清单</h2>
<table><tr><th>结果</th><th>项目</th><th>说明</th></tr>%s</table>
<h2>关键指标</h2>
<table>
<tr><th>指标</th><th>数值</th><th>说明</th></tr>
<tr><td>到中心最远距离</td><td>%.1f m (p99: %.1f m)</td><td>超出雷达量程过多说明可能有飞点</td></tr>
<tr><td>整体密度(最近邻中位)</td><td>%.2f cm</td><td>所有距离混合</td></tr>
<tr><td>最近邻 p90 / p99</td><td>%.2f cm / %.2f cm</td><td>尾部越厚说明稀疏区域越多</td></tr>
<tr><td>平面覆盖占用率</td><td>%.0f%%</td><td>主体范围 1m 网格有点比例</td></tr>
</table>
<h2>空间范围</h2>
<table><tr><th>轴</th><th>min (m)</th><th>max (m)</th><th>跨度 (m)</th><th>均值 (m)</th></tr>%s</table>
<h2>密度 vs 距离 (物理衰减对照)</h2>
<table><tr><th>到原点距离</th><th>采样点数</th><th>最近邻中位</th></tr>%s</table>
<h2>高度分层</h2>
<table><tr><th>z 区间</th><th>点数</th><th>占比</th></tr>%s</table>
<h2>高度层峰 (Z 直方图)</h2>
<table><tr><th>峰位置</th><th>半宽</th></tr>%s</table>
%s
<h2>图 1: 密度图 (覆盖)</h2>
<img src="data:image/png;base64,%s">
<div class="caption">左: 俯视 XY 密度(亮=点多) 右: 侧视 XZ(看楼高/地面)。看轮廓是否完整、有无意外空洞、有无拉丝。</div>
<h2>图 2: 高程图 (全局一致性)</h2>
<img src="data:image/png;base64,%s">
<div class="caption">颜色=平均高度。街道应连续同色, 楼应色块清晰; 颜色撕裂/马赛克=漂移; 楼街交界糊=墙斜/重影。</div>
<h2>图 3: Z 直方图 (垂直结构)</h2>
<img src="data:image/png;base64,%s">
<div class="caption">一个峰=一个层。峰窄且唯一=该层平整; 两个相邻窄峰(间隔10~30cm)=垂直重影实锤。</div>
<h2>如何解读</h2>
<div class="warnbox"><b>三张图分工:</b> 密度图=扫没扫全(覆盖), 高程图=对不对得上(全局一致), 直方图=高度层正不正常(垂直结构)。三张全过=图基本可信; 任一不过, 按对应方向排查。</div>
<div class="footer">由 pcd_report.py v1.0 生成 | 生成时间 %s</div>
</body></html>''' % (
        base, ov_bg, ov_color, base, ', '.join(fields), data_type, n_total,
        res['n'], res['nan_inf'], elapsed, overall[1], narrative, chk_html,
        res['dist']['max'], res['dist']['p99'], res['nn_all']['med'] * 100,
        res['nn_all']['p90'] * 100, res['nn_all']['p99'] * 100, res['coverage'],
        rows_range, rows_nn, rows_layers, rows_peaks,
        ('<div class="warnbox"><b>&#9888; 检测到垂直重影:</b> 最低层附近存在双峰, z 方向可能有分层漂移, 建议在 rviz 中确认地面是否双层。</div>'
         if res['ghost'] else ''),
        imgs['density'], imgs['elev'], imgs['zhist'], time.strftime('%Y-%m-%d %H:%M:%S'))
    out = os.path.join(os.path.dirname(path), os.path.splitext(base)[0] + '_report.html')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(html)
    return out

# ============ 6. 主流程 ============
def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    elif HAS_TK:
        root = tk.Tk(); root.withdraw(); root.attributes('-topmost', True)
        path = filedialog.askopenfilename(title='选择 PCD 点云文件',
                                          filetypes=[('PCD 文件', '*.pcd'), ('所有文件', '*.*')])
        root.destroy()
        if not path:
            print('未选择文件, 退出'); return
    else:
        print('用法: python pcd_report.py <文件.pcd>'); return

    if not os.path.exists(path):
        print('文件不存在: %s' % path); return
    print('>> 读取 %s ...' % os.path.basename(path))
    raw, fields, n_total, data_type = load_pcd(path)
    xyz = np.column_stack([raw[f].astype(np.float64) for f in ['x', 'y', 'z']])
    print('>> 分析中 (%d 点) ...' % xyz.shape[0])
    res = analyze(xyz, n_total)
    print('>> 绘图 ...')
    imgs = make_plots(xyz, res)
    checks, overall = judge(res)
    elapsed = time.time() - T_START
    narrative = generate_narrative(res, checks)
    out = html_report(path, fields, n_total, data_type, res, imgs, checks, overall, elapsed, narrative)
    print('>> 报告已生成: %s' % out)
    print('>> 总体判定: %s' % overall[1])
    try:
        webbrowser.open('file:///' + out.replace(chr(92), '/'))
    except Exception:
        pass

if __name__ == '__main__':
    main()
