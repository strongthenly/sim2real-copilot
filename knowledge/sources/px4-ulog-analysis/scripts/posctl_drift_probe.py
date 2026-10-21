#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
POSCTL 定点偏移探针 (px4-ulog-analysis 配套, 2026-08-18 实机校准)
用法: python posctl_drift_probe.py <log.ulg>
输出: 模式时间线 / 三航向源对拍 / 电机分配 / 振动分段RMS / EKF reset+桨不平衡 / 摇杆

判定速览:
  - flag_control_position_enabled 全程=0 -> 不是定点模式(自稳/手动), 偏移性质另论
  - 视觉yaw独跳、IMU姿态yaw稳定 -> 视觉估计漂移
  - 三源同步大幅旋转 + 姿态环打满(tq_z>0.4, yaw速率指令与实测反向) + 电机对角饱和(M3/M4->0)
    + 振动RMS分段放大 + imbalanced_prop全程非零 -> 机械故障(桨/电机), 视觉yaw漂只是症状
依赖: pip install pyulog numpy
"""
import sys
import numpy as np
from pyulog import ULog


def mask_of(d):
    ts = d.data['timestamp'].astype(np.int64)
    return (ts > 0) & (ts < 1e15)


def quat_yaw(q):
    w, x, y, z = q
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def seg_stats(t, vals, t0, seg=2.0, tmax=20.0):
    print(f"  {'段(s)':>8} {'rms':>8} {'max':>8}")
    for s in np.arange(0, tmax, seg):
        sel = (t >= (s * 1e6 + t0)) & (t < ((s + seg) * 1e6 + t0))
        if sel.sum() > 5:
            print(f"  {s:7.0f}-{s+seg:3.0f} {vals[sel].std():8.2f} {np.abs(vals[sel]).max():8.2f}")


def main(path):
    ulog = ULog(path, disable_str_exceptions=True)
    data = {d.name: d for d in ulog.data_list}

    vcm = data.get('vehicle_control_mode')
    t0 = 0
    if vcm:
        m = mask_of(vcm)
        t0 = int(vcm.data['timestamp'][m].astype(np.int64).min())

    print("=" * 74)
    print("1) 模式时间线 (vehicle_control_mode 变化点)")
    if vcm:
        m = mask_of(vcm)
        t = vcm.data['timestamp'][m].astype(np.int64)
        armed = vcm.data['flag_armed'][m]
        man = vcm.data['flag_control_manual_enabled'][m]
        pos = vcm.data['flag_control_position_enabled'][m]
        vel = vcm.data['flag_control_velocity_enabled'][m]
        alt = vcm.data['flag_control_altitude_enabled'][m]
        prev = None
        for i in range(len(t)):
            key = (armed[i], man[i], alt[i], vel[i], pos[i])
            if key != prev:
                print(f"  t={(t[i]-t0)/1e6:8.1f}s armed={armed[i]} manual={man[i]} alt={alt[i]} vel={vel[i]} pos={pos[i]}")
                prev = key
        if not np.any(pos):
            print("  !! flag_control_position_enabled 全程=0 -> 不是定点模式(自稳/手动), 先确认模式再定性")

    print("=" * 74)
    print("2) 三航向源对拍 (实际姿态 vs 视觉 vs EKF heading), 每~1.5s")
    att = data.get('vehicle_attitude')
    vo = data.get('estimator_visual_odometry_aligned')
    vlp = data.get('vehicle_local_position')
    if att and vo and vlp:
        m1 = mask_of(att); ta = att.data['timestamp'][m1].astype(np.int64)
        m2 = mask_of(vo); tv = vo.data['timestamp'][m2].astype(np.int64)
        m3 = mask_of(vlp); tl = vlp.data['timestamp'][m3].astype(np.int64)
        att_yaw = np.array([quat_yaw([att.data['q[%d]' % j][m1][i] for j in range(4)]) for i in range(len(ta))])
        vis_yaw = np.array([quat_yaw([vo.data['q[%d]' % j][m2][i] for j in range(4)]) for i in range(len(tv))])
        ekf_hdg = np.degrees(vlp.data['heading'][m3])
        print(f"  {'t(s)':>8} {'att_yaw':>8} {'vis_yaw':>8} {'ekf_hdg':>8}")
        tt = np.arange(0, (tl[-1] - t0) / 1e6, 1.5) * 1e6 + t0
        for ts_ in tt:
            ia = min(np.searchsorted(ta, ts_), len(ta) - 1)
            iv = min(np.searchsorted(tv, ts_), len(tv) - 1)
            il = min(np.searchsorted(tl, ts_), len(tl) - 1)
            print(f"  {(ts_-t0)/1e6:8.1f} {att_yaw[ia]:8.1f} {vis_yaw[iv]:8.1f} {ekf_hdg[il]:8.1f}")
    else:
        print("  (缺话题, 跳过)")

    print("=" * 74)
    print("3) 电机输出 (actuator_motors): 全程采样 + 饱和统计, 看对角饱和")
    am = data.get('actuator_motors')
    if am:
        m4 = mask_of(am)
        t = am.data['timestamp'][m4].astype(np.int64)
        mot = {f'M{j+1}': am.data['control[%d]' % j][m4] for j in range(4)}
        print(f"  {'t(s)':>8} " + " ".join(f"{k:>6}" for k in mot))
        step = max(1, len(t) // 18)
        for i in range(0, len(t), step):
            print(f"  {(t[i]-t0)/1e6:8.1f} " + " ".join(f"{mot[k][i]:6.2f}" for k in mot))
        for k in mot:
            v = mot[k]
            print(f"  {k}: min={v.min():.2f} max={v.max():.2f} <0.05占比={(v < 0.05).mean()*100:.0f}%")

    print("=" * 74)
    print("4) 振动分段 (vehicle_acceleration 各轴 RMS/峰值; gyro x 峰值)")
    acc = data.get('vehicle_acceleration')
    if acc:
        m5 = mask_of(acc); t = acc.data['timestamp'][m5].astype(np.int64)
        for axis, key in [('ax', 'xyz[0]'), ('ay', 'xyz[1]'), ('az', 'xyz[2]')]:
            print(f"  -- {axis} (m/s^2)")
            seg_stats(t, acc.data[key][m5], t0)
    av = data.get('vehicle_angular_velocity')
    if av:
        m6 = mask_of(av); t = av.data['timestamp'][m6].astype(np.int64)
        print("  -- gyro wx (deg/s)")
        seg_stats(t, np.degrees(av.data['xyz[0]'][m6]), t0)

    print("=" * 74)
    print("5) EKF reset计数 / 桨不平衡检测 / 视觉融合标志")
    if vlp:
        m3 = mask_of(vlp)
        for k in ['xy_reset_counter', 'heading_reset_counter']:
            if k in vlp.data:
                d = np.diff(vlp.data[k][m3])
                jumps = np.where(d != 0)[0]
                if len(jumps) == 0:
                    print(f"  {k}: 无跳变 (运动连续, 无估计重置)")
                else:
                    print(f"  {k}: 跳变 @ {[(vlp.data['timestamp'][m3][j]-t0)/1e6 for j in jumps]}")
    fd = data.get('failure_detector_status')
    if fd:
        m7 = mask_of(fd)
        for k in ['imbalanced_prop_metric', 'fd_imbalanced_prop']:
            if k in fd.data:
                v = fd.data[k][m7]
                nz = np.count_nonzero(v)
                mx = np.nanmax(v) if nz else 0.0
                print(f"  {k}: 非零 {nz}/{len(v)} (max={mx:.3f})")
    esf = data.get('estimator_status_flags')
    if esf:
        m8 = mask_of(esf)
        for k in ['cs_ev_pos', 'cs_fake_pos', 'cs_inertial_dead_reckoning']:
            if k in esf.data:
                print(f"  {k}: on占比 {(esf.data[k][m8]==1).mean()*100:.0f}%")

    print("=" * 74)
    print("6) 摇杆 (manual_control_setpoint): x/y 是否打杆")
    mcs = data.get('manual_control_setpoint')
    if mcs:
        m9 = mask_of(mcs)
        xs = mcs.data['x'][m9]; ys = mcs.data['y'][m9]
        print(f"  stick x: mean={np.abs(xs).mean():.3f} max={np.abs(xs).max():.3f}, y: mean={np.abs(ys).mean():.3f} max={np.abs(ys).max():.3f}")

    print("=" * 74)
    print("判定提示:")
    print("  - 视觉yaw独跳、IMU姿态yaw稳定 -> 视觉估计漂移")
    print("  - 三源同步大转 + 姿态环打满拉不住 + 电机对角饱和 + 振动RMS放大 + imbalanced_prop全程非零 -> 机械故障(桨/电机), 视觉yaw漂只是症状")
    print("DONE")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python posctl_drift_probe.py <log.ulg>")
        sys.exit(1)
    main(sys.argv[1])
