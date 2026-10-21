#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 ulog 高度源诊断脚本 —— "高度一直 X m" / "自稳突然爬升" / "高度跳变" 类问题

用法:  python altitude_source_probe.py <log.ulg>

输出:
  - baro 原始高度(压力换算,标准大气公式)时间线 + 最大跳变 —— 真实高度基准
  - EKF local z/vz 时间线 + 最大跳变
  - estimator_aid_src_* 高度/位置源 fused 统计 (fake_hgt 密集=fake=无真实高度源)
  - estimator_status_flags 的 cs_* 标志 (cs_fake_hgt / cs_inertial_dead_reckoning)
  - nav_state / 控制模式 flags / 解锁时刻 / event 时间线
  - 手动油门(throttle) / 姿态欧拉角 / hover_thrust 悬停推力估计
  - 时间线网格: baro vs EKF 高度 vs vz vs 油门 vs 俯仰/横滚

兼容微空 nxtpx4v2 定制固件 PX4 v1.17:
  - sensor_baro 无 altitude 字段 -> 压力换算
  - manual_control_setpoint 油门字段是 throttle (不是 z)
  - 电机输出在 actuator_motors.control[]

判读要点:
  - baro 换算高度 = 真实高度; EKF 高度仅参考
  - EKF AMSL 钉死 = EKF2_BARO_CTRL=0 (baro 融合被关)
  - fake_hgt fused 密集 + cs_inertial_dead_reckoning=1 = 无真实高度源(纯惯性)
  - 姿态大角度时纯惯性 EKF 高度/vz 会严重失真 (与 baro 对比即见)
  - 自稳爬升: 找 vz≈0 时的手动油门值 = 实际悬停点, 对比 MPC_THR_HOVER
"""
import sys
import numpy as np
from pyulog import ULog

def get(ulog, name, idx=0):
    cnt = 0
    for d in ulog.data_list:
        if d.name == name:
            if cnt == idx:
                return d.data
            cnt += 1
    return None

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    ulog = ULog(path, disable_str_exceptions=True)

    # ---- 时间基 ----
    t0 = None
    for name in ('vehicle_local_position', 'sensor_baro', 'vehicle_status'):
        d = get(ulog, name)
        if d is not None and t0 is None:
            ts = np.asarray(d['timestamp'], dtype=np.float64)
            good = ts > 1e10
            t0 = (ts[good].min() if np.any(good) else ts.min())

    def t_sec(ts):
        ts = np.asarray(ts, dtype=np.float64)
        good = ts > 1e10
        return (ts[good] - t0) / 1e6 if np.any(good) else (ts - t0) / 1e6

    def interp_at(d, field, t_target):
        if d is None or field not in d:
            return float('nan')
        ts = t_sec(d['timestamp'])
        v = np.asarray(d[field], dtype=np.float64)
        return float(np.interp(t_target, ts, v))

    def quat_to_euler(q):
        w, x, y, z = q
        roll = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
        pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
        return roll, pitch

    # ---- 1. baro 原始高度 ----
    baro = get(ulog, 'sensor_baro')
    if baro is not None:
        ts = t_sec(baro['timestamp'])
        p = np.asarray(baro['pressure'], dtype=np.float64)
        alt = 44330.0 * (1.0 - np.power(p / 101325.0, 0.1903))
        print("=" * 78)
        print(f"[sensor_baro] n={len(ts)}  (压力换算高度, QNH=1013.25)")
        print(f"  t: {ts.min():.1f}~{ts.max():.1f}s  高度: 首={alt[0]:.2f} 末={alt[-1]:.2f} "
              f"min={alt.min():.2f} max={alt.max():.2f} 变化幅度={alt.max()-alt.min():.1f}m")
        da = np.abs(np.diff(alt))
        idx = np.argsort(da)[-5:][::-1]
        for i in sorted(idx):
            print(f"    跳变 @t={ts[i]:.1f}s: {alt[i]:.2f} -> {alt[i+1]:.2f} (Δ={da[i]:.2f}m)")
    else:
        print("[sensor_baro] 不存在")

    # ---- 2. EKF local z ----
    lp = get(ulog, 'vehicle_local_position')
    if lp is not None:
        ts = t_sec(lp['timestamp'])
        z = np.asarray(lp['z'], dtype=np.float64)
        vz = np.asarray(lp.get('vz', np.zeros_like(z)), dtype=np.float64)
        print("=" * 78)
        print(f"[vehicle_local_position] n={len(ts)}  z(NED向下+), 高度=-z")
        print(f"  z: 首={z[0]:.2f} 末={z[-1]:.2f} min={z.min():.2f} max={z.max():.2f}  "
              f"高度范围 {-z.max():.2f}~{-z.min():.2f}")
        print(f"  vz: min={vz.min():.2f} max={vz.max():.2f}")
        dz = np.abs(np.diff(z))
        idx = np.argsort(dz)[-5:][::-1]
        for i in sorted(idx):
            print(f"    z跳变 @t={ts[i]:.1f}s: {z[i]:.2f}->{z[i+1]:.2f} (Δ={dz[i]:.2f}m) vz={vz[i]:.2f}->{vz[i+1]:.2f}")
    else:
        print("[vehicle_local_position] 不存在")

    # ---- 3. 高度/位置辅助源 fused ----
    print("=" * 78)
    print("[estimator_aid_src_*] 融合状态 (fake_hgt 密集=fake=无真实高度源):")
    for name in ('estimator_aid_src_fake_hgt', 'estimator_aid_src_baro_hgt',
                 'estimator_aid_src_gnss_hgt', 'estimator_aid_src_ev_hgt',
                 'estimator_aid_src_fake_pos', 'estimator_aid_src_ev_pos',
                 'estimator_aid_src_ev_vel', 'estimator_aid_src_ev_yaw'):
        d = get(ulog, name)
        if d is None:
            print(f"  {name}: 话题不存在")
            continue
        ts = t_sec(d['timestamp'])
        fused = np.asarray(d.get('fused', []), dtype=np.float64)
        innov = np.asarray(d.get('innovation', []), dtype=np.float64)
        n_fused = int(fused.sum()) if len(fused) else 0
        iv = innov[np.abs(innov) > 1e-9] if len(innov) else np.array([])
        iv_s = f" innovation末={innov[-1]:.2f}" if len(innov) else ""
        print(f"  {name}: n={len(ts)} fused={n_fused}/{len(ts)}{iv_s}")

    # ---- 4. estimator_status_flags ----
    d = get(ulog, 'estimator_status_flags')
    if d is not None:
        print("=" * 78)
        print("[estimator_status_flags] 置 1 的标志:")
        ts = t_sec(d['timestamp'])
        for k in sorted(d.keys()):
            if k == 'timestamp':
                continue
            v = np.asarray(d[k], dtype=np.int64)
            if v.max() > 0:
                print(f"  {k}: {v.mean()*100:.0f}%  (末={v[-1]})")

    # ---- 5. 模式/解锁/事件 ----
    d = get(ulog, 'vehicle_status')
    if d is not None:
        ts = t_sec(d['timestamp'])
        ns = np.asarray(d['nav_state'], dtype=np.int64)
        print("=" * 78)
        print("[vehicle_status] nav_state 变化:")
        prev = None
        for i in range(len(ts)):
            if ns[i] != prev:
                print(f"  t={ts[i]:.1f}s nav_state={ns[i]}")
                prev = ns[i]
    d = get(ulog, 'vehicle_control_mode')
    if d is not None:
        print("[vehicle_control_mode] flags: " + ", ".join(
            f"{k}={np.asarray(d[k])[0]}" for k in d.keys()
            if k.startswith('flag_') and np.asarray(d[k])[0] == 1))
    d = get(ulog, 'event')
    if d is not None:
        ts = t_sec(d['timestamp'])
        ids = np.asarray(d.get('id', []), dtype=np.int64)
        args = d.get('arguments[0]', np.zeros(len(ts)))
        ev = [(ts[i], ids[i], args[i]) for i in range(len(ts)) if ids[i] != 0 and ts[i] >= 0]
        if ev:
            print("[event] t>=0 的事件:")
            for t, i, a in ev:
                print(f"  t={t:.1f}s id={i} arg0={a}")

    # ---- 6. 悬停推力 ----
    d = get(ulog, 'hover_thrust_estimate')
    if d is not None and 'hover_thrust' in d:
        ht = np.asarray(d['hover_thrust'], dtype=np.float64)
        print("=" * 78)
        print(f"[hover_thrust_estimate.hover_thrust] 首={ht[0]:.3f} 末={ht[-1]:.3f} "
              f"min={ht.min():.3f} max={ht.max():.3f}  (实际悬停油门, 对比 MPC_THR_HOVER)")

    # ---- 7. 时间线网格 ----
    mc = get(ulog, 'manual_control_setpoint')
    att = get(ulog, 'vehicle_attitude')
    throttle_field = 'throttle' if mc is not None and 'throttle' in mc else ('z' if mc is not None and 'z' in mc else None)
    if lp is not None and (baro is not None or True):
        print("=" * 110)
        print("时间线 (t | baro | EKF高度(=-z) | vz | 油门 | 俯仰 | 横滚):")
        grid = np.arange(0, t_sec(lp['timestamp']).max() + 0.25, 0.5)
        for t in grid:
            b = float('nan')
            if baro is not None:
                pv = interp_at(baro, 'pressure', t)
                if not np.isnan(pv):
                    b = 44330.0 * (1.0 - np.power(pv / 101325.0, 0.1903))
            z = interp_at(lp, 'z', t)
            vz = interp_at(lp, 'vz', t)
            th = interp_at(mc, throttle_field, t) if throttle_field else float('nan')
            if att is not None:
                q = [interp_at(att, f'q[{i}]', t) for i in range(4)]
                roll, pitch = quat_to_euler(q)
            else:
                roll = pitch = float('nan')
            print(f"  t={t:5.1f}s  baro={b:7.2f}m  EKF高度={-z:7.3f}  vz={vz:6.2f}  "
                  f"油门={th:6.3f}  俯仰={pitch:6.1f}°  横滚={roll:6.1f}°")

    print("\n完成")

if __name__ == '__main__':
    main()
