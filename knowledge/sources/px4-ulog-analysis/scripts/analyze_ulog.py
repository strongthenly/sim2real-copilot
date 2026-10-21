# -*- coding: utf-8 -*-
"""
PX4 ULog 飞行日志一键分析（调参效果评估）

用法:
    python analyze_ulog.py <log.ulg> [输出目录]

输出:
    控制台统计 + 5 张 PNG (1_attitude / 2_rates / 3_rateerr_fft /
    4_torque_motor / 5_gyro_fft) 到输出目录（默认日志同目录 analysis_<name>/）

依赖:
    pip install pyulog numpy matplotlib
    (建议加 -i https://pypi.tuna.tsinghua.edu.cn/simple)

判读要点（详见 px4-ulog-analysis 技能 SKILL.md）:
    - 角速度误差 RMS <0.5 deg/s = 内环跟踪优秀; >2 = rate P 不足
    - 4-8Hz 频带能量占比 <2% = 无振荡(区分打杆 vs 真振荡的关键)
    - 扭矩/电机饱和率≈0 = 输出余量充足
    - 陀螺 30-80Hz 能量占比 <20% = 振动正常; 峰频率*60 = 电机 RPM
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog


def get(ulog, name, multi=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi:
            return d
    return None


def tsec(ts, t0):
    return (ts - t0) / 1e6


def quat_to_euler(q0, q1, q2, q3):
    w, x, y, z = q0, q1, q2, q3
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def wrap180(a):
    return (a + 180) % 360 - 180


def fft_series(t, v, mask):
    vv = v[mask]
    n = len(vv)
    if n < 128:
        return None, None
    dt = np.median(np.diff(t[mask]))
    V = np.abs(np.fft.rfft(vv * np.hanning(n)))
    fr = np.fft.rfftfreq(n, d=dt)
    V[0] = 0
    return fr, V


def band_energy(t, v, mask, lo, hi):
    """频带能量占比（判定振荡的核心指标）"""
    fr, V = fft_series(t, v, mask)
    if fr is None:
        return None
    tot = V.sum()
    if tot <= 0:
        return 0.0
    return V[(fr >= lo) & (fr <= hi)].sum() / tot


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    base = os.path.splitext(os.path.basename(path))[0]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(path) or '.', 'analysis_' + base)
    os.makedirs(out, exist_ok=True)

    ulog = ULog(path, disable_str_exceptions=True)
    T0 = ulog.start_timestamp
    dur = (ulog.last_timestamp - T0) / 1e6
    print("日志: %s  时长 %.1f s" % (os.path.basename(path), dur))

    # ---------- 读取 ----------
    att = get(ulog, 'vehicle_attitude')
    att_sp = get(ulog, 'vehicle_attitude_setpoint')
    rate_sp = get(ulog, 'vehicle_rates_setpoint')
    angvel = get(ulog, 'vehicle_angular_velocity')
    torq = get(ulog, 'vehicle_torque_setpoint')
    act = get(ulog, 'actuator_motors')
    sens = get(ulog, 'sensor_combined')
    accd = get(ulog, 'vehicle_acceleration')
    rcs = get(ulog, 'rate_ctrl_status')
    cm = get(ulog, 'vehicle_control_mode')
    ld = get(ulog, 'vehicle_land_detected')
    ar = get(ulog, 'actuator_armed')
    bs = get(ulog, 'battery_status')

    miss = [n for n, d in [('vehicle_attitude', att), ('vehicle_attitude_setpoint', att_sp),
                           ('vehicle_rates_setpoint', rate_sp), ('vehicle_angular_velocity', angvel)] if d is None]
    if miss:
        print("缺少关键 topic: %s，无法分析" % miss)
        sys.exit(2)

    # ---------- 飞行段（armed 与 land_detected 自动切出） ----------
    FLY0, FLY1 = 0.5, dur - 0.5
    if ar is not None:
        armed = ar.data['armed'].astype(bool)
        ats = ar.data['timestamp']
        v = ats >= T0  # 过滤 uint64 垃圾时间戳
        if armed[v].any():
            idx = np.where(armed & v)[0]
            FLY0 = max(tsec(ats[idx[0]], T0), 0.0)
            FLY1 = min(tsec(ats[idx[-1]], T0), dur)
    if ld is not None:
        landed = ld.data['landed'].astype(bool)
        lts = ld.data['timestamp']
        v = lts >= T0
        nz = np.where((~landed) & v)[0]
        if len(nz):
            i0 = nz[0]
            land_idx = np.where(landed[i0:])[0]
            if len(land_idx):
                FLY0 = max(FLY0, tsec(lts[i0], T0) + 0.4)
                FLY1 = min(FLY1, tsec(lts[i0 + land_idx[0]], T0) - 0.3)
    if FLY1 <= FLY0:
        FLY0, FLY1 = 2.0, min(dur, 80.0)
    print("飞行段: %.1f - %.1f s" % (FLY0, FLY1))

    def in_fly(t):
        return (t >= FLY0) & (t <= FLY1)

    # ---------- 模式（用 flags，别硬编码 nav_state） ----------
    if cm is not None:
        flags = ['flag_control_offboard_enabled', 'flag_control_position_enabled',
                 'flag_control_velocity_enabled', 'flag_control_altitude_enabled',
                 'flag_control_attitude_enabled', 'flag_control_rates_enabled',
                 'flag_control_manual_enabled']
        seen = set()
        cts = cm.data['timestamp']
        for i in range(len(cts)):
            key = tuple(int(cm.data[f][i]) for f in flags)
            if key not in seen:
                seen.add(key)
                print("t=%6.1fs " % tsec(cts[i], T0) + " ".join(
                    "%s=%d" % (f.replace('flag_control_', '').replace('_enabled', ''), k)
                    for f, k in zip(flags, key)))

    # ---------- 时间轴与误差 ----------
    t_att = tsec(att.data['timestamp'], T0)
    r, p, y = quat_to_euler(att.data['q[0]'], att.data['q[1]'], att.data['q[2]'], att.data['q[3]'])
    t_attsp = tsec(att_sp.data['timestamp'], T0)
    rs, ps, ys = quat_to_euler(att_sp.data['q_d[0]'], att_sp.data['q_d[1]'], att_sp.data['q_d[2]'], att_sp.data['q_d[3]'])
    t_rate = tsec(rate_sp.data['timestamp'], T0)
    rsp = rate_sp.data['roll']; psp = rate_sp.data['pitch']; ysp = rate_sp.data['yaw']
    t_ang = tsec(angvel.data['timestamp'], T0)
    rax = angvel.data['xyz[0]']; pay = angvel.data['xyz[1]']; yaz = angvel.data['xyz[2]']

    err_r = wrap180(r - np.interp(t_att, t_attsp, rs))
    err_p = wrap180(p - np.interp(t_att, t_attsp, ps))
    err_y = wrap180(y - np.interp(t_att, t_attsp, ys))
    er_r = rax - np.interp(t_ang, t_rate, rsp)
    er_p = pay - np.interp(t_ang, t_rate, psp)
    er_y = yaz - np.interp(t_ang, t_rate, ysp)

    # ---------- 跟踪误差统计 ----------
    print("\n=== 飞行段跟踪误差统计 ===")
    print("%-26s %8s %8s %8s" % ('metric', 'RMS', 'mean', '|max|'))
    for nm, (tt, vv) in [('Roll att err (deg)', (t_att, err_r)), ('Pitch att err (deg)', (t_att, err_p)),
                         ('Yaw att err (deg)', (t_att, err_y)), ('Roll rate err (deg/s)', (t_ang, er_r)),
                         ('Pitch rate err (deg/s)', (t_ang, er_p)), ('Yaw rate err (deg/s)', (t_ang, er_y))]:
        m = in_fly(tt)
        v = vv[m]
        print("%-26s %8.3f %8.3f %8.3f" % (nm, np.sqrt(np.mean(v ** 2)), np.mean(v), np.max(np.abs(v))))

    # 悬停/机动分段（阈值 30 deg/s = 0.52 rad/s）
    rate_mag = np.sqrt(rax ** 2 + pay ** 2 + yaz ** 2)
    active = (rate_mag > 0.52) & in_fly(t_ang)
    hover = (~(rate_mag > 0.52)) & in_fly(t_ang)
    print("\n机动段(rate>30deg/s)占比: %.1f%%" % (100 * active.sum() / max(in_fly(t_ang).sum(), 1)))
    for nm, vv in [('roll_rate_err', er_r), ('pitch_rate_err', er_p), ('yaw_rate_err', er_y)]:
        print("%-16s hover RMS=%7.3f  active RMS=%7.3f" % (
            nm, np.sqrt(np.mean(vv[hover] ** 2)), np.sqrt(np.mean(vv[active] ** 2))))

    # 最平稳连续悬停段（供频谱对比）
    low = (rate_mag < 0.1) & in_fly(t_ang)
    best, start = None, None
    for i in range(len(low)):
        if low[i] and start is None:
            start = i
        elif not low[i] and start is not None:
            if best is None or (i - start) > (best[1] - best[0]):
                best = (start, i)
            start = None
    if start is not None and (best is None or (len(low) - start) > (best[1] - best[0])):
        best = (start, len(low))
    hover_idx = None
    if best and (best[1] - best[0]) >= 128:
        hover_idx = np.arange(best[0], best[1])
        print("最平稳连续悬停段: %.1f-%.1fs (%d 样本)" % (t_ang[best[0]], t_ang[best[1] - 1], best[1] - best[0]))

    # ---------- 振荡判定（频带能量占比） ----------
    print("\n=== 振荡判定 ===")
    mf = in_fly(t_ang)
    for nm, vv in [('roll', er_r), ('pitch', er_p), ('yaw', er_y)]:
        fr, V = fft_series(t_ang, vv, mf)
        e48 = band_energy(t_ang, vv, mf, 4, 8)
        if fr is not None:
            sel = (fr >= 0.5) & (fr <= 25)
            pk = fr[sel][np.argmax(V[sel])]
            print("%-6s FFT峰=%.2fHz  4-8Hz能量占比=%.2f%%  (<2%%=无振荡)" % (nm, pk, 100 * e48))
    if hover_idx is not None:
        mh = np.zeros(len(t_ang), dtype=bool); mh[hover_idx] = True
        e48h = band_energy(t_ang, er_r, mh, 4, 8)
        print("悬停段 roll 4-8Hz 能量占比=%.2f%% (真振荡会悬停持续存在且占比大)" % (100 * e48h))

    # ---------- 输出饱和 ----------
    if torq is not None:
        t_tq = tsec(torq.data['timestamp'], T0)
        tq_mag = np.sqrt(torq.data['xyz[0]'] ** 2 + torq.data['xyz[1]'] ** 2 + torq.data['xyz[2]'] ** 2)
        m = in_fly(t_tq)
        print("\n扭矩: |T| RMS=%.4f max=%.4f  饱和(|T|>0.9)占比=%.2f%%" % (
            np.sqrt(np.mean(tq_mag[m] ** 2)), tq_mag[m].max(), 100 * (tq_mag[m] > 0.9).sum() / max(m.sum(), 1)))
    if act is not None:
        t_mot = tsec(act.data['timestamp'], T0)
        mots = [act.data.get('control[%d]' % i) for i in range(4)]
        m = in_fly(t_mot)
        if mots[0] is not None:
            mall = np.column_stack([mots[i][m] for i in range(4)])
            print("电机输出: min=%.3f max=%.3f mean=%.3f  (顶1.0=推力余量不足)" % (mall.min(), mall.max(), mall.mean()))

    # ---------- 积分项 ----------
    if rcs is not None:
        t_rc = tsec(rcs.data['timestamp'], T0)
        m = in_fly(t_rc)
        print("\n角速度积分项(限幅±0.3, 贴限幅=重心/配平问题):")
        for nm, vv in [('roll_integ', rcs.data['rollspeed_integ']), ('pitch_integ', rcs.data['pitchspeed_integ']),
                       ('yaw_integ', rcs.data['yawspeed_integ'])]:
            v = vv[m]
            print("  %-12s min=%+.3f max=%+.3f" % (nm, v.min(), v.max()))

    # ---------- 振动 ----------
    if sens is not None:
        t_gy = tsec(sens.data['timestamp'], T0)
        m = in_fly(t_gy)
        print("\n陀螺振动(30-80Hz能量占比<20%正常):")
        for nm, gv in [('gyro_x', sens.data['gyro_rad[0]']), ('gyro_y', sens.data['gyro_rad[1]']),
                       ('gyro_z', sens.data['gyro_rad[2]'])]:
            v = gv[m]
            fr, V = fft_series(t_gy, gv, m)
            e = band_energy(t_gy, gv, m, 30, 80)
            pk = 0.0
            if fr is not None:
                sel = (fr >= 30) & (fr <= 80)
                if sel.sum():
                    pk = fr[sel][np.argmax(V[sel])]
            print("%s: 原始RMS=%.4f rad/s  30-80Hz占比=%.1f%%  峰=%.1fHz(~%.0fRPM)" % (
                nm, np.sqrt(np.mean(v ** 2)), 100 * e, pk, pk * 60))
    if accd is not None:
        t_acc = tsec(accd.data['timestamp'], T0)
        m = in_fly(t_acc)
        for nm, f in [('acc_x', 'xyz[0]'), ('acc_y', 'xyz[1]'), ('acc_z', 'xyz[2]')]:
            v = accd.data[f][m]
            print("%s 振动RMS=%.3f m/s^2 (水平<0.5好)" % (nm, np.sqrt(np.mean((v - v.mean()) ** 2))))
    if bs is not None and 'voltage_v' in bs.data:
        print("电池电压: min=%.2f max=%.2f V" % (bs.data['voltage_v'].min(), bs.data['voltage_v'].max()))

    # 起降段
    print("\n起降段角速度误差峰值(deg/s):")
    for nm, (t0_, t1_) in [('起飞段', (FLY0 - 0.5, FLY0 + 1.0)), ('着陆段', (FLY1 - 1.0, FLY1 + 1.5))]:
        m = (t_ang >= t0_) & (t_ang <= t1_)
        if m.sum():
            print("%s: roll max=%.2f pitch max=%.2f yaw max=%.2f (落地15-20=触地瞬态正常)" % (
                nm, np.degrees(np.max(np.abs(er_r[m]))), np.degrees(np.max(np.abs(er_p[m]))),
                np.degrees(np.max(np.abs(er_y[m])))))

    # ================= 图表 =================
    plt.rcParams.update({'font.size': 9, 'figure.dpi': 110})

    # 图1 姿态
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for ax, (a, asp), nm in zip(axes, [((t_att, r), (t_attsp, rs), 'Roll'), ((t_att, p), (t_attsp, ps), 'Pitch'),
                                       ((t_att, y), (t_attsp, ys), 'Yaw')]:
        ax.plot(a[0], a[1], 'b-', lw=0.8, label='actual')
        ax.plot(asp[0], asp[1], 'r--', lw=0.8, label='setpoint')
        ax.set_ylabel(nm + ' (deg)'); ax.legend(loc='upper right'); ax.grid(alpha=0.3)
        ax.axvspan(FLY0, FLY1, color='g', alpha=0.06)
    axes[-1].set_xlabel('time (s)')
    fig.suptitle('Attitude: setpoint vs actual (green = flight)')
    fig.tight_layout(); fig.savefig(os.path.join(out, '1_attitude.png')); plt.close(fig)

    # 图2 角速度
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for ax, (a, asp), nm in zip(axes, [((t_ang, rax), (t_rate, rsp), 'Roll rate'),
                                       ((t_ang, pay), (t_rate, psp), 'Pitch rate'),
                                       ((t_ang, yaz), (t_rate, ysp), 'Yaw rate')]):
        ax.plot(a[0], np.degrees(a[1]), 'b-', lw=0.7, label='actual')
        ax.plot(asp[0], np.degrees(asp[1]), 'r--', lw=0.7, label='setpoint')
        ax.set_ylabel(nm + ' (deg/s)'); ax.legend(loc='upper right'); ax.grid(alpha=0.3)
        ax.axvspan(FLY0, FLY1, color='g', alpha=0.06)
    axes[-1].set_xlabel('time (s)')
    fig.suptitle('Angular rate: setpoint vs actual')
    fig.tight_layout(); fig.savefig(os.path.join(out, '2_rates.png')); plt.close(fig)

    # 图3 角速度误差频谱
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    for ax, vv, nm in zip(axes, [er_r, er_p, er_y], ['Roll', 'Pitch', 'Yaw']):
        fr, V = fft_series(t_ang, vv, mf)
        if fr is not None:
            ax.semilogy(fr, V, 'b-', lw=0.7, label='whole flight')
        if hover_idx is not None:
            mh = np.zeros(len(t_ang), dtype=bool); mh[hover_idx] = True
            fr2, V2 = fft_series(t_ang, vv, mh)
            if fr2 is not None:
                ax.semilogy(fr2, V2, 'r-', lw=0.9, label='hover')
        ax.set_ylabel(nm + ' rate err'); ax.grid(alpha=0.3); ax.legend(loc='upper right'); ax.set_xlim(0, 25)
    axes[-1].set_xlabel('Frequency (Hz)')
    fig.suptitle('Rate error spectrum (blue=flight, red=hover)')
    fig.tight_layout(); fig.savefig(os.path.join(out, '3_rateerr_fft.png')); plt.close(fig)

    # 图4 扭矩+电机
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    if torq is not None:
        axes[0].plot(t_tq, torq.data['xyz[0]'], 'b-', lw=0.7, label='Tx')
        axes[0].plot(t_tq, torq.data['xyz[1]'], 'g-', lw=0.7, label='Ty')
        axes[0].plot(t_tq, torq.data['xyz[2]'], 'm-', lw=0.7, label='Tz')
        axes[0].axhline(1.0, color='r', ls='--', lw=0.8); axes[0].axhline(-1.0, color='r', ls='--', lw=0.8)
        axes[0].set_ylabel('Torque SP'); axes[0].legend(loc='upper right'); axes[0].grid(alpha=0.3)
        axes[0].axvspan(FLY0, FLY1, color='g', alpha=0.06)
    if act is not None and mots[0] is not None:
        for i in range(4):
            axes[1].plot(t_mot, mots[i], lw=0.7, label='M%d' % i)
        axes[1].axhline(1.0, color='r', ls='--', lw=0.8)
        axes[1].set_ylabel('Motor output'); axes[1].legend(loc='upper right', ncol=4); axes[1].grid(alpha=0.3)
        axes[1].axvspan(FLY0, FLY1, color='g', alpha=0.06)
    axes[2].plot(t_ang, np.degrees(er_r), 'b-', lw=0.5, label='roll')
    axes[2].plot(t_ang, np.degrees(er_p), 'g-', lw=0.5, label='pitch')
    axes[2].plot(t_ang, np.degrees(er_y), 'm-', lw=0.5, label='yaw')
    axes[2].set_ylabel('Rate err (deg/s)'); axes[2].set_xlabel('time (s)')
    axes[2].legend(loc='upper right'); axes[2].grid(alpha=0.3); axes[2].axvspan(FLY0, FLY1, color='g', alpha=0.06)
    fig.suptitle('Torque / motor / rate error')
    fig.tight_layout(); fig.savefig(os.path.join(out, '4_torque_motor.png')); plt.close(fig)

    # 图5 陀螺振动频谱
    if sens is not None:
        fig, axes = plt.subplots(3, 1, figsize=(12, 8))
        for ax, gv, nm in zip(axes, [sens.data['gyro_rad[0]'], sens.data['gyro_rad[1]'], sens.data['gyro_rad[2]']],
                              ['gyro X', 'gyro Y', 'gyro Z']):
            fr, V = fft_series(t_gy, gv, m)
            if fr is not None:
                ax.semilogy(fr, V, 'b-', lw=0.7, label='flight')
            ax.set_ylabel(nm + ' (rad/s)'); ax.grid(alpha=0.3); ax.legend(loc='upper right'); ax.set_xlim(0, 80)
        axes[-1].set_xlabel('Frequency (Hz)')
        fig.suptitle('Gyro vibration spectrum (0-80Hz)')
        fig.tight_layout(); fig.savefig(os.path.join(out, '5_gyro_fft.png')); plt.close(fig)

    print("\n图表已保存到", out)
    for f in sorted(os.listdir(out)):
        print(" -", f)


if __name__ == '__main__':
    main()
