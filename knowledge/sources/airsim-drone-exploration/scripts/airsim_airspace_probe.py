#!/usr/bin/env python3
"""AirSim 飞行方向空域探测（只读，不发任何控制指令）。

用途：起飞前判断「某个 ENU 方向是否空旷」——外部规划器/实验脚本往那边飞之前先看这个。

原理：AirSim RPC getMultirotorState 取位姿 + 四元数，getLidarData 取点云（默认在
SensorLocalFrame = 机体系 x前/y右/z下），用机体系四元数把点云旋到世界 NED
（x北/y东/z下，地面 z=0、向上为负），只保留"空中带"点，再按 ENU 方向统计
1m 宽走廊内的点数与最近障碍距离。

坐标系备忘：MAVROS/ENU 的 +X = 东 = AirSim 世界 NED 的 +y。

依赖：只要 python3 + msgpack（不需要装 airsim pip 包）。
用法（WSL 里跑，AirSim 场景已 Play）：
    python3 airsim_airspace_probe.py
    python3 airsim_airspace_probe.py --directions 0,90,180,-90 --corridor 1.0
    python3 airsim_airspace_probe.py --lidar TwinLidarPreview --vehicle PX4
"""
import argparse
import math
import socket
import sys

try:
    import msgpack
except ImportError:
    sys.exit("需要 msgpack：pip3 install msgpack（或 apt install python3-msgpack）")


def call(host, port, method, args, timeout=20):
    """单次 AirSim RPC 调用。返回 result，失败抛异常。"""
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.settimeout(timeout)
        s.sendall(msgpack.packb([0, 1, method, args, ], use_bin_type=True))
        up = msgpack.Unpacker(raw=False)
        while True:
            data = s.recv(1 << 20)
            if not data:
                raise RuntimeError("RPC closed")
            up.feed(data)
            for resp in up:
                if resp[0] == 1 and resp[1] == 1:
                    if resp[2] is not None:
                        raise RuntimeError(str(resp[2]))
                    return resp[3]


def quat_to_rot(qw, qx, qy, qz):
    """返回把机体系向量旋到世界系(NED)的 3x3 矩阵（行主序，9 元）。"""
    return (
        1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy),
    )


def apply_rot(R, v):
    x, y, z = v
    return (R[0] * x + R[1] * y + R[2] * z,
            R[3] * x + R[4] * y + R[5] * z,
            R[6] * x + R[7] * y + R[8] * z)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=41451)
    ap.add_argument("--vehicle", default="PX4")
    ap.add_argument("--lidar", default="TwinLidarPreview")
    ap.add_argument("--directions", default="0,90,180,-90",
                    help="要检查的 ENU 方向（度，逗号分隔；0=+X=东，90=+Y=北）")
    ap.add_argument("--corridor", type=float, default=1.0, help="走廊半宽（米）")
    ap.add_argument("--min-range", type=float, default=0.6, help="忽略此距离内的点（自身/近场）")
    ap.add_argument("--height-band", default="-3.0,-0.3",
                    help="保留的世界系 NED z 区间（地面 z=0，向上为负）")
    a = ap.parse_args()

    zlo, zhi = (float(x) for x in a.height_band.split(","))

    # ---- AirSim 可达性 ----
    try:
        paused = call(a.host, a.port, "simIsPaused", [])
    except Exception as exc:
        print("AirSim RPC 不可达（%s:%d）：%s" % (a.host, a.port, exc))
        print("→ Blocks 只有编辑器工程、无打包 exe，无法自动进入 Play："
              "请在 UE4 编辑器打开场景并点 ▶ Play 后重试。")
        return 3
    if paused:
        print("AirSim 处于暂停（simIsPaused=true）→ 请在场景窗口取消暂停。")
        return 3

    # ---- 位姿 ----
    st = call(a.host, a.port, "getMultirotorState", [a.vehicle])
    k = st["kinematics_estimated"]
    p, o = k["position"], k["orientation"]
    px, py, pz = p["x_val"], p["y_val"], p["z_val"]
    qw, qx, qy, qz = o["w_val"], o["x_val"], o["y_val"], o["z_val"]
    yaw_ned = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    print("车辆 %s @ 世界 NED (%.2f, %.2f, %.2f)   yaw(NED,0=北)=%.1f°  → ENU yaw=%.1f°"
          % (a.vehicle, px, py, pz, math.degrees(yaw_ned),
             math.degrees(math.pi / 2 - yaw_ned)))

    # ---- 点云 ----
    try:
        d = call(a.host, a.port, "getLidarData", [a.lidar, ""])
    except Exception as exc:
        print("取 LiDAR 「%s」失败：%s" % (a.lidar, exc))
        print("→ 确认 settings.json 里该传感器的名字，和场景是否已 Play。")
        return 4
    pts = d.get("point_cloud") or []
    if not pts:
        print("LiDAR 「%s」返回 0 点 → 传感器未出数据（多半场景没 Play 或 LockStep 下 PX4 没连）。" % a.lidar)
        return 4
    xs, ys, zs = pts[0::3], pts[1::3], pts[2::3]
    print("LiDAR %s: %d 点   机体系范围 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]"
          % (a.lidar, len(xs), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))

    # ---- 机体系 → 世界 NED，只留空中带 ----
    R = quat_to_rot(qw, qx, qy, qz)
    air = []
    for x, y, z in zip(xs, ys, zs):
        wx, wy, wz = apply_rot(R, (x, y, z))
        WX, WY, WZ = px + wx, py + wy, pz + wz
        if zlo < WZ < zhi:
            air.append((WX, WY))
    print("空中带 (NED z ∈ [%.1f, %.1f]) 点数 = %d" % (zlo, zhi, len(air)))
    if not air:
        print("→ 空中带内无点：附近该高度大概率空旷（或 LiDAR 没扫到）。")
        return 0

    # ---- 逐方向统计 ----
    print("\n方向              走廊内点数   最近障碍")
    verdict = []
    for tok in a.directions.split(","):
        tok = tok.strip()
        if not tok:
            continue
        deg = float(tok)
        rad = math.radians(deg)
        # ENU 单位向量 → NED 分量（ENU +X=东=NED +y，ENU +Y=北=NED +x）
        ex, ey = math.cos(rad), math.sin(rad)
        nx, ny = ey, ex
        cnt, best = 0, None
        for X, Y in air:
            proj = X * nx + Y * ny
            perp = abs(-X * ny + Y * nx)
            if proj > a.min_range and perp < a.corridor:
                cnt += 1
                if best is None or proj < best:
                    best = proj
        label = "ENU dir=%+7.1f°" % deg
        print("  %-16s %6d      %s" % (label, cnt,
                                       ("%.2f m" % best) if best else "该距离外无点（较空旷）"))
        verdict.append((deg, best))

    ok = [v for v in verdict if v[1] is None]
    print("\n=== 结论 ===")
    if ok:
        print("  最空旷方向：%s（走廊内无障碍）"
              % "、".join("dir=%+g°" % d for d, _ in ok))
    order = sorted(verdict, key=lambda t: (t[1] is not None, t[1] if t[1] is not None else 1e9))
    print("  按最近障碍排序：" + " > ".join(
        "dir=%+g°(%s)" % (d, "无" if b is None else "%.2fm" % b) for d, b in order))
    print("  提醒：把试验的**预计前向距离**（巡航时长 × 速度 + 刹车距离）和上面的最近障碍比一比；"
          "边际 <1m 就别飞，或换方向。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
