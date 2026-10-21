#!/usr/bin/env python3
"""
SUPER 测试航迹生成器（fouruav 航点任务用）
生成 CSV 格式航点文件（global_path_publisher 用 csv.reader 解析，必须逗号分隔+表头 x,y,z,qx,qy,qz,qw）
支持: 直行 / L形(前进+右移) / C形(前进+右移+后退)，圆角 R 平滑，自交检查

用法:
  python3 generate_waypoints.py straight 12.5            # 直行 12.5m (x轴)
  python3 generate_waypoints.py lshape 1.2 2.0           # 前进1.2m + 右移2m
  python3 generate_waypoints.py cshape 2.0 2.0           # 前进2m + 右移2m + 后退2m
  python3 generate_waypoints.py straight 8.0 -o /tmp/t.txt
  # 同时生成 endpoint.txt 提示（终点 z=0.75, switch_dis=0.1）
"""
import math
import sys
import os

Z = 0.75        # 航迹高度（飞行测试用；地面测试可改 0.3）
R = 0.5         # 圆角半径 m（越大转弯越平滑）
STEP = 0.05     # 直线采样间距 m
OUT = "/home/fouruav/catkin_ws/src/task_run_scripts/data_guidepoints/guidepoints_new.txt"
ENDPOINT = "/home/fouruav/catkin_ws/src/task_run_scripts/data_guidepoints/endpoint.txt"


def add_line(pts, p0, p1):
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(round(d / STEP)))
    for i in range(1, n + 1):
        t = i / n
        pts.append((p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t))


def add_arc(pts, cx, cy, th0_deg, th1_deg):
    """圆弧，圆心(cx,cy) 半径R，角度递增（含起点终点），每2度采样"""
    th0, th1 = math.radians(th0_deg), math.radians(th1_deg)
    if th1 < th0:
        th1 += 2 * math.pi
    n = max(1, int((th1 - th0) / math.radians(2)))
    for i in range(n + 1):
        t = th0 + (th1 - th0) * i / n
        pts.append((cx + R * math.cos(t), cy + R * math.sin(t)))


def dedup(pts):
    uniq = []
    for p in pts:
        if not uniq or math.hypot(p[0] - uniq[-1][0], p[1] - uniq[-1][1]) > 1e-6:
            uniq.append(p)
    return uniq


def check_self_cross(pts):
    def seg_cross(a, b, c, d):
        def cross(o, p, q):
            return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
        d1, d2 = cross(a, b, c), cross(a, b, d)
        d3, d4 = cross(c, d, a), cross(c, d, b)
        return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))
    return sum(1 for i in range(len(pts) - 1) for j in range(i + 2, len(pts) - 1)
               if seg_cross(pts[i], pts[i + 1], pts[j], pts[j + 1]))


def straight(dist):
    pts, x = [(0.0, 0.0)], 0.0
    while x <= dist:
        pts.append((x, 0.0))
        x += STEP
    if pts[-1][0] != dist:
        pts.append((dist, 0.0))
    return dedup(pts), (dist, 0.0)


def lshape(fwd, right):
    """前进 fwd → 右移 right（x+ 转 y-，顺时针），圆心(1.2,0) 180°→270°"""
    pts = [(0.0, 0.0)]
    add_line(pts, (0.0, 0.0), (fwd - R, 0.0))
    add_arc(pts, fwd, 0.0, 180, 270)
    add_line(pts, (fwd, -R), (fwd, -right))
    return dedup(pts), (fwd, -right)


def cshape(fwd, right):
    """前进 fwd → 右移 right → 后退 fwd（C形）"""
    pts = [(0.0, 0.0)]
    add_line(pts, (0.0, 0.0), (fwd - R, 0.0))
    add_arc(pts, fwd, 0.0, 180, 270)
    add_line(pts, (fwd, -R), (fwd, -(right - R)))
    add_arc(pts, fwd, -right, 90, 180)
    add_line(pts, (fwd - R, -right), (0.0, -right))
    return dedup(pts), (0.0, -right)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    kind = sys.argv[1]
    if kind == "straight":
        dist = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
        pts, end = straight(dist)
    elif kind == "lshape":
        fwd = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2
        right = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
        pts, end = lshape(fwd, right)
    elif kind == "cshape":
        fwd = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
        right = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
        pts, end = cshape(fwd, right)
    else:
        print(__doc__)
        sys.exit(1)

    out = OUT
    if "-o" in sys.argv:
        out = sys.argv[sys.argv.index("-o") + 1]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write("x,y,z,qx,qy,qz,qw\n")
        for p in pts:
            f.write(f"{p[0]:.6f},{p[1]:.6f},{Z:.6f},0,0,0,1\n")

    total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    cross = check_self_cross(pts)
    print(f"生成 {len(pts)} 点 → {out}")
    print(f"形状: {kind} | 起点{pts[0]} 终点{pts[-1]} | 总路程 {total:.2f}m | 自交 {cross}(应=0)")
    print(f"endpoint.txt 应写: {end[0]:.6f} {end[1]:.6f} {Z:.6f} 0.1")
    print("⚠️ 若终点 z 受 goal_z_min/max 钳制，检查 fsm 段配置（地面 0.0/0.3，飞行 0.6/0.825）")


if __name__ == "__main__":
    main()
