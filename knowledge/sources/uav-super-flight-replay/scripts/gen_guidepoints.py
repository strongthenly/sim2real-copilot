#!/usr/bin/env python3
"""
gen_guidepoints.py — 手工生成航点 CSV 文件（SUPER 复飞用）
本会话（2026-08-14）4 次手工生成后沉淀：直线 / L形(圆角) / 任意折线。

用法（在 fouruav 上，或本地生成后 pscp 传）：
  python3 gen_guidepoints.py --shape line --len 12.5 --z 0.75 \
      --out /home/fouruav/catkin_ws/src/task_run_scripts/data_guidepoints/guidepoints_new.txt
  python3 gen_guidepoints.py --shape lshape --len1 1.2 --len2 2.0 --r 0.5 --z 0.75 --out ...
  python3 gen_guidepoints.py --shape poly --pts "0,0 1.2,0 1.2,-2" --z 0.75 --out ...

输出：
  CSV 7 列带表头: x,y,z,qx,qy,qz,qw   （5cm 间距，global_path_publisher 只认逗号+表头）
  endpoint.txt 空格 4 列: x y z 0.1  （waypoint_mission 数据源，必须 ≥4 列见坑41）

关键纪律：
  - 格式必须 CSV 表头（global_path_publisher 用 csv.reader，空格分隔会报"文件为空"）
  - endpoint.txt 空格格式必须 4 列（x y z switch_dis），3 列会被跳过=0 航点=CH10 触发后无目标
  - 生成后验证：head -3 看表头、wc -l 看点数、python 算相邻点距（>5cm 基本安全，<1cm 走廊必失败）
  - 备份旧文件再覆盖（.bak_<tag>）

⚠️ lshape 方向约定（2026-08-24 实测）：len1=前进段（x 正），len2="右移"段实现为 **y 负方向**
  （ROS 惯例 x 前 y 左 → 右 = -y）。"前进2m 右移2m" = `--shape lshape --len1 2 --len2 2 --r 0.5`
  → 终点 (2, -2, 0.75)；若实际地图上方向反了（要 y 正），改用 `--shape poly --pts "0,0 2,0 2,2"`。
  生成后先看打印的"范围 x:..~.. y:..~.."和 endpoint 坐标确认方向，再传机载。
"""
import argparse
import math


def write_csv(path, pts, z):
    with open(path, 'w') as f:
        f.write("x,y,z,qx,qy,qz,qw\n")
        for p in pts:
            f.write(f"{p[0]:.6f},{p[1]:.6f},{z:.6f},0,0,0,1\n")
    print(f"生成 {len(pts)} 点 → {path}")


def add_line(pts, p0, p1, step):
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(round(d / step)))
    for i in range(1, n + 1):
        t = i / n
        pts.append((p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t))


def add_arc(pts, cx, cy, r, th0_deg, th1_deg):
    th0, th1 = math.radians(th0_deg), math.radians(th1_deg)
    if th1 < th0:
        th1 += 2 * math.pi
    n = max(1, int((th1 - th0) / math.radians(2)))
    for i in range(n + 1):
        t = th0 + (th1 - th0) * i / n
        pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))


def dedup(pts):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-6:
            out.append(p)
    return out


def self_cross_count(pts):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    cnt = 0
    for i in range(len(pts) - 1):
        for j in range(i + 2, len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            c, d = pts[j], pts[j + 1]
            d1, d2 = cross(a, b, c), cross(a, b, d)
            d3, d4 = cross(c, d, a), cross(c, d, b)
            if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
                cnt += 1
    return cnt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--shape', choices=['line', 'lshape', 'poly'], required=True)
    ap.add_argument('--len', type=float, default=2.0, help='直线长度(m)')
    ap.add_argument('--len1', type=float, default=1.2, help='L形前进段(m)')
    ap.add_argument('--len2', type=float, default=2.0, help='L形右移段(m)')
    ap.add_argument('--r', type=float, default=0.5, help='L形圆角半径(m)')
    ap.add_argument('--pts', default='0,0 1.2,0 1.2,-2', help='折线顶点 "x,y x,y ..."')
    ap.add_argument('--z', type=float, default=0.75)
    ap.add_argument('--step', type=float, default=0.05)
    ap.add_argument('--out', required=True, help='输出 guidepoints CSV 路径')
    ap.add_argument('--endpoint', help='可选: 输出 endpoint.txt 路径(空格4列)')
    args = ap.parse_args()

    pts = [(0.0, 0.0)]
    if args.shape == 'line':
        add_line(pts, (0, 0), (args.len, 0), args.step)
    elif args.shape == 'lshape':
        add_line(pts, (0, 0), (args.len1 - args.r, 0), args.step)   # 直线1
        add_arc(pts, args.len1, 0.0, args.r, 180, 270)              # 圆角 (右转90°)
        add_line(pts, (args.len1, -args.r), (args.len1, -args.len2), args.step)  # 直线2
    else:  # poly
        for i, sp in enumerate(args.pts.split()):
            x, y = map(float, sp.split(','))
            if i == 0:
                pts[0] = (x, y)
            else:
                add_line(pts, pts[-1], (x, y), args.step)

    pts = dedup(pts)
    write_csv(args.out, pts, args.z)
    print(f"范围 x:{min(p[0] for p in pts):.2f}~{max(p[0] for p in pts):.2f} "
          f"y:{min(p[1] for p in pts):.2f}~{max(p[1] for p in pts):.2f}")
    total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    print(f"总路程 {total:.2f}m, 自交 {self_cross_count(pts)} (应=0)")

    if args.endpoint:
        last = pts[-1]
        with open(args.endpoint, 'w') as f:
            f.write(f"{last[0]:.6f} {last[1]:.6f} {args.z:.6f} 0.1\n")
        print(f"endpoint → {args.endpoint} = ({last[0]:.2f}, {last[1]:.2f}, {args.z:.2f})")


if __name__ == '__main__':
    main()
