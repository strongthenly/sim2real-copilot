#!/usr/bin/env python3
"""
生成 C 形开放测试航迹（前进→右移→后退），供 fouruav 轨迹复飞测试。
用法: python3 make_c_shape_waypoints.py [前进m] [右移m] [输出路径]
默认: 前进2m 右移2m -> (0,0)→(2,0)→(2,-2)→(0,-2)，z=0.75，圆角R=0.3
输出: 空格格式 "x y z"（每行3列；endpoint.txt 需另写4列含 switch_dis）
注意:
  - 终点=起点 的"闭环"航迹会触发 close-to-goal 不飞，必须开放(终点≠起点)
  - z 统一 0.75；goal_z_min/max 地面测试临时 0.0/0.3，飞行前改回 0.6/0.825
"""
import math, sys

def add_line(pts, p0, p1, step=0.05):
    d = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
    n = max(2, int(d/step))
    for i in range(1, n+1):
        t = i/n
        pts.append((p0[0]+(p1[0]-p0[0])*t, p0[1]+(p1[1]-p0[1])*t))

def add_arc(pts, cx, cy, R, th0, th1, step_deg=3):
    th0r, th1r = math.radians(th0), math.radians(th1)
    if th1r < th0r: th1r += 2*math.pi
    n = max(2, int((th1r-th0r)/(math.radians(step_deg))))
    for i in range(1, n+1):
        t = th0r + (th1r-th0r)*i/n
        pts.append((cx+R*math.cos(t), cy+R*math.sin(t)))

def gen_c_shape(forward, right, z=0.75, R=0.3, out="/home/fouruav/catkin_ws/src/task_run_scripts/data_guidepoints/guidepoints_new.txt"):
    pts = []
    pts.append((0.0, 0.0))
    add_line(pts, (0.0, 0.0), (forward-R, 0.0))
    add_arc(pts, forward, 0.0, R, -90, 0)                 # 拐角1: (F,0) 右上
    add_line(pts, (forward, -R), (forward, -(right-R)))
    add_arc(pts, forward, -right, R, 180, 270)            # 拐角2: (F,-right) 左下
    add_line(pts, (forward-R, -right), (0.0, -right))
    with open(out, 'w') as f:
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {z:.6f}\n")
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    dist = sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]) for i in range(len(pts)-1))
    print(f"生成 {len(pts)} 点, x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}] z={z}, 总路程 {dist:.2f}m")
    print(f"起点 {pts[0]} 终点 {pts[-1]} (开放, 非闭环)")
    print(f"endpoint.txt 写: {pts[-1][0]:.6f} {pts[-1][1]:.6f} {z:.6f} 0.1")

if __name__ == "__main__":
    fwd = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    right = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    out = sys.argv[3] if len(sys.argv) > 3 else "/home/fouruav/catkin_ws/src/task_run_scripts/data_guidepoints/guidepoints_new.txt"
    gen_c_shape(fwd, right, out=out)
