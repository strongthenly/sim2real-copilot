#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mid360_fov_measure.py — 实测雷达真实【垂直视角 / 盲区 / 安装俯仰角】

为什么需要它：FUEL 选视点高度靠"可见性模型"，而该模型的 FOV 参数若与实机不符，
选出的高度就是错的（会导致"前沿看不到 / 永远贴上限飞"）。规格书和仿真建模值
【都不能当结论】，必须用实机点云实测。

两种模式：
  A) 实时（LiDAR 在跑即可，不需要整套栈）—— 最准，尤其是飞机【在地面静止】时
     python3 mid360_fov_measure.py live /livox/lidar 15
  B) 离线（从 rosbag 取前 N 帧；注意确认 bag 是【这个雷达】录的 ✗）
     python3 mid360_fov_measure.py bag /path/x.bag 15

判读：
  · 仰角直方图两端【陡峭截断】= 真实 FOV 边界
  · 地面静止测最可靠：地面必然充满下视角，截断位置就是真实下界
  · 最近点距离分布 → 真实盲区；直方图整体偏移 → 安装俯仰角
  · 对比假设（Mid-360 常见值：上 +52° / 下 −7° / 水平 360°）
    若实测下界明显 > −7°（如 −20°），把 down_angle 改成实测弧度即可（不用重编译）
"""
import sys, math, io

MODE = sys.argv[1] if len(sys.argv) > 1 else 'live'
SRC  = sys.argv[2] if len(sys.argv) > 2 else '/livox/lidar'
NF   = int(sys.argv[3]) if len(sys.argv) > 3 else 15

hist, dists, ntot, frames = {}, [], 0, 0
zmin, zmax = 1e9, -1e9


def feed(msg, read_points):
    global ntot, frames, zmin, zmax
    frames += 1
    for p in read_points(msg):
        x, y, z = p[0], p[1], p[2]
        r_xy = math.hypot(x, y)
        el = math.degrees(math.atan2(z, r_xy))          # 仰角（相对雷达水平面）
        hist[int(math.floor(el))] = hist.get(int(math.floor(el)), 0) + 1
        d = math.hypot(r_xy, z)
        if d > 1e-6:
            dists.append(d)
        ntot += 1
        if el < zmin: zmin = el
        if el > zmax: zmax = el


if MODE == 'bag':
    import rosbag
    from sensor_msgs import point_cloud2 as pc2
    bag = rosbag.Bag(SRC, 'r')
    topic = None
    for t, ti in sorted(bag.get_type_and_topic_info().topics.items()):
        if 'PointCloud2' in ti[0] and ti[1] > 0:
            topic = t; break
    print("bag 点云话题: %s（若这不是本机雷达录的，结论无效 ✗）" % topic)
    got = 0
    for _, msg, _ in bag.read_messages(topics=[topic]):
        got += 1
        if got > NF: break
        feed(msg, lambda m: pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True))
    bag.close()
else:
    import rospy, time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2 as pc2
    print(__doc__)
    print("订阅 %s 采样 %d 秒…（建议飞机在地面静止）" % (SRC, NF))
    rospy.init_node('mid360_fov_measure')
    rospy.Subscriber(SRC, PointCloud2,
                     lambda m: feed(m, lambda x: pc2.read_points(x, field_names=('x', 'y', 'z'),
                                                                 skip_nans=True)),
                     queue_size=5)
    t0 = time.time()
    while not rospy.is_shutdown() and time.time() - t0 < NF:
        time.sleep(0.2)

print("\n帧数 %d  点数 %d" % (frames, ntot))
if ntot == 0:
    sys.exit("✗ 没收到点云 —— 确认话题名: rostopic list | grep -iE 'livox|lidar|cloud'")

ks = sorted(hist.keys())
mx = max(hist.values())
print("\n=== 仰角直方图（1° 分箱）===")
for k in ks:
    print("  %+5d°  %9d  %s" % (k, hist[k], '#' * int(60.0 * hist[k] / mx)))
lo = next((k for k in ks if hist[k] > mx * 0.002), None)
hi = next((k for k in reversed(ks) if hist[k] > mx * 0.002), None)
print("\n★★ 实测仰角范围: %.2f° ~ %.2f°" % (zmin, zmax))
print("★★ 0.2%% 阈值截断: 下 %s°  上 %s°" % (lo, hi))
print("★★ 常见假设:      下 -7°     上 +52°")
if lo is not None and lo <= -15:
    print("   ⇒ 实测下视角【明显大于 7°】⇒ 假设不成立，改成实测弧度（algorithm.xml 的 down_angle）")
elif lo is not None and lo > -10:
    print("   ⇒ 实测下视角接近 -7° ⇒ 假设成立（近处地面/矮物确为盲区，需飞低或远看）")
if dists:
    dists.sort()
    print("★ 最近点距离: min %.3f m  P1 %.3f m  P50 %.3f m" % (
        dists[0], dists[len(dists) // 100], dists[len(dists) // 2]))
print("--FOVMEASURE-DONE--")
