#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云几何量化诊断 —— 判"相机看得到障碍、LiDAR 点云里却没有"到底是不是 FOV 盲区。

同时订阅两路点云并打印几何统计：
  1) 桥原始点云（AirSim 直出，传感器局部系）  默认 /twin/lidar/points_preview
  2) FAST-LIVO2 输出（世界系 camera_init）      默认 /cloud_registered

关键判读：
  * 俯仰角全部 >= VerticalFOVLower(如 -7°)、且「水平距离 min」远大于 0
    → 是 LiDAR 垂直 FOV 盲区，不是桥/算法坏了。
  * 盲区半径理论值 = 无人机对地高度 / tan(|VerticalFOVLower|)
        3m 高 + -7° 下界  →  ≈24.4m
    实测值应与理论吻合（本会话：3m 高，最近点恒为 23.08m；前进 2m 后仍是 23.08m）。

用法：
  source /opt/ros/noetic/setup.bash && source <ws>/devel/setup.bash
  python3 cloud_geometry_analyze.py [原始点云话题] [LIVO输出话题]
  默认: /twin/lidar/points_preview  /cloud_registered

配合：AirSim settings.json 里 TwinLidarPreview 的 VerticalFOVLower/Upper。
修法见 SKILL.md「相机能看到障碍、点云里却没有」节（调 VerticalFOVLower / 加 Pitch / 飞低）。
"""
import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def analyze(topic, name, timeout=12.0):
    try:
        msg = rospy.wait_for_message(topic, PointCloud2, timeout=timeout)
    except Exception as e:
        print("[%s] 等待 %s 超时: %s" % (name, topic, e))
        return
    pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
    print("=" * 60)
    print("[%s]  %s   frame_id=%s" % (name, topic, msg.header.frame_id))
    print("  点数: %d" % len(pts))
    if len(pts) == 0:
        print("  (空点云 —— 查桥是否在发 / AirSim 是否在跑 / PX4 是否连着->LockStep 时间是否在走)")
        return

    for i, ax in enumerate('xyz'):
        v = pts[:, i]
        print("  %s: min=%.3f max=%.3f 范围=%.3f 中位=%.3f" % (ax, v.min(), v.max(), v.max() - v.min(), np.median(v)))

    # 俯仰角（相对雷达水平面）—— 判 FOV 盲区的核心指标
    r = np.linalg.norm(pts[:, :2], axis=1)
    ang = np.degrees(np.arctan2(pts[:, 2], np.maximum(r, 1e-6)))
    print("  俯仰角: min=%.1f° max=%.1f°  中位=%.1f°" % (ang.min(), ang.max(), np.median(ang)))
    print("  俯仰角 >0(上方)占比: %.1f%%" % (100.0 * np.sum(ang > 0) / len(ang)))
    print("  俯仰角 < -7°(超出 MID360 FOV 下界)占比: %.1f%%" % (100.0 * np.sum(ang < -7) / len(ang)))

    # 水平距离 —— 盲区半径的直接体现
    print("  水平距离: min=%.2f  max=%.2f" % (r.min(), r.max()))

    # 自动判读
    if r.min() > 5.0 and ang.min() >= -8.0:
        print("  >>> 判读: 最近点 %.2fm 且俯仰角下界≈FOV 下界"
              " → 典型的 LiDAR 垂直 FOV 盲区（盲区半径≈高度/tan(|下界|)），不是链路故障。" % r.min())


def main():
    topics = sys.argv[1:]
    raw = topics[0] if len(topics) > 0 else '/twin/lidar/points_preview'
    livo = topics[1] if len(topics) > 1 else '/cloud_registered'
    rospy.init_node('cloud_geometry_analyze', anonymous=True)
    analyze(raw, '桥原始点云')
    analyze(livo, 'FAST-LIVO2 输出')


if __name__ == '__main__':
    main()
