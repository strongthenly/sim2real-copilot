#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_topic_truth_probe.py —— 判一个 ROS 话题【到底有没有内容】（不是"有没有在发"）

为什么需要它（2026-10-08 fouruav 实测踩坑）：
    `rostopic hz /planning_vis/frontier` 报 **354~369 Hz**，
    于是判断"前沿可视化在狂发、肯定有东西" → 结论完全错。
    实际取一帧看：`ns=frontier type=CUBE_LIST color=(0,0,0) pts=0`  ← **空消息**
    一个话题可以【高频发空消息】。RViz 里因此什么都没有，而 hz 让你以为一切正常。

⇒ 铁律：**判定"能不能画出来"必须取一帧看内容**，不能只看 hz / publisher 数量。
   同理 `rostopic info` 显示 publisher 存在，也不代表消息非空。

用法：
    source /opt/ros/noetic/setup.bash && source <ws>/devel/setup.bash
    python3 ros_topic_truth_probe.py                       # 用内置默认话题表
    python3 ros_topic_truth_probe.py /topic/a /topic/b     # 指定话题
    python3 ros_topic_truth_probe.py --all                 # 自动扫 /sdf_map /planning* 等前缀

输出：每个话题一行，`有内容` / `空` / `没有该话题`，并给出关键字段
（点数 / marker 类型+颜色+点数 / 数组长度 / 图像尺寸 / Path 位姿数）。

注意：本脚本只读，不发布、不修改任何东西。
"""
from __future__ import print_function

import sys

try:
    import rospy
    from sensor_msgs.msg import PointCloud2, Image, LaserScan
    from visualization_msgs.msg import Marker, MarkerArray
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import PoseStamped
except Exception as e:  # pragma: no cover
    print("需要 source ROS 环境后再跑（import 失败: %s）" % e)
    sys.exit(2)

MARKER_TYPES = {
    0: "ARROW", 1: "CUBE", 2: "SPHERE", 3: "CYLINDER", 4: "LINE_STRIP",
    5: "LINE_LIST", 6: "CUBE_LIST", 7: "SPHERE_LIST", 8: "POINTS",
    9: "TEXT_VIEW_FACING", 10: "MESH_RESOURCE", 11: "TRIANGLE_LIST",
}

DEFAULT_TOPICS = [
    "/heading_planner/frontier", "/heading_planner/visib", "/heading_planner/box",
    "/sdf_map/unknown", "/sdf_map/depth_cloud",
    "/sdf_map/occupancy_all", "/sdf_map/occupancy_local",
    "/sdf_map/occupancy_local_inflate", "/sdf_map/occupancy_inflate", "/sdf_map/esdf",
    "/planning_vis/frontier", "/planning_vis/trajectory", "/planning_vis/topo_path",
    "/planning_vis/yaw", "/planning_vis/visib_constraint", "/planning/travel_traj",
]

SCAN_PREFIXES = ("/sdf_map", "/planning", "/heading_planner", "/exploration", "/waypoint")


def zrange_of_pc(msg, cap=3000):
    """取 PointCloud2 的 z 范围（用于判断点云是"贴地一片"还是"真有立体结构"）。"""
    import struct
    n = msg.width * msg.height
    zs = []
    step = msg.point_step
    for i in range(0, min(n, cap)):
        try:
            zs.append(struct.unpack_from("<f", msg.data, i * step + 8)[0])
        except Exception:
            break
    if not zs:
        return None
    zs.sort()
    return zs[0], zs[-1]


def probe(topic):
    """取一帧消息，返回 (ok, 描述字符串)。只读。"""
    # ---- PointCloud2 ----
    try:
        m = rospy.wait_for_message(topic, PointCloud2, timeout=4)
        n = m.width * m.height
        if n == 0:
            return False, "空点云 (width=%d height=%d, point_step=%d)" % (m.width, m.height, m.point_step)
        zr = zrange_of_pc(m)
        extra = "  z: %.2f~%.2f" % zr if zr else ""
        return True, "点数=%d  frame=%s%s" % (n, m.header.frame_id, extra)
    except Exception:
        pass
    # ---- Marker ----
    try:
        m = rospy.wait_for_message(topic, Marker, timeout=3)
        c = m.color
        npts = len(m.points) + len(m.colors) + (1 if m.mesh_resource else 0)
        desc = "type=%s ns=%s color=(%.2f,%.2f,%.2f) a=%.2f scale=%.2f pts=%d" % (
            MARKER_TYPES.get(m.type, m.type), m.ns or "-", c.r, c.g, c.b, c.a,
            m.scale.x, len(m.points))
        if npts == 0 and m.type in (6, 7, 8, 11, 4, 5):
            return False, "空 marker（列表型但没有点）  " + desc
        return True, desc
    except Exception:
        pass
    # ---- MarkerArray ----
    try:
        m = rospy.wait_for_message(topic, MarkerArray, timeout=3)
        if not m.markers:
            return False, "空 MarkerArray"
        return True, "MarkerArray 长度=%d" % len(m.markers)
    except Exception:
        pass
    # ---- Image ----
    try:
        m = rospy.wait_for_message(topic, Image, timeout=3)
        if m.width == 0 or m.height == 0:
            return False, "空图像 (%dx%d)" % (m.width, m.height)
        return True, "图像 %dx%d enc=%s" % (m.width, m.height, m.encoding)
    except Exception:
        pass
    # ---- Path ----
    try:
        m = rospy.wait_for_message(topic, Path, timeout=3)
        return (len(m.poses) > 0), "Path 位姿数=%d frame=%s" % (len(m.poses), m.header.frame_id)
    except Exception:
        pass
    # ---- Odometry / PoseStamped ----
    for cls, label in ((Odometry, "Odometry"), (PoseStamped, "PoseStamped")):
        try:
            m = rospy.wait_for_message(topic, cls, timeout=3)
            return True, "%s frame=%s" % (label, m.header.frame_id)
        except Exception:
            pass
    return None, "未知类型 / 没有数据"


def main():
    args = sys.argv[1:]
    if "--all" in args:
        args = [a for a in args if a != "--all"]
        try:
            topics = sorted(rospy.get_published_topics() and [t for t, _ in rospy.get_published_topics()])
        except Exception:
            topics = []
        topics = [t for t in topics if t.startswith(SCAN_PREFIXES)]
    elif args:
        topics = args
    else:
        topics = DEFAULT_TOPICS

    rospy.init_node("topic_truth_probe", anonymous=True, disable_signals=True)
    print("=== 话题真实内容体检（只读）===")
    has, empty, missing = [], [], []
    for t in topics:
        ok, desc = probe(t)
        if ok is True:
            has.append(t)
            print("  [有内容] %-38s %s" % (t, desc))
        elif ok is False:
            empty.append(t)
            print("  [  空  ] %-38s %s" % (t, desc))
        else:
            missing.append(t)
            print("  [无数据] %-38s %s" % (t, desc))
    print()
    print("有内容 %d 个 / 空 %d 个 / 无数据 %d 个" % (len(has), len(empty), len(missing)))
    print("★ RViz 配置里【只保留"有内容"那批】；空的会让 RViz 显示 No message，且撑长显示树。")
    print("★ 空 ≠ 坏了：很多话题要"功能真正开始跑"之后才有内容（例：探索未开始 → frontier 为空）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
