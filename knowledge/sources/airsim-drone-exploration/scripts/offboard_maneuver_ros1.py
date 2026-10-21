#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offboard 多段机动 (ROS1 Noetic + MAVROS + PX4) —— 前进 / 转向 任意组合

SEGMENTS 按顺序写:
    ("fwd",   10.0)   # 沿当前机头方向前进 N 米 (保持高度)
    ("right", 90.0)   # 原地右转 N 度 (ENU: yaw -= N)
    ("left",  90.0)   # 原地左转 N 度 (ENU: yaw += N)
    ("hold",   5.0)   # 原地悬停 N 秒

实测 (ros1-twin, 2026-09-10):
    [("fwd",10), ("right",90), ("fwd",20)]
      -> 正确走出 L 形 (x: +9.7m 然后 y: -19.7m, 两段严格正交)
      -> 高度全程 2.98m 不变, 每段误差约 0.3m
    [("fwd", 2)]  -> 目标 2m 实到 1.855m

前提:
  1) MAVROS 已在跑且 /mavros/state 出 connected: True
     (ROS1 侧坑: 缺 GeographicLib geoid 会秒退; 14540 被占 -> 用 PX4 专用 mavlink
      `mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard`
      + `roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610`)
  2) 无人机已起飞/悬停(PX4 在 LOITER 也行, 脚本会切 OFFBOARD)

用法:  python3 offboard_maneuver_ros1.py

⚠️ 脚本结束就不再发 setpoint -> PX4 offboard 超时 -> 若 settings 里 COM_OBL_ACT=1,
   无人机会 AUTO.LAND(自动降落)。这是安全行为不是故障; 要它留在空中悬停,
   必须保持脚本运行, 或在末尾把 set_mode 切回 AUTO.LOITER / POSCTL。
"""
import math

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# ==================== 参数 ====================
SEGMENTS = [
    ("fwd", 10.0),
    ("right", 90.0),
    ("fwd", 20.0),
]
SPEED = 2.0          # 前进速度 m/s
TURN_SEC = 4.0       # 转向平滑过渡用时 s
PRE_STREAM_SEC = 2.0 # 切 OFFBOARD 前预发 setpoint 的时长 s (PX4 要求 >0.5s)
HOLD_SEC = 5.0       # 结束时原地悬停保持 s
# ==============================================

state = State()
cur = PoseStamped()


def cb_s(m):
    global state
    state = m


def cb_p(m):
    global cur
    cur = m


rospy.init_node('offboard_maneuver', anonymous=True)
rospy.Subscriber('/mavros/state', State, cb_s)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_p)
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rate = rospy.Rate(20)

sp = PoseStamped()


def set_yaw(y):
    """位置 setpoint 里带姿态 -> PX4 同时跟踪 yaw; 四元数只绕 z 转"""
    sp.pose.orientation.x = 0.0
    sp.pose.orientation.y = 0.0
    sp.pose.orientation.z = math.sin(y / 2.0)
    sp.pose.orientation.w = math.cos(y / 2.0)


def set_pos(p):
    sp.pose.position.x = p.x
    sp.pose.position.y = p.y
    sp.pose.position.z = p.z


def hold(p, y, sec):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < sec:
        set_pos(p); set_yaw(y); pub.publish(sp); rate.sleep()


def fly(p0, yaw, dist, label):
    """沿 yaw 方向匀速直线飞 dist 米 (位置 setpoint 按 SPEED*dt 递增)"""
    rospy.loginfo('--- %s (yaw=%.1f°, %.1fm) ---', label, math.degrees(yaw), dist)
    t0 = rospy.Time.now(); d = 0.0; last = t0; lastlog = -99.0
    while not rospy.is_shutdown():
        now = rospy.Time.now(); dt = (now - last).to_sec(); last = now
        d = min(d + SPEED * dt, dist)
        set_pos(Point(p0.x + math.cos(yaw) * d, p0.y + math.sin(yaw) * d, p0.z))
        set_yaw(yaw); pub.publish(sp); rate.sleep()
        p = cur.pose.position
        moved = math.sqrt((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2)
        if d - lastlog >= 5.0:
            lastlog = d
            rospy.loginfo('  指令 d=%.1fm | pos x=%.2f y=%.2f z=%.2f | 实际 %.1fm',
                          d, p.x, p.y, p.z, moved)
        if d >= dist:
            break
    hold(Point(p0.x + math.cos(yaw) * dist, p0.y + math.sin(yaw) * dist, p0.z), yaw, 1.5)


def turn(p_hold, y0, y1):
    """位置不动, 只把 yaw 从 y0 平滑插值到 y1 (PX4 原地转向)"""
    rospy.loginfo('--- 转向 %.1f° -> %.1f° ---', math.degrees(y0), math.degrees(y1))
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        f = min((rospy.Time.now() - t0).to_sec() / TURN_SEC, 1.0)
        set_pos(p_hold); set_yaw(y0 + (y1 - y0) * f); pub.publish(sp); rate.sleep()
        if f >= 1.0:
            break
    hold(p_hold, y1, 1.5)


def main():
    rospy.loginfo('等待 FCU ...')
    while not rospy.is_shutdown() and not state.connected:
        rate.sleep()
    rospy.sleep(1.0)

    q = cur.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    p0 = Point(cur.pose.position.x, cur.pose.position.y, cur.pose.position.z)
    z0 = p0.z
    rospy.loginfo('起点 x=%.2f y=%.2f z=%.2f yaw=%.1f°', p0.x, p0.y, z0, math.degrees(yaw))

    # 预发 setpoint (>=2s) 才允许切 OFFBOARD
    set_pos(p0); set_yaw(yaw)
    for _ in range(int(20 * PRE_STREAM_SEC)):
        pub.publish(sp); rate.sleep()

    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/arming')
    sm = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    ar = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    if state.mode != 'OFFBOARD':
        rospy.loginfo('切 OFFBOARD (当前 %s)', state.mode)
        sm(custom_mode='OFFBOARD')
    if not state.armed:
        rospy.loginfo('Arm')
        ar(True)
    rospy.sleep(0.5)

    pt = Point(p0.x, p0.y, z0)
    for seg, val in SEGMENTS:
        if seg == 'fwd':
            fly(pt, yaw, val, '前进 %.0fm' % val)
            pt = Point(pt.x + math.cos(yaw) * val, pt.y + math.sin(yaw) * val, z0)
        elif seg in ('right', 'left'):
            d = -1.0 if seg == 'right' else 1.0    # ENU: 逆时针(yaw+)为左
            y1 = yaw + d * math.radians(val)
            turn(pt, yaw, y1)
            yaw = y1
        elif seg == 'hold':
            hold(pt, yaw, val)
        else:
            rospy.logwarn('未知段类型: %s', seg)

    p = cur.pose.position
    rospy.loginfo('=' * 55)
    rospy.loginfo('✅ 机动完成: 终点 x=%.2f y=%.2f z=%.2f', p.x, p.y, p.z)
    rospy.loginfo('   起终点直线距离 %.2f m (起点 %.2f,%.2f)',
                  math.sqrt((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2), p0.x, p0.y)
    rospy.loginfo('保持悬停 %.0fs (之后脚本退出 -> offboard 超时可能 AUTO.LAND) ...', HOLD_SEC)
    hold(Point(p.x, p.y, z0), yaw, HOLD_SEC)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
