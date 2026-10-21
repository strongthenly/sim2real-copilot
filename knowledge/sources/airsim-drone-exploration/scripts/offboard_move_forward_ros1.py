#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offboard 沿机头方向前进 N 米（ROS1 + MAVROS + PX4），保持当前高度。

用途：
  * 需要"让无人机动一下"来做验证（观察 RViz 轨迹/地图是否跟随、相机视角变化等）
  * 控制回路（MAVROS offboard → PX4）的最小可用动作模板

做法：
  1. 读 /mavros/local_position/pose 的四元数 → yaw
  2. 目标 = 当前位置 + (cos yaw, sin yaw, 0) * DIST，z 保持当前
  3. 预发 setpoint 3s（切 OFFBOARD 的前置条件）→ 切 OFFBOARD → 未解锁则 arm
  4. 持续发 setpoint 直到 err<0.2m（或 25s 超时），期间打印「已移动 / 距目标」

实测（ros1-twin + AirSim，2026-09-10）：
  目标 2.000m → 实到 1.855m（误差 0.185m，判据 err<0.2 命中）；
  z 2.98→2.98 稳住；模式 OFFBOARD。机头朝向世界 +x 时表现为只动 x。

前置：
  * MAVROS 已连上（见 SKILL.md「ROS1 MAVROS 秒退两坑」：geoid + 专用 mavlink 14610/14611）
  * 无人机已在空中（本会话是 hover_verify 后的 AUTO.LOITER 3m 悬停态）

用法：
  source /opt/ros/noetic/setup.bash && source <ws>/devel/setup.bash
  python3 offboard_move_forward_ros1.py [DIST]
"""
import sys
import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0

state = State()
cur = PoseStamped()


def cb_s(m):
    global state
    state = m


def cb_p(m):
    global cur
    cur = m


def main():
    rospy.init_node('move_forward', anonymous=True)
    rospy.Subscriber('/mavros/state', State, cb_s)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_p)
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20)

    rospy.loginfo('等待 FCU ...（卡住 = MAVROS 没起，不是脚本问题）')
    while not rospy.is_shutdown() and not state.connected:
        rate.sleep()
    rospy.sleep(1.5)

    q = cur.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    p0 = cur.pose.position
    rospy.loginfo('当前位置: x=%.3f y=%.3f z=%.3f  yaw=%.1f°', p0.x, p0.y, p0.z, math.degrees(yaw))

    tx = p0.x + DIST * math.cos(yaw)
    ty = p0.y + DIST * math.sin(yaw)
    tz = p0.z
    rospy.loginfo('目标(前进 %.1fm): x=%.3f y=%.3f z=%.3f', DIST, tx, ty, tz)

    sp = PoseStamped()
    sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = tx, ty, tz
    sp.pose.orientation.w = 1.0

    rospy.loginfo('预发 setpoint 3s ...')
    for _ in range(60):
        pub.publish(sp)
        rate.sleep()

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

    rospy.loginfo('开始前进 ...')
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 25:
        pub.publish(sp)
        rate.sleep()
        p = cur.pose.position
        err = math.sqrt((p.x - tx) ** 2 + (p.y - ty) ** 2 + (p.z - tz) ** 2)
        moved = math.sqrt((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2)
        rospy.loginfo('pos x=%.3f y=%.3f z=%.3f | 已移动 %.3fm | 距目标 %.3fm', p.x, p.y, p.z, moved, err)
        if err < 0.20:
            rospy.loginfo('=' * 50)
            rospy.loginfo('✅ 已到达! 实际移动 %.3f m, 误差 %.3f m', moved, err)
            break

    rospy.loginfo('悬停保持 5s ...')
    t1 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t1).to_sec() < 5:
        pub.publish(sp)
        rate.sleep()
    rospy.loginfo('完成（保持 OFFBOARD 悬停）')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
