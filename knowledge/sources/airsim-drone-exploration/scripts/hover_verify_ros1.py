#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  OFFBOARD 定点悬停验证 —— ROS1 (Noetic) + MAVROS + PX4 + AirSim
#
#  用途：验证"定点模式真的能用"的最严格一层。
#        起飞到目标点 -> 悬停 N 秒 -> 测量最大漂移 -> 自动 LAND。
#
#  与 ROS2 版 (scripts/offboard_planner_test.py) 的分工：
#    - offboard_planner_test.py = ROS2/lyuav，飞方形航线（验证"环境可被算法驱动"）
#    - 本脚本                    = ROS1/ros1-twin，原地悬停+漂移量化（验证"定点"）
#
#  跑法（MAVROS / PX4 / AirSim Blocks 都开着）：
#      source /opt/ros/noetic/setup.bash
#      python3 ~/hover_verify.py            # 先跑 MAVROS：见下方 fcu_url
#
#  MAVROS 起法（ROS1，ros1-twin）—— 推荐走「PX4 专用 MAVLink 专线」：
#      1) 先在 PX4 控制台(tmux px4)开专线：
#         mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard
#      2) source /opt/ros/noetic/setup.bash && source ~/twin_ws/devel/setup.bash
#         roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610
#  ⚠️ 默认 fcu_url:=udp://:14540@127.0.0.1:14580 实测会报
#     `[FATAL] FCU: DeviceError:udp:bind: Address already in use`（14540 被占）
#     -> 改用上面的专线端口（先 ss -lunp | grep 14540 核占用）。
#  ⚠️ MAVROS 若「秒退」并报
#     `[FATAL] UAS: GeographicLib exception: File not readable
#      /usr/share/GeographicLib/geoids/egm96-5.pgm`
#     -> 缺 geoid 数据：sudo geographiclib-get-geoids egm96-5，
#        或从同机已有该数据的发行版直接拷 /usr/share/GeographicLib/geoids/*。
#  起没起来看：rosnode list | grep mavros  +  rostopic echo -n 1 /mavros/state
#  （`topic ... does not appear to be published yet` = MAVROS 没在跑，不是连不上）
#
#  判据：悬停期间三维最大漂移 < 0.5 m  =>  ✅ 定点正常
#        > 0.5 m / 持续单向漂移        =>  ⚠️ 查定位源（EKF / vision_pose）
# ============================================================
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

TARGET = (0.0, 0.0, 5.0)   # 目标点 x y z (m)
HOVER_SEC = 20.0           # 悬停观察时长
RATE_HZ = 20.0

state = State()
cur = PoseStamped()


def cb_state(msg):
    global state
    state = msg


def cb_pose(msg):
    global cur
    cur = msg


def main():
    rospy.init_node('hover_verify', anonymous=True)
    rospy.Subscriber('/mavros/state', State, cb_state)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_pose)
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(RATE_HZ)

    rospy.loginfo('等待 FCU 连接 ...')
    while not rospy.is_shutdown() and not state.connected:
        rate.sleep()
    rospy.loginfo('FCU 已连接, mode=%s', state.mode)

    sp = PoseStamped()
    sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = TARGET
    sp.pose.orientation.w = 1.0

    # 预发 setpoint(>2s) 才允许切 OFFBOARD —— PX4 的硬性前提
    rospy.loginfo('预发 setpoint 3s ...')
    for _ in range(int(RATE_HZ * 3)):
        pub.publish(sp)
        rate.sleep()

    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/arming')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    rospy.loginfo('切 OFFBOARD ...')
    set_mode(custom_mode='OFFBOARD')
    rospy.loginfo('Arm ...')
    arming(True)
    rospy.sleep(1.0)

    rospy.loginfo('悬停 %.0fs, 目标 %s ...', HOVER_SEC, str(TARGET))
    t0 = rospy.Time.now()
    samples = []
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < HOVER_SEC:
        pub.publish(sp)
        rate.sleep()
        p = cur.pose.position
        samples.append((p.x, p.y, p.z))
        rospy.loginfo('pos x=%.3f y=%.3f z=%.3f  mode=%s', p.x, p.y, p.z, state.mode)

    if samples:
        # ⚠️ 只统计「到达目标后的稳态段」——否则起飞爬升(z: 0 -> TARGET[2])会被算成漂移，
        #    实测误报 “最大漂移 5.000 m / ⚠️ 漂移偏大”(目标高度就是 5m)。
        #    稳态样本不足 10 个才退回全样本。
        stable = [s for s in samples if s[2] > TARGET[2] - 0.5]
        if len(stable) < 10:
            stable = samples
        xs = [s[0] for s in stable]
        ys = [s[1] for s in stable]
        zs = [s[2] for s in stable]
        drift = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        rospy.loginfo('=' * 50)
        rospy.loginfo('(稳态段 %d/%d 样本)', len(stable), len(samples))
        rospy.loginfo('x 范围: %.3f ~ %.3f', min(xs), max(xs))
        rospy.loginfo('y 范围: %.3f ~ %.3f', min(ys), max(ys))
        rospy.loginfo('z 范围: %.3f ~ %.3f', min(zs), max(zs))
        rospy.loginfo('最大漂移: %.3f m', drift)
        rospy.loginfo('判定: %s',
                      '✅ 定点正常(<0.5m)' if drift < 0.5 else '⚠️ 漂移偏大, 检查定位')

    rospy.loginfo('落地 (LAND) ...')
    set_mode(custom_mode='AUTO.LAND')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
