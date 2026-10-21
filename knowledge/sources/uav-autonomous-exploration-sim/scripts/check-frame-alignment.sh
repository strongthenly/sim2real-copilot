#!/bin/bash
# ============================================================
#  坐标系对齐诊断：对比 FAST-LIVO2(camera_init) 与 PX4 EKF(ENU)
#
#  用途：算法驱动的无人机「飞错方向 / 疯狂撞墙」时的第一步。
#  只读，不发任何控制指令 —— 安全，可以在无人机停着时跑。
#
#  用法: bash check-frame-alignment.sh [master_uri]
#        SAMPLES=20 bash check-frame-alignment.sh        # 连续采样次数(默认 12)
#  判读(★ 以【连续采样/残差法】为准；单次采样在无人机旋转时会骗你):
#        稳定样本的平均残差 |p_px4 - Rz(psi)·p_fuel| < 0.3m ⇒ 对齐正确
#        残差 > 1m ⇒ psi 符号可能反了（试 psi_offset_deg:=-psi 手动指定）
#        单次采样 YAW 偏差 > 10° 且位置偏差 < 0.5m ⇒ 典型『朝向没对齐』(=撞墙头号原因)
# ============================================================
export ROS_MASTER_URI="${1:-http://127.0.0.1:11311}"
# 按需改这两个话题名
FUEL_POSE_TOPIC="${FUEL_POSE_TOPIC:-/aft_mapped_to_init}"   # nav_msgs/Odometry，算法侧位姿
PX4_POSE_TOPIC="${PX4_POSE_TOPIC:-/mavros/local_position/pose}"  # geometry_msgs/PoseStamped

source /opt/ros/noetic/setup.bash 2>/dev/null

python3 - <<'PY'
import math, os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

fuel_topic = os.environ.get('FUEL_POSE_TOPIC', '/aft_mapped_to_init')
px4_topic  = os.environ.get('PX4_POSE_TOPIC', '/mavros/local_position/pose')

rospy.init_node('check_frame_alignment', anonymous=True, disable_signals=True)

def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def wrap(d):
    while d > 180: d -= 360
    while d < -180: d += 360
    return d

fuel = px4 = None

try:
    o = rospy.wait_for_message(fuel_topic, Odometry, timeout=10)
    p, q = o.pose.pose.position, o.pose.pose.orientation
    fuel = (p.x, p.y, p.z, math.degrees(yaw_of(q)))
    print("=== 算法侧  %s (frame=camera_init/world) ===" % fuel_topic)
    print("   pos = (%.3f, %.3f, %.3f)" % fuel[:3])
    print("   yaw = %.2f deg" % fuel[3])
except Exception as e:
    print("=== 算法侧无数据: %s" % e)
    print("    (先起 SLAM/里程计链路)")

try:
    m = rospy.wait_for_message(px4_topic, PoseStamped, timeout=10)
    p, q = m.pose.position, m.pose.orientation
    px4 = (p.x, p.y, p.z, math.degrees(yaw_of(q)))
    print("")
    print("=== PX4/MAVROS  %s (frame=ENU) ===" % px4_topic)
    print("   pos = (%.3f, %.3f, %.3f)" % px4[:3])
    print("   yaw = %.2f deg" % px4[3])
except Exception as e:
    print("")
    print("=== PX4 无数据: %s" % e)
    print("    (MAVROS 没起 / 没连上 PX4)")

print("")
print("========== 单次采样（快，但无人机旋转时不可信）==========")
if fuel and px4:
    dx, dy, dz = (fuel[0]-px4[0]), (fuel[1]-px4[1]), (fuel[2]-px4[2])
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    dyaw = wrap(fuel[3] - px4[3])
    print("位置偏差 : dx=%.3f  dy=%.3f  dz=%.3f  (模长 %.3f m)" % (dx, dy, dz, dist))
    print("YAW 偏差 : %.2f deg   <-- 关键！" % dyaw)
    if abs(dyaw) > 10:
        print(">>> 朝向差 %.0f 度：算法目标点会被 PX4 朝错误方向执行 —— 『撞墙』头号原因" % dyaw)
        print(">>> 修法：输出桥里做 yaw 对齐（见下方连续采样/残差法确认）")
    elif dist > 0.5:
        print(">>> 朝向基本一致，但原点差 %.2f m：加平移对齐（起飞时记一次位置差）" % dist)
    else:
        print(">>> 看起来已对齐 —— 但请以下方【连续采样】复核，再查防撞参数")
else:
    print("数据不全 —— 先确保 SLAM 链路 + MAVROS 都跑着")

# ---------- 连续采样 + 残差法（★ 判据以此为准） ----------
# 原理：对齐正确 ⇒ p_px4 ≈ Rz(psi)·p_fuel，其中 psi = wrap(yaw_px4 - yaw_fuel)
# 为什么要多次采样：两个话题的 yaw 来自不同时刻，无人机一旋转，单次做差就没意义。
if fuel and px4:
    print("")
    print("========== 连续采样（残差法，判据以此为准）==========")
    N = int(os.environ.get('SAMPLES', '12'))
    rows = []
    prev_yaw = None
    for i in range(N):
        try:
            o = rospy.wait_for_message(fuel_topic, Odometry, timeout=3)
            m = rospy.wait_for_message(px4_topic, PoseStamped, timeout=3)
        except Exception:
            break
        f, x = o.pose.pose, m.pose
        yf, yp = yaw_of(f.orientation), yaw_of(x.orientation)
        psi = yp - yf
        exp_x = f.position.x*math.cos(psi) - f.position.y*math.sin(psi)
        exp_y = f.position.x*math.sin(psi) + f.position.y*math.cos(psi)
        err = math.hypot(x.position.x - exp_x, x.position.y - exp_y)
        rotating = (prev_yaw is not None) and (abs(wrap(math.degrees(yp - prev_yaw))) > 5.0)
        prev_yaw = yp
        rows.append((err, math.degrees(psi), rotating))
        print("  #%02d  psi=%7.1f deg   残差=%6.3f m  %s"
              % (i+1, math.degrees(psi), err, "<< 姿态在转，忽略" if rotating else ""))
        rospy.sleep(0.5)

    good = [r for r in rows if not r[2]]
    if good:
        avg_err = sum(r[0] for r in good) / len(good)
        avg_psi = sum(r[1] for r in good) / len(good)
        print("  稳定样本 %d/%d：psi ≈ %.2f deg   平均残差 = %.3f m"
              % (len(good), len(rows), avg_psi, avg_err))
        if avg_err < 0.3:
            print(">>> 对齐正确（残差 <0.3m）：算法目标已被正确变换")
            print(">>> 还撞墙 ⇒ 去查防撞参数（obstacles_inflation / max_ray_length / clearance_threshold / max_vel）")
        elif avg_err < 1.0:
            print(">>> 基本对齐（残差 0.3~1m）：多为滞后，可接受")
        else:
            print(">>> 对齐异常（残差 >1m）：可能 psi 符号反了 ⇒ 试 psi_offset_deg:=%.2f 手动指定" % (-avg_psi))
    elif rows:
        print(">>> 样本全在旋转中，无法判定 —— 让无人机悬停稳了再跑")
    else:
        print(">>> 拿不到连续样本（链路可能断了）")
PY
