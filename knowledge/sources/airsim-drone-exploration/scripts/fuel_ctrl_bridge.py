#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FUEL 输出控制桥:  FUEL  →  MAVROS/PX4        （2026-09-12 验证通过）

  订阅:  /planning/pos_cmd              (quadrotor_msgs/PositionCommand)
         /mavros/local_position/pose    (geometry_msgs/PoseStamped)  ← seed 悬停用
         /mavros/state                  (mavros_msgs/State)
  发布:  /mavros/setpoint_position/local (geometry_msgs/PoseStamped, ENU)  mode=position
         /mavros/setpoint_raw/local      (mavros_msgs/PositionTarget)      mode=raw

坐标系: FUEL 的 position 在 camera_init(ENU)。MAVROS 的 setpoint_position/local 在 ROS 侧
        同样是 ENU，MAVROS 内部自动转 PX4 的 NED —— 所以**不做手工 ENU↔NED 转换**（少一个错源）。

★ seed 悬停（关键，解决时序矛盾）:
     FUEL 触发前不输出轨迹 → 本桥收不到 pos_cmd → 若"没指令就不发 setpoint"，
     PX4 无法进 OFFBOARD → **无人机飞不起来** → 也就永远触发不了。
     所以收到首条 FUEL 指令前，自动发 (当前x, 当前y, seed_z) 的 setpoint：
     先把无人机托到 seed_z 并切 OFFBOARD；一旦 FUEL 出轨迹就无缝接管。

模式:  ~mode=position (默认, 稳) 位置+yaw
       ~mode=raw         位置+速度+加速度+yaw（轨迹跟踪更准，需 PX4 加速度限制已配好）

用法:  rosrun fuel_bridge fuel_ctrl_bridge.py
       # 或经 launch: enable_ctrl:=true seed_z:=1.5 ctrl_mode:=position
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand

# MAVROS PositionTarget type_mask 的"忽略位"常量
IGN = dict(px=1, py=2, pz=4, vx=8, vy=16, vz=32,
           afx=64, afy=128, afz=256, force=512, yaw=1024, yaw_rate=2048)


def yaw_to_quat(yaw):
    """把 yaw(rad) 转成只绕 z 轴的四元数 (x,y,z,w)"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class FuelCtrlBridge:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "position")            # position | raw
        self.auto_offboard = rospy.get_param("~auto_offboard", True)
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.offboard_wait = rospy.get_param("~offboard_wait", 3.0)  # 预发 setpoint 秒数
        self.seed_z = rospy.get_param("~seed_z", 1.5)                # >0 启用 seed 悬停

        self.state = State()
        self.last_cmd = None
        self.cur_pos = None      # 当前本地位置（ENU）
        self.seeded = False
        self.t0 = None           # 首次发出 setpoint 的时间
        self.sm = None
        self.ar = None

        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_cb)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cmd_cb)

        self.pub_pos = rospy.Publisher("/mavros/setpoint_position/local",
                                       PoseStamped, queue_size=10)
        self.pub_raw = rospy.Publisher("/mavros/setpoint_raw/local",
                                       PositionTarget, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)
        rospy.loginfo("[fuel_ctrl] 控制桥启动 mode=%s rate=%.0fHz auto_offboard=%s seed_z=%.2f",
                      self.mode, self.rate_hz, self.auto_offboard, self.seed_z)

    # ---------------- callbacks ----------------
    def state_cb(self, msg):
        self.state = msg

    def pos_cb(self, msg):
        self.cur_pos = msg.pose.position

    def cmd_cb(self, msg):
        self.last_cmd = msg

    # ---------------- seed 悬停 ----------------
    def publish_seed(self):
        if self.seed_z <= 0 or self.cur_pos is None:
            return
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "world"
        sp.pose.position.x = self.cur_pos.x
        sp.pose.position.y = self.cur_pos.y
        sp.pose.position.z = self.seed_z
        sp.pose.orientation.w = 1.0
        self.pub_pos.publish(sp)
        if not self.seeded:
            self.seeded = True
            rospy.loginfo("[fuel_ctrl] seed 悬停开始: 抬到 %.2f m 等 FUEL 轨迹接管", self.seed_z)
        if self.t0 is None:
            self.t0 = rospy.Time.now()

    # ---------------- 切 OFFBOARD + Arm（seed / FUEL 两阶段共用） ----------------
    def try_offboard(self):
        if not self.auto_offboard or not self.state.connected or self.t0 is None:
            return
        if (rospy.Time.now() - self.t0).to_sec() < self.offboard_wait:
            return  # 预发阶段：PX4 硬要求先连续收到 setpoint 才允许切 OFFBOARD
        if self.sm is None:
            # ⚠️ 不能做阻塞式服务等待（wait_for_service 会卡住 50Hz 的 tick 回调）。
            # 惰性建 ServiceProxy，服务不存在时调用抛异常、被下面捕获。
            self.sm = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self.ar = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            rospy.loginfo("[fuel_ctrl] set_mode/arming 代理已创建")
        if self.state.mode != "OFFBOARD":
            try:
                self.sm(custom_mode="OFFBOARD")
                rospy.loginfo("[fuel_ctrl] → 切 OFFBOARD")
            except Exception as e:
                rospy.logwarn_throttle(5.0, "[fuel_ctrl] set_mode 失败: %s", e)
        elif not self.state.armed:
            try:
                self.ar(True)
                rospy.loginfo("[fuel_ctrl] → Arm")
            except Exception as e:
                rospy.logwarn_throttle(5.0, "[fuel_ctrl] arming 失败: %s", e)

    # ---------------- main loop ----------------
    def tick(self, _):
        cmd = self.last_cmd
        if cmd is None:
            # 还没收到 FUEL 指令 → seed 悬停（先托起来，才能切 OFFBOARD）
            self.publish_seed()
            self.try_offboard()
            return

        p = cmd.position
        yaw = cmd.yaw
        if self.t0 is None:
            self.t0 = rospy.Time.now()
            rospy.loginfo("[fuel_ctrl] 收到首条 FUEL 指令，FUEL 轨迹接管")

        if self.mode == "raw":
            tgt = PositionTarget()
            tgt.header.stamp = rospy.Time.now()
            tgt.header.frame_id = "world"
            tgt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            tgt.type_mask = IGN['force'] | IGN['yaw_rate']   # 用 pos+vel+acc+yaw
            tgt.position.x, tgt.position.y, tgt.position.z = p.x, p.y, p.z
            tgt.velocity.x, tgt.velocity.y, tgt.velocity.z = \
                cmd.velocity.x, cmd.velocity.y, cmd.velocity.z
            tgt.acceleration_or_force.x = cmd.acceleration.x
            tgt.acceleration_or_force.y = cmd.acceleration.y
            tgt.acceleration_or_force.z = cmd.acceleration.z
            tgt.yaw = yaw
            tgt.yaw_rate = 0.0
            self.pub_raw.publish(tgt)
        else:
            sp = PoseStamped()
            sp.header.stamp = rospy.Time.now()
            sp.header.frame_id = "world"
            sp.pose.position.x = p.x
            sp.pose.position.y = p.y
            sp.pose.position.z = p.z
            qx, qy, qz, qw = yaw_to_quat(yaw)
            sp.pose.orientation.x = qx
            sp.pose.orientation.y = qy
            sp.pose.orientation.z = qz
            sp.pose.orientation.w = qw
            self.pub_pos.publish(sp)

        self.try_offboard()


if __name__ == "__main__":
    rospy.init_node("fuel_ctrl_bridge")
    FuelCtrlBridge()
    rospy.spin()
