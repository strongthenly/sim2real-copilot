#!/usr/bin/env python3
"""最小 MAVROS offboard 规划器 —— 验证 AirSim+PX4 无人机能被"算法"驱动。

用途：
  1) 验证仿真环境是否可被算法驱动（发 setpoint → MAVROS → PX4 → AirSim 无人机飞）
  2) 作为探索/规划控制器的起点模板（把方形航点换成你的 frontier/规划器输出即可）

前提：MAVROS 已连上 PX4（`/mavros/state connected: true`）、PX4 SITL + AirSim Play 都开着。
跑法（WSL）：
  source /opt/ros/jazzy/setup.bash
  python3 offboard_planner_test.py

判读：
  - 日志出现 `-> set OFFBOARD` → `-> arm` → `-> target ...`
  - AirSim 里无人机解锁 → 升到 5m → 飞方形航线
  - QGC 里模式变 OFFBOARD、高度变化
常见失败：
  - 一直不 arm：PX4 preflight 未过（SITL 常有 `no heading reference` 警告——通常不影响）
  - 切 OFFBOARD 被拒：setpoint 流必须持续 >1-2s 才允许切；本脚本在 2s（40 * 0.05s）后切
  - 需先设 home：QGC 里在地图右键 "Set home here"，或让 PX4 用 SDK 起点的 home
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class OffboardPlanner(Node):
    def __init__(self):
        super().__init__('offboard_planner')
        self.state = State()
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        # 方形航线（ENU，相对 home，单位 m）——替换成你的规划器输出
        self.targets = [(0, 0, 5), (5, 0, 5), (5, 5, 5), (0, 5, 5), (0, 0, 5)]
        self.idx = 0
        self.n = 0
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz：PX4 offboard 要求持续流
        self.get_logger().info('offboard planner started; targets=%s' % str(self.targets))

    def state_cb(self, msg):
        self.state = msg

    def pose(self, x, y, z):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.w = 1.0
        return p

    def tick(self):
        self.n += 1
        x, y, z = self.targets[self.idx]
        self.pub.publish(self.pose(x, y, z))          # 持续 20Hz 发 setpoint

        if self.n == 40:                               # 流 2s 后切 OFFBOARD（PX4 要求先有流）
            if self.state.mode != 'OFFBOARD':
                r = SetMode.Request(); r.custom_mode = 'OFFBOARD'
                self.mode_cli.call_async(r)
                self.get_logger().info('-> set OFFBOARD')
        if self.n % 100 == 0:                          # 每 5s 尝试 arm
            if not self.state.armed:
                r = CommandBool.Request(); r.value = True
                self.arm_cli.call_async(r)
                self.get_logger().info('-> arm')
        if self.n % 300 == 0:                          # 每 15s 换下一个航点
            self.idx = (self.idx + 1) % len(self.targets)
            self.get_logger().info('-> target %s' % str(self.targets[self.idx]))
        if self.n % 200 == 0:
            self.get_logger().info('state: mode=%s armed=%s' % (self.state.mode, self.state.armed))


def main():
    rclpy.init()
    n = OffboardPlanner()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
