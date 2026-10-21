# Gazebo 四旋翼 + 3D LiDAR world（gz-sim8）

用于给无人机探索算法做感知/驱动，四大件：world + 可移动四旋翼 + 3D LiDAR + bridge。

## 可移动四旋翼：用官方 MulticopterVelocityControl(不是自写 MulticopterControl)
官方示例 `gz-sim/examples/worlds/multicopter_velocity_control.sdf`。四旋翼的"飞"由两个系统组合：
- `gz-sim-multicopter-motor-model-system`（每 rotor 一个，连 motor_speed）
- `gz-sim-multicopter-control-system` = `MulticopterVelocityControl`（订阅 `/.../gazebo/command/twist` 速度指令）

**必踩坑：MulticopterVelocityControl 必须先发 `enable` 才接管**。没 enable 时四旋翼不动/不稳。
```bash
gz topic -t /quadrotor/gazebo/command/enable -m gz.msgs.Boolean -p "data: true"
```
launch 里自动 enable：
```python
EnableProcess(cmd=['gz','topic','-t','/quadrotor/gazebo/command/enable','-m','gz.msgs.Boolean','-p','data: true'])
LaunchDescription([gz, bridge, TimerAction(period=4.0, actions=[enable])])
```

## 控制指令话题
- 输入：`/quadrotor/gazebo/command/twist`（gz.msgs.Twist, 线速度+yaw）。注意**不是 `/cmd_vel`**。
- bridge 映射：`ros /cmd_vel -> gz /quadrotor/gazebo/command/twist`（ROS_TO_GZ, geometry_msgs/Twist<->gz.msgs.Twist）。

## 3D LiDAR（gpu_lidar）
- `gpu_lidar` 靠 GPU 渲染生成点云（WSL 下 libEGL denied 时数据仍出，只是可能慢/无 GUI）。
- 3D 配置：`<vertical><samples>16</samples><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></vertical>` + horizontal 360。
- gz topic `/lidar`（gz.msgs.LaserScan），bridge `GZ_TO_ROS -> /scan`(sensor_msgs/LaserScan)。

## LiDAR 几何坑（ranges 全 .inf）
LiDAR 悬停 z、垂直视野(±v)、障碍高度(h) 三者要匹配，否则扫不到。
- 例：无人机 z=2m、垂直 ±0.3rad → 在水平 3m 处垂直覆盖 z∈[2-0.93, 2+0.93]=[1.07,2.93]。墙高 1m(中心 z0.5) 就"全在视锥下方"→ `.inf`。
- 修：墙加高到 4m（z0~4），或降无人机、扩垂直视野。验证：`ros2 topic echo /scan --once` 看 ranges 非 `.inf`。

## 真实外观 X 型四旋翼（用户偏好）
用户要"真实外形"（非光杆 box）。SDF 里 base_link 放多个 visual：机身 box + 4 条 X 型臂(rotated thin box) + 4 电机座(cylinder) + 4 螺旋桨(rotor link, thin cylinder)。collision 可简化一个 box。

## 感知+位姿
- 位姿：`/mavros/local_position/odom`、`/mavros/local_position/pose`（PX4 定位，探索要）。
- 探索算法订阅 `/scan`(或 PointCloud2) + 位姿，发 setpoint/`/cmd_vel`。

## 与 PX4 结合
- 若用 PX4 真飞控（Gazebo+PX4+MAVROS）：PX4 控制它 spawn 的 `x500_0`，px4 的 gz_x500 不带 LiDAR（需改 x500_base 加 sensor）。
- 若用简化四旋翼（MulticopterVelocityControl）：不接 PX4，探索节点直接发 `/cmd_vel`。
