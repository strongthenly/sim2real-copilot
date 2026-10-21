# PX4-ROS2 接入细节 + 主流 UAV 探索做法（实测记录）

本节是 `wsl2-ros2-setup` SKILL.md 第 8/9 节的**会话级**补充（错误信息、正确仓库、证据），供深挖时对照。

## 关键仓库名（全是坑，实测确认）
| 用途 | 正确仓库 / 命令 | 错误（坑） |
|---|---|---|
| PX4 本体 | `https://github.com/PX4/PX4-Autopilot.git`（`make px4_sitl gz_x500` 编译） | — |
| PX4 Fast-RTPS bridge | **`PX4/px4_ros_com`** + **`PX4/px4_msgs`**（同放一个 ws 的 `src/` 一起 `colcon build`） | ❌ `PX4/PX4-ROS2-bridge`（不存在，404） |
| uXRCE agent | **`micro-ROS/micro-ROS-Agent`**（大小写/连字符：`micro-ROS-Agent`） | ❌ `micro-ROS/micro_ros_agent`（不存在） |

## PX4-ROS2 在 Jazzy 上的环境级死结（证据链）
1. 现代 PX4(main) **只走 uXRCE-DDS**：`pxh>` 里 `rtps status` → `Invalid command: rtps`（Fast-RTPS 已移除），故 `px4_ros_com`(Fast-RTPS) 也接不上。
2. uXRCE 的 agent（`micro_ros_agent` main）CMake **强制 `find_package(fastdds 3)`**：Jazzy 只有 `ros-jazzy-fastrtps`(Fast RTPS 2.11)，**没有 `ros-jazzy-fastdds`**（`apt` 报 `Unable to locate`）。
3. 装 `ros-rolling-fastdds`(v3.6.1) 提供 fastdds3，但混进 Jazzy 后 `colcon build micro_ros_agent` 报 **target 冲突**：
   ```
   CMake Error at .../fastrtps/shared-targets.cmake:42:
     Some (but not all) targets in this export set were already defined.
     Targets Defined: eProsima_atomic
     Targets not yet defined: fastrtps
   ```
   即 fastdds3 与 fastrtps2 **无法在同一 CMake 项目共存**（同含 `eProsima_atomic` 等 target）。
4. 退到 `micro-ROS-Agent` **v2.0.x**（用 fastrtps2）：`git clone --branch v2.0.4` 报 `Remote branch v2.0.4 not found` —— v2.0.4 是 **tag**，clone 后要先 `git fetch --tags` 再 `git checkout v2.0.4`。且 v2 agent 的 XRCE 协议比 PX4 main 旧，**很可能连不上**。

**结论**：在 Jazzy 上，PX4-SITL + ROS2(uXRCE) 通信是**环境级死结**，不是配置能绕开。要它真通需换 ROS2 rolling 或干净独立装 fastdds3（另一个项目）。

## 主流 UAV 探索做法（GitHub 调研）
- **HKUST-Aerial-Robotics/FUEL**（1471⭐，RA-L/ICRA 2021）：Gazebo `uav_simulator` 简化四旋翼 + 深度相机 + frontier/信息结构(FIS) + 分级规划；Ubuntu 18.04(Melodic)/20.04(Noetic)、`catkin_make`；RViz **2D Nav Goal** 触发（`exploration_manager/rviz.launch` + `exploration.launch`）。
- **Robotics-STAR-Lab/RACER**（789⭐，TRO 2023 最佳论文）：多机分布式探索，沿用 FUEL 式仿真栈。
- **Robotics-STAR-Lab/EPIC**（236⭐）：点云 LiDAR 轻量探索框架。
- 做法共性：**简化四旋翼动力学 + 探索算法直接发指令，不接 PX4**；用 RViz 2D Nav Goal 作为启动/触发。
- ROS1 为主；ROS2 复刻 = `gz-sim` `MulticopterControl` 简化四旋翼 + `ros_gz_bridge`(`/cmd_vel` ROS→gz、`/scan` gz→ROS) + 探索节点发 `/cmd_vel`。

## ROS2 简化四旋翼 SDF 结构（**用 gz-sim 官方 `multicopter_velocity_control.sdf`，响应 /cmd_vel**）

⚠️ **别用泛化的 `gz-sim` `MulticopterControl` 插件**（`gz-sim-multicopter-control-system` 的 `MulticopterControl`）。实测：它只让四旋翼悬停（`/scan` range 稳定不变 = 位置固定），但**不响应 `/cmd_vel` 的水平移动**（你发 `linear.x` 它原地不动）。要能飞又能动，用官方 `examples/worlds/multicopter_velocity_control.sdf` 的做法：**`MulticopterMotorModel`(每个旋翼一个) + `MulticopterVelocityControl`**。

关键点：
- **速度指令话题是 `<robotNamespace>/gazebo/command/twist`**（如 `/X3/gazebo/command/twist`，注意**带模型名前缀**，不是 `/cmd_vel`）→ `ros_gz_bridge` 里把 ROS `/cmd_vel` 的 `gz_topic_name` 指向它（`direction: ROS_TO_GZ`，`geometry_msgs/msg/Twist`→`gz.msgs.Twist`）。
- 每个旋翼一个 `MulticopterMotorModel`：`<jointName><namespace>/rotor_N_joint</jointName>`、`<linkName><namespace>/rotor_N</linkName>`、`<turningDirection>ccw/cw</turningDirection>`（交替）、`<maxRotVelocity>800</maxRotVelocity>`、`<motorConstant>8.54858e-06</motorConstant>`、`<momentConstant>0.016</momentConstant>`、`<commandSubTopic>gazebo/command/motor_speed</commandSubTopic>`、`<motorSpeedPubTopic>motor_speed/N</motorSpeedPubTopic>`、`<motorType>velocity</motorType>`。
- `MulticopterVelocityControl`（插件名 `gz::sim::systems::MulticopterVelocityControl`）：`<commandSubTopic>gazebo/command/twist</commandSubTopic>`、`<enableSubTopic>enable</enableSubTopic>`、`<comLinkName><namespace>/base_link</comLinkName>`、`<velocityGain>2.7 2.7 2.7</velocityGain>`、`<attitudeGain>2 3 0.15</attitudeGain>`、`<angularRateGain>0.4 0.52 0.18</angularRateGain>`、`<maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>`，再加 `<rotorConfiguration>`（4 个 `<rotor>`，每个给 `<jointName>`/`<forceConstant>8.54858e-06</forceConstant>`/`<momentConstant>0.016</momentConstant>`/`<direction>1|-1</direction>`，方向 0/1 与 2/3 相反）。
- ⚠️ **官方 X3/X4 模型通过 `<include>` 从 `fuel.gazebosim.org` 远程拉取，没法往远程模型里塞 LiDAR** → 自写模型 link/joint + 抄官方插件参数 + 在 `base_link` 加 `gpu_lidar`。加 `<include>` 前注意网络能拉到 fuel。
- world 里 `gz-sim-sensors-system` 插件建议写 `<render_engine>ogre2</render_engine>`（软渲染更稳）；`<physics><max_step_size>0.004</max_step_size>`（四旋翼对步长敏感）。

bridge.yaml 需含两条映射：`/scan`(GZ→ROS, `gz.msgs.LaserScan`→`sensor_msgs/msg/LaserScan`) 与 `/cmd_vel`(ROS→GZ, `geometry_msgs/msg/Twist`→`gz.msgs.Twist`，`gz_topic_name=/quadrotor/gazebo/command/twist`)。

**注意**：四旋翼动力学/控制器是 Gazebo 配置类问题，**无 canonical test，只能上机调**。判读"能不能动"用 **`/scan` range 是否随时间变化**（`ros2 topic echo /scan --once` 两次对比；不变 = 没动；变 = 在动）。

## 通用教训（本会话多次触发）
- **PowerShell 宿主跑 wsl 多行/含 `$`/括号的 `bash -lc "..."` 会因引号嵌套被 PowerShell 破坏**（报 `syntax error near unexpected token` 或变量被提前展开）。统一做法：**用 `write_file` 把脚本写到 `/mnt/c/...`（Windows 可写路径），然后 `wsl -d <distro> -- bash /mnt/c/路径/脚本.sh`**。本会话所有诊断/配置脚本都是这么跑的。
- 改 `.wslconfig`/apt 源/或 ROS/Gazebo `.sdf` 等配置文件：**先备份、保留用户原有行（如 NVIDIA JetPack 的 `kernel=`）、贴 diff 透明汇报**；`.wslconfig` 路径用正斜杠避免"转义字符无效"。
