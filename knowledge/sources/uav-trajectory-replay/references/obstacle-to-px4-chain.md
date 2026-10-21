# obstacle_to_px4 避障链路调试（B 分支 feat/obstacle-to-px4，2026-08-10 实机）

## 链路总览

```
点云 → obstacle_to_px4_node (36 扇区高度带滤波) → /mavros/obstacle/send (sensor_msgs/LaserScan)
     → MAVROS distance_sensor 插件 → MAVLink OBSTACLE_DISTANCE → PX4 避障
```

- launch：`roslaunch task_run_scripts obstacle_to_px4.launch drone:=mid360`
- 配置：`task_run_scripts/config/mid360.yaml`（cloud_topic / target_frame / num_bins / h_fov_deg / height_band / range / angle_sign / debug_markers）
- 源码：`task_run_scripts/src/obstacle_to_px4.cpp`——功能注释明确：点云→机体系→高度带滤波→水平扇区最近距离→LaserScan→/mavros/obstacle/send
- 输出话题：`/mavros/obstacle/send`（LaserScan，~20Hz）、`/obstacle_to_px4/sector_marker`（RViz 扇形可视化）、`/obstacle_to_px4/status`（String，如 `min_dist=0.36 m no_cover=0/36 cloud_age=0.00s`）

## 坑 1：`no cloud received yet, stop publishing`（TF 帧缺失，2026-08-10 实机）

**现象**：节点启动正常、参数加载对，但一直 WARN `no cloud received yet`。

**根因**：launch 里静态变换是 `livox_frame → aft_mapped`，但 **TF 树里没有 `livox_frame` 这个帧**（FAST-LIVO2 不发布它）。原始 `/livox/lidar` 点云在 livox_frame 系，节点 `lookupTransform(target_frame=aft_mapped, msg.frame=livox_frame)` 失败 → 丢点云。

**关键事实（实测）**：
- `/livox/lidar` 的 frame 是 `livox_frame`，但该帧在 TF 树中不存在
- FAST-LIVO2 发布的动态 TF 是 `camera_init → aft_mapped`（`/aft_mapped_to_init` 消息：frame_id=camera_init, child=aft_mapped）
- `/cloud_registered` 的 frame 是 `camera_init` → **camera_init→aft_mapped 变换可用**（rosrun tf tf_echo camera_init aft_mapped 有输出）

**修复**：`config/mid360.yaml` 的 `cloud_topic` 从 `/livox/lidar` 改为 `/cloud_registered`（FAST-LIVO2 畸变校正后点云，camera_init 系）。节点 lookupTransform 变成 `aft_mapped → camera_init`（存在）→ 立即工作。

**判读**：日志出现 `cloud pts=3905 processed=229 first=(2.65,-0.26,-0.16)` + `pub bin0=0.45 bin4=0.64 ... min=0.37` = 正常。备份：mid360.yaml.bak_livox。

**注意**：`angle_sign=-1.0` 是 PX4 俯视顺时针约定，若左右镜像改为 1.0；`debug_markers: true` 时 RViz 看 sector_marker 定位障碍扇区。

## 坑 2：数据正常但 PX4 不避障（COM_OB_ACT=0）

**链路最后一段是 PX4 参数开关**，默认关闭：
- `rosservice call /mavros/param/get "param_id: 'COM_OB_ACT'"` → 0 = 关闭（出厂默认）
- 相关参数：COM_OB_RADIUS（生效半径）、COM_OB_W_HGT（高度带宽）、COM_OB_PRM_AV（安全裕度）
- MPC_POS_MODE=4（位置控制，已配好不用动）

**开启**（QGC 或命令）：
```bash
rosservice call /mavros/param/set "param_id: 'COM_OB_ACT' value: {integer: 1}"
rosservice call /mavros/param/set "param_id: 'COM_OB_RADIUS' value: {integer: 5}"
```
⚠️ 开启后首次真机：小半径(2m)+低速，确认行为（减速/绕行）再加大。

**排查顺序**：/mavros/obstacle/send 有 20Hz 数据 → MAVROS 插件已加载（px4_pluginlists.yaml 有 distance_sensor）→ 最后查 PX4 COM_OB_ACT。**obstacle_to_px4 节点工作 ≠ PX4 在避障**，别把节点正常当成避障生效。

## 坑 3：min_dist 持续很小（如 0.36m）

可能：机架/雷达自身遮挡、测试环境物体太近。RViz 看 sector_marker 定位是哪个扇区；确认是自身遮挡再考虑调 range_min 或加最小距离滤波。
