# odin1 采集流程与直出配置（2026-08 实测，fouruav）

## control_command.yaml 直出开关（`~/odin_ws/src/odin_ros_driver/config/control_command.yaml`）

| 开关 | 值(默认) | 话题 | 说明 |
|---|---|---|---|
| sendrgbcompressed | 1 | `/odin1/image/compressed` | JPEG 彩图 |
| sendrgb | 1 | `/odin1/image` | bgr8 解码图，**pcd2depth_node 订阅的是它不是 compressed** |
| sendrgbundistort | 0 | `/odin1/image/undistorted` | 去畸变图（关） |
| sendimu | 1 | `/odin1/imu` | IMU |
| sendodom | 1 | `/odin1/odometry` | 位姿 |
| send_odom_baselink_tf | 1 | /tf | RViz 显示 cloud_raw 必需 |
| senddtof | 1 | `/odin1/cloud_raw` | 点云（10fps 或 14.5fps，dtof_fps 100/145） |
| sendcloudslam | 1 | `/odin1/cloud_slam` | SLAM 地图 |
| sendcloudrender | 1 | 彩色渲染点云 | 主机处理 |
| senddepth | 0 | `/odin1/depth_img_competetion` + `_cloud` | **深度图（拼写是 competetion）**；吃算力，Jetson NX 慎开 |
| sendreprojection | 0 | `/odin1/reprojected_image` | cloud_slam 重投影到图像（需 /odin1/wiwc） |
| sendoverlay | 0 | `/odin1/overlay_image` | 叠加图（订阅 reprojected + undistorted） |
| **recorddata** | **1** | 无话题 | **🎯 olx 直出录制**：rgb+odometry+slam cloud 录成私有 olx 格式 → `~/odin_ws/src/odin_ros_driver/recorddata/<开始时间>/` |

**recorddata=1 就是用户丢失的 .olx/.bin 文件的来源**——采集完必须整个文件夹拷走（SDK 注释强调 "please copy the full folder"）。

## 录 bag 命令（含全部直出话题）

```bash
rosbag record -O out.bag \
  /odin1/cloud_raw /odin1/cloud_slam \
  /odin1/image/compressed /odin1/image \
  /odin1/depth_img_competetion /odin1/depth_img_competetion_cloud \
  /odin1/reprojected_image /odin1/overlay_image \
  /odin1/odometry /odin1/imu /tf /tf_static
```

## 一键采集脚本（start_record.sh，用户要求"弱智也能操作"）

操作人只做 3 步：开终端 → `bash ~/start_record.sh` → 走 → Ctrl+C。脚本自动：sed 开直出开关 → pkill 旧驱动 → nohup roslaunch odin1_ros1.launch → sleep 12 → rostopic list 确认 → rosbag record（Ctrl+C 时 trap 自动关驱动并打印 bag 路径 + olx 目录提示）。

用户偏好：**采集操作给"对面是弱智"级傻瓜指令**（一键脚本 + 3 步说明），不要给原理。用户说"直接给我"= 完整脚本内容直接贴聊天里，别只说"已部署"。

## 性能参考（SLAM 地图投影深度图，rgbd_all.py）

- 投影耗时 ≈ 每帧 1.5~1.8s × slam 点数规模（2238 万点地图 → 5306 帧 ≈ 2.6 小时/单 bag）
- 多进程并行可 ×3~4，但**用户选择继续等**——尊重，别反复推销优化
