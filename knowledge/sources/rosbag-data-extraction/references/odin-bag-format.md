# odin 系统（留形科技）bag 处理细节

## 非标准 bag 格式（字节级）

odin SDK（`host_sdk_sample`，callerid 可见）录制的 ROS1 bag v2.0：
- 文件头记录：`#ROSBAG V2.0
` + u32 hlen + hdr + **u32 dlen + data（必须跳过）**
- header 字段 = 循环 `u32 len + "key=value"`，value 为裸字节：string 无长度前缀、u32 4B、u64 8B
- chunk 记录（op=0x05）：`compression` 裸字符串（如 "none"）、`size` 是 **4 字节 u32**
- 连接记录（op=0x07）：只有 `conn/op/topic`，**无 type/md5sum/message_definition**
- 消息记录（op=0x02）：`conn u32 + time u64（8B 小端，非标准 time 语义？实测按 sec+nsec 读更对）`，data=消息字节
- 消息体本身是标准 ROS 序列化（可手工解析 PointCloud2 等）

## 话题与消息布局

- `/odin1/cloud_raw`：sensor_msgs/PointCloud2，每帧 49152 点（MID360 全帧），fields = x,y,z,intensity(u8),confidence(u16),offset_time，frame_id=odin1_base_link
- `/odin1/cloud_slam`：sensor_msgs/PointCloud2，frame_id=odom，fields = x,y,z,**rgb(f32 打包)**，每帧 6k~20k 点（非累积式，帧间波动）
- `/odin1/image/compressed`：JPEG 1600×1296（120° 鱼眼）
- `/odin1/odometry`：nav_msgs/Odometry，frame_id=odom / child=odin1_base_link（雷达在 odom 系的位姿）
- 图像与 cloud_raw 同 header 发布（seq 相同、stamp 差 <0.1s），帧数一一对应

## 相机标定（fouruav `~/odin_ws/src/odin_ros_driver/config/calib.yaml`，设备 O1-N090100105）

- 模型：FishPoly（多项式鱼眼，maxIncidentAngle=120°）
- 内参：A11=730.1594, A12=-0.3840, A22=730.0633, u0=786.583, v0=647.191
- 畸变：k2=1.458e-2, k3=-8.673e-2, k4=1.644e-1, k5=-1.974e-1, k6=1.160e-1, k7=-2.809e-2
- 外参 Tcl_0（雷达→相机）：
  ```
  [0.00186, -0.99994, -0.01105, 0.03118,
   -0.00663,  0.01104, -0.99992, 0.0121,
    0.99998,  0.00193, -0.00661, -0.03035,
    0,0,0,1]
  ```
- SDK 投影算法（pointcloud_depth_converter.cpp）：`Kcl = K4x4 × Tcl`，点变换后 `u=round(x/z), v=round(y/z)`，depth=相机系 z（米），3×3 邻域填充；`postProcessDepthImage` 有 Sobel 边缘抑制
- 去畸变（inv_map）：每像素 `y=(v-v0)/A22; x=(u-u0-A12y)/A11; r=√(x²+y²); θ=atan(r); θd=θ+k2θ²+…+k7θ⁷; xd=x·θd/r; map=(xd·A11+A12·yd+u0, yd·A22+v0)`，再双线性采样

## SDK 直出话题（control_command.yaml register_keys 控制）

| 开关 | 话题 | 内容 |
|---|---|---|
| sendrgbcompressed=1 | /odin1/image/compressed | JPEG 彩图 |
| sendrgb=1 | /odin1/image | bgr8 解码图（pcd2depth 依赖） |
| sendrgbundistort=0 | — | 去畸变图（默认关） |
| senddtof=1 | /odin1/cloud_raw | 点云 |
| sendcloudslam=1 | /odin1/cloud_slam | SLAM 地图 |
| sendodom=1 | /odin1/odometry | 位姿 |
| senddepth=0 | /odin1/depth_img_competetion | 深度图（注意官方拼写 competetion；默认关，吃算力） |
| senddepth=0 | /odin1/depth_img_competetion_cloud | 深度彩色点云 |
| sendreprojection=0 | /odin1/reprojected_image | SLAM 点云重投影到图像 |
| sendoverlay=0 | /odin1/overlay_image | 叠加融合图 |
| recorddata=1 | 文件 | **olx 直出录制** → `~/odin_ws/src/odin_ros_driver/recorddata/<时间戳>/`（rgb+odometry+slam 的 olx 私有格式，拷走整个文件夹） |

## 采集操作（fouruav，ROS1 Noetic）

1. 改 control_command.yaml 开直出开关（sed 或 nano）
2. `source /opt/ros/noetic/setup.bash && source ~/odin_ws/devel/setup.bash`
3. `roslaunch odin_ros_driver odin1_ros1.launch`（节点 host_sdk_sample + pcd2depth_node + cloud_reprojection_node + image_overlay_node）
4. `rosbag record -O out.bag <话题列表>`；异常退出产生 .bag.active，改名 .bag 仍可顺序解析
5. 采集完拷 bag + recorddata 整个文件夹

一键采集脚本模板（start_record.sh，放 fouruav 家目录）：
```bash
#!/bin/bash
source /opt/ros/noetic/setup.bash; source ~/odin_ws/devel/setup.bash
CFG=~/odin_ws/src/odin_ros_driver/config/control_command.yaml
sed -i 's/senddepth: 0/senddepth: 1/' $CFG
sed -i 's/sendreprojection: 0/sendreprojection: 1/' $CFG
sed -i 's/sendoverlay: 0/sendoverlay: 1/' $CFG
pkill -f host_sdk_sample 2>/dev/null; sleep 1
nohup roslaunch odin_ros_driver odin1_ros1.launch > /tmp/odin_launch.log 2>&1 &
sleep 12
BAG=~/record_$(date +%m%d_%H%M%S).bag
trap 'pkill -f host_sdk_sample 2>/dev/null; echo "bag: $BAG; olx: ~/odin_ws/src/odin_ros_driver/recorddata/"' EXIT
rosbag record -O $BAG /odin1/cloud_raw /odin1/cloud_slam /odin1/image/compressed /odin1/image \
  /odin1/depth_img_competetion /odin1/depth_img_competetion_cloud /odin1/reprojected_image /odin1/overlay_image \
  /odin1/odometry /odin1/imu /tf /tf_static
```

## 数据规模参考（2026-07-27 批次）

- 7 个 bag 共 22.5GB；cloud_raw ~10Hz、图像与点云同帧率
- SLAM 地图全帧合并 0.03m 去重：小 bag 128 万点，大 bag 2238 万点
- 全帧 RGB-D 导出：~1.4 万帧、每帧 ~0.8MB，大 bag（5306 帧 × 2238 万点投影）需数小时——注意电脑睡眠会挂起任务（墙钟跳变）
