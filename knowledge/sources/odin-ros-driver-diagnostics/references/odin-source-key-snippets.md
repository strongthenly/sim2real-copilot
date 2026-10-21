# odin_ros_driver 关键源码片段（2026-08-12 排查"定位数据一直没有"时提取）

## pcd2depth_ros.cpp（ROS1 main，退出逻辑 ≈31-35 行）
```cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_projector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    int senddepth=0;
    pnh.param("register_keys/senddepth", senddepth, 0);
    if(senddepth==0)
    {
        ROS_INFO("Depth image will not be published.");
        return 0;
    }
    ...
```
→ senddepth=0 时节点启动即干净退出，roslaunch 报 "finished cleanly"。

## cloud_reprojection_ros.cpp（ROS1 main，退出逻辑 ≈457-462 行）
```cpp
    int sendreprojection = 0;
    pnh.param("register_keys/sendreprojection", sendreprojection, 0);
    if(sendreprojection == 0)
    {
        ROS_INFO("Cloud reprojection will not be published.");
        return 0;
    }
    ...
```
→ 与 pcd2depth 同构。ROS2 版本在 pcd2depth_ros2.cpp:165 / cloud_reprojection_ros.cpp:270。

## odin_to_mavros.cpp（全文 ≈40 行，假数据坑）
```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry vins_odom;                       // 全局缓存，初始全 0
void vins_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
   vins_odom = *msg;                                // 仅收到数据时更新
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vins_to_mavros");
  ros::NodeHandle nh;
  ros::Publisher pos_pub=nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose",10);
  ros::Subscriber vins_sub = nh.subscribe<nav_msgs::Odometry> ("/odin1/odometry_highfreq", 2, vins_cb);
  geometry_msgs::PoseStamped pose;
  setlocale(LC_ALL,"");
  ros::Rate rate(30.0);
  ROS_INFO("odin开始融合");
  while (ros::ok())
  {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = vins_odom.pose.pose.position.x;
    pose.pose.position.y = vins_odom.pose.pose.position.y;
    pose.pose.position.z = vins_odom.pose.pose.position.z;
    pose.pose.orientation.x = vins_odom.pose.pose.orientation.x;
    pose.pose.orientation.y = vins_odom.pose.pose.orientation.y;
    pose.pose.orientation.z = vins_odom.pose.pose.orientation.z;
    pose.pose.orientation.w = vins_odom.pose.pose.orientation.w;
    pos_pub.publish(pose);                          // 无条件发布，30Hz 恒定
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
```
→ 无新鲜度检查：上游断流照发缓存值。vision_pose 30Hz 恒定的 delta(0.033~0.034s)/std(0.0001) 是 timer 级稳定，真实位姿流不会这么齐，这是死值指纹。

## control_command.yaml 关键键（config/ 下）
- `register_keys/senddepth: 0` → pcd2depth_node 启动即退（正常）
- `register_keys/sendreprojection: 0` → cloud_reprojection_node 启动即退（正常）
- `register_keys/sendodom: 1` → /odin1/odometry、odometry_highfreq 等位姿话题
- `register_keys/senddtof: 1` → /odin1/cloud_raw；`dtof_fps: 100`（=10fps）
- `register_keys/sendcloudslam: 0`、`sendcloudrender: 0`
- `register_keys/strict_usb3.0_check: 0`（不严查 USB3）
- `register_keys/devstatuslog: 1` → log/Driver_*/dev_status.csv（**目录空 = 设备没连上**）
- `register_keys/custom_map_mode: 0`=里程计 1=SLAM建图 2=重定位；`resetalgo: 0`
- `recorddata: 0`；开启时 olx 存 ws/src/odin_ros_driver/recorddata/{开始时间}/，整个文件夹拷走

## 现场证据（2026-08-12，设备未连接时的观测）
- rosnode list：/host_sdk_sample /image_overlay_node /odin_to_mavros /mavros 全活，host_sdk_sample pid 存在（Ssl）
- rostopic hz 12s：/odin1/odometry、odometry_highfreq、cloud_raw、imu、wiwc、camera_pose_visual 全 "no new messages"
- /mavros/vision_pose/pose 仍 30Hz（假数据）
- log/Driver_20260812_205018/ 空（无 dev_status.csv）；lsusb Bus 002 只有 root hub
