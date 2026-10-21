---
name: ros1-bag-data-extraction
description: 无需安装 ROS 从 ROS1 bag 提取点云/图像/位姿并转换（PCD/LAS/稠密地图/深度图/RGB-D），Windows/Linux 通用。覆盖标准 bag（rosbags 库）与索引损坏/自定义非标准 bag（内置顺序解析器+消息字节嗅探）。触发词：bag、rosbag、点云、PCD、LAS、深度图、RGB-D、PointCloud2、CustomMsg、稠密地图。
---

# ROS1 bag 数据提取与转换（无需 ROS）

## 适用场景
- 从 bag 提取点云（sensor_msgs/PointCloud2、livox CustomMsg）、图像、位姿
- 转 PCD / LAS / 稠密点云地图 / 深度图 / RGB-D 对
- bag 无法被标准工具读取（索引缺失、自定义 writer 非标准格式、自研 SDK 录制）
- 用户"点云文件丢了只有 bag"的补救场景：bag 里的数据是完整的，只差格式转换

## 环境（Windows 本机）
- `winget install -e --id Python.Python.3.12` 装真 Python（商店占位符不可用）
- `pip install rosbags numpy scipy pillow matplotlib`（清华镜像 `-i https://pypi.tuna.tsinghua.edu.cn/simple`）
- 用户桌面 `bag_tools/` 有现成脚本族：`bag2pcd.py`（bag→PCD/LAS+话题检测）、`merge_slam.py`（SLAM 地图合并）、`bag2map_color.py`（稠密彩色拼接）、`rgbd_all.py`（RGB-D 全帧导出）、`depth_visual.py`（深度补洞可视化）、`batch_*.py`（批量）

## 标准 bag：rosbags 库
- `Reader(path)` → `connections`（topic/msgtype/msgcount），`messages(connections=[conn])` 拿原始字节
- API 坑：`Writer(path)` 无 typestore 参数（`add_connection` 传）；`write()` 收 bytes（先 `ts.serialize_ros1(msg, typename)`）；反序列化用 `ts.deserialize_ros1(raw, typename)`（Reader 没有 deserialize 方法）

## 非标准/索引损坏 bag：顺序解析器（关键）
特征（自定义 writer 常见）：header 字段值为**裸字符串（无 u32 长度前缀）**、chunk size 是 4B u32（不是 u64）、连接记录只有 conn/op/topic（无 type/md5sum）、rosbags 打开 connections=0 但文件头 `#ROSBAG V2.0` 正常。
- **必须跳过文件头记录的 data_len+data 部分**（文件头也是完整记录！不跳则后续全部错位，症状：op 解析乱、话题空）
- 记录格式：`u32 header_len + header + u32 data_len + data`；header 字段 = 循环 `(u32 len, 'key=value')`（不是 
 分隔）
- chunk (op=0x05)：compression + size（**长度自适应**：4B 或 8B）+ 数据；chunk 内记录同格式（连接 op=0x07、消息 op=0x02，消息记录 header 含 conn u32 + time u64）
- 连接无 type 时用**消息字节嗅探**：PointCloud2 = fields 数组字段名含 x/y/z（强校验，能排除 Imu/Pose/Image 等误判）；CustomMsg = timebase>1e15 且 points 数组长度与 point_num 匹配

## 消息解析血泪坑
- **ROS1 Header 时间戳 = sec u32 + nsec u32，不是 u64**！误读 u64 会把时间弄乱（差 13 年、帧间隔异常）。正确：`stamp = sec + nsec/1e9`
- `tf.tell()` 返回**字节偏移**；`np.memmap(dtype=float32)` 是**元素索引** → 差 4 倍，存偏移时 `//4`
- `np.minimum.at` 做"每像素最近深度"必须用 **inf 初始化**（0 是无效值，min(0, z)=0 会把有效深度全吞掉）
- Odometry 解析：跳 frame_id + **child_frame_id 两个 string** 再读 pose（7×f64）——漏跳一个就全是 inf/nan
- PointCloud2 的 rgb 字段常为 datatype=7 (f32)（PCL 位模式打包 R<<16|G<<8|B）→ 用 `.view(np.uint32)`，**不是** `.astype()`（数值转换会毁掉颜色）

## 深度图生成：SLAM 地图投影优先
- 单帧雷达点云投影到相机视角覆盖仅 ~2%（稀疏，无法看）；guided filter 补洞可到 100% 彩色但细长物（电线杆）形状失真、视场外（鱼眼大角度）无数据
- **最佳方案：SLAM 全局地图投影**——用位姿把 odom 系稠密地图变换到当前相机视角：`p_cam = Kcl @ inv(T_odom) @ p_slam`，深度=相机系 z，`u=round(x/z)`。覆盖 40%+、形状与彩图对齐、视场全覆盖
- 深度渲染用 turbo 色标（红近蓝远），16bit 原始深度（mm）与彩色可视化分开保存（定量 vs 查看）

## 用户协作偏好（重要）
- 大任务先**"理解-复述-确认方案"再执行**：用户会明确要求"你理解一下，告诉我你的接下来的方案"——目录结构、文件命名、是否抽稀都要先确认
- RGB-D 导出约定：**每帧一个文件夹（时间戳命名）**，内含 `color.jpg`（去畸变彩图）+ `depth.png`（彩色可视化深度）+ `depth_raw.png`（16bit 原始深度）
- 用户要"干净、彩色、准确"的地图：SLAM 优化坐标系 > odometry 拼接（后者有漂移→重影/糊/灰）

## 参考
- `references/odin-liuxing-format.md` — 留形科技 odin 系统（odin1）bag 结构、标定参数、SDK 深度算法
