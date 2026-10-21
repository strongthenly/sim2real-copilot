---
name: rosbag-data-extraction
description: 从 ROS1 bag 提取点云/图像/深度数据（PCD/LAS/RGB-D）——支持标准 bag（rosbags 库）与非标准/索引损坏 bag（内置顺序解析器+消息字节嗅探）。覆盖 odin SDK 自定义 bag 格式、时间戳解析、PointCloud2 RGB 打包、深度图投影等坑。触发词：bag、点云、pcd、深度图、RGB-D、rosbag、提取、激光数据转文件。
---

# ROS1 bag 数据提取与转换

把 bag 里的点云/图像转成可用文件（PCD/LAS/深度图/RGB-D 对）。核心经验来自 odin 系统（留形科技自研设备）录制 bag 的逆向解析，但顺序解析器对**任何**索引损坏/非标准 bag 都适用。

## 双路径读取策略

1. **标准 bag**：`rosbags` 库（`pip install rosbags`，纯 Python，Windows/Linux 通用，无需 ROS）。`Reader(path)` 读 connections（topic/msgtype/msgcount）；`typestore.serialize_ros1/deserialize_ros1` 序列化。注意 rosbags 0.11 的 `Writer(path)` 无 typestore 参数（在 add_connection 传），`write()` 收 bytes。
2. **非标准/索引损坏 bag**：内置顺序解析器（见下）。判定：rosbags 打开后 `connections` 为空 → 自动降级。

## 非标准 bag 特征清单（odin SDK 录制的 bag）

- header 字段值为**裸字节**（string 无 u32 长度前缀、uint32 就是 4B、uint64 就是 8B）——标准工具解析必然错位
- chunk header 的 `size` 是 **4 字节 u32**（标准是 8 字节 u64）→ 长度自适应：`struct.unpack('<Q' if len(v)>=8 else '<I', v)`
- **文件头记录也有 `data_len + data` 部分，必须跳过**（magic 13B + u32 hlen + hdr + u32 dlen + data）——漏跳则后面全部错位，这是最容易踩的坑
- 连接记录（op=0x07）可能只有 `conn/op/topic` 三字段，**没有 type/md5sum/msgdef** → 消息类型靠字节嗅探
- header 字段解析：循环 `(u32 len, 'key=value')`，value 按裸类型解释（op=1B、string=原样、整数=小端）
- 索引段可能缺失/不完整（`.bag.active` 直接改名）——顺序扫描 chunk（op=0x05）内部记录即可，不依赖末尾索引

## 消息字节嗅探（无 type 时识别点云类型）

- PointCloud2：header 后 `height≤10`，fields 数组能解析且**字段名集合含 {x,y,z}**（强校验，防误判 Imu/Odometry/Image）
- Livox CustomMsg：header 后 u64 巨大（>1e15 纳秒时间戳）+ `point_num` 与 points 数组长度匹配
- 布局：`seq u32 + stamp u64(注意!!) + frame_id string` ——见下方时间戳坑

## 关键坑（全部踩过）

- **ROS1 Header 时间戳 = `sec u32 + nsec u32`（8 字节，不是 u64）**。用 `struct.unpack_from('<I', raw, 4)` + `unpack_from('<I', raw, 8)`，`stamp = sec + nsec/1e9`。误读成 u64 会得到"2017 年/2031 年"这类错乱值
- **PointCloud2 的 rgb 字段是 datatype=7 (float32)，PCL 风格打包**——取位模式用 `arr['rgb'].view(np.uint32)`，**不能用 `.astype(np.uint32)`**（数值转换毁掉颜色）
- **深度图取最近点必须 `np.full(inf)` 初始化 + `np.minimum.at`**——用 0 初始化会被 `min(0, z)=0` 吞掉全部有效值
- `np.minimum.at` 对 2D 数组用**扁平索引** `flat[nv*W+nu]` 最稳
- **memmap 偏移单位**：`tf.tell()` 是字节，memmap float32 是元素——`tell()//4` 才是元素偏移；错位 4 倍会导致静默读到错误帧数据
- 图像帧率与点云帧率可能都是 10Hz 且**同 header 发布**（seq 相同、stamp 差 <0.1s）→ 按序或 bisect 按时间戳匹配都行

## 深度图/RGB-D 导出（SLAM 投影方案）

- **单帧雷达点云投影深度图只有 ~1.9% 有效像素**（4.9 万点散在 1600×1296，相机 120°FOV 只覆盖前方）→ 稀疏、细长物体（电线杆）变形、视场边缘无数据
- **根治：用 SLAM 全局地图投影**（如 cloud_slam 全部帧合并+0.03m 体素去重）→ 覆盖率 ~40%，形状准、边缘也有深度
- 投影：`T_cam = Kcl @ inv(T_odom)`，`p_cam = slam_homo @ T_cam.T`，`u=round(x/z), v=round(y/z)`，depth=相机系 z，3×3 邻域 minimum.at 填充
- 彩色可视化：turbo 色标归一化（0~25m），补洞区乘 0.55~0.8 亮度区分
- **补洞（可选）**：最近有效值填充 + 以彩图灰度为引导的 guided filter（`scipy.ndimage.uniform_filter` 实现，保边缘）；补洞值是估计值，定量分析用原始 16bit

## 用户偏好（odin RGB-D 导出）

- 全帧导出（不抽稀，除非指定 `--every N`）
- **每帧一个文件夹**（时间戳命名），内含 `color.jpg`（去畸变，与深度对齐）+ `depth.png`（彩色可视化）+ `depth_raw.png`（16bit mm 原始）
- 深度要"清晰、彩色、可视化"——不要给黑底稀疏深度图

## 工具脚本（Windows 桌面 C:\Users\<USER>\Desktop\bag_tools\）

- `bag2pcd.py`：bag→PCD（map/frames/merge 三模式，--list 列话题）
- `bag2map_color.py` / `merge_slam.py`：稠密彩色拼接 / SLAM 干净地图
- `rgbd_all.py`：全帧 RGB-D 导出（每帧文件夹）；`depth_visual.py`：深度补洞+彩色化
- `batch_*.py`：批量版（跳过已存在）
- `range_image.py`：球面深度图（无需内参）
- `convert_pcd2las.py`：PCD→LAS 1.2（格式 3，RGB）

## 参考
- `references/odin-bag-format.md` — odin 非标准 bag 格式细节、calib.yaml 内参外参、SDK 直出话题、采集配置与一键采集脚本
