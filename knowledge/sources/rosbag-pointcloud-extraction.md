---
name: rosbag-pointcloud-extraction
description: 从 ROS1 bag 提取点云转 PCD（Windows/Linux 均可，无需安装 ROS）——话题检测、标准 bag 用 rosbags 库、索引损坏/自研非标准格式 bag 用内置顺序解析器 + 消息字节嗅探。触发词：bag、rosbag、PCD、点云地图、cloud_slam、cloud_raw、PointCloud2、CustomMsg、点云提取。
---

# ROS1 bag → PCD 点云提取

## 场景
用户的无人机 bag（fouruav / odin SDK 录制）需要检测话题、提取点云、转 PCD 点云地图，通常在 Windows 本机完成（无需 ROS）。

## 快速路径（工具已就绪）
Windows 本机 Python 3.12 + rosbags 已装（清华镜像）。一站式工具在 `C:\Users\<USER>\Desktop\bag_tools\bag2pcd.py`：

```powershell
python bag2pcd.py <bag路径> --list [--quick]                    # 列出全部话题，标出点云话题(★)
python bag2pcd.py <bag路径> --topic /xxx/cloud_slam --mode map   # 取最后一帧 = SLAM全局地图
python bag2pcd.py <bag路径> --topic /xxx/cloud_raw --mode frames --every 5   # 逐帧存
python bag2pcd.py <bag路径> --topic /xxx/cloud_raw --mode merge --voxel 0.1   # 拼接+体素滤波
python batch_convert.py <bag目录> <输出目录>     # 批量: 目录下每个 bag 取 SLAM 地图末帧→PCD(每 bag 只扫1遍)
python bag2map_color.py <bag> <输出目录> [--color-max 0.5]  # 稠密彩色地图(odometry位姿拼接+slam着色), 用户主需求
python batch_map_color.py <bag目录> <输出目录>    # 批量稠密彩色地图(后台跑, 22GB约50分钟)
python check_color.py <pcd> [输出.png]            # 着色率统计+3D预览
python view_pcd.py <pcd文件> <输出.png>          # matplotlib 3D 预览(RGB着色), 交付时 MEDIA 内联展示
python convert_pcd2las.py <pcd文件或目录> <输出目录>  # PCD→LAS 1.2(点格式3, 含RGB), 测绘通用格式
python merge_slam.py <bag> <输出目录> [--voxel 0.03]   # cloud_slam 全部帧合并+体素去重=干净彩色地图(**最终交付方案**)
python batch_merge_slam.py <bag目录> <输出目录>        # 批量 SLAM 彩色地图(后台, 几分钟)
python rgbd_all.py <bag> <输出根目录>                  # 全帧 RGB-D: 每帧一个时间戳文件夹(color.jpg+depth.png彩色+depth_raw.png 16bit)
python batch_rgbd_all.py <bag目录> <输出根目录>        # 批量全帧 RGB-D(后台, 大bag每帧~2s)
python slam_depth.py <bag> <输出目录> [帧号...]        # SLAM 地图投影深度图(覆盖~40.8%, 用户认可方案)
python depth_visual.py <rgbd目录> [输出目录]           # 稀疏深度补洞+guided filter+伪彩色可视化
python rgbd_export.py <bag> <输出目录> [--every N] [--max-pairs N]  # 单帧雷达投影 RGB-D(稀疏1.9%, 仅测试/调试用)
```

若工具丢失，用 `references/ros1-bag-format.md` 里的核心解析器重建（顺序解析 + 嗅探 + PCD 输出全部可复现）。

## 工作流
1. **找 bag**：bag 可能藏在同名文件夹里（`xxx.bag\xxx.bag`，rosbag record 到目录的典型结构），先 search_files 确认路径。
2. **话题检测**：标准 bag 用 `rosbags.rosbag1.Reader` 秒出连接；但**非标准/索引损坏 bag 会静默返回 `connections=0`（不报错！）**——看到 0 连接别怀疑代码，立刻切顺序解析。
3. **判定点云话题**：
   - SLAM 地图话题：名字含 `slam/map/registered`，frame_id=`odom`，带 `rgb` 字段。**⚠️ 实测非累积式**：每帧 1.3 万点左右独立波动（5997~19752），map 取末帧只有 1~2 万点（稀疏）——只适合快速预览。**最终交付方案 = `merge_slam.py`：cloud_slam 全部帧合并 + 0.03m 体素去重**（每 bag 几十~几百万彩色点，99%+ 着色率，无重影，SLAM 优化坐标系）——用户明确要求"干净、彩色、准确"，odometry 拼接不满足（见稠密地图节）
   - 原始雷达帧：每帧几万点（MID360 一帧 49152 点），frame_id=雷达坐标系，无颜色 → 稠密拼接的主数据源
4. **转 PCD 后读回验证**：POINTS 数、坐标范围、rgb 字段，防解析错位。
5. 成果用 CloudCompare（免费）打开查看；或 `view_pcd.py` 出 matplotlib 预览图（RGB 着色）直接 MEDIA 发给用户。

## 批量转换（多 bag）
用户常把一次飞行录成目录下多个 bag（如 `odin_data_20260727_235552.bag\` 下 7 个，共 20GB+）。`batch_convert.py` 逻辑：
- `scan_bag_topics(path, keep_last=True)` **一次扫描同时**拿话题列表、帧数、每话题最后一条消息字节——map 模式转换无需第二遍扫描
- 每个 bag 自动嗅探类型 → 优先选名字含 slam/map/registered 的话题 → 末帧存 `输出目录\<bag名>_<话题名>.pcd`
- 单个 bag 失败只标 ❌ 不影响其它；最后汇总 `完成: X/N`
- 22GB 全扫约 5-15 分钟：**用 terminal 后台运行 + notify_on_complete**，别前台干等

## 核心难点：非标准/自研 bag（odin SDK）
odin SDK 录的 bag 特征：
- header 字段值为**裸类型**：string 无 u32 长度前缀、chunk `size` 是 4B u32（标准是 8B）
- 连接记录只有 `conn/op/topic`，**没有 type/md5sum/message_definition** → 消息类型只能靠字节嗅探
- rosbags 打开静默返回空连接；但文件头、chunk 数据区、消息体仍是标准 ROS 序列化 → **顺序解析可行**

判定流程：
1. `Reader(path).connections` 为空 → 顺序解析器扫 chunk（连接记录 op=0x07 + 消息记录 op=0x02）
2. **类型嗅探（强校验）**：header 后解析 fields 数组，字段名集合 ⊇ {x,y,z} → `sensor_msgs/PointCloud2`；`timebase u64 > 1e15` 且点数与数组长度匹配 → Livox `CustomMsg`。弱条件（height≤10 之类）会把 Imu/Pose/Image 误判成点云——必须校验字段名。

## 坑速查
- **ROS1 bag 文件头记录也有 `data_len+data` 部分**，读完 header 必须跳过，否则后面所有记录错位（最隐蔽的坑）
- header 字段 = 循环 `(u32 len + "key=value")`，**无分隔符**，不能按 `
` split
- chunk `size` 字段按 `len(v)` 自适应：4B 用 `<I`，8B 用 `<Q`
- PointCloud2 手工解析：`header(seq u32+stamp u64+frame_id string) + height/width u32 + nfields u32 + fields[] + is_bigendian u8 + point_step/row_step u32 + data_len u32 + data`
- **RGB 位模式（最坑）**：odin/cloud_slam 的 rgb 字段是 **datatype=7 (float32)**（PCL 风格，RGB 打包在位模式里）。提取必须 `arr['rgb'].view(np.uint32)`；**`astype(np.uint32)` 是数值转换，会把所有颜色毁成近 0（着色率 0%，肉眼全灰/全黑）**。仅当 datatype=6 (u32) 时才可 astype。PCD 输出写回时 `rgb.view(np.float32)`
- **scipy cKDTree 哨兵索引**：`query(..., distance_upper_bound=)` 超阈值点的返回 idx = n（即点数，**越界 1**）。必须 `valid=np.isfinite(d)` → `colors=slam_rgb[np.where(valid, idx, 0)]` → `colors[~valid]=灰`
- **np.minimum.at 做"每像素最近深度"必须 inf 初始化**：`np.minimum.at(depth, idx, z)` 若 depth 初始 0，min(0, z)=0 会把有效深度全吞成 0（深度图全黑）。先 `depth=np.full(..., np.inf)`，最后 `depth[~isfinite]=0`。另外 at 的索引用扁平 `nv*W+nu` 比元组 (nv,nu) 稳
- **ROS1 时间戳 = sec u32@4 + nsec u32@8**（`sec + nsec/1e9` 秒），**不是 u64@4**。按 u64 读会得到 13 年量级的"看似合理"错值（图像↔点云差 4.2e8 秒就是这坑）。图像与点云同 header 发布时（odin）dt ~1e-10s，帧数相同可按序配对
- **memmap 元素索引 vs 字节偏移**：`tf.tell()` 是字节，`np.memmap(dtype=float32)` 要 `//4` 转元素偏移，否则错位读帧（字节偏移恰好落在 4 倍帧边界时数据"看起来正常"但帧号全错）、越界切片静默返回空（不报错）
- **Odometry 消息解析**：header(seq+stamp+frame_id) 之后还有 **child_frame_id string**，跳过两个 string 再读 pose 7×f64，否则矩阵全是 inf/nan（T 矩阵乱码 → 投影 0 命中）
- 深度图有效像素低（单帧雷达 1.9%、SLAM 地图投影 40.8%）是物理事实，不是 bug；用户要"清晰彩色"用 SLAM 投影或补洞，不要硬填
- Livox CustomMsg 布局：`header + timebase u64 + point_num u32 + lidar_id u8 + points[](19B/点)`——ROS 序列化无对齐：offset_time u32 + xyz f32×3 + reflectivity/tag/line u8×3 = 19B
- PCD binary 头：`FIELDS/SIZE/TYPE/COUNT/WIDTH/HEIGHT/POINTS/DATA binary`，字段全部 f32，二进制数据紧凑排列（无 padding）
- rosbags API 陷阱：`Writer(path)`/`Reader(path)` 不收 typestore 参数；`add_connection(topic, type, typestore=ts)`；`write` 传 `ts.serialize_ros1(msg, typename)` 的 bytes；反序列化用 `ts.deserialize_ros1(raw, typename)`（Reader 无 deserialize 方法）；Writer 构造测试 bag 也走这条
- pip 国内镜像：`-i https://pypi.tuna.tsinghua.edu.cn/simple`
- Windows 装 Python：winget 静默装（`winget install -e --id Python.Python.3.12 --silent`），装完需手动把 `C:\Users\<USER>\AppData\Local\Programs\Python\Python312\` 加到 PATH 前置（WindowsApps 商店占位符会拦截 `python`）

## PCD → LAS（通用点云格式）
LAS 是测绘/点云行业最通用交换格式（CloudCompare、LiDAR360、大疆智图均支持）。`scripts/convert_pcd2las.py` 手写 LAS 1.2 点格式 3 writer（34B/点含 RGB，无需 laspy）：
- header 227B：`LASF` 签名 + 版本 1.2 + 点格式 3 + 记录长 34 + scale(0.001) + offset(各轴 min) + max/min 六值；`assert len(hdr)==227`
- 坐标：`(x - offset)/scale` 转 int32，1mm 精度
- RGB：PCD 的 uint32 打包解包为 r/g/b u16（8bit << 8）；intensity 字段 ×257 映射 0-255→0-65535
- 点记录 dtype：x/y/z i4 + intensity u2 + bit_fields u1(置1) + classification u1 + scan_angle i1 + user_data u1 + point_source u2 + gps_time f8 + r/g/b u2 = 34B
- **验证 LAS 时别按记忆猜 header 偏移**（本会话验证脚本 offset 算错过一次，误报格式=227）：签名 offset 0、版本 24、点格式 104、记录长 106、legacy 点数 107；或直接读回 struct 核对 LASF/1.2/3/34
- 预览用 `view_pcd.py` 出 matplotlib 3D 图（RGB 着色、等比例轴），交付时 MEDIA 内联；标题中文"点"缺字体是 DejaVu 无 CJK，无害

## 深度图 / RGB-D 生成（用户认可方案：SLAM 地图投影）
用户需求演进的最终形态："一帧彩图对应深度"、全帧导出、每帧一个文件夹。**关键决策：用 cloud_slam 全局地图投影，不用单帧雷达**（单帧 1.9% 覆盖率、左边缘电线杆无深度、细长物形状变形——用户逐一投诉过）。

目录结构（用户确认）：`rgbd_all\<bag名>\<时间戳>\{color.jpg, depth.png(彩色可视化turbo), depth_raw.png(16bit mm)}`

管线（详见 `references/odin-depth-pipeline.md`）：
1. 收集 cloud_slam 全部帧合并+0.03m 去重（float32, 大bag 2238万点）作为投影地图
2. 收集 odometry 位姿（stamp + 4×4 矩阵）
3. 每帧：`T_cam = Kcl @ inv(T_odom)`，`pc = slam_homo @ T_cam.T`，`u=round(x/z), v=round(y/z)`，z>0.05，3×3 邻域 minimum.at（inf 初始化）
4. 彩图去畸变：FishPoly 逆映射 remap（与深度针孔视角像素级对齐）
5. 性能：每帧 ≈1.3~2s（128万~2238万点），大 bag 5306 帧约 2.5~3h；**电脑睡眠会中断任务**（墙钟跳变），提醒用户关睡眠；用后台 + notify_on_complete，用户会反复问进度

深度图可视化：turbo 伪彩色（红近蓝远，0~25m 归一化）；补洞（`depth_visual.py`：distance_transform_edt 最近值填充 + 彩图灰度引导 guided filter）后覆盖 100%，但**补洞是估计值**——定量分析用 depth_raw.png（16bit 雷达真实测量）。

## 稠密彩色地图（用户主需求："大小和 bag 差不多" + "要有色"）
**⚠️ 已知缺陷（用户实测反馈）**：odometry 位姿有漂移，拼接结果**重影/电线杆糊/灰色团**（颜色匹配不上）；用户最终选定 `merge_slam.py`（cloud_slam 合并）作为交付方案，本方案仅当用户明确要"完整数据量/大文件"时用（如"大小和 bag 差不多"），交付时说明权衡。

`cloud_raw` 占 bag 95% 体积（每帧 49152 点、无颜色）→ 用 `/odin1/odometry` 位姿逐帧变换到 odom 系拼接，**大小与 bag 同量级**（每点 16B：xyz+rgb，7.9GB bag → ~2.5GB PCD）；颜色用 `cloud_slam` **全部帧**（merge 后 432 万点）做 scipy KDTree 最近邻参考。脚本 `bag2map_color.py`（两遍扫描，内存安全可跑 1.7 亿点）。

要点：
- **第一遍**：收 odometry 位姿（nav_msgs/Odometry 解析：header + child_frame_id + pose 7×f64 + cov 36×f64 + twist 6×f64 + cov 36×f64，共 719B）+ cloud_slam 全部帧 xyz+rgb 颜色参考 + 统计 cloud_raw 总点数
- **第二遍**：逐帧 `p' = R(quat)·p + t` 变换 + KDTree 查色（`distance_upper_bound=0.5`m，超阈值给灰 0x808080）+ 流式写 PCD（先写占位 header，逐帧 append）
- 位姿对齐按**帧序号**（cloud_raw 第 i 帧 ↔ odometry 第 i 条，两者帧数一致）；位姿约定 = 雷达系→odom 系（frame_id=odom / child=odin1_base_link），首条 ≈ (0,0,0) 单位四元数
- 着色率 ~78% 属正常（灰色 = slam 未覆盖区域）；`--color-max` 调大提覆盖但引入颜色错位
- 交付验证：`check_color.py` 报着色率 + RGB 均值 + 出预览图；用户要求"要有色"时务必检查着色率，0% = rgb 解析 bug（见坑速查）
- 批量：`batch_map_color.py` 遍历目录逐 bag 调 bag2map_color（跳过已存在），**22.5GB/7 bag 约 50 分钟 → terminal 后台 + notify_on_complete**

## 原始格式 .olx/.bin（留形科技 odin 私有）——bag 补救路线
用户 odin 系统（**留形科技**自研）的原始点云数据是 `.olx` + `.bin` 私有格式，**无公开文档**（网搜只有二手平台 OLX，别浪费时间）。数据丢失只剩 bag 时：
1. **技术上完全可行**：bag 里点云已完整提取（地图末帧/原始帧/位姿都在），转 .olx/.bin 只是最后一步格式转换
2. 拿格式定义三条路（按优先级）：
   - **odin SDK**（用户在用 host_sdk_sample，bag callerid 可见）：示例代码大概率有导出 .olx/.bin 的函数或头文件格式定义 → 在 SDK 目录搜 `olx`/`bin`/`export`/`save`/`pointcloud` 关键词
   - **残留样本逆向**：哪怕 1KB 的 .olx 或几百 KB 的 .bin，直接逆向二进制（与逆向非标准 bag 同理）
   - **找厂商**：向留形科技要《点云数据格式说明》或导出工具（正规渠道）
3. 确认 .olx/.bin 关系（.olx 是否工程/索引、.bin 是否裸点云）与 .bin 每点字节数，能大幅加速逆向

**✅ 已确认（SSH fouruav 实查）**：`control_command.yaml` 里 `recorddata: 1` 就是 **olx 直出录制开关**——SDK 实时把 rgb+odometry+slam 点云录成 olx 格式存到 `~/odin_ws/src/odin_ros_driver/recorddata/<开始时间>/`，注释强调"copy the full folder"。用户丢失的 .olx/.bin 就是这个目录产的；以后采集直接拷走该文件夹即得官方原格式。

## odin SDK 关键信息（fouruav: ~/odin_ws/src/odin_ros_driver/）
- **内参外参**：`config/calib.yaml`（设备 O1-N090100105）——FishPoly 鱼眼模型、A11/A22/A12/u0/v0、k2~k7 畸变、`Tcl_0`（雷达→相机外参 4×4）。深度图/RGB-D 全靠它，别再去标定
- **话题开关**：`config/control_command.yaml` register_keys——sendrgbcompressed/senddtof/sendcloudslam/sendodom/sendimu=1；senddepth/sendreprojection/sendoverlay 默认 0（pcd2depth_node 源码里 senddepth=0 直接 return 不发深度）；recorddata=1
- **SDK 直出话题**（源码 grep 确认）：深度图 `/odin1/depth_img_competetion`（拼写如此）、深度点云 `/odin1/depth_img_competetion_cloud`、重投影 `/odin1/reprojected_image`、叠加 `/odin1/overlay_image`；pcd2depth 订阅的是 **`/odin1/image`（bgr8）不是 compressed**，录 bag 时两者都要录
- **采集**：`source /opt/ros/noetic/setup.bash && source ~/odin_ws/devel/setup.bash && roslaunch odin_ros_driver odin1_ros1.launch`；录 bag 用 `rosbag record -O xxx.bag /odin1/cloud_raw /odin1/cloud_slam /odin1/image/compressed /odin1/image /odin1/odometry /odin1/imu /tf /tf_static [深度话题]`。给非技术操作者做一键脚本（自动 sed 开关+启动+record，Ctrl+C 停止+trap 关驱动），参考桌面 `start_record.sh`

## 协作方式
用户偏好"指挥官模式"（命令由用户执行）或直接让 AI 在 Windows 本机动手（本会话两种都出现过），详见 uav-ros-diagnostics 技能。
