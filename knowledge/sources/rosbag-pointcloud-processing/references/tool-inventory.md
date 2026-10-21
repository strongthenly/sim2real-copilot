# bag_tools 脚本清单（C:\Users\<USER>\Desktop\bag_tools\）

全部 Python 3 脚本，Windows/Linux 通用（无需 ROS）。bag 源数据通常在 `C:\Users\<USER>\Desktop\odin_data_*_235552.bag\`（多个 .bag，odin SDK 非标准格式）。

## 转换主工具
| 脚本 | 功能 | 用法要点 |
|---|---|---|
| bag2pcd.py | 检测话题 + 转 PCD | `--list` 列话题；`--topic X --mode map\|frames\|merge --voxel 米 --out 目录`；标准/非标准 bag 自动选路径 |
| bag2map.py | odometry 拼接 cloud_raw → 稠密 PCD（xyz+intensity） | 有漂移→重影，已被彩色版取代 |
| bag2map_color.py | 稠密拼接 + cloud_slam 颜色 KDTree 传递（0.5m 阈值） | 大但脏；仅需要"大小接近 bag"时用 |
| merge_slam.py | **cloud_slam 全部帧合并（--voxel 0.03 去重）→ 干净彩色地图** | 用户最终要的"干净、彩色、准确"；批量 batch_merge_slam.py |
| rgbd_export.py | RGB-D 对（--every N 抽稀，旧方案单帧雷达深度） | 已被 rgbd_all 取代 |
| rgbd_all.py | **RGB-D 全帧：每帧一个时间戳文件夹（color.jpg + depth.png 彩色可视化 + depth_raw.png 16bit）** | 深度用 SLAM 地图投影；批量 batch_rgbd_all.py |
| slam_depth.py | SLAM 地图 → 相机视角深度图 | T_cam = Kcl @ inv(T_odom) |
| range_image.py | 雷达球面深度图（无需内参） | 360°×俯仰，16bit mm + 预览 |
| depth_visual.py | 稀疏深度补洞（最近值+guided filter）+ turbo 渲染 | 可视化用；定量用 16bit 原始 |
| convert_pcd2las.py | PCD → LAS 1.2（格式3，xyz+intensity+rgb） | 批量转通用格式 |
| view_pcd.py / check_color.py | 3D 预览 PNG / 着色率检查 | 验证用 |

## 调试/辅助
- probe2.py / sniff.py / debug_*.py / check_*.py：bag 结构探测、消息嗅探、单步调试（踩坑遗留，可复用）
- make_test_bag.py：rosbags 造标准测试 bag（Writer 无 typestore 参数，add_connection 传；write 需 serialize_ros1）
- batch_*.py：批量遍历 bag 目录（自动跳过已存在输出）
- start_record.sh：odin 一键采集脚本（操作人只会 Ctrl+C）

## 关键命令示例
```powershell
python bag2pcd.py <bag> --list --quick
python merge_slam.py <bag> <out> --voxel 0.03
python rgbd_all.py <bag> <out_root>          # 每帧一个文件夹
python convert_pcd2las.py <pcd目录> <out>
```

## 注意
- 用户会自己移动/删除文件——运行时先确认路径存在（曾遇 slam_map.pcd 被删）
- 大 bag（>7GB）全帧 RGB-D 每帧 ~1.4-3s，总时长数小时；电脑睡眠会暂停任务（墙钟跳变），提醒保持唤醒
