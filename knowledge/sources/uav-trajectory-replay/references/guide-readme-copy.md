# guide_Readme.md 副本（2026-08-10 写入 fouruav: ~/catkin_ws/src/task_run_scripts/guide_Readme.md）

机上使用手册的完整内容备份，若机上文件丢失可从这里恢复。

## 1. 系统架构
- 两套复飞：①SUPER 航点任务（按钮1/3，避障规划，走 /planning/pos_cmd）②Path Replay（UDP RecordPathReplay，纯回放无避障）
- 数据流：guidepoints_new.txt → waypoint_mission（抽稀0.1m）→ CH10 拨杆触发 → /waypoint_goal → fsm_node → /planning/pos_cmd → bridge → /mavros/setpoint_raw/local → PX4

## 2. 目录
- data_guidepoints/guidepoints_new.txt = 始终最新（按钮3 加载）
- data_guidepoints/guidepoints_YYYYMMDD_HHMMSS.txt = 历史归档
- data/real_mission.txt = 按钮1 默认（空格格式 x y z 切换距离）
- 配置：waypoint.yaml（start_trigger_type=1、switch_dis=0.1、waypoint_downsample_dist=0.1）、super_to_px4_bridge.yaml（init_goal/z=1.2、前馈、geofence_en=false）、real_flight.yaml（cloud_topic=/cloud_registered）
- 按钮：UserCmd1=启动SUPER(默认航点)、UserCmd2=关闭SUPER(先POSCTL再杀)、UserCmd3=SUPER复飞(加载guidepoints_new.txt)

## 3. 使用流程
### 录制（时间戳归档自动）
```bash
python3 -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b'RecordPathStart',('127.0.0.1',6003))"
# 手持走轨迹，保持稳定高度
python3 -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b'RecordPathStop',('127.0.0.1',6003))"
```
纪律：起点=起飞点；全程同一高度（与 init_goal 一致最好）；**终点保持高度再 Stop，别落地**

### 复飞操作
① start_fastlivo2.sh false → ② 按钮3 → ③ 解锁+切OFFBOARD（bridge 自动起飞 1.2m 悬停）→ ④ 悬停稳定后拨杆（CH10 先高>1500 再拉低<1500，pitch<1200）→ ⑤ 盯 RViz 黄点 → ⑥ 按钮2 关闭

### RViz 怎么看
- 黄点（/fsm_node/visualization/goal）= 当前目标，动态
- 蓝点+编号（/waypoint_mission/mkr）= 全部航点，静态

## 4. 关键参数
| 参数 | 当前值 | 含义 |
|---|---|---|
| waypoint_downsample_dist | 0.1 | 抽稀间距(m, 3D)，0=不抽稀 |
| switch_dis | 0.1 | 距航点3D距离<此值=到达 |
| init_goal/z | 1.2 | 起飞悬停高度(m) |
| velocity_ff_gain | 0.7 | 速度前馈增益(0.5~1.0，过大抖动) |
| geofence_en | false | 地理围栏(建议真飞开启) |
| max_vel | 2.0 | 规划最大速度(建议首飞1.0) |

## 5. 故障排查表
| 现象 | 原因 | 处理 |
|---|---|---|
| RViz 白屏/无地图 | fsm_node 没跑或同名顶替 | 确认仅一个 launch；ps 查 fsm_node |
| 看不到蓝点航点 | RViz 后启动错过发布 | Waypoints 显示项取消勾选再勾上 |
| 拨杆不触发 | CH10 需下降沿(先高后低)；pitch<1200 | 确认拨杆顺序 |
| 复飞无轨迹 | 没切 OFFBOARD/没拨杆 | 按流程走 |
| 按钮2 后推流黑屏 | 旧版 rviz 模糊匹配误杀 | 已修复为 top_down.rviz 精确匹配 |
| 录制带降落段 | 终点落地 | 重新录，终点保持高度 |
| 航迹高度与起飞高度差 | 录 0.75 vs init 1.2 | 统一两者 |

## 6. 备份与回滚
- nx_command_server.py.bak_newfile、User_define_commands.yaml.bak_newfile（改后路）
- 回滚：cp xxx.bak_newfile xxx

## 7. 安全提醒
1. 首飞前不装桨完整地面演练（按钮3→切OFFBOARD→拨杆→按钮2）
2. 首次真飞：开阔场地、低速(max_vel=1.0)、低增益(velocity_ff_gain=0.3~0.5)
3. 建议开 geofence_en=true（当前关闭）
4. 紧急接管：遥控器切 Position 优先于一切
