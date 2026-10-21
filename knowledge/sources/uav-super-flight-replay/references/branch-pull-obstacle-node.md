# B 分支（feat/obstacle-to-px4）拉取与编译记录（2026-08-10）

## 背景

用户要在 fouruav 上拉取"别人的 task 文件夹"做另一项测试（避障功能），担心覆盖本地修改。最终拉取的是 `feat/obstacle-to-px4` 分支。

## 仓库结构（拉取前必须知道）

```
catkin_ws (git, 远程 <ORG>/<PRIVATE_WS_REPO>.git)
└── src/task_run_scripts (子模块, jihaofly/task_run_scripts.git)
    └── scripts/NX_code (嵌套子模块, jihaofly/NX_code.git, upstream=Liu404mob/NX_code.git)
```

- `.git` 是 48 字节的 gitdir 指针文件（`gitdir: ../../.git/modules/src/task_run_scripts`），不是目录 → 子模块标志
- 嵌套子模块的 gitdir 指向 `modules/src/task_run_scripts/modules/scripts/NX_code`
- 顶层 `git status` 显示 ` m src/task_run_scripts`（小写 m = 子模块指针有未提交内容）

## 分支差异侦察

```bash
git fetch origin                       # 无风险，先 fetch
git log --oneline HEAD..origin/main    # 看落后多少提交
git log --oneline origin/feat/obstacle-to-px4 -5   # 看 feature 分支内容
```

B 分支包含：`efcb65b feat: PX4 obstacle avoidance node (OBSTACLE_DISTANCE)`、`5edb1bb feat: 点击固定高度、航点跟随航点高度`（可能修复 z 覆盖问题）、`7b83f8c MID360 双模式配置`。

## 快照保护（关键步骤，先做）

```bash
# 停服务避免文件占用
PID=$(ss -tlnp | grep 6004 | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2); [ -n "$PID" ] && kill $PID

# 完整 tar 快照（含未跟踪文件！git stash 不含 untracked）
cd ~ && tar czf ~/task_run_scripts_backup_20260810.tar.gz \
  --exclude='task_run_scripts/log' --exclude='task_run_scripts/.git' \
  -C ~/catkin_ws/src task_run_scripts
tar tzf ~/task_run_scripts_backup_20260810.tar.gz | wc -l   # 验证
# 抽查关键文件：waypoint.yaml / nx_command_server.py / data_guidepoints / guide_Readme.md
```

## 拉取流程（stash → checkout → submodule update）

```bash
# ① 嵌套子模块先 stash（子模块有自己的 stash 栈）
cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code
git stash push -m "before_feat_obstacle_NXcode"

# ② 顶层 stash
cd ~/catkin_ws/src/task_run_scripts
git stash push -m "before_feat_obstacle"

# ③ 切分支 + 更新子模块
git checkout feat/obstacle-to-px4        # 显示 M scripts/NX_code 正常，跟随子模块指针
git submodule update --init --recursive  # 检出 NX_code 到分支指定 commit (fceb9df)
```

## 编译（livox ROS_EDITION 坑）

**失败**：裸 `catkin_make -j4` → `livox_ros_driver2/CMakeLists.txt:224: find_package(ament_cmake_auto REQUIRED)` 找不到。
**原因**：livox_ros_driver2 1.2.6 是 ROS1/ROS2 双版本，`CMakeLists.txt` 开头 `if(ROS_EDITION STREQUAL "ROS1")` 分支，未传变量默认走 ROS2（需要 ament_cmake_auto、rosidl，Noetic 没有）。
**修复**：
```bash
cd ~/catkin_ws && catkin_make -j4 -DROS_EDITION=ROS1
```
- 之前 build/livox_ros_driver2 的失败产物要 `rm -rf` 清掉
- Jetson 全量编译 10-20 分钟（FAST-LIVO2 的 laserMapping 最久）
- 后台跑：`nohup bash -c 'source /opt/ros/noetic/setup.bash && catkin_make -j4 -DROS_EDITION=ROS1' > /tmp/catkin_make.log 2>&1 &`
- 轮询：`grep -c "error:" /tmp/catkin_make.log` + `ps aux | grep cc1plus`
- 验证新节点：`ls ~/catkin_ws/devel/lib/task_run_scripts/obstacle_to_px4_node`

## obstacle_to_px4 节点（B 分支新增）

- `src/obstacle_to_px4.cpp` + `launch/obstacle_to_px4.launch`（`drone:=mid360` 参数）
- launch 含 `livox_to_aft_mapped` 静态变换：`args="0 0 0 0 0.349 0 livox_frame aft_mapped 100"`（前倾 20°，与 imu_tf.cpp setRPY(0, M_PI/9, 0) 一致）
- 参数（mid360.yaml）：cloud_topic/target_frame/num_bins=36/h_fov_deg=360/height_band=1.5/range_min=0.2/range_max=20/accumulation_window=0.2/publish_rate=20/max_cloud_age=0.5/angle_sign=-1.0（PX4 俯视顺时针为正，左右镜像改 1.0）/debug_markers

### 故障：no cloud received yet（TF 不匹配）

**现象**：launch 正常，但 `[obstacle_to_px4] no cloud received yet, stop publishing` 持续。
**排查链**：
1. livox 驱动在跑、`/livox/lidar` 10Hz 有数据（排除数据源）
2. `rosrun tf tf_monitor` → 帧列表只有 camera_*/base_link_frd/map_ned/odom_ned，**没有 livox_frame、没有 aft_mapped**？—— 注意 aft_mapped 实际存在（`/aft_mapped_to_init` 消息 child_frame_id="aft_mapped"），但 tf_monitor 只列了 static 帧
3. `rostopic echo /aft_mapped_to_init -n 1` → frame_id="camera_init", child_frame_id="aft_mapped"
4. `rostopic echo /cloud_registered -n 1` → frame_id="camera_init"
5. `rosrun tf tf_echo camera_init aft_mapped` → ✅ 有变换；`rosrun tf tf_echo livox_frame aft_mapped` → ❌ "livox_frame does not exist"

**结论**：FAST-LIVO2 发布 `camera_init→aft_mapped` 动态 TF，但**没有 livox_frame 帧**。obstacle_to_px4 默认订阅 `/livox/lidar`（frame=livox_frame）→ lookupTransform 失败 → 丢点云。

**修复**：mid360.yaml `cloud_topic: /livox/lidar → /cloud_registered`（camera_init 系，有 TF 链）。改后重启 launch 即工作。

## 工作后的完整链路 + PX4 侧参数（最后一环）

**话题链**（实测）：
```
/cloud_registered → obstacle_to_px4(36扇区) → /mavros/obstacle/send (sensor_msgs/LaserScan, 20Hz)
→ MAVROS distance_sensor 插件(px4_pluginlists.yaml 第7行已启用) → MAVLink OBSTACLE_DISTANCE → PX4
```
- 验证数据在流：`rostopic hz /mavros/obstacle/send`（~20Hz）；`rostopic echo /obstacle_to_px4/status -n 1` → `min_dist=0.36 m no_cover=0/36`（全扇区有覆盖）
- 节点源码（obstacle_to_px4.cpp:3,59）：`点云 -> 机体系 -> 高度带滤波 -> 水平扇区最近距离 -> LaserScan -> /mavros/obstacle/send`

**⚠️ 关键：节点工作了 ≠ PX4 在避障**。PX4 端默认 `COM_OB_ACT=0`（避障关闭）会忽略 OBSTACLE_DISTANCE。实测参数：
```
COM_OB_ACT    = 0   ← 必须设 1 才启用
COM_OB_RADIUS = 0   ← 生效半径(m)，首测建议 2~5
COM_OB_W_HGT  = 0   ← 高度带宽(m)，建议 1.5
COM_OB_PRM_AV = 0   ← 安全裕度
MPC_POS_MODE  = 4   ← 位置控制模式(已 OK)
```
设置（命令行或 QGC）：
```bash
rosservice call /mavros/param/set "param_id: 'COM_OB_ACT' value: {integer: 1}"
rosservice call /mavros/param/set "param_id: 'COM_OB_RADIUS' value: {integer: 2}"
```
**验证避障真正生效**：设完查 `rosservice call /mavros/param/get "param_id: 'COM_OB_ACT'"` 返回 1；真机首测小半径+低速，确认减速/绕行行为。
**注意**：status 的 `min_dist` 若持续 ~0.3-0.4m（接近 range_min=0.2），可能是机架/雷达自身遮挡或测试环境近物——开启避障后 PX4 会一直认为有近距障碍而减速，需查明来源或调整 height_band/range_min。
