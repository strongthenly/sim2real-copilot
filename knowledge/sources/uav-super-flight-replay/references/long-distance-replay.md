# 长距离复飞方案（2026-08-23 定稿）

> 已同步写入桌面 SUPER.md 第九章（备份 SUPER.md.bak_20260823）。本文是技能库持久副本。

## 一、背景与结论

| 项 | 结论 |
|---|---|
| 已验证 | 12.5m 直行复飞 ✅（251 点，循迹误差 ≤0.1m） |
| 地图模式 | 滑动窗口（map_sliding_en=true，robocentric 中心随飞机）→ 单程无 30m 硬上限 |
| 长距离单程（50-100m+） | ✅ 可飞（飞到哪扫到哪，走廊只需前方几米） |
| 返航/往返 | ❌ 不可行——滑动窗口身后地图消散 = 摸黑，须全局地图（坑62） |

## 二、地图模式判定（⚠️ 方法论教训，agent 首答犯过）

- ROG-Map = **滑动窗口**（map_sliding_en=true，窗口中心随飞机）。依据：
  ① `references/autonomous-exploration.md` 第 8 行明确记载"rog_map 是滑动局部地图（map_sliding_en=true，原点随飞机移动，滑出边界清除）"
  ② 用户实测铁证："飞机飞走旧区域消散"（坑62 背景）
- **⚠️ 别仅凭 real_flight.yaml 的 map_size[30,30,6] + fix_map_origin 推断"固定原点 30m 硬上限"**——agent 首答曾这样误判，被 references 纠正。**判断 ROG-Map 行为以 references 实测记录为准，不以参数表外观为准**。
- 确认命令（⚠️ 2026-08-24 起 config 已分传感器，MID360S 用 mid360_real_flight.yaml，旧 real_flight.yaml 路径失效）：`grep -nE 'map_sliding|map_size|fix_map_origin' ~/catkin_ws/src/task_run_scripts/config/mid360_real_flight.yaml`
- **实测段结构（2026-08-24 摸底，机载已确认开启）**：`map_sliding: {enable: true, threshold: 0.3}`——yaml 段名是 **map_sliding**（不是老记录里的 map_sliding_en！），threshold=机器人偏离地图中心 0.3m 触发窗口滑动
- 注意：滑动的是窗口，**坐标系原点仍是雷达启动位置**（FAST-LIVO2 camera_init / odin odom 零点）——"换地点必须重启雷达"与滑动无关；fix_map_origin 是 z 偏移，也不是"固定地图"。

## 三、fix_map_origin [0,0,1.5] 语义（地图 z 定位总开关）

- **本质：地图窗口中心在垂直方向对准飞行高度层**。地图系 z = odom 系 z + 1.5。
- 窗口 z 范围：virtual_ground -3.0（量化 -1.95）~ virtual_ceil 3.0（钳制 1.95，坑43）→ 覆盖 odom 系 [-0.45, 3.45]：飞行高度 0.6-1.2m 在窗口中间，地面 z=0 压到窗口底边（干扰最小）。
- 事故史：改 [0,0,0] = 地面点散布全屏障碍（坑44）；叠加 goal_z 钳制 = 轨迹入地（坑63）。**红线勿动**。
- 改作业高度层时须**联动**调 fix_map_origin + virtual_ground + virtual_ceil 三个，不能只动一个。
- 源码确认：prob_map.cpp `robot_state_.p = fix_map_origin`（初始化）→ posToGlobalIndex 相对地图原点换算。

## 四、滑动窗口设置与验证

- 开关（实测段结构）：`map_sliding: {enable: true, threshold: 0.3}`（段名 map_sliding 不是 map_sliding_en；threshold=偏离中心超 0.3m 滑动）。**2026-08-24 摸底确认机载已开启**，无需再设——改动前先 `grep -nE 'map_sliding' <实际加载的 yaml>` 确认
- 备份先行：`cp real_flight.yaml real_flight.yaml.bak_<tag>`；一次只改一个参数（坑44 铁律）；改完必须确认 FSM 重启加载（文件 mtime < 进程启动时间）
- 验证（现象为准，配置会骗人）：① RViz occ 网格始终围绕飞机 ② 飞到 20m 外回头看，原点区域障碍格子消失 = 滑动生效
- 实现机制：robot_state_ 初始化 = fix_map_origin → 每帧 posToGlobalIndex 相对地图原点换算 → 飞机移动超阈值窗口平移、滑出边界体素清除

## 五、长距离三批改动清单

**批次1：飞行前必查（零风险，不改会出事）**
1. `goal_z_min/max` 0.0/0.3 → **0.6/0.825**（坑63 轨迹入地）
2. `raycasting.enable: true`（坑17 默认关=障碍永不消失）
3. `p_miss: 0.49`（必须 <0.4999，坑53 调大=全屏障碍）
4. `grep '临时|test|TODO' config/*.yaml` 无输出（坑63 教训）
5. 确认 FSM 加载 task_run_scripts 那份 yaml 且已重启加载（坑28/44）

**批次2：长距离适配（一次只改一个，备份+重启验证）**
| 参数 | 当前 | 建议 |
|---|---|---|
| `map_sliding_en` | 待确认 | true（与 map_size 同段） |
| `max_vel`/`max_acc` | **0.3/0.4**（2026-08-24 摸底，main 低速保守版，非 SUPER.md 的 1.0 时代值） | **0.3→1.0 + 0.4→0.6**（坑49：1.0 配 0.6 加速距离 0.85m；配套 max_omg 2.5→6.0 / max_acc_thr 20→25 / penna_jerk -5e5→-7.5e5 试飞绕行/俯冲后再按需，一次只改一组） |
| `inflation_step` | 2 | 开阔→3（防擦桨，坑64）；窄通道→保持 2（坑59） |
| `map_size` z/`virtual_ceil_height` | 6/3 | 环境有 2m+ 障碍→8/4（坑43 高处越界=OCCUPIED） |
| 航迹录制 | — | 终点留 0.5m+ 余量 + 空旷处（防吸附坑34/漂移坑58） |

**批次3：全局地图（长期，返航前提）**
坑62 方案：探索→PCD/OctoMap 保存→下次加载先验→配 frontend=true+终点 0.3。实施前只做单程。

## 六、红线（勿动）

fix_map_origin [0,0,1.5]（z 定位总开关，坑44/63）、robot_r 0.35（CIRI 拒绝阈值，坑64 方向反了）、penna_pos 1e7、frontend_in_known_free=true（坑56 宁停勿撞）、bridge 禁前馈（防摇摆，坑61）

## 七、运营注意

1. 单向推进，不指望回退/返航（滑动窗口固有代价）
2. AVOID_WAIT 停车 = 安全机制，障碍不走就不飞，须有人值守
3. 换地点必须重启雷达（坐标系原点=雷达启动位置）
4. 电池：100m@1m/s≈100s，PX4 低电量保护纳入计划
5. 长距离漂移 → 终点判定 0.3 兜底 + 航迹终点留余量

## 八、⚠️ patch 工具编辑 markdown 的代码块边界坑（2026-08-23 实操）

- **现象**：用 patch(mode=replace) 在 markdown 文件末尾追加章节时，old_string 取自**代码块内部**的行（如 SUPER.md 第八章 8.4 的列表行），该代码块的结束符 ``` 紧跟其后——new_string 若不显式包含它，``` 会被"推"到 new_string 末尾 → 原代码块未闭合（新章节被渲染进代码块）+ 文件末尾多一个孤立 ```
- **修复两步**：① 在 new_string 中于原代码块内容后补回 ```（恢复原闭合位置）② 删除文件末尾被推走的 ```
- **预防**：patch 前 read_file 确认 old_string 是否处于 ``` 代码块内（看上下文边界行）；追加新章节时 new_string 必须保留原结构（代码块闭合符位置不动）
- **验证**：patch 后 read_file 检查 ``` 成对 + 新章节正常渲染（表格/标题不落入代码块）

## 九、⭐ CH9 中断返航设计（2026-08-23 用户需求：CH10 去程 + 飞行中 CH9 立即从当前位置返航）

- **FSM 零改动原理**：guide_path 注入 = global_path[最近点]→最后一点，**最后一点=飞行方向** → 发布器切换发布内容即切换飞行方向；返航 = ① guide_path 换折返路径 ② goal 换航迹起点 A
- **return_trigger.py（新增 1 纯 Python 节点，零编译）**：订阅 /mavros/rc/in 监听 CH9 下降沿（同 waypoint_mission 的 CH10 逻辑 start_trigger_type=1）→ 顺序执行：
  1. `pkill` 精确杀 waypoint_mission（停止它继续发终点 B 的 goal——**竞态关键**：先杀再发新 goal，否则新旧 goal 交替发布 FSM 乱转）
  2. 发布返航 goal = 航迹起点 A → /waypoint_goal（PoseStamped latch 重复发）
  3. 通知 global_path_publisher 切"折返模式"：重新发布 /global_path = [当前位置] + 航迹[最近点→起点]（终点=A）→ FSM 收到新 goal+新 guide_path 自动重规划
- **返航路径**：推荐**折返**（当前位置→航迹最近点→沿航迹倒走回起点；走过的路，边飞边扫+方案④先验兼容）；**直线**（当前位置→起点）穿未知/未走过区域，走廊失败+撞墙风险
- **⚠️ 先验证遥控器通道数**：典型遥控器仅 8 物理通道，CH9 可能不存在！`rostopic echo /mavros/rc/in -n 1` 看 channels 数组长度（len≥9 才有 CH9）；没有就改用 CH8 两段开关或 QGC UserCmd4（逻辑一样，触发源换）
- 依赖：滚动机制成立 → CH9 触发后自动 yaw 转向 + 边飞边扫 → 零 C++ 闭环；全程机制 → 先方案④（航迹先验）

## 十、git 分支实测与开发前置确认（2026-08-23/24 直连侦察+实操）

- **机上初始分支状态（可能与预期不符）**：
  - task_run_scripts：**feat/obstacle-to-px4**（避障分支，非 main！本地还有 feat/4uav-guide、main；origin/main 存在）
  - SUPER：feat/guide-path-c（**无 main**，origin/HEAD→dev/4uav）
  - NX_code（嵌套子模块）：feat/4uav-guide（无 main 有 master，origin/HEAD→master）
  - NX_code 子模块 gitlink **漂移**（`git submodule status` 行首 `+`）→ 之后 `git submodule update` 会拉回父仓库旧指针
- **⭐ 分支新旧判断法**：`git log --oneline A..B`——`HEAD..main` 空 = main 是 HEAD 祖先 = 切本地 main 是回退（丢 obstacle-to-px4 独有 30+ 提交）；但 **本地 main 旧 ≠ origin/main 旧**！`git pull origin main --ff-only` 后 origin/main 已合入全部开发成果（obstacle/路径A/RViz插件，+5651 行）——判断"切 main 丢不丢工作"必须看 `HEAD..origin/main`，不是本地 main
- **切 main+拉最新实操（2026-08-24 用户"不管，直接切"）**：①快照 `tar czf ~/task_run_scripts_backup_<TS>.tar.gz --exclude='task_run_scripts/log' --exclude='task_run_scripts/.git' -C ~/catkin_ws/src task_run_scripts` ②工作区 `M scripts/NX_code`（gitlink 漂移）挡 checkout → 先 `git checkout -- scripts/NX_code`（只影响父仓库指针，子模块内容不动）③`git checkout -f main` ④`git pull origin main --ff-only`。⚠️ pull 快进**覆盖本地未提交的跟踪文件**（data_guidepoints/guidepoints_new.txt、endpoint.txt 被 origin/main 版替换——快照兜底）；未跟踪归档 guidepoints_YYYYMMDD*.txt 不受影响
- **2026-08-24 后续状态**：task_run_scripts 已在 main=origin/main（ccbb463，config 重构后），并拉 **feat/long-distance-replay** 开发分支（长距离复飞开发基线）；SUPER 继续 feat/guide-path-c；NX_code 子模块仍未 update（写操作，涉及按钮配置，先确认）
- **⭐ 阶段0 执行实录（按钮 launch 回归修复，2026-08-24）**：origin/main 重构后 `real_flight.launch` 默认 config_path 仍指向**已删除的** config/real_flight.yaml → 直接启动 FSM 加载失败/用默认参数。修复 NX_code/User_define_commands.yaml 三处：①按钮1/3 `roslaunch task_run_scripts real_flight.launch` → `roslaunch task_run_scripts fastlivo2/mid360_real_flight.launch`（薄包装 include real_flight.launch + 覆盖 config_path + rviz_config；use_waypoint/waypoint_data_path 命令行参数自动透传）②按钮2 pkill 改 `'mid360_real_flight[.]launch'`（字符类防自误杀；原模式 'roslaunch task_run_scripts real_flight.launch' 匹配不到新命令行——中间插了 fastlivo2/mid360_）。改后必须重启 nx_command_server（坑22）。**重启又踩 pkill 自杀**（"杀旧+启新"同一 plink -m 脚本 → pkill 匹配到自身 nohup 启动行 → exit 128）：安全流程= `ss -tlnp | grep 6004` 拿 PID → `kill -9 <PID>` → **单独启动脚本（无 pkill）** → 验证新 PID + curl 6004 健康检查 + 日志 profile='mid360'。备份 `User_define_commands.yaml.bak_longdist`
- **长距离开发四阶段**：0 修回归 → 1 提速（max_vel 0.3→1.0 + max_acc 0.4→0.6）→ 2 录 30-50m 航迹试飞判滚动/全程机制（日志 `Using FULL GLOBAL idx` 推进 vs failed 刷屏）→ 3 按结果（卡停则方案④先验 C++）→ 4 分批提交（task_run_scripts 推 feat/long-distance-replay，SUPER 续 feat/guide-path-c）
- **开发/飞行前必做**：`git branch --show-current`（三仓库）+ rosparam 确认实际生效配置（坑21：obstacle-to-px4 与 4uav-guide 的 config/launch 不同；坑28）
- fouruav IP 2026-08-24 实测 = **<NET_WLAN_A>.45**（47/144.14 已不用；hostkey SHA256:<ONBOARD_HOSTKEY_FP>… 不变=同一台机器——IP 会变但 hostkey 是稳定身份）

## 十一、⭐ 阶段0 最终修正（2026-08-24 晚，推翻上节中间结论）

**1. roslaunch 子目录根因修正（最重要）**：上节"按钮1/3 → `fastlivo2/mid360_real_flight.launch`"被实测推翻——`roslaunch task_run_scripts fastlivo2/mid360_real_flight.launch` 必 RLException。**真根因：文件名参数含 `/` 一律被当文件系统相对路径（相对 CWD）**；不含 `/` 的纯文件名会**递归搜索整个 launch/ 树（含子目录）**。实测 `roslaunch --nodes task_run_scripts mid360_real_flight.launch`（纯文件名）**成功**（5 节点，无参数/带参数都行）。**⚠️ 不要为"修复"在 launch/ 根目录复制同名文件**——会踩 `RLException: multiple files named [xxx.launch] in package`（agent 曾误建根目录转发文件踩中，删除即恢复）。验证法：`roslaunch --nodes task_run_scripts <纯文件名> [args]`（只解析不启动）。

**2. 按钮最终态（用户拍板序列）**：
- 按钮1 = **复飞**：`(nohup rosrun task_run_scripts global_path_publisher.py _file:=.../guidepoints_new.txt ... &) && sleep 1 && (nohup roslaunch task_run_scripts mid360_real_flight.launch use_waypoint:=true waypoint_data_path:=.../endpoint.txt ... &)`——纯文件名 launch（config_path/rviz_config 由包装默认值提供）
- 按钮2 = 关闭 SUPER（修复版，见下）；按钮3 = **"空按钮"**（用户自行改的，agent 改完必须复查 mtime+内容——坑63/64 同款；原 command 在备份 .bak_adjust2）

**3. 按钮2 "没反应+rviz 残留"双 bug 修复模板**（用户场景：启动雷达→按钮1 弹 rviz→按钮2 无反应 rviz 依旧）：
- ①`&&` 断链：`rosservice call ... && sleep && pkill ...`——mavros 不在时第一条非 0 → 全链断 → 后续 pkill 不执行 = "没反应"
- ②裸 pkill 模式自杀：command 以 `bash -c '整段'` 执行，bash 自身 argv 含 'fsm_node' 等文本 → `pkill -f 'fsm_node'` 杀自己（exit 128）→ 兜底杀 rviz 的 pkill 永远执行不到 = rviz 残留（坑10/23 同源，不限于 plink -m）
- **修复**（已上机）：`rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}' 2>/dev/null || true; sleep 1; pkill -SIGINT -f 'real_flight[.]launch' 2>/dev/null; sleep 2; pkill -f 'fsm_[n]ode' 2>/dev/null; pkill -f 'super_to_px4_bridg[e]' 2>/dev/null; pkill -f 'waypoint_missio[n]' 2>/dev/null; pkill -f 'map_to_worl[d]' 2>/dev/null; pkill -f 'top_down[.]rviz' 2>/dev/null; echo button2_done`——`|| true` 防断链 + **全部正则字符类**（正则仍匹配目标，不匹配自身 argv）+ echo 完成标记。pkill 模式用 `'real_flight[.]launch'`（子串同时匹配 mid360_real_flight.launch 和 real_flight.launch 两种命令行）

**4. RViz 绿线不显示 = rviz 配置缺 GlobalPathMkr 显示项**（按钮1 修好 launch 后用户报"轨迹没有显示"）：数据链路全通（发布器 log `发布 51 个航点`、FSM log `GLOBAL PATH received: 51 pts`、进程全在）但 `rostopic info /global_path_marker` 的 **Subscribers: None** → mid360_top_down.rviz（origin/main 新增 588 行）漏加 MarkerArray 显示项（坑25 同款，每个 rviz 配置都要加）。修复：block 格式 rviz 文件字符串 replace 在 Goal 显示块后插入 GlobalPathMkr 项（Class: rviz/MarkerArray, Topic: /global_path_marker, Namespaces: {global_path: true}）→ python yaml.safe_load 验证 → **按钮2→按钮1 重启才加载**。顺带："启动雷达"弹的 rviz 是 slam_rviz.rviz（点云），SUPER 的是 mid360_top_down.rviz，别混。

**5. 摸底补充（2026-08-24 机载实况）**：`map_sliding: {enable: true, threshold: 0.3}`（已开，段名 map_sliding 非 map_sliding_en）；**max_vel=0.3/max_acc=0.4/max_omg=2.5/max_acc_thr=20/penna_jerk=-5e5 全低速保守版**（长距离第一步提速）；scan_after_takeoff_en 在 super_to_px4_bridge.yaml（默认关，scan_angle_deg=90、scan_yaw_rate=5°/s）；guidepoints_new.txt 当前 2.5m/51 点（需录 30-50m）。goal_z 0.6/1.3、frontend=true、robot_r=0.35、corridor_bound_dis=0.6、omni=true、p_miss=0.49 均已就绪（ccbb463）。
