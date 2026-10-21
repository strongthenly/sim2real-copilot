# git 多仓库合并 master 实录（2026-08-19，NX_code 合并 17 提交）

fouruav 机载代码分布在 3 个 git 仓库（嵌套子模块结构），遥控器/平板端软件升级后机载服务函数必须跟 master 同步。本文是完整操作实录，可复用。

## 三仓库拓扑（先分清再动手）

| 仓库 | 默认分支 | 工作分支 | 关系 |
|---|---|---|---|
| task_run_scripts | **origin/main**（无 master） | feat/4uav-guide | 主仓库，含 NX_code 子模块 |
| NX_code | **origin/master** | feat/4uav-guide | task_run_scripts 的嵌套子模块（scripts/NX_code） |
| SUPER | **origin/dev/4uav**（origin/HEAD 指向它） | feat/guide-path-c | 独立仓库 |

- 查"远端最新"：`git branch -r` 看默认分支名再 log，**别默认 master**（task_run_scripts 的 `git log origin/master` 空是正常的）
- 查差异规模：`git rev-list --left-right --count origin/<默认分支>...HEAD`（左=远端独有，右=本地独有）

## 合并流程（NX_code 示例）

```bash
cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code
# 1. 备份当前工作区
mkdir -p ~/merge_backup_$(date +%Y%m%d_%H%M%S)
cp odom_to_mavros.py Radar_Launcher.yaml nx_command_server.py ~/merge_backup_*/
# 2. 确认干净
git status --short   # 必须空
# 3. fetch + 差异规模
git fetch origin
git rev-list --left-right --count origin/master...HEAD
# 4. 冲突预测：merge-base + 两边 diff 取交集
BASE=$(git merge-base origin/master HEAD)
git diff --name-only $BASE origin/master | sort > /tmp/master_files.txt
git diff --name-only $BASE HEAD | sort > /tmp/head_files.txt
comm -12 /tmp/master_files.txt /tmp/head_files.txt   # = 冲突候选
# 5. merge（本地操作，不 push 不碰远端 master）
git merge origin/master
```

## 冲突解决（本会话 2 个）

### Radar_Launcher.yaml → 直接取 master 版
master 改用 `start_odin.sh`（统一启动脚本，已含 odom_to_mavros/odom_throttle 启动），本地的手动 `sleep 12 && python3 odom_to_mavros.py` 不再需要（重复启动会被单实例锁拒掉）。
```bash
git checkout --theirs Radar_Launcher.yaml
git add Radar_Launcher.yaml
```

### odom_to_mavros.py → master 版 + 保留本地保命修复
冲突块 = HEAD 侧(单实例锁+旧注释) vs master 侧(空)。正确合并：保留锁代码，删旧注释，丢弃 master 空侧。
- 保命修复确认：`grep -n '_LOCK\|0.05\|_last_pub' odom_to_mavros.py` 应都有
- 语法检查：`python3 -m py_compile odom_to_mavros.py`

## ⭐ 跨仓库依赖坑（最容易漏）

**子模块合并 ≠ 主仓库合并**。NX_code 的 master 版 `start_odin.sh` 依赖：
```
roslaunch task_run_scripts odin1_ros1.launch          # 在 task_run_scripts launch/odin/
roslaunch task_run_scripts odin_px4_and_others.launch # 同上（注释：含 odom_throttle + odom_to_mavros）
```
这些 launch 文件**只在 task_run_scripts 的 origin/main**（`git ls-tree origin/main -- launch/odin/` 确认有 odin1_ros1/odin_px4_and_others/odin_real_flight 三个），当前分支工作区没有。**NX_code merge 完必须继续 merge task_run_scripts origin/main**（当时 main 领先 27 提交），否则启动链断。

通用教训：**合并前检查启动链/依赖的每个文件在哪个仓库哪个分支**。

## ⭐ 轻量替代：定向拉取（用户选方案A，2026-08-19 实机）

**不想 merge 整个 task_run_scripts 时**（用户只要 NX_code 合并），用 `git checkout origin/main -- <path>` 只拉需要的文件/目录——不改分支历史、不产生 merge commit，文件以**未提交**状态进入工作区，验证正常后再决定是否提交：

```bash
cd ~/catkin_ws/src/task_run_scripts
git checkout origin/main -- launch/odin/              # 拉整个目录（3 个 launch）
git checkout origin/main -- config/control_command.yaml # 注意！odin1_ros1.launch 还依赖这个
# 验证：git status 显示 A(新增未提交)；ls launch/odin/
```

**odin1_ros1.launch 依赖清单（拉完逐个查，缺一个就启动失败）**：
1. `config/control_command.yaml`（`$(find task_run_scripts)/config/` 那份——cloud_reprojection/image_overlay 节点 rosparam load 它、start_odin.sh 也 sed 写它。**task_run_scripts 侧这份原本不存在**，odin_ros_driver/config/ 那份是另一个文件别混）
2. `rviz/odin_rviz_me.rviz`（当前分支已有 ✅）
3. odin_ros_driver 包本身（其 launch_ROS1/odin1_ros1.launch 是原版，start_odin.sh 用的是 task_run_scripts 包装版，两个不同）

**症状与验证**：
- 症状 = 平板点"启动感知系统"→ RViz 不弹、`host_sdk_sample`/`rviz` 进程都没有
- 日志铁证 = `RLException: [odin1_ros1.launch] is neither a launch file in package [task_run_scripts]`（在 `log/odin_<时间戳>_<true|false>/odin1_ros1.log`）
- 独立信号 = start_odin.sh 打 `⚠️ 未找到 .../config/control_command.yaml，跳过 recorddata 设置`
- 拉完验证 = `timeout 15 roslaunch task_run_scripts odin1_ros1.launch 2>&1 | head` 应看到 PARAMETERS 清单（cloud_reprojection_node/register_keys 等）而非 RLException
- ⚠️ roslaunch 解析成功 ≠ 传感器起来（还要 USB/权限），但 RLException 消失 = 文件依赖问题已解决

## 验证清单

```bash
# master 新增文件到位
ls nx_file_transfer.py nx_paths.py restart_qgc_services.sh start_odin.sh
# 保命修复在
grep -n '_LOCK\|0.05\|_last_pub' odom_to_mavros.py
# 取 master 版的文件与 master 零差异
git diff origin/master -- Radar_Launcher.yaml | wc -l   # = 0
# 语法
python3 -m py_compile odom_to_mavros.py
# launch 依赖
ls ~/catkin_ws/src/task_run_scripts/launch/odin/
# 分支独立 + master 未污染
git status -sb        # ## feat/4uav-guide...origin/feat/4uav-guide [ahead N]
git rev-parse origin/master
```

## 安全保证（用户硬性要求）

- merge 是本地操作，不 push 远端 master 就不会变
- 合并后分支身份不变（feat/4uav-guide），不是切到 master
- 回滚：未提交 `git merge --abort`；已提交 `git reset --hard HEAD~1`

## 提交顺序（嵌套子模块）

1. 先 NX_code 内部 commit → push NX_code
2. 再 task_run_scripts `git add scripts/NX_code`（指针更新）→ commit → push
3. push 后 `git status -sb` 无 ahead = 同步完成

## 服务重启（master 版，2026-08-19 实测）

合并后重启遥控器交互服务（不影响飞控，安全）：
```bash
# 停止：必须用"纯净"停止脚本——只含 pkill/pgrep 行，连 `cat <文件名>` 都会触发 pkill 自匹配自杀（坑43④）
for pid in $(pgrep -f '[n]x_command_server2'); do kill -9 $pid; done
for pid in $(pgrep -f '[r]viz_full_stream2'); do kill -9 $pid; done
for pid in $(pgrep -f '[n]x_touch_server2'); do kill -9 $pid; done
# 启动：master 统一入口 start_qgc_servers.sh（roscore + touch + command + stream）
cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code
nohup bash start_qgc_servers.sh > /tmp/start_qgc_servers_manual.log 2>&1 &
```
验证：三服务日志就绪（`HTTP 6004` / `UDP 6003` / `UDP 6002` / `等待遥控器连接 (端口 6000)`）+ `雷达配置已加载: profile='odin'`。日志目录 `log/<时间戳>/` + `log/latest` 软链（保留 10 份）。

## 平板按钮机制（顺带查清）

- 平板端定义按钮**显示文案**，点击发 `UserCmd1/2/3`（按序号）
- 机载 `nx_command_server.py:2585` 收到 → 读 `User_define_commands.yaml` 对应 command 执行
- `name` 字段只是显示名，机载不校验 → 平板显示"系统自检"还是"按钮3"由平板 APP 决定
- 改按钮功能 = 改 yaml command 字段；按钮每次点击重读 yaml，无需重启服务

## ⭐ 名字与功能错位坑（2026-08-19 用户踩到）

**master 提交 `4345230` 把 UserCmd3 的 name 从"按钮3"改成"系统自检"（对齐平板文案），但 command 没动**（还是复飞功能）→ 平板点"系统自检"按钮会**真的启动复飞**！改名字必须同步检查 command 是否匹配，别只改标签。

排查/修改命令：
```bash
grep -A3 'UserCmd[123]:' User_define_commands.yaml   # 看名字+命令本体
# 修改：备份 + python 精确替换 + yaml 验证
cp User_define_commands.yaml User_define_commands.yaml.bak_btn_$(date +%Y%m%d_%H%M%S)
python3 -c "import yaml; print(yaml.safe_load(open('User_define_commands.yaml')).keys())"
```
"空功能"占位命令：`echo "[系统自检] 未配置功能, 仅占位" && true`

## 按钮配置最终拍板（2026-08-19 用户决定）

| 槽位 | name | 功能 | command 要点 |
|---|---|---|---|
| UserCmd1 | 智能航线任务 | **复飞功能（原按钮3）** | global_path_publisher.py guidepoints_new.txt + real_flight.launch use_waypoint:=true endpoint.txt |
| UserCmd2 | 停止SUPER | 安全停止（不变） | 杀进程（不变） |
| UserCmd3 | 系统自检 | **空（无功能）** | echo 占位 |

- **用户明确"只要我之前推送分支配置的按钮3的功能"**——改配置前先问清要哪套功能（A=SUPER 动静态避障 real_flight.launch 默认航点 vs B=复飞 global_path+endpoint），别拿 master 版替代推送分支版
- **跨仓库坑症状**：合并后 UserCmd1 若仍指向 `odin_real_flight.launch`（不存在于当前分支）→ 点击启动失败报"launch 文件不存在" = 未合并 task_run_scripts origin/main 的直接症状（见上文跨仓库依赖坑）
