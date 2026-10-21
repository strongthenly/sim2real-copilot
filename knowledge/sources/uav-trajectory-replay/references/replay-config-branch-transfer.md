# 复飞配置的 git 分支间导入/恢复 + CH9 返航移除 + UserCmd 对照（fouruav，2026-08-25）

场景：task_run_scripts 需要在分支间搬移复飞配置（不切分支）、去掉发布器内嵌的 CH9 返航、核对平板按钮命令。

## 1. 分支间导入特定文件（不切分支）

需求：把 `origin/feat/long-distance-replay` 的复飞配置搬到当前 `main`，但保留 main 更新的 NX_code 子模块指针。

```bash
cd ~/catkin_ws/src/task_run_scripts
# 先备份（本会话惯例 .bak_import_时间戳）
cp f f.bak_import_$(date +%Y%m%d_%H%M%S)
# 从指定分支检出特定文件到工作区+索引（不切分支！）
git checkout origin/feat/long-distance-replay -- \
  scripts/global_path_publisher.py \
  data_guidepoints/guidepoints_new.txt \
  data_guidepoints/endpoint.txt \
  rviz/mid360_top_down.rviz
# 校验一致性（md5 对比本地 vs 分支版本）
git show "origin/feat/long-distance-replay:$f" | md5sum
```

要点：
- `git checkout <branch> -- <file>` 只搬文件不换分支，文件进入 **staged+modified 状态**（`git status` 显示 `M `）。
- **不含 `.git` 历史**：它只取该分支当前快照的 blob。
- 子模块指针（`scripts/NX_code`）**不包含在文件列表里就别动它** —— 想保留 main 的新指针就只列要搬的文件。
- 判"导入是否成功"用 md5 与 `git show branch:file` 对比，不要只看 git status。

## 2. 恢复工作区到 HEAD（撤销本会话所有导入/修改）

```bash
git checkout HEAD -- <file1> <file2> ...   # 恢复到 HEAD 版本（同时清 staged+worktree）
rm -f *.bak_import_*                        # 删除本会话创建的备份
# 验证：md5 与 HEAD 一致 + git status 只剩子模块指针漂移
```

`git status -sb` 中 `MM file` = 索引和 worktree 都有改动（先 checkout 导入又手工改了）；`git checkout HEAD -- f` 一次清掉。

## 3. 去掉发布器内嵌 CH9 返航（global_path_publisher.py 198→131 行）

要删的（grep 确认零残留）：
- 全局变量：`_returned`、`_last_ch9`、`_pub_goal`
- 函数：`publish_return_goal()`、`trigger_return()`、`rc_callback()`
- main() 里：`_pub_goal = rospy.Publisher("/waypoint_goal", ...)`、`rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback, ...)`、循环内 `if _returned: publish_return_goal()`
- import：`subprocess`、`from mavros_msgs.msg import RCIn`（PoseStamped 保留！build_msgs() 还在用）

保留：`~reverse` 反向发布参数、绿线 marker（/global_path_marker, LINE_STRIP）、/global_path 发布。
验证：`python3 -m py_compile` + `grep -n "CH9\|_last_ch9\|_returned\|_pub_goal\|RCIn\|subprocess\|publish_return_goal\|trigger_return\|rc_callback\|waypoint_goal"` 零输出。

## 4. User_define_commands.yaml 三态对照（NX_code 子模块内）

| 槽位 | 工作区临时测试版(08-21) | git HEAD 正式版 | 用户拍板定义(08-19) |
|---|---|---|---|
| UserCmd1 | "智能航线任务"→仅 CP_DIST=-1 | "SUPER航点任务"→odin_real_flight.launch | 智能航线任务=复飞（发布器+real_flight+endpoint） |
| UserCmd2 | "停止SUPER"→定点+杀进程 | 同左 | 停止 |
| UserCmd3 | "系统自检"→仅 CP_DIST=0.5 | "系统自检"→复飞命令 | 系统自检=空功能 |

坑：工作区版是 08-21 临时测试版（注释"原航线任务功能暂时移除"），**git HEAD 才是正式版**。改按钮命令前先 `git show HEAD:User_define_commands.yaml` 对照，别把临时版当基准。user 拍板(08-19) 的定义见主技能路径A 小节。

## 5. 相关脚本
- `scripts/check_git_sync.sh`（windows-ssh-automation 技能）— 多仓库同步检查（子模块 .git 是文件，用 `git -C d rev-parse --git-dir` 判定）。
