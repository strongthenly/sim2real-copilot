# QGC 图传/投屏服务栈故障排查实录（2026-08-13 实测案例）

## 服务栈拓扑
- 拉起者：`~/.config/autostart/qgc_servers.desktop` → `start_qgc_servers.sh`（GNOME autostart，**仅登录时跑一次**；杀后必须手动 `nohup bash ... & disown` 重拉）
- 端口：TCP 6000 = 图传 worker（等遥控器连接）；UDP 6002 = nx_touch_server2（触控）；UDP 6003 = nx_command_server2（QGC 指令）；TCP 6004 = 指令桥（QGC 心跳目标）；11311 = roscore；8000 = nginx webdav；21 = FTP
- 进程树：start_qgc_servers.sh → roscore + nx_touch_server2.py + nx_command_server2.py + rviz_full_stream2.py（supervisor）→ `rviz_full_stream2.py --worker`（子进程）
- supervisor 逻辑（rviz_full_stream.py 514+ 行）：`signal.signal(SIGINT/SIGTERM, SIG_IGN)`；Popen `--worker`（preexec_fn=os.setsid）；心跳 `/dev/shm/rviz_stream_heartbeat.txt`，>4s 不更新 → worker.kill()；worker 死 → sleep 1s → 无限重生（约每 2 秒一轮）
- worker：`rospy.init_node('full_screen_streamer')`（**依赖 roscore**，master 不可达即崩）→ TCP 6000 listen(1) → 遥控器连接后 mss 截屏（虚拟屏 DP-1 1920x1080）+ JPEG 推流；把 QGC IP 写 `/dev/shm/qgc_ip.txt`
- 设置：`~/.config/liyumen/stream_settings.json`；分辨率白名单 {(640,360),(960,540),(1280,720),(1600,900),(1920,1080)}，超白名单回退 960x540；jpegQuality 35~85、fps 8~25
- 日志：`~/catkin_ws/src/task_run_scripts/log/latest/`（软链最新 RUN_TAG 目录，保留 10 份）：start_qgc_servers.log / roscore.log / nx_touch_server2.log / nx_command_server2.log / rviz_full_stream2.log

## 症状
遥控器虚拟投屏连不上。机载侧：supervisor 进程活着，但 `ss -tlnp` 无 6000；`rviz_full_stream2.log` 每 2 秒刷一条 `⚠️ 惊骇！图传进程崩溃 (退出码: 1)，准备重启...`（无限循环）。

## 排查实录（命令 + 判读）
1. `ss -tlnp | grep 6000` → 无 → worker 崩循环（supervisor 只是看门狗，本身不监听）
2. `tail log/latest/rviz_full_stream2.log` → 全是 supervisor 崩溃重试，**无 worker traceback**（代码 `except Exception as e: sys.exit(1)` 吞异常，不打印 e）
3. 前台跑 worker（source devel/setup.bash 后）：`timeout 20 python3 -u rviz_full_stream2.py --worker` → 退出码 1、零输出（异常被吞）。要拿真异常用 traceback 脚本：
   ```bash
   python3 -c "
   import sys; sys.argv=['x','--worker']
   import traceback
   from rviz_full_stream import FullScreenStreamer
   try:
       FullScreenStreamer().run()
   except Exception:
       traceback.print_exc()
   "
   ```
4. 看 `log/latest/roscore.log` → 关键证据：
   ```
   xml.etree.ElementTree.ParseError: not well-formed (invalid token): line 83, column 1
   ...
   RLException: Invalid <param> tag: Cannot load command parameter [rosversion]:
   command [['rosversion', 'roslaunch']] returned with code [1].
   WARNING: disk usage in log directory [/home/fouruav/.ros/log] is over 1GB.
   ```
   = rosversion 扫到坏 package.xml → roslaunch 加载 roscore 配置失败 → roscore 起不来。
5. 定位坏文件：
   ```bash
   for f in $(find /home/fouruav/catkin_ws/src /opt/ros/noetic/share -name package.xml 2>/dev/null); do
     python3 -c "import xml.etree.ElementTree as ET; ET.parse('$f')" 2>/dev/null || echo "BAD: $f"
   done
   # → BAD: /home/fouruav/catkin_ws/src/task_run_scripts/package.xml
   ```
6. `cd task_run_scripts && git status -s` → `UU package.xml`（both modified = **未解决的 rebase/merge 冲突**）；`grep -n '<<<<<<<\|=======\|>>>>>>>' package.xml` → 冲突标记（该案例 feat 分支加 `<depend>rviz</depend>` vs HEAD 的 tf2_ros/visualization_msgs）
   - 本案例 git 状态：`* (非分支，正变基 feat/obstacle-to-px4)`，即 rebase 卡在 package.xml 冲突，新机器克隆/拷贝过来后一直没解决。

## 根因链
git rebase 卡在 package.xml 冲突 → 冲突标记残留 → XML 不合法 → rosversion exit 1 → roslaunch RLException → roscore 起不来（11311 不监听）→ 图传 worker `rospy.init_node` 连不上 master 崩 → supervisor 无限重启 → TCP 6000 永不监听 → 遥控器投屏连不上。

## 修复步骤（用户规矩：先备份 + 贴 diff + 可回退）
1. 备份：`cp package.xml package.xml.bak.$(date +%Y%m%d_%H%M%S)`（案例：package.xml.bak.20260813_130454，untracked 留在工作区）
2. 删冲突标记行、**保留两侧内容**（合并语义：两边改动都要，不能只留一边）：
   ```bash
   python3 -c "
   lines=[l for l in open('package.xml').read().split(chr(10)) if not l.startswith(('<<<<<<<','=======','>>>>>>>'))]
   open('package.xml','w').write(chr(10).join(lines))"
   ```
3. 验证：`python3 -c "import xml.etree.ElementTree as ET; ET.parse('package.xml')"` 无报错；`source devel/setup.bash && rosversion roslaunch` 输出版本号（案例 1.17.4）
4. `git add package.xml && git -c core.editor=true rebase --continue`（`-c core.editor=true` 防编辑器卡死；后续 commit 有冲突逐个解）。案例：5 个 commit 全部应用成功，回到分支 feat/obstacle-to-px4 HEAD 86dd92e
5. 重启整套服务（见 SKILL.md 第 3 节命令）——**必须 pkill -9 清干净旧 supervisor**（忽略 SIGTERM），pgrep 确认零残留再重拉

## 重启实录要点
- `pkill -TERM -f start_qgc_servers.sh` 只杀了 bash 本体；**rviz_full_stream2.py（supervisor）没死**：① 命令行不含 "start_qgc_servers" 匹配不到；② 代码里 `SIG_IGN` 了 SIGTERM。残留 supervisor 继续拉 worker 占住 6000 → 新一套的 worker bind 失败 → 又崩循环 → **新旧两套 supervisor 抢端口**。
- 正确清理：TERM 一波 → `pkill -9 -f "rviz_full_stream2|nx_touch_server2|nx_command_server2|start_qgc_servers"` + `pkill -9 -f "rosmaster|rosout"` → sleep 2 → `pgrep -af "nx_|rviz_full_stream|rosmaster|roscore|start_qgc"` 应为空。
- roscore 从 `nohup roscore` 到 11311 LISTEN 需 **>20s**（本机 .ros/log 1.6GB，disk-usage 检查拖慢；首测 sleep 10 误判失败）。验证等 25s。
- 验证三连：① `ss -tlnp | grep -E "6000|6004|11311"` 全 LISTEN；② 心跳 `cat /dev/shm/rviz_stream_heartbeat.txt` 与 `date +%s` 差值 <2s；③ 日志出现 `等待遥控器连接 (端口 6000)...` + `[screen] 截图线程启动 monitors[1] 1920x1080` 且无"崩溃"行。

## 相关坑
- worker 依赖 roscore 是隐式的（rospy.init_node）：**图传挂了先查 roscore**，别在流代码里瞎找。
- 吞异常模式 `except Exception as e: sys.exit(1)` 在 rviz_full_stream2.py / rviz_full_stream.py 多处出现——所有"退出码 1 无日志"都按此处理。
- `.ros/log` 1.6GB（864 目录）：`source /opt/ros/noetic/setup.bash && rosclean purge -y` 可清，不影响运行。
- 新机器 SSH：fouruav@<NET_WLAN_B>.31（fouruav-3.local），密码 <SSH_PASSWORD>，hostkey `SHA256:<ONBOARD_HOSTKEY_FP>`。
- plink 远程命令里 `$?`/`$!` 会被 PowerShell 先展开 → 复杂脚本一律本地写好 → base64 → `echo $b64 | base64 -d | bash`（技能第 0 节已有配方）。
