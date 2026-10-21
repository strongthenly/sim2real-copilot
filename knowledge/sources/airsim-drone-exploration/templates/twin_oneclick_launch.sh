#!/bin/bash
# ============================================================================
#  一键启动: PX4(SITL) + AirSim 传感器桥 + FAST-LIVO2 + RViz + MAVROS(控制回路)
#  用法:  bash ~/<本脚本>.sh
#  前提:  Windows 侧 Unreal/Blocks 已在 Play（AirSim 在等 TCP 4560）
#  适用:  ROS1 Noetic 数字孪生工作区（如 ros1-twin + ~/twin_ws）
#
#  ⚠️ 使用前必改：脚本里出现的每个词，都不能被自己的 pkill -f 命中
#     （本模板一律用 pkill -x 精确进程名，故安全；若你要加 pkill -f <词>，
#      先确认脚本文件名/内容里没有该词的连续串，否则会自杀 → "Terminated"）
#
#  已内置的四个实战教训（详见 references/startup-chain-troubleshooting.md）：
#   ① LockStep：必须先等 PX4↔AirSim TCP4560 建立，桥才有数据（首帧约 30s）
#   ② run_id 冲突：清 roslaunch 要清"全部"，不是只清目标那个
#   ③ roscore 冷启动慢（可 2min+）：master 还在就复用 + 等待循环必须打印进度
#   ④ 不要无条件杀在跑的 PX4（重启一次多等几十秒，还打断 4560 链路）
# ============================================================================
set -u
WS=$HOME/twin_ws
PREFIX=$WS/src/FAST-LIVO2
PX4_DIR=$HOME/PX4-AirSim                       # PX4 v1.11.3（AirSim 专用版）
CFG=$PREFIX/config/TwinSim.yaml                # 算法配置（话题/lidar_type/标定）
CAM=$PREFIX/config/TwinSim_camera.yaml         # 相机配置（cam_model 必填！）
RVIZ_CFG=$PREFIX/rviz_cfg/fast_livo2.rviz      # Fixed Frame 应为 camera_init
BRIDGE_PKG=twin_bridge                         # AirSim → /twin/* 桥
LIDAR_TOPIC=/twin/lidar/points_preview

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

echo "[1/8] 清理旧节点（保留在跑的 PX4 与 roscore）..."
# ⚠️ pkill -x 精确匹配进程名；不要用 pkill -f <关键词>（会误杀命令行含该词的进程，含脚本自身）
pkill -x fastlivo_mapping 2>/dev/null
pkill -x rviz              2>/dev/null
pkill -f "rosbag play"     2>/dev/null
pkill -f "bridge.py"       2>/dev/null
# roslaunch 要清"全部"：残留的 roslaunch（如以前跑过的 roslaunch mavros）占着 master 的
# /run_id，会让新 roslaunch 报 RLException: run_id ... does not match
pkill -f roslaunch         2>/dev/null
# ⚠️ 这里**不杀** rosmaster/roscore：master 还在就复用（roscore 冷启动本机可慢到 2min+）
# ⚠️ 这里也**不杀** PX4（tmux kill-session -t px4）：见文件头 ④
for i in $(seq 1 10); do
  pgrep -f "rosmaster|roslaunch" >/dev/null 2>&1 || break
  sleep 1
done
echo "      残留 roslaunch/rosmaster: $(pgrep -f 'rosmaster|roslaunch' 2>/dev/null | wc -l) 个（期望 0）"
sleep 2

echo "[2/8] 检查/启动 PX4 (tmux 会话 px4) ..."
if pgrep -x px4 >/dev/null 2>&1 || pgrep -f "bin/px4" >/dev/null 2>&1; then
  echo "      PX4 已在跑（复用，不重启）"
elif [ -d "$PX4_DIR" ]; then
  # ⚠️ PX4 SITL 必须有 tty：后台无终端 → pxh> 读到 stdin EOF 会立刻 "Exiting NOW." 退出
  tmux kill-session -t px4 2>/dev/null
  tmux new-session -d -s px4 "cd $PX4_DIR && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log"
  echo "      已启动 PX4（tmux attach -t px4 查看 / Ctrl+B D 脱离）"
else
  echo "      ⚠️ 没找到 $PX4_DIR，跳过"
fi

echo "[3/8] 确保 ROS master 就绪 ..."
# ⚠️ roscore 冷启动慢的根因常是同一脚本前面的 make px4_sitl 抢 CPU（DNS 已实测排除）。
#    等待循环必须打印进度，否则界面静默 2 分钟 → 用户报"卡住了"，其实脚本还在正常走。
if timeout 6 rosnode list >/dev/null 2>&1; then
  echo "      master 已在跑，直接复用（省 2 分钟）"
else
  echo "      启动 roscore（本机可能需 1-3 分钟，进度见下）..."
  setsid roscore > /tmp/roscore.log 2>&1 < /dev/null &
  READY=0
  for i in $(seq 1 60); do
    if timeout 4 rosnode list >/dev/null 2>&1; then READY=1; echo "      master 就绪（第 ${i} 次探测）"; break; fi
    [ $((i % 10)) -eq 0 ] && echo "         ... 仍在等 master（${i}0s）"
    sleep 2
  done
  [ "$READY" = "1" ] || { echo "      ⚠️ master 未就绪（别用 /tmp/roscore.log 为空当判据，它输出本就少）"; }
fi

echo "[4/8] 等 PX4 <-> AirSim 建立 TCP4560 ..."
# ⚠️ AirSim 是 LockStep：必须先有 PX4 连上 4560，仿真才跑；否则 Sensors 零数据、桥空转。
#    判据 = ss 里出现 :4560 的 established（"有监听"不算）。
CONNECTED=0
for i in $(seq 1 40); do
  if ss -tn 2>/dev/null | grep -q ':4560'; then
    echo "      4560 已连接（第 $((i*3))s）"; CONNECTED=1; break
  fi
  [ $((i % 5)) -eq 0 ] && echo "      ... 等待中 $((i*3))s（确认 Windows 侧 Blocks 处于 Play）"
  sleep 3
done
[ "$CONNECTED" = "1" ] || echo "      ⚠️ 120s 未连上 → 查 Blocks 是否 Play；PX4 日志 /tmp/px4_run.log"

echo "[5/8] 检查/启动传感器桥 ..."
if timeout 6 rostopic echo -n 1 $LIDAR_TOPIC/header >/dev/null 2>&1; then
  echo "      桥已在跑，$LIDAR_TOPIC 有数据"
else
  setsid roslaunch $BRIDGE_PKG bridge.launch host:=127.0.0.1 imu_rate:=220 rgb_rate:=30 depth_rate:=5 \
       > /tmp/bridge.log 2>&1 < /dev/null &
  echo "      已启动桥，等首帧数据（约需 30s：等 LiDAR 初始化 + 仿真步进）..."
  OK=0
  for i in $(seq 1 25); do
    if timeout 4 rostopic echo -n 1 $LIDAR_TOPIC/header >/dev/null 2>&1; then
      echo "      $LIDAR_TOPIC 有数据（第 $((i*3))s）"; OK=1; break
    fi
    [ $((i % 5)) -eq 0 ] && echo "      ... 等桥出数据 $((i*3))s"
    sleep 3
  done
  if [ "$OK" != "1" ]; then
    echo "      ⚠️ 桥起了但 60s 无数据 → 先回查 [4/8] 的 TCP4560 是否 established + Blocks 是否 Play；日志末尾："
    tail -12 /tmp/bridge.log 2>/dev/null
  fi
fi

echo "[6/8] 加载配置 + 启动 FAST-LIVO2 ..."
rosparam load $CFG
rosparam load $CAM /laserMapping     # ⚠️ 相机配置必须 load 到 /laserMapping，即使 img_en:0
setsid rosrun fast_livo fastlivo_mapping __name:=laserMapping > /tmp/fastlivo.log 2>&1 < /dev/null &
sleep 6

echo "[7/8] 启动 RViz ..."
setsid rviz -d $RVIZ_CFG > /tmp/rviz.log 2>&1 < /dev/null &

echo "[8/8] 启动 MAVROS 控制回路（offboard / 接规划器必须有，否则 hover_verify 会一直卡在'等待 FCU 连接'）..."
# ① geoid 数据：缺了 MAVROS 会秒退（[FATAL] UAS: GeographicLib exception: File not readable
#    /usr/share/GeographicLib/geoids/egm96-5.pgm）
if [ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]; then
  echo "      ⚠️ 缺 geoid 数据 → sudo geographiclib-get-geoids egm96-5"
  echo "         或从同机已有该数据的发行版拷 /usr/share/GeographicLib/geoids/*"
fi
# ② 给 PX4 开专用 MAVLink 专线；默认 udp://:14540@...:14580 常报
#    [FATAL] FCU: DeviceError:udp:bind: Address already in use（14540 被占）
tmux send-keys -t px4 "mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard" Enter 2>/dev/null
sleep 3
pkill -f "roslaunch mavros" 2>/dev/null      # 安全：本脚本文件名不含该串
sleep 2
setsid roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610 \
     > /tmp/mavros.log 2>&1 < /dev/null &
sleep 15

echo "----------------------------------------"
rosnode list 2>/dev/null
echo "----------------------------------------"
rosnode list 2>/dev/null | grep -qi mavros \
  && echo "MAVROS: 已启动" \
  || echo "⚠️ MAVROS 没起 → 看 /tmp/mavros.log（缺 geoid？14540 被占？）"
timeout 8 rostopic echo -n 1 /mavros/state 2>&1 | grep -E "connected|mode" || true
echo "----------------------------------------"
timeout 6 rostopic hz /aft_mapped_to_init 2>&1 | head -3
echo "----------------------------------------"
echo "看坐标:   rostopic echo /aft_mapped_to_init/pose/pose/position"
echo "定点验证: python3 <hover_verify_ros1.py 的副本>  # 稳态漂移 <0.5m = 定点 OK"
echo "日志:     /tmp/px4_run.log /tmp/bridge.log /tmp/fastlivo.log /tmp/rviz.log /tmp/mavros.log"
echo "（RViz 不出图 → Global Options 的 Fixed Frame 填 camera_init）"
