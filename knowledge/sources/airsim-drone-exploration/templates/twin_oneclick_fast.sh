#!/bin/bash
# ===========================================================================
# twin_livo_fast.sh  —  一键启动（默认只补"缺失"的组件；--full 才清理+验证）
#
# 设计要点（都是踩出来的，别改回去）：
#   1. 绕开 roscore：本机 roscore 包装脚本冷启动 ~140s，但它内部的
#      rosmaster(2s)/rosout(0s) 极快 -> 直接起这两个。全量启动 140s -> 6s。
#   2. 幂等：已在跑的一律复用，绝不无条件 kill（重启一次就是分钟级）。
#   3. 每个组件单独判断 + 报告"本次启动了哪些 / 耗时多少"，用户能一眼验证。
#   4. 两个脚本合并成一个：原来"全量 twin_livo_rviz.sh + 快补 twin_livo_fast.sh"
#      两套冗余，用户问"rviz 那个还要保留吗" -> 合并，rviz 版独有的
#      "清残留 roslaunch / 等 TCP4560 / 等 /twin 并验证" 降级为 --full 开关。
#
# 配套：twin_oneclick_stop.sh 默认 **停 PX4+桥+算法+RViz，只保留 ROS master 常驻**
#       （--all 才连 master 一起停）。这是让下次启动 0~6 秒的关键——
#       别为了"清干净"顺手杀 master，那会让下次启动多等两分钟。
#
# 用法: bash ~/twin_livo_fast.sh          # 日常（常驻时 0-2 秒）
#       bash ~/twin_livo_fast.sh --full   # 异常时：清残留 roslaunch + 启动 + 验证链路
# ===========================================================================
START=$(date +%s)
FULL=0
[ "$1" = "--full" ] && FULL=1

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash
ROOT=~/twin_ws/src/FAST-LIVO2
started=""
el(){ echo $(( $(date +%s) - START )); }

# ---- 0) --full: 清残留 roslaunch（防 run_id 冲突） ------------------------
if [ $FULL -eq 1 ]; then
  echo "  [full] 清理残留 roslaunch（防 run_id 冲突）..."
  pkill -f roslaunch 2>/dev/null
  sleep 2
fi

# ---- 1) ROS master（绕开慢的 roscore） ------------------------------------
if ! timeout 2 rosnode list >/dev/null 2>&1; then
  echo "  [1/5] 启动 ROS master（rosmaster+rosout，约 4 秒）..."
  setsid rosmaster --core -p 11311 > /tmp/rosmaster.log 2>&1 < /dev/null &
  for i in $(seq 1 30); do
    timeout 2 rosnode list >/dev/null 2>&1 && break
    sleep 1
  done
  rosparam set /rosdistro noetic 2>/dev/null      # roscore 原本会设这两个
  rosparam set /rosversion 1.17.4 2>/dev/null
  setsid rosrun rosout rosout __name:=rosout > /tmp/rosout.log 2>&1 < /dev/null &
  sleep 2
  started="$started master($(el)s)"
else
  echo "  [1/5] master 已在跑 ✓"
fi

# ---- 2) PX4（tmux 保 tty；已在跑就别重启） --------------------------------
if ! pgrep -f "bin/px4" >/dev/null 2>&1; then
  echo "  [2/5] 启动 PX4 ..."
  tmux kill-session -t px4 2>/dev/null
  tmux new-session -d -s px4 "cd ~/PX4-AirSim && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log"
  started="$started PX4"
else
  echo "  [2/5] PX4 已在跑 ✓"
fi

# ---- 3) 传感器桥（AirSim RPC 41451 -> /twin/*） ---------------------------
if ! pgrep -f "bridge.py" >/dev/null 2>&1; then
  echo "  [3/5] 启动仿真桥 ..."
  setsid roslaunch twin_bridge bridge.launch host:=127.0.0.1 imu_rate:=220 rgb_rate:=30 depth_rate:=5 \
      > /tmp/bridge.log 2>&1 < /dev/null &
  started="$started 桥"
else
  echo "  [3/5] 桥已在跑 ✓"
fi

# ---- 4) FAST-LIVO2（cam_model 必须 load，哪怕 img_en:0） ------------------
if ! pgrep -f "fastlivo_mapping" >/dev/null 2>&1; then
  echo "  [4/5] 启动 FAST-LIVO2 ..."
  rosparam load $ROOT/config/TwinSim.yaml
  rosparam load $ROOT/config/TwinSim_camera.yaml /laserMapping
  setsid rosrun fast_livo fastlivo_mapping __name:=laserMapping > /tmp/fastlivo.log 2>&1 < /dev/null &
  started="$started FAST-LIVO2"
else
  echo "  [4/5] FAST-LIVO2 已在跑 ✓"
fi

# ---- 5) RViz（Fixed Frame 须为 camera_init） ------------------------------
if ! pgrep -x rviz >/dev/null 2>&1; then
  echo "  [5/5] 启动 RViz ..."
  setsid rviz -d $ROOT/rviz_cfg/fast_livo2.rviz > /tmp/rviz.log 2>&1 < /dev/null &
  started="$started RViz"
else
  echo "  [5/5] RViz 已在跑 ✓"
fi

# ---- --full: 等待并验证链路（异常时才需要） -------------------------------
if [ $FULL -eq 1 ]; then
  echo ""
  echo "  [full] 等 PX4 <-> AirSim TCP4560（最多 120s；LockStep 下没它仿真不跑）..."
  for i in $(seq 1 40); do
    ss -tn 2>/dev/null | grep -q ':4560' && { echo "         ✅ 4560 已连接"; break; }
    [ $((i % 5)) -eq 0 ] && echo "         ... 等 4560（已 $(el)s）"
    sleep 3
  done
  echo "  [full] 等 /twin 出数据（最多 60s；桥首帧约 30s）..."
  for i in $(seq 1 20); do
    timeout 4 rostopic echo -n 1 /twin/lidar/points_preview/header >/dev/null 2>&1 && { echo "         ✅ /twin 有数据"; break; }
    [ $((i % 5)) -eq 0 ] && echo "         ... 等数据（已 $(el)s）"
    sleep 3
  done
fi

echo ""
echo "本次启动: ${started:-（无需启动, 全部已在跑）}"
echo "耗时: $(el) 秒"
# 实测：环境已就绪 = 0 秒；清空后全量 = 6 秒；--full = 4 秒（含验证）
