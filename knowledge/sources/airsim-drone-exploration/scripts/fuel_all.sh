#!/bin/bash
# ============================================================
#  一键: FAST-LIVO2 + MAVROS + 起飞 1.5m + FUEL 自主探索
#                                                    v2  2026-09-12
#
#  前置(唯一手动项): Windows 上 AirSim Blocks 已点 ▶ Play
#  用法: bash ~/fuel_all.sh [seed_z] [ctrl_mode] [goal_x] [pcd]
#        seed_z    悬停/起飞高度, 默认 1.5
#        ctrl_mode position(默认,稳) | raw(带速度/加速度前馈)
#        goal_x    触发点 x(Nav Goal), 默认 3.0 ; z 用 seed_z
#        pcd       1=退出时保存 PCD, 0=不保存(默认)   ← v2 新增
#  配套关闭: bash ~/fuel_stop.sh            (给它 SIGINT 才会存 PCD)
#            bash ~/fuel_stop.sh --all      (连 master 一起停)
#
#  它把"每次都会踩"的几步全包了:
#    ① 等 FAST-LIVO2 预热(30~60s,"No point" 属正常)
#    ② PX4 专用 MAVLink(14610/14611) —— PX4 一重启这条专线就没了
#    ③ 起 MAVROS 并自检 connected
#    ④ 起 FUEL 后**自动发 Nav Goal 触发探索** —— FUEL 不触发不飞!
#  v2 新增:
#    ⑤ **/clock 冻结硬检查** —— 点云"时有时无"的真因是仿真冻结, 不是等不够久
#    ⑥ 起飞超时**给诊断**(打印 z/mode/armed + 三类原因)而非干等
#    ⑦ 第 4 个参数 pcd → 导出 PCD_SAVE 给 twin_livo_fast.sh
# ============================================================
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash
source ~/fuel_ws/devel/setup.bash

SEED_Z=${1:-1.5}
MODE=${2:-position}
GOAL_X=${3:-3.0}
PCD=${4:-0}
export PCD_SAVE=$PCD      # twin_livo_fast.sh 读它来决定是否 rosparam set pcd_save_en

say(){ echo "[fuel_all] $*"; }

# ---------- 0) AirSim ----------
say "0/6 检查 AirSim (RPC 41451)..."
if timeout 3 bash -c "echo > /dev/tcp/127.0.0.1/41451" 2>/dev/null; then
  say "     [OK] AirSim 在 Play"
else
  say "     [X] AirSim 不可连 —— 请先在 UE4 Blocks 窗口点绿色 Play，再重跑本脚本"
  exit 1
fi

# ---------- 1) twin 链路（v2 用 --full：自带 4560 / /clock / /twin / /cloud_registered 检查 + FK 自动重启）----------
say "1/6 启动 twin 链路（--full 会验证链路并对 FAST-LIVO2 自动重启）..."
bash ~/twin_livo_fast.sh --full

# ---------- 2) ★ /clock 硬检查：仿真到底有没有在步进 ----------
# 这是"点云有时没有"的总开关：PX4 不在 / tmux 挂了 ⇒ 4560 断 ⇒ LockStep 不步进
# ⇒ /clock 停 ⇒ 所有传感器话题(含 /twin/lidar、/cloud_registered)全无数据。等再久也没用。
say "2/6 复核 /clock 步进（LockStep 冻结检测）..."
if timeout 6 rostopic hz /clock 2>/dev/null | grep -q "average"; then
  say "     [OK] /clock 在走"
else
  say "     [X] /clock 不走 → AirSim 未步进（PX4 没连上 4560 / tmux 挂了 / AirSim 没 Play）"
  say "         这是『点云有时没有』的真正原因：不是等不够久，而是仿真冻住了。"
  say "         检查: ss -tn | grep 4560 ; tmux ls ; tail -20 /tmp/px4_run.log"
  exit 1
fi

if timeout 6 rostopic hz /cloud_registered 2>/dev/null | grep -q "average"; then
  say "     [OK] /cloud_registered 有数据"
else
  say "     [X] FAST-LIVO2 无输出 —— 查 tail -20 /tmp/fastlivo.log"; exit 1
fi

# ---------- 3) PX4 专用 MAVLink ----------
say "3/6 开 PX4 专用 MAVLink (14610/14611)..."
tmux send-keys -t px4 "mavlink start -u 14610 -o 14611" Enter
sleep 3

# ---------- 4) MAVROS ----------
if pgrep -f mavros_node >/dev/null 2>&1; then
  say "4/6 MAVROS 已在跑"
else
  say "4/6 启动 MAVROS..."
  setsid roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610 > /tmp/mavros.log 2>&1 < /dev/null &
fi
ok=0
for i in $(seq 1 25); do
  if timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q "connected: True"; then ok=1; break; fi
  sleep 2
done
if [ $ok -eq 1 ]; then say "     [OK] MAVROS 已连 PX4"; else
  say "     [X] MAVROS 未连上 —— 查 tail /tmp/mavros.log（多半是 PX4 的 14610 没开成功）"; exit 1
fi

# ---------- 5) FUEL（含控制桥，自动起飞到 seed_z）----------
say "5/6 启动 FUEL（控制桥 seed_z=$SEED_Z, mode=$MODE, PCD=$PCD）→ 自动起飞..."
pkill -f exploration_node   2>/dev/null
pkill -f fuel_input_bridge  2>/dev/null
pkill -f fuel_ctrl_bridge   2>/dev/null
sleep 2
setsid roslaunch fuel_bridge fuel_real.launch enable_ctrl:=true \
  ctrl_mode:=$MODE seed_z:=$SEED_Z > /tmp/fuel_real.log 2>&1 < /dev/null &

# v2: 超时给诊断，不再干等
say "     等起飞到 ${SEED_Z} m（seed 悬停 → 切 OFFBOARD）..."
ok=0
for i in $(seq 1 30); do
  z=$(timeout 4 rostopic echo -n1 /mavros/local_position/pose/pose/position/z 2>/dev/null | head -1)
  st=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep -oE 'mode: "[^"]*"|armed: (True|False)' | tr '\n' ' ')
  if [ -n "$z" ] && awk -v a="$z" -v b="$SEED_Z" 'BEGIN{exit !(a > b-0.35)}'; then
    ok=1; say "     [OK] 高度 ${z} m | $st"; break
  fi
  [ $((i % 5)) -eq 0 ] && say "     ... 高度=${z:-?} | $st （已 $((i*2))s）"
  sleep 2
done
if [ $ok -eq 0 ]; then
  say "     [X] 未起飞到 ${SEED_Z}m —— 诊断："
  say "         当前: z=${z:-无数据} | $st"
  say "         控制桥日志: tail -15 /tmp/fuel_real.log"
  say "         常见原因: (a) 不是 OFFBOARD → 控制桥 set_mode 失败（MAVROS 没连）"
  say "                   (b) 一直 armed=False → PX4 拒绝 arm（查飞控状态）"
  say "                   (c) seed 没发 → /mavros/local_position/pose 无数据（EKF 未就绪）"
  exit 1
fi

# ---------- 6) 触发探索（不触发它永远悬停！见 references §11）----------
say "6/6 触发探索（发 2D Nav Goal）..."
timeout 6 rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: ${GOAL_X}, y: 0.0, z: ${SEED_Z}}, orientation: {w: 1.0}}}" >/dev/null 2>&1
sleep 6

if grep -qE "Triggered|PLAN_TRAJ" /tmp/fuel_real.log 2>/dev/null; then
  say "     [OK] 探索已触发"
else
  say "     [!] 未见 Triggered —— 查 tail -20 /tmp/fuel_real.log"
  exit 1
fi

echo ""
say "==================== 完成 ===================="
say "FUEL 日志 : tail -f /tmp/fuel_real.log"
say "无人机位置: rostopic echo /mavros/local_position/pose/pose/position"
say "探索耗时  : grep -E 'Cost mat|Total time' /tmp/fuel_real.log"
say "关闭      : bash ~/fuel_stop.sh          # 默认留 master"
say "            bash ~/fuel_stop.sh --all    # 全停"
[ "$PCD" = "1" ] && say "PCD       : 关闭时存到 ~/twin_ws/src/FAST-LIVO2/Log/pcd/"
