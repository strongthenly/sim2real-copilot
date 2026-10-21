#!/bin/bash
# ============================================================
#  仿真链路体检（ROS1 ros1-twin / AirSim + PX4 + twin_bridge + FAST-LIVO2 + FUEL）
#
#  用途：用户报"点云时有时无 / 没数据 / 算法不出图"时【第一件事】跑这个，
#        直接判出"仿真是否冻结"，避免误判成桥坏了/等不够久。
#
#  用法：bash check-sim-health.sh
#  注意：所有等待都有上限；不用 pkill，纯只读探针。
# ============================================================
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash 2>/dev/null
[ -f ~/twin_ws/devel/setup.bash ] && source ~/twin_ws/devel/setup.bash 2>/dev/null
[ -f ~/fuel_ws/devel/setup.bash ] && source ~/fuel_ws/devel/setup.bash 2>/dev/null

echo "================ 仿真链路体检 ================"
echo "ROS_MASTER_URI = $ROS_MASTER_URI"
echo ""

echo "--- 1) 进程 ---"
for p in "bin/px4" "bridge.py" "fastlivo_mapping" "mavros_node" \
         "exploration_node" "fuel_input_bridge" "fuel_ctrl_bridge"; do
  pgrep -f "$p" >/dev/null 2>&1 && echo "  OK   $p" || echo "  --   $p"
done
echo ""
echo "--- 2) tmux（无 px4 会话 = tmux server 死了，PX4 被 SIGHUP 杀）---"
tmux ls 2>&1 | head -5
echo ""
echo "--- 3) PX4 <-> AirSim TCP 4560 ---"
ss -tn 2>/dev/null | grep ':4560' | head -1 || echo "  [X] 无 4560 连接 —— PX4 没连上 AirSim"
echo ""
echo "--- 4) ★ /clock 是否在步进（LockStep 冻结的总开关）---"
if timeout 7 rostopic hz /clock 2>/dev/null | grep -q "average"; then
  echo "  [OK] /clock 在走 —— 仿真在步进"
  CLOCK_OK=1
else
  echo "  [X] /clock 不走 ⇒ 仿真【已冻结】"
  echo "      链条: PX4死/tmux死 -> 4560断 -> LockStep 不步进 -> /clock 停 -> 所有传感器话题全无数据"
  echo "      ⇒ 等 90s/900s 都没用，先修 PX4/4560，别查桥和算法"
  CLOCK_OK=0
fi
echo ""
echo "--- 5) 数据链路频率 ---"
hz(){ printf "  %-32s " "$1"; timeout 7 rostopic hz "$1" 2>/dev/null | grep -E "average rate" | head -1 || echo "无数据"; }
hz /twin/lidar/points_preview
hz /twin/imu/data_raw
hz /cloud_registered
hz /aft_mapped_to_init
echo ""
echo "--- 6) FUEL 侧（若在跑）---"
hz /odom_world
hz /map_ros/cloud
hz /sdf_map/occupancy_all
hz /planning/pos_cmd
echo ""
echo "--- 7) MAVROS / 飞控状态 ---"
timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -E "connected|armed|mode" | head -3 \
  || echo "  /mavros/state 无数据（MAVROS 没跑 / 没连）"
echo "  当前位置: $(timeout 5 rostopic echo -n1 /mavros/local_position/pose/pose/position 2>/dev/null | tr '\n' ' ')"
echo "  当前setpoint: $(timeout 5 rostopic echo -n1 /mavros/setpoint_position/local/pose/position 2>/dev/null | tr '\n' ' ')"
echo ""
echo "--- 8) FUEL 是否真的在探索（判据：Triggered! 与 [FSM]: state:）---"
if [ -f /tmp/fuel_real.log ]; then
  grep -E "Triggered|\[FSM\]: state" /tmp/fuel_real.log 2>/dev/null | tail -5 \
    || echo "  未见 Triggered/[FSM] —— 大概率停在 WAIT_TRIGGER（要 pub /move_base_simple/goal）"
  echo "  注: 'to visit:' / 'cost mat size' 出自 0.5s 的 frontierCallback，【不代表探索在跑】"
else
  echo "  /tmp/fuel_real.log 不存在（FUEL 没起过）"
fi
echo ""
echo "--- 9) FAST-LIVO2 是否已初始化完（冷启动 30~60s，期间 No point 正常）---"
tail -3 /tmp/fastlivo.log 2>/dev/null || echo "  无 /tmp/fastlivo.log"
echo ""
echo "================ 判读速查 ================"
[ "${CLOCK_OK:-0}" = "0" ] && echo "结论: 仿真冻结 —— 修 PX4(4560) 优先，别动桥/算法" \
                          || echo "结论: 仿真在步进 —— 再看第 8 项是否已触发探索"
