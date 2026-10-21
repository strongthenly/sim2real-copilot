#!/bin/bash
# 复飞系统一键状态检查（fouruav / SUPER + Path Replay）
# 用法：bash check_replay_state.sh  （在 fouruav 上执行；或 write_file 后 plink -m 发送执行）
# 只读侦察，不修改任何东西。覆盖：进程 / 输入源 / 航点参数 / HTTP状态 / 触发记录 / 残留。
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/fouruav/catkin_ws/devel/setup.bash 2>/dev/null
TS=/home/fouruav/catkin_ws/src/task_run_scripts

echo "=== 1. SUPER 进程（fsm_node/bridge/waypoint） ==="
ps aux | grep -E "fsm_node|super_to_px4_bridge|waypoint_mission" | grep -v grep | awk '{print $2, $11, $12}' | head -6
echo "(空=SUPER 未启动)"

echo ""
echo "=== 2. 关键服务存活（推流/SLAM/MAVROS/Livox） ==="
echo -n "推流 rviz_full_stream: "; pgrep -f "rviz_full_stream2" | head -1
echo -n "SLAM fastlivo: "; pgrep -f "fastlivo_mapping" | head -1
echo -n "MAVROS: "; pgrep -f "mavros_node" | head -1
echo -n "Livox: "; pgrep -f "livox_ros_driver2" | head -1

echo ""
echo "=== 3. 输入源（点云/飞控状态） ==="
timeout 6 rostopic hz /cloud_registered --window 5 2>&1 | tail -1
timeout 4 rostopic echo /mavros/state -n 1 2>/dev/null | grep -E "mode:|armed:" | head -2

echo ""
echo "=== 4. waypoint_mission 参数（确认数据源指向） ==="
rosparam get /waypoint_mission/ 2>&1

echo ""
echo "=== 5. Path Replay HTTP 状态 ==="
echo -n "check_replay_status: "; curl -s --max-time 3 http://127.0.0.1:6004/check_replay_status; echo ""
echo -n "check_file: "; curl -s --max-time 3 http://127.0.0.1:6004/check_file; echo ""
echo -n "check_pose: "; curl -s --max-time 3 http://127.0.0.1:6004/check_pose; echo ""

echo ""
echo "=== 6. 触发/航点日志（real_flight.log） ==="
grep -aE "Mavros triggered|Load [0-9]+ waypoints|Get goal|Fsm-Test" $TS/log/real_flight.log 2>/dev/null | tail -5 || echo "(日志为空或不存在)"

echo ""
echo "=== 7. 残留进程（top_down rviz / map_to_world） ==="
ps aux | grep -E "top_down|map_to_world" | grep -v grep | awk '{print $2, $11}' || echo "(无残留)"

echo ""
echo "=== 8. 录制轨迹文件状态 ==="
F=$TS/scripts/NX_code/My_Flight_Path.txt
if [ -f "$F" ]; then
  echo "行数(含表头): $(wc -l < $F)"
  python3 - "$F" <<'PYEOF'
import sys
raw = open(sys.argv[1], 'rb').read()
nul = raw.count(b'\x00')
print("NUL字节数: %d %s" % (nul, "(文件损坏,需清理)" if nul else "(正常)"))
PYEOF
else
  echo "不存在"
fi
