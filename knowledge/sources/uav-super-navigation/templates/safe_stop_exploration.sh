#!/bin/bash
# ============================================================
#  实机无人机「停止探索 / 任务急停」的安全模板（fouruav 可直接用）
#
#  ★★ 铁律：绝不停 FAST-LIVO2 定位链 ★★
#     FAST-LIVO2 → odom_to_mavros.py → /mavros/local_position/odom → PX4 视觉定位源
#     树林里没有 GPS ⇒ 连 POSCTL（定点悬停）都靠它
#     ⇒ 一断就是"一瞬间失去 SLAM，飞不了定点" —— 用户明确提过这条
#
#  ★★ 实现铁律：只用 rosnode kill + 精确节点名，一个 pkill pattern 都不用 ★★
#     理由：pkill -f 的 pattern 有撞名误杀风险；rosnode kill 是精确的。
#     附带好处：SUPER 的 rviz 精确名是 /rviz，而 FAST-LIVO2 的 rviz 叫 /rviz_<数字>
#     ⇒ 杀 /rviz 只会命中 SUPER 的，绝不会碰用户的定位 rviz ✓
#
#  ★★ 顺序 = 安全顺序 ★★：① 切 POSCTL（脱 OFFBOARD → 定点悬停）→ ② 停探索 → ③ 停 SUPER
#     永远先让飞机进入不需要 OFFBOARD setpoint 流的模式，再断软件。
#     这是照搬机上既有 UserCmd2「先定点再杀进程」的做法。
#
#  ★★ 不要写 set -u ★★
#     ROS 的 setup.bash 内部读 $ROS_DISTRO，首次 source 时它还没定义
#     ⇒ set -u 会把它变成致命错误 ⇒ 脚本第一步就死（按钮静默失败）
#
#  用法:
#     bash safe_stop_exploration.sh              # 只停探索（SUPER 与飞机不动）
#     bash safe_stop_exploration.sh --with-super # 连 SUPER 一起停（= 任务急停）
# ============================================================
source /opt/ros/noetic/setup.bash
source /home/fouruav/catkin_ws/devel/setup.bash   # ← 改成目标机的 ws
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

WITH_SUPER=0
[ "${1:-}" = "--with-super" ] && WITH_SUPER=1

# ---- 白名单：只停这些（改成目标机的实际节点名）----
FUEL_NODES="/goal_arbiter /exploration_node /fuel_input_bridge /traj_server /waypoint_generator"
SUPER_NODES="/fsm_node /super_to_px4_bridge /waypoint_mission /map_to_world /global_path_publisher"
RVIZ_NODES="/rviz"    # ★ 精确名：只命中 real_flight.launch 声明的 /rviz

# ---- 保护名单：必须一直活着（仅用于停前/停后比对，永不杀）----
PROTECT="/mavros /laserMapping /livox_lidar_publisher2 /imu_tf \
/odom_to_map_broadcaster /camera_initial_to_map_broadcaster /odom_to_mavros /rosout"

snap() {
  local L; L=$(timeout 6 rosnode list 2>/dev/null || echo "")
  for n in $PROTECT; do
    if echo "$L" | grep -qx "$n"; then echo "    [OK]   $n"; else echo "    [--]   $n  (不在)"; fi
  done
}

killnodes() {
  for n in $1; do
    if timeout 4 rosnode list 2>/dev/null | grep -qx "$n"; then
      echo "    kill $n"; timeout 4 rosnode kill "$n" >/dev/null 2>&1 || true
    fi
  done
}

echo "[stop] ============ 停止探索 ============"
echo "[stop] 停前快照（保护名单）:"; snap

echo
echo "[stop] ① 切 POSCTL（飞机脱离 OFFBOARD → 定点悬停）"
rosservice call /mavros/set_mode '{custom_mode: "POSCTL"}' 2>&1 | head -2 || true
sleep 1

echo
echo "[stop] ② 停探索"
killnodes "$FUEL_NODES"
sleep 1

if [ "$WITH_SUPER" = "1" ]; then
  echo
  echo "[stop] ③ 停 SUPER"
  killnodes "$SUPER_NODES"
  sleep 2
  echo
  echo "[stop] ④ 收掉 SUPER 的 RViz（精确名 /rviz）"
  killnodes "$RVIZ_NODES"
  sleep 1
else
  echo
  echo "[stop] ③ 跳过停 SUPER —— 飞机仍由 SUPER 托管"
fi

echo
echo "[stop] 停后快照（保护名单）★ 必须与停前一致:"
snap
echo
echo "[stop] 当前剩余节点:"
timeout 6 rosnode list 2>/dev/null | head -25
echo "[stop] ★ 未触碰 FAST-LIVO2 / MAVROS / roscore / PX4 / livox / 相机 / imu_tf"
echo "[stop] done"
