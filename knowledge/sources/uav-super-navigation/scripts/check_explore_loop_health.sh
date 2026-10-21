#!/bin/bash
# check_explore_loop_health.sh — 一气看完 FUEL→SUPER 探索闭环的健康状况
#
# 用途：飞机"能起飞但飞不动 / 最后卡住 / 撞障碍"时，一条命令拿到分层判据，
#       避免逐条手敲（本会话同一批检查手打了 6+ 次）。
#
# 用法：
#   bash ~/fuel_ws/src/fuel_bridge/scripts/check_explore_loop_health.sh [ROS_MASTER_URI]
#   默认 ROS_MASTER_URI=http://127.0.0.1:11311
#
# 判据对照（配套 SKILL.md 坑 P / 坑 Q）：
#   odom <20Hz        → MAVLink 流率被 Normal 模式掐死（坑 P）
#   pos_cmd 无数据     → SUPER 没出轨迹；再看 fsm 日志定性
#   Ill corridor / Cannot simplify sfcs → 安全走廊退化（目标超范围/地图没建到，坑 Q-1）
#   仲裁器连续同目标    → 死锁，别等它自解（坑 Q-1）
#   Next view 重复      → 飞机没动 ⇒ FUEL 判未到达 ⇒ 反复发同一目标（坑 Q-2）

set -o pipefail
URI="${1:-${ROS_MASTER_URI:-http://127.0.0.1:11311}}"
export ROS_MASTER_URI="$URI"

# ROS 环境：能 source 就 source，source 不到不致命（后面用超时兜底）
source /opt/ros/noetic/setup.bash 2>/dev/null || true

FSM_LOG="${FSM_LOG:-/tmp/super_fsm.log}"
FUEL_LOG="${FUEL_LOG:-/tmp/fuel_real_super.log}"
ARB_LOG="${ARB_LOG:-/tmp/goal_arbiter.log}"
BRIDGE_LOG="${BRIDGE_LOG:-/tmp/super_bridge.log}"

hr(){ printf '%s\n' "------------------------------------------------------------"; }
hz(){ timeout 8 rostopic hz "$1" 2>/dev/null | grep average | head -1 | sed 's/.*rate: //'; }
one(){ timeout 6 rostopic echo -n1 "$1" 2>/dev/null | grep -vE '^---' | head -"${2:-1}"; }
cnt(){ [ -f "$1" ] && grep -c -- "$2" "$1" 2>/dev/null || echo 0; }

echo "==================== 探索闭环体检 ===================="
echo "ROS_MASTER_URI = $URI"
hr

echo "[1] 进程"
for p in rosmaster mavros bin/px4 fsm_node super_to_px4_bridge \
         exploration_node traj_server waypoint_generator goal_arbiter \
         fastlivo_mapping rviz; do
  n=$(pgrep -f "$p" 2>/dev/null | wc -l)
  [ "$n" = "0" ] && mark=" ← 缺!" || mark=""
  printf '    %-24s %s%s\n' "$p" "$n" "$mark"
done
hr

echo "[2] 飞行状态（坑 O/M）"
st=$(one /mavros/state)
echo "    armed : $(echo "$st" | awk '/^armed:/{print $2}')"
echo "    mode  : $(echo "$st" | awk '/^mode:/{gsub(/"/,"",$2); print $2}')"
echo "    位置  : $(one /mavros/local_position/pose/pose/position | tr '\n' ' ')"
hr

echo "[3] ⭐ 关键频率（坑 P 的判据梯）"
printf '    %-34s %s\n' "/mavros/local_position/odom  (需≥20Hz)"  "$(hz /mavros/local_position/odom)"
printf '    %-34s %s\n' "/mavros/imu/data"                       "$(hz /mavros/imu/data)"
printf '    %-34s %s\n' "/cloud_registered"                      "$(hz /cloud_registered)"
printf '    %-34s %s\n' "/planning/pos_cmd  (SUPER 是否在出轨迹)"  "$(hz /planning/pos_cmd)"
printf '    %-34s %s\n' "/waypoint_goal"                         "$(hz /waypoint_goal)"
printf '    %-34s %s\n' "/fsm_node/rog_map/occ"                  "$(hz /fsm_node/rog_map/occ)"
hr

echo "[4] 防抢控制：/planning/pos_cmd 的发布者必须【只有 1 个】"
timeout 6 rostopic info /planning/pos_cmd 2>/dev/null \
  | sed -n '/Publishers/,/Subscribers/p' | grep '\*' | sed 's/^/    /'
echo "    （多发布者=有节点绕过改道在抢控制；注意先 pgrep 交叉确认不是僵尸注册，坑 H）"
hr

echo "[5] 目标范围（z 超 virtual_ceil 量化后的 ±2.25 → 走廊必退化，坑 Q-1）"
echo "    最新放行目标:"
timeout 8 rostopic echo -n1 /waypoint_goal 2>/dev/null \
  | grep -E "x:|y:|z:" | head -3 | sed 's/^/      /'
echo "    FUEL 最近 view:"
grep -E "Next view" "$FUEL_LOG" 2>/dev/null | tail -3 | sed 's/^/      /'
hr

echo "[6] ⭐ 死锁签名"
printf '    %-38s %s\n' "Ill corridor"                    "$(cnt "$FSM_LOG" 'Ill corridor')"
printf '    %-38s %s\n' "Cannot simplify sfcs"            "$(cnt "$FSM_LOG" 'Cannot simplify sfcs')"
printf '    %-38s %s\n' "GenerateExpTrajectory failed"    "$(cnt "$FSM_LOG" 'GenerateExpTrajectory failed')"
printf '    %-38s %s\n' "BackOpt failed"                  "$(cnt "$FSM_LOG" 'BackOpt')"
printf '    %-38s %s\n' "PlanFromRest failed"             "$(cnt "$FSM_LOG" 'PlanFromRest failed')"
echo
echo "    FSM 各状态停留（WAIT_GOAL 后期暴涨 = 收目标不动）:"
for s in WAIT_GOAL GENERATE_TRAJ FOLLOW_TRAJ AVOID_WAIT; do
  printf '      %-16s %s\n' "$s" "$(cnt "$FSM_LOG" "Current state: $s")"
done
echo
echo "    ★ 仲裁器是否在重复放行【同一个】目标（上一目标==本次目标 = 死锁）:"
tail -4 "$ARB_LOG" 2>/dev/null | sed 's/^/      /' || echo "      (无仲裁器日志)"
hr

echo "[7] 桥是否在位置保持（坑 O：有轨迹但飞机不动）"
tail -4 "$BRIDGE_LOG" 2>/dev/null | sed 's/^/    /'
if grep -q 'RC signal lost' "$BRIDGE_LOG" 2>/dev/null; then
  echo "    ⚠️ 出现 'RC signal lost ... Position hold' ⇒ 仿真无遥控器，failsafe_en 没关（坑 O）"
  echo "       修: rosparam set /super_to_px4_bridge/failsafe_en false"
fi
hr

echo "[8] 判读速查"
cat <<'EOF'
    armed=False / mode!=OFFBOARD          → 坑 O（auto_arm / failsafe_en）
    odom ~1Hz                             → 坑 P（mavlink stream 流率，跑 twin_livo_fast.sh 会自动设）
    pos_cmd 无数据 + Ill corridor         → 坑 Q-1（目标超已建图范围 / z 超 ±2.25）
    仲裁器连续放行同一目标                 → 坑 Q-1 死锁，须在 goal_arbiter 加超时放弃
    bridge 刷 POS_HOLD / RC signal lost   → 坑 O（failsafe_en）
    Next view 连续重复                     → 飞机没动 ⇒ FUEL 判未到达 ⇒ 反复发同一目标
EOF
echo "======================================================"
