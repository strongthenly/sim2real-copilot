#!/bin/bash
# ============================================================
#  twin 数字孪生栈 —— 一键关闭（与 templates/twin_oneclick_fast.sh 配对）
#  用法: bash ~/twin_livo_stop.sh [--all]
#
#  ★ 默认（用户 2026-09 明确要求的取舍）:
#      停 桥 / FAST-LIVO2 / RViz / **PX4**，但 **保留 ROS master 常驻**
#    理由：master 本体很轻（rosmaster 2s 就能起），但 **roscore 包装脚本重启一次要
#          140s**（见 SKILL.md 排障 §3）→ 留着它，下次启动只重起 PX4/桥/算法（约 20-40s）。
#          **别为了"清干净"顺手杀 master —— 那会让下次启动多等两分多钟。**
#    --all: 连 rosout / rosmaster / roslaunch 一起清（彻底清场，下次完整启动）
#
#  ⚠️ 本文件的存在理由之一就是 pkill 的自杀坑 —— 照抄时务必保留 `pkill -x`：
#     规则：进程名是单个可执行名（rviz / px4 / rosmaster / roscore）一律用 `pkill -x`；
#           必须用 -f 时写成不含连续目标串的正则（如 `pkill -f 'rv[i]z'`）。
#           自查：脚本文件名 + 脚本内容里出现的每个词，是否会被自己的 pkill 命中。
# ============================================================
ALL=0
[ "$1" = "--all" ] && ALL=1

echo "=== 停算法 + 仿真 ==="
pkill -f fastlivo_mapping        2>/dev/null && echo "  - FAST-LIVO2"
pkill -x rviz                    2>/dev/null && echo "  - RViz"          # ← 必须 -x，别用 -f
pkill -f "roslaunch twin_bridge" 2>/dev/null && echo "  - 桥(roslaunch)"
pkill -f bridge.py               2>/dev/null && echo "  - 桥(bridge.py)"
pkill -f "rosbag play"           2>/dev/null && echo "  - rosbag play"
tmux kill-session -t px4         2>/dev/null && echo "  - PX4 (tmux 会话)"
pkill -x px4                     2>/dev/null && echo "  - PX4 进程"

if [ "$ALL" = "1" ]; then
  echo "=== --all: 连 ROS master 一起停 ==="
  pkill -f roslaunch 2>/dev/null
  pkill -f rosout    2>/dev/null && echo "  - rosout"
  pkill -f rosmaster 2>/dev/null && echo "  - rosmaster"
  pkill -f roscore   2>/dev/null
  echo ""
  echo "⚠️  已彻底清场（连 master 也停了）"
  echo "   下次启动会慢：master ~6s + PX4 ~20-40s"
else
  echo ""
  echo "✅ 已停：算法 + PX4；ROS master 保留常驻"
  echo "   → 下次: bash ~/twin_livo_fast.sh   （重起 PX4+桥+算法，约 20-40s）"
  echo "   → 要连 master 一起停: bash ~/twin_livo_stop.sh --all"
fi

sleep 2
echo ""
echo "=== 剩余进程 ==="
pgrep -af "fastlivo_mapping|bridge.py|bin/px4|rosmaster" 2>/dev/null | head -6 || echo "  （无）"
pgrep -ax rviz 2>/dev/null
echo ""
echo "提示: AirSim(Blocks) / QGC 在 Windows 侧，需手动关闭（关窗口 / Esc）"
