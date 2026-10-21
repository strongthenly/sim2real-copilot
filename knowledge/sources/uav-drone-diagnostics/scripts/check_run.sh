#!/bin/bash
# 每轮建图关闭后的验收检查：节点残留 / 进程残留 / PCD 完整性 / 关闭顺序日志
# 用法: bash check_run.sh [轮次号]   （在 NX 上运行）
# 背景: 反复启动/关闭 FAST-LIVO2 的标准验收流程——残留检测是用户的一票否决项。
#   通过 QGC 按钮停止后，旧进程保存 PCD 需 30~90s，检查前先确认日志出现
#   "✅ 旧建图 PCD 保存完成" 或等待足够时间，否则会误报"进程残留"。
ROUND="${1:-?}"
echo "======== 第 ${ROUND} 轮检查 ========"

echo "--- [1/3] 节点残留检测 (rosnode list) ---"
source /opt/ros/noetic/setup.bash 2>/dev/null
export ROS_MASTER_URI=http://localhost:11311
NODES=$(timeout 10 rosnode list 2>&1)
echo "$NODES"
BAD=$(echo "$NODES" | grep -E 'laserMapping|livox|imu_tf|mavros|republish|realsense|fastlivo' || true)
if [ -n "$BAD" ]; then
    echo "❌ 残留节点: $BAD"
else
    echo "✅ 无建图节点残留"
fi

echo "--- [1b] 进程残留检测 (排除常驻视频流 rviz_full_stream2) ---"
# 常驻服务 rviz_full_stream2.py / full_screen_streamer 不算残留，必须排除，否则误报
PROCS=$(pgrep -af 'fastlivo|livox_ros|laserMapping|mavros|imu_tf|realsense|msg_MID360|roslaunch task_run_scripts' | grep -v 'check_run\|pgrep\|rviz_full_stream2\|rviz_full_stream' || true)
if [ -n "$PROCS" ]; then
    echo "❌ 残留进程:"; echo "$PROCS"
else
    echo "✅ 无残留进程"
fi

echo "--- [2/3] PCD 完整性 ---"
python3 /home/fouruav/catkin_ws/src/task_run_scripts/scripts/check_pcd.py 2>&1 | tail -6

echo "--- [3/3] 最新一轮关闭顺序日志 ---"
# 只取目录（排除 fastlivo2_launch.log 等文件），按时间排序取最新
LATEST_LOG=$(find /home/fouruav/catkin_ws/src/task_run_scripts/log -maxdepth 1 -type d -name 'fastlivo2_*' -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)
echo "日志目录: $LATEST_LOG"
if [ -n "$LATEST_LOG" ] && [ -d "$LATEST_LOG" ]; then
    echo "[启动行]"; grep -h 'Fast-LIVO2 启动' "$LATEST_LOG"/*.log 2>/dev/null | tail -1
    echo "[关闭序列]"
    grep -h '正在关闭\|✅ 旧建图\|✅ 可以安全\|⚠️ 未找到\|Raw point cloud data saved' "$LATEST_LOG"/*.log 2>/dev/null | tail -8
    echo "[节点退出]"
    grep -h 'killing on exit\|shutting down\|done' "$LATEST_LOG"/fastlivo.log 2>/dev/null | tail -4
else
    echo "⚠️ 未找到本轮日志目录（fastlivo2_时间戳_true/false），无法检查关闭顺序"
fi
echo "======== 第 ${ROUND} 轮检查结束 ========"
