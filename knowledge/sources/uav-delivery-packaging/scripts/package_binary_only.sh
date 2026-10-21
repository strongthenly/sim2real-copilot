#!/bin/bash
# 方案 B1: 只给编译好的二进制 + 运行配置, 零源码
# 用法: 改 SRC/PKG 变量后 plink -m 传远端执行, 或机载本地直接跑
# 前置: 已确认二进制为最新 (见 SKILL.md Step 2 新鲜度验证)
set -e
SRC=~/catkin_ws/src/task_run_scripts
PKG=/tmp/task_run_scripts_binary_pkg
OUT=/tmp/task_run_scripts_binary_latest.tar.gz
rm -rf "$PKG"
mkdir -p "$PKG/bin" "$PKG/config" "$PKG/launch" "$PKG/rviz" "$PKG/data_guidepoints"

echo "=== 1. 拷贝二进制 (ELF 编译产物) ==="
# 核心规划链 (super_planner + mission_planner) + task_run_scripts 节点
cp ~/catkin_ws/devel/lib/super_planner/fsm_node "$PKG/bin/"
cp ~/catkin_ws/devel/lib/super_planner/super_to_px4_bridge "$PKG/bin/"
cp ~/catkin_ws/devel/lib/mission_planner/waypoint_mission "$PKG/bin/"
for b in imu_tf_node obstacle_to_px4_node realman_monitor_node slam_to_mavros_node; do
  [ -f ~/catkin_ws/devel/lib/task_run_scripts/$b ] && cp ~/catkin_ws/devel/lib/task_run_scripts/$b "$PKG/bin/"
done
ls -la "$PKG/bin/"

echo ""
echo "=== 2. 拷贝运行配置 (yaml/launch/rviz/航迹, 排除 .py/.bak/日志) ==="
cp "$SRC"/config/*.yaml "$PKG/config/" 2>/dev/null || echo "!! config 拷贝异常"
cp "$SRC"/launch/real_flight.launch "$PKG/launch/" 2>/dev/null
mkdir -p "$PKG/launch/odin" "$PKG/launch/fastlivo2"
cp "$SRC"/launch/odin/odin_real_flight.launch "$PKG/launch/odin/" 2>/dev/null
cp "$SRC"/launch/fastlivo2/mid360_real_flight.launch "$PKG/launch/fastlivo2/" 2>/dev/null
cp "$SRC"/rviz/top_down.rviz "$SRC"/rviz/mid360_top_down.rviz "$PKG/rviz/" 2>/dev/null
cp "$SRC"/data_guidepoints/guidepoints_new.txt "$SRC"/data_guidepoints/endpoint.txt "$PKG/data_guidepoints/" 2>/dev/null

echo ""
echo "=== 3. 自检: 包内不得有源码/敏感文件 (全为 0 才通过) ==="
echo "--- .py 文件数 (应为 0) ---"
find "$PKG" -name "*.py" | wc -l
echo "--- .bak/.log/.git 文件数 (应为 0) ---"
find "$PKG" \( -name "*.bak*" -o -name "*.log" -o -name ".git" \) | wc -l
echo "--- 文本内容 IP/密码扫描 (应为空) ---"
grep -rniE "192\.168\.|10\.48\.|password|passwd|secret|token|ssh://|git@" "$PKG"/config "$PKG"/launch "$PKG"/rviz "$PKG"/data_guidepoints 2>/dev/null | head -5
echo "--- 包内所有文件清单 ---"
find "$PKG" -type f | sort

echo ""
echo "=== 4. 打包压缩 ==="
cd /tmp
tar czf "$OUT" -C "$PKG" .
ls -la "$OUT"
echo "打包完成: $OUT"
