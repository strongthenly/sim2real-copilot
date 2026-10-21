#!/bin/bash
# 方案 B+ 一键打包: 全量 src + devel, 防源码泄露 (.py->.pyc, C++ 源码删, 带可运行环境)
# 2026-08-25 fouruav 实测。改 WORKSPACE 变量即可用于其他机器。
# 用法: 改完变量后 bash package_src_pyc.sh  （机载上执行，Windows 侧用 plink -m 传）
set -e
TS=$(date +%Y%m%d_%H%M%S)
WORKSPACE=~/catkin_ws
PKG=/tmp/ws_src_pkg_$TS
rm -rf "$PKG"; mkdir -p "$PKG"

echo "=== 0. 体积侦察 (先看大头, 排除巨型数据文件) ==="
du -sh "$WORKSPACE"/src/*/ 2>/dev/null | sort -rh | head -10
find "$WORKSPACE/src" -type f \( -name "*.pcd" -o -name "*.bag" -o -name "*.ply" -o -name "*.bin" -o -name "*.mp4" \) -printf '%s %p\n' 2>/dev/null | sort -rn | head -8 | awk '{printf "  %.1fMB %s\n", $1/1048576, $2}'
echo ">>> 若上面有 GB 级文件, 先在下方 EXCLUDE_DATASET=1 排除再继续"

# 是否排除点云/bag 等数据文件 (强烈建议 1)
EXCLUDE_DATASET=1

echo "=== 1. 拷贝 src ==="
cp -a "$WORKSPACE/src" "$PKG/src"
find "$PKG/src" -name ".git" -type d -exec rm -rf {} + 2>/dev/null || true
find "$PKG/src" -name ".git" -type f -delete 2>/dev/null || true
find "$PKG/src" -name "*.log" -delete 2>/dev/null || true
find "$PKG/src" -name "*.bak*" -delete 2>/dev/null || true
find "$PKG/src" -type d -name "log" -exec rm -rf {} + 2>/dev/null || true
if [ "$EXCLUDE_DATASET" = "1" ]; then
  find "$PKG/src" -type f \( -name "*.pcd" -o -name "*.bag" -o -name "*.ply" -o -name "*.bin" -o -name "*.mp4" \) -delete
fi

echo "=== 2. 删除 C++ 源码 ==="
CXX_COUNT=$(find "$PKG/src" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "*.cc" -o -name "*.cxx" -o -name "*.c" \) | wc -l)
find "$PKG/src" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "*.cc" -o -name "*.cxx" -o -name "*.c" \) -delete
echo "已删 C++ 源文件: $CXX_COUNT"

echo "=== 3. Python -> .pyc (同目录 xx.pyc; 编译失败保留 .py 并警告) ==="
find "$PKG/src" -name "*.py" -type f | while read f; do
  d=$(dirname "$f"); b=$(basename "$f" .py)
  if python3 -m py_compile "$f" 2>/dev/null; then
    pyc=$(find "$d/__pycache__" -name "${b}*.pyc" 2>/dev/null | head -1)
    [ -n "$pyc" ] && cp "$pyc" "$d/$b.pyc" && rm -f "$f" || echo "[警告] 无 pyc: $f"
  else
    echo "[编译失败-保留py] $f"
  fi
done
find "$PKG/src" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
echo "src 剩余 .py: $(find "$PKG/src" -name '*.py' | wc -l) (应=0), .pyc: $(find "$PKG/src" -name '*.pyc' | wc -l)"

echo "=== 4. launch 里 type=xx.py -> type=xx.pyc ==="
grep -rl 'type="[^"]*\.py"' "$PKG/src" --include="*.launch" 2>/dev/null | while read f; do
  sed -i 's/type="\([^"]*\)\.py"/type="\1.pyc"/g' "$f"; echo "  launch 已改: $(basename "$f")"
done

echo "=== 5. 拷贝 devel (可运行环境, 必须带) ==="
cp -a "$WORKSPACE/devel" "$PKG/devel"
find "$PKG/devel" -name ".git" -type d -exec rm -rf {} + 2>/dev/null || true
find "$PKG/devel" -name "*.log" -delete 2>/dev/null || true
find "$PKG/devel" -name "*.bak*" -delete 2>/dev/null || true

echo "=== 6. devel 里 .py 转 pyc (保留 __pycache__ 供 import) ==="
find "$PKG/devel" -name "*.py" -type f | while read f; do
  python3 -m py_compile "$f" 2>/dev/null && rm -f "$f"
done
echo "devel 剩余 .py: $(find "$PKG/devel" -name '*.py' | wc -l) (应=0)"

echo "=== 7. 打包前自检 ==="
echo ".cpp: $(find "$PKG/src" -name '*.cpp' | wc -l)  .h: $(find "$PKG/src" \( -name '*.h' -o -name '*.hpp' \) | wc -l)  .py: $(find "$PKG/src" -name '*.py' | wc -l)  .git: $(find "$PKG/src" -name '.git' | wc -l)"
du -sh "$PKG"

echo "=== 8. 打包 (可选 strip 二进制: 去掉 debug_info+符号表防逆向, 体积减半) ==="
# for b in "$PKG"/devel/lib/*/*; do [ -x "$b" ] && file "$b" | grep -q ELF && strip --strip-unneeded "$b" 2>/dev/null; done
cd /tmp && tar czf ws_src_binary_$TS.tar.gz -C "$PKG" .
ls -la /tmp/ws_src_binary_$TS.tar.gz
echo "$PKG" > /tmp/last_pkg_path.txt
echo "完成: /tmp/ws_src_binary_$TS.tar.gz"
