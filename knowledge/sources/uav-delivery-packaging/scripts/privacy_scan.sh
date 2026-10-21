#!/bin/bash
# 发货前隐私扫描 (给客户部署前必跑)
# 用法: bash privacy_scan.sh <打包目录>
# 期望: token/密钥/SSH私钥 = 0; IP/绝对路径/用户名 列出来评估
PKG="${1:?用法: privacy_scan.sh <打包目录>}"
[ -d "$PKG" ] || { echo "!! 目录不存在: $PKG"; exit 1; }
echo "===== 隐私扫描: $PKG ====="

echo ""
echo "=== 1. IP 地址 (192.168/10./172.16-31/169.254) ==="
grep -rnoE "(192\.168\.[0-9]+\.[0-9]+|10\.[0-9]+\.[0-9]+\.[0-9]+|172\.(1[6-9]|2[0-9]|3[01])\.[0-9]+\.[0-9]+|169\.254\.[0-9]+\.[0-9]+)" "$PKG" 2>/dev/null | grep -v "\.pyc\|\.so" | head -20
echo "--- IP 出现文件数: $(grep -rlE '(192\.168\.|10\.|172\.(1[6-9]|2[0-9]|3[01])\.|169\.254\.)' "$PKG" 2>/dev/null | grep -v '\.pyc\|\.so' | wc -l) ---"

echo ""
echo "=== 2. 绝对路径 /home/<user> ==="
for u in fouruav ubuntu nvidia; do
  N=$(grep -rl "/home/$u" "$PKG" 2>/dev/null | grep -v "\.pyc\|\.so" | wc -l)
  [ "$N" -gt 0 ] && echo "/home/$u: $N 个文件" && grep -rl "/home/$u" "$PKG" 2>/dev/null | grep -v "\.pyc\|\.so" | head -8
done

echo ""
echo "=== 3. 用户名扫描 ==="
grep -rnoE "(fouruav|jihaofly|jihaoxiang)" "$PKG" 2>/dev/null | grep -v "\.pyc\|\.so" | head -10
echo "--- 含用户名文件数: $(grep -rlE '(fouruav|jihaofly|jihaoxiang)' "$PKG" 2>/dev/null | grep -v '\.pyc\|\.so' | wc -l) ---"

echo ""
echo "=== 4. token/密码/密钥 (期望 0) ==="
grep -rnoE "(github_pat_[A-Za-z0-9_]+|ghp_[A-Za-z0-9]+|password\s*[:=]|passwd\s*[:=]|secret\s*[:=]|token\s*[:=]|BEGIN (RSA|OPENSSH|EC) PRIVATE KEY)" "$PKG" 2>/dev/null | grep -v "\.pyc\|\.so" | head -10
echo "--- 扫描完成 ---"

echo ""
echo "=== 5. SSH 密钥/凭据文件 (期望 0) ==="
find "$PKG" \( -name "*.pem" -o -name "*.key" -o -name "id_rsa*" -o -name "id_ed25519*" -o -name "known_hosts" -o -name "*.p12" \) 2>/dev/null | head -10
echo "--- 结束 ---"

echo ""
echo "=== 6. 残留 .git / 日志 / 备份 ==="
echo ".git: $(find "$PKG" -name '.git' | wc -l)"
echo "*.log: $(find "$PKG" -name '*.log' | wc -l)"
echo "*.bak: $(find "$PKG" -name '*.bak*' | wc -l)"
