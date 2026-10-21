#!/bin/bash
# ============================================================
# get_latest_ulog.sh —— 小白一键版
# 自动装依赖 + 自动探测串口/波特率 + 自动下载飞控最新 ulg
# 用法: bash get_latest_ulog.sh   (什么都不用填)
# 下载到: ~/ulog/
# ============================================================

echo "=============================================="
echo "  PX4 日志(ulg) 一键下载工具"
echo "=============================================="

# ---- [1/3] 自动安装依赖 ----
echo "[1/3] 检查依赖 ..."
if ! python3 -c "import pymavlink" 2>/dev/null; then
    echo "      首次使用，正在自动安装（约 1 分钟，请稍候）..."
    pip3 install pymavlink -i https://pypi.tuna.tsinghua.edu.cn/simple --user 2>/dev/null \
      || pip3 install pymavlink --user 2>/dev/null \
      || { echo "      【失败】依赖安装失败，请检查网络后重跑"; exit 1; }
fi
python3 -c "import pymavlink" 2>/dev/null && echo "      依赖就绪"

# ---- [2/3] 自动探测串口 + 波特率，下载最新 ulg ----
echo "[2/3] 正在寻找飞控（自动探测串口和波特率）..."
python3 - <<'PYEOF'
# -*- coding: utf-8 -*-
import os, sys, glob, time

OUTDIR = os.path.expanduser('~/ulog')

try:
    from pymavlink import mavutil
except Exception as e:
    print("【失败】缺少 pymavlink，请先运行: pip3 install pymavlink --user")
    sys.exit(1)

# 候选串口：USB 优先（独立通道，不影响 MAVROS），TELEM 兜底
usb_ports  = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
uart_ports = sorted(glob.glob('/dev/ttyTHS*') + glob.glob('/dev/ttyS*'))

# 波特率候选（PX4 默认 57600，其次 921600/115200/460800）
BAUD_LIST = [57600, 921600, 115200, 460800]

def try_connect(port, baud):
    """尝试用指定串口+波特率连飞控，连上返回连接对象，否则返回 None"""
    try:
        m = mavutil.mavlink_connection(port, baud=baud)
        hb = m.wait_heartbeat(timeout=2)
        if hb is not None:
            return m
        m.close()
    except Exception:
        pass
    return None

conn = None
port_used = baud_used = None

# 1) 先试 USB 串口（波特率通常无所谓，先 57600 再 921600）
for port in usb_ports:
    for baud in BAUD_LIST[:2]:
        m = try_connect(port, baud)
        if m:
            conn, port_used, baud_used = m, port, baud
            break
    if conn:
        break

# 2) 再试 TELEM 串口（试全部波特率）
if conn is None:
    for port in uart_ports:
        for baud in BAUD_LIST:
            m = try_connect(port, baud)
            if m:
                conn, port_used, baud_used = m, port, baud
                break
        if conn:
            break

if conn is None:
    print("")
    print("【失败】没找到飞控，请按下面逐条检查后重跑：")
    print("  1. 飞控是否已通电？（要插电池或接电源）")
    print("  2. 飞控 USB 线是否插到 NX 的 USB 口？（换根线，充电线不行）")
    print("  3. 如果走 TELEM 串口，确认 MAVROS 已停止（它占用了串口）")
    print("")
    print("  当前检测到的串口设备：")
    for p in (usb_ports + uart_ports):
        print("    " + p)
    if not (usb_ports + uart_ports):
        print("    （无，串口设备都没出现）")
    sys.exit(1)

print("      已连上飞控：串口 %s，波特率 %d" % (port_used, baud_used))

# ---- 初始化 MAVLink FTP ----
try:
    from pymavlink import mavftp
    sysid = conn.target_system or 1
    ftp = mavftp.MAVFTP(conn, target_system=sysid, target_component=1)
except Exception as e:
    print("【失败】FTP 初始化失败: %s" % e)
    print("      请升级 pymavlink: pip3 install -U pymavlink --user")
    sys.exit(1)

def list_dir(path):
    """列出目录，返回 [(name, is_dir), ...]，兼容不同 pymavlink 版本"""
    entries = None
    for meth in ('list_directory', 'ls', 'list'):
        fn = getattr(ftp, meth, None)
        if fn:
            try:
                entries = fn(path)
                break
            except Exception:
                continue
    if entries is None:
        return []
    out = []
    for e in entries:
        if isinstance(e, (tuple, list)):
            name = str(e[0])
            is_dir = bool(e[2]) if len(e) > 2 else False
        else:
            name = str(getattr(e, 'name', e))
            is_dir = bool(getattr(e, 'is_dir', False))
        out.append((name, is_dir))
    return out

def find_ulgs(path, depth=0):
    """递归找 .ulg 文件，返回完整路径列表"""
    if depth > 3:
        return []
    ulgs = []
    for name, is_dir in list_dir(path):
        full = path.rstrip('/') + '/' + name
        if name.lower().endswith('.ulg'):
            ulgs.append(full)
        elif is_dir:
            ulgs.extend(find_ulgs(full, depth + 1))
        elif '.' not in name:
            sub = find_ulgs(full, depth + 1)
            if sub:
                ulgs.extend(sub)
    return ulgs

print("[3/3] 正在读取飞控 SD 卡日志列表 ...")
ulgs = find_ulgs('/log')
if not ulgs:
    print("")
    print("【失败】飞控 /log 目录下没找到 .ulg 日志文件")
    print("  /log 目录内容：")
    for name, is_dir in list_dir('/log'):
        print("    %s%s" % (name, ' [目录]' if is_dir else ''))
    print("  提示：确认飞控 SD 卡已插入、且飞过（有飞行才会生成日志）")
    sys.exit(1)

ulgs = sorted(ulgs)
latest = ulgs[-1]
print("      共 %d 个日志，最新的是：%s" % (len(ulgs), latest))

os.makedirs(OUTDIR, exist_ok=True)
dst = os.path.join(OUTDIR, latest.replace('/', '_'))
if not dst.lower().endswith('.ulg'):
    dst += '.ulg'

print("      正在下载 ...")
ok = False
for meth in ('download', 'get'):
    fn = getattr(ftp, meth, None)
    if fn:
        try:
            fn(latest, dst)
            ok = True
            break
        except Exception as e:
            print("      %s 失败: %s" % (meth, e))
if not ok:
    print("【失败】下载出错，请把上面报错发给技术人员")
    sys.exit(1)

sz = os.path.getsize(dst) if os.path.exists(dst) else 0
print("")
print("==============================================")
print("  【成功】日志已下载完成")
print("  文件：%s" % dst)
print("  大小：%.2f MB" % (sz / 1024.0 / 1024.0))
print("  下一步：把该文件发给技术人员分析即可")
print("==============================================")
PYEOF
