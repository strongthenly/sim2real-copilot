#!/usr/bin/env python3
"""Livox 雷达广播探测脚本 — 抓取并解析雷达每秒的 430 字节设备广播。

在 Jetson 机载电脑上运行（无需 sudo，绑定用户端口）：
    python3 livox_broadcast_probe.py [--port 56201] [--timeout 6]

输出：设备 SN、型号、固件、上位机 IP 配置（TLV 解析），用于判断
"雷达活着吗 / 雷达期望上位机是谁 / 网关是什么"。

用法示例（远程执行）：
    echo "cGxpbms=" | base64 -d   # 或直接 scp/粘贴本文件
"""
import argparse
import re
import socket
import struct
import sys
import time


def parse_ips(data: bytes):
    """扫描包中所有 192.168.x.y 形式的 IP，并标注出现次数。"""
    ips = {}
    for i in range(len(data) - 3):
        if data[i] == 192 and data[i + 1] == 168:
            ip = f"192.168.{data[i+2]}.{data[i+3]}"
            ips[ip] = ips.get(ip, 0) + 1
    return ips


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=56201,
                    help="监听端口（push_msg 主机端口，默认 56201）")
    ap.add_argument("--timeout", type=float, default=6.0)
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("", args.port))
    s.settimeout(args.timeout)
    print(f"[*] 监听 {args.port} 端口 {args.timeout}s，等待雷达广播...")
    try:
        data, addr = s.recvfrom(4096)
    except socket.timeout:
        print("[!] 超时，无广播（雷达未上电 / 网线问题 / 网段不对）")
        return 1

    print(f"[+] 收到 {len(data)} 字节，来自 {addr}")
    print(f"[+] 十六进制前64字节: {data[:64].hex(' ')}")

    # ASCII 字符串（SN、DevType、FmVer 等）
    print("[+] ASCII 字段:")
    for st in re.findall(rb"[\x20-\x7e]{6,}", data):
        print(f"    {st.decode()}")

    # 常见 IP 配置
    ips = parse_ips(data)
    if ips:
        print("[+] 包内 IP 字段（192.168.x.y）:")
        for ip, cnt in sorted(ips.items(), key=lambda x: -x[1]):
            print(f"    {ip}  (出现 {cnt} 次)")

    # TLV 粗解析（Livox 设备信息块，key 常见含义）
    # 04 = 雷达自身 IP/掩码/网关；06/07 = 点云/IMU 上位机 IP+端口
    for m in re.finditer(rb"\x04\x00\x0c\x00(.{12})", data):
        b = m.group(1)
        if len(b) == 12:
            lip = f"{b[0]}.{b[1]}.{b[2]}.{b[3]}"
            mask = f"{b[4]}.{b[5]}.{b[6]}.{b[7]}"
            gw = f"{b[8]}.{b[9]}.{b[10]}.{b[11]}"
            print(f"[+] TLV04 雷达网络配置: IP={lip} 掩码={mask} 网关={gw}")
    for m in re.finditer(rb"\x06\x00\x08\x00(.{8})", data):
        b = m.group(1)
        if len(b) == 8:
            hip = f"{b[0]}.{b[1]}.{b[2]}.{b[3]}"
            port = struct.unpack("<H", b[4:6])[0]
            print(f"[+] TLV06 点云上位机: {hip}:{port}")
    for m in re.finditer(rb"\x07\x00\x08\x00(.{8})", data):
        b = m.group(1)
        if len(b) == 8:
            hip = f"{b[0]}.{b[1]}.{b[2]}.{b[3]}"
            port = struct.unpack("<H", b[4:6])[0]
            print(f"[+] TLV07 IMU上位机: {hip}:{port}")

    print("[*] 判读: 有 SN + DevType = 雷达活着；TLV06/07 的 IP = 雷达期望的主机上位机 IP，主机网卡必须拥有它")
    return 0


if __name__ == "__main__":
    sys.exit(main())
