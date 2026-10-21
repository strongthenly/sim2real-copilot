#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D435 / USB 相机 → RTSP 图传服务（GStreamer + Jetson 硬件编码）

实测可用：Jetson Orin NX, Ubuntu 20.04, GStreamer 1.16.2,
          libgstrtspserver-1.0（系统已装，无需 apt 装 gstreamer1.0-rtsp）

拉流：rtsp://<机载IP>:8554/live
验证：ffprobe -rtsp_transport tcp -i rtsp://127.0.0.1:8554/live
     → Video: h264 (Constrained Baseline), yuv420p, 640x480, 14.99 fps

部署为 systemd user service：
    cp d435_rtsp_server.py /tmp/   # 或 /opt/rtsp/
    # ~/.config/systemd/user/d435-rtsp.service:
    # [Unit]
    # Description=D435 RGB RTSP Stream Service
    # After=network.target
    # [Service]
    # Type=simple
    # ExecStart=/usr/bin/python3 -u /tmp/d435_rtsp_server.py
    # Restart=on-failure
    # RestartSec=3
    # [Install]
    # WantedBy=default.target
    systemctl --user daemon-reload
    systemctl --user enable d435-rtsp.service
    systemctl --user start  d435-rtsp.service      # 注意：不要用 --now（plink 下会退 128）

═══════════════ 踩过的坑（2026-10 实测，全部导致 DESCRIBE 400）═══════════════
1) v4l2src 输出 caps 是 YUY2 而非 YUYV。写成 YUYV 会导致整条管线构建失败，
   在 RTSP 场景下只表现为 "400 Bad Request"，看不到真实原因。
2) 必须用 GstRtspServer.RTSPServer.new()，不要用 RTSPServer()。
   后者端口能 LISTEN、OPTIONS 返回 200，但 DESCRIBE 恒 400。
3) set_launch() 的管线字符串不能带外层括号 ()。
4) rtph264pay 必须命名为 pay0。
5) nvvidconv 不接受 YUY2/RGB，必须先 videoconvert 到 I420。
6) v4l2src 不要加 io-mode=2，D435 的 uvcvideo 驱动会拒。
═══════════════════════════════════════════════════════════════════════════
"""
import sys
import signal

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

Gst.init(None)

# ============ 可调参数 ============
DEVICE  = "/dev/video4"   # D435 RGB。用 `v4l2-ctl --list-devices` 确认哪个是 RGB
WIDTH   = 640
HEIGHT  = 480
FPS     = 15
BITRATE = 1200000         # bps
PORT    = "8554"
PATH    = "/live"
USE_HW  = True            # True=Jetson NVENC；False=软件 x264

if USE_HW:
    ENCODER = (
        f"nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
        f"nvv4l2h264enc bitrate={BITRATE} "
        f"iframeinterval={FPS * 2} insert-sps-pps=true"
    )
else:
    ENCODER = (
        f"x264enc tune=zerolatency speed-preset=ultrafast "
        f"bitrate={BITRATE // 1000} key-int-max={FPS * 2}"
    )

PIPELINE = (
    f"v4l2src device={DEVICE} ! "
    f"video/x-raw,format=YUY2,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
    f"videoconvert ! video/x-raw,format=I420 ! "
    f"{ENCODER} ! "
    f"rtph264pay name=pay0 pt=96 config-interval=1"
)


def main():
    factory = GstRtspServer.RTSPMediaFactory.new()
    factory.set_launch(PIPELINE)
    factory.set_shared(True)              # 多客户端共享同一条流

    server = GstRtspServer.RTSPServer.new()   # ← 必须用 .new()
    server.set_service(PORT)
    server.set_address("0.0.0.0")

    mounts = server.get_mount_points()
    mounts.add_factory(PATH, factory)

    server.attach(None)

    print("[INFO] RTSP 图传服务已启动", flush=True)
    print(f"[INFO] 拉流地址: rtsp://<本机IP>:{PORT}{PATH}", flush=True)
    print(f"[INFO] 编码器: {'Jetson NVENC 硬件' if USE_HW else 'x264 软件'}", flush=True)
    print(f"[INFO] 视频源: {DEVICE} {WIDTH}x{HEIGHT}@{FPS}fps", flush=True)

    loop = GLib.MainLoop()
    signal.signal(signal.SIGINT, lambda *a: loop.quit())
    signal.signal(signal.SIGTERM, lambda *a: loop.quit())
    loop.run()
    print("[INFO] 服务停止", flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
