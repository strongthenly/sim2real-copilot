---
name: uav-ros-sensor-diagnostics
description: Diagnose flight controller (PX4/ArduPilot via MAVROS serial) and LiDAR (Livox MID360 via Ethernet) connections on a ROS companion computer (Jetson NX). Covers serial recon, radar subnet config, launch-file inspection, MAVROS verification, and git recovery of zeroed files.
---

# UAV ROS Sensor Diagnostics (飞控 + 雷达连接测试)

## When to use
User wants to test flight controller (飞控) and radar/LiDAR (雷达) connections on a drone companion computer (Jetson NX class, ROS1 noetic common) — e.g. "测试飞控和雷达连接情况".

## Workflow preference (指挥官模式 — commander mode)
This user prefers: **agent performs the initial connection, then hands command batches to the user, who runs them on the remote terminal and pastes output back**. Verify each batch's output before issuing the next. Keep commands read-only until the picture is clear. Do NOT run remote commands yourself if the user said they will operate.

## Phase 1 — Recon (read-only)
```bash
ls -l /dev/ttyTHS* /dev/ttyS* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null   # serial devices (Jetson UART = ttyTHS*)
ip -4 addr show | grep -E "^[0-9]+:|inet "                            # NICs + subnets
ls ~/catkin_ws/src/                                                    # packages reveal the stack
cat ~/catkin_ws/src/*/launch/*.launch 2>/dev/null | head -100          # launch files = the system's "manual"
ps aux | grep -iE "livox|mavlink|px4|ros" | grep -v grep               # what is already running
```
- **Jetson UART ports are `/dev/ttyTHS*`** — the flight controller usually lives there, NOT on ttyACM/ttyUSB
- Package names tell the story: `mavros`, `livox_ros_driver2`, `FAST_LIO/FAST-LIVO2`, `slam_to_mavros` → PX4 + Livox + LIO-SLAM stack
- Note NIC MAC prefixes: onboard Jetson NIC has NVIDIA OUI (3c:6d:66...), USB-adapter NICs often have locally-administered MACs (86:...)

## Phase 2 — Flight controller (MAVROS over serial)
- Find FCU port + baud in launch files: `fcu_url` e.g. `/dev/ttyTHS0:921600`
- **Raw serial test MUST set the baud rate** — plain `cat` uses 115200 default and shows nothing even when the FC is alive:
  ```bash
  timeout 3 bash -c 'stty -F /dev/ttyTHS0 921600 raw -echo; cat /dev/ttyTHS0' | xxd | head -20
  ```
  Expect MAVLink v2 frames `fd ...` (v1 = `fe ...`). No output → retry with 115200; still nothing → FC unpowered / cable wrong / wrong port.
- Start MAVROS: `source ~/catkin_ws/devel/setup.bash && roslaunch <pkg> <file>.launch`
  - **Success markers**: `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot` + `MAVROS started. MY ID 1.240, TARGET ID 1.1` + `IMU: ... detected!` + `RC_CHANNELS message detected!`
  - `CMD: Unexpected command 520` is a benign PX4 reply, not an error
- Verify topics: `rostopic echo /mavros/state -n 1` (connected/armed/mode), `rostopic hz /mavros/imu/data --window 10` (~50 Hz normal)

## Phase 3 — LiDAR (Livox MID360 over Ethernet)
- Read driver config JSON: `host_net_info.cmd_data_ip` = the **host** IP the NIC must carry; `lidar_configs[].ip` = the **lidar's** IP
- Add the required subnet to the NIC the radar cable is in (temporary, non-destructive):
  ```bash
  sudo ip addr add <NET_LAN2>.5/24 dev eth1
  ```
- **Ping the LIDAR's IP, not the host IP** — pinging your own host IP always succeeds (false positive!)
- **IP-conflict trap (radar unreachable root cause)**: if the host NIC is given the SAME IP as the radar's fixed IP, the radar is effectively hijacked — the host answers for it, Livox SDK finds no device. `ping <that IP>` returns ~0.08 ms with `ttl=64` (host NIC replying for itself). The REAL radar answers with `ttl=255` and 1–3 ms RTT. Diagnosis: ask the user what the radar's IP is / check `ttl` and RTT. Fix: move the HOST to a different address on the same subnet and let the radar keep its IP:
  ```bash
  sudo ip addr del <NET_LAN2>.5/24 dev eth1 && sudo ip addr add <NET_LAN2>.2/24 dev eth1
  ```
- Interpret results: `目标主机不可达` (Dest Unreachable) from your own IP = ARP unanswered → radar unpowered / wrong port / wrong IP; timeout = packets dropped. `ip neigh show` reveals who actually answered.
- **SDK init success ≠ radar connected**: `Init livox lidars succ` only means the SDK started. Look in the driver log (`~/.ros/log/*/livox_lidar_publisher2*.log`, path carries a timestamp; `latest/` may be stale) for device SN / `online` / `connect timeout`.
- **`Params check failed, all livox lidars config is empty` = wrong JSON schema**: Livox driver2 SDK requires `host_net_info` as an OBJECT with `cmd_data_ip`/`push_msg_ip`/`point_data_ip`/`imu_data_ip` keys. A file written with an ARRAY of `{"host_ip": ...}` objects parses as "no config". Compare against a known-good file; the launch file's `user_config_path` param decides which JSON is loaded (`grep user_config_path <launch>`).
- **Multiple config files may disagree on the radar IP** (e.g. `MID360_config.json` says .196, `MID360s_config.json` says .175). User-stated device IP is ground truth — verify by ping TTL before editing.
- Top hardware suspects for unreachable MID360: power (needs 12–24 V), cable/port, IP drift. Subnet sweep to find a live IP: `for i in $(seq 1 254); do ping -c 1 -W 0.2 192.168.2.$i & done; wait`

## Config-edit hygiene (user preference: 更体面)
User dislikes whole-file heredoc overwrites for config tweaks. Preferred dignified workflow:
1. Backup: `cp cfg.json cfg.json.bak`
2. Structured edit with python3 (only touch target fields, keep formatting valid):
   ```bash
   python3 -c "
   import json
   p='cfg.json'; d=json.load(open(p))
   for k in ['cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip']:
       d['MID360']['host_net_info'][k]='<NET_LAN2>.2'
   d['lidar_configs'][0]['ip']='<NET_LAN2>.5'
   json.dump(d,open(p,'w'),indent=2); print('OK')"
   ```
3. Validate: `python3 -m json.tool cfg.json | grep '"ip"'` (also proves JSON is legal)
Manual `nano` editing is acceptable; heredoc overwrite only as last resort.

## Pitfalls
- `roslaunch px4.launch` (bare filename) fails with `Invalid roslaunch XML syntax: no element found` — always `roslaunch <package> <file>.launch` after sourcing `devel/setup.bash`.
- **Zeroed launch files**: if several `.launch` files are 0 bytes with the same mtime, they were bulk-overwritten (e.g. XML pasted into a shell hit a `>` redirect). Find all victims: `ls -la <dir>/ | grep " 0 "`. Recovery: `git checkout -- launch/` if the package is a git repo (check `git status` first).
- `Cannot load message class for [livox_ros_driver/CustomMsg]` → package messages not built: `find ~/catkin_ws/devel -name "*CustomMsg*"`; empty result = rebuild (`catkin_make` in catkin_ws).
- Hand-reconstructing wiped files is error-prone (heredoc paste can pick up trailing junk → `junk after document element`). Prefer git restore; manual rebuild only as fallback.
- On Jetson, `dmesg` needs sudo; user in `dialout` group can access serial without sudo.

See `references/fouruav-jetson.md` for the concrete layout of one such machine.

## 不在本技能范围（去对应技能）
- **图传 RTSP / 数传 UDP 14550 / 遥控器看不到画面** → `uav-video-telemetry-streaming`
  （含 D435→RTSP 可跑管线、AirCore 6000 私有协议诊断、mavlink-router 配置）
- **从 Windows 用 plink/pscp 无人值守运维机载电脑** → 该技能的
  `references/windows-plink-remote-ops.md`
