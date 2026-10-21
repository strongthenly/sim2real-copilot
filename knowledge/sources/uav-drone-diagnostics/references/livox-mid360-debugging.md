# Livox MID360 debugging — port map, TLV decode, IP-conflict case study

## Ports (from MID360_config.json)
- lidar_net_info (radar side): cmd 56100, push_msg 56200, point_data 56300, imu 56400, log 56500
- host_net_info (Jetson side): same numbers +1 (56101, 56201, 56301, 56401, 56501)
- Discovery handshake: SDK broadcasts 24 B from host:56000 → 255.255.255.255:56000; radar replies 48 B within ~1 ms.
- Radar device-info broadcast: 56200 → 255.255.255.255:56201, 1 Hz, 430 B. Bind a python UDP socket to '' :56201 to capture.

## 430-byte device broadcast (TLV decode, hexdump from session)
```
0000: aa 00 ae 01 e6 9e 00 00 02 01 00 01 00 00 00 00
0010: 00 00 64 27 86 ef fb f9 1f 00 00 00 00 00 01 00
0020: 01 01 00 01 00 00 04 00 0c 00 c0 a8 02 05 ff ff
0030: ff 00 c0 a8 02 01 05 00 08 00 ff ff ff ff 89 db
0040: 00 00 06 00 08 00 c0 a8 01 05 ed db 00 00 07 00
0050: 08 00 c0 a8 01 05 51 dc 00 00 ...
00c0: ... 41 52 4d 43 50 35 34 30 30 33 30 36 31 37 00 00  → "ARMCP540030617" (SN)
00d0: ... 44 65 76 54 79 70 65 3a 4d 69 64 2d 33 36 30 ... → "DevType:Mid-360 FmType:App FmVer:35010108 BuildTime:2025/06/09"
0140: 05 80 06 00 9c 5a 8a 46 db 41  → type 0x05, MAC 9c:5a:8a:46:db:41
```
Key TLV fields:
- type 0x04: lidar IP config → `c0 a8 02 05 ff ff ff 00 c0 a8 02 01` = IP <NET_LAN2>.5 / 255.255.255.0 / gw <NET_LAN2>.1
- type 0x06 and 0x07: **configured host (upper-computer) IPs** → `c0 a8 01 05` = <NET_LIDAR>.5 ← if the radar expects a host IP the Jetson doesn't hold, no data flows
- SN string and DevType string are plain ASCII in the payload; MAC OUI 9c:5a:8a = DJI BAIWANG (Livox)

## Case study: no point cloud despite "Init livox lidars succ"
Symptom chain (session 2026-08-01):
1. Config said lidar IP <NET_LAN2>.196; `ping <NET_LAN2>.196` unreachable. User: "radar itself is <NET_LAN2>.5".
2. `ip addr add <NET_LAN2>.5/24 dev eth1` → ping <NET_LAN2>.5 "works" but ttl=64, ~0.08 ms = **self-ping: Jetson had claimed the radar's own IP → IP conflict**.
3. Re-IP host to <NET_LAN2>.2 (`sudo ip addr del <NET_LAN2>.5/24 dev eth1; sudo ip addr add <NET_LAN2>.2/24 dev eth1`) → ping 2.5 now ttl=255, ~1-2 ms = real radar.
4. Fixed MID360_config.json with python3 json: host_net_info 4 fields → <NET_LAN2>.2; lidar_configs[0].ip → <NET_LAN2>.5.
5. Driver runs, "Init livox lidars succ", but NO /livox/lidar topic. Wire capture shows radar alive (1 Hz 430 B broadcast + ARP replies) and discovery handshake OK — but no point data.
6. TLV decode reveals radar's configured host IP = <NET_LIDAR>.5 (legacy from HAP-era config). Fix: `sudo ip addr add <NET_LIDAR>.5/24 dev eth1` + restart driver.

**Rule: "SDK init success" ≠ "device connected".** Topic appears only after full connection. Diagnose with tcpdump/python sockets watching 56000/56200, not driver logs (SDK logs are disabled by default: "Livox lidar logger disable").

## Config files seen in the workspace
- `livox_ros_driver2/config/MID360_config.json` — single MID360 (lidar_type 8), object-form host_net_info (works)
- `task_run_scripts/config/MID360s_config.json` — array-form host_net_info (broken: SDK reports "all livox lidars config is empty") + different lidar IP
- `HAP_config.json` / `mixed_HAP_MID360_config.json` — HAP (type 9) setups on 192.168.1.x — source of the stale <NET_LIDAR>.5 host IP
- `launch/msg_MID360.launch` (package livox_ros_driver2) loads MID360_config.json; `task_run_scripts/launch/msg_MID360s.launch` loads MID360s_config.json
