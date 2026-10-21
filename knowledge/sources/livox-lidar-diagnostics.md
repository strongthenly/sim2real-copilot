---
name: livox-lidar-diagnostics
description: Diagnose why a Livox LiDAR (MID360 / HAP / Avia) produces no point cloud on a ROS companion computer (Jetson NX, etc.). Covers network-level health checks, SDK broadcast discovery handshake, the dev_type-mismatch silent drop, config format fixes (old object vs new array+host_ip+lidar_ip), and IP/gateway conflicts. Use whenever livox_ros_driver2 initializes ("Init livox lidars succ") but never publishes /livox/lidar.
---

# Livox LiDAR connectivity diagnostics (ROS companion computer)

## Goal: fast verdict first
Users typically want "is the radar alive" answered quickly, THEN point cloud. Give the hardware verdict first (it's often the quick win), then debug the SDK side. Radar hardware is usually fine — the broken part is host-side config.

## 1. Confirm radar is alive (network level)
```bash
ping -c 3 <radar_ip>                       # ttl=255 + real ms latency = embedded device, not a PC
ip neigh show dev <iface>                  # which MAC answered
curl -s https://api.macvendors.com/<mac>   # "DJI BAIWANG" (大疆百旺) = Livox confirmed
```
Radar broadcasts device info every second to `255.255.255.255:56201` (430-byte UDP). Listen with a python socket (bind `('',56201)`, no sudo needed) and regex ASCII strings out: you get SN, `DevType:Mid-360`, `FmVer`, `BuildTime` — instant identity proof.

## 2. SDK discovery handshake
SDK broadcasts 24B on UDP 56000; radar replies 48B within ~1ms. If you see this exchange in tcpdump (`udp port 56000`), the radar is fully functional — the problem is SDK-side device matching. Note: 48B response is only sent AFTER a 24B trigger; the radar's periodic 430B push on 56201 is independent.

## 3. The silent drop: dev_type mismatch
`device_manager.cpp` drops a discovered device when `type_lidars_cfg_map_.find(detection_data->dev_type) == end()`. Symptom: logs show "Init livox lidars succ", no livox topics ever appear, no error message. This is the #1 cause of "SDK sees radar but never connects".
- Radar firmware may report `dev_type=0x23 (35)` while SDK enum has `kLivoxLidarTypeMid360 = 9` (older SDK vs newer firmware).
- Parse the 48B DetectionData reply to read dev_type: after the 24B SdkPacket header — data[0]=ret_code, data[1]=dev_type, data[2..17]=SN, data[18..21]=lidar_ip, data[22..23]=cmd_port.

## 4. Fix: switch config to NEW format (array + lidar_ip)
Old object format (`"host_net_info": {"cmd_data_ip": ...}`) keys the device map by dev_type → fails on mismatch. The NEW array format registers the radar into `custom_lidars_cfg_map_` keyed by **IP**, bypassing dev_type matching entirely:
```json
"MID360": {
  "lidar_net_info": { "cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500 },
  "host_net_info": [
    { "host_ip": "<host_ip>", "lidar_ip": ["<radar_ip>"], "cmd_data_port": 56101, "push_msg_port": 56201, "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501 }
  ]
},
"lidar_configs": [ { "ip": "<radar_ip>", "pcl_data_type": 1, "pattern_mode": 0, "extrinsic_parameter": {...} } ]
```
Success signal in driver log: `Lidar ip:<radar_ip> ... unicast is enabled` (was `Device type:9 ...` before). Keep `lidar_configs` for the ROS layer.

## 5. IP / gateway conflicts
- **Host must NOT hold the radar's own IP.** If radar = <NET_LAN2>.5, host uses a different IP in the same subnet. (Pinging the radar's IP from a host that also owns that IP gives ttl=64/0.08ms — local loopback, NOT the radar. Radar replies show ttl=255.)
- Radar may be configured with a **cross-subnet host + gateway** (e.g. radar <NET_LAN2>.5, gw <NET_LAN2>.1, host <NET_LIDAR>.5). The host NIC must carry BOTH the gateway IP and the host IP:
```bash
sudo ip addr add <NET_LAN2>.1/24 dev eth1
sudo ip addr add <NET_LIDAR>.5/24 dev eth1
```
Radar's configured host IPs are readable from its broadcast TLV keys 0x0006 (point-data host IP:port) / 0x0007 (IMU host IP:port).

## 6. ROS1 driver ignores broadcast-code args
livox_ros_driver2 **ROS1** build reads only `user_config_path`; `bd_list` / `cmdline_str` args are parsed by the **ROS2** build only (`cmdline_input_bd_code`). Don't burn time passing broadcast codes to ROS1. (Livox broadcast code == device SN, e.g. `ARMCP540030617`.)

## 7. After the IP-match fix: `Send command failed ... command_id:257`
`kCommandIDLidarGetInternalInfo (0x0101)` failing with `get cmd handler failed` = SDK's `CreateCommandHandler` doesn't know the firmware's dev_type (still 35). SDK is too old for the firmware → upgrade Livox-SDK2, rebuild, reinstall to /usr/local, relink the driver.

## 7b. Compile-time version mismatch: DoubleEcho symbols not declared (new-machine deployment)
```
pub_handler.cpp:329: error: 'kLivoxLidarDoubleEchoData' was not declared in this scope
pub_handler.cpp:463: error: 'LivoxLidarDoubleEchoRawPoint' was not declared in this scope
```
Root cause: **driver source is NEWER than the installed SDK headers**. New livox_ros_driver2 commits (`Support Avia2 Lidar`, `support Mid-360s`) added double-echo (DoubleEcho) handling in pub_handler.cpp, but the SDK headers installed at `/usr/local/include/livox_lidar_api.h` / `livox_lidar_def.h` are old (no `DoubleEcho` types). This is **NOT** the ROS1/ROS2 `-DROS_EDITION` trap — cmake logs show "Defining ROS1" fine; the missing definitions are in the SDK headers.
Diagnosis:
```bash
grep -c "DoubleEcho" /usr/local/include/livox_lidar_api.h /usr/local/include/livox_lidar_def.h   # 0 = old SDK
grep -rln "kLivoxLidarDoubleEchoData" ~/catkin_ws/src/livox_ros_driver2/                          # only pub_handler.cpp references it
```
Fix (2 options): A. upgrade/reinstall Livox-SDK2 (rebuild + `make install` to /usr/local) to match driver source; B. revert pub_handler.cpp to the pre-DoubleEcho version (ask whether MID-360S needs double-echo). Check `git log --oneline -3` in livox_ros_driver2 — recent Avia2/Mid-360s commits ⇒ new source that needs the matching SDK.

## Pitfalls
- "SDK init success" ≠ device connected. `Init livox lidars succ` only means config parsed and sockets opened.
- ping-OK ≠ data flow. Ping proves L2/L3 only; SDK still needs dev_type or IP registration to accept the device.
- Multiple IPs on one NIC (e.g. 144.14 / 2.2 / 1.5 / 2.1 on eth1) are normal in this drone setup — don't remove them while debugging.
- sudo over SSH: `echo PASS | sudo -S cmd` (see windows-ssh-automation skill).
- `rostopic hz` errors like `Cannot load message class for [livox_ros_driver/CustomMsg]` are env-var issues (forgot `source devel/setup.bash`) — verify `PYTHONPATH` contains `devel/lib/python3/dist-packages` before suspecting the message build.

## Reference
- `references/livox-sdk-protocol-notes.md` — SdkPacket/DetectionData byte layouts, port table, key constants, config format comparison, ready-to-run python probe snippets.
