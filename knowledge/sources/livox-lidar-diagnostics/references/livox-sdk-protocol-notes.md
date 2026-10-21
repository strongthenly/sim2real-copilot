# Livox SDK2 protocol & config notes (from real debugging session)

## Port table
| port | purpose |
|---|---|
| 56000 | detection (broadcast discover) |
| 56100 | cmd data |
| 56200 | push msg (device info) |
| 56300 | point cloud data |
| 56400 | IMU data |
| 56500 | log data |

Host side usually binds the +1 ports (56101, 56201, ...) as seen in config files.

## Detection handshake
- SDK sends 24B broadcast to `255.255.255.255:56000`
- Radar replies 48B from its IP to broadcast:56000 within ~1ms
- Radar also sends a periodic 430B push to `255.255.255.255:56201` every second (device info, independent of SDK)
- 0x88f7 ethertype frames = Livox proprietary link-layer heartbeat

## SdkPacket layout (`#pragma pack(1)`)
| offset | size | field |
|---|---|---|
| 0 | 1 | sof = 0xAA |
| 1 | 1 | version (kSdkVer = 0) |
| 2 | 2 | length (uint16 LE, includes wrapper) |
| 4 | 4 | seq_num |
| 8 | 2 | cmd_id |
| 10 | 1 | cmd_type |
| 11 | 1 | sender_type |
| 12 | 6 | rsvd |
| 18 | 2 | crc16_h (CCITT over first 18 bytes) |
| 20 | 4 | crc32_d (over data) |
| 24 | n | data |

`GetPacketWrapperLen() = sizeof(SdkPacket) - 1 = 24`; `data_len = length - 24`.

## DetectionData (in data, 24 bytes)
| offset | size | field |
|---|---|---|
| 0 | 1 | ret_code (0 = success) |
| 1 | 1 | dev_type |
| 2 | 16 | sn |
| 18 | 4 | lidar_ip |
| 22 | 2 | cmd_port |

## Key constants
- `kCommandIDLidarSearch = 0x0000`
- `kCommandIDLidarWorkModeControl = 0x0100`
- `kCommandIDLidarGetInternalInfo = 0x0101` (the 257 that failed)
- `kCommandIDLidarPushMsg = 0x0102`
- `kDetectionPort = 56000`
- Device types (SDK 1.2.5): `kLivoxLidarTypeMid360 = 9`, `kLivoxLidarTypeHAP = 15`, `kLivoxLidarTypeIndustrialHAP = 10`
- **Real MID360 firmware (FmVer 35010108, BuildTime 2025/06/09) reported dev_type = 0x23 (35)** — not in SDK enum → silent drop

## 430B device-info broadcast (56201) — ASCII fields found
- `SN: ARMCP540030617` (also the broadcast code)
- `DevType:Mid-360 FmType:App FmVer:35010108 BuildTime:2025/06/09`
- TLV keys: 0x0004 = lidar IP cfg (ip/mask/gw), 0x0005 = state-info host, 0x0006 = point-data host IP:port, 0x0007 = IMU host IP:port

## Config formats
Old (object) — keys device map by dev_type:
```json
"host_net_info": { "cmd_data_ip": "<NET_LIDAR>.5", ... }
```
New (array) — keys by IP, bypasses dev_type check:
```json
"host_net_info": [ { "host_ip": "<NET_LIDAR>.5", "lidar_ip": ["<NET_LAN2>.5"], ... } ]
```
Routing in `ParseLidarCfg`: `IsArray()` → `ParseNewLidarCfg` (fills `custom_lidars_cfg_ptr` → `custom_lidars_cfg_map_` keyed by lidar_ip) vs `IsObject()` → `ParseOldLidarCfg` (fills `lidars_cfg_ptr` → `type_lidars_cfg_map_` keyed by device_type).

## Ready-to-run probe snippets (run on the companion computer, no sudo for python)

```bash
# 1. Radar identity from periodic push (no sudo)
python3 -c "
import socket, time, re
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind(('', 56201)); s.settimeout(6)
try:
    data, addr = s.recvfrom(4096)
    for st in re.findall(rb'[\x20-\x7e]{6,}', data):
        print('field:', st.decode())
except socket.timeout: print('timeout')
s.close()
"
```

```bash
# 2. Parse 48B detection reply (send 24B trigger first)
python3 -c "
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind(('', 56000)); s.settimeout(6)
trig = bytes.fromhex('aa001800a407000000000000000000000000d49900000000')
s.sendto(trig, ('255.255.255.255', 56000))
while True:
    try: data, addr = s.recvfrom(4096)
    except socket.timeout: print('timeout'); break
    if len(data) == 48 and addr[0] != '<NET_LIDAR>.5':  # exclude own broadcasts
        d = data[24:]
        print('ret=%d dev_type=%d SN=%s IP=%s cmd_port=%d' % (
            d[0], d[1], d[2:18].decode(errors='replace').rstrip(chr(0)),
            '.'.join(str(x) for x in d[18:22]), int.from_bytes(d[22:24],'little')))
        break
s.close()
"
```

```bash
# 3. Watch handshake live (needs sudo)
sudo timeout 10 tcpdump -i eth1 -n 'udp port 56000'
```
