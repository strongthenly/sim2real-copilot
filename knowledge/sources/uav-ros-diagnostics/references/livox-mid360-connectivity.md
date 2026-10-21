# Livox MID360 雷达连接细节（本会话实测）

## 设备身份
- 厂商 OUI `9c:5a:8a` = **DJI BAIWANG（大疆百旺）**，即 Livox
- MID360 的 SDK 枚举：`kLivoxLidarTypeMid360 = 9`（驱动日志 "Device type:9" 就是 MID360，正常）
- 广播码（broadcast code）**默认 = 设备 SN**（如 ARMCP540030617）
- 固件日志格式：`SN: xxx / DevType:Mid-360 / FmType:App / FmVer:35010108 / BuildTime:2025/06/09`

## 端口约定（来自 config JSON）
| 用途 | 雷达端口 | 主机端口 |
|---|---|---|
| cmd_data | 56100 | 56101 |
| push_msg | 56200 | 56201 |
| point_data | 56300 | 56301 |
| imu_data | 56400 | 56401 |
| log_data | 56500 | 56501 |

## 广播协议（抓包实测）
- 雷达每秒向 `255.255.255.255:56201` 发 430 字节 push_msg，含设备信息 + **上位机 IP 配置 TLV**
- SDK 向 `255.255.255.255:56000` 发 24 字节广播，雷达 1ms 内回 48 字节 → 发现握手成功 ≠ 连接成功
- 广播包 TLV 解析关键：`04` = 雷达自身 IP/掩码/网关；`06`/`07` = 点云/IMU 上位机 IP+端口（`c0 a8 01 05` = <NET_LIDAR>.5）

## 连接失败的常见根因（按优先级）
1. **主机 IP 与雷达配置的上位机 IP 不匹配**：雷达只推数据给配置里记录的主机 IP。广播包 TLV 06/07 能读出该 IP，主机网卡必须加上
2. **IP 冲突**：主机把雷达 IP 占了（如都配 <NET_LAN2>.5）→ ping 通但 ttl=64 是主机自己
3. **跨网段缺网关**：雷达 <NET_LAN2>.5 + 上位机 <NET_LIDAR>.5，雷达需经网关 <NET_LAN2>.1 转发；主机网卡要补 `ip addr add <NET_LAN2>.1/24 dev ethX`，否则雷达持续发 "who-has <NET_LAN2>.1" ARP 无应答
4. **ROS1 驱动不读 bd_list**：ROS1 版 main() 只 getParam `user_config_path` 等，无 `cmdline_input_bd_code`（ROS2 版才有）。传 `bd_list:=SN` 无效，SDK 走 auto-connect
5. **config JSON 格式**：SDK2 支持两种 host_net_info 格式：
   - 对象：`{"cmd_data_ip": "<NET_LIDAR>.5", ...}`（ROS 驱动用这种）
   - 数组：`[{"host_ip": "<NET_LIDAR>.5", "multicast_ip": "224.1.1.5", ...}]`（官方 quick_start 用）
   - 数组格式给驱动会报 `Params check failed, all livox lidars config is empty`

## 验证工具
- 官方 SDK 测试：`~/SDK/Livox-SDK2/build/samples/livox_lidar_quick_start/livox_lidar_quick_start <config.json>`（需传配置路径，否则 "Params Invalid"）
- 官方配置示例：`~/SDK/Livox-SDK2/samples/livox_lidar_quick_start/{config,mid360_config,hap_config}.json`
- 点云判据：`rostopic hz /livox/lidar` → 约 10Hz
- 驱动日志：`ls -t ~/.ros/log/*/livox_lidar_publisher2*.log | head -1 | xargs tail -30`

## 本会话修复后的配置（可作模板）
eth1 上四个 IP 共存：<NET_RC>.14（地面站）、<NET_LAN2>.2（SDK 主机）、<NET_LIDAR>.5（雷达期望上位机）、<NET_LAN2>.1（雷达网关）。config 的 host_net_info 全填 <NET_LIDAR>.5，lidar_configs[0].ip 填 <NET_LAN2>.5。
注意：`ip addr add` 是临时的，重启会丢；要持久化需写 netplan/rc.local。
