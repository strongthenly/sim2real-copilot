---
id: PLATFORM-fouruav
status: 已验证
revalidate_on: [任何硬件更换, 系统重装, 固件升级]
---

# 平台事实：fouruav

> 来源：`knowledge/00_handbook.md` §1。修改硬件后必须同步更新此文件，并把受影响的 gap-ledger 条目标为"待复验"。

| 项 | 值 |
|---|---|
| 机载电脑 | Jetson Orin NX |
| 系统 | Ubuntu 20.04 + ROS1 Noetic |
| 主雷达 | Livox MID-360S（水平 360°，垂直 -7°~+52°） |
| 相机 | Intel RealSense D435 |
| 定位 | FAST-LIVO2（唯一定位源） |
| 飞控 | PX4，经 MAVROS 串口 |
| 规划器 | SUPER（避障/航点）、FUEL（前沿探索） |
| 地面站 | QGC，数传 UDP 14550 |
| lan1 | `<NET_RC>.14/24`（遥控器图传目标） |
| lan2 | `<NET_LAN2>.5/24`（雷达网段） |

## 待补充（建议优先补齐，sim-to-real 建模需要）
- [ ] 起飞质量、轴距、惯量
- [ ] 电机 / 桨叶型号与推力曲线
- [ ] 电池型号与放电曲线
- [ ] PX4 固件确切版本与基线参数导出
