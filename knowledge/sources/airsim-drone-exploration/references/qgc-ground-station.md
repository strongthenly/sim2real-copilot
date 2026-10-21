# QGC(QGroundControl) 接 PX4 SITL —— 验证/控制/航线任务

> 用途：AirSim 的 `VehicleType:"PX4"` 接上 PX4 SITL 后，用 QGC **最直观地验证"PX4 真的在控制 AirSim 无人机"**（能看到姿态/位置，能 Arm/起飞），并**下发航线 Mission 让无人机自动飞**。

## 1. 装 QGC（Windows，装 E 盘）
- 最新版下载（走代理）：GitHub `mavlink/qgroundcontrol` releases → `QGroundControl-installer-AMD64.exe`
  ```powershell
  curl.exe -L --proxy http://127.0.0.1:7899 `
    https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.4/QGroundControl-installer-AMD64.exe `
    -o E:\QGC-installer.exe   # 约 129MB
  ```
- 运行安装器，路径选 E 盘。

## 2. 连接 PX4 SITL
- 启动顺序：先 WSL 起 PX4 SITL（`make px4_sitl_default none_iris`，等 `pxh>`）→ UE 打开 Blocks 点 Play → 再开 QGC。
- QGC 一般**自动检测**到 PX4（PX4 SITL 默认往 **UDP 14550** 发心跳）。
- 没自动连时：QGC 左上 **应用设置 → Comm Links → 添加 UDP**，端口 **14550**（或 14540）。
- ⚠️ **别插真实 Pixhawk 硬件**，否则 QGC 会去连它而不是 SITL。

## 3. 验证成功的标志
- QGC 左上显示 **`Ready` + `Hold`**（绿）+ 出现飞行器仪表 → 说明 QGC 经 PX4 连上了 AirSim 的无人机。
- 点 **起飞(Takeoff)**（可能先要 **Arm/Slide to arm** 解锁）→ **AirSim(UE) 里无人机由 PX4 控制升空**，QGC 高度从 0 开始涨。
- 需要 home 时：地图**右键 → "Set home here"** 设 home 再起飞。

## 4. 航线任务（.plan 文件，让无人机自动飞）
- QGC 地图显示的位置（如 `Lat 10.88, Lon -173.12`）是**默认 LPE 坐标**，SITL 无所谓。
- 模板文件：`templates/airsim_px4_mission.plan`（起飞 + 4 环绕航点 + RTL）。坐标基于 home（改 `plannedHomePosition` 和航点经纬到你的 home 附近；纬度 0.0001° ≈ 11m）。
- **导入**：QGC 左侧 **Plan 视图** → 菜单 **File → Load**（或右上文件夹图标）选 `.plan` → 地图出现航线。
- **上传执行**：右上 **Upload**（传到 PX4）→ **Arm → Start Mission** → AirSim 里无人机自动按航线飞。
- 航点撞楼就**在 QGC Plan 视图里直接拖**调位置/高度。

### .plan 关键字段（QGC v2 mission）
- `"firmwareType": 12`(PX4)、`"vehicleType": 2`(多旋翼)、`"plannedHomePosition": [lat, lon, alt]`
- 每个 item：`"command"`(MAV_CMD)、`"frame"`(3=GLOBAL_REL_ALT, 2=MISSION)、`"doJumpId"`、`"params": [p1..p4, lat, lon, alt]`
- 常用 MAV_CMD：**16=WAYPOINT**、**22=TAKEOFF**、**21=LAND**、**20=RETURN_TO_LAUNCH**、**19=LOITER(unlimited)**

## 5. 与 Python API / MAVROS 的关系
- QGC 适合**手动验证控制**（看/控/航线）；**算法控制**走两条：MAVROS(ROS2,推荐) 或 airsim Python API。
- ⚠️ **airsim Python API 的坑**：AirSim v1.8.2 的 `PythonClient/airsim` 依赖老 `msgpackrpc`，而它钉死 `tornado<5`，**tornado 4.x 只支持 Python ≤3.9**；在 Python 3.12 上 `pip install airsim` metadata 失败、装本地包也报 `No module named 'backports'`/`tornado.platform.auto`。**出路**：① 用 Python 3.9 venv 跑 airsim；② 优先用 **MAVROS(ROS2)** 控 PX4（更贴合 ROS2 探索栈）。
