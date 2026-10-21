# PX4 v1.11.3 在现代 Ubuntu(24.04 / GCC13) 上的编译复现（AirSim 专用）

> 背景：AirSim 只支持 PX4 **v1.11.x**，但 v1.11.3(2022 时代)配的是 GCC 9/10。
> 在新系统(24.04 默认 GCC 13)上直接编会连挂三处。本页是完整可复现的修复序列。

## 0. 前置
- 单独 clone（别动现有 v1.18）：
  ```bash
  git clone -c http.proxy=http://127.0.0.1:7899 --recursive --branch v1.11.3 \
    https://github.com/PX4/PX4-Autopilot.git ~/PX4-AirSim
  ```
- 装旧编译器（universe 组件，apt 代理端口要指向真实 Clash 端口，本项目 7899）：
  ```bash
  sudo apt install -y gcc-10 g++-10
  ```

## 1. 坑 A：新 glibc 让 `PTHREAD_STACK_MIN` 变 `long` → `math::max` 类型不匹配
报错：
```
platforms/common/px4_work_queue/WorkQueueManager.cpp:257: error: no matching function for call to 'max(long int, int)'
  257 |  const size_t stacksize_adj = math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize));
compilation terminated due to -Wfatal-errors.
```
- 根因：新 glibc 的 `PTHREAD_STACK_MIN` 是 `long`，`PX4_STACK_ADJUSTED(...)` 返回 `size_t`，`math::max(long,size_t)` 无匹配。**任何编译器都拒，不是 GCC 版本问题**。
- ⚠️ 别改成 `std::max(...)`：该文件没 `#include <algorithm>` → 报 `'max' is not a member of 'std'`。
- **正确修法**（同类型即可，保留 PX4 的 `math::max`）：
  ```cpp
  const size_t stacksize_adj = math::max(static_cast<size_t>(PTHREAD_STACK_MIN), static_cast<size_t>(PX4_STACK_ADJUSTED(wq->stacksize)));
  ```
- 改前备份 `WorkQueueManager.cpp.bak`（用户偏好：改前备份 + 贴 diff）。WSL 文件可用 `patch` 工具走 UNC 路径 `\\wsl.localhost\<distro>\home\<user>\...` 直接改。

## 2. 坑 B：老 PX4 用 GCC13 编不过 → 必须用 gcc-10 编
- 第一次失败的 WorkQueueManager 报错在 gcc-13 下出现；换 gcc-10 后该处能过。
- **但 CC/CXX 环境变量对"已 configure 的 build 缓存"无效**：CMake 已把 `CMAKE_CXX_COMPILER=/usr/bin/c++`(gcc-13) 缓存，`CC=gcc-10 make` 仍走 gcc-13。
- **必须先删 build 缓存**，让 CMake 首次 configure 就吃到 gcc-10：
  ```bash
  cd ~/PX4-AirSim && rm -rf build/px4_sitl_default && \
    CC=gcc-10 CXX=g++-10 make px4_sitl_default none_iris
  ```
- 成功标志：CMake 配置阶段打印 `The CXX compiler identification is GNU 10.5.0`。

## 3. 坑 C：apt 拉 gcc-10 时 `Ign:` + 卡 0%
- 现象：`Ign:...gcc-10...` 一串 + `0% [Working]` 卡住。
- 根因：apt 代理端口指错（本项目 Clash 实际是 **7899**，apt 却配了 7890）。检查 `/etc/apt/apt.conf.d/proxy`。
- 修：`sudo sed -i 's/7890/7899/g' /etc/apt/apt.conf.d/proxy && sudo apt update`。

## 4. 验证编译成功
编译完（`make px4_sitl_default none_iris`）会**直接启动** SITL：
```
INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
INFO  [simulator] Simulator connected on TCP port 4560.
INFO  [commander] Ready for takeoff!
pxh>
```
然后 UE(Blocks) Play → AirSim 连上 PX4（PX4 侧出现 `partner IP: 127.0.0.1`）。

> 注：PX4 侧可能出现 `ERROR [mavlink] Unknown param name: COM_OBL_ACT` / `WARN command 520 unsupported`
> —— 是**非致命警告**（参数名/命令在 v1.11.3 里叫法不同），不影响连接与起飞。
