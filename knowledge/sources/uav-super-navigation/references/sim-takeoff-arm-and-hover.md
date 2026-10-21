# 仿真联调：起飞 / 解锁 / 悬停等待（2026-09-22 实测实录）

配套 SKILL.md 坑 O。适用：`ros1-twin` + AirSim + FAST-LIVO2 + SUPER，`~/super_explore_all.sh` 启动链。

---

## 1. 诊断：飞机到底起没起

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
source /opt/ros/noetic/setup.bash

# ① 飞控状态（最关键的两个字段）
timeout 8 rostopic echo -n1 /mavros/state | grep -E 'connected|armed|guided|mode'
#   connected: True   armed: False   guided: True   mode: "AUTO.LOITER"     ← 没起飞

# ② 实际高度
timeout 8 rostopic echo -n1 /mavros/local_position/pose/pose/position
#   z: -0.045   ← 在地面

# ③ 控制桥日志（它负责 arm + 起飞 + OFFBOARD）
tail -25 /tmp/super_bridge.log
```

### 三种状态的判读表

| `armed` | `mode` | `z` | 含义 | 下一步 |
|---|---|---|---|---|
| False | `AUTO.LOITER` | ≈0 | **没解锁**（本次症状） | 查 `auto_arm` 是否生效 |
| True | `AUTO.LOITER` | ≈0 | 解锁了但没进 OFFBOARD | 查 `auto_offboard` / `commander mode offboard` 被拒 |
| True | `OFFBOARD` | 上升中→稳定 | 正常起飞 | 等悬停稳定后跑算法 |

### setpoint 流检查（OFFBOARD 的前置条件）

```bash
for t in /mavros/setpoint_raw/local /mavros/setpoint_position/local; do
  echo "$t : $(timeout 5 rostopic hz $t 2>/dev/null | grep average | head -1)"
done
```
- 起飞阶段 bridge 走 **`setpoint_position/local`**（实测 ~70Hz）
- 进入 FOLLOW_TRAJ 后才发 **`setpoint_raw/local`**
- 两个都没有 = bridge 没在发，飞机必然不动

### PX4 侧的 OFFBOARD 失联行为（决定"掉线即降落"）

```bash
for p in COM_OBL_ACT COM_OBL_RC_ACT COM_OF_LOSS_T; do
  echo "$p = $(timeout 4 rosrun mavros mavparam get $p 2>/dev/null | tail -1)"
done
# COM_OBL_ACT = 1      → OFFBOARD 丢失后 LAND
# COM_OBL_RC_ACT = 0   → 有 RC 时 Hold
# COM_OF_LOSS_T = 0.5  → setpoint 断流 0.5s 即退出 OFFBOARD
```
> **`COM_OF_LOSS_T = 0.5`** 是所有 `super_cmd_topic` 超时设计（bridge 的 `timeout_threshold: 0.5`）的来源：
> 发点流一停就掉模式，这就是"bridge 必须持续发 setpoint"的原因。

---

## 2. 根因与修法

### 2.1 `super_to_px4_bridge.yaml` 的两个开关

```yaml
auto_arm: false          # ← 默认关
auto_offboard: false     # ← 默认关
init_goal:
  x: 0.0
  y: 0.0
  z: 0.6                 # ← 注意：这份 yaml 里是 0.6，但【没被加载】时用的是代码默认 1.0
```

源码侧（`SUPER/super_planner/Apps/super_to_px4_bridge.cpp`）：
```cpp
private_nh.param<bool>("auto_arm", auto_arm, false);                              // :172
private_nh.param<bool>("auto_offboard", auto_offboard, false);                    // 附近
private_nh.param<std::string>("super_cmd_topic", super_cmd_topic, "/planning/pos_cmd");  // :151 默认值正确
private_nh.param<double>("init_goal/x", init_goal.pose.position.x, 0.0);          // :193
private_nh.param<double>("init_goal/y", init_goal.pose.position.y, 0.0);          // :194
private_nh.param<double>("init_goal/z", init_goal.pose.position.z, 1.0);          // :195 ← 默认 1.0
```
**默认值对仿真友好的部分**：`super_cmd_topic` 默认就是 `/planning/pos_cmd`（正确）、`init_goal` x/y 默认 0。
**默认值有问题的部分**：`auto_arm` / `auto_offboard` 默认 false，`init_goal/z` 默认 1.0（与 yaml 的 0.6 不一致）。

### 2.2 「直接调二进制」丢失 yaml —— 这是根因 2

`real_flight.launch`（实机路径，会加载 yaml）：
```xml
<node pkg="super_planner" type="super_to_px4_bridge" name="super_to_px4_bridge" output="screen">
  <rosparam command="load" file="$(find task_run_scripts)/config/super_to_px4_bridge.yaml"/>
</node>
```

`super_explore_all.sh`（仿真路径，绕过 launch，**不加载 yaml**）：
```bash
setsid "$BRIDGE_BIN" > /tmp/super_bridge.log 2>&1 < /dev/null &
```

**丢失的参数（yaml 里有、代码默认值不同或不存在）**：
`max_velocity_xy: 1.2`、`max_velocity_z: 1.0`、`max_yaw_rate: 1.5`、
`position_control_gain`、`max_position_error`、`use_current_yaw`、`use_yaw_angle_control`、
`use_velocity_ff` / `use_accel_ff` / `velocity_ff_gain`、`scan_after_takeoff_en` 系列、
`land_speed: 0.3` / `land_height: 0.15` / `auto_disarm: true`、
`failsafe_en` / `battery_threshold: 20.0` / `rc_loss_timeout` / `geofence_en`、
`init_goal`。

> **这不是"少一个 arm 开关"的小事** —— 是**整份飞行调参集体回落到源码默认值**，含 failsafe 与降落速度。

### 2.3 落地写法（`super_explore_all.sh` 步骤 4 已改）

```bash
setsid "$FSM_BIN" _config_path:="$SUPER_CFG" > /tmp/super_fsm.log 2>&1 < /dev/null &

# ⚠️ 直接调二进制不会读 launch 里那行 <rosparam command="load">
BRIDGE_YAML=~/super_sim_ws/src/task_run_scripts/config/super_to_px4_bridge.yaml
if [ -f "$BRIDGE_YAML" ]; then
  rosparam load "$BRIDGE_YAML" /super_to_px4_bridge 2>/dev/null \
    && say "     [OK] 已加载 bridge yaml（控制增益/限速/failsafe 生效）" \
    || say "     [!] bridge yaml 加载失败，将用源码默认值"
fi
rosparam set /super_to_px4_bridge/auto_arm true      2>/dev/null
rosparam set /super_to_px4_bridge/auto_offboard true 2>/dev/null
setsid "$BRIDGE_BIN" > /tmp/super_bridge.log 2>&1 < /dev/null &
```

**为什么用 `rosparam load` + `set` 而不是 `_auto_arm:=true` 私有参数**：
私有参数只能覆盖那一个键，其余仍丢失；`rosparam load` 把整份 yaml 装进私有 ns，再 `set` 覆盖少数几项，
是"全都要 + 改两个"的唯一简写法。
（实测过：**不加任何参数时 bridge 日志也显示 `Auto arm: true`** —— 那是参数服务器上的残留值，
所以**必须显式设置，不能依赖残留状态**。）

### 2.4 实测成功日志（三条一起出现才算真的生效）

```
[INFO] ...   Auto arm: true
[INFO] ...   Auto OFFBOARD: true
[INFO] ...   Auto disarm: true
[INFO] ...   Publishing initial setpoints for takeoff to 1.00 meters...
```
> 注意最后一行是 **1.00 meters** —— 印证了 yaml 的 `init_goal.z: 0.6` 没被读到（走的是代码默认 1.0）。

---

## 3. `6.5)` 悬停等待步骤（用户要求：先悬停再跑算法）

位置：步骤 6（仲裁器）之后、步骤 7（链路自检 + 触发探索）之前；仅在 `WITH_SUPER=1` 时执行。

```bash
if [ "$WITH_SUPER" = "1" ]; then
  say "6.5) 等飞机起飞并悬停稳定（bridge 已开 auto_arm/auto_offboard；起飞高度 1.0m）..."
  ok_hover=0; armed=""; mode=""; z=""; zprev=""; stable=0
  for i in $(seq 1 45); do
    st=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null)
    armed=$(echo "$st" | awk '/^armed:/{print $2}')
    mode=$(echo "$st"  | awk '/^mode:/{gsub(/"/,"",$2); print $2}')
    z=$(timeout 4 rostopic echo -n1 /mavros/local_position/pose/pose/position/z 2>/dev/null | tail -1)
    [ $((i % 5)) -eq 0 ] && say "     ... armed=${armed:-?} mode=${mode:-?} z=${z:-?}（已 ${i}s）"
    if [ "$armed" = "True" ] && [ "$mode" = "OFFBOARD" ] && [ -n "$z" ]; then
      if [ -n "$zprev" ]; then
        if awk -v a="$z" -v b="$zprev" 'BEGIN{exit !(a>0.4 && ((a-b<0.02 && b-a<0.02)))}'; then
          stable=$((stable + 1))
        else
          stable=0
        fi
      fi
      zprev="$z"
      [ $stable -ge 3 ] && { ok_hover=1; break; }
    fi
    sleep 1
  done
  if [ "$ok_hover" = "1" ]; then
    say "     [OK] 已起飞并悬停稳定：z=${z}m（armed=$armed mode=$mode，连续 3s 高度波动<2cm）"
    say "         现在才触发探索算法"
  else
    say "     [!] 45s 内未等到悬停稳定（armed=${armed:-?} mode=${mode:-?} z=${z:-?}）"
    say "         查 /tmp/super_bridge.log。常见原因："
    say "           - 未解锁：PX4 预检没过（传感器/EKF 未就绪）→ 看 pxh 里的 'Preflight Fail'"
    say "           - 未进 OFFBOARD：auto_offboard 被拒 → pxh 里执行 commander mode offboard"
  fi
fi
```

### 判据设计要点

| 要点 | 原因 |
|---|---|
| **`a > 0.4`（离地）而非 `≈ 目标高度`** | 目标高度可能来自代码默认（1.0）而非 yaml（0.6），写死必失效 |
| **`\|Δz\| < 0.02` 连续 3 次** | 用"相邻差值"衡量稳定，不依赖任何绝对目标值 |
| **`armed == True` && `mode == OFFBOARD`** | 少任一项都说明起飞链没走完 |
| **超时后仍继续（只警告）** | 联调时算法链路本身也要验证；但必须明确告知"飞机可能没起" |

### 边界测试（必须用脚本里逐字相同的 awk 表达式）

```python
AWK = 'BEGIN{exit !(a>0.4 && ((a-b<0.02 && b-a<0.02)))}'
cases = [(0.50, 0.51, True),   # 离地 + 变化 1cm   -> stable
         (0.50, 0.60, False),  # 变化 10cm         -> unstable
         (0.20, 0.21, False),  # 离地不足 0.4m     -> unstable（正确地排除）
         (1.00, 0.995, True)]  # 悬停高度微降       -> stable
for prev, now, want in cases:
    rc = subprocess.run(['awk','-v','a=%s'%now,'-v','b=%s'%prev, AWK]).returncode
    stable = (rc == 0)          # ★ rc==0 才是"判据成立"
    assert stable == want
```
> **踩过的坑**：写成 `got = 0 if rc == 0 else 1` 再用 `if got` 判 stable → **语义反了，4/4 全报 WRONG**，
> 一度误判"脚本判据有 bug"。`awk 'BEGIN{exit !(cond)}'` 的 **退出码 0 = cond 成立**。

---

## 4. 排查顺序速查（下次照这个走）

```
飞机没起飞
├─ 1) rostopic echo -n1 /mavros/state   → armed / mode 是什么？
├─ 2) tail -25 /tmp/super_bridge.log    → 停在 "Waiting for OFFBOARD ... manual switch required" ?
│                                         ⇒ auto_offboard 没开（原因 1）
├─ 3) bridge 日志里 Auto arm 是哪来的？  → 不显式设置就可能只是参数残留
├─ 4) bridge 日志有没有 "Publishing initial setpoints for takeoff to X meters"？
│                                         ⇒ X 不是 yaml 里的值 = yaml 没被加载（原因 2）
└─ 5) 脚本里有没有 rosparam load 那份 yaml？
```
