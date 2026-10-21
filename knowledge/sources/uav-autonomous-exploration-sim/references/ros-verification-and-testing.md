# ROS 工作空间的工程化与验证（实测配方）

来源：FUEL 接入阶段④⑤ 的实操（WSL `ros1-twin`，ROS Noetic，catkin 工作空间 `~/fuel_ws`）。
适用：任何"自写桥 / 脚本 + catkin 包"的 ROS 项目。

---

## 1. ★ Makefile = canonical 验证入口（先建这个）

**问题**：ROS 工作空间 + shell 脚本的项目**没有 canonical test/lint/build 命令**，验证机制每轮都判 unverified。
**错误应对**：在 `C:\...\AppData\Local\Temp` 下建 `hermes-verify-*.sh` 跑验证 —— 该脚本**自身**会被计入 changed path，于是"建脚本→要求验证→脚本又是改动"形成**死循环**（保留同样循环，删除也一样，因为建/删都算改动）。
**正解**：项目根放 `Makefile`，harness 立刻识别并返回 `canonical_command: make lint` 判 PASS。

```makefile
# 用法: make test | make lint   （在 WSL 内执行）
.PHONY: test lint

test:
	bash fuel_bridge/test/smoke_test.sh

# 语法检查（用 ast.parse，不留 __pycache__）
lint:
	python3 -c "import ast,glob;[ast.parse(open(f).read()) for f in glob.glob('fuel_bridge/scripts/*.py')];print('scripts py syntax ok')"
	python3 -c "import xml.etree.ElementTree as E; E.parse('fuel_bridge/launch/fuel_real.launch'); print('launch XML ok')"
```

- **Makefile 必须用 TAB 缩进**。
- `lint` **不要用 `python3 -m py_compile`** —— 它每次生成 `__pycache__/`，跑完工作区就脏，还得再清一次（本轮踩过）。
- 跑法：`wsl -d <distro> -u <user> -- bash -c "cd /mnt/c/.../<proj> && make test"`。

**兜底（项目不适合建 Makefile 时）**：验证脚本经 base64 传入、在 WSL `/tmp` 临时落盘、跑完即删，Windows 侧零新增文件：

```powershell
$s = @'
#!/bin/bash
...验证逻辑...
'@
$b = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($s))
wsl.exe -d ros1-twin -u <USER> -- bash -c "echo $b | base64 -d > /tmp/hermes-verify-x.sh; bash /tmp/hermes-verify-x.sh; rm -f /tmp/hermes-verify-x.sh"
```
这同时绕开 PowerShell 对中文/`$`/`()`/内嵌引号的转义地狱 —— 长 shell 逻辑一律走这条路或写成 `.sh` 文件，**别内联**。

---

## 2. 验证阶梯（成本从低到高，逐级上）

1. **语法**：`bash -n x.sh`；`python3 -c "import ast; ast.parse(open(f).read())"`；`xml.etree.ElementTree.parse(launch)`。
   XML 合法**不等于** launch 合法 —— 见 §3。
2. **launch 真解析**：`roslaunch <pkg> <file> [args] --dry-run`（抓 arg 前向引用等 XML 查不出的错）。
3. **一致性**：`md5sum` 对比 **Windows 备份 ↔ WSL 部署**。`patch` 改的常是 Windows 侧文件、跑的却是 WSL 那份 ⇒ 漏同步会产出"静态检查 FAIL"的**假警报**（本轮踩过）。
4. **行为**：起**独立 master（换端口，如 11351/11352）** + 目标节点，用 Python 直发假消息、订阅输出断言。**独立 master 保证不干扰在跑的链路**。
5. **真实链路**：才轮到 AirSim/PX4 在环。

---

## 3. launch / 参数的坑

- **`<arg>` 必须"先定义后引用"**：`<arg name="init_z" default="$(arg seed_z)"/>` 若写在 `seed_z` 定义之前 → roslaunch 直接报错。修法：把被引用的 arg **上移到文件顶部**。
- **参数路径 = node 名 + param 名叠加**：`<node name="traj_server">` 内写 `<param name="traj_server/init_z">` ⇒ 完整路径 **`/traj_server/traj_server/init_z`**。节点用 `nh("~")` 读 `"traj_server/init_z"` 正好命中；查 `/traj_server/init_z` 会 ERROR "is not set" 而误判参数没传。
- **`rosparam` 值在节点构造时读一次** ⇒ 节点已在跑时改它**无效**，必须停掉重启（脚本要主动提示，别让用户以为改了也没用）。
- **XML 注释里禁止连续双连字符 `--`**：`<!-- ---------- 标题 ---------- -->` 会让 launch 文件**非法**。

---

## 4. catkin 工作空间的版本判读

**`catkin_install_python` 在 devel 空间放的是 relay wrapper，不是副本**：

```
$ head devel/lib/fuel_bridge/fuel_ctrl_bridge.py
#!/usr/bin/python3
# generated from catkin/cmake/template/script.py.in
# creates a relay to a python script source file, acting as that file.
python_script = '/home/<USER>/fuel_ws/src/fuel_bridge/scripts/fuel_ctrl_bridge.py'
with open(python_script, 'r') as fh:
    context = {...}
    exec(compile(fh.read(), python_script, 'exec'), context)
```

⇒ `grep -c "<新函数名>" devel/lib/<pkg>/<script>.py` 会返回 **0**，这是**正常的**，不代表旧版。
**正确判据**：`head` 看是否是 wrapper（含 `generated from catkin`）+ 看 `src` 那份的 `mtime`。
（本轮据此误判"跑的是旧版"，白查一轮；实际是新版，真因是缺上游数据。）

---

## 5. 日志位置（诊断提示别指错文件）

- roslaunch **各节点**的 stdout/stderr → **`~/.ros/log/<uuid>/<node>-N.log`**（`~/.ros/log/latest/` 是指向最新一次运行的软链）。
- `/tmp/<name>.log` 只在启动脚本**显式重定向**时才有（如 `setsid rosnode ... > /tmp/x.log`）。
- ⇒ 让用户 grep 某个节点的打印，一律给 `~/.ros/log/latest/<node>*.log`。给错文件会得出"没生效"的**假结论**。

---

## 6. 写断言的两个通病

- **`grep` 关键字以 `--` 开头要加 `-e`**：`grep -q "--no-save" file` 报 `unrecognized option '--no-save'`，静默变成 FAIL ⇒ `grep -q -e "--no-save"`。同理 `--all` / `--keep-px4`。
- **验证 ROS 转发必须"先订阅、后发布"**：publisher 默认不 latch，`rostopic pub` **之后**才起的 `rostopic echo` 会错过消息 ⇒ 症状像"桥没转发"，实为测试方法错。正确顺序：先后台 `rostopic echo > file` → 再 pub → 再 grep 文件。
- **测试消息别省字段**：`quadrotor_msgs/PositionCommand` 缺 `velocity/acceleration` 会让 `rostopic pub` 构造失败、桥根本没收到 ⇒ 断言全 FAIL，看着像代码 bug。**更稳的做法是直接用 Python 发消息**（`rospy.Publisher`），绕开 `rostopic pub` 的 YAML/引号坑。
- **浮点断言用容差**：四元数/位置往返会有 ~0.01 量级误差，`grep "z: 1.5"` 会因打印成 `1.51` 而 FAIL ⇒ 用 `awk -v a="$x" 'BEGIN{exit !(a>1.4 && a<1.6)}'` 之类。

---

## 7. 桥类测试的顺序约束（重要设计限制）

带"阶段"的桥（如 `seed 悬停 → FUEL 接管`）内部常有一个**一旦置位就永不回退**的状态（例：`last_cmd != None` 即永走 FUEL 分支）。
⇒ 「seed 悬停」「未对齐时拒绝转发」「对齐后按朝向差旋转」**三者不能共用一个桥实例**。

做法：**起两个实例，各配不同参数**：
| 实例 | 参数 | 测什么 |
|---|---|---|
| A | `~align_samples:=1000`（对齐永不完成） | 输入转发 + seed 悬停 + **未对齐时拒绝转发（防撞）** |
| B | `~align_samples:=5` | 朝向对齐 + **按 `Rz(psi)` 旋转目标 + yaw 同步偏移** |

另：断言前**等异步状态生效**（先发 3~5 条指令 + `sleep 1`，再 `clear()` 计数），否则测到的是上一阶段的残留。

---

## 8. 数值对齐的验算方式

坐标变换类改动，**用解析解对拍**最有说服力：给桥喂一对固定的位姿（FUEL 系 yaw=0 / PX4 系 yaw=82.24°），发目标 `(3,0,1.5)`，断言输出 == `(3·cos82.24°, 3·sin82.24°, 1.5) ≈ (0.405, 2.973, 1.5)` 且 yaw == 82.24°。
实测两组参数（1.5 / 2.5）都跟随 ⇒ 证明是**继承关系**而非巧合。
