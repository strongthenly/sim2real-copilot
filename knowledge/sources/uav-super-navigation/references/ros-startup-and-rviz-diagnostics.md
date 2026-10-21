# ROS 机器人：启动慢 / 「操作员看不到 RViz」 的排查手册

> 场景：Jetson/Orin 机载 ROS1 机器人，操作员通过**地面站串流**看机载屏幕。
> 2026-10-08 在 fouruav 上为这几件事绕了 8 轮才定位，把过程固化成流程。

## 0. 铁律：操作员说"看不到"时，先拿证据，别改配置

"RViz 没有任何显示" 至少有三种含义，处理方式完全不同：
1. **窗口根本没出现**（进程崩了 / 没 spawn / 没有 DISPLAY）
2. **窗口出现了，但被别的窗口盖住**
3. **窗口正常显示，但内容不是我要的那一层**（← 本次真正的情况）

★ 本次教训：我按 (1) 排查了 6 轮（窗口几何 → 配置序列化 → DISPLAY → 日志…），
用户补了一句"**frontier 是爆红的**"才明白 RViz 一直正常，是 (3)。
**正确第一步**：抓一帧操作员实际看到的画面（§3），并问清"是黑屏，还是内容不对"。

## 1. 启动慢 / roslaunch 卡住 —— 先量 `~/.ros/log`

```
症状：每次 roslaunch 启动要几十秒到几分钟，日志末尾停在
        Checking log directory for disk usage. This may take a while.
        Press Ctrl-C to interrupt
      附近可能还有 "Error: Could not open file for writing."
      ★ 这条 Error 是 rosclean 自己写不进去 → 【障眼法】，不是根因
实测：~/.ros/log = 4.9 GB / 941 个历史目录
      两个看似无关的症状其实都是它：
        ① 启动极慢
        ② 排在 launch 最后的 rviz 迟迟不 spawn → 看起来"完全没有显示"
修：  rm -rf ~/.ros/log/* ; mkdir -p ~/.ros/log
```
★ **很多精简镜像里 `rosclean` 根本没装** → 日志没人清 → 一路涨到几个 GB。
   别指望用户记得清，在自己的启动脚本开头加护栏：
```bash
LOGSZ=$(timeout 15 du -sm "$HOME/.ros/log" 2>/dev/null | cut -f1)
if [ "${LOGSZ:-0}" -gt 1024 ]; then
  echo "~/.ros/log 有 ${LOGSZ}MB > 1GB → 清理（否则 roslaunch 卡在磁盘检查）"
  rm -rf "$HOME/.ros/log"/* ; mkdir -p "$HOME/.ros/log"
fi
```
★ 注意区分：`~/.ros/log`（roslaunch 自己的）要清；
   `~/catkin_ws/**/log/`（自己 launch 里 `> xxx.log` 写的）通常很小，别一起删。

## 2. 脚本里的 `set -u` 会让 `source ROS setup.bash` 当场失败
```
/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: 行 3: ROS_DISTRO：未绑定的变量
```
ROS 的 `setup.bash` 内部读 `$ROS_DISTRO`，首次 source 时它还没定义 →
`set -u` 把它变成致命错误 → **脚本第一步就死**（按钮点了没反应，看着像"静默失败"）。
**解：所有要 source ROS 的脚本里都不要写 `set -u`**（`set -e` 同理慎用）。

## 3. 抓"操作员实际看到的画面"（决定性证据）

```bash
# 与地面站串流同一个库 mss，抓整个 X 屏幕
DISPLAY=:0 python3 - <<'PY'
import os; os.environ.setdefault("DISPLAY", ":0")
import mss, mss.tools
with mss.mss() as sct:
    mon = sct.monitors[1]
    img = sct.grab(mon)
    mss.tools.to_png(img.rgb, img.size, output="/tmp/screen.png")
    print(mon)
PY
# 然后 pscp 把 /tmp/screen.png 拉回本地看
```

**⭐⭐ 串流的本质必须搞清**：地面站看到的**不是"某个 rviz 的画面"**，而是
**整个屏幕的截图**。本次实测 `NX_code/rviz_full_stream.py`：
```python
import mss
with mss.mss() as sct:
    monitor_index = 1 if len(sct.monitors) > 1 else 0
    monitor = sct.monitors[monitor_index]
    sct_img = sct.grab(monitor)      # ← 抓整个屏幕
# settings: width/height/jpegQuality/fps（只是缩放）
```
⇒ **屏幕上最上层是什么，操作员就看什么**。
⇒ 多开 rviz 会互相遮挡，**被盖住的那个 = "没有显示"**。
⇒ 想让人看到新窗口，就得**显式给它窗口几何（或全屏）**。

## 4. 读窗口真实几何：用 `-tree`，**不要**用 `-children`

```bash
xwininfo -root -tree | grep -iE 'rviz|RViz'
#  0x2c0000e "explore_fuel.rviz* - RViz": ("rviz" "rviz")  1855x1118+20+90  +65+142
```
- `xwininfo -root -children` 只列**根的直接子窗口** → 会看到一堆 `20x20` / `640x480`
  的**内部小窗**（`"Tool Properties"`、`"rviz"`），非常误导
- `-tree` 才展开到 `"xxx.rviz* - RViz"` 那种**真窗口**，带真实宽高与绝对坐标
★ 本次教训：我拿 `-children` 的输出当成"rviz 主窗口是 20×20"，白查一轮。
   判断窗口是否真的小，只认 `-tree` 里那个 `*.rviz - RViz` 的条目。

## 5. 无显示器下验证 rviz 配置能不能加载（很好用）

```bash
QT_QPA_PLATFORM=offscreen rviz -d /path/x.rviz
```
- 能起来 = 配置可加载；起不来 = 配置真有问题
- ★ 输出里的这些是 **offscreen/无 X 的固有噪音**，**不代表配置坏**：
  - `qt.qpa.xcb: could not connect to display`
  - `OGRE EXCEPTION(3:RenderingAPIException): Invalid parentWindowHandle ... GLXWindow::create`
  - `OpenGL device: NVIDIA Tegra Orin (nvgpu)/integrated`
- **务必拿一个已知能用的配置做对照**跑一遍：两边噪音一致 ⇒ 配置没问题。
★ 这也说明：`rviz` 从 ssh（无 DISPLAY）启动必然核心转储，
  **不能用"ssh 下 rviz 起不来"推断物理机上起不来**。

## 6. `Window Geometry`：只在**确认**不对时才动

rviz 把窗口位置尺寸存在 `.rviz` 的 `Window Geometry:` 段（`X` / `Y` / `Width` / `Height`
+ 一长串 `QMainWindow State` 二进制）。从别的机器/仿真拷来的配置**可能缺这几个键**，
Qt 就会给一个很小的窗口。
★ 但**先量再说**：本次 `exp.rviz` 其实带了 `X:65 Y:24 Width:1855`，我差点白改一节。
   （第 4 节的 `-tree` 才是判据。）
**要迁移几何**：把可用那份的 `Window Geometry:` 段**整段文本替换**过去，
**其余字节一动不动**——不要 `yaml.load` → `yaml.dump` 往返，
那会重排/改类型，rviz 的 Qt 类型化 YAML 读取器容易因此失效。

## 7. 多 rviz 共存 + 一台屏幕

操作员的机上通常**已经开着** SLAM/定位用的 rviz（如 `slam_rviz.rviz`，由
`start_fastlivo2.sh` 里的 `rosrun rviz rviz -d ...` 起，全屏）。
新起的探索 rviz 若不给几何，就被盖住 = "没有显示"。
⇒ 用 §4 确认几何，必要时把新窗口放到 SLAM 窗口没占的区域，或让操作员自己拖。

## 8. `roslaunch` 一旦有节点退出，会把同 launch 的 rviz 一起 `killing on exit`
日志里看到
```
process[rviz-5]: started with pid [...]
[rviz-5] killing on exit
```
⇒ 这是**别的节点先退出了**，rviz 是被连坐的，别去查 rviz。
roslaunch 默认：launch 内任一节点退出 ⇒ 整个 launch 结束。
（`super_to_px4_bridge` 之类会**阻塞等 OFFBOARD**，等不到就退出——
   所以"按了按钮 → 切 OFFBOARD"这个顺序本身就会导致中间态掉 launch，属设计问题不是崩溃。）

## 9. 「窗口在，但 3D 视图全黑」= Fixed Frame 找不到

**这是"黑屏"的第一嫌疑，也是最常见的一个**（区别于 §0 的 (1)(2)(3) 三种"看不到"）：
```
RViz 左侧顶部 Global Options → Fixed Frame = 某个 frame
该 frame 不在 TF 树里（或与其他 frame 不连通）
⇒ RViz 报 \"Fixed Frame [xxx] does not exist\"，3D 视图【全黑】（面板/显示树都还在）
```
**决定性判据：**
```bash
rosrun tf tf_echo <FixedFrame> <另一个已知存在的frame>
#   Could not find a connection between 'A' and 'B' because they are not
#   part of the same tree. Tf has two or more unconnected trees.     ← ★ 树断了
rosrun tf tf_monitor            # ★ 信息量更大：列出每个 frame 的 parent / 发布者 / 延迟
```
`tf_monitor` 能直接告诉你 `Frame world exists with parent map.` 这类**父子关系全貌**，
比逐个 `tf_echo` 快得多 —— 先跑它。

**排查要点：**
- **不同 `.rviz` 用不同 Fixed Frame**：本次机载上 `slam_rviz.rviz` 用 `map`（一直正常 ✓），
  而拷来的 `explore_fuel.rviz` 用 `world`（依赖 `real_flight.launch` 里那条
  `static_transform_publisher ... map world`）⇒ **两份配置的"正常/黑屏"会不一样**，
  别拿一份的表现去推断另一份。
- **RViz 启动早于 TF 就绪时会黑屏**：等静态 TF 起来后，在 Global Options 里
  **把 Fixed Frame 重选一次**（下拉点一下）通常就恢复，不必重启。
- **别把黑屏归因到渲染**：`OGRE`/`GLX` 报错在无显示器场景下是噪音（§5）；
  有显示器的物理机上若黑屏，**先查 Fixed Frame / TF**。
- **⚠️ agent 自己 ssh 起的那个 rviz 必然黑/崩**（无 DISPLAY，§5）⇒
  看到黑屏先问一句：**这个 rviz 进程是谁起的？**
  `ps -eo pid,etime,cmd | grep 'rviz -d'` —— ssh 起的那份结论无效，清掉让操作员在本机重按。
