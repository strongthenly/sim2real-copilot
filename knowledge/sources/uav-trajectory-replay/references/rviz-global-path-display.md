# 路径A RViz 全局路径显示修复链（2026-08-11 实机，最终生效方案）

## 问题：/global_path 话题有数据、FSM 收到、但 RViz 无轨迹线

排查顺序（每步有判据）：
1. `rostopic info /global_path` → 看 **RViz 在不在 Subscribers 列表**。不在 = 显示项没订阅
2. 确认 RViz 加载的是哪份 rviz 配置（`ps -eo pid,lstart,args | grep "rviz -d"`，对比 lstart vs top_down.rviz mtime）
3. `python3 -c "import yaml; d=yaml.safe_load(open('top_down.rviz')); print(len(d['Visualization Manager']['Displays']))"`

## 两层根因

### 第一层：Path 显示项 Color 语法
`Color: 0, 255, 0`（逗号）不是 RViz 标准 → 显示项静默失效。必须分号 `Color: 0; 255; 0`。

### 第二层（真正的病根）：top_down.rviz 是"压缩单行式"非标准格式
- 特征：每行 `- Alpha: 0.5      Cell Size: 1`，字段用多空格挤在一行（老版本 RViz/工具生成）
- python `yaml.safe_load` 解析 `Displays` 数量 = 0（因为顶层键不是标准 `Visualization Manager` 结构，或解析异常）
- **用 python 标准 YAML 多行块混插进这种文件 → RViz 解析错乱、显示项全部失效**（即使语法看着对）
- 之前"新增 Waypoints 显示项"能生效是因为当时用字符串插入恰好插到了可解析位置；后续用 python 改 Color/加 Path 块就破坏了

## 最终生效方案：MarkerArray 代替 Path

**MarkerArray 显示项比 Path 可靠**（Waypoints 同款已被验证稳定）。global_path_publisher.py 额外发布：

```python
from visualization_msgs.msg import Marker, MarkerArray
marker = Marker()
marker.header.frame_id = frame_id
marker.ns = "global_path"
marker.id = 0
marker.type = Marker.LINE_STRIP      # 轨迹线
marker.action = Marker.ADD
marker.scale.x = 0.08                # 线宽 8cm
marker.color = (0.0, 1.0, 0.0, 1.0)  # 绿色
for (x,y,z) in wps:
    pt = Point(); pt.x,pt.y,pt.z = x,y,z
    marker.points.append(pt)
pub_marker = rospy.Publisher("/global_path_marker", MarkerArray, queue_size=1, latch=True)
```

RViz 显示项（标准格式，和 Waypoints 同款）：
```yaml
- Class: rviz/MarkerArray
  Enabled: true
  Name: GlobalPathMkr
  Namespaces:
    global_path: true
  Queue Size: 10
  Topic: /global_path_marker
  Value: true
```

## 正确修改 top_down.rviz 的方法（2026-08-11 会话末修正）

**⚠️ 先分清文件是"可解析压缩格式"还是"完全无法解析"两种情形，方法不同：**

### 情形 A：yaml.safe_load 能解析（Displays 数量 > 0）——用字符串插入，别用 yaml.dump 重建
- **实测（2026-08-11 会话末）：字符串插入 + 保持压缩格式的块 = 成功**（加 Unknown 显示项，插入后 `yaml.safe_load` 验证 16 显示项解析通过，提交 c7ccf9c）
- 做法：找到同类显示项块（如 Occ 的 `Topic: /fsm_node/rog_map/occ` 到 `Value: true` 的完整块），在其后**字符串 replace 插入新块**（缩进对齐、字段名/顺序与 RViz 压缩格式一致：Alpha/Class/Color/Enabled/Name/Topic/Value 等）
- **❌ 别用 yaml.dump 整体重建**：2026-08-11 曾用 yaml.dump 重建（9 显示项），结果**用户熟悉的 SUPER 界面全丢**（Sensing/Frontend/Backend/TrajOpt 分组消失，用户抱怨"没有之前的SUPER界面了"）——必须回退原文件再重新追加。yaml.dump 会把压缩格式展开成多行标准格式，RViz 能读但**显示项分组/颜色细节丢失**
- 插入后**必须验证**：`python3 -c "import yaml; d=yaml.safe_load(open('top_down.rviz')); print(len(d['Visualization Manager']['Displays']))"` 数量正确 + 新显示项字段在

### 情形 B：yaml.safe_load 解析失败 / Displays 数量 = 0（文件确实损坏）
才考虑 yaml.dump **整体重建标准格式**（9 个显示项：Grid/GlobalCloud/LocalCloud/Occ/Waypoints/GlobalPathMkr/Goal/FrontendPath/ExpTraj，Fixed Frame=map），旧文件备份 .bak_oldformat，重建后必须让用户确认界面完整（显示项分组会丢，需重新加）。

## 判读
- 重启 SUPER（按钮2→按钮3）后 `rostopic info /global_path_marker` 的 Subscribers 应出现 rviz
- RViz 里 GlobalPathMkr 勾选可见绿色轨迹线（latch 发布，勾选即显示）
