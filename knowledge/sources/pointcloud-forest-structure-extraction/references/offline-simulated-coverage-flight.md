# 离线「模拟补全飞行」：在真实点云上验证覆盖驱动规划（不飞无人机）

> 用途：验证「探索目标 = 物体点云覆盖率」的规划改动（选点准则 / 视点高度 / 停止判据）。
> 合成测试台（`test_core.py`）只能证明"逻辑跑得通"；**真实点云的遮挡与密度才是决定性的**。
> 实测一轮 40×40 m / 120 视点 × 2 种策略 ≈ **3 分钟**，可随改随验。

---

## 一、四个关键设计（缺任一条，结论就不可信）

| # | 设计 | 为什么必须 |
|---|---|---|
| 1 | **固定视点预算**（如 120）+ 报**收敛曲线** | 不固定预算，"谁飞得多"会冒充"谁规划得好" ✗ |
| 2 | **`NEXT_MODE` 模块常量**切换新旧策略 | 保证 A/B 两组跑在**同一套检测 + 同一套度量**上 —— 否则差异可能来自检测 ✗ |
| 3 | **LiDAR 首回波模拟 + 真实 FOV** | 不模拟遮挡与 FOV，等于假装"飞过去就能扫到" ✗ |
| 4 | **体素尺寸与被测系统一致** | 实测 0.10 m 抽稀下"检出 89% 落在真树干上"✓，**0.05 m 下超检 9 倍** ✗（见第五节） |

---

## 二、LiDAR 首回波模拟（核心技巧，物理正确且很快）

不用 ray-casting：**同一个"方向"上只保留最近的那个点，就是首回波**。

```python
def sim_scan(pts, p, RANGE=12.0, CELL_DEG=0.7, FOV_UP=52.0, FOV_DN=-7.0):
    d = pts - np.asarray(p, np.float64)
    dist = np.linalg.norm(d, axis=1)
    m = (dist > 0.3) & (dist <= RANGE)
    if not m.any():
        return np.zeros((0, 3))
    org = np.flatnonzero(m)                       # ★ 保住原始索引，最后要用它取回点
    d, dist = d[m], dist[m]
    az = np.arctan2(d[:, 1], d[:, 0])
    el = np.degrees(np.arcsin(np.clip(d[:, 2] / dist, -1.0, 1.0)))
    fov = (el >= FOV_DN) & (el <= FOV_UP)         # ★ 真实 FOV：MID360 = [-7°, +52°]
    if not fov.any():
        return np.zeros((0, 3))
    org, az, el, dist = org[fov], az[fov], el[fov], dist[fov]
    ia = np.floor((az + math.pi) / (2 * math.pi) * (360.0 / CELL_DEG)).astype(np.int64)
    ie = np.floor((el + 90.0) / CELL_DEG).astype(np.int64)
    key = ia * 1000000 + ie
    o = np.lexsort((dist, key))                   # 同方向内按距离升序
    k_s = key[o]
    first = np.ones(k_s.size, bool)
    first[1:] = k_s[1:] != k_s[:-1]               # 每个方向的第一条 = 首回波
    return pts[org[o][first]]
```

**⭐ FOV 必须来自实测配置**（本机 `settings.json`：`VerticalFOVUpper: 52 / VerticalFOVLower: -7`）。
带上它，模拟才会**自动复现"站在 AGL 1.5 m 看不到树干下部"**这一真实盲区 ✓

---

## 三、闭环主循环

```python
for i in range(BUDGET):
    if i % DETECT_EVERY == 0:          # ① detect() 很贵（本例 5 s）—— 不必每个视点都重跑
        core.detect(); core.build_occ()
    g = core.next_goal(pos, last)      # ② 选点（策略由 NEXT_MODE 决定）
    if g is None:
        break                          # ③ 收敛：core.stop_reason 给出原因
    t, b, gp = g
    path += hypot(gp - pos)            # ④ 累计路径 —— 与"达标数"一起报，才看得出性价比
    core.add_points(sim_scan(P, gp), None)
    t.tries[b] += 1
    pos = last = gp
```

**A/B 的关键**：`TSC.NEXT_MODE = "gain"` / `"nearest"`，其余一切不动：

```python
# core 里
if NEXT_MODE == "nearest":                       # 旧策略（仅作对照）
    score = -cost; ...
else:                                            # 第3层
    gain  = (1.0 / N_BINS) * (1.0 + GAIN_CLOSE * t.frac())   # 收尾加权
    score = gain / max(d + (0 if t is last_tree else COST_SWITCH), 1e-3)
```

---

## 四、实测结果（40×40 m 真实 LAS，120 视点预算）

### 合成场景（`test_core.py`，无干扰）——第3层有效 ✓

```
改前: iters 28  path 162 m  检测 59  达标 58    ← 检测 59 但只 58 达标
改后: iters 21  path 146 m  检测 58  达标 58    ← 58/58 = 100%
```

### 真实点云 —— **没有迁移过来** ✗

| 版本 | 策略 | 路径 | 检出 | 达标 | 平均覆盖 |
|---|---|---|---|---|---|
| 固定 AGL 1.5 m | nearest | 143 m | 343 | 141 | 0.704 |
| 固定 AGL 1.5 m | gain | 136 m | 339 | **163** | 0.736 |
| **自适应高度** | nearest | 130 m | 327 | 158 | 0.723 |
| **自适应高度** | gain | 149 m | 326 | **149** | 0.724 |

**两个结论**：
1. **自适应高度：覆盖率一点没动（0.723/0.724 vs 0.704/0.736）** ✗
2. **A/B 从 +22 棵翻转成 −9 棵** ✗ ⇒ **此时不能声称"第3层更好"**

---

## 五、自适应飞行高度：怎么算、以及**为什么它没用**

### 实现（物理推导）

```
视点高 zv、水平距 Dh 时，FOV 下沿能看到的最高处 = zv − Dh·tan(|FOV_DN|)
⇒ 要看见目标层 zt： zv ≤ zt + Dh·tan(7°)   （Dh ≈ t.r + VP_DIST ≈ 2.4 m ⇒ 余量 ≈ 0.29 m）
⇒ zv = clip(zt + FOV_MARGIN, zg + AGL_MIN, zg + FLY_AGL)   # 只下调，不超过原默认
★ 最后仍必须做 zv > Z_MAX or zv < Z_MIN 的检查 —— 最大高度限制依旧强制生效
```

需要 **(扇区 × 高度层)** 的二维统计才能知道"缺在哪一层"：

```python
N_ZBIN = 4                               # 树干带 1.0~2.2 m 分 4 层（每层 0.3 m）
self.bins2d = np.zeros((N_BINS, N_ZBIN), int)
# _merge() 里随 bins 一起填（用点的 z 减去树基 gz）
agl = Qz - best.gz
kz  = clip(((agl - TRUNK_LO) / (TRUNK_HI - TRUNK_LO) * N_ZBIN).astype(int), 0, N_ZBIN - 1)
best.bins2d = np.maximum(best.bins2d,
                         np.bincount(b * N_ZBIN + kz, minlength=N_BINS * N_ZBIN).reshape(N_BINS, N_ZBIN))

def low_missing_z(self, b):              # 该扇区最低的未覆盖层 → 视点照它定高
    for k in range(N_ZBIN):
        if self.bins2d[b][k] < BIN_MIN_Z:
            return self.gz + TRUNK_LO + (k + 0.5) * (TRUNK_HI - TRUNK_LO) / N_ZBIN
```

**⭐ `bins2d` 只用来算视点高度，不参与达标判定** —— 达标仍看 `bins >= BIN_MIN`，
这样不会破坏上一节（度量层）的验收基线 ✓

### ⚠️ 但实测它**完全没效果** ✗ —— 因为瓶颈不在这里

```
压低视点 → 确实能看到更低处 ✓
但这些回波点：① 被算到"邻近的过检假树"上  ② 径向偏出 ±0.06 m 圆环就不计入
⇒ 覆盖率纹丝不动 ✗
```

**保留这个改动** ✓（物理上正确、无害，真树比例上去后会发挥作用），
**但不要把它当成提升覆盖率的办法** ✗。

---

## 六、真实点云上暴露的两个真正的瓶颈

**① 过检 9 倍（密度相关）**
```
同一区域：真值 37 棵 / 检出 326 棵（0.05 m 抽稀）；离真值 ≤1 m 的只有 30 棵
⇒ 覆盖率分子被摊到 326 个假树上，谁都到不了 BIN_MIN
⚠️ 同一份代码在 0.10 m 抽稀下曾是"89% 落在真树干上" ✓ ⇒ 结论随密度变，必须同尺度复验
```
修法方向：在 RANSAC 内点判据上加**内点线密度**（`内点数 / (2πr × zspan) >= DENS_MIN`）
或**角度最大间隙 `<= GAP_MAX`**（真实树干弧段连续，枝叶小环必有大空隙）。

**② 覆盖率口径（分子）偏窄**：`bins` 只统计 RANSAC 内点（±0.06 m 薄环）。

---

## 七、给用户的汇报口径（本次被反复追问，务必遵守）

- **每处改动都要能归层**（①感知 / ②度量 / ③规划），并直接回答"对我的目标有什么用"
- **不要因为"合成场景 100%"就宣布成功** —— 真实点云上的结论才作数
- **A/B 翻转时必须当场说明"结论不成立"**，不要替符号找解释 ——
  诚实说"指标不可靠、先修指标"，比包装一个假结论强得多
