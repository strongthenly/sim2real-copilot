# -*- coding: utf-8 -*-
"""
============================================================
 分布式一致性协同控制 demo —— "参考值对单个成员可以是未知的"
============================================================

控制律(每个个体 i, 2D 位置 x_i):

        u_i = - SUM_j a_ij (x_i - x_j)  -  b_i (x_i - r)
              (只跟邻居比状态)            (只有知道参考值的个体才有这项)

  a_ij : 通信拓扑(谁和谁连, 1=连, 0=不连)
  b_i  : 1 = 这个体"知道"参考值 r ; 0 = "不知道"(r 从不出现在它的控制律里)
  r    : 参考值(群要收敛到的目标)

★核心结论: 只要每个连通分量里"至少有 1 个知道 r 的成员", 全群就收敛到 r;
  即使绝大多数个体 b_i = 0(对它们参考值完全未知).
  条件等价于: 通信图存在"以领导者为根的有向生成树".

离散迭代:  x_i(k+1) = x_i(k) + eps * u_i
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

EPS = 0.2           # 步长 (须满足 eps < 2/λmax(L+B))
STEPS = 1000        # 迭代步数
R = np.array([4.0, 3.0])   # 参考值(目标)

# 5 个成员的初始位置(固定, 便于复现)
X0 = np.array([[0.0, 0.0], [1.0, 0.2], [0.4, 1.0], [1.4, 1.3], [0.1, 1.7]])


# ---------------------------------------------------------------- 基础函数
def lap(A):
    """拉普拉斯矩阵 L = D - W"""
    return np.diag(A.sum(axis=1)) - A


def edges_of(A):
    N = A.shape[0]
    return [(i, j) for i in range(N) for j in range(i + 1, N) if A[i, j] > 0]


def components(A):
    """无向连通分量"""
    N = A.shape[0]
    seen = [False] * N
    comps = []
    for s in range(N):
        if seen[s]:
            continue
        stack, comp = [s], []
        while stack:
            u = stack.pop()
            if seen[u]:
                continue
            seen[u] = True
            comp.append(u)
            for v in range(N):
                if A[u, v] > 0 and not seen[v]:
                    stack.append(v)
        comps.append(sorted(comp))
    return comps


def simulate(A, b, r, x0, eps=EPS, steps=STEPS):
    """x(k+1) = x(k) - eps * ( L x + diag(b)(x - r) )"""
    L = lap(A)
    N = A.shape[0]
    X = np.zeros((steps + 1, N, 2))
    X[0] = x0
    bcol = b[:, None]
    for k in range(steps):
        x = X[k]
        X[k + 1] = x - eps * (L @ x + bcol * (x - r))
    return X


def anchor_ok(A, b):
    """每个连通分量里是否都至少有 1 个知道参考值的成员"""
    return all(b[c].max() > 0 for c in components(A))


# ---------------------------------------------------------------- 5 个场景
def chain():
    A = np.zeros((5, 5))
    for i in range(4):
        A[i, i + 1] = A[i + 1, i] = 1
    return A, np.array([1, 0, 0, 0, 0.0])

def ring():
    A = np.zeros((5, 5))
    for i in range(5):
        A[i, (i + 1) % 5] = A[(i + 1) % 5, i] = 1
    return A, np.array([1, 0, 0, 0, 0.0])

def complete():
    return np.ones((5, 5)) - np.eye(5), np.array([1, 0, 0, 0, 0.0])

def split():
    A = np.zeros((5, 5))
    for i, j in [(0, 1), (1, 2), (0, 2), (3, 4)]:
        A[i, j] = A[j, i] = 1
    return A, np.array([1, 0, 0, 0, 0.0])

def no_anchor():
    return np.ones((5, 5)) - np.eye(5), np.zeros(5)


SCEN = [
    ("链式(仅0号知道)", chain()),
    ("环形(仅0号知道)", ring()),
    ("全连(仅0号知道)", complete()),
    ("断开: 右组无人知道", split()),
    ("没人知道参考值", no_anchor()),
]

# ---------------------------------------------------------------- 跑 + 打印
print("=" * 68)
print("参考值 r =", R, "   5 个成员的初始位置:")
print(X0)
print("=" * 68)

results = {}
for title, (A, b) in SCEN:
    X = simulate(A, b, R, X0)
    Lb = lap(A) + np.diag(b)
    eig = np.linalg.eigvalsh((Lb + Lb.T) / 2)
    ok = anchor_ok(A, b)
    final = X[-1]
    err = np.linalg.norm(final - R, axis=1).max()
    results[title] = (A, b, X)
    print(f"\n### 场景: {title}")
    print("   知道 r 的成员 b =", b.astype(int).tolist())
    print("   每个连通分量含锚点? ->", "是 (应收敛到 r)" if ok else "否 (会分裂, 收敛不到 r)")
    print(f"   L+B 最小非零特征值 λ2 = {eig[np.abs(eig)>1e-9].min():.4f}")
    for i, p in enumerate(final):
        print(f"      x{i} = ({p[0]:6.3f},{p[1]:6.3f})   到参考值距离 = {np.linalg.norm(p-R):6.3f}")
    print(f"   >>> 最大偏差 max||x_i - r|| = {err:.4f}  " + ("(≈0, 收敛到参考值)" if err < 0.05 else "(≠0, 未收敛到参考值!)"))

A, b, X = results["链式(仅0号知道)"]
print("\n### 传播延迟(链式, 只有 0 号知道 r): 各成员首次进入 r 的 5% 误差带所用步数")
for i in range(5):
    d = np.linalg.norm(X - R, axis=2)[:, i]
    idx = np.where(d < 0.05 * np.linalg.norm(X0[i] - R))[0]
    print(f"   x{i}(距锚点跳数 {i}) : " + (f"{idx[0]} 步" if len(idx) else "未达到"))

# ---------------------------------------------------------------- 静态对比图
fig, axes = plt.subplots(2, 3, figsize=(15, 9))
colors = plt.cm.tab10(np.linspace(0, 1, 5))
for ax, (title, (A, b)) in zip(axes.ravel(), SCEN):
    X = results[title][2]
    for (i, j) in edges_of(A):
        ax.plot([X0[i, 0], X0[j, 0]], [X0[i, 1], X0[j, 1]], color="0.8", lw=1, zorder=1)
    for i in range(5):
        ax.plot(X[:, i, 0], X[:, i, 1], color=colors[i], lw=1.5, zorder=2)
        ax.scatter(*X0[i], marker="o", facecolor="none", edgecolor=colors[i], s=70, zorder=3)
        ax.scatter(*X[-1, i], color=colors[i], s=40, zorder=4)
        ax.annotate(f"x{i}" + ("★" if b[i] > 0 else ""), X[0, i], fontsize=9, color=colors[i])
    ax.scatter(*R, marker="*", s=320, color="red", zorder=5)
    ax.annotate("参考值 r", R, fontsize=9, color="red")
    ok = anchor_ok(A, b)
    ax.set_title(f"{title}\n" + ("→ 收敛到 r" if ok else "→ 收敛不到 r(分裂)"))
    ax.set_aspect("equal", "box")
    ax.grid(alpha=0.3)

ax = axes.ravel()[5]
for title, (A, b) in SCEN:
    X = results[title][2]
    err = np.linalg.norm(X - R, axis=2).max(axis=1)
    ax.semilogy(err, lw=1.6, label=title)
ax.set_title("全体到参考值的最大偏差随时间")
ax.set_xlabel("迭代步 k"); ax.set_ylabel("max ‖x_i - r‖  (log)")
ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=8)
fig.suptitle("分布式一致性协同控制：参考值只被少数成员知道时的收敛行为", fontsize=14)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig("consensus_compare.png", dpi=130)
print("\n已保存对比图: consensus_compare.png")

# ---------------------------------------------------------------- 动画(链式)
A, b, X = results["链式(仅0号知道)"]
fig2, ax2 = plt.subplots(figsize=(7, 6))
def draw(k):
    ax2.clear()
    x = X[k]
    for (i, j) in edges_of(A):
        ax2.plot([x[i, 0], x[j, 0]], [x[i, 1], x[j, 1]], color="0.75", lw=1.2, zorder=1)
    for i in range(5):
        ax2.plot(X[:k + 1, i, 0], X[:k + 1, i, 1], color=colors[i], lw=1.2, alpha=0.5, zorder=2)
        edge = "red" if b[i] > 0 else colors[i]
        ax2.scatter(*x[i], color=colors[i], s=90, edgecolor=edge,
                    linewidth=3 if b[i] > 0 else 0.5, zorder=4)
        ax2.annotate(f"x{i}" + ("(知道r)" if b[i] > 0 else ""), x[i], fontsize=9)
    ax2.scatter(*R, marker="*", s=420, color="red", zorder=5)
    ax2.annotate("参考值 r", R, fontsize=10, color="red")
    ax2.set_xlim(-0.6, 4.8); ax2.set_ylim(-0.8, 3.8)
    ax2.set_aspect("equal", "box"); ax2.grid(alpha=0.3)
    ax2.set_title(f"链式拓扑, 只有 x0 知道参考值 —— 第 {k} 步\n(红边=知道r的成员; 其余全靠邻居带动)")
ani = FuncAnimation(fig2, draw, frames=range(0, STEPS + 1, 10), interval=80)
ani.save("consensus_anim.gif", writer=PillowWriter(fps=14))
print("已保存动画: consensus_anim.gif")
print("\n完成。")
