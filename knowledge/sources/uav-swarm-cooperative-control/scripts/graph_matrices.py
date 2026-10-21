# -*- coding: utf-8 -*-
"""交互拓扑: 加权邻接矩阵 W / 入度矩阵 D / 拉普拉斯 L = D - W  的构造与作用"""
import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=120)

# 约定: w_ij = 边 (j -> i) 的权重, 即 "i 能收到 j 的信息"
def build(W, name, simulate=False, x0=None):
    N = W.shape[0]
    D = np.diag(W.sum(axis=1))     # 入度矩阵 = W 的"行和"放到对角
    L = D - W                      # 拉普拉斯
    print("="*66)
    print(name)
    print("W 加权邻接 (w_ij = j->i 的边权):\n", W, sep="")
    print("D 入度矩阵 = diag(W行和):\n", D, sep="")
    print("L = D - W:\n", L, sep="")
    print("检查 L 行和 L@1 =", L.sum(axis=1), "(应全 0)")
    ev = np.sort(np.linalg.eigvals(L).real)
    print("L 特征值(实部排序):", np.round(ev,4),
          " 0特征值重数 =", int(np.sum(np.abs(ev)<1e-9)))
    nz = ev[np.abs(ev) > 1e-9]
    if nz.size: print(f" λ2(代数连通度) = {nz.min():.4f}")
    # 左零特征向量 xi (xi^T L = 0) —— 决定 x'=-Lx 的最终共识值
    w, v = np.linalg.eig(L.T)
    xi = v[:, np.argmin(np.abs(w))].real
    xi = xi / xi.sum()
    print("左零特征向量 xi (xi^T L=0, 归一化 ξ^T 1=1):", np.round(xi,4),
          "\n  -> 最终共识值 = ξ^T x(0)  (非负分量集中在'信息源/根')")
    if simulate:
        x = x0.copy(); X=[x.copy()]
        for _ in range(4000):
            x = x - 0.05*(L@x); X.append(x.copy())
        print(" 仿真 x'=-Lx 末态:", np.round(X[-1],4),
              " 与 ξ^T x(0) =", round(float(xi@x0),4), "一致")
    return W,D,L

# 例A: 无向加权图 (1--2 权2, 2--3 权1)
build(np.array([[0,2,0],[2,0,1],[0,1,0]],float),
      "例A: 无向加权图  1--2(2)  2--3(1)")

# 例B: 有向图  1->2(1), 1->3(1), 2->4(1.5), 3->4(2)  (节点1是根)
WB = np.zeros((4,4))
for j,i,w in [(0,1,1.0),(0,2,1.0),(1,3,1.5),(2,3,2.0)]:  # 边 j->i
    WB[i,j] = w
build(WB, "例B: 有向图  1->2(1) 1->3(1) 2->4(1.5) 3->4(2), 节点1为根",
      simulate=True, x0=np.array([7.0, 1.0, 2.0, 5.0]))
