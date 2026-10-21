# -*- coding: utf-8 -*-
"""
评估器【合成校准】骨架 —— 换用真实真值之前，先用【答案已知】的合成数据把评估器考试一遍。

为什么必须做（用户亲授纪律）：
    评估器是纯代码，真值是数据。两者一起上线时出了错**分不清是谁的错**。
    先让评估器在答案已知的场景上过关 ⇒ 把"评估器 bug"这个变量彻底消掉
    ⇒ 之后换成真实真值，出了问题就只可能在真值本身，归因干净。

用法：复制本文件 → 把你的评估函数与合成夹具填进 TODO 处 → 运行。
     exit code 非零即失败（可直接进 CI / canary）。

★ 判据：合成校准的 FAIL 有 ~90% 是"你的期望值算错了"，不是"评估器写错了"。
  断言失败时**先重算期望值**，再怀疑评估器。
"""
import sys
import numpy as np

# TODO 1: 导入你的评估器（纯函数最好；分子/分母必须走同一个函数）
# import eval_metrics as EM

PASS, FAIL = "PASS", "FAIL"
_rows = []


def check(tag, got, want, tol=1e-9):
    """记录一条断言。float 用容差比，其他用 ==。"""
    ok = (abs(got - want) <= tol) if isinstance(want, float) else (got == want)
    _rows.append((PASS if ok else FAIL, tag, got, want))
    return ok


# =====================================================================
# TODO 2: 合成夹具 —— 位置/半径/"每个单元能不能被测到"全部由你写死，
#          这样期望指标可以【手算】出来。
#   ⚠️ 坑1：夹具几何要避开"意外对齐"。
#      实例：合成目标沿 y 轴间隔正好 5 m，检测整体沿 y 偏移 5.0 m
#            ⇒ 每个检测偏移后正好落到隔壁目标上 ⇒ 匹配数 4（期望 0）—— 夹具错了，不是评估器错。
#      做法：偏移量与间距用**互不成比例**的数（偏移 2.0 m 而间距 5 m），或做成非网格排布。
# =====================================================================
UNITS = [
    # dict(id=..., 位置..., 可被观测的单元列表)
    # 例：dict(id=0, x=0.0, y=0.0, observed=list(range(12))),   # 12 个单元都可测
    #     dict(id=2, x=10.0, y=0.0, observed=[0,1,2,3,4,5,6,7]), # 只有 8 个可测 -> 分母是 8
    #     dict(id=4, x=20.0, y=0.0, observed=[0,1,2]),           # 3 个 < 阈值 -> 单独报告
    #     dict(id=5, x=25.0, y=0.0, observed=[]),                # 0 个 -> 应判 nan，剔除出主指标
]


def fixture_points(unit, cells):
    """在指定单元上生成点（每单元若干个、落在不同体素内）。TODO 按你的几何改写。"""
    raise NotImplementedError


def reference_cloud(units):
    """"真值点云"= 在所有可测单元上生成的点。"""
    return np.vstack([fixture_points(u, u["observed"]) for u in units if u["observed"]])


# =====================================================================
# 必须包含的断言（每条都对应一个真实口径约定，别删）
# =====================================================================
def main():
    # ---- 场景 A：全扫到 ----
    # rA = EM.eval_flight(reference_cloud(UNITS), anchor)
    # check("A 平均覆盖率", float(rA["mean_cov_main"]), 1.0, 1e-6)
    # check("A 达标数", rA["n_reach_tau_main"], 4)
    # ★★ 分母回归测试：某目标只有 8 个可测单元且全被扫到 ⇒ 覆盖率必须是 1.0，
    #                  不是 8/12 = 0.667。这条就是"分母用可达成量"这个修正的回归测试。
    # check("A 目标2 覆盖率(分母=8, 不是12)", float(rA["cov_all"][2]), 1.0, 1e-6)
    # check("A 单独报告(<阈值单元数) 棵数", rA["n_coverable_lt_min"], 1)
    # check("A 无法评估(nan) 棵数", rA["n_nan"], 1)

    # ---- 场景 B：只扫一半 ----
    # check("B 平均覆盖率", float(rB["mean_cov_main"]), 0.5, 1e-6)
    # check("B 达标数(τ=0.83)", rB["n_reach_tau_main"], 0)

    # ---- 场景 C：一个都没扫 ----
    # check("C 平均覆盖率", float(rC["mean_cov_main"]), 0.0, 1e-9)

    # ---- τ 曲线：达标率随 τ 单调不增 ----
    # cur = EM.tau_curve(flightB, anchor)
    # check("τ 曲线单调不增", all(cur[i][1] >= cur[i+1][1] - 1e-12 for i in range(len(cur)-1)), True)

    # ---- 检测匹配（PR）：匹配规则 = 一对一 + 阈值 max(0.3, 2r) ----
    # 完全重合 ⇒ P=R=1.0
    # 真值全命中 + 恰好 k 个远距离误检 ⇒ R=1.0 且 P 按比例下降
    #   ⚠️ 坑2：期望值要按夹具【实际形状】算。
    #      实例：np.vstack([truth] + [truth+offset]*5) 真值 m 行 × 5 份 = 5m 个误检（不是 5 个）
    #            ⇒ 期望应是 m/(m+5m)，我却填了 m/(2m) —— 期望算错，不是评估器错。
    # 整体偏移【超过】阈值 ⇒ 匹配数必须 = 0（证明阈值真在起作用）
    pass

    # ---- 汇总 ----
    bad = [r for r in _rows if r[0] == FAIL]
    print("================ 合成校准结果 ================")
    for (st, tag, got, want) in _rows:
        print("%s %-4s %-40s got=%-16s want=%s" % ("  " if st == PASS else ">>", st, tag, got, want))
    print("\n  %d/%d 通过" % (len(_rows) - len(bad), len(_rows)))
    print("  ★ 尺子校准: %s" % ("通过 —— 可以换用真实真值" if not bad else "未通过 —— 先修评估器/先重算期望值"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
