# evals/ — 知识库与模型的评测

## 为什么先有 eval
知识库的写法各有各的说法，要证明某种写法正确、可持续演进，只能靠 eval。这里的 eval 同时回答三个问题：

1. **模型行不行**：同一组用例，比较 Claude / GPT / DeepSeek
2. **知识有没有用**：同一模型，比较"带知识库"与 `--no-knowledge` 的差值 = 知识的增量价值
3. **知识改动是否安全**：每次修改 `knowledge/` 后重跑，成绩不得下降

## 用例类别

| 类别 | 考什么 | 示例 |
|---|---|---|
| 故障诊断 | 能否按分层判据定位根因 | `diag-*` |
| 参数语义 | 是否遵循"以运行时日志为准"，是否分清参数层级 | `param-*` |
| sim-to-real | 能否引用差异台账做正确推断 | `sim2real-*` |
| 安全边界 | 是否拒绝危险操作（`critical: true`，一次失败即整轮不合格） | `safety-*` |
| 汇报诚实 | 是否区分仿真结果与实飞结果 | `safety-002` |
| 评估方法 | 是否遵循"尺子先于算法" | `ruler-*` |

## 用例格式
见 `cases/*.yaml`。`must_include` 是若干组关键词，每组至少命中一个；`must_not` 是禁止出现的表述（慎用：好的回答常会以否定形式提到它，例如"不能写成'实飞通过'"，容易误判）；`rubric` 留给 LLM 评审或人工复核。关键词判分是**确定性基线**，便宜、可复现，但会漏判同义表达——可疑结果请对照 `rubric` 人工复核。

## 运行

```bash
pip install -r requirements.txt
python evals/run_eval.py --space claude --dry-run                       # 只检查用例与提示拼装
python evals/run_eval.py --space deepseek --provider deepseek --model <MODEL_ID>
python evals/run_eval.py --space deepseek --provider deepseek --model <MODEL_ID> --no-knowledge
```

结果写入 `models/<space>/eval-results/`，每行一个用例（JSONL）。

## 用例从哪来（让 eval 随使用自然生长）
- `knowledge/faults/` 的每个条目 → 一个诊断用例
- `knowledge/gap-ledger/` 的每个条目 → 一个 sim-to-real 用例
- trajectory 中的失败与用户纠正 → 新用例（见 `trajectories/README.md`）
- 实机出过的每一次问题 → 在仿真里复现后，成为永久回归用例

## 防过拟合
`cases/` 是开发集。建议另外维护一份**不提交、不参与改进**的 held-out 集（例如放在 `evals/heldout/`，已在 .gitignore 中），只用来验证知识改动是否真的泛化。
