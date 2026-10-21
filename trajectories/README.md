# trajectories/ — 工作轨迹：知识的原料

完整记录每次与 agent 的工作过程，是后续一切改进的来源。**事后补不回来，所以从第一天就记。**

## 目录
| 路径 | 内容 | 是否提交 |
|---|---|---|
| `raw/` | 原始记录，可能含口令、IP、日志原文 | **否**（已在 .gitignore） |
| `curated/` | 脱敏后、有价值的轨迹 | 是（提交前跑 `tools/check_secrets.py`） |
| `examples/` | 格式示例 | 是 |

## 格式（每个会话一行 JSON，文件为 `.jsonl`）

```json
{
  "session_id": "2026-10-21-001",
  "model": "deepseek",                 // 对应 models/<name>/
  "task": "诊断仿真中飞机悬停不动",
  "layer": "L1",
  "env": "sim | hil | real",
  "steps": [{"tool": "...", "args": "...", "result_summary": "..."}],
  "prediction": "仿真预测的结果（如有）",
  "real_outcome": "实机实际结果（如有）",
  "user_corrections": ["不对，实际生效的配置在 task_run_scripts 里"],
  "outcome": "solved | partially | failed",
  "knowledge_used": ["knowledge/gap-ledger/GAP-0002-no-rc-in-sim.md"],
  "linked_logs": ["flight_0412.ulg", "sim_run_2201"]
}
```

## 挖掘：轨迹 → 知识 → eval

```bash
python tools/mine_trajectories.py trajectories/curated/*.jsonl
```

脚本按以下规则生成**候选**文件（不会直接修改 knowledge/）：

| 信号 | 产出 | 去向 |
|---|---|---|
| `user_corrections` 非空 | 候选知识草稿 | `models/<model>/inbox/` |
| `prediction` 与 `real_outcome` 不一致 | 候选差异台账条目 | `models/<model>/inbox/` |
| `outcome` 为 failed | 候选 eval 用例 | `evals/candidates/` |

然后人工审核 → 跑 eval → 晋升到 `knowledge/` 或 `evals/cases/`。

## 还可以从轨迹里看什么
- **工具缺口**：agent 反复手写同类脚本 → 应该做成固定工具
- **知识没被用上**：`knowledge_used` 里有正确条目但仍然失败 → 是知识写法问题，不是检索问题
- **模型差异**：同类任务不同模型的失败模式 → 写入 `models/<m>/notes.md`
