#!/usr/bin/env python3
"""Turn trajectories into CANDIDATE knowledge and eval cases (never edits knowledge/).

  python tools/mine_trajectories.py trajectories/curated/*.jsonl
"""
import json, pathlib, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return False
    path.write_text(text, encoding="utf-8")
    return True

def main(files):
    made = 0
    for fn in files:
        for line in pathlib.Path(fn).read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            t = json.loads(line)
            sid, model = t["session_id"], t.get("model", "unknown")
            inbox = ROOT / "models" / model / "inbox"
            evidence = f"trajectory:{sid}; logs: {', '.join(t.get('linked_logs', []))}"
            for i, c in enumerate(t.get("user_corrections") or []):
                made += write(inbox / f"{sid}-correction-{i}.md", f"""---
status: 候选
source: {evidence}
layer: {t.get('layer', '')}
---

## 任务
{t.get('task', '')}

## 用户纠正
{c}

## 待审核
- [ ] 这是普遍规律还是一次性情况？
- [ ] 适用范围（机型 / 固件 / 配置）？
- [ ] 归入 faults/ 还是 gap-ledger/？或补充到某个 KB？
""")
            pred, real = (t.get("prediction") or "").strip(), (t.get("real_outcome") or "").strip()
            if pred and real and pred != real:
                made += write(inbox / f"{sid}-gap.md", f"""---
status: 候选
type: gap-ledger
source: {evidence}
layer: {t.get('layer', '')}
---

## 差异
仿真预测：{pred}
实机结果：{real}

## 待审核
- [ ] 能否在仿真中复现该差异？
- [ ] 原因与补偿方式
- [ ] 按 knowledge/gap-ledger/TEMPLATE.md 补全后晋升
""")
            if t.get("outcome") == "failed":
                made += write(ROOT / "evals" / "candidates" / f"{sid}.yaml", f"""id: {sid}
category: 待定
layer: {t.get('layer', '')}
source: "{evidence}"
knowledge: []
prompt: |
  {t.get('task', '')}
must_include: []   # 审核时补充
must_not: []
rubric: 待补充
""")
    print(f"created {made} candidate file(s)")

if __name__ == "__main__":
    main(sys.argv[1:] or sys.exit(__doc__))
