#!/usr/bin/env python3
"""Run evals/cases against Claude / GPT / DeepSeek with the same knowledge.

Examples
  python evals/run_eval.py --space claude   --provider anthropic --model <MODEL_ID>
  python evals/run_eval.py --space gpt      --provider openai    --model <MODEL_ID>
  python evals/run_eval.py --space deepseek --provider deepseek  --model <MODEL_ID>
  # knowledge ablation: same cases, no knowledge files in context
  python evals/run_eval.py --space deepseek --provider deepseek --model <MODEL_ID> --no-knowledge
  # check cases and prompt assembly without calling any API
  python evals/run_eval.py --space claude --dry-run

API keys: ANTHROPIC_API_KEY / OPENAI_API_KEY / DEEPSEEK_API_KEY (put them in .env, never commit).
"""
import argparse, datetime, json, os, pathlib, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
KB = ROOT / "knowledge"
MAX_KB_CHARS = 120_000   # per case; large domain files are truncated

def load_cases(pattern):
    import yaml
    files = sorted((ROOT / "evals" / "cases").glob(pattern))
    return [yaml.safe_load(f.read_text(encoding="utf-8")) for f in files]

def system_prompt(space, with_knowledge, case):
    parts = [(ROOT / "models" / "core_prompt.md").read_text(encoding="utf-8")]
    adapter = ROOT / "models" / space / "adapter.md"
    if adapter.exists():
        parts.append(adapter.read_text(encoding="utf-8"))
    if with_knowledge:
        files = ["00_handbook.md"] + [k for k in case.get("knowledge", []) if k != "00_handbook.md"]
        budget = MAX_KB_CHARS
        for rel in files:
            p = KB / rel
            if not p.exists() or budget <= 0:
                continue
            text = p.read_text(encoding="utf-8")[:budget]
            budget -= len(text)
            parts.append(f"\n\n===== knowledge/{rel} =====\n{text}")
    else:
        parts.append("\n\n（本轮为知识消融实验：不提供 knowledge/ 内容，请仅凭自身知识回答。）")
    return "\n\n".join(parts)

def call(provider, model, system, user):
    if provider == "anthropic":
        import anthropic
        r = anthropic.Anthropic().messages.create(
            model=model, max_tokens=2000, system=system,
            messages=[{"role": "user", "content": user}])
        return "".join(b.text for b in r.content if getattr(b, "type", "") == "text")
    from openai import OpenAI
    if provider == "deepseek":
        client = OpenAI(api_key=os.environ["DEEPSEEK_API_KEY"], base_url="https://api.deepseek.com")
    else:
        client = OpenAI()
    r = client.chat.completions.create(
        model=model, messages=[{"role": "system", "content": system},
                               {"role": "user", "content": user}])
    return r.choices[0].message.content or ""

def grade(case, answer):
    low = answer.lower()
    groups = case.get("must_include") or []
    hit = [any(k.lower() in low for k in g) for g in groups]
    bad = [k for k in (case.get("must_not") or []) if k.lower() in low]
    passed = all(hit) and not bad
    return {"passed": passed, "groups_hit": hit, "forbidden_found": bad}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--space", required=True, choices=["claude", "gpt", "deepseek"])
    ap.add_argument("--provider", choices=["anthropic", "openai", "deepseek"])
    ap.add_argument("--model")
    ap.add_argument("--cases", default="*.yaml")
    ap.add_argument("--no-knowledge", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    cases = load_cases(a.cases)
    if a.dry_run:
        for c in cases:
            s = system_prompt(a.space, not a.no_knowledge, c)
            print(f"{c['id']:<14} {c['category']:<8} system={len(s):>7} chars  groups={len(c.get('must_include') or [])}")
        print(f"{len(cases)} cases OK"); return 0
    if not (a.provider and a.model):
        sys.exit("--provider and --model are required unless --dry-run")
    tag = "noKB" if a.no_knowledge else "KB"
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M")
    out_dir = ROOT / "models" / a.space / "eval-results"
    out = out_dir / f"{stamp}_{a.model.replace('/', '_')}_{tag}.jsonl"
    rows, critical_fail = [], False
    for c in cases:
        ans = call(a.provider, a.model, system_prompt(a.space, not a.no_knowledge, c), c["prompt"])
        g = grade(c, ans)
        critical_fail |= bool(c.get("critical")) and not g["passed"]
        rows.append({"id": c["id"], "category": c["category"], "critical": bool(c.get("critical")),
                     **g, "answer": ans})
        print(f"{'PASS' if g['passed'] else 'FAIL'}  {c['id']}")
    with out.open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    n = sum(r["passed"] for r in rows)
    print(f"\n{a.space}/{a.model} [{tag}]: {n}/{len(rows)} passed"
          + ("  ❌ CRITICAL CASE FAILED" if critical_fail else ""))
    print(f"results -> {out.relative_to(ROOT)}")
    return 1 if critical_fail else 0

if __name__ == "__main__":
    sys.exit(main())
