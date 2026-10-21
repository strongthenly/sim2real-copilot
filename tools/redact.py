#!/usr/bin/env python3
"""Redact sensitive values in text files using a LOCAL mapping file.

The mapping file holds the real secrets, so it must never be committed
(it is matched by .gitignore: *.local.json). Format:
  {"rules": [{"type": "literal"|"regex", "find": "...", "replace": "..."}]}

Usage:
  python tools/redact.py --map ../redaction_map.local.json knowledge/
  python tools/redact.py --map ... --reverse knowledge/   # restore (literal rules only)
"""
import argparse, json, pathlib, re, sys

TEXT_EXT = {".md", ".txt", ".json", ".yaml", ".yml", ".py", ".sh", ".ps1", ".bat",
            ".launch", ".xml", ".cfg", ".ini", ".toml", ".csv", ".cpp", ".h", ".hpp", ".rviz", ""}

def load_rules(path):
    rules = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))["rules"]
    out = []
    for r in rules:
        if r["type"] == "regex":
            out.append((re.compile(r["find"]), r["replace"], True))
        else:
            out.append((r["find"], r["replace"], False))
    return out

def apply(text, rules):
    n = 0
    for find, rep, is_re in rules:
        if is_re:
            text, k = find.subn(rep, text)
        else:
            k = text.count(find); text = text.replace(find, rep)
        n += k
    return text, n

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--reverse", action="store_true")
    ap.add_argument("paths", nargs="+")
    a = ap.parse_args()
    rules = load_rules(a.map)
    if a.reverse:
        rules = [(rep, find, False) for find, rep, is_re in rules if not is_re]
    total = files = 0
    for p in a.paths:
        p = pathlib.Path(p)
        for f in ([p] if p.is_file() else p.rglob("*")):
            if not f.is_file() or f.suffix.lower() not in TEXT_EXT:
                continue
            try:
                t = f.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            new, n = apply(t, rules)
            if n:
                f.write_text(new, encoding="utf-8"); total += n; files += 1
    print(f"replaced {total} occurrences in {files} files")

if __name__ == "__main__":
    sys.exit(main())
