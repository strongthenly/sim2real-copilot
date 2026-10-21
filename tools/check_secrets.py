#!/usr/bin/env python3
"""Scan for things that must not land in a public repo. Exit 1 if anything is found.

Run before every commit:  python tools/check_secrets.py .
Contains only generic patterns - no real values.
"""
import pathlib, re, sys

PATTERNS = {
    "private_ip": re.compile(r"\b(?:192\.168|10\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01]))\.\d{1,3}\.\d{1,3}\b"),
    "ssh_fingerprint": re.compile(r"SHA256:(?!x{10})[A-Za-z0-9+/]{40,}"),
    "plink_password": re.compile(r"-pw\s+['\"]?(?!<)[^\s'\"<>]+"),
    "cn_password": re.compile(r"(?:密码|口令)[：:\s]*`?\d+`?(?![\d.])"),
    "github_token": re.compile(r"\b(?:ghp|gho|ghs|github_pat)_[A-Za-z0-9_]{20,}"),
    "api_key": re.compile(r"\bsk-[A-Za-z0-9]{20,}"),
    "private_key": re.compile(r"-----BEGIN [A-Z ]*PRIVATE KEY-----"),
    "windows_user_path": re.compile(r"C:\\\\?Users\\\\?(?!<)(?!Public)(?!\.\.\.)[A-Za-z0-9_.-]+", re.I),
}
# Generic, publicly documented values that are fine to keep.
ALLOW = {"172.25.16.1"}
ALLOW_PW = {"$PW", "$pw", "%PW%", "PASSWORD"}
SKIP_DIRS = {".git", "trajectories/raw"}

def main(root="."):
    hits = 0
    for f in pathlib.Path(root).rglob("*"):
        if not f.is_file() or any(s in f.as_posix() for s in SKIP_DIRS):
            continue
        if f.name == "check_secrets.py":
            continue
        try:
            text = f.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for i, line in enumerate(text.splitlines(), 1):
            for name, pat in PATTERNS.items():
                for m in pat.finditer(line):
                    v = m.group(0)
                    if v in ALLOW or (name == "plink_password" and v.split()[-1].strip("'\"") in ALLOW_PW):
                        continue
                    hits += 1
                    print(f"{f}:{i}: [{name}] {v}")
    print(f"\n{hits} potential leak(s)" if hits else "OK: no leaks found")
    return 1 if hits else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "."))
