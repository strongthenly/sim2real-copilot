#!/usr/bin/env python3
"""Read PX4 .ulg log identity fields (ver_hw / ver_sw / sys_mcu / sys_name / sys_uuid).

Usage: python ulg_header_probe.py <log.ulg> [more.ulg ...]

Why raw-byte regex instead of strict parsing:
QGC-downloaded / newer .ulg files are not always standard ULog v0 framing
(observed: version byte = 1, B-message framing non-standard) so hand-rolled
strict parsers fail. The info fields are stored as e.g. "char[14] ver_hw"
followed immediately by the ASCII value, so a regex over raw bytes is robust.

Answers: which flight controller / firmware is this log from, and therefore
which PX4 release firmware asset to download (mapping table in SKILL.md):
  HKUST_NXT_DUAL  -> hkust_nxt-dual_default.px4
  PX4_FMU_V5      -> px4_fmu-v5_default.px4
"""
import re
import sys

KEYS = [b'ver_hw', b'ver_sw', b'sys_mcu', b'sys_name', b'sys_os_name', b'sys_uuid']


def probe(path):
    with open(path, 'rb') as f:
        d = f.read()
    print('FILE:', path)
    for key in KEYS:
        m = re.search(re.escape(key) + rb'([ -~]{3,64})', d)
        if m:
            print('  %-11s -> %s' % (key.decode(), m.group(1).decode('ascii', 'replace')))
        else:
            print('  %-11s -> (not found)' % key.decode())


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        probe(p)
