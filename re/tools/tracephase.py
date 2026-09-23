#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later WITH SANE-exception
# Copyright (C) 2026 Evan McClain
"""Split a neatcap trace into SNCmd phases and summarise each phase as an
ordered list of device operations (register writes, AFE writes, bulk
uploads) so traces of different scan modes can be diffed."""
import re
import sys


def parse(path):
    phases = []
    cur = None
    for line in open(path):
        line = line.rstrip('\n')
        m = re.match(r'### SNCmd\(0x(\w+) (\S+) p5', line)
        if m:
            cur = {'cmd': int(m.group(1), 16), 'name': m.group(2), 'ops': []}
            phases.append(cur)
            continue
        if cur is None or line.startswith('###'):
            continue
        f = line.split()
        if not f:
            continue
        if f[0] == 'CO' and f[2] == '0083':
            data = bytes.fromhex(f[5])
            pairs = [(data[i], data[i + 1]) for i in range(0, len(data), 2)]
            cur['ops'].append(('W', pairs))
        elif f[0] == 'CO' and f[2] == '0082':
            data = bytes.fromhex(f[5])
            addr = int.from_bytes(data[:4], 'little')
            ln = int.from_bytes(data[4:8], 'little')
            cur['ops'].append(('HDR', int(f[3], 16), addr, ln))
        elif f[0] == 'CO':
            cur['ops'].append(('CO', f[1], f[2], f[3], f[5] if len(f) > 5 else ''))
        elif f[0] == 'BO':
            cur['ops'].append(('BO', int(f[1]), f[2]))
        elif f[0] == 'CI':
            cur['ops'].append(('CI', f[2], f[3], int(f[4])))
        elif f[0] == 'BI':
            cur['ops'].append(('BI', int(f[1])))
        elif f[0] == 'Sleep':
            cur['ops'].append(('SLEEP', int(f[1])))
    return phases


def fmt(op):
    if op[0] == 'W':
        return 'W ' + ' '.join('%02x=%02x' % p for p in op[1])
    if op[0] == 'HDR':
        return 'HDR %s addr=%08x len=%d' % ('out' if op[1] else 'in', op[2], op[3])
    if op[0] == 'BO':
        return 'BO %d %s' % (op[1], op[2][:64] + ('...' if len(op[2]) > 64 else ''))
    return ' '.join(str(x) for x in op)


if __name__ == '__main__':
    want = sys.argv[2] if len(sys.argv) > 2 else None
    for ph in parse(sys.argv[1]):
        if want and ph['name'] != want:
            continue
        print('#### %s (0x%x)' % (ph['name'], ph['cmd']))
        for op in ph['ops']:
            print(fmt(op))
