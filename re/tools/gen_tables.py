#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later WITH SANE-exception
# Copyright (C) 2026 Evan McClain
"""Generate src/nm1000_tables.h from neatcap traces of the vendor driver.

Each SNCmd phase becomes a small byte-code program replayed by nm1000.c.
Values that differ per device or per scan (AFE offset/gain from the flash
calibration header, the shading tables themselves, the line count) are
replaced with placeholder ops that the driver fills in at run time.

usage: gen_tables.py CAPDIR WARM_OPEN_TRACE REAL_SCAN_TRACE > nm1000_tables.h
  CAPDIR holds np_<res>_<bpp>.log traces (see re/README.md)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from tracephase import parse  # noqa: E402

OP_END, OP_W, OP_SLEEP, OP_8C, OP_BULK, OP_AFECAL, OP_SHADING, OP_LINCNT, OP_WAITMOTOR = range(9)

MODES = [(res, bpp) for res in (150, 200, 300, 600) for bpp in (24, 8)]


def phase(phases, name, nth=0):
    hits = [p for p in phases if p['name'] == name]
    return hits[nth]['ops']


def is_flash_hdr(op):
    return op[0] == 'HDR' and (op[2] >> 24) == 0x03


def encode(ops, calib=False, start=False):
    """Encode trace ops into the byte-code program."""
    out = []
    ops = list(ops)
    # In the calibration phase, the AFE write that immediately precedes the
    # first shading upload carries the per-device offset/gain from flash.
    afecal_idx = None
    if calib:
        for i, op in enumerate(ops):
            if op[0] == 'HDR' and op[1] == 1 and (op[2] >> 24) == 0x10 and op[3] > 4096:
                j = i - 1
                while ops[j][0] != 'W':
                    j -= 1
                afecal_idx = j
                break
    skip_next_bo = False
    shading_ch = 0
    i = 0
    while i < len(ops):
        op = ops[i]
        kind = op[0]
        if kind == 'CI' and op[2] == '4022' and op[3] == 3:
            # "while motor enabled: write 02=58, re-read status" — the
            # iteration count is timing-dependent, so replay it as a wait.
            out.append([OP_WAITMOTOR])
            while (i + 2 < len(ops) and ops[i + 1] == ('W', [(0x02, 0x58)]) and
                   ops[i + 2][0] == 'CI' and ops[i + 2][2] == '4022'):
                i += 2
        elif kind in ('CI', 'BI'):
            pass  # other reads: status polls the driver handles itself
        elif kind == 'W':
            if i == afecal_idx:
                out.append([OP_AFECAL])
            elif start and any(r == 0x25 for r, _ in op[1]):
                rest = [(r, v) for r, v in op[1] if r not in (0x25, 0x26, 0x27)]
                if rest:
                    out.append([OP_W, len(rest)] + [b for p in rest for b in p])
                out.append([OP_LINCNT])
            else:
                out.append([OP_W, len(op[1])] + [b for p in op[1] for b in p])
        elif kind == 'SLEEP':
            ms = op[1]
            out.append([OP_SLEEP, ms & 0xff, ms >> 8])
        elif kind == 'CO' and op[2] == '008c':
            out.append([OP_8C, int(op[3], 16), int(op[4], 16)])
        elif kind == 'HDR':
            if is_flash_hdr(op):
                skip_next_bo = True  # SPI flash traffic: done natively
            elif op[1] == 1:
                nxt = ops[i + 1]
                assert nxt[0] == 'BO' and nxt[1] == op[3], (op, nxt)
                data = bytes.fromhex(nxt[2])
                addr, ln = op[2], op[3]
                if calib and ln > 4096:
                    pad = int.from_bytes(data[4:8], 'little')
                    # Word pair 0 is normally zero, but some modes put the
                    # table's AFE header there (per-device, so flag it).
                    hdr_first = 1 if any(data[0:4]) else 0
                    out.append([OP_SHADING, shading_ch] + list(addr.to_bytes(4, 'little')) +
                               list(ln.to_bytes(4, 'little')) + list(pad.to_bytes(4, 'little')) +
                               [hdr_first])
                    shading_ch += 1
                else:
                    out.append([OP_BULK] + list(addr.to_bytes(4, 'little')) +
                               list(ln.to_bytes(2, 'little')) + list(data))
                i += 1
            else:
                raise SystemExit('unexpected bulk-in header in a replayed phase: %r' % (op,))
        elif kind == 'BO':
            if not skip_next_bo:
                raise SystemExit('bulk out without header: %r' % (op,))
            skip_next_bo = False
        else:
            raise SystemExit('unhandled op %r' % (op,))
        i += 1
    out.append([OP_END])
    return [b for o in out for b in o]


def c_array(name, prog):
    lines = ['static const uint8_t %s[%d] = {' % (name, len(prog))]
    for k in range(0, len(prog), 16):
        lines.append('    ' + ', '.join('0x%02x' % b for b in prog[k:k + 16]) + ',')
    lines.append('};')
    return '\n'.join(lines)


def flash_region(res, bpp):
    return {(600, 24): 0, (300, 24): 1, (600, 8): 2, (300, 8): 3}[(600 if res == 600 else 300, bpp)]


def main():
    capdir, warm_trace, scan_trace = sys.argv[1:4]
    print('// SPDX-License-Identifier: GPL-2.0-or-later WITH SANE-exception')
    print('// Copyright (C) 2026 Evan McClain')
    print('/* Generated by re/tools/gen_tables.py from traces of the vendor driver.')
    print(' * Do not edit by hand. */')
    print('#ifndef NM1000_TABLES_H\n#define NM1000_TABLES_H\n\n#include <stdint.h>\n')
    print('enum { OP_END, OP_W, OP_SLEEP, OP_8C, OP_BULK, OP_AFECAL, OP_SHADING, OP_LINCNT, OP_WAITMOTOR };\n')

    warm = parse(warm_trace)
    print(c_array('prog_common_init', encode(phase(warm, 'open'))))
    scan = parse(scan_trace)
    print(c_array('prog_lamp_on', encode(phase(scan, 'lamp'))))
    print(c_array('prog_stop', encode(phase(scan, 'stop'))))
    print()

    entries = []
    for res, bpp in MODES:
        ph = parse(os.path.join(capdir, 'np_%d_%d.log' % (res, bpp)))
        tag = '%d_%s' % (res, 'color' if bpp == 24 else 'gray')
        print(c_array('prog_setparams_' + tag, encode(phase(ph, 'set-params'))))
        print(c_array('prog_calib_' + tag, encode(phase(ph, 'load-calib'), calib=True)))
        print(c_array('prog_start_' + tag, encode(phase(ph, 'start'), start=True)))
        print()
        entries.append((res, bpp, tag))

    print('struct nm1000_mode {')
    print('    int dpi;\n    int color;\n    int flash_region;\n    int pixels;')
    print('    const uint8_t *setparams, *calib, *start;\n};\n')
    print('static const struct nm1000_mode nm1000_modes[] = {')
    for res, bpp, tag in entries:
        pixels = res * 85 // 10 & ~3
        print('    {%d, %d, %d, %d, prog_setparams_%s, prog_calib_%s, prog_start_%s},' %
              (res, bpp == 24, flash_region(res, bpp), pixels, tag, tag, tag))
    print('};\n')
    print('#endif')


if __name__ == '__main__':
    main()
