#!/usr/bin/env python3
"""Before / after tables of the low-height study (integ/lowh/results/ol).

'after' = the copy's hold.py = proposed.patch = variant geo2_local (make_variants.py builds it byte for byte).
'first try' = variant geo_local (the warp also started at the last fix; dropped, see README). 'local' = the unit change
alone. Median over seeds 11-13, worst seed in brackets. Columns: halves refusals, fixes per s (sent, after the Pipe's 10
good fixes), longest time without a sent fix (s), sent-fix error RMS (m), re-anchors (Hold lost).
A: re-anchor at the true position after 'lost' (runs *_g2_*); lowh_fine from *_before/after.json (corner path only).
B: re-anchor at Hold's own last fix (runs *_relast_*).
Run: ~/.venvs/vio/bin/python integ/lowh/final_tables.py > integ/lowh/results/before_after.txt
"""
import glob
import json
import os
from collections import defaultdict

import numpy as np

from jsonz import jload

HERE = os.path.dirname(os.path.abspath(__file__))
OL = os.path.join(HERE, "results", "ol")
NAMES = {"geo2_local": "after", "after_geo_local": "first try", "geo_local": "first try", "after": "first try"}
ORDER = ["before", "after", "first try", "local", "geo"]


def load(pattern, keep=lambda d: True):
    out = defaultdict(list)
    for f in glob.glob(os.path.join(OL, pattern)):
        d = jload((f))
        if d.get("bad") or d.get("diag") or not keep(d):
            continue
        v = NAMES.get(d["version"], d["version"])
        out[(d["ground"], d["lens"], int(d["height"]), v)].append(d["summary"])
    return out


def cell(S, key, worst):
    x = [s[key] for s in S if s[key] is not None]
    return f"{np.median(x):6.2f} [{worst(x):6.2f}]" if x else f"{'-':>15s}"


def table(title, runs):
    print(title)
    print(f"{'ground':11s} {'lens':4s} {'h':>3s} {'version':9s} {'halves':>15s} {'fix/s':>15s} {'gap s':>15s} {'err rms m':>15s} "
          f"{'re-anchors':>15s} {'n':>2s}")
    for k in sorted({k[:3] for k in runs}):
        for v in ORDER:
            S = runs.get(k + (v,))
            if S:
                print(f"{k[0]:11s} {k[1]:4s} {k[2]:3d} {v:9s} {cell(S, 'halves_refused', max)} {cell(S, 'fix_hz', min)} "
                      f"{cell(S, 'longest_gap_s', max)} {cell(S, 'err_rms_m', max)} {cell(S, 'reanchors', max)} {len(S):2d}")
    print()


def main():
    a = load("ol_*_g2_*.json*", lambda d: d.get("reanchor_at", "truth") == "truth")
    fine = load("ol_lowh_fine_*_s1[123]_before.json*") | load("ol_lowh_fine_*_s1[123]_after.json*")
    table("A. Re-anchor at the true position after 'lost' (a perfect FC position: it resets the fix error to 0)", a | fine)
    b = load("ol_*_g2_relast_*.json*")            # before, after
    b.update({k: v for k, v in load("ol_*_s1[123]_relast_*.json*").items() if k[3] != "before"})   # first try, geo, local
    table("B. Re-anchor at Hold's own last fix after 'lost' (apb_main.Pipe without an FC position)", b)


if __name__ == "__main__":
    main()
