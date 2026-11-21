#!/usr/bin/env python3
"""Totals of the bad-picture replays (ol_low.py --bad), over seeds 11-13: per bad kind, ground, height and version, the
frames in the 10 s bad window, Hold fixes (var not None), Hold fixes later than 0.5 s into the window, fixes sent after
the Pipe's 10 good fixes, the largest error of a Hold fix in the window, and 'halves disagree' refusals.
Run: ~/.venvs/vio/bin/python integ/lowh/safety_totals.py _g2 [_var] > integ/lowh/results/safety_totals.txt
"""
import glob
import json
import os
import sys
from collections import defaultdict

from jsonz import jload

HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = {"geo2_local": "after", "after_geo_local": "first try", "geo_local": "first try"}


def main():
    for tag in sys.argv[1:] or ["_g2"]:
        T = defaultdict(lambda: [0, 0, 0, 0, 0.0, 0, 0])
        for f in glob.glob(os.path.join(HERE, "results", "ol", f"ol_*{tag}_bad_*.json*")):
            d = jload((f))
            b = d["bad_window"]
            t = T[(d["bad"], d["ground"], int(d["height"]), NAMES.get(d["version"], d["version"]))]
            t[0] += b["frames"]; t[1] += b["hold_fixes"]; t[2] += b["hold_fixes_after_0_5s"]; t[3] += b["sent"]
            t[4] = max(t[4], b["max_err_m"]); t[5] += b["halves_refused"]; t[6] += 1
        print(f"runs with tag {tag}")
        print(f"{'bad kind':14s} {'ground':11s} {'h':>3s} {'version':10s} {'frames':>6s} {'Hold fixes':>10s} {'>0.5 s':>7s} "
              f"{'sent':>5s} {'max err m':>9s} {'halves':>6s} {'runs':>4s}")
        for k in sorted(T, key=lambda k: (k[0], k[1], k[2], ["before", "after", "first try", "local"].index(k[3])
                                          if k[3] in ("before", "after", "first try", "local") else 9, k[3])):
            t = T[k]
            print(f"{k[0]:14s} {k[1]:11s} {k[2]:3d} {k[3]:10s} {t[0]:6d} {t[1]:10d} {t[2]:7d} {t[3]:5d} {t[4]:9.2f} {t[5]:6d} {t[6]:4d}")
        print()


if __name__ == "__main__":
    main()
