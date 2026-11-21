#!/usr/bin/env python3
"""Compact comparison of probe files (integ/lowh/results/probe): per file and height, the still-hover gap (p90 / max px)
and refusals, and the refusals of the drift frames per drift share d / h (4 directions x 3 seeds = 12 frames each).
A refusal is any reason (halves, weak peak, points disagree, ...); 'halves' counts only 'halves disagree'.
Run: ~/.venvs/vio/bin/python integ/lowh/probe_compare.py PATTERN   (e.g. 'v_*_ottawa2022_*' or 'before2_*')
"""
import glob
import json
import os
import sys

import numpy as np

from jsonz import jload

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    pats = sys.argv[1:] or ["*"]
    files = sorted({f for p in pats for f in glob.glob(os.path.join(HERE, "results", "probe", p + ".json*"))})
    print("still: gap p90/max px, refused n/18 | drift d/h 0.05 / 0.10 / 0.20 / 0.30: refused (halves) of 12 | "
          "fix error |e| median m over accepted drift frames")
    for f in files:
        d = jload((f))
        rows = d["rows"]
        print(f"== {d['label']}")
        for h in sorted({r["h"] for r in rows}):
            st = [r for r in rows if r["h"] == h and r["dir"] == "still"]
            g = [r["gap"] for r in st if r["gap"] is not None]
            cells = []
            for sh in (0.05, 0.1, 0.2, 0.3):
                s = [r for r in rows if r["h"] == h and r["share"] == sh]
                if s:
                    cells.append(f"{sum(not r['ok'] for r in s):2d} ({sum(r['why'].startswith('whole picture: halves') for r in s):2d})")
            e = [abs(r.get("err_fwd", 0) or 0) + 0 for r in rows if r["h"] == h and r["share"] > 0 and r["ok"]]
            e = [np.hypot(r["err_fwd"], r["err_right"]) for r in rows if r["h"] == h and r["share"] > 0 and r["ok"]]
            print(f"  {h:4.0f} m  still {np.percentile(g, 90) if g else float('nan'):5.2f}/{max(g) if g else float('nan'):5.2f} "
                  f"ref {sum(not r['ok'] for r in st):2d}/{len(st)} | drift " + " / ".join(cells)
                  + f" | err {np.median(e) if e else float('nan'):.3f}")


if __name__ == "__main__":
    main()
