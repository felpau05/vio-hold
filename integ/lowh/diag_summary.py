#!/usr/bin/env python3
"""Which input error makes the halves refusals at 5 m (ottawa2022, crop lens, seeds 11-13)? Diagnostic replays
(ol_low.py --diag, tag _diag): Hold gets the true height, the true attitude, no camera vibration, or all of these plus
the true lens. Not results for the camera or the fix: diagnostics only. Sum of 'halves disagree' refusals over 3 seeds
(per seed in brackets), per hold.py version.
Run: ~/.venvs/vio/bin/python integ/lowh/diag_summary.py > integ/lowh/results/diag_5m.txt
"""
import glob
import json
import os
from collections import defaultdict

from jsonz import jload

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    T = defaultdict(list)
    for f in glob.glob(os.path.join(HERE, "results", "ol", "ol_ottawa2022_crop_5m_s1[123]_diag_*.json*")):
        d = jload((f))
        T[(",".join(d["diag"]), d["version"])].append(d["summary"]["halves_refused"])
    for f in glob.glob(os.path.join(HERE, "results", "ol", "ol_ottawa2022_crop_5m_s1[123]_var_*.json*")):
        d = jload((f))
        if not d.get("bad"):
            T[("normal feeds", d["version"])].append(d["summary"]["halves_refused"])
    feeds = ["normal feeds", "true_height", "true_att", "vib0", "true_height,true_att,vib0,exact_lens"]
    versions = sorted({k[1] for k in T}, key=lambda v: (v != "before", v))
    print(f"{'version':18s} " + " ".join(f"{f[:26]:>28s}" for f in feeds))
    for v in versions:
        print(f"{v:18s} " + " ".join(f"{sum(T[(f, v)]):5d} {str(sorted(T[(f, v)])):>22s}" if T.get((f, v)) else f"{'-':>28s}"
                                     for f in feeds))


if __name__ == "__main__":
    main()
