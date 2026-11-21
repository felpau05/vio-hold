#!/usr/bin/env python3
"""Error timeline of ol_kfmap.py results: per 2.5 s bin the mean fix error of sent fixes, and the events (re-anchor,
forced re-anchor, relocalization). Run: ~/.venvs/vio/bin/python integ/kfmap/timeline.py <result .json.gz> [...]"""
import gzip
import json
import sys

import numpy as np

B = 2.5
for p in sys.argv[1:]:
    d = json.load(gzip.open(p, "rt"))
    rows = d["rows"]
    T = d["seconds"]
    out = []
    for b in np.arange(0, T, B):
        w = [r for r in rows if b <= r["t"] < b + B]
        e = [r["err"] for r in w if r["sent"]]
        ev = "".join("R" if r["reanchor"] else "F" if r.get("forced") else "" for r in w)
        ev += "A" * sum(1 for x in d["relocs"] if x["accepted"] and b <= x["t_s"] < b + B)
        out.append(f"{np.mean(e):.2f}{ev}" if e else f" -- {ev}")
    print(d["label"][-40:].ljust(40), " ".join(x.ljust(5) for x in out))
