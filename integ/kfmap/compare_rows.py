#!/usr/bin/env python3
"""Frame-by-frame identity check of two versions of one ol_kfmap.py replay: every Est field kept (pos, var, state, why,
n, sent) must be equal, floats bit for bit.
Run: ~/.venvs/vio/bin/python integ/kfmap/compare_rows.py <label> <versionA> <versionB> [<versionA> <versionB> ...]"""
import gzip
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
KEYS = ("t", "pos", "var", "state", "why", "n", "sent", "err_all", "reanchor")


def load(label, v):
    with gzip.open(os.path.join(HERE, "results", "ol", f"{label}_{v}.json.gz"), "rt") as f:
        return json.load(f)


def main(label, pairs):
    ok_all = True
    for a, b in pairs:
        ra, rb = load(label, a)["rows"], load(label, b)["rows"]
        diff = [i for i, (x, y) in enumerate(zip(ra, rb)) if any(x[k] != y[k] for k in KEYS)]
        same = len(ra) == len(rb) and not diff
        ok_all &= same
        print(f"{label}: {a} vs {b}: frames {len(ra)} / {len(rb)}, frames that differ {len(diff)}"
              + (f" (first at t {ra[diff[0]]['t']} s)" if diff else "") + (" -> IDENTICAL" if same else " -> DIFFERENT"))
    return ok_all


if __name__ == "__main__":
    lab, vs = sys.argv[1], sys.argv[2:]
    sys.exit(0 if main(lab, list(zip(vs[::2], vs[1::2]))) else 1)
