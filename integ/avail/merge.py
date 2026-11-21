#!/usr/bin/env python3
"""Merge builder B's runs (px4sim/runs/<label>.json) into px4sim/results.json and rewrite results.txt;
print builder B's own table (case, drift RMS, max, % frames with a fix, fix Hz, set, seed) to stdout.
Run: ~/.venvs/vio/bin/python integ/avail/merge.py LABEL... > integ/avail/summary_b.txt"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PX = os.path.join(os.path.dirname(os.path.dirname(HERE)), "px4sim")
sys.path.insert(0, PX)
import matrix  # noqa: E402

HOLDOUT = ("holdout_quarry", "holdout_field", "holdout_water")


def which_set(cfg):
    g, sd = cfg.get("ground", "ottawa2019fall"), int(cfg.get("seed", 1))
    if g in HOLDOUT or sd >= 101:
        return "hold-out"
    if g in ("ottawa2022", "farmland") or 11 <= sd <= 19:
        return "validation"
    return "tuning"


res = matrix.load()
print(f"{'case':38s} {'rms m':>6s} {'max m':>6s} {'%fix':>6s} {'fixHz':>6s} {'age':>5s}  set        seed  events")
for lab in sys.argv[1:]:
    p = os.path.join(PX, "runs", lab + ".json")
    if not os.path.exists(p):
        print(f"{lab:38s} (no result file)")
        continue
    r = json.load(open(p))
    cfg = r.get("cfg", {})
    res[lab] = {k: v for k, v in r.items() if k != "cfg"}
    f = lambda k, w=6, d=2: f"{r[k]:{w}.{d}f}" if isinstance(r.get(k), (int, float)) else "-".rjust(w)   # noqa: E731
    ev = "; ".join(f"{t:g}s {w}" for t, w in r.get("events", []))[:110]
    print(f"{lab:38s} {f('rms')} {f('max')} {f('pct_frames_fix', 6, 1)} {f('fix_rate_hz', 6, 1)} {f('age_ms_p50', 5, 0)}  "
          f"{which_set(cfg):10s} {cfg.get('seed', 1):4}  {ev}")
json.dump(res, open(matrix.RESULTS, "w"), indent=1, default=str)
matrix.table(res)
