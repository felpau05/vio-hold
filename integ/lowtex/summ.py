#!/usr/bin/env python3
"""Table of closed-loop runs (px4sim/runs/<label>.json): drift RMS / max, fixes per second, % frames with a
fix, board compute per frame, Position-mode events, top refusal reasons.
Run: ~/.venvs/vio/bin/python integ/lowtex/summ.py LABEL... > integ/lowtex/summary.txt"""
import json
import os
import sys

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "px4sim", "runs")
print(f"{'label':22s} {'rms':>6s} {'max':>6s} {'fix/s':>6s} {'%fix':>5s} {'proc':>5s} {'age':>4s}  events | top refusals")
for lab in sys.argv[1:]:
    p = os.path.join(RUNS, lab + ".json")
    if not os.path.exists(p):
        print(f"{lab:22s} (no result file)")
        continue
    r = json.load(open(p))
    f = lambda k, d=2: f"{r[k]:6.{d}f}" if isinstance(r.get(k), (int, float)) else "     -"   # noqa: E731
    ev = "; ".join(f"{t:g}s {w}" for t, w in r.get("events", []))[:150]
    why = ", ".join(f"{k} {v}" for k, v in list((r.get("hold_why") or {}).items())[:3])
    print(f"{lab:22s} {f('rms')} {f('max')} {f('fix_rate_hz', 1)} {f('pct_frames_fix', 0)[1:]} {f('proc_ms_mean', 0)[1:]} "
          f"{f('age_ms_p50', 0)[2:]}  {ev} | {why}")
