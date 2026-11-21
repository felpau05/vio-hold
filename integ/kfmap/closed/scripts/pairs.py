#!/usr/bin/env python3
"""base against py + (b), pair by pair, from integ/kfmap/closed/ab_table.json (run metrics_ab.py first). 30 s from the
switch. Counts at 5-10 m: pairs with a lower RMS, lower max, shorter gap; py + (b) runs with a sent fix more than 0.5 m
wrong (30 s and 60 s windows; the hover-15 runs included). Writes integ/kfmap/closed/pairs.txt."""
import json
import os

OUT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
res = json.load(open(os.path.join(OUT, "ab_table.json")))
by = {}
for r in res:
    h = int(r["label"].split("_")[2][:-1])
    seed = int(r["label"].split("_")[4][1:])
    by.setdefault((h, r["ground"], seed), {})[r["version"]] = r
f = lambda x, p=2: "   -" if x is None else f"{x:.{p}f}"   # noqa: E731
lines = ["h   ground          seed  rms30 base->pyb   max30           gap30           ferr30          fmax30          "
         "bad30 (>0.5 m)/sent        FC yaw chg30"]
n = dict(pairs=0, rms=0, mx=0, gap=0)
bad_pyb = []
for k in sorted(by):
    v = by[k]
    for x in ("pyb", "pybh15"):
        if x in v and k[0] <= 10:
            bad_pyb.append((k, x, v[x]["nbad30"], v[x]["nbad60"]))
    if "base" not in v or "pyb" not in v:
        continue
    b, p = v["base"], v["pyb"]
    lines.append(f"{k[0]:<3d} {k[1]:15s} {k[2]:4d}  {f(b['rms30'])} -> {f(p['rms30'])}    {f(b['max30'])} -> {f(p['max30'])}"
                 f"    {f(b['gap30'])} -> {f(p['gap30'])}    {f(b['ferr30'])} -> {f(p['ferr30'])}    {f(b['ferr_max30'])} -> "
                 f"{f(p['ferr_max30'])}    {b['nbad30']}/{b['nsent30']} -> {p['nbad30']}/{p['nsent30']}"
                 f"        {f(b['yaw_chg30'], 1)} -> {f(p['yaw_chg30'], 1)}")
    if k[0] <= 10:
        n["pairs"] += 1
        n["rms"] += p["rms30"] < b["rms30"]
        n["mx"] += p["max30"] < b["max30"]
        n["gap"] += p["gap30"] < b["gap30"]
lines += ["", f"5-10 m: {n['pairs']} pairs; py + (b) lower RMS in {n['rms']}, lower max in {n['mx']}, shorter longest gap in "
          f"{n['gap']}.", "py + (b) runs at 5-10 m (incl. hover 15 s) with sent fixes more than 0.5 m wrong (30 s / 60 s): "
          + (", ".join(f"{k[0]}m {k[1]} s{k[2]} {x}: {a}/{c}" for k, x, a, c in bad_pyb if a or c) or "none"),
          f"py + (b) runs at 5-10 m counted: {len(bad_pyb)}; with any such fix: {sum(1 for *_, a, c in bad_pyb if a or c)}.",
          "(-1 = no fix in the last 1 s before the switch, so the relative error has no reference.)"]
open(os.path.join(OUT, "pairs.txt"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
