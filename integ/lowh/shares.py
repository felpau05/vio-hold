#!/usr/bin/env python3
"""How much of the halves refusals comes from noise and how much from geometry (drift from the keyframe)?

Sequences (the repository's hold.py, flight replays of results/ol/*_t1_before.json): the refusal rate (centre gap > 4 px)
of whole-picture calls with true drift from the keyframe under 0.05 x height is taken as the noise rate (geometry is small
there: under 1.4 px at 5 m in the probe, vib 0). Excess = refusals above that rate at larger drifts, counted as geometry.
Probes (controlled, results/probe/before2_*): still-hover gap with and without camera vibration, and the drift response.
Run: ~/.venvs/vio/bin/python integ/lowh/shares.py > integ/lowh/results/shares.txt
"""
import glob
import json
import os

import numpy as np

from jsonz import jfind, jload

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, "results")


def main():
    print("1. Flight replays, repository hold.py (results/ol/*_t1_before.json, 3 seeds pooled), halves gap at the centre's scale")
    print(f"{'ground':11s} {'h':>3s} {'calls':>6s} {'refused':>7s} {'rate d<0.05h':>12s} {'rate d>=0.05h':>13s} "
          f"{'noise share':>11s} {'geometry share':>14s}")
    for g in ("ottawa2022", "farmland"):
        for h in (5, 10, 20, 40):
            x = []
            for f in glob.glob(os.path.join(R, "ol", f"ol_{g}_crop_{h}m_s*_t1_before.json*")):
                d = jload((f))
                x += [(np.hypot(c["drift_fwd"], c["drift_right"]) / h, c["gap"]) for r in d["rows"] for c in r["dense"]
                      if c["cur"] and c["gap"] is not None]
            if not x:
                continue
            x = np.array(x)
            lo, hi = x[:, 0] < 0.05, x[:, 0] >= 0.05
            ref = x[:, 1] > 4.0
            r0 = ref[lo].mean() if lo.any() else float("nan")
            r1 = ref[hi].mean() if hi.any() else float("nan")
            noise = r0 * len(x)
            n_ref = ref.sum()
            geo = max(0.0, n_ref - noise)
            print(f"{g:11s} {h:3d} {len(x):6d} {n_ref:7d} {100 * r0:11.0f}% {100 * r1:12.0f}% "
                  f"{100 * min(noise, n_ref) / max(n_ref, 1):10.0f}% {100 * geo / max(n_ref, 1):13.0f}%")
    print()
    print("2. Probes (results/probe/before2_*): still hover, halves gap px p90 / max over 3 seeds x 6 frames")
    print(f"{'ground':11s} {'h':>3s} {'centre vib 0.2':>15s} {'centre vib 0':>13s} {'local vib 0.2':>14s}")
    for g in ("ottawa2022", "farmland", "lowh_fine"):
        a = jload(jfind(os.path.join(R, "probe", f"before2_{g}_crop")))["rows"]
        b = jload(jfind(os.path.join(R, "probe", f"before2_{g}_crop_vib0")))["rows"]
        for h in (5.0, 10.0, 20.0, 40.0):
            def st(rows, key):
                v = [r[key] for r in rows if r["h"] == h and r["dir"] == "still" and r.get(key) is not None]
                return f"{np.percentile(v, 90):5.2f} / {max(v):5.2f}"
            print(f"{g:11s} {h:3.0f} {st(a, 'gap_centre'):>15s} {st(b, 'gap_centre'):>13s} {st(a, 'gap_local'):>14s}")
    print()
    print("3. Probes, no vibration (before2_*_vib0): drift from the keyframe d = share x h, halves gap px (centre scale),")
    print("   median over 4 directions x 3 seeds, against the geometric prediction of hold.py's linear model (1 m step)")
    for g in ("lowh_fine", "ottawa2022"):
        rows = jload(jfind(os.path.join(R, "probe", f"before2_{g}_crop_vib0")))["rows"]
        for h in (5.0, 10.0, 40.0):
            cells = []
            for sh in (0.05, 0.1, 0.2):
                s = [r for r in rows if r["h"] == h and r["share"] == sh and r.get("gap_centre") is not None]
                if s:
                    cells.append(f"d/h {sh:.2f}: {np.median([r['gap_centre'] for r in s]):4.1f} "
                                 f"(pred {np.median([r['geom_gap'] for r in s]):4.1f})")
            print(f"  {g:11s} {h:3.0f} m  " + "  ".join(cells))


if __name__ == "__main__":
    main()
