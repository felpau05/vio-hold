#!/usr/bin/env python3
"""Tables of the open-loop flight replays (integ/lowh/results/ol/*.json).

1. Per run and version: fixes per s (sent, after the Pipe's 10 good fixes), longest time without a sent fix, sent-fix
   error RMS and max (m), 'halves disagree' refusals, re-anchors (Hold lost), sent fixes from the whole-picture path.
2. Halves gap against the true drift from the keyframe (whole-picture calls on the current keyframe): median gap per
   bin of |drift| / height, and the share of calls over 4 px.
Run: ~/.venvs/vio/bin/python integ/lowh/ol_summary.py [--tag _t1] [--bad] > results/<name>.txt
"""
import argparse
import glob
import json
import os
from collections import defaultdict

import numpy as np

from jsonz import jload

HERE = os.path.dirname(os.path.abspath(__file__))
BINS = [0, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 10]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="")
    ap.add_argument("--bad", action="store_true", help="bad-picture runs only (their window table)")
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(HERE, "results", "ol", f"ol_*{a.tag}_*.json*")))
    runs = [jload((f)) for f in files]
    runs = [r for r in runs if bool(r.get("bad")) == a.bad and (a.tag == "" and "_t1_" not in r["label"] or a.tag and a.tag in r["label"])]
    runs.sort(key=lambda r: (r["ground"], r["lens"], r.get("bad") or "", r["height"], r["seed"], r["version"]))
    if a.bad:
        print("Bad picture from 10 to 20 s. hold fixes = Hold gave a fix (var not None); sent = after the Pipe's 10 good fixes;")
        print("max err = largest error of a Hold fix in the window (m).")
        print(f"{'ground':11s} {'bad':14s} {'h':>4s} {'seed':>4s} {'version':7s} {'frames':>6s} {'holdfix':>7s} {'>0.5s':>5s} {'sent':>4s} "
              f"{'wholepic':>8s} {'maxerr':>6s} {'halves':>6s}  refusals")
        for r in runs:
            b = r["bad_window"]
            print(f"{r['ground']:11s} {r['bad']:14s} {r['height']:4g} {r['seed']:4d} {r['version']:7s} {b['frames']:6d} {b['hold_fixes']:7d} "
                  f"{b['hold_fixes_after_0_5s']:5d} {b['sent']:4d} {b['whole_picture_fixes']:8d} {b['max_err_m']:6.2f} {b['halves_refused']:6d}  "
                  f"{dict(list(b['why'].items())[:4])}")
        return
    print("Per run (55 s of the recorded hover, 12 Hz to Hold). fix/s and gap: sent fixes; err: sent-fix error vs truth.")
    print(f"{'ground':11s} {'lens':4s} {'h':>4s} {'seed':>4s} {'version':7s} {'fix/s':>5s} {'gap s':>5s} {'rms m':>5s} {'max m':>5s} "
          f"{'halves':>6s} {'reanc':>5s} {'wholepic/sent':>13s} {'gap px p50/p90':>14s}")
    agg = defaultdict(list)
    for r in runs:
        s = r["summary"]
        g = s["halves_gap_px"]
        print(f"{r['ground']:11s} {r['lens']:4s} {r['height']:4g} {r['seed']:4d} {r['version']:7s} {s['fix_hz']:5.2f} {s['longest_gap_s']:5.2f} "
              f"{s['err_rms_m'] if s['err_rms_m'] is not None else float('nan'):5.2f} {s['err_max_m'] if s['err_max_m'] is not None else float('nan'):5.2f} "
              f"{s['halves_refused']:6d} {s['reanchors']:5d} {s['sent_whole_picture']:6d}/{s['sent']:<6d} "
              f"{(g['median'] if g['median'] is not None else float('nan')):6.2f}/{(g['p90'] if g['p90'] is not None else float('nan')):<6.2f}")
        agg[(r["ground"], r["lens"], r["height"], r["version"])].append(r)
    print("\nMedian over seeds (worst seed in brackets):")
    print(f"{'ground':11s} {'lens':4s} {'h':>4s} {'version':7s} {'fix/s':>12s} {'gap s':>12s} {'rms m':>12s} {'halves':>10s} {'reanc':>8s}")
    for k, rs in agg.items():
        S = [x["summary"] for x in rs]
        f = lambda key, worst: (np.median([s[key] for s in S if s[key] is not None]), worst([s[key] for s in S if s[key] is not None]))   # noqa: E731
        fr, gp, rm, hv, ra = f("fix_hz", min), f("longest_gap_s", max), f("err_rms_m", max), f("halves_refused", max), f("reanchors", max)
        print(f"{k[0]:11s} {k[1]:4s} {k[2]:4g} {k[3]:7s} {fr[0]:5.2f} [{fr[1]:5.2f}] {gp[0]:5.2f} [{gp[1]:5.2f}] {rm[0]:5.2f} [{rm[1]:5.2f}] "
              f"{hv[0]:4.0f} [{hv[1]:3.0f}] {ra[0]:3.0f} [{ra[1]:2.0f}]")
    print("\nHalves gap (px) against the true drift from the keyframe / height, whole-picture calls on the current keyframe,")
    print("all seeds: median gap [share of calls over 4 px] (number of calls)")
    keys = sorted({(r["ground"], r["lens"], r["height"], r["version"]) for r in runs})
    print(f"{'ground':11s} {'lens':4s} {'h':>4s} {'version':7s} " + " ".join(f"{f'{lo:.2f}-{hi:.2f}':>18s}" for lo, hi in zip(BINS[:-1], BINS[1:])))
    for k in keys:
        calls = [(math_hypot(g), g["gap"]) for r in agg[k] for row in r["rows"] for g in row["dense"]
                 if g["cur"] and g["gap"] is not None]
        if not calls:
            continue
        x = np.array(calls)
        x[:, 0] /= k[2]
        cells = []
        for lo, hi in zip(BINS[:-1], BINS[1:]):
            m = (x[:, 0] >= lo) & (x[:, 0] < hi)
            cells.append(f"{np.median(x[m, 1]):5.1f} [{100 * np.mean(x[m, 1] > 4):3.0f}%] ({m.sum():4d})" if m.any() else " " * 18)
        print(f"{k[0]:11s} {k[1]:4s} {k[2]:4g} {k[3]:7s} " + " ".join(f"{c:>18s}" for c in cells))


def math_hypot(g):
    return float(np.hypot(g["drift_fwd"], g["drift_right"]))


if __name__ == "__main__":
    main()
