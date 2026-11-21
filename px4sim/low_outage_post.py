#!/usr/bin/env python3
"""Post-run numbers for px4sim/low_outage_run.py results, from px4sim/runs/<label>.npz and the onboard logs.

Outage runs: PX4's position error (EKF minus truth, in PX4's frame) at the outage start and at the re-anchor;
its change is the dead-reckoning drift that the restart's re-anchor keeps as a shift of the hold point. Also the
drone's mean true position (from the hold point) 0-20 s and 33-60 s after the hold start.
All runs: the fix gaps (longest time between two sent fixes) during the hold.

Run: ~/.venvs/vio/bin/python px4sim/low_outage_post.py   (writes px4sim/low_outage_post.txt)
"""
import csv
import glob
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
RESULTS = os.path.join(HERE, "low_outage_results.json")
OUT = os.path.join(HERE, "low_outage_post.txt")


def main():
    res = json.load(open(RESULTS))
    lines = []
    for label, r in res.items():
        p = os.path.join(RUNS, label + ".npz")
        if not os.path.exists(p) or "rms" not in r:
            continue
        d = np.load(p)
        te, hp = float(d["t_engage"]), d["hold_pt"]
        tr, ekf = d["truth"], d["ekf"]
        t = tr[:, 0] - te
        yo = math.radians(r.get("ekf_yaw_minus_true_deg", 0.0))
        R = np.array([[math.cos(yo), -math.sin(yo)], [math.sin(yo), math.cos(yo)]])
        tt = ekf[:, 0] - te
        tn = np.interp(ekf[:, 0], tr[:, 0], tr[:, 1])
        tE = np.interp(ekf[:, 0], tr[:, 0], tr[:, 2])
        err = ekf[:, 1:3] - (R @ np.vstack([tn, tE])).T          # PX4 position minus truth, PX4 frame
        lines.append(label)
        sent = []
        for f in sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv"))):
            sent += [float(x["t"]) - te + float(x["age_ms"] or 0) / 1e3 for x in csv.DictReader(open(f))
                     if x["t"] and x["sent"] == "1"]
        sent = np.array(sorted(s for s in sent if 0 <= s <= t[-1]))
        if len(sent) > 1:
            g = np.diff(sent)
            i = int(np.argmax(g))
            lines.append(f"  longest gap between sent fixes in the hold: {g[i]:.2f} s (from {sent[i]:.2f} s); "
                         f"gaps over 1 s: {int(np.sum(g > 1.0))}")
        oc = (r.get("case_args") or {}).get("outage")
        if oc:
            a, b = oc[0], oc[0] + oc[1]
            m = r.get("outage_metrics", {})
            t_re = m.get("first_frame_after", b)
            ea = np.array([np.interp(a, tt, err[:, 0]), np.interp(a, tt, err[:, 1])])
            eb = np.array([np.interp(t_re, tt, err[:, 0]), np.interp(t_re, tt, err[:, 1])])
            lines.append(f"  PX4 position error at the outage start {np.linalg.norm(ea):.2f} m, at the re-anchor "
                         f"({t_re:.2f} s) {np.linalg.norm(eb):.2f} m; change {np.linalg.norm(eb - ea):.2f} m")
            for lo, hi in ((0, a), (b + 10, t[-1])):
                w = (t >= lo) & (t <= hi)
                mn = np.array([tr[w, 1].mean() - hp[0], tr[w, 2].mean() - hp[1]])
                lines.append(f"  mean true position {lo:.0f}-{hi:.0f} s from the hold point: N {mn[0]:+.2f} E {mn[1]:+.2f} m "
                             f"({np.linalg.norm(mn):.2f} m)")
    open(OUT, "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
