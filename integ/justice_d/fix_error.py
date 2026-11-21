#!/usr/bin/env python3
"""Fix error of the onboard program in a closed-loop run: each sent fix (onboard log north, east) against the true
position at its time, turned into the autopilot's heading frame. The constant offset (anchor) is removed; RMS and
worst of what is left. Separates the estimator's error from the drone's response. Also the longest gap without a
sent fix around the hold start (the EKF dead-reckons in a gap, and the hold point is taken on its estimate).
Run: ~/.venvs/vio/bin/python integ/justice_d/fix_error.py <label> ...   (apsim/runs/<label>.npz, _onboard.csv, .json)
"""
import csv
import json
import math
import os
import sys

import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.join(PROJ, "apsim", "runs")


def main(labels):
    out = {}
    for lab in labels:
        d = np.load(os.path.join(RUNS, lab + ".npz"))
        r = json.load(open(os.path.join(RUNS, lab + ".json")))
        tr, te = d["truth"], float(d["t_engage"])
        dy = math.radians(r["ekf_yaw_minus_true_deg"])
        ro = r["cfg"]["readout_s"]
        rows = [x for x in csv.DictReader(open(os.path.join(RUNS, lab + "_onboard.csv"))) if x["sent"] == "1" and x["t"]]
        ts = np.array([float(x["t"]) for x in rows])
        pre = ts[(ts > te - 10) & (ts < te + 3)]
        gaps = np.diff(pre)
        k = int(np.argmax(gaps))
        t = np.array([float(x["t"]) for x in rows]) + ro / 2
        m = (t >= te) & (t <= te + 60)
        p = np.array([[float(x["north"]), float(x["east"])] for x in rows])[m]
        t = t[m]
        N, E = np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])
        c, s = math.cos(dy), math.sin(dy)
        true = np.c_[c * N - s * E, s * N + c * E]
        e = p - true
        e0 = e - e.mean(0)
        n = np.linalg.norm(e0, axis=1)
        out[lab] = dict(fixes=int(len(e)), offset_m=np.round(e.mean(0), 3).tolist(), rms_m=round(float(np.sqrt(np.mean(n ** 2))), 3),
                        worst_m=round(float(n.max()), 3), jitter_m=round(float(np.std(np.diff(e0, axis=0)) / math.sqrt(2)), 3),
                        gap_before_hold_s=round(float(gaps[k]), 2), gap_ends_s=round(float(pre[k + 1] - te), 2))
        print(f"{lab:32s} fixes {len(e):4d}  fix error minus offset: RMS {out[lab]['rms_m']:.3f} m, worst {out[lab]['worst_m']:.3f} m, "
              f"fix-to-fix jitter {out[lab]['jitter_m']:.3f} m; offset {out[lab]['offset_m']}; longest gap without a sent fix in "
              f"[-10, +3] s of the hold start {out[lab]['gap_before_hold_s']} s ending {out[lab]['gap_ends_s']:+} s")
    return out


if __name__ == "__main__":
    res = main(sys.argv[1:])
    json.dump(res, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "fix_error.json"), "w"), indent=1)
