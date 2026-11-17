"""Fix error against the truth for one closed-loop run (px4sim/runs/<label>.npz + its onboard CSV).
Run: ~/.venvs/vio/bin/python integ/fixerr.py <label>"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(os.path.dirname(HERE), "px4sim", "runs")
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "px4sim"))
import camera as C  # noqa: E402

label = sys.argv[1]
d = np.load(os.path.join(RUNS, label + ".npz"))
r = json.load(open(os.path.join(RUNS, label + ".json")))
tr, est, te = d["truth"], d["est"], float(d["t_engage"])
yaw_off = math.radians(r["ekf_yaw_minus_true_deg"])
rows = []
for p in sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv"))):
    rows += list(csv.DictReader(open(p)))
off = est[0, 0] - float(rows[0]["t"]) if len(est) and rows and rows[0]["t"] else 0.0
ok = ~np.isnan(est[:, 1])
t = est[ok, 0]
ro = 0.0184
ti = t + ro / 2                       # est time: t is row-0 time; the fix is mid-frame
N = np.interp(ti, tr[:, 0], tr[:, 1])
E = np.interp(ti, tr[:, 0], tr[:, 2])
c, s = math.cos(yaw_off), math.sin(yaw_off)
Np, Ep = c * N - s * E, s * N + c * E
eN, eE = est[ok, 1] - Np, est[ok, 2] - Ep
sent = est[ok, 7] == 1
m = sent & (t >= te)
print(f"fixes in hold {m.sum()}, error after removing the mean: RMS N {np.std(eN[m]):.3f} E {np.std(eE[m]):.3f} m, "
      f"mean N {eN[m].mean():.2f} E {eE[m].mean():.2f}")
# lag estimate: correlation of fix vs truth shifted
for lag in (0.0, 0.05, 0.1, 0.15, 0.2, 0.3):
    N2 = np.interp(ti - lag, tr[:, 0], tr[:, 1]); E2 = np.interp(ti - lag, tr[:, 0], tr[:, 2])
    a, b = est[ok, 1][m] - (c * N2 - s * E2)[m], est[ok, 2][m] - (s * N2 + c * E2)[m]
    print(f"  truth shifted by {lag*1e3:.0f} ms: error std N {a.std():.3f} E {b.std():.3f}")
ages = np.array([float(x["age_ms"]) for x in rows if x["sent"] == "1" and x["age_ms"]])
procs = np.array([float(x["proc_ms"]) for x in rows if x["state"]])
print("age ms p5/50/95", np.percentile(ages, [5, 50, 95]).round(1), "proc ms p50/95", np.percentile(procs, [50, 95]).round(1))
