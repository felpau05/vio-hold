#!/usr/bin/env python3
"""Print the sent-fix error against the truth (FC heading frame) of one SITL run over time, with re-anchor notes,
and the EKF position error (ArduPilot LOCAL_POSITION_NED from the npz 'ekf' array) at the same times.
Run: ~/.venvs/vio/bin/python integ/diag10/cl_series.py <label> [t_from t_to step]   (t relative to the switch)"""
import csv, glob, json, math, os, sys
import numpy as np
PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.join(PROJ, "apsim", "runs")
lab = sys.argv[1]
a, b, st = (float(x) for x in sys.argv[2:5]) if len(sys.argv) > 4 else (-12, 60, 1.0)
d = np.load(os.path.join(RUNS, lab + ".npz")); r = json.load(open(os.path.join(RUNS, lab + ".json")))
tr, te, ekf = d["truth"], float(d["t_engage"]), d["ekf"]
dy, ro = math.radians(r["ekf_yaw_minus_true_deg"]), r["cfg"]["readout_s"]
rows = []
for p in sorted(glob.glob(os.path.join(RUNS, lab + "_onboard*.csv"))):
    rows += [x for x in csv.DictReader(open(p)) if x["t"]]
rows.sort(key=lambda x: float(x["t"]))
c, s = math.cos(dy), math.sin(dy)
def T(t):
    N, E = np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])
    return np.array([c * N - s * E, s * N + c * E])
print("ekf cols sample", ekf[:2])
nxt = a
for x in rows:
    t = float(x["t"]) + ro / 2 - te
    if t < a or t > b:
        continue
    note = x["note"]
    if x["sent"] == "1" and (t >= nxt or note):
        e = np.array([float(x["north"]), float(x["east"])]) - T(t + te)
        k = np.searchsorted(ekf[:, 0], t + te)
        ee = ekf[min(k, len(ekf) - 1), 1:3] - T(ekf[min(k, len(ekf) - 1), 0])
        print(f"{t:7.2f} n {x['n']:>4} err {e[0]:+.2f} {e[1]:+.2f}  ekf_err {ee[0]:+.2f} {ee[1]:+.2f}  {note[:60]}")
        nxt = t + st
    elif "re-anchored" in note or "engaged" in note:
        k = np.searchsorted(ekf[:, 0], t + te)
        ee = ekf[min(k, len(ekf) - 1), 1:3] - T(ekf[min(k, len(ekf) - 1), 0])
        print(f"{t:7.2f} ---- {x['state']:>8} {x['why'][:30]:30s} ekf_err {ee[0]:+.2f} {ee[1]:+.2f} NOTE {note[:70]}")
