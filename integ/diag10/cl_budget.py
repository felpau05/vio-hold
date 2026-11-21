#!/usr/bin/env python3
"""Closed-loop error split from the onboard logs of finished ArduCopter SITL runs (no simulator is run here).

For each run: the error of every sent fix against the truth (FC heading frame, as apsim/engage_matrix.fix_err), and how
the error changes between consecutive sent fixes, split by what happened between them:
  reanchor  a 're-anchored to FC position / last position' note lies between the two fixes
  kf        a keyframe event of <label>_kf.csv lies between them (only runs that have a _kf.csv)
  dense     both fixes from the whole-picture path (n <= 60: 48 virtual points + a few corners; inferred, not logged)
  corner    both fixes from the corner path (n > 60)
  mixed     one of each
The error e(t) is the sum of its increments; the share of each class is sum of squares of its increments over the sum
of squares of all increments (a high-frequency split), and the 'level' split is the variance of the cumulative sum of
each class's increments over the hold (what each class adds to e over 60 s).
Run: ~/.venvs/vio/bin/python integ/diag10/cl_budget.py [label ...]   (default: the JUSTICE_D w20g40 runs at 5-30 m)
"""
import csv
import glob
import gzip
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(PROJ, "apsim", "runs")
AB = os.path.join(PROJ, "integ", "lowh", "closed", "onboard_csv")


def rows_of(path):
    op = gzip.open if path.endswith(".gz") else open
    return sorted([x for x in csv.DictReader(op(path, "rt")) if x["t"]], key=lambda x: float(x["t"]))


def one(label, rows, npz, js, kf=None):
    d = np.load(npz)
    r = json.load(open(js))
    tr, te = d["truth"], float(d["t_engage"])
    dy, ro = math.radians(r["ekf_yaw_minus_true_deg"]), r["cfg"]["readout_s"]
    ev = []        # (t, kind)
    for x in rows:
        if "re-anchored" in x["note"]:
            ev.append((float(x["t"]), "reanchor"))
    if kf is not None:
        for x in csv.DictReader(open(kf)):
            if x["event"] == "kf":
                ev.append((float(x["t_frame"]), "kf"))
    sent = [x for x in rows if x["sent"] == "1"]
    t = np.array([float(x["t"]) for x in sent])
    tt = t + ro / 2
    p = np.array([[float(x["north"]), float(x["east"])] for x in sent])
    n = np.array([int(x["n"]) for x in sent])
    N, E = np.interp(tt, tr[:, 0], tr[:, 1]), np.interp(tt, tr[:, 0], tr[:, 2])
    c, s_ = math.cos(dy), math.sin(dy)
    e = p - np.c_[c * N - s_ * E, s_ * N + c * E]
    rel = tt - te
    pre = e[(rel >= -1) & (rel < 0)]
    hold = (rel >= 0) & (rel <= 60)
    out = dict(label=label, n_sent_hold=int(hold.sum()))
    if len(pre):
        er = e - pre.mean(0)
        out["ferr60"] = round(float(np.sqrt(np.mean(np.sum(er[hold] ** 2, 1)))), 3)
    out["abs_err_rms_hold"] = round(float(np.sqrt(np.mean(np.sum(e[hold] ** 2, 1)))), 3)
    out["share_dense_fixes_hold"] = round(float(np.mean(n[hold] <= 60)), 3)
    # increments
    idx = np.flatnonzero(hold)
    cls = {}
    if len(idx) > 2:
        evt = np.array([x[0] for x in ev]) if ev else np.zeros(0)
        evk = [x[1] for x in ev]
        for a, b in zip(idx[:-1], idx[1:]):
            lo, hi = t[a], t[b]
            kinds = {evk[i] for i in np.flatnonzero((evt > lo) & (evt <= hi))}
            if "reanchor" in kinds:
                k = "reanchor"
            elif "kf" in kinds:
                k = "kf"
            else:
                da, db = n[a] <= 60, n[b] <= 60
                k = "dense" if da and db else "corner" if not da and not db else "mixed"
            cls.setdefault(k, []).append(e[b] - e[a])
        tot = sum(float(np.sum(np.array(v) ** 2)) for v in cls.values())
        out["incr"] = {k: dict(count=len(v), rms_step_m=round(float(np.sqrt(np.mean(np.sum(np.array(v) ** 2, 1)))), 3),
                               sum_vec_m=np.round(np.sum(v, 0), 3).tolist(),
                               share_sq=round(float(np.sum(np.array(v) ** 2)) / tot, 3)) for k, v in cls.items()}
        # level split: cumulative per class from the first fix of the hold
        cum = {k: np.zeros(2) for k in cls}
        traj = {k: [] for k in cls}
        ks = list(cls)
        it = {k: iter(v) for k, v in cls.items()}
        order = []
        for a, b in zip(idx[:-1], idx[1:]):
            lo, hi = t[a], t[b]
            evt2 = [x for x in ev if lo < x[0] <= hi]
            kinds = {x[1] for x in evt2}
            if "reanchor" in kinds:
                k = "reanchor"
            elif "kf" in kinds:
                k = "kf"
            else:
                da, db = n[a] <= 60, n[b] <= 60
                k = "dense" if da and db else "corner" if not da and not db else "mixed"
            order.append(k)
            cum[k] = cum[k] + next(it[k])
            for kk in ks:
                traj[kk].append(cum[kk].copy())
        out["level_rms_m"] = {k: round(float(np.sqrt(np.mean(np.sum((np.array(v) - np.mean(v, 0)) ** 2, 1)))), 3)
                              for k, v in traj.items()}
        out["reanchors_in_hold"] = sum(1 for x in ev if x[1] == "reanchor" and 0 <= x[0] - te <= 60)
        out["kf_events_in_hold"] = sum(1 for x in ev if x[1] == "kf" and 0 <= x[0] - te <= 60)
        # high-frequency jitter: RMS of e minus its 1 s running mean
        k = 12
        if len(idx) > 2 * k:
            eh = e[idx]
            sm = np.array([eh[max(0, i - k // 2):i + k // 2 + 1].mean(0) for i in range(len(eh))])
            out["jitter_rms_m"] = round(float(np.sqrt(np.mean(np.sum((eh - sm) ** 2, 1)))), 3)
            out["slow_rms_m"] = round(float(np.sqrt(np.mean(np.sum((sm - sm.mean(0)) ** 2, 1)))), 3)
    # EKF position error at re-anchors (the anchor the program got): fix error of the first sent fix after it
    return out


def main():
    labels = sys.argv[1:]
    jobs = []
    if not labels:
        for js in sorted(glob.glob(os.path.join(RUNS, "*.json"))):
            lab = os.path.basename(js)[:-5]
            if lab.endswith(("_settings", "_lens")):
                continue
            try:
                c = json.load(open(js))["cfg"]
            except Exception:
                continue
            if c.get("video") == "JUSTICE_D" and c.get("variant") == "apb" and c.get("height") in (5, 10, 20, 30) \
                    and glob.glob(os.path.join(RUNS, lab + "_onboard*.csv")) and os.path.exists(os.path.join(RUNS, lab + ".npz")):
                labels.append(lab)
    for lab in labels:
        rows = []
        for p in sorted(glob.glob(os.path.join(RUNS, lab + "_onboard*.csv"))):
            rows += rows_of(p)
        rows.sort(key=lambda x: float(x["t"]))
        kf = os.path.join(RUNS, lab + "_kf.csv")
        c = json.load(open(os.path.join(RUNS, lab + ".json")))["cfg"]
        o = one(lab, rows, os.path.join(RUNS, lab + ".npz"), os.path.join(RUNS, lab + ".json"),
                kf if os.path.exists(kf) else None)
        o.update(height=c.get("height"), wind=c.get("wind"), ground=c.get("ground"), seed=c.get("seed"))
        print(json.dumps(o))


if __name__ == "__main__":
    main()
