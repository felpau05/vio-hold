#!/usr/bin/env python3
"""Closed loop before and after the proposed patch: the gap (as integ/justice_d/fix_error.py: the longest time between
two sent fixes in [-10, +3] s of the hold start), the reasons inside it, the EKF error at the switch (as
ekf_at_switch.py) and the hold error (true distance from the hold point over the 60 s hold, RMS and max).
Before: the existing runs in apsim/runs. After: the runs of integ/gap/run_closed.sh in integ/gap/results/closed.
Run: ~/.venvs/vio/bin/python integ/gap/closed_summary.py > integ/gap/results/closed_summary.txt
"""
import csv
import glob
import json
import math
import os
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
BEFORE = os.path.join(PROJ, "apsim", "runs")
AFTER = os.path.join(HERE, "results", "closed")
PAIRS = [("jd_apb_100m_w20g40_s11", "gapfix_jd_apb_100m_w20g40_s11"), ("jd_apb_100m_w20g40_s12", "gapfix_jd_apb_100m_w20g40_s12"),
         ("jd_apb_100m_w20g40_s13", "gapfix_jd_apb_100m_w20g40_s13"), ("jdcal2_apb_100m_w20g40_s11", "gapfix_jdcal_apb_100m_w20g40_s11"),
         ("jdcal3_apb_100m_w20g40_s11", None), ("jdcal_apb_100m_w20g40_s11", None)]


def one(d_dir, lab, sitl_json):
    p = os.path.join(d_dir, lab + ".npz")
    if not os.path.exists(p):
        return None
    d = np.load(p)
    r = json.load(open(sitl_json))
    tr, ekf, te = d["truth"], d["ekf"], float(d["t_engage"])
    rows = [x for x in csv.DictReader(open(os.path.join(d_dir, lab + "_onboard.csv"))) if x["t"]]
    ts = np.array([float(x["t"]) for x in rows if x["sent"] == "1"])
    pre = ts[(ts > te - 10) & (ts < te + 3)]
    k = int(np.argmax(np.diff(pre)))
    lo, hi = pre[k], pre[k + 1]
    why = Counter(x["why"].split(" (")[0] for x in rows if lo < float(x["t"]) < hi)
    hold = Counter(x["why"].split(" (")[0] for x in rows if te + 3 < float(x["t"]) < te + 60 and x["sent"] != "1")
    dy = math.radians(r["ekf_yaw_minus_true_deg"])
    c, s = math.cos(dy), math.sin(dy)
    N, E = np.interp(ekf[:, 0], tr[:, 0], tr[:, 1]), np.interp(ekf[:, 0], tr[:, 0], tr[:, 2])
    diff = ekf[:, 1:3] - np.c_[c * N - s * E, s * N + c * E]
    m = (ekf[:, 0] > lo - 2) & (ekf[:, 0] <= lo)
    err = diff - diff[m].mean(0)
    at = lambda t: float(np.hypot(np.interp(t, ekf[:, 0], err[:, 0]), np.interp(t, ekf[:, 0], err[:, 1])))  # noqa: E731
    return dict(gap=hi - lo, lo=lo - te, hi=hi - te, why=dict(why), hold_unsent=dict(hold), ekf_switch=at(te),
                rms=r.get("rms"), max=r.get("max"), load=(r.get("load_1min_at_start"), r.get("load_1min_at_end")))


def main():
    print(__doc__.split("Run:")[0])
    for b, a in PAIRS:
        for tag, d_dir, lab, js in (("before", BEFORE, b, os.path.join(PROJ, "integ", "justice_d", "closed", b + ".json")),
                                    ("after", AFTER, a, os.path.join(AFTER, f"{a}.json") if a else None)):
            if lab is None:
                continue
            if not js or not os.path.exists(js):
                print(f"{tag:6s} {lab:36s} (not run)")
                continue
            x = one(d_dir, lab, js)
            if x is None:
                print(f"{tag:6s} {lab:36s} (no npz)")
                continue
            print(f"{tag:6s} {lab:36s} gap {x['gap']:.2f} s ({x['lo']:+.2f} to {x['hi']:+.2f}); EKF error at switch "
                  f"{x['ekf_switch']:.2f} m; hold RMS {x['rms']:.3f} max {x['max']:.3f} m; load {x['load']}; reasons in gap "
                  f"{x['why']}; unsent in hold +3..+60 s {x['hold_unsent']}")


if __name__ == "__main__":
    main()
