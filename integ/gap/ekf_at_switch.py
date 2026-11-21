#!/usr/bin/env python3
"""Q3: the autopilot's position error at the switch (the Loiter target is the EKF position at that moment).

EKF error = ArduCopter's LOCAL_POSITION_NED (npz 'ekf') minus the true position turned into the EKF's heading frame
(ekf_yaw_minus_true_deg), with the constant frame offset removed: the mean of that difference over the 2 s before the
longest gap starts (fixes were sent then). Printed at the start and the end of the longest gap and at the switch,
with the true mean distance from the hold point over the 60 s hold (run json 'rms').
Run: ~/.venvs/vio/bin/python integ/gap/ekf_at_switch.py > integ/gap/results/ekf_at_switch.txt
"""
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gap_reasons import LABELS, RUNS  # noqa: E402


def main():
    print(__doc__.split("Run:")[0])
    print(f"{'run':30s} {'gap s':>5s} {'gap from/to s':>14s} {'EKF err gap start':>17s} {'gap end':>8s} {'switch':>7s} "
          f"{'switch+5s':>9s} {'hold RMS':>8s} {'hold max':>8s}")
    for lab in LABELS:
        d = np.load(os.path.join(RUNS, lab + ".npz"))
        r = json.load(open(os.path.join(RUNS, lab + ".json")))
        tr, ekf, te = d["truth"], d["ekf"], float(d["t_engage"])
        dy = math.radians(r["ekf_yaw_minus_true_deg"])
        c, s = math.cos(dy), math.sin(dy)
        N, E = np.interp(ekf[:, 0], tr[:, 0], tr[:, 1]), np.interp(ekf[:, 0], tr[:, 0], tr[:, 2])
        diff = ekf[:, 1:3] - np.c_[c * N - s * E, s * N + c * E]
        rows = [x for x in csv.DictReader(open(os.path.join(RUNS, lab + "_onboard.csv"))) if x["t"] and x["sent"] == "1"]
        ts = np.array([float(x["t"]) for x in rows])
        pre = ts[(ts > te - 10) & (ts < te + 3)]
        k = int(np.argmax(np.diff(pre)))
        lo, hi = pre[k], pre[k + 1]
        m = (ekf[:, 0] > lo - 2) & (ekf[:, 0] <= lo)
        err = diff - diff[m].mean(0)
        at = lambda t: float(np.hypot(np.interp(t, ekf[:, 0], err[:, 0]), np.interp(t, ekf[:, 0], err[:, 1])))  # noqa: E731
        print(f"{lab:30s} {hi - lo:5.2f} {lo - te:+6.2f}/{hi - te:+6.2f} {at(lo):17.2f} {at(hi):8.2f} {at(te):7.2f} "
              f"{at(te + 5):9.2f} {r['rms']:8.3f} {r['max']:8.3f}")


if __name__ == "__main__":
    main()
