"""Failure cases in closed loop: what the drone did during and after the failure, and what the onboard
program sent. Run: ~/.venvs/vio/bin/python integ/failure_report.py <label> [t_fail dur]  (writes nothing)"""
import collections
import csv
import glob
import json
import math
import os
import sys

import numpy as np

RUNS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "px4sim", "runs")


def report(label, t_fail=20.0, dur=3.0):
    d = np.load(os.path.join(RUNS, label + ".npz"))
    r = json.load(open(os.path.join(RUNS, label + ".json")))
    tr, te, hp = d["truth"], float(d["t_engage"]), d["hold_pt"]
    t = tr[:, 0] - te
    m = (t >= 0) & (t <= 60)
    dist = np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    # horizontal speed from 0.2 s differences
    k = np.searchsorted(tr[:, 0], tr[:, 0] - 0.2)
    sp = np.hypot(tr[:, 1] - tr[k, 1], tr[:, 2] - tr[k, 2]) / np.maximum(tr[:, 0] - tr[k, 0], 1e-3)
    win = (t >= t_fail) & (t <= t_fail + dur + 10)
    pre = (t >= 0) & (t < t_fail)
    rows = []
    for p in sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv"))):
        rows += [x for x in csv.DictReader(open(p)) if x["t"]]
    fw = [x for x in rows if t_fail <= float(x["t"]) - te < t_fail + dur]
    sent_in = sum(x["sent"] == "1" for x in fw)
    back = [float(x["t"]) - te for x in rows if x["sent"] == "1" and float(x["t"]) - te >= t_fail + dur]
    whys = collections.Counter(x["why"].split(" (")[0] for x in fw if x["why"])
    notes = [x["note"] for x in rows if x["note"] and float(x["t"]) >= te]
    i_end = np.searchsorted(t, t_fail + dur)
    return (f"{label}: drift RMS {r.get('rms', math.nan):.2f} m, max {r.get('max', math.nan):.2f} m; before the failure: "
            f"max {dist[pre].max():.2f} m, speed max {sp[pre].max():.2f} m/s. Failure {t_fail:g}-{t_fail + dur:g} s: "
            f"fixes sent {sent_in} of {len(fw)} frames ({dict(whys.most_common(3))}); at its end {dist[i_end]:.2f} m; "
            f"next 10 s: max {dist[win].max():.2f} m, speed max {sp[win].max():.2f} m/s; first fix after it at "
            f"{(back[0] - t_fail - dur) if back else math.nan:.2f} s; re-anchors in the hold {len(notes)}; "
            f"end of hold {dist[m][-1]:.2f} m")


if __name__ == "__main__":
    a = sys.argv[1:]
    print(report(a[0], *(float(x) for x in a[1:3])))
