#!/usr/bin/env python3
"""FC yaw error (ATTITUDE yaw minus the true yaw of the SIM_STATE truth log, deg) of the A/B runs of the copy 'ab', at
four times: the first sent fix of the flight, the switch to LOITER, 30 s and 60 s after the switch. With the flight time
from take-off (true height 0.3 m over its start) to each point, and the time from the first sent fix to the switch.
Versions: base, h15 (base + 15 s hover), pyb (py + (b)), pybh15 (py + (b) + 15 s hover).
Run: KF_CLOSED=<folder of the copies> ~/.venvs/vio/bin/python integ/kfmap/closed/scripts/yaw_table.py
Writes integ/kfmap/closed/yaw_table.txt and yaw_table.json."""
import csv
import glob
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
PROJ = os.path.dirname(os.path.dirname(os.path.dirname(OUT)))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import camera as C   # noqa: E402

RUNS = os.path.join(os.environ["KF_CLOSED"], "ab", "apsim", "runs")
VERS = ("base", "h15", "pyb", "pybh15")
NAME = {"base": "base", "h15": "base+hover15", "pyb": "py+(b)", "pybh15": "py+(b)+hover15"}


def one(label):
    d = np.load(os.path.join(RUNS, label + ".npz"))
    tr, att, te = d["truth"], d["att"], float(d["t_engage"])
    ye = np.degrees(np.unwrap((np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - C.q_yaw(tr[:, 4:]) + np.pi)
                              % (2 * np.pi) - np.pi))
    h = tr[:, 3]
    t_off = float(tr[np.argmax(h > h[0] + 0.3), 0])
    rows = []
    for p in sorted(glob.glob(os.path.join(RUNS, label + "_onboard*.csv"))):
        rows += [x for x in csv.DictReader(open(p)) if x["t"]]
    sent = sorted(float(x["t"]) for x in rows if x["sent"] == "1")
    t_fix = sent[0] if sent else None
    pts = {"first fix": t_fix, "switch": te, "+30 s": te + 30, "+60 s": te + 60}
    out = dict(label=label)
    for k, t in pts.items():
        out[k] = None if t is None or t > tr[-1, 0] else (round(float(np.interp(t, tr[:, 0], ye)), 1), round(t - t_off, 1))
    out["fix_to_switch"] = None if t_fix is None else round(te - t_fix, 1)
    out["h_switch"] = round(float(np.interp(te, tr[:, 0], h)), 1)
    w = (tr[:, 0] >= te) & (tr[:, 0] <= te + 60)
    out["max_abs_hold"] = round(float(np.abs(ye[w]).max()), 1)
    w2 = (tr[:, 0] >= (t_fix or te)) & (tr[:, 0] <= te + 60)
    out["max_abs_from_fix"] = round(float(np.abs(ye[w2]).max()), 1)
    return out


def main():
    res = []
    for p in sorted(glob.glob(os.path.join(RUNS, "lowap_apb_*.npz"))):
        lab = os.path.basename(p)[:-4]
        m = re.match(r"lowap_apb_(\d+)m_w20g40_s(\d+)_([a-z0-9]+)(?:_(ottawa2022|farmland))?$", lab)
        if not m or m.group(3) not in VERS:
            continue
        r = one(lab)
        r.update(h=int(m.group(1)), seed=int(m.group(2)), ver=m.group(3), ground=m.group(4) or "ottawa2019fall")
        res.append(r)
    res.sort(key=lambda r: (r["h"], r["ground"], r["seed"], VERS.index(r["ver"])))
    f = lambda x: "       -     " if x is None else f"{x[0]:+6.1f} @{x[1]:5.1f}s"   # noqa: E731
    lines = ["FC yaw error (deg) @ flight time from take-off (s). first fix = first sent fix of the flight.",
             f"{'h':>3s} {'ground':14s} {'seed':>4s} {'version':15s} {'first fix':>13s} {'switch':>13s} {'+30 s':>13s} "
             f"{'+60 s':>13s} {'fix->switch s':>13s} {'max |err| hold':>14s} {'max |err| from fix':>18s}"]
    for r in res:
        lines.append(f"{r['h']:3d} {r['ground']:14s} {r['seed']:4d} {NAME[r['ver']]:15s} {f(r['first fix'])} {f(r['switch'])} "
                     f"{f(r['+30 s'])} {f(r['+60 s'])} {r['fix_to_switch'] if r['fix_to_switch'] is not None else '-':>13} "
                     f"{r['max_abs_hold']:14.1f} {r['max_abs_from_fix']:18.1f}")
    lines += ["", __doc__.split("Run:")[0].strip()]
    open(os.path.join(OUT, "yaw_table.txt"), "w").write("\n".join(lines) + "\n")
    json.dump(res, open(os.path.join(OUT, "yaw_table.json"), "w"), indent=1)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
