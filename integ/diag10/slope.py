#!/usr/bin/env python3
"""Scale of the measured motion: slope of fix increments on true increments over windows of w s (per axis), for a
replay result (integ/diag10/results/*.json.gz) or a SITL run's onboard log (apsim/runs/<label>). Slope 1 = the fix
follows the true motion; under 1 = the fix under-reports the motion. All sent fixes in [0, 60] s after the switch.
Run: ~/.venvs/vio/bin/python integ/diag10/slope.py <file.json.gz | label> ..."""
import csv, glob, gzip, json, math, os, sys
import numpy as np
PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def slopes(t, p, q, ws=(1.0, 3.0)):
    out = {}
    for w in ws:
        dp, dq = [], []
        for i in range(len(t)):
            j = np.searchsorted(t, t[i] + w)
            if j < len(t) and abs(t[j] - t[i] - w) < 0.1:
                dp.append(p[j] - p[i]); dq.append(q[j] - q[i])
        dp, dq = np.array(dp), np.array(dq)
        if len(dp) < 10:
            continue
        out[w] = [round(float(np.polyfit(dq[:, a], dp[:, a], 1)[0]), 3) for a in (0, 1)] + \
                 [round(float(np.sqrt(np.mean(np.sum((dp - dq) ** 2, 1)))), 3)]
    return out


for arg in sys.argv[1:]:
    if arg.endswith(".json.gz"):
        r = json.load(gzip.open(arg))
        rows = [x for x in r["rows"] if x["sent"] and 0 <= x["t"] <= 60]
        lab = r["flight"]
        d = np.load(os.path.join(PROJ, "apsim", "runs", lab + ".npz"))
        tr, te = d["truth"], float(d["t_engage"])
        att = d["att"]
        import sys as _s
        _s.path.insert(0, os.path.join(PROJ, "px4sim"))
        import camera as C
        ty = C.q_yaw(tr[:, 4:])
        dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
        t = np.array([x["t"] for x in rows]) + 0.007
        e = np.array([x["err"] for x in rows])
        c, s = math.cos(dy), math.sin(dy)
        N, E = np.interp(t + te, tr[:, 0], tr[:, 1]), np.interp(t + te, tr[:, 0], tr[:, 2])
        q = np.c_[c * N - s * E, s * N + c * E]
        p = q + e
        print(os.path.basename(arg)[:-8], slopes(t, p, q))
    else:
        d = np.load(os.path.join(PROJ, "apsim", "runs", arg + ".npz"))
        js = json.load(open(os.path.join(PROJ, "apsim", "runs", arg + ".json")))
        tr, te = d["truth"], float(d["t_engage"])
        dy, ro = math.radians(js["ekf_yaw_minus_true_deg"]), js["cfg"]["readout_s"]
        rows = []
        for f in sorted(glob.glob(os.path.join(PROJ, "apsim", "runs", arg + "_onboard*.csv"))):
            rows += [x for x in csv.DictReader(open(f)) if x["t"] and x["sent"] == "1"]
        rows.sort(key=lambda x: float(x["t"]))
        t = np.array([float(x["t"]) for x in rows]) + ro / 2
        m = (t >= te) & (t <= te + 60)
        p = np.array([[float(x["north"]), float(x["east"])] for x in rows])
        c, s = math.cos(dy), math.sin(dy)
        N, E = np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])
        q = np.c_[c * N - s * E, s * N + c * E]
        print(arg, "height", js["cfg"]["height"], slopes(t[m], p[m], q[m]))
