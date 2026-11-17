#!/usr/bin/env python3
"""Analysis of apsim runs for the control work (integ/control).

  an.py lag LABEL...      ArduPilot's own timing of our fixes (dataflash VISP: CTimeMS - RTimeUS) against
                          the program's VISO_DELAY_MS advice from its log (apb_main.delay_advice)
  an.py err LABEL...      fix error against the truth: white part (fix-to-fix), slow part, and the error of
                          speed from fix differences (apb_main.Speed windows)
  an.py tab PREFIX...     RMS / max / tilt wobble of the runs whose label starts with PREFIX, middle and worst
The truth comes from SIM_STATE (apsim/runs/LABEL.npz), not from the program.
"""
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(PROJ, "apsim", "runs")
sys.path.insert(0, PROJ)


def onboard_rows(label):
    p = os.path.join(RUNS, label + "_onboard.csv")
    return [r for r in csv.DictReader(open(p))] if os.path.exists(p) else []


def lag(label):
    from pymavlink import mavutil
    import apb_main
    b = os.path.join(HERE, "bin", label + ".BIN")
    out = dict(label=label)
    if os.path.exists(b):
        m = mavutil.mavlink_connection(b)
        d, viso = [], None
        while True:
            x = m.recv_match(type=["VISP", "PARM"])
            if x is None:
                break
            if x.get_type() == "PARM":
                if x.Name == "VISO_DELAY_MS":
                    viso = x.Value
                continue
            d.append(x.CTimeMS - x.RTimeUS / 1000.0)
        d = np.array(d)
        if len(d):
            # the stamp is the capture time on the onboard clock = sim time (onboard.py), the FC's boot clock
            out.update(fc_lag_ms_p10=float(np.percentile(d, 10)), fc_lag_ms_p50=float(np.median(d)),
                       fc_lag_ms_p90=float(np.percentile(d, 90)), viso_delay_ms=viso,
                       timing_err_ms_rms=float(np.sqrt(np.mean((d - (viso or 0)) ** 2))) if viso is not None else None)
    a = [float(r["age_ms"]) for r in onboard_rows(label) if r["sent"] == "1" and r["age_ms"]]
    if len(a) >= 100:
        out.update(advice_ms=apb_main.delay_advice(a), age_p50=float(np.median(a)))
    return out


def truth(label):
    z = np.load(os.path.join(RUNS, label + ".npz"))
    r = json.load(open(os.path.join(RUNS, label + ".json")))
    return z, r


def err(label, windows=(0.3, 0.4, 0.6)):
    z, r = truth(label)
    tr = z["truth"]
    t0 = float(z["t_engage"])
    yaw = np.radians(r.get("ekf_yaw_minus_true_deg", 0.0))
    rows = [x for x in onboard_rows(label) if x["var"] and float(x["t"]) >= t0]
    if len(rows) < 50:
        return dict(label=label, n=len(rows))
    t = np.array([float(x["t"]) for x in rows])
    p = np.array([[float(x["north"]), float(x["east"])] for x in rows])
    cs, sn = np.cos(yaw), np.sin(yaw)
    N, E = np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])
    q = np.c_[cs * N - sn * E, sn * N + cs * E]                   # truth in the FC's heading frame
    e = p - q
    e = e - e.mean(0)
    k = 25                                                        # about 2 s of fixes
    slow = np.array([e[max(0, i - k):i + k + 1].mean(0) for i in range(len(e))])
    out = dict(label=label, n=len(rows), err_rms_axis=float(np.sqrt(np.mean(e ** 2))),
               white_sd_axis=float(np.sqrt(np.mean((e - slow) ** 2))), slow_sd_axis=float(np.sqrt(np.mean(slow ** 2))),
               var_reported_sd_axis=float(np.sqrt(np.median([float(x["var"]) for x in rows]) / 2)))
    vt = np.gradient(q, axis=0) / np.gradient(t)[:, None]          # true speed at the fix times
    for w in windows:
        ve = []
        for i in range(len(t)):
            m = (t > t[i] - w) & (t <= t[i])
            if m.sum() < 4 or t[m][-1] - t[m][0] < w / 2:
                continue
            tt = t[m] - t[m].mean()
            v = tt @ (p[m] - p[m].mean(0)) / (tt @ tt)
            tm = t[m].mean()
            vtrue = np.array([np.interp(tm, t, vt[:, 0]), np.interp(tm, t, vt[:, 1])])
            ve.append(v - vtrue)
        ve = np.array(ve)
        out[f"speed_err_sd_w{w}"] = float(np.sqrt(np.mean(ve ** 2))) if len(ve) else None
    return out


def tab(prefixes):
    res = json.load(open(os.path.join(PROJ, "apsim", "results.json")))
    lines = []
    for pre in prefixes:
        ks = sorted(k for k in res if k.startswith(pre))
        rows = [res[k] for k in ks if isinstance(res[k].get("rms"), (int, float))]
        for k in ks:
            r = res[k]
            lines.append(f"  {k:44s} rms {r.get('rms', float('nan')):.2f} max {r.get('max', float('nan')):.2f} "
                         f"roll {r.get('roll_rms_deg', float('nan')):.2f} pitch {r.get('pitch_rms_deg', float('nan')):.2f} "
                         f"fixHz {r.get('fix_rate_hz') or float('nan'):.1f} age {r.get('age_ms_p50') or float('nan'):.0f} "
                         f"{'; '.join(str(e[1]) for e in r.get('events', [])[1:])[:80]}")
        if rows:
            rm, mx = np.array([r["rms"] for r in rows]), np.array([r["max"] for r in rows])
            tilt = np.array([np.hypot(r["roll_rms_deg"], r["pitch_rms_deg"]) for r in rows])
            lines.append(f"{pre}: n {len(rows)}  RMS middle {np.median(rm):.2f} worst {rm.max():.2f} | max middle "
                         f"{np.median(mx):.2f} worst {mx.max():.2f} | tilt wobble middle {np.median(tilt):.2f} deg")
    return "\n".join(lines)


if __name__ == "__main__":
    what, args = sys.argv[1], sys.argv[2:]
    if what == "tab":
        print(tab(args))
    else:
        for a in args:
            print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in
                              (lag(a) if what == "lag" else err(a)).items()}))
