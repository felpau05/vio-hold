#!/usr/bin/env python3
"""Item 3: where the calm-air drift comes from, from a closed-loop run's own logs (no new flight).
Fix error = sent fix minus the truth (PX4 heading frame, offset at the first fix removed), split into a
slow part (1 s moving mean) and frame jitter; gaps between sent fixes; reported sigma; tilt-source error.
Run: ~/.venvs/vio/bin/python integ/avail/calm_why.py px4sim/runs/apb_100m_calm"""
import csv, json, math, sys
import numpy as np
sys.path.insert(0, "px4sim")
import camera as C

base = sys.argv[1]
d = np.load(base + ".npz")
tr, att, te = d["truth"], d["att"], float(d["t_engage"])
rows = [r for p in [base + "_onboard.csv"] for r in csv.DictReader(open(p)) if r["t"]]
t = np.array([float(r["t"]) for r in rows]); sent = np.array([r["sent"] == "1" for r in rows])
N = np.array([float(r["north"] or "nan") for r in rows]); E = np.array([float(r["east"] or "nan") for r in rows])
V = np.array([float(r["var"] or "nan") for r in rows])
sel = sent & (t >= te) & (t <= te + 60)
ty = C.q_yaw(tr[:, 4:])
dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
tn, te_ = np.interp(t[sel], tr[:, 0], tr[:, 1]), np.interp(t[sel], tr[:, 0], tr[:, 2])
c, s = math.cos(dy), math.sin(dy)
P = np.c_[c * tn - s * te_, s * tn + c * te_]
err = np.c_[N[sel], E[sel]] - P
err -= err[:20].mean(0)
k = 25
slow = np.array([np.convolve(err[:, i], np.ones(k) / k, mode="same") for i in (0, 1)]).T
fast = err - slow
gaps = np.diff(t[sel])
# truth drift over the hold, and PX4 roll/pitch vs true roll/pitch
m = (tr[:, 0] >= te) & (tr[:, 0] <= te + 60)
r_t, p_t = C.q_roll_pitch(tr[m, 4:])
am = (att[:, 0] >= te) & (att[:, 0] <= te + 60)
out = dict(case=base.split("/")[-1], fixes=int(sel.sum()), fix_hz=float(sel.sum() / 60),
           err_rms_axis=[float(np.sqrt(np.mean(err[:, i] ** 2))) for i in (0, 1)],
           slow_rms_axis=[float(np.sqrt(np.mean(slow[:, i] ** 2))) for i in (0, 1)],
           jitter_rms_axis=[float(np.sqrt(np.mean(fast[k:-k, i] ** 2))) for i in (0, 1)],
           sigma_reported_median=float(np.sqrt(np.nanmedian(V[sel]) / 2)),
           gap_p50_ms=1e3 * float(np.median(gaps)), gap_p99_ms=1e3 * float(np.percentile(gaps, 99)),
           gap_max_ms=1e3 * float(gaps.max()), gaps_over_0p5s=int(np.sum(gaps > 0.5)),
           whys={w: int(sum(1 for r in rows if te <= float(r["t"]) <= te + 60 and r["why"].split(" (")[0] == w))
                 for w in sorted({r["why"].split(" (")[0] for r in rows if r["why"]})},
           true_roll_rms_deg=float(np.degrees(np.std(r_t))), true_pitch_rms_deg=float(np.degrees(np.std(p_t))),
           px4_minus_true_roll_rms_deg=float(np.degrees(np.std(np.interp(att[am, 0], tr[m, 0], r_t) - att[am, 1]))),
           px4_minus_true_pitch_rms_deg=float(np.degrees(np.std(np.interp(att[am, 0], tr[m, 0], p_t) - att[am, 2]))))
print(json.dumps(out))
