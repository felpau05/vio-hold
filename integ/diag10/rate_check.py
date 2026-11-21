#!/usr/bin/env python3
"""Do the ATTITUDE body rates agree with the change of the ATTITUDE angles (and of the true attitude)? Body rates from
consecutive rotation matrices, compared with the reported rates: best time lag and scale per axis, over the hold.
Run: ~/.venvs/vio/bin/python integ/diag10/rate_check.py <label>"""
import os, sys
import numpy as np
import cv2
HERE = os.path.dirname(os.path.abspath(__file__)); PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(PROJ, "px4sim")]
sys.dont_write_bytecode = True
import camera as C
import replay as RP
hm = RP.load_hold(os.path.join(HERE, "hold_base.py"), "hb")
lab = sys.argv[1]
d = np.load(os.path.join(PROJ, "apsim", "runs", lab + ".npz")); tr, att, te = d["truth"], d["att"], float(d["t_engage"])
a = att[(att[:, 0] > te) & (att[:, 0] < te + 60)]
R = hm.euler_R(a[:, 1], a[:, 2], a[:, 3])
w = np.array([cv2.Rodrigues(R[i].T @ R[i + 1])[0].ravel() / (a[i + 1, 0] - a[i, 0]) for i in range(len(a) - 1)])
tm = (a[1:, 0] + a[:-1, 0]) / 2
rep = a[:, 4:7]
for ax, nm in enumerate("pqr"):
    best = None
    for lag in np.arange(-0.04, 0.0401, 0.0025):
        x = np.interp(tm, a[:, 0] + lag, rep[:, ax])
        c = np.corrcoef(x, w[:, ax])[0, 1]
        if best is None or c > best[1]:
            best = (lag, c, np.polyfit(x, w[:, ax], 1)[0], np.mean(w[:, ax] - x))
    print(f"{lab} axis {nm}: reported rate lags the angle change by {best[0]*1e3:+.1f} ms (corr {best[1]:.3f}, scale {best[2]:.3f}, "
          f"mean diff {np.degrees(best[3]):+.3f} deg/s); rms rate {np.degrees(np.std(w[:, ax])):.1f} deg/s")
# truth body rates
i = (tr[:, 0] > te) & (tr[:, 0] < te + 60)
q = tr[i, 4:]; tt = tr[i, 0]
Rt = np.array([C.q_matrix(q[k:k + 1])[0] for k in range(len(q))])
wt = np.array([cv2.Rodrigues(Rt[k].T @ Rt[k + 1])[0].ravel() / (tt[k + 1] - tt[k]) for k in range(len(q) - 1)])
tmt = (tt[1:] + tt[:-1]) / 2
for ax, nm in enumerate("pqr"):
    best = None
    for lag in np.arange(-0.04, 0.0401, 0.0025):
        x = np.interp(tmt, a[:, 0] + lag, rep[:, ax])
        c = np.corrcoef(x, wt[:, ax])[0, 1]
        if best is None or c > best[1]:
            best = (lag, c, np.polyfit(x, wt[:, ax], 1)[0], np.mean(wt[:, ax] - x))
    print(f"  vs TRUE body rate, axis {nm}: lag {best[0]*1e3:+.1f} ms corr {best[1]:.3f} scale {best[2]:.3f} mean diff {np.degrees(best[3]):+.3f} deg/s")
