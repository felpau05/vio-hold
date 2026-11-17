"""Estimator error apart from the autopilot: sent fix minus the true position (turned into PX4's heading frame),
less its median over the first 3 s of the hold (the anchor offset). fixerr.py LABEL ... (px4sim/runs/LABEL.npz).
Prints the fix error RMS / worst (cm) next to the drift RMS / worst (cm)."""
import json
import math
import os
import sys

import numpy as np

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "px4sim", "runs")


def fixerr(lab):
    d = np.load(os.path.join(RUNS, lab + ".npz"))
    j = json.load(open(os.path.join(RUNS, lab + ".json")))
    yo = math.radians(j["ekf_yaw_minus_true_deg"])
    tr, te, est = d["truth"], float(d["t_engage"]), d["est"]
    s = est[(est[:, 0] >= te) & (est[:, 7] == 1)]
    if len(s) < 10:
        return None
    T = np.c_[np.interp(s[:, 0], tr[:, 0], tr[:, 1]), np.interp(s[:, 0], tr[:, 0], tr[:, 2])]
    R = np.array([[math.cos(yo), -math.sin(yo)], [math.sin(yo), math.cos(yo)]])
    e = s[:, 1:3] - T @ R.T
    e -= np.median(e[s[:, 0] < te + 3], axis=0) if (s[:, 0] < te + 3).sum() > 3 else e[:5].mean(0)
    r = np.hypot(e[:, 0], e[:, 1])
    return dict(fix_rms_cm=100 * float(np.sqrt(np.mean(r * r))), fix_max_cm=100 * float(r.max()),
                drift_rms_cm=100 * j.get("rms", math.nan), drift_max_cm=100 * j.get("max", math.nan), n_fix=len(s))


if __name__ == "__main__":
    for lab in sys.argv[1:]:
        r = fixerr(lab)
        print(f"{lab:28s} " + ("no fixes" if r is None else
              f"fix err {r['fix_rms_cm']:6.1f} / {r['fix_max_cm']:6.1f} cm   drift {r['drift_rms_cm']:6.1f} / {r['drift_max_cm']:6.1f} cm   fixes {r['n_fix']}"))
