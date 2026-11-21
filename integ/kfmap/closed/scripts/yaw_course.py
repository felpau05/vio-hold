#!/usr/bin/env python3
"""FC yaw error (ATTITUDE yaw minus the true yaw, deg) over time for A/B runs of the copy 'ab', every 5 s from -20 s to
60 s after the switch, with the FC texts about yaw. Run: KF_CLOSED=... python yaw_course.py <label> [...]"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(HERE))))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import camera as C   # noqa: E402

runs = os.path.join(os.environ["KF_CLOSED"], "ab", "apsim", "runs")
for lab in sys.argv[1:]:
    d = np.load(os.path.join(runs, lab + ".npz"))
    tr, att, te = d["truth"], d["att"], float(d["t_engage"])
    t = tr[:, 0] - te
    ye = np.degrees(np.unwrap((np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - C.q_yaw(tr[:, 4:]) + np.pi) % (2 * np.pi) - np.pi))
    print(lab, " ".join(f"{x:+.0f}:{np.interp(x, t, ye):+.1f}" for x in range(-20, 61, 5)))
