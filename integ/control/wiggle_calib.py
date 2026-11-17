#!/usr/bin/env python3
"""Video delay (and readout) from the take-off wiggle of one apsim run, with calib.py's own search.

  wiggle_calib.py LABEL      the run must have wiggle_s > 0 and settings raw_record true, log_dir
                             apsim/runs/LABEL_raw (the onboard program's raw chunks: frames as Hold gets them,
                             the FC's ATTITUDE in the onboard clock)
The truth is the camera the simulator rendered (case readout_s, exposure_s, video_delay_s), not the program's
setting, which the case can make wrong by est_delay_err_s.
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, PROJ)
import calib  # noqa: E402
from hold import make_R_bc  # noqa: E402

RUNS = os.path.join(PROJ, "apsim", "runs")


def solve(label):
    r = json.load(open(os.path.join(RUNS, label + ".json")))
    s = json.load(open(os.path.join(RUNS, label + "_settings.json")))
    t0, t1 = r["wiggle_t"]
    imgs, tg, att = [], [], []
    for p in sorted(glob.glob(os.path.join(RUNS, label + "_raw", "raw_*.npz"))):
        d = np.load(p)
        m = (d["t_grab"] >= t0 + 0.5) & (d["t_grab"] <= t1)
        imgs.append(d["imgs"][m])
        tg.append(d["t_grab"][m])
        att.append(d["att"])
        K, D = d["K"], d["D"]
    att = np.concatenate(att)
    att = att[np.unique(att[:, 0], return_index=True)[1]]
    rec = dict(imgs=np.concatenate(imgs), t_grab=np.concatenate(tg), att=att, K=K, D=D)
    c = r["cfg"] if "cfg" in r else {}
    rr = calib.Rec(rec)
    R = make_R_bc(s["top"], s["mirror"], tuple(s["tilt_deg"]), s["twist_deg"])
    lat, ro, res = rr.timing(R)
    true = s["video_delay_s"] - c.get("est_delay_err_s", 0.0)
    return dict(label=label, frames=int(len(rr.tg)), matches=int(len(rr.ia)), rate_dps=round(rr.rate_dps, 1),
                found_delay_ms=round(lat * 1e3, 1), true_delay_ms=round(true * 1e3, 1),
                setting_ms=round(s["video_delay_s"] * 1e3, 1), error_ms=round((lat - true) * 1e3, 1),
                found_readout_ms=round(ro * 1e3, 1), true_readout_ms=round(s["readout_s"] * 1e3, 1),
                resid_deg=round(res, 3))


if __name__ == "__main__":
    for a in sys.argv[1:]:
        print(json.dumps(solve(a)))
