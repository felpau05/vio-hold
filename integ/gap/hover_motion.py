#!/usr/bin/env python3
"""What moves in the still hover before the hold start of each closed-loop run (the input of the liveness test).

For [-8, 0] s of the hold start: the change of roll, pitch and yaw over 1 s windows (median and max, deg) for
  true   - the simulator's true attitude (SIM_STATE quaternion)
  fc     - ArduCopter's ATTITUDE
  hold   - what Hold gets: ATTITUDE through apb_main.att_filter (GyroAtt, att_tau_s 20 s)
plus the GyroAtt yaw minus the FC yaw (the filter's lean toward the FC: (FC - ours) / tau per second), the true
horizontal speed and the true height change. Picture scale: f = 318.5 px (calibrated lens at 640x360), so 0.1 deg
of tilt = 0.56 px at the centre, and a yaw turn of 0.1 deg moves a point 300 px off centre by 0.52 px.
Run: ~/.venvs/vio/bin/python integ/gap/hover_motion.py > integ/gap/results/hover_motion.txt
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import apb_main     # noqa: E402
import camera as C  # noqa: E402
import settings as S  # noqa: E402
from gap_reasons import LABELS, RUNS  # noqa: E402


def main():
    print(__doc__.split("Run:")[0])
    print(f"{'run':30s} {'axis':5s} {'true 1s med/max':>16s} {'fc 1s med/max':>15s} {'hold 1s med/max':>16s}")
    for lab in LABELS:
        d = np.load(os.path.join(RUNS, lab + ".npz"))
        tr, att, te = d["truth"], d["att"], float(d["t_engage"])
        s = S.load(os.path.join(RUNS, lab + "_settings.json"))
        f = apb_main.att_filter(s)
        filt = np.array([f(tuple(r))[:4] for r in att[::2]]) if f else att[::2, :4]
        r, p = C.q_roll_pitch(tr[:, 4:])
        y = C.q_yaw(tr[:, 4:])
        tt = np.arange(te - 8, te + 0.001, 0.1)

        def series(t, cols):
            return [np.interp(tt, t, np.degrees(np.unwrap(c))) for c in cols]
        true = series(tr[:, 0], (r, p, y))
        fc = series(att[:, 0], (att[:, 1], att[:, 2], att[:, 3]))
        ho = series(filt[:, 0], (filt[:, 1], filt[:, 2], filt[:, 3]))
        for k, ax in enumerate(("roll", "pitch", "yaw")):
            def mm(v):
                d1 = np.abs(v[10:] - v[:-10])
                return f"{np.median(d1):.3f}/{d1.max():.3f}"
            print(f"{lab:30s} {ax:5s} {mm(true[k]):>16s} {mm(fc[k]):>15s} {mm(ho[k]):>16s}")
        dyaw = ho[2] - fc[2]
        N, E, H = (np.interp(tt, tr[:, 0], tr[:, k]) for k in (1, 2, 3))
        v = np.hypot(np.diff(N), np.diff(E)) / 0.1
        print(f"{lab:30s} GyroAtt yaw - FC yaw: {dyaw[0]:+.2f} deg at -8 s, {dyaw[-1]:+.2f} deg at 0 s; true horizontal "
              f"speed median {np.median(v):.3f} m/s (1 s mean max {np.convolve(v, np.ones(10) / 10, 'valid').max():.3f}); "
              f"true height change -8..0 s {H[-1] - H[0]:+.2f} m, last 4 s {H[-1] - H[-41]:+.2f} m")


if __name__ == "__main__":
    main()
