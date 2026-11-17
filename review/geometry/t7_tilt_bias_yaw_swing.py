#!/usr/bin/env python3
"""A constant roll/pitch bias (FC level error or camera mount tilt b) does not cancel against the
keyframe once the drone yaws: error ~= h * b * 2 sin(d_yaw / 2). sim_test feeds the estimator
ATT_BIAS = (0.5, -0.3) deg roll/pitch with a +/-10 deg yaw swing, so part of its 300 m error is this
term, and a real mount tilt of 1-2 deg makes it larger. Nothing in Hold estimates the bias.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t7_tilt_bias_yaw_swing.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                                # noqa: E402
from common_hover import hover_path, run, rms    # noqa: E402

TEX = I.make_texture(seed=5)
T = 20.0
for h0 in (100, 300):
    for (rb, pb), yaw_amp in (((0.0, 0.0), 10.0), ((0.5, -0.3), 0.0), ((0.5, -0.3), 10.0), ((1.5, 0.0), 10.0)):
        pos, att, hf = hover_path(h0, yaw_amp_deg=yaw_amp)
        bias = lambda t, rb=rb, pb=pb: np.c_[np.full(len(t), math.radians(rb)), np.full(len(t), math.radians(pb)), 0 * t]
        out, _ = run(h0, pos, att, hf, tex=TEX, T=T, att_err_fn=bias)
        r, used, worst = rms(out)
        b = math.radians(math.hypot(rb, pb))
        dpsi = 2 * math.radians(yaw_amp)
        print(f"  h {h0:3d} m  roll/pitch bias ({rb:+.1f}, {pb:+.1f}) deg  yaw swing +/-{yaw_amp:4.1f} deg: "
              f"rms {r:5.2f} m  worst {worst:5.2f} m   (peak h*b*2sin(dpsi/2) = {h0 * b * 2 * math.sin(dpsi / 2):4.2f} m)")
