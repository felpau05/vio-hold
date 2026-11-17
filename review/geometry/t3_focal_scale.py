#!/usr/bin/env python3
"""High finding: a focal-length (K scale) error gives a position error of about h * eps * (tilt change).

Both simulators render and estimate with the same K, so they cannot show it. Here the frames are
rendered with the calibrated K and Hold gets K with fx, fy scaled by (1 + eps). Attitude and height
feeds are exact, and the rolling-shutter model is exact, so the only error is the K scale.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t3_focal_scale.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                       # noqa: E402
from common_hover import hover_path, run, rms   # noqa: E402

TEX = I.make_texture(seed=5)
print("Hover with a +/-6 deg tilt wobble; exact attitude, height and row timing; only K differs.")
for h0 in (100, 300):
    pos, att, hf = hover_path(h0)
    a = att(np.arange(0, 10, 0.01))
    tilt_rms = math.degrees(math.sqrt(np.mean(a[:, 0] ** 2 + a[:, 1] ** 2)))
    for eps in (0.0, 0.01, 0.025, -0.025):
        Ke = I.K_FIELD.copy()
        Ke[0, 0] *= 1 + eps
        Ke[1, 1] *= 1 + eps
        out, _ = run(h0, pos, att, hf, K_est=Ke, tex=TEX)
        r, used, worst = rms(out)
        pred = h0 * abs(eps) * math.radians(tilt_rms) * math.sqrt(2)   # rms of (tilt - tilt at keyframe)
        print(f"  h {h0:3d} m  eps {eps:+6.1%}  rms {r:5.2f} m  worst {worst:5.2f} m  used {used:4.0%}"
              f"   (h*eps*tilt_change rough guess {pred:4.2f} m; tilt rms {tilt_rms:.1f} deg)")
