#!/usr/bin/env python3
"""Rolling-shutter direction: what if the sensor reads bottom-to-top (or the picture is flipped in memory)?

The code assumes row 0 (image top) is read first. Frames here are rendered with the bottom row read
first. Hold is then run four ways. Attitude and height are exact, so the timing model is the only error.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t4_readout_direction.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                                # noqa: E402
from common_hover import hover_path, run, rms    # noqa: E402

RO = 0.016
TEX = I.make_texture(seed=5)
top_row_time = RO * (I.H - 1) / I.H          # when reading upward, the top row is read last
print(f"Readout {RO*1e3:.0f} ms. Hover tilt wobble up to ~1 rad/s (sim_test-like).")
for h0 in (100, 300):
    pos, att, hf = hover_path(h0)
    rows = [
        ("true top-first, code top-first (correct)", dict(reverse=False, ro_est=RO)),
        ("true bottom-first, code top-first, t0 = first row read", dict(reverse=True, ro_est=RO)),
        ("true bottom-first, code global model, t0 = mid-frame", dict(reverse=True, ro_est=0.0, t0_shift=RO / 2)),
        ("true bottom-first, code readout = -16 ms, t0 = top row", dict(reverse=True, ro_est=-RO, t0_shift=top_row_time)),
    ]
    for label, kw in rows:
        out, _ = run(h0, pos, att, hf, tex=TEX, **kw)
        r, used, worst = rms(out)
        states = [o[1] for o in out]
        print(f"  h {h0:3d} m  {label:56s} rms {r:5.2f} m  worst {worst:5.2f} m  used {used:4.0%}  "
              f"degraded {states.count('degraded'):3d}/{len(states)}")
