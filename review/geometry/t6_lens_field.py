#!/usr/bin/env python3
"""Item 5: lens scaling to one analog field, field parity, wrong-profile guard, exposure timing.

(a) An analog field holds every other frame line, so field row r is frame line 2r (even field) or
    2r+1 (odd field). load_lens scales like an area resize, which puts field row r at frame line
    2r + 0.5. The two parities are therefore off by -/+0.25 field row. If apb_main feeds both
    fields (60 Hz) the image jumps by 0.5 row every frame.
(b) load_lens does not check "projection"; the "frame" (Brown) profile goes through cv2.fisheye.
(c) Timing: t0 must be the mid-exposure time of row 0; auto-exposure moves it.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t6_lens_field.py
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                                # noqa: E402
from common_hover import hover_path, run, rms    # noqa: E402
from hold import load_lens                       # noqa: E402

fy = I.K_FIELD[1, 1]
print(f"(a) field fy = {fy:.1f} px/rad. Parity offset 0.25 row = {0.25 / fy * 1e3:.2f} mrad; "
      f"jump between parities 0.5 row = {0.5 / fy * 1e3:.2f} mrad")
for h0 in (100, 200, 300):
    print(f"    h {h0:3d} m: 0.5-row parity jump = {h0 * 0.5 / fy:.2f} m along body x (if fields alternate)")
TEX = I.make_texture(seed=5)
for h0 in (100, 300):
    pos, att, hf = hover_path(h0)
    base, _ = run(h0, pos, att, hf, tex=TEX, T=6.0)
    alt, _ = run(h0, pos, att, hf, tex=TEX, T=6.0, cy_alt=0.25)
    e = np.array([[r[2], r[3]] for r in alt], float)
    jump = np.nanmedian(np.linalg.norm(np.diff(e, axis=0), axis=1))
    print(f"    full images, h {h0:3d} m: one parity rms {rms(base)[0]:.2f} m | alternating parities rms "
          f"{rms(alt)[0]:.2f} m, median frame-to-frame jump {jump:.2f} m")

print("(b) 'Round 2 Metashape Frame' is a Brown (frame) model; load_lens feeds it to cv2.fisheye anyway:")
for name in ("Round 2 independent k2", "Round 2 Metashape Frame"):
    p = I.lens_profile(name)
    K, D = load_lens(I.LENS_JSON, name, I.W, I.H)
    th = math.radians(40)
    if p["projection"] == "frame":       # true Brown model: r = tan(th) * (1 + k1 r^2 + k2 r^4 + k3 r^6)
        r = math.tan(th)
        rd = r * (1 + p["k1"] * r ** 2 + p["k2"] * r ** 4 + p["k3"] * r ** 6)
    else:
        rd = th * (1 + p["k1"] * th ** 2 + p["k2"] * th ** 4)
    u = K[0, 2] + K[0, 0] * rd
    n = cv2.fisheye.undistortPoints(np.array([[[u, K[1, 2]]]]), K, D).reshape(2)
    got = math.degrees(math.atan(n[0]))
    print(f"    {name:26s} projection={p['projection']:20s} pixel of a 40.0 deg ray -> load_lens/_rays says {got:5.1f} deg")

print("(c) constant timing error (t0 vs true mid-exposure of row 0), exact everything else:")
for h0 in (100, 300):
    pos, att, hf = hover_path(h0)
    for dt in (0.002, 0.005):
        out, _ = run(h0, pos, att, hf, tex=TEX, T=6.0, t0_shift=dt)
        print(f"    h {h0:3d} m  t0 off by {dt*1e3:.0f} ms: rms {rms(out)[0]:.2f} m  worst {rms(out)[2]:.2f} m")
