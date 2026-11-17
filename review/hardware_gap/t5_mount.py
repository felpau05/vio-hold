#!/usr/bin/env python3
"""Item 5: camera mounting differs from R_BC_NADIR. Ottawa photo ground, 100 m, open loop.

The true camera is rendered with a different camera-to-body rotation; Hold keeps R_BC_NADIR.
"gain" is the 2x2 least-squares map from true displacement to estimated displacement
(identity = correct; -I = every move reported backwards = positive feedback in a position loop).

Run: ~/.venvs/vio/bin/python t5_mount.py      (about 2 minutes; 15 s runs)
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

D = math.radians
TT = 15.0


def gain(rows):
    sent = [r for r in rows if r[2] is not None]
    if len(sent) < 20:
        return None
    E = np.array([r[3] for r in sent])
    Tt = np.array([r[4] for r in sent])
    E, Tt = E - E.mean(0), Tt - Tt.mean(0)
    A, *_ = np.linalg.lstsq(Tt, E, rcond=None)
    return A.T


def show(name, rows):
    res = X.score(rows)
    A = gain(rows)
    gs = "n/a" if A is None else f"[[{A[0, 0]:+.2f} {A[0, 1]:+.2f}] [{A[1, 0]:+.2f} {A[1, 1]:+.2f}]]"
    print(f"  {name:44s} {X.fmt(res)}  gain {gs}", flush=True)


def main():
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    for yaw in (10.0, 45.0):
        pos, att_fn, h_fn = X.hover(h0, yaw_deg=yaw)
        att, hgt = X.fc_feeds(att_fn, h_fn, TT)
        print(f"--- {h0} m hover, yaw swing +-{yaw:.0f} deg")
        cases = [("nominal mount", X.R_BC_NADIR), ("tilt 3 deg (about body x)", X.rot_x(D(3)) @ X.R_BC_NADIR)]
        if yaw == 45.0:
            cases += [("tilt 5 deg (about body x)", X.rot_x(D(5)) @ X.R_BC_NADIR)]
        if yaw == 10.0:
            cases += [("turned 5 deg about the optical axis", X.R_BC_NADIR @ X.rot_z(D(5))),
                      ("picture turned 90 deg", X.R_BC_NADIR @ X.rot_z(D(90))),
                      ("picture turned 180 deg", X.R_BC_NADIR @ X.rot_z(D(180)))]
        for name, R in cases:
            rays = X.body_rays(X.K0, X.D0, X.W, X.H, R)
            fr = X.frames_for(pos, att_fn, h_fn, g, T=TT, noise=5.0, rays=rays)
            show(name, X.run(fr, att, hgt, pos))
            if name == "nominal mount" and yaw == 10.0:
                show("picture mirrored left-right", X.run([(t, cv2.flip(im, 1)) for t, im in fr], att, hgt, pos))
    print(f"--- climb {h0} -> {h0 + 5 * TT:.0f} m in {TT:.0f} s (5 m/s), yaw +-10 deg")
    pos, att_fn, _ = X.hover(h0)
    h_fn = lambda t: h0 + 5.0 * np.asarray(t, float)
    att, hgt = X.fc_feeds(att_fn, h_fn, TT)
    for name, R in (("nominal mount", X.R_BC_NADIR), ("tilt 3 deg (about body x)", X.rot_x(D(3)) @ X.R_BC_NADIR)):
        rays = X.body_rays(X.K0, X.D0, X.W, X.H, R)
        fr = X.frames_for(pos, att_fn, h_fn, g, T=TT, noise=5.0, rays=rays)
        rows = X.run(fr, att, hgt, pos)
        sent = [r for r in rows if r[2] is not None]
        last = np.mean([np.linalg.norm(r[3] - r[4]) for r in sent[-30:]])
        show(name + f" (error over last 1 s {last:.2f} m)", rows)


if __name__ == "__main__":
    main()
