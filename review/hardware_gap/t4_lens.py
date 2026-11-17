#!/usr/bin/env python3
"""Item 4: lens model wrong. Hover with 5 m wander, Ottawa photo ground.

True camera: the "Round 1 independent k2" profile (what sim_test uses). The model given to Hold has
fx, fy scaled, or the principal point moved by 10 / 30 px at 960x720 (= /2.67 in x, /3 in y at
360x240). Last rows: the true lens is a different profile in the same file (a Justice D with another
lens) while Hold keeps Round 1.

Run: ~/.venvs/vio/bin/python t4_lens.py       (about 2 minutes; 15 s runs)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S


def model(df=0.0, dfy=None, dcx=0.0, dcy=0.0):
    K = X.K0.copy()
    K[0, 0] *= 1 + df
    K[1, 1] *= 1 + (df if dfy is None else dfy)
    K[0, 2] += dcx * X.W / 960
    K[1, 2] += dcy * X.H / 720
    return K


def main():
    for h0 in (100, 200):
        g = X.Ground("ottawa2022", h0)
        pos, att_fn, h_fn = X.hover(h0, wander=5.0)
        frames = X.frames_for(pos, att_fn, h_fn, g, T=15.0, noise=5.0)
        att, hgt = X.fc_feeds(att_fn, h_fn, 15.0)
        tr = np.array([pos(np.array([t]))[0] for t, _ in frames])
        print(f"--- {h0} m, wander +-{np.abs(tr).max():.1f} m (max |N|,|E|)")
        cases = [("exact lens", model())]
        cases += [(f"focal {s:+d}%", model(s / 100)) for s in (2, -2, 5, -5, 10, -10)]
        cases += [("fy := fx (square pixels assumed)", model(0, dfy=X.K0[0, 0] / X.K0[1, 1] - 1))]
        cases += [(f"principal point +{d} px x and y @960", model(dcx=d, dcy=d)) for d in (10, 30)]
        for name, K in cases:
            res = X.score(X.run(frames, att, hgt, pos, K=K))
            print(f"  {name:38s} {X.fmt(res)}", flush=True)
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    pos, att_fn, h_fn = X.hover(h0, wander=5.0)
    att, hgt = X.fc_feeds(att_fn, h_fn, 15.0)
    print(f"--- {h0} m, true lens is another profile; Hold keeps 'Round 1 independent k2'")
    for prof in ("Nominal Justice", "Generic 132 deg"):
        Kt, Dt = X.lens(prof)
        rays = X.body_rays(Kt, Dt, X.W, X.H, X.R_BC_NADIR)
        frames = X.frames_for(pos, att_fn, h_fn, g, T=15.0, noise=5.0, rays=rays, fx=Kt[0, 0])
        res = X.score(X.run(frames, att, hgt, pos))
        print(f"  true '{prof}' (fx {Kt[0, 0] / X.K0[0, 0] - 1:+.0%} vs model) {X.fmt(res)}", flush=True)


if __name__ == "__main__":
    main()
