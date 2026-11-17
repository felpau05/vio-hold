#!/usr/bin/env python3
"""Item 8 (and 3, 4): independent test of Hold._rays and of the keyframe/position solve.

My own quaternion attitude, my own mount (from "image top toward the nose"), my own fisheye model
and my own rolling-shutter projector put known ground points into pixels. Hold._rays then turns the
pixels back into camera-to-ground offsets. Nothing from the project's geometry (euler_R,
R_BC_NADIR, RAYS_B, render) is imported here; Hold uses its own euler_R internally (under test).

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t1_rays_independent.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I          # noqa: E402
from hold import Hold      # noqa: E402

deg = math.radians
H0, T0, RO = 100.0, 1.0, 0.016
K, D, H = I.K_FIELD, I.D_FIELD, I.H
R_ME = I.mount((1, 0, 0))                      # my own mount for "image top toward the nose"


def const(*v):
    return lambda t: np.tile(np.asarray(v, float), (len(np.atleast_1d(t)), 1))


def ground_points(pos_fn, att_fn, h_fn, R_bc, n=12):
    """Ground points that land inside the image: unproject a pixel grid at the mid-row pose."""
    tm = T0 + RO / 2
    uu, vv = np.meshgrid(np.linspace(15, I.W - 15, n), np.linspace(10, H - 10, n))
    rays = I.unproject_cam(uu.ravel(), vv.ravel(), K, D) @ R_bc.T
    q = I.att_quat(*np.asarray(att_fn(np.array([tm])), float)[0])
    b = I.qrot(q, rays)
    keep = b[:, 2] > math.cos(deg(50)) * np.linalg.norm(b, axis=1)
    b = b[keep]
    return pos_fn(np.array([tm]))[0] + h_fn(np.array([tm]))[0] * b[:, :2] / b[:, 2:3]


CASES = [
    ("level hover", const(0, 0), const(0, 0, 0)),
    ("pure roll +12 deg", const(0, 0), const(deg(12), 0, 0)),
    ("pure pitch +12 deg (nose up)", const(0, 0), const(0, deg(12), 0)),
    ("pure yaw +60 deg", const(0, 0), const(0, 0, deg(60))),
    ("roll 8, pitch -10, yaw 135", const(3, -2), const(deg(8), deg(-10), deg(135))),
    ("moving north 6 m/s", lambda t: np.c_[6 * t, 0 * t], const(0, deg(-8), 0)),
    ("moving east 6 m/s, yaw 40", lambda t: np.c_[0 * t, 6 * t], const(deg(8), 0, deg(40))),
    ("roll rate +1.5 rad/s", const(0, 0), lambda t: np.c_[1.5 * (t - T0), 0 * t, 0 * t]),
    ("pitch rate -1.5 rad/s + north 4 m/s", lambda t: np.c_[4 * t, 0 * t],
     lambda t: np.c_[0 * t, -1.5 * (t - T0), 0 * t]),
    ("yaw rate +1.5 rad/s, roll 6", const(0, 0), lambda t: np.c_[deg(6) + 0 * t, 0 * t, 1.5 * (t - T0)]),
]


def check(name, pos_fn, att_fn, R_true, R_code, ro_true, ro_code, reverse=False, verbose=True):
    h_fn = lambda t: np.full(len(np.atleast_1d(t)), H0)
    P = ground_points(pos_fn, att_fn, h_fn, R_true)
    px, t_true = I.project_ground(P, pos_fn, att_fn, h_fn, T0, ro_true, K, D, R_true, H, reverse=reverse)
    att, hgt = I.feeds(att_fn, h_fn, 3.0, att_hz=1000.0)
    hold = Hold(K, D, R_code, ro_code)
    g, t_code, ok, h = hold._rays(px.astype(np.float32), T0, H, att, hgt)
    C_true_i = pos_fn(t_true)
    g_true = P - C_true_i
    err_g = np.linalg.norm(g - g_true, axis=1)[ok]
    # Position solve exactly as process(): C = G - g + vel * (tref - t), with G known exactly.
    tref = T0 + ro_code / 2
    vel = (pos_fn(np.array([tref + 1e-3])) - pos_fn(np.array([tref - 1e-3])))[0] / 2e-3
    C = (P - g + vel * (tref - t_code)[:, None])[ok]
    pos_err = C.mean(0) - pos_fn(np.array([tref]))[0]
    # Keyframe side, exactly as _new_kf(): G = pos + vel * (t - tref) + g
    G = (pos_fn(np.array([tref]))[0] + vel * (t_code - tref)[:, None] + g)[ok]
    err_G = np.linalg.norm(G - P[ok], axis=1)
    if verbose:
        print(f"  {name:38s} pts {ok.sum():3d}  max|t err| {np.abs(t_code - t_true).max()*1e3:6.3f} ms  "
              f"max|g err| {err_g.max():7.4f} m  max|G err| {err_G.max():7.4f} m  "
              f"pos err N {pos_err[0]:+8.4f} E {pos_err[1]:+8.4f} m")
    return pos_err, err_g


print(f"A. Correct conventions (true = code): h = {H0} m, readout {RO*1e3:.0f} ms top-to-bottom")
worst = 0
for name, pf, af in CASES:
    e, eg = check(name, pf, af, R_ME, R_ME, RO, RO)
    worst = max(worst, eg.max())
print(f"  -> worst per-point offset error over all cases: {worst*100:.3f} cm (row times interpolated, so ~0 is expected)")

print("\nB. Same, camera physically turned 180/90 deg but R_bc updated to match: is t = t0 + row/rows*readout still right?")
for mname in ("top->tail (turned 180 deg)", "top->right wing (turned 90 deg)"):
    for name, pf, af in (CASES[7], CASES[8]):
        check(f"{mname[:14]} | {name}", pf, af, I.MOUNTS[mname], I.MOUNTS[mname], RO, RO)

print("\nC. Sensor reads BOTTOM-to-top, code assumes top-to-bottom (t0 = time of first row the code assumes)")
for name, pf, af in (CASES[7], CASES[8], CASES[9]):
    # true top row is read at T0 + RO*(H-1)/H when reading upward; give the code t0 = T0 (bottom read first)
    check("reversed | " + name, pf, af, R_ME, R_ME, RO, RO, reverse=True)
print("   ...and with readout = -RO and t0 = time of the top row (Hold accepts a negative readout):")
for name, pf, af in (CASES[7], CASES[8], CASES[9]):
    h_fn = lambda t: np.full(len(np.atleast_1d(t)), H0)
    P = ground_points(pf, af, h_fn, R_ME)
    px, t_true = I.project_ground(P, pf, af, h_fn, T0, RO, K, D, R_ME, H, reverse=True)
    att, hgt = I.feeds(af, h_fn, 3.0, att_hz=1000.0)
    hold = Hold(K, D, R_ME, -RO)
    t_top = T0 + RO * (H - 1) / H
    g, t_code, ok, _ = hold._rays(px.astype(np.float32), t_top, H, att, hgt)
    print(f"  {'neg readout | ' + name:38s} max|t err| {np.abs(t_code - t_true).max()*1e3:6.3f} ms  "
          f"max|g err| {np.linalg.norm(g - (P - pf(t_true)), axis=1)[ok].max():7.4f} m")

print("\nD. Wrong mount in the code (true mount differs from R_BC_NADIR-equivalent): offset of a pure 2 m move")
for mname, Rt in I.MOUNTS.items():
    rows = []
    for d in ((2.0, 0.0), (0.0, 2.0)):
        h_fn = lambda t: np.full(len(np.atleast_1d(t)), H0)
        a0 = const(0, 0, deg(20))
        P = ground_points(const(0, 0), a0, h_fn, Rt)
        gs = []
        for pf in (const(0, 0), const(*d)):
            px, _ = I.project_ground(P, pf, a0, h_fn, T0, 0.0, K, D, Rt, H)
            att, hgt = I.feeds(a0, h_fn, 3.0)
            g, _, ok, _ = Hold(K, D, R_ME, 0.0)._rays(px.astype(np.float32), T0, H, att, hgt)
            gs.append(g)
        rows.append((gs[0] - gs[1]).mean(0))         # estimated camera move = g_kf - g_now
    M = np.array(rows).T / 2.0
    print(f"  true {mname:34s} estimated-move matrix [[{M[0,0]:+.2f} {M[0,1]:+.2f}] [{M[1,0]:+.2f} {M[1,1]:+.2f}]]"
          f"  eig {np.round(np.linalg.eigvals(M), 2)}")
    # tilt: true +10 deg pitch, no move: reported change
    att_t = const(0, deg(10), deg(20))
    P = ground_points(const(0, 0), const(0, 0, deg(20)), lambda t: np.full(len(np.atleast_1d(t)), H0), Rt)
    gs = []
    for af in (const(0, 0, deg(20)), att_t):
        px, _ = I.project_ground(P, const(0, 0), af, lambda t: np.full(len(np.atleast_1d(t)), H0), T0, 0.0, K, D, Rt, H)
        att, hgt = I.feeds(af, lambda t: np.full(len(np.atleast_1d(t)), H0), 3.0)
        g, _, ok, _ = Hold(K, D, R_ME, 0.0)._rays(px.astype(np.float32), T0, H, att, hgt)
        gs.append(g[ok].mean(0) if ok.sum() > 3 else np.array([np.nan, np.nan]))
    print(f"  {'':39s} pitch +10 deg with no move -> reported move {np.linalg.norm(gs[0] - gs[1]):6.2f} m at {H0:.0f} m")
