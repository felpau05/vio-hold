#!/usr/bin/env python3
"""Critical finding: the camera mount is a hard-coded guess, and a wrong guess makes a fly-away.

Closed loop with full images: an own renderer draws each frame for the TRUE mount, Hold (with the
code's mount, image top toward the nose) estimates the position, and a simple position controller
tilts the drone to hold the anchor. Only the true mount changes between runs.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t2_mount_closed_loop.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I          # noqa: E402
from hold import Hold      # noqa: E402

G, H0, T, RO, YAW = 9.81, 50.0, 12.0, 0.016, math.radians(30)
KP, KD, AMAX = 0.6, 1.4, 3.0
K, D = I.K_FIELD, I.D_FIELD
R_CODE = I.mount((1, 0, 0))            # equals R_BC_NADIR (checked in t0_conventions.py)
TEX = I.make_texture(seed=3)


def run(R_true, label):
    rnd = I.Renderer(K, D, R_true, I.W, I.H, TEX, 8 * H0)
    p, v = np.array([1.0, -0.5]), np.zeros(2)              # start 1.1 m off the hold point
    tilt = np.zeros(2)                                      # (roll, pitch) actually flown
    log_t, log_a = [-2.0, -1.0], [[0, 0, YAW], [0, 0, YAW]]
    hold = Hold(K, D, R_CODE, RO)
    hold.reanchor(p.copy())
    target = np.zeros(2)
    dt = 1 / I.FPS
    hist = []
    from hold import Series
    att, hgt = Series(1e9), Series(1e9)
    for t in np.arange(-2, 0, 0.005):
        att.add(t, 0.0, 0.0, YAW)
    hgt.add(-2.0, H0)
    hgt.add(T + 2, H0)
    for k in range(int(T * I.FPS)):
        t0 = k * dt
        # flight-controller attitude samples at 200 Hz up to the end of this frame (exact)
        ts = np.arange(t0, t0 + dt, 0.005)
        for ti in ts:
            att.add(ti, tilt[0], tilt[1], YAW)
        p_fn = lambda tt, p=p.copy(), v=v.copy(), t0=t0: p + np.outer(tt - t0, v)
        a_fn = lambda tt, r=tilt[0], q=tilt[1]: np.c_[np.full(len(tt), r), np.full(len(tt), q), np.full(len(tt), YAW)]
        h_fn = lambda tt: np.full(len(tt), H0)
        img = rnd.frame(t0, RO, p_fn, a_fn, h_fn)
        e = hold.process(img, t0, att, hgt)
        if e.state == "lost":
            hold.reanchor(e.pos)
        est, vest = e.pos, hold.vel
        a_cmd = -KP * (est - target) - KD * vest
        a_cmd *= min(1.0, AMAX / max(np.linalg.norm(a_cmd), 1e-9))
        # tilt needed for that acceleration, in body axes (nose toward YAW)
        c, s = math.cos(YAW), math.sin(YAW)
        ab = np.array([c * a_cmd[0] + s * a_cmd[1], -s * a_cmd[0] + c * a_cmd[1]])
        want = np.array([math.atan(ab[1] / G), -math.atan(ab[0] / G)])     # roll, pitch
        tilt += (want - tilt) * min(1.0, dt / 0.15)                        # 0.15 s attitude lag
        acc = G * np.array([-math.tan(tilt[1]), math.tan(tilt[0])])       # body accel from tilt
        acc = np.array([c * acc[0] - s * acc[1], s * acc[0] + c * acc[1]]) - 0.3 * v   # drag
        v = v + acc * dt
        p = p + v * dt
        hist.append((t0 + dt, *p, e.state, *est))
    d = np.array([np.hypot(h[1], h[2]) for h in hist])
    states = [h[3] for h in hist]
    marks = [int(s * I.FPS) - 1 for s in (2, 4, 6, 8, 10, 12)]
    print(f"  {label:34s} true distance from hold point at 2/4/6/8/10/12 s: "
          + " ".join(f"{d[m]:6.1f}" for m in marks) + " m   "
          f"(ok {states.count('ok')}, degraded {states.count('degraded')}, lost {states.count('lost')})")
    return d


print(f"Closed loop at {H0:.0f} m, start 1.1 m off, yaw {math.degrees(YAW):.0f} deg, code mount = image top toward nose")
for name, Rt in I.MOUNTS.items():
    run(Rt, name)
