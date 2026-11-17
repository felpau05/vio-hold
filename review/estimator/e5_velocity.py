#!/usr/bin/env python3
"""Check 5: the velocity filter  vel = 0.7 vel + 0.3 (pos - pos_prev)/dt  and its uses
(row-time compensation in the fix, new-keyframe G, jump-gate prediction).

A) ottawa2019fall, 3 seeds x 10 s per height. Modes: Hold as is; vel forced to 0 before each frame;
   vel forced to the TRUE velocity (oracle). Velocity noise = RMS(|vel - true vel|).
B) Camera blackout of 3 s while the drone moves at 1 m/s (t = 4..9 s), sim_ground behaviour (no
   re-anchor: Hold restarts from its own last fix with its stale vel and stale t) and sim_test
   behaviour (caller re-anchors from truth + 0.4 m). Does it come back, and how far off?
Run: ~/.venvs/vio/bin/python e5_velocity.py     (about 60 s)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S


def vtrue(pos_fn, t, d=1e-3):
    return (pos_fn(np.array([t + d]))[0] - pos_fn(np.array([t - d]))[0]) / (2 * d)


def job_A(h0, seed, mode):
    pos, att, h = S.hover(h0)

    def hook(i, hold):
        if mode == "vel=0":
            hold.vel = np.zeros(2)
        elif mode == "vel=true" and hold.t is not None:
            hold.vel = vtrue(pos, hold.t)
    vn = []

    def post(i, hold, e, img, t, a, g):
        if e.var is not None:
            vn.append(np.linalg.norm(hold.vel - vtrue(pos, e.t)))
    rec, _ = C.scenario("ottawa2019fall", h0, T=10.0, seed=seed, hook=hook, post=post)
    s = rec.sent
    dx, dy = np.diff(rec.px[s]), np.diff(rec.py[s])
    tx, ty = np.diff(rec.tx[s]), np.diff(rec.ty[s])
    jit = math.sqrt(np.mean((dx - tx) ** 2 + (dy - ty) ** 2) / 2)
    return dict(h=h0, seed=seed, mode=mode, rms=rec.summary()["rms"], jit=jit,
                vnoise=float(np.sqrt(np.mean(np.square(vn)))) if mode == "Hold" else np.nan,
                vtrue=float(np.sqrt(np.mean([np.sum(vtrue(pos, t) ** 2) for t in rec.t[s]]))))


def job_B(h0, reanchor, seed=1):
    pos0, att, h = S.hover(h0)

    def pos(t):
        t = np.asarray(t)
        return pos0(t) + np.c_[np.clip(t - 4.0, 0, 5.0), np.zeros(len(np.atleast_1d(t)))]
    dark = (5.0, 8.0)

    def mod(t0, img):
        if dark[0] <= t0 < dark[1]:
            return np.clip(np.full_like(img, 90) + S.rng.normal(0, 5, img.shape), 0, 255).astype(np.uint8)
        return img
    rec, _ = C.scenario2("ottawa2019fall", h0, 14.0, pos_fn=pos, att_fn=att, h_fn=h, img_mod=mod, seed=seed,
                         reanchor_on_lost=np.array([0.4, -0.3]) if reanchor else None)
    after = rec.t >= dark[1]
    s = rec.sent
    first_ok = rec.t[after & s]
    whys = {}
    for w in rec.why[after]:
        if w:
            k = w if not w[0].isdigit() else w.split(" ", 1)[1]
            whys[k] = whys.get(k, 0) + 1
    vstale = float(np.linalg.norm([rec.vx[rec.t < dark[0]][-1], rec.vy[rec.t < dark[0]][-1]]))
    return dict(h=h0, re=reanchor, back=(first_ok[0] - dark[1]) if len(first_ok) else np.nan,
                sent_after=float(np.mean(s[after])), err_after=float(np.nanmean(rec.err[after & s])) if (after & s).any() else np.nan,
                err_end=float(rec.err[-1]), whys=whys, vstale=vstale,
                lost_dark=float(np.mean(rec.state[(rec.t > dark[0] + 0.5) & (rec.t < dark[1])] == "lost")),
                sent_dark=int(np.sum(s[(rec.t > dark[0] + 0.5) & (rec.t < dark[1])])))


if __name__ == "__main__":
    A_jobs = [dict(h0=h, seed=s, mode=m) for h in (100, 200, 400) for s in (1, 2, 3) for m in ("Hold", "vel=0", "vel=true")]
    B_jobs = [dict(h0=h, reanchor=r) for h in (20, 50, 100) for r in (False, True)]
    res = C.pmap(job_A, A_jobs)
    resB = C.pmap(job_B, B_jobs)
    print("A) velocity term: ottawa2019fall, 3 seeds x 10 s (true drone speed RMS shown)")
    print(" h(m)  mode       RMS(m)  frame jitter per axis(m)  vel noise RMS(m/s)  true speed RMS(m/s)")
    for h in (100, 200, 400):
        for m in ("Hold", "vel=0", "vel=true"):
            rs = [r for r in res if r["h"] == h and r["mode"] == m]
            print(f" {h:4d}  {m:9s}  {np.sqrt(np.mean([r['rms']**2 for r in rs])):5.2f}    {np.mean([r['jit'] for r in rs]):6.3f}"
                  f"                   {np.nanmean([r['vnoise'] for r in rs]) if m == 'Hold' else float('nan'):6.2f}"
                  f"              {np.mean([r['vtrue'] for r in rs]):5.2f}")
    print("\nB) 3 s blackout at t = 5 s while moving 1 m/s north (t = 4..9 s)")
    print(" h(m)  after loss          vel at loss(m/s)  sent in dark  back after(s)  sent after  mean err after(m)  "
          "err at end(m)  why not sent after")
    for r in resB:
        print(f" {r['h']:4d}  {'re-anchor (sim_test)' if r['re'] else 'own pos (sim_ground)':20s}  {r['vstale']:5.2f}"
              f"            {r['sent_dark']:3d}          {r['back']:5.2f}        {r['sent_after']:5.0%}       "
              f"{r['err_after']:6.2f}           {r['err_end']:6.2f}       {r['whys']}")
