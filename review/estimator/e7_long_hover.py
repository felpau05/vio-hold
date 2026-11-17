#!/usr/bin/env python3
"""Check 7: 120 s hover at 100 m and 200 m over ottawa2019fall with a slow 5 m wander.

Two FC-attitude cases:
  sim_ground   constant bias + white noise (as sim_test.fc_feeds)
  bias wander  plus a slow tilt-bias wander on roll and pitch (Gauss-Markov, sigma 0.2 deg,
               tau 30 s), like an EKF tilt estimate that drifts with temperature and manoeuvres.
               sim_ground has no such term.
Logs the error, keyframe changes and the sent 99 % bound; also the bound of honest.py
(SIG_BIAS = 0.2 deg, TAU 30 s for the wander case). Plot: e7_long_hover.png.
Run: ~/.venvs/vio/bin/python e7_long_hover.py     (about 150 s on 16 threads)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402
from honest import HonestHold  # noqa: E402

T = 120.0
S = C.S


def wander_hover(h0):
    pos0, att, h = S.hover(h0)

    def pos(t):
        t = np.asarray(t)
        return pos0(t) + np.c_[5.0 * np.sin(2 * np.pi * t / 60.0), 5.0 * np.sin(2 * np.pi * t / 45.0 + 1.0)]
    return pos, att, h


def gauss_markov(seed, sigma_deg=0.2, tau=30.0, dt=0.01, T=T):
    r = np.random.default_rng(1000 + seed)
    n = int((T + 3) / dt)
    a = math.exp(-dt / tau)
    b = np.zeros((n, 2))
    b[0] = r.normal(0, math.radians(sigma_deg), 2)
    for k in range(1, n):
        b[k] = a * b[k - 1] + math.radians(sigma_deg) * math.sqrt(1 - a * a) * r.normal(size=2)
    ts = np.arange(n) * dt - 1.5
    return lambda t: np.array([np.interp(t, ts, b[:, 0]), np.interp(t, ts, b[:, 1]), 0.0])


def job(h0, wander, seed):
    import cv2
    cv2.setNumThreads(2)              # 8 jobs x 2 threads: keeps the run under 3 minutes
    pos, att, h = wander_hover(h0)
    hold = HonestHold(S.K, S.D, S.R_BC_NADIR, S.READOUT)
    if wander:
        hold.SIG_BIAS, hold.TAU_BIAS = math.radians(0.2), 30.0
    rec, _ = C.scenario("ottawa2019fall", h0, T=T, seed=seed, pos_fn=pos, att_fn=att, h_fn=h,
                        att_bias_fn=gauss_markov(seed) if wander else None, hold=hold)
    s = rec.sent
    return dict(h=h0, wander=wander, seed=seed, t=rec.t[s].tolist(), err=rec.err[s].tolist(),
                vo=rec.var_old[s].tolist(), vn=rec.var[s].tolist(),
                sw=rec.t[(rec.kf_new == 1)].tolist()[1:], n=rec.n[s].tolist(), summ=rec.summary())


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    jobs = [dict(h0=h, wander=w, seed=s) for h in (100, 200) for w in (False, True) for s in (1, 2)]
    res = C.pmap(job, jobs)
    print(f"120 s hover, 5 m wander, ottawa2019fall. Error by 20 s window (mean m); false-fix old -> honest.py")
    print(" h(m)  FC attitude   seed  kf changes   0-20   20-40  40-60  60-80  80-100 100-120   RMS  "
          "inliers first/last 10 s  false-fix old -> new")
    for r in res:
        t, e = np.array(r["t"]), np.array(r["err"])
        win = [np.mean(e[(t >= a) & (t < a + 20)]) for a in range(0, 120, 20)]
        q_old = 2 * e ** 2 / np.array(r["vo"])
        q_new = 2 * e ** 2 / np.array(r["vn"])
        n = np.array(r["n"])
        print(f" {r['h']:4d}  {'bias wander' if r['wander'] else 'sim_ground ':11s}   {r['seed']}     {len(r['sw']):4d}     "
              + "  ".join(f"{w:5.2f}" for w in win)
              + f"   {r['summ']['rms']:5.2f}      {np.median(n[t < 10]):4.0f} / {np.median(n[t > 110]):4.0f}"
              f"            {np.mean(q_old > 9.21):5.1%} -> {np.mean(q_new > 9.21):5.1%}")
    fig, axs = plt.subplots(2, 2, figsize=(13, 7), sharex=True)
    for i, h in enumerate((100, 200)):
        for j, w in enumerate((False, True)):
            ax = axs[i, j]
            for r, col in zip([r for r in res if r["h"] == h and r["wander"] == w], ("#1f6feb", "#2ea043")):
                t, e = np.array(r["t"]), np.array(r["err"])
                ax.plot(t, e, color=col, lw=0.6, label=f"seed {r['seed']} error")
                ax.plot(t, np.sqrt(9.21 * np.array(r["vo"]) / 2), color=col, lw=1, ls=":",
                        label=f"seed {r['seed']} 99% bound, Hold var")
                ax.plot(t, np.sqrt(9.21 * np.array(r["vn"]) / 2), color=col, lw=1, ls="--",
                        label=f"seed {r['seed']} 99% bound, honest.py")
                for ts in r["sw"]:
                    ax.axvline(ts, color=col, alpha=0.3, lw=0.5)
            ax.set(title=f"{h} m, {'FC tilt-bias wander 0.2 deg' if w else 'sim_ground attitude'}",
                   ylabel="position error (m)")
            ax.grid(alpha=0.3)
            if i == 1:
                ax.set_xlabel("time (s)")
    axs[0, 0].legend(fontsize=7, ncol=2)
    fig.suptitle("120 s hover with 5 m wander (sim). Thin vertical lines: keyframe changes.", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(C.HERE, "e7_long_hover.png"), dpi=110)
    print("wrote e7_long_hover.png")
