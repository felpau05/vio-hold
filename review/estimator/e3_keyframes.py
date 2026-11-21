#!/usr/bin/env python3
"""Check 3: keyframe changes. How often, why, what each one costs, and a calmer rule.

A) Switch count and trigger in a 20 s sim_ground hover, by site/height/yaw swing. Also: how often
   Hold.rate (from the noisy FC attitude) is below KF_RATE, i.e. how often the 'slow rotation'
   preference can act at all.
Re-written after the E6 fix: Hold itself now has the calmer rule (KF_GAP, KF_LOW, n_first), so the old
KfRuleHold comparison (C) and the forced-change random-walk (B, patched Hold._new_kf, which no longer
exists) are gone. What is left verifies the fix on the cases that used to switch.
Run: ~/.venvs/vio/bin/python e3_keyframes.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402
from hold import Hold  # noqa: E402


def yaw_hover(h0, yaw_deg):
    pos, att, h = C.S.hover(h0)

    def att2(t):
        a = att(t)
        a[:, 2] = math.radians(yaw_deg) * np.sin(2 * np.pi * 0.05 * np.asarray(t))
        return a
    return pos, att2, h


def job_A(site, h0, yaw, seed=1, T=20.0, rule=False):
    pos, att, h = yaw_hover(h0, yaw)
    hold = None
    rec, _ = C.scenario(site, h0, T=T, seed=seed, att_fn=att, hold=hold)
    sw = (rec.kf_new == 1)
    sw[0] = False
    sw &= rec.state != "init"                         # changes on a sent fix (not inits after a loss)
    low_n = int(np.sum(sw & (rec.n < 2 * Hold.MIN_PTS)))
    half = int(np.sum(sw & (rec.n >= 2 * Hold.MIN_PTS)))
    reinit = int(np.sum((rec.state == "init"))) - 1
    s = rec.sent
    return dict(site=site, h=h0, yaw=yaw, seed=seed, rule=rule, sw=int(sw.sum()), low_n=low_n, half=half,
                reinit=reinit, slow=float(np.mean(rec.rate < Hold.KF_RATE)), rate_med=float(np.median(rec.rate)),
                rms=rec.summary()["rms"], lost=rec.summary()["lost"], n=float(np.median(rec.n[s])),
                end=float(np.nanmean(rec.err[-30:])))


if __name__ == "__main__":
    A_jobs = [dict(site="ottawa2019fall", h0=h, yaw=10) for h in (50, 100, 200, 300, 400)]
    A_jobs += [dict(site="ottawa2019fall", h0=100, yaw=y) for y in (30, 60, 90)]
    A_jobs += [dict(site="prairie", h0=50, yaw=10, seed=s) for s in (1, 3)] + [dict(site="prairie", h0=100, yaw=10)]
    A = C.pmap(job_A, A_jobs)
    print("keyframe changes in a 20 s hover. Before the fix (FINDINGS E6): ottawa 100 m yaw 30/60/90 = 3/11/15 changes,"
          " RMS 0.82/1.24/2.10 m; prairie 50 m = 191-199 changes, RMS 1.3-1.9 m.")
    print(" site            h(m) yaw+- seed  changes  n<40  n<n0/2  re-inits lost  inliers  RMS(m)  end err(m)  rate<KF_RATE  median rate")
    for r in A:
        print(f" {r['site']:15s} {r['h']:4d}  {r['yaw']:3d}   {r['seed']}  {r['sw']:6d}  {r['low_n']:4d}  {r['half']:5d}   {r['reinit']:5d}  {r['lost']:4.0%}"
              f"   {r['n']:6.0f}   {r['rms']:5.2f}   {r['end']:6.2f}      {r['slow']:6.1%}      {r['rate_med']:.2f}")
