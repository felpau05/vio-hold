#!/usr/bin/env python3
"""Check 3: keyframe changes. How often, why, what each one costs, and a calmer rule.

A) Switch count and trigger in a 20 s sim_ground hover, by site/height/yaw swing. Also: how often
   Hold.rate (from the noisy FC attitude) is below KF_RATE, i.e. how often the 'slow rotation'
   preference can act at all.
B) Random walk: force a keyframe change every k frames (ottawa2019fall, 5 seeds x 8 s) and
   measure the drift of the fix error from the first to the last second against the number of
   changes. Slope = error variance each change adds.
C) A calmer rule (KfRuleHold below): switch only when inliers fall under half of what this
   keyframe gave on its first tracked frame (not half of the corner count n0), or under
   MIN_PTS + 10; and at most once per second unless forced. Compared on the cases that switch.
Run: ~/.venvs/vio/bin/python e3_keyframes.py     (about 140 s on 16 threads)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402
from hold import Hold  # noqa: E402


class KfRuleHold(Hold):
    MIN_GAP = 1.0

    def process(self, img, t0, att, hgt):
        had = self.kf is not None
        self._block = had
        try:
            e = super().process(img, t0, att, hgt)
        finally:
            self._block = False
        if had and self.kf is not None and e.var is not None:
            kf = self.kf
            kf.setdefault("n_first", e.n)
            gap = e.t - getattr(self, "t_switch", -math.inf)
            if e.n < self.MIN_PTS + 10 or (e.n < 0.5 * kf["n_first"] and gap > self.MIN_GAP):
                if Hold._new_kf(self, img, t0, att, hgt):
                    self.t_switch = e.t
        return e

    def _new_kf(self, *a):
        if getattr(self, "_block", False):
            return False
        return super()._new_kf(*a)


def yaw_hover(h0, yaw_deg):
    pos, att, h = C.S.hover(h0)

    def att2(t):
        a = att(t)
        a[:, 2] = math.radians(yaw_deg) * np.sin(2 * np.pi * 0.05 * np.asarray(t))
        return a
    return pos, att2, h


def job_A(site, h0, yaw, seed=1, T=20.0, rule=False):
    pos, att, h = yaw_hover(h0, yaw)
    hold = KfRuleHold(C.S.K, C.S.D, C.S.R_BC_NADIR, C.S.READOUT) if rule else None
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


def job_B(h0, k, seed, T=8.0):
    cnt = [0]

    def post(i, hold, e, img, t, att, hgt):
        if k and e.var is not None and i % k == 0 and hold.kf is not None:
            hold._new_kf(img, t, att, hgt)
            cnt[0] += 1
    rec, _ = C.scenario("ottawa2019fall", h0, T=T, seed=seed, post=post)
    s = rec.sent
    ex, ey = rec.ex, rec.ey
    d = np.array([np.nanmean(ex[-30:]) - np.nanmean(ex[1:31]), np.nanmean(ey[-30:]) - np.nanmean(ey[1:31])])
    hm = h0
    loc = rec.var - rec.kf_var - (Hold.ATT_NOISE * hm) ** 2
    keep = s & (rec.kf_new == 0) & (loc > 0)
    return dict(h=h0, k=k, seed=seed, nsw=int(np.sum(rec.kf_new)) - 1, d2=float(d @ d),
                spread_n=float(np.median(loc[keep])) if keep.any() else np.nan, n=float(np.median(rec.n[s])))


if __name__ == "__main__":
    A_jobs = [dict(site="ottawa2019fall", h0=h, yaw=10) for h in (50, 100, 200, 300, 400)]
    A_jobs += [dict(site="ottawa2019fall", h0=100, yaw=y) for y in (30, 60, 90)]
    A_jobs += [dict(site="prairie", h0=h, yaw=10) for h in (50, 100)]
    C_jobs = [dict(site="prairie", h0=50, yaw=10, seed=s, rule=r) for s in (1, 3) for r in (False, True)]
    C_jobs += [dict(site="ottawa2019fall", h0=100, yaw=90, seed=s, rule=r) for s in (1, 3) for r in (False, True)]
    B_jobs = [dict(h0=h, k=k, seed=s) for h in (100, 300) for k in (0, 30, 10, 3, 1) for s in range(1, 6)]
    res = C.pmap(job_A, A_jobs + C_jobs)
    A, Cr = res[:len(A_jobs)], res[len(A_jobs):]
    B = C.pmap(job_B, B_jobs)

    print("A) keyframe changes in a 20 s hover (seed 1). trigger: 'n<40' = inliers under 2*MIN_PTS,")
    print("   'n<n0/2' = inliers under half the keyframe's corner count while Hold.rate < KF_RATE")
    print(" site            h(m) yaw+-  changes  n<40  n<n0/2  re-inits  inliers  RMS(m)  end err(m)  "
          "frames with rate<KF_RATE  median rate(rad/s)")
    for r in A:
        print(f" {r['site']:15s} {r['h']:4d}  {r['yaw']:3d}    {r['sw']:5d}  {r['low_n']:4d}  {r['half']:5d}   {r['reinit']:5d}"
              f"   {r['n']:6.0f}   {r['rms']:5.2f}   {r['end']:6.2f}        {r['slow']:6.1%}              {r['rate_med']:.2f}")

    print("\nB) forced keyframe changes every k frames, ottawa2019fall, 5 seeds x 8 s")
    print(" h(m)  k   changes  drift RMS first->last s (m)  per-change sigma (m)  spread^2/n (m^2)  inliers")
    for h in (100, 300):
        base = None
        for k in (0, 30, 10, 3, 1):
            rs = [r for r in B if r["h"] == h and r["k"] == k]
            nsw = np.mean([r["nsw"] for r in rs])
            d2 = np.mean([r["d2"] for r in rs])
            if k == 0:
                base = d2
            per = math.sqrt(max(d2 - base, 0) / nsw) if nsw > 0 else float("nan")
            print(f" {h:4d} {k:3d}  {nsw:7.0f}   {math.sqrt(d2):6.2f}                      {per:6.3f}"
                  f"               {np.mean([r['spread_n'] for r in rs]):.4f}          {np.mean([r['n'] for r in rs]):.0f}")

    print("\nC) Hold's rule vs the calmer rule (KfRuleHold), 20 s")
    print(" site            h(m) yaw+-  seed  rule     changes  re-inits lost  inliers  RMS(m)  end err(m)")
    for r in Cr:
        print(f" {r['site']:15s} {r['h']:4d}  {r['yaw']:3d}    {r['seed']}   {'calmer' if r['rule'] else 'Hold  '}   {r['sw']:5d}"
              f"    {r['reinit']:4d}  {r['lost']:4.0%}  {r['n']:6.0f}   {r['rms']:5.2f}   {r['end']:6.2f}")
