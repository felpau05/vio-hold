#!/usr/bin/env python3
"""Check 1, part 2: the proposed variance model (honest.py) on runs it was NOT tuned on.

SIG_TH = 0.13 deg was read off e1_variance.py (seeds 1-6, ottawa2019fall). Here: other seeds
(31-42), and other sites. Same estimate as Hold; only the variance differs. For each group:
  false-fix  = share of fixes outside the 99 % bound (ideal 1 %)
  out-50     = share outside the 50 % bound (ideal 50 %; much lower = too cautious)
  NEES       = mean of e^2 / per-axis var (ideal 2 for 2 axes)
Also: prairie 50 m (about 10 keyframe changes per second) and +-60 deg yaw swings.
Run: ~/.venvs/vio/bin/python e1b_honest_var.py      (about 120 s on 16 threads)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402
from honest import HonestHold  # noqa: E402


def yaw_att(h0, yaw_deg):
    _, att, _ = C.S.hover(h0)

    def f(t):
        a = att(t)
        a[:, 2] = np.radians(yaw_deg) * np.sin(2 * np.pi * 0.05 * np.asarray(t))
        return a
    return f


def job(site, h0, seed, T, yaw=10):
    hold = HonestHold(C.S.K, C.S.D, C.S.R_BC_NADIR, C.S.READOUT)
    rec, _ = C.scenario(site, h0, T=T, seed=seed, hold=hold, att_fn=yaw_att(h0, yaw))
    s = rec.sent
    return dict(site=site, h=h0, yaw=yaw, seed=seed, e2=rec.err[s] ** 2, vo=rec.var_old[s], vn=rec.var[s])


def stats(e2, var_trace):
    q = 2 * e2 / var_trace            # = e^2 / per-axis variance
    return np.mean(q > 9.21), np.mean(q > 1.386), np.mean(q)


if __name__ == "__main__":
    groups = [("ottawa2019fall", h, 10, 12, 8.0) for h in (100, 200, 300, 400)]
    groups += [(st, h, 10, 8, 8.0) for st, h in (("ottawa2022", 200), ("suburb", 300), ("farmland", 150), ("prairie", 100))]
    # many keyframe changes (low detail) and large yaw swings: the terms Hold lacks
    groups += [("prairie", 50, 10, 6, 16.0), ("ottawa2019fall", 100, 60, 6, 16.0), ("ottawa2019fall", 200, 60, 6, 16.0)]
    jobs = [dict(site=st, h0=h, seed=k, T=T, yaw=y) for st, h, y, n, T in groups for k in range(31, 31 + n)]
    res = C.pmap(job, jobs)
    print("Held-out seeds. old = Hold's var; new = honest.py (SIG_TH 0.13 deg, anchor + frame + switch + yaw terms)")
    print(" site            h(m) yaw+-  runs  RMS(m) | old: false-fix out-50  NEES | new: false-fix out-50  NEES")
    for st, h, y, n, T in groups:
        rs = [r for r in res if r["site"] == st and r["h"] == h and r["yaw"] == y]
        e2 = np.concatenate([r["e2"] for r in rs])
        a = stats(e2, np.concatenate([r["vo"] for r in rs]))
        b = stats(e2, np.concatenate([r["vn"] for r in rs]))
        print(f" {st:15s} {h:4d}  {y:3d}   {len(rs):4d}   {np.sqrt(e2.mean()):5.2f}  |      {a[0]:6.1%}  {a[1]:6.1%} {a[2]:5.2f} |"
              f"      {b[0]:6.1%}  {b[1]:6.1%} {b[2]:5.2f}")
    e2 = np.concatenate([r["e2"] for r in res])
    a = stats(e2, np.concatenate([r["vo"] for r in res]))
    b = stats(e2, np.concatenate([r["vn"] for r in res]))
    print(f" ALL                                {np.sqrt(e2.mean()):5.2f}  |      {a[0]:6.1%}  {a[1]:6.1%} {a[2]:5.2f} |"
          f"      {b[0]:6.1%}  {b[1]:6.1%} {b[2]:5.2f}")
    print(" (ideal: false-fix 1 %, out-50 50 %, NEES 2.0)")
