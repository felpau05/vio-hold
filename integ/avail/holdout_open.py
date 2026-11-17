#!/usr/bin/env python3
"""Item 5, open loop: sim_ground.one (20 s hovers, sim_ground's own renderer) on the three HOLD-OUT sites
with HOLD-OUT seeds 101-106, pooled with sim_ground.pooled. Final numbers only; nothing is tuned on them.
Run: ~/.venvs/vio/bin/python integ/avail/holdout_open.py > integ/avail/holdout_open.txt"""
import multiprocessing as mp
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
SITES, HEIGHTS, SEEDS = ("holdout_quarry", "holdout_field", "holdout_water"), (100, 122), range(101, 107)


def job(a):
    import cv2
    cv2.setNumThreads(1)
    import sim_ground as G
    return G.one(*a)[0]


if __name__ == "__main__":
    import numpy as np
    import sim_ground as G
    jobs = [(s, h, sd) for s in SITES for h in HEIGHTS for sd in SEEDS]
    with mp.get_context("spawn").Pool(12) as pool:
        res = pool.map(job, jobs)
    print("HOLD-OUT, open loop (sim_ground renderer), 20 s hovers, seeds 101-106 pooled; rms/p95/worst in m")
    for s in SITES:
        for h in HEIGHTS:
            p = G.pooled([r for r, j in zip(res, jobs) if j[0] == s and j[1] == h])
            print(f"{s:15s} {h:4d} m  sent {p['used']:.1%}  rms {p['rms']:.2f}  p95 {p['p95']:.2f}  worst {p['worst']:.2f}  "
                  f"outside 99% {p['false_fix']:.1%}  rms per seed median {np.nanmedian(p['rms_seed']):.2f} worst {np.nanmax(p['rms_seed']):.2f}")
