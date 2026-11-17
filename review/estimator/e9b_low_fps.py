#!/usr/bin/env python3
"""Check 9, part 2: does the hold still work at the frame rate the i.MX8M Plus would reach with
the code as written (~5-6 fps on one A53 core, ~15-18 fps on four; see e9_compute.py)?
ottawa2019fall, sim_ground settings, 20 s, 2 seeds. LOST_AFTER stays 10 frames.
Run: ~/.venvs/vio/bin/python e9b_low_fps.py      (about 40 s)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S


def job(h0, fps, seed):
    C.reseed(seed)
    cfg = C.Cfg()
    pos, att, h = S.hover(h0)
    frames = C.make_frames("ottawa2019fall", pos, att, h, 20.0, cfg, fps=fps)
    a, g = C.fc_feeds(att, h, 20.0, cfg)
    rec = C.run(frames, a, g, pos, cfg)
    d = rec.summary()
    d.update(h=h0, fps=fps, seed=seed, n=float(np.median(rec.n[rec.sent])) if rec.sent.any() else 0)
    return d


if __name__ == "__main__":
    res = C.pmap(job, [dict(h0=h, fps=f, seed=s) for h in (100, 300) for f in (30.0, 15.0, 6.0) for s in (1, 2)])
    print(" h(m)  fps  seed  fixes sent  lost   RMS(m)  false-fix  kf changes  inliers")
    for r in res:
        print(f" {r['h']:4d}  {r['fps']:3.0f}   {r['seed']}     {r['sent']:5.0%}    {r['lost']:4.0%}   {r.get('rms', float('nan')):5.2f}"
              f"    {r.get('false_fix', float('nan')):5.1%}      {r['kf_switches']:4d}     {r['n']:5.0f}")
