"""How steady is PX4 SITL's lockstep clock against wall time? From a run's npz (truth sample times and
their arrival times). Also: the error fc_link.Clock would have if the onboard process ran on wall time.
Run: ~/.venvs/vio/bin/python integ/clock_wobble.py <label>"""
import os
import sys

import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJ)
import fc_link  # noqa: E402

d = np.load(os.path.join(PROJ, "px4sim", "runs", sys.argv[1] + ".npz"))
t, a, te = d["truth"][:, 0], d["arr"], float(d["t_engage"])
o = a - t
b = np.floor(t).astype(int)
ub = np.unique(b)
bm = np.array([o[b == k].min() for k in ub])
bt = np.array([t[b == k][np.argmin(o[b == k])] for k in ub])
r = 1e3 * np.diff(bm) / np.diff(bt)
print(f"{sys.argv[1]}: sim clock {1 - np.median(r) / 1e3:.4f} x wall time; wall-minus-sim offset change "
      f"p5 {np.percentile(r, 5):.1f} p50 {np.median(r):.1f} p95 {np.percentile(r, 95):.1f} max {np.abs(r).max():.1f} ms per s")


def local(tq):
    i = np.searchsorted(t, tq)
    j0 = max(0, i - 12)
    return tq + np.min(a[j0:i + 1] - t[j0:i + 1])


for nb in (30, 10, 5):
    c, errs = fc_link.Clock(nbins=nb), []
    for k in range(0, len(t), 2):
        c.add(t[k], a[k])
        if t[k] > te and c.ready() and k % 25 == 0:
            errs.append(1e3 * (c.to_local(t[k]) - local(t[k])))
    e = np.array(errs)
    print(f"fc_link.Clock(nbins={nb}) on wall time during the hold: error mean {e.mean():.1f} ms, "
          f"p95 {np.percentile(np.abs(e), 95):.1f} ms, max {np.abs(e).max():.1f} ms")
