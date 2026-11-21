#!/usr/bin/env python3
"""tests/run_regression.py E6 scene (100 m hover, ottawa2019fall, PAL, yaw swings of +-60 and +-90 deg at 0.05 Hz), hold.py
version 2 only, per seed, for the hold.py of a project copy. Used to find which part of the first try broke E6.
Run: ~/.venvs/vio/bin/python integ/lowh/e6_check.py /path/to/project_copy 1,3,11,12,13
(the copy's tests/run_regression.py and hold.py are imported; seeds 1 and 3 are those of the regression test itself)
"""
import sys, os, numpy as np
d = sys.argv[1]
sys.path[:0] = [os.path.join(d, 'tests'), d]
os.chdir(d)
import run_regression as RR
X, S, HM = RR.X, RR.S, RR.HM
assert os.path.dirname(HM.__file__) == d, HM.__file__
calls = []
orig = HM.Hold._dense
def dense(self, kf, img, idx, guess, J, p_lk):
    r = orig(self, kf, img, idx, guess, J, p_lk)
    calls.append((self.frames, None if isinstance(r, str) else 'ok', r if isinstance(r, str) else '', self.dq[1] if hasattr(self, 'dq') else None))
    return r
HM.Hold._dense = dense
for yaw in (60, 90):
    for seed in [int(x) for x in sys.argv[2].split(",")]:
        calls.clear()
        _, a, _ = S.hover(100)
        def att_fn(t, a=a, yaw=yaw):
            r = a(t); r[:, 2] = np.radians(yaw) * np.sin(2 * np.pi * 0.05 * np.asarray(t)); return r
        fr, att, hgt, pos = X.scene('ottawa2019fall', 100, 20.0, seed, att_fn=att_fn)
        h = RR.make(False)
        rec = X.run(fr, att, hgt, pos, hold=h, t_err=S.FRAME_T_ERR)
        s = RR.summ(rec)
        ok = sum(1 for c in calls if c[1]); 
        why = {}
        for c in calls:
            if not c[1]: why[c[2].split(' (')[0]] = why.get(c[2].split(' (')[0], 0) + 1
        e = rec.err[rec.sent]
        print(f"yaw {yaw} seed {seed}: rms {s['rms']:.2f} end {s['end']:.2f} sent {s['sent']:.2f} kfs {s['kfs']} dense calls {len(calls)} ok {ok} why {why}; err by 5 s: " +
              " ".join(f"{np.sqrt(np.mean(rec.err[rec.sent & (rec.t >= t) & (rec.t < t + 5)]**2)) if (rec.sent & (rec.t >= t) & (rec.t < t + 5)).any() else float('nan'):.2f}" for t in (0, 5, 10, 15)))
