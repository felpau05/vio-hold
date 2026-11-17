#!/usr/bin/env python3
"""A constant tilt error between camera and FC attitude (camera-mount misalignment, or FC level
bias) does not cancel when the drone yaws. The keyframe stores G with the tilt error pointing one
way; after a yaw change dpsi it points another way, and the fix is off by 2 h beta sin(dpsi/2).
Hold's variance has no term for it, and nothing in the code estimates it.

Test: ottawa2019fall, beta = 1 deg roll bias in the FC feed (the same as a 1 deg camera mount
error), all other sim errors as sim_ground except the default ATT_BIAS (set to 0 so beta is the
only bias). The drone yaws by dpsi at 1 rad/s starting at t = 3 s, then holds. Error = mean over
the last 2 s, against the prediction.
Run: ~/.venvs/vio/bin/python e7b_mount_yaw.py     (about 40 s)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S


def job(h0, beta, dpsi):
    pos, att, h = S.hover(h0)

    def att2(t):
        a = att(t)
        a[:, 2] = np.clip(1.0 * (np.asarray(t) - 3.0), 0, math.radians(dpsi))
        return a
    cfg = C.Cfg(att_bias=np.radians([beta, 0.0, 0.0]))
    rec, _ = C.scenario("ottawa2019fall", h0, T=10.0, cfg=cfg, att_fn=att2)
    last = rec.sent & (rec.t > 8.0)
    e = np.array([np.mean(rec.ex[last]), np.mean(rec.ey[last])])
    q = 2 * rec.err[last] ** 2 / rec.var[last]
    return dict(h=h0, beta=beta, dpsi=dpsi, err=float(np.linalg.norm(e)),
                pred=2 * h0 * math.radians(beta) * math.sin(math.radians(dpsi) / 2),
                sw=int(np.sum(rec.kf_new)) - 1, ff=float(np.mean(q > 9.21)),
                sig=float(np.sqrt(np.mean(rec.var[last]) / 2)))


if __name__ == "__main__":
    jobs = [dict(h0=h, beta=b, dpsi=d) for h in (100, 200) for b in (0.0, 1.0) for d in (30, 90, 180)]
    res = C.pmap(job, jobs)
    print(" h(m)  mount/tilt error(deg)  yaw change(deg)  error after turn(m)  predicted 2h.beta.sin(dpsi/2)(m)  "
          "kf changes  sent sigma/axis(m)  false-fix after")
    for r in res:
        print(f" {r['h']:4d}         {r['beta']:3.1f}                {r['dpsi']:3d}            {r['err']:6.2f}"
              f"                 {r['pred']:6.2f}                       {r['sw']:3d}          {r['sig']:5.2f}          {r['ff']:5.1%}")
