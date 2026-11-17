#!/usr/bin/env python3
"""Hold.rate (max |Euler-angle rate| from the FC attitude in a window around the frame) against
KF_RATE = 0.3 rad/s: how much of it is attitude noise, how much is the sim's hover motion.
No rendering; same formula as Hold._check_inputs. Run: ~/.venvs/vio/bin/python e3b_rate_noise.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S


def rates(att, ts, readout=S.READOUT):
    a = att.array()
    out = []
    for t0 in ts:
        t1 = t0 + readout
        w = a[(a[:, 0] > t0 - 0.03) & (a[:, 0] < t1 + 0.03)]
        r = np.diff(np.c_[w[:, 1], w[:, 2], np.unwrap(w[:, 3])], axis=0) / np.diff(w[:, 0])[:, None]
        out.append(np.abs(r).max())
    return np.array(out)


if __name__ == "__main__":
    ts = np.arange(0, 20, 1 / 30)
    pos, att_fn, h_fn = S.hover(100)
    calm = lambda t: np.radians(np.c_[1 * np.sin(2 * np.pi * 0.3 * np.asarray(t)), 1 * np.sin(2 * np.pi * 0.25 * np.asarray(t) + 1),
                                      np.zeros(len(np.atleast_1d(t)))])
    print(" attitude profile          FC noise(deg)  median rate(rad/s)  frames with rate < KF_RATE 0.3")
    for name, fn in (("sim hover (+-6 deg)", att_fn), ("calm hover (+-1 deg)", calm)):
        for noise in (0.1, 0.05, 0.0):
            C.reseed(1)
            att, _ = C.fc_feeds(fn, h_fn, 20.0, C.Cfg(att_noise=math.radians(noise)))
            r = rates(att, ts)
            print(f" {name:24s}   {noise:4.2f}          {np.median(r):5.2f}              {np.mean(r < 0.3):6.1%}")
