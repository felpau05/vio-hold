#!/usr/bin/env python3
"""Item 1: what a frame-time error does to the hold (Ottawa photo ground, sim hover attitude).

Every sim result so far used a 2 ms stamp error (sim_test.FRAME_T_ERR). On the board the stamp error
is whatever the capture chain and the MAVLink clock mapping leave over. This script adds a constant
offset to the frame stamp, and a per-frame random error (frame stamped when read() returns).

Run: ~/.venvs/vio/bin/python t1_timing.py      (about 90 s)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S


def main():
    pos, att_fn, h_fn = X.hover(100)
    t = np.arange(0, 20, 0.0005)
    r = np.degrees(np.diff(att_fn(t), axis=0) / 0.0005)
    w_rms = np.sqrt((r[:, :2] ** 2).mean(0))
    print(f"sim hover attitude rate: roll/pitch RMS {w_rms[0]:.1f}/{w_rms[1]:.1f} deg/s, "
          f"peak {np.abs(r[:, 0]).max():.0f}/{np.abs(r[:, 1]).max():.0f} deg/s")
    for h0 in (100, 200):
        g = X.Ground("ottawa2022", h0)
        pos, att_fn, h_fn = X.hover(h0)
        frames = X.frames_for(pos, att_fn, h_fn, g, noise=5.0)
        att, hgt = X.fc_feeds(att_fn, h_fn, 20.0)
        print(f"--- {h0} m (offset added to the true first-row time; sim baseline already has +2 ms)")
        for d_ms in (0, 5, 10, 20, -20, 40):
            d = d_ms / 1000
            res = X.score(X.run(frames, att, hgt, pos, t_off=S.FRAME_T_ERR + d))
            pred = h0 * math.radians(math.hypot(*w_rms)) * abs(d)
            print(f"  stamp error {d_ms:+4d} ms: {X.fmt(res)}   (h*w_rms*dt = {pred:.2f} m)", flush=True)
        rng = np.random.default_rng(7)
        jit = {round(t0, 6): rng.uniform(-0.0167, 0.0167) for t0, _ in frames}
        res = X.score(X.run(frames, att, hgt, pos, t_off_fn=lambda t0: S.FRAME_T_ERR + jit[round(t0, 6)]))
        print(f"  random +-16.7 ms per frame (stamped at read()): {X.fmt(res)}", flush=True)


if __name__ == "__main__":
    main()
