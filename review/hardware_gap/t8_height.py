#!/usr/bin/env python3
"""Item 7: barometric height errors.

Part A (Ottawa photo, hover with 5 m wander): height scale error, a terrain offset, and a slow
drift (1 m/min) seen 5 minutes into the hold.
Part B (synthetic texture, true height 3 m, drone moves 2 m east): what Hold does when the
baro height is near zero or negative, as it is on the ground and in ground effect at take-off.

Run: ~/.venvs/vio/bin/python t8_height.py      (about 2 minutes)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S


def part_a():
    for h0 in (100, 200):
        g = X.Ground("ottawa2022", h0)
        pos, att_fn, h_fn = X.hover(h0, wander=5.0)
        frames = X.frames_for(pos, att_fn, h_fn, g, noise=5.0)
        print(f"--- {h0} m, wander +-5 m")
        cases = [("exact (sim: +0.5 m bias, 0.15 m noise)", None)]
        cases += [(f"height scale {k:+d}%", (lambda k: lambda t, h: k / 100 * h)(k)) for k in (2, -2, 5, -5, 10, -10)]
        cases += [("ground under the hold 5 m lower than take-off", lambda t, h: -5.0),
                  ("drift 1 m/min, 5 min into the hold (5..5.3 m)", lambda t, h: 5.0 + t / 60.0)]
        for name, err in cases:
            att, hgt = X.fc_feeds(att_fn, h_fn, 20.0, hgt_err=err)
            res = X.score(X.run(frames, att, hgt, pos))
            print(f"  {name:48s} {X.fmt(res)}", flush=True)


def part_b():
    h0, T = 3.0, 10.0
    print(f"B. true height {h0:.0f} m, drone moves 2 m east in {T:.0f} s; baro height = true + error")
    g = X.Ground("synth", h0)
    _, att_fn, _ = X.hover(h0)
    small = lambda t: 0.3 * att_fn(t)                     # gentle attitude near the ground
    pos = lambda t: np.c_[np.zeros(len(t)), 0.2 * np.asarray(t)]
    h_fn = lambda t: np.full(len(t), h0)
    frames = X.frames_for(pos, small, h_fn, g, T=T)
    for err in (0.0, -1.0, -2.0, -2.5, -3.5):
        att, hgt = X.fc_feeds(small, h_fn, T, hgt_err=lambda t, h, e=err: e - 0.5)
        rows = X.run(frames, att, hgt, pos)
        sent = [r for r in rows if r[2] is not None]
        if len(sent) > 10:
            e = np.array([r[3][1] for r in sent])
            tr = np.array([r[4][1] for r in sent])
            gain = np.polyfit(tr, e, 1)[0]
            ok = sum(r[1] == "ok" for r in rows) / len(rows)
            print(f"  baro {h0 + err:+.1f} m (true {h0:.0f}): reported east move / true move = {gain:+.2f}; "
                  f"state 'ok' on {ok:.0%} of frames", flush=True)
        else:
            print(f"  baro {h0 + err:+.1f} m: {len(sent)} fixes sent")
    att, _ = X.fc_feeds(small, h_fn, T)
    hgt = X.Series(1e9)
    for t in np.arange(-1, T + 1, 0.1):
        hgt.add(t, 0.0)
    try:
        X.run(frames[:60], att, hgt, pos)
        print("  baro exactly 0.0 m: no crash")
    except ZeroDivisionError as ex:
        print(f"  baro exactly 0.0 m: Hold.process raises ZeroDivisionError ({ex}) -> the program dies")


if __name__ == "__main__":
    part_a()
    part_b()
