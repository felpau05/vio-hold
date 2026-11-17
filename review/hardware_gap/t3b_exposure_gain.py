#!/usr/bin/env python3
"""Item 3b: exposure time (motion blur) and automatic gain changes.

Each row integrates light over [t_row - exposure, t_row] (the sim renders an instant). Hold is told
t0 = first-row readout time, so a long exposure is also a hidden time offset of exposure/2.
Gain: the analog camera's auto exposure scales brightness; LK assumes constant brightness against
a keyframe taken seconds earlier. Ottawa photo ground, 100 m hover, 10 s.

Run: ~/.venvs/vio/bin/python t3b_exposure_gain.py      (about 2 minutes)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S
T = 10.0


def main():
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    pos, att_fn, h_fn = X.hover(h0)
    att, hgt = X.fc_feeds(att_fn, h_fn, T)
    print(f"--- exposure, {h0} m hover, {T:.0f} s")
    base = None
    for exp_ms in (0.0, 8.0, 16.7):
        fr = X.frames_for(pos, att_fn, h_fn, g, T=T, noise=5.0, exposure=exp_ms / 1000, n_sub=5 if exp_ms else 1)
        if base is None:
            base = fr
        a = X.score(X.run(fr, att, hgt, pos))
        b = X.score(X.run(fr, att, hgt, pos, t_off=S.FRAME_T_ERR - exp_ms / 2000))
        print(f"  exposure {exp_ms:4.1f} ms: stamp at readout: {X.fmt(a)}\n"
              f"                     stamp at mid-exposure: {X.fmt(b)}", flush=True)
    print(f"--- automatic gain, {h0} m hover, no blur")
    tt = np.array([t for t, _ in base])

    def scaled(gain_fn, offset_fn=lambda t: 0.0):
        return [(t, np.clip(im.astype(np.float32) * gain_fn(t) + offset_fn(t), 0, 255).astype(np.uint8))
                for t, im in base]
    cases = [
        ("constant", lambda t: 1.0),
        ("slow hunting x(1 +- 0.15) at 0.2 Hz", lambda t: 1 + 0.15 * np.sin(2 * np.pi * 0.2 * t)),
        ("slow hunting x(1 +- 0.30) at 0.2 Hz", lambda t: 1 + 0.30 * np.sin(2 * np.pi * 0.2 * t)),
        ("cloud: step to x0.6 at 5 s", lambda t: 1.0 if t < 5 else 0.6),
        ("step to x0.6 at 5 s, back to x1 at 7 s", lambda t: 0.6 if 5 <= t < 7 else 1.0),
    ]
    for name, gf in cases:
        rows = X.run(scaled(gf), att, hgt, pos)
        res = X.score(rows)
        states = [r[1] for r in rows]
        print(f"  gain {name:40s} {X.fmt(res)}  degraded {states.count('degraded') / len(states):4.0%} "
              f"lost {states.count('lost') / len(states):4.0%}", flush=True)
    _ = tt


if __name__ == "__main__":
    main()
