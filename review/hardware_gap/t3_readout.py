#!/usr/bin/env python3
"""Item 3a: rolling-shutter readout model wrong (true 16 ms), and readout running bottom to top.

Two ways the stamp can sit:
  first-row: t0 is the true first-row time; only the slope (readout) is wrong.
  mid-frame: the mid-frame time is right (what a timing sweep tends to find when it fits latency and
             readout together, and what sim_test.run assumes); the slope is wrong.
Ottawa photo ground, sim hover.

Run: ~/.venvs/vio/bin/python t3_readout.py      (about 2 minutes; 15 s runs)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S


def main():
    for h0 in (100, 200):
        g = X.Ground("ottawa2022", h0)
        pos, att_fn, h_fn = X.hover(h0)
        frames = X.frames_for(pos, att_fn, h_fn, g, T=15.0, noise=5.0)
        att, hgt = X.fc_feeds(att_fn, h_fn, 15.0)
        print(f"--- {h0} m, true readout 16 ms (top to bottom)")
        for ro_ms in (16, 0, 8, 24, 33):
            ro = ro_ms / 1000
            first = X.score(X.run(frames, att, hgt, pos, readout=ro))
            mid = X.score(X.run(frames, att, hgt, pos, readout=ro, t_off=S.FRAME_T_ERR + (S.READOUT - ro) / 2))
            print(f"  model {ro_ms:2d} ms  first-row stamp: rms {first['rms']:.2f} p95 {first['p95']:.2f} m sent "
                  f"{first['used']:.0%} | mid-frame stamp: rms {mid['rms']:.2f} p95 {mid['p95']:.2f} m sent "
                  f"{mid['used']:.0%}", flush=True)
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    pos, att_fn, h_fn = X.hover(h0)
    frames = X.frames_for(pos, att_fn, h_fn, g, T=15.0, noise=5.0, readout=-S.READOUT)
    att, hgt = X.fc_feeds(att_fn, h_fn, 15.0)
    print(f"--- {h0} m, camera reads bottom row first (sensor flip setting), true readout 16 ms")
    # frames carry the time the first row READ (bottom row); the top row is read 16 ms later
    for name, ro, toff in (("model +16 ms, stamp = first row read", S.READOUT, 0.0),
                           ("model -16 ms, stamp = top row time", -S.READOUT, S.READOUT * (1 - 1 / X.H)),
                           ("model 0 ms (global shutter), mid-frame stamp", 0.0, S.READOUT / 2)):
        res = X.score(X.run(frames, att, hgt, pos, readout=ro, t_off=S.FRAME_T_ERR + toff))
        print(f"  {name:45s} {X.fmt(res)}", flush=True)


if __name__ == "__main__":
    main()
