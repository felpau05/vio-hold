#!/usr/bin/env python3
"""Compute (scope 8; E9, H6): ms per Hold.process call at 360 x 240 with one OpenCV thread.

ottawa2019fall at 100 m, sim_ground settings, 300 frames. Each setting runs 3 times after a 30-frame
warm-up; the table gives the run with the median mean. Version 1 (tests/hold_v1.py) runs on the same
frames for a same-machine 'before'. The laptop's CPU governor is printed: 'powersave' clocks vary,
so compare rows of one run, not runs of different days. Also: does the hold work with frames at
12.5 and 8.3 per second (every 2nd / 3rd frame)?
Run: ~/.venvs/vio/bin/python tests/compute.py     (about 90 s) -> tests/compute.txt
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2                  # noqa: E402
import numpy as np          # noqa: E402

import harness as X         # noqa: E402
import hold_v1              # noqa: E402
from hold import Hold       # noqa: E402


def timed(make, frames, att, hgt, pos):
    out = []
    for _ in range(3):
        hold = make()
        hold.reanchor(pos(np.array([0.0]))[0])
        ts, err = [], []
        for i, (t0, img) in enumerate(frames):
            c0 = time.perf_counter()
            e = hold.process(img, t0 + X.S.FRAME_T_ERR, att, hgt)
            if i >= 30:
                ts.append(time.perf_counter() - c0)
            if e.var is not None:
                err.append(np.linalg.norm(e.pos - pos(np.array([e.t]))[0]))
        out.append((np.mean(ts) * 1e3, np.percentile(ts, 95) * 1e3, np.max(ts) * 1e3, np.sqrt(np.mean(np.square(err))),
                    len(err) / len(frames)))
    return sorted(out)[1]


def main():
    cv2.setNumThreads(1)
    gov = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read().strip() \
        if os.path.exists("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") else "?"
    frames, att, hgt, pos = X.scene(h0=100, T=12.0, seed=1)
    K, D, R, ro = X.S.K, X.S.D, X.S.R_BC_NADIR, X.S.READOUT
    rows = [("version 1 (300 corners, LK 21x21, 3 levels, 3 LK calls)", lambda: hold_v1.Hold(K, D, R, ro), frames),
            ("version 2 defaults (150 corners, LK 15x15, 2 levels)", lambda: Hold(K, D, R, ro), frames),
            ("version 2, backward check every 2nd frame", lambda: Hold(K, D, R, ro, fb_every=2), frames),
            ("version 2, 80 corners, backward check every 2nd frame", lambda: Hold(K, D, R, ro, max_pts=80, fb_every=2), frames),
            ("version 2 defaults, frames at 12.5 per second", lambda: Hold(K, D, R, ro), frames[::2]),
            ("version 2 defaults, frames at 8.3 per second", lambda: Hold(K, D, R, ro), frames[::3])]
    lines = [f"Hold.process at 360 x 240, cv2.setNumThreads(1), CPU governor '{gov}', 100 m ottawa2019fall",
             " setting                                                   mean ms  p95 ms  max ms   RMS m  fixes sent"]
    for name, make, fr in rows:
        m, p95, mx, rms, sent = timed(make, fr, att, hgt, pos)
        lines.append(f" {name:56s}  {m:6.1f}  {p95:6.1f}  {mx:6.1f}   {rms:5.2f}   {sent:6.1%}")
        print(lines[-1], flush=True)
    open(os.path.join(X.HERE, "compute.txt"), "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
