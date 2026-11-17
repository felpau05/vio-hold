#!/usr/bin/env python3
"""Compute budget: time per Hold.process call on this laptop, one thread.

The APB companion is an i.MX8M Plus (4x Cortex-A53). This script cannot run there; it gives the
laptop number and the parts of the loop, so the team knows what to time on the board first.

Run: ~/.venvs/vio/bin/python t9_cpu.py      (about 40 s)
"""
import os
import platform
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S


def cpu_name():
    try:
        for line in open("/proc/cpuinfo"):
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def main():
    cv2.setNumThreads(1)
    print(f"CPU: {cpu_name()}; OpenCV threads 1")
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    pos, att_fn, h_fn = X.hover(h0)
    frames = X.frames_for(pos, att_fn, h_fn, g, T=10.0, noise=5.0)
    att, hgt = X.fc_feeds(att_fn, h_fn, 10.0)
    hold = X.Hold(X.K0, X.D0, X.R_BC_NADIR, S.READOUT)
    hold.reanchor(np.zeros(2))
    dts = []
    for t0, img in frames:
        a = time.perf_counter()
        hold.process(img, t0 + S.FRAME_T_ERR, att, hgt)
        dts.append(time.perf_counter() - a)
    d = np.array(dts[5:]) * 1e3
    print(f"Hold.process 360x240: mean {d.mean():.2f} ms, p95 {np.percentile(d, 95):.2f} ms, max {d.max():.2f} ms "
          f"(budget 33.3 ms per frame at 30 Hz, 16.7 ms per field at 60 Hz)")
    rows = [(i * 0.01, 0.1, 0.2, 0.3) for i in range(4000)]
    s = X.Series(5.0)
    s.rows = list(rows)
    a = time.perf_counter()
    for i in range(50):
        s.rows.append((40 + i * 0.01, 0.1, 0.2, 0.3))
        s.array()
    print(f"Series.array() rebuild with 4000 rows (every new attitude sample): {(time.perf_counter() - a) / 50 * 1e3:.2f} ms")
    img = frames[0][1]
    big = cv2.resize(img, (720, 480))
    for name, im in (("360x240", img), ("720x480", big)):
        a = time.perf_counter()
        for _ in range(20):
            p = cv2.goodFeaturesToTrack(im, 300, 0.01, 7)
        t_gftt = (time.perf_counter() - a) / 20 * 1e3
        a = time.perf_counter()
        for _ in range(20):
            cv2.calcOpticalFlowPyrLK(im, im, p, None, winSize=(21, 21), maxLevel=3)
        t_lk = (time.perf_counter() - a) / 20 * 1e3
        print(f"  {name}: goodFeaturesToTrack(300) {t_gftt:.2f} ms, one LK pass on {len(p)} points {t_lk:.2f} ms "
              f"(process() runs 3 LK passes per frame)")


if __name__ == "__main__":
    main()
