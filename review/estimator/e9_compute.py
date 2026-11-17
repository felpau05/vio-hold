#!/usr/bin/env python3
"""Check 9: compute cost of Hold.process on this laptop, and what it means for the i.MX8M Plus.

Frames are rendered first (ottawa2019fall 100 m, 10 s = 300 frames); only Hold.process is timed.
Configs: as written (300 corners, LK 21x21, 3 pyramid levels) with 1 and 4 OpenCV threads, then
leaner settings. A cProfile breakdown shows where the time goes. Also times Series.array() on a
live-sized attitude buffer (4000 rows), which is rebuilt whenever a new sample has arrived.
Run: ~/.venvs/vio/bin/python e9_compute.py       (about 60 s)
"""
import cProfile
import io
import os
import pstats
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S


class FewCorners:
    def __init__(self, real, n):
        self._real, self.n = real, n

    def __getattr__(self, k):
        return getattr(self._real, k)

    def goodFeaturesToTrack(self, img, n, q, d):
        return self._real.goodFeaturesToTrack(img, self.n, q, d)


def time_run(frames, att, hgt, threads, lk=None, corners=None, reps=1):
    cv2.setNumThreads(threads)
    old_lk = dict(C.Hm.LK)
    if lk:
        C.Hm.LK.update(lk)
    real = C.Hm.cv2
    if corners:
        C.Hm.cv2 = FewCorners(real, corners)
    ts, errs = [], []
    try:
        for _ in range(reps):
            hold = C.Hold(S.K, S.D, S.R_BC_NADIR, S.READOUT)
            pos, _, _ = S.hover(100)
            hold.reanchor(pos(np.array([0.0]))[0])
            for t0, img in frames:
                c0 = time.perf_counter()
                e = hold.process(img, t0 + S.FRAME_T_ERR, att, hgt)
                ts.append(time.perf_counter() - c0)
                if e.var is not None:
                    errs.append(np.linalg.norm(e.pos - pos(np.array([e.t]))[0]))
    finally:
        C.Hm.LK.clear()
        C.Hm.LK.update(old_lk)
        C.Hm.cv2 = real
    ts = np.array(ts[5:]) * 1e3
    return ts.mean(), np.percentile(ts, 95), ts.max(), float(np.sqrt(np.mean(np.square(errs))))


if __name__ == "__main__":
    C.reseed(1)
    cfg = C.Cfg()
    pos, att_fn, h_fn = S.hover(100)
    frames = C.make_frames("ottawa2019fall", pos, att_fn, h_fn, 10.0, cfg)
    att, hgt = C.fc_feeds(att_fn, h_fn, 10.0, cfg)
    print("Hold.process per frame on this laptop (Intel Core 7 240H), 300 frames at 100 m")
    print(" config                                       threads  mean(ms)  p95(ms)  max(ms)  RMS err(m)")
    rows = [("as written (300 corners, LK 21x21, 3 levels)", 1, None, None),
            ("as written", 4, None, None),
            ("as written", 16, None, None),
            ("LK 15x15, 2 levels", 1, dict(winSize=(15, 15), maxLevel=2), None),
            ("LK 15x15, 2 levels, 120 corners", 1, dict(winSize=(15, 15), maxLevel=2), 120),
            ("LK 15x15, 2 levels, 120 corners", 4, dict(winSize=(15, 15), maxLevel=2), 120)]
    for name, th, lk, nc in rows:
        m, p95, mx, rms = time_run(frames, att, hgt, th, lk, nc)
        print(f" {name:45s}   {th:3d}    {m:6.1f}   {p95:6.1f}   {mx:6.1f}    {rms:5.2f}")
    # profile
    cv2.setNumThreads(1)
    pr = cProfile.Profile()
    pr.enable()
    time_run(frames, att, hgt, 1)
    pr.disable()
    s = io.StringIO()
    pstats.Stats(pr, stream=s).sort_stats("tottime").print_stats(8)
    print("\ncProfile, 1 thread, top self-time:")
    print("\n".join(line for line in s.getvalue().splitlines() if line.strip()[:1].isdigit() or "tottime" in line))
    # live Series rebuild cost
    ser = C.Hm.Series(5.0)
    for k in range(4000):
        ser.add(k * 0.01, 0.1, 0.2, 0.3)
    c0 = time.perf_counter()
    for k in range(200):
        ser.add(40 + k * 0.01, 0.1, 0.2, 0.3)       # a new sample arrives -> cache invalid
        ser.array()
    print(f"\nSeries.array() rebuild with ~4000 rows: {(time.perf_counter() - c0) / 200 * 1e3:.2f} ms per call "
          f"(called for att, hgt each frame, after every new sample)")
