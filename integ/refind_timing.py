"""Time Hold.process on the frame after 'lost' (the re-find of the last/home keyframe), offline.
Camera: px4sim/camera.py at 100 m, slow drift; break-up bursts make Hold lose the anchor.
Run: ~/.venvs/vio/bin/python integ/refind_timing.py"""
import math
import os
import sys
import time
import cProfile
import pstats

import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import camera as C   # noqa: E402
import hold          # noqa: E402
import sitl          # noqa: E402

c = dict(sitl.DEFAULTS, true_lens="Round 1 independent k2", vib_deg=0.0)
cam = C.Camera(c, C.photo_ground("ottawa2019fall"))
H0 = 100.0


def att_at(t):
    return 0.02 * math.sin(1.3 * t), 0.015 * math.sin(0.9 * t + 1), 0.3


def pose(t):
    t = np.asarray(t, float)
    q = np.array([C.qmul(C.q_axis(C.E_D, y), C.qmul(C.q_axis(C.E_E, p), C.q_axis(C.E_N, r)))
                  for r, p, y in (att_at(x) for x in t)])
    return 0.5 * t, 0.2 * t, np.full(len(t), H0), q


att, hgt = hold.Series(1e9), hold.Series(1e9)
for t in np.arange(-1, 30, 0.01):
    att.add(t, *att_at(t), 0.0, 0.0, 0.0)
    hgt.add(t, H0)
K, D = hold.load_lens(C.LENS_JSON, "Round 1 independent k2", cam.W, cam.H)
h = hold.Hold(K, D, hold.make_R_bc(), c["readout_s"])
times = []
SLOW = int(sys.argv[1]) if len(sys.argv) > 1 else 130
for k in range(200):
    t0 = k / 25 + 0.002
    img = cam.finish(cam.render(t0, pose))
    if 100 <= k < 130:
        img = np.clip(np.random.default_rng(k).normal(128, 45, img.shape), 0, 255).astype(np.uint8)   # static
    w = time.perf_counter()
    if k == SLOW:
        pr = cProfile.Profile()
        pr.enable()
    e = h.process(img, t0, att, hgt)
    if k == SLOW:
        pr.disable()
    times.append((k, time.perf_counter() - w, e.state, e.why))
for k, dt, st, why in times[98:140]:
    print(f"frame {k}: {dt * 1e3:8.1f} ms  {st:8s} {why or ''}")
pstats.Stats(pr).sort_stats("cumulative").print_stats(12)
