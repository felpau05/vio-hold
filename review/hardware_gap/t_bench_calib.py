#!/usr/bin/env python3
"""Proves the event-day hand-rotation check works (items 1, 3, 4, 5).

Part A replays sim_test.timing_test exactly (pure rotation 1.5 m over the floor, 60 ms capture
latency, 3 ms attitude lag) against bench_calib.sweep, the reference for the missing apb_main.sweep.
Part B: drone turned by hand with a 0.2 m lever arm, camera aimed at a far scene (60 m). The true
camera differs from the model in latency, mounting, picture orientation, mirror and focal length.
The check must find each one from 10 s of video plus the FC attitude.

Run: ~/.venvs/vio/bin/python t_bench_calib.py     (about 2 minutes)
"""
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X                      # noqa: E402
import bench_calib as B                  # noqa: E402

S = X.S
T = 10.0


def hand_att(t):
    return np.radians(np.c_[15 * np.sin(2 * np.pi * 0.5 * t), 12 * np.sin(2 * np.pi * 0.4 * t + 1),
                            20 * np.sin(2 * np.pi * 0.3 * t)])


def recording(h0, latency, R_bc_true, fscale_true=1.0, mirror=False, lever=0.0):
    K = X.K0.copy()
    K[0, 0] *= fscale_true
    K[1, 1] *= fscale_true
    rays = X.body_rays(K, X.D0, X.W, X.H, R_bc_true)
    a = hand_att
    pos = (lambda t: lever * np.c_[np.sin(a(t)[:, 1]), -np.sin(a(t)[:, 0])]) if lever else \
        (lambda t: np.zeros((len(t), 2)))
    hf = lambda t: np.full(len(t), float(h0))
    g = X.Ground("synth", h0 * 1.5)       # 8 * 1.5 * h0 metres of texture
    t0s = np.arange(0, T, 1 / X.FPS)
    imgs = []
    for t0 in t0s:
        im = X.render(t0, pos, a, hf, g, rays=rays)
        imgs.append(cv2.flip(im, 1) if mirror else im)
    att, _ = X.fc_feeds(a, hf, T)
    return dict(imgs=np.array(imgs), t_grab=t0s + latency, att=att.array(), K=X.K0, D=X.D0,
                hgt=np.array([[-1, h0], [T + 1, h0]]))


def main():
    t_start = time.time()
    print("A. sim_test.timing_test replay (pure rotation at 1.5 m, latency 60 ms, attitude lag 3 ms)")
    rec = recording(1.5, S.LATENCY, X.R_BC_NADIR)
    path = os.path.join(X.HERE, "bench_recording.npz")
    np.savez_compressed(path, **rec)
    lat, ro = B.sweep(path, X.R_BC_NADIR)
    want = S.LATENCY - S.ATT_LAG
    ok = abs(lat - want) < 0.011 and abs(ro - S.READOUT) < 0.0045
    print(f"   sweep found latency {lat * 1e3:.0f} ms (expected {want * 1e3:.0f}), readout {ro * 1e3:.0f} ms "
          f"(true {S.READOUT * 1e3:.0f})  -> timing_test assertion {'PASSES' if ok else 'FAILS'}")
    os.remove(path)

    print("B. far scene (60 m), hand pivot 0.2 m, latency 75 ms, true readout 16 ms")
    cases = [
        ("nominal mount", X.R_BC_NADIR, 1.0, False),
        ("tilt roll +3 pitch -2, focal -5%", X.rot_x(math.radians(3)) @ X.rot_y(math.radians(-2)) @ X.R_BC_NADIR,
         0.95, False),
        ("picture turned 180", X.R_BC_NADIR @ X.rot_z(math.pi), 1.0, False),
        ("picture turned 90", X.R_BC_NADIR @ X.rot_z(math.pi / 2), 1.0, False),
        ("picture mirrored left-right", X.R_BC_NADIR, 1.0, True),
    ]
    for name, R_true, fs, mir in cases:
        rec = recording(60.0, 0.075, R_true, fs, mir, lever=0.2)
        b = B.Bench(rec)
        m = b.full_check()
        print(f"   {name:34s} -> found turn {m['turn_deg']:3d} deg, mirror {str(m['mirror']):5s}, "
              f"tilt roll {m['roll_deg']:+.2f} pitch {m['pitch_deg']:+.2f} yaw {m['yaw_deg']:+.2f} deg, "
              f"focal x{m['fscale']:.3f}, latency {m['lat'] * 1e3:.0f} ms (true {75 - S.ATT_LAG * 1e3:.0f}), "
              f"readout {m['ro'] * 1e3:.0f} ms; residual {m['resid_deg']:.3f} deg vs next-best orientation "
              f"{m['other_orientation_resid_deg']:.3f} deg", flush=True)
    print(f"done in {time.time() - t_start:.0f} s")


if __name__ == "__main__":
    main()
