"""apb_est: the APB attitude filter and barometer height on synthetic data with a known truth.

  python tests_apb/test_apb_est.py
"""
import math
import os
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import apb_est as E          # noqa: E402
import apb_sensors as A      # noqa: E402
from hold import euler_R     # noqa: E402

G = E.G0
fails = []


def check(ok, text):
    print(("PASS " if ok else "FAIL ") + text)
    if not ok:
        fails.append(text)


def synth(seconds=40, hz=200, bias=(0.0093, -0.0008, -0.0017), still_s=3.0, seed=1, walk_acc=0.0):
    """Body rotating by hand after still_s still seconds; gyro = rates + bias + noise; accel = specific force."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, seconds, 1.0 / hz)
    w = np.zeros((len(t), 3))
    m = t > still_s
    w[m, 0] = 0.5 * np.sin(2 * math.pi * 0.3 * t[m])       # roll rate
    w[m, 1] = 0.4 * np.sin(2 * math.pi * 0.21 * t[m] + 1)  # pitch rate
    w[m, 2] = 0.3 * np.sin(2 * math.pi * 0.13 * t[m])      # yaw rate
    R, Rs = np.eye(3), []
    for k in range(len(t)):
        Rs.append(R)
        if k + 1 < len(t):
            R = R @ cv2.Rodrigues(w[k] / hz)[0]
            u, _, vt = np.linalg.svd(R)
            R = u @ vt
    Rs = np.array(Rs)
    lin = np.zeros((len(t), 3))
    if walk_acc:
        lin[m, 0] = walk_acc * np.sin(2 * math.pi * 1.8 * t[m])     # walking bob, NED north
    f_ned = lin - np.array([0, 0, G])                                # specific force in NED
    acc = np.einsum("kji,kj->ki", Rs, f_ned)                         # R^T f
    gyro = np.c_[t, w + np.array(bias) + rng.normal(0, 0.0012, w.shape)]
    accel = np.c_[t, acc + rng.normal(0, 0.03, acc.shape)]
    return gyro, accel, Rs, t


def angles(R):
    return math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1, min(1, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])


def run(gyro, accel, chunk=37):
    f = E.ApbAtt()
    out = []
    for k in range(0, len(gyro), chunk):                # fed in pieces, as live
        lo, hi = gyro[k, 0], gyro[min(k + chunk, len(gyro)) - 1, 0]
        out += f.update(gyro[k:k + chunk], accel[(accel[:, 0] > lo - 1e-9 - (0 if k else 1)) & (accel[:, 0] <= hi)])
    return f, np.array(out)


t0 = time.time()
# 1: bias, roll/pitch against truth while rotating by hand
gyro, accel, Rs, t = synth()
f, out = run(gyro, accel)
check(f.bias is not None and np.abs(f.bias - [0.0093, -0.0008, -0.0017]).max() < 0.0005,
      f"gyro bias found {np.round(f.bias, 4)} (true 0.0093 -0.0008 -0.0017 rad/s)")
idx = np.searchsorted(t, out[:, 0])
tru = np.array([angles(Rs[i]) for i in idx])
err = np.degrees(np.abs(out[:, 1:3] - tru[:, :2])).max(0)
check(err.max() < 0.5, f"roll/pitch error while turning: max {err.round(3)} deg (limit 0.5)")
yerr = np.degrees(np.abs(np.angle(np.exp(1j * (out[:, 3] - tru[:, 2]))))).max()
check(yerr < 1.0, f"yaw drift over 37 s of turning: {yerr:.2f} deg (limit 1.0; no magnetometer)")
check(len(out) and out[0, 0] >= 0.9 * E.APB["apb_bias_s"] - 1e-6,
      f"no attitude before the bias is known (first row at {out[0, 0]:.2f} s; needs 90 % of {E.APB['apb_bias_s']} s still)")

# 2: walking accelerations must not tilt the estimate much (gate + slow lean)
gyro, accel, Rs, t = synth(walk_acc=1.5, seed=2)
f, out = run(gyro, accel)
idx = np.searchsorted(t, out[:, 0])
tru = np.array([angles(Rs[i]) for i in idx])
err = np.degrees(np.abs(out[:, 1:3] - tru[:, :2])).max(0)
check(err.max() < 2.0, f"roll/pitch error with a 1.5 m/s^2 walking bob: max {err.round(2)} deg (limit 2)")

# 3: never still -> no attitude at all (not ready), never a guess
gyro, accel, _, _ = synth(still_s=0.0, seed=3)
f, out = run(gyro, accel)
check(len(out) == 0 and f.bias is None, f"never still: no attitude rows ({len(out)}), bias unknown")

# 4: sensor -> body rotation is applied (sensor axes 90 deg turned about z)
gyro, accel, Rs, t = synth(seed=4)
Rzs = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.]])     # body = Rsb @ sensor, so sensor = Rsb^T @ body
g2, a2 = gyro.copy(), accel.copy()
g2[:, 1:] = (Rzs.T @ gyro[:, 1:].T).T
a2[:, 1:] = (Rzs.T @ accel[:, 1:].T).T
f = E.ApbAtt({"imu_R_sb": Rzs.tolist()})
out = np.array(f.update(g2, a2))
idx = np.searchsorted(t, out[:, 0])
tru = np.array([angles(Rs[i]) for i in idx])
err = np.degrees(np.abs(out[:, 1:3] - tru[:, :2])).max(0)
check(err.max() < 0.5, f"imu_R_sb applied: roll/pitch error {err.round(3)} deg")

# 5: barometer height: zero on the floor, lift 0.5 m, smoothing
bh = E.BaroHeight({"floor_lens_m": 0.05, "baro_smooth_s": 0.5})
tb = np.arange(0, 20, 0.2)
h_true = np.where(tb < 5, 0.0, np.where(tb < 10, 1.0, 1.5))
p = 101300.0 - h_true * 12.0          # ~12 Pa per metre
bh.zero(float(p[tb < 3].mean()))
hs = np.array(bh.update(np.c_[tb, p]))
h_at = lambda x: hs[np.searchsorted(hs[:, 0], x), 1]      # noqa: E731
check(abs(h_at(4.0) - 0.05) < 0.01 and abs(h_at(9.0) - 1.05) < 0.03 and abs(h_at(19.0) - 1.55) < 0.03,
      f"baro height: floor {h_at(4.0):.3f}, 1 m {h_at(9.0):.3f}, 1.5 m {h_at(19.0):.3f} (+0.05 lens)")
check(abs(A.pa_to_m(-12.0) - 1.0) < 0.03, f"pa_to_m(-12 Pa) = {A.pa_to_m(-12.0):.3f} m")

# 6: load_rec merges chunks and drops the 1 s overlaps
with tempfile.TemporaryDirectory() as d:
    g = np.c_[np.arange(0, 3, 0.005), np.zeros((600, 3))]
    for k, (lo, hi) in enumerate(((0, 2.0), (1.0, 3.0))):
        m = (g[:, 0] >= lo) & (g[:, 0] < hi)
        np.savez(f"{d}/x_{k:03d}.npz", imgs=np.zeros((1, 4, 4), np.uint8), t_frame=np.array([lo]), seq=np.array([k]),
                 gyro=g[m], accel=g[m], baro=np.c_[g[m][::40, 0], np.full(m.sum() // 40 + (m.sum() % 40 > 0), 1e5)])
    _, gg, aa, bb = E.load_rec([f"{d}/x_001.npz", f"{d}/x_000.npz"])
    check(len(gg) == 600 and np.all(np.diff(gg[:, 0]) > 0), f"load_rec: {len(gg)} unique gyro rows of 600, in order")

print(f"{'ALL PASS' if not fails else f'{len(fails)} FAIL'} in {time.time() - t0:.0f} s")
sys.exit(1 if fails else 0)
