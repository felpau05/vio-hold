#!/usr/bin/env python3
"""Items 5 and 1, closed loop: does the drone stay or fly away?

The project simulators are open loop: the estimate never steers the drone. This script closes the
loop with a crude multirotor (point mass, tilt = commanded acceleration / g with a 0.1 s attitude
lag, small hover wobble, steady wind) and a PX4-like cascade (MPC_XY_P 0.95, MPC_XY_VEL_P_ACC 1.8,
5 m/s and 20 deg limits). The EKF stand-in is an alpha-beta filter: it integrates the IMU
acceleration (0.1 m/s^2 bias, noise) and corrects position AND velocity from each fix Hold sends,
because with GNSS off the fixes are the only thing that makes horizontal velocity observable.
After 'lost', Hold re-anchors to that estimate, as the design says. A frame is processed 66 ms after
its first row (two frame periods), when its attitude has arrived. Ottawa photo, 100 m.

It is a toy controller, not PX4. It shows the sign of the loop, not exact numbers.

Run: ~/.venvs/vio/bin/python t6_closed_loop.py      (about 2 minutes)
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S
G = 9.81
H0 = 100.0
T = 30.0


def fly(name, R_bc_true=X.R_BC_NADIR, mirror=False, stamp_err=0.0, ground=None, seed=3):
    rng = np.random.default_rng(seed)
    rays = X.body_rays(X.K0, X.D0, X.W, X.H, R_bc_true)
    dt = 1 / 300
    wind = np.array([0.3, -0.2])                     # m/s^2 steady push
    p, v = np.zeros(2), np.zeros(2)
    att = np.zeros(3)
    a_cmd = np.zeros(2)
    p_est, v_est = np.zeros(2), np.zeros(2)
    acc_bias = np.array([0.1, -0.07])
    tl, pl, al = [], [], []
    att_s, hgt_s = X.Series(1e9), X.Series(1e9)
    hold = X.Hold(X.K0, X.D0, X.R_BC_NADIR, S.READOUT)
    hold.reanchor(np.zeros(2))
    t, next_att, next_hgt = -1.0, -1.0, -1.0
    sent = lost = 0
    maxd = 0.0
    log_every = []

    def wobble(tt):
        return np.radians([1.5 * math.sin(2 * math.pi * 0.7 * tt) + 0.2 * math.sin(2 * math.pi * 15 * tt),
                           1.5 * math.sin(2 * math.pi * 0.6 * tt + 2) + 0.2 * math.sin(2 * math.pi * 13 * tt + 1),
                           3 * math.sin(2 * math.pi * 0.05 * tt)])

    def step():
        nonlocal t, p, v, att, next_att, next_hgt
        tilt_cmd = np.array([math.atan(a_cmd[1] / G), -math.atan(a_cmd[0] / G), 0.0])
        att = att + (tilt_cmd - att) * dt / 0.1
        att_true = att + wobble(t)
        acc = np.array([-G * math.tan(att_true[1]), G * math.tan(att_true[0])]) + wind
        v = v + acc * dt
        p = p + v * dt
        t += dt
        tl.append(t)
        pl.append(p.copy())
        al.append(att_true.copy())
        if t >= next_att:
            k = max(0, len(tl) - 1 - int(round(S.ATT_LAG / dt)))
            att_s.add(t, *(al[k] + S.ATT_BIAS + rng.normal(0, S.ATT_NOISE, 3)))
            next_att += 0.01
        if t >= next_hgt:
            hgt_s.add(t, H0 + 0.5 + rng.normal(0, 0.15))
            next_hgt += 0.1

    while t < 0:
        step()
    v_prev = v.copy()
    for k in range(int(T * X.FPS)):
        tf = k / X.FPS
        while t < tf + 0.066:
            step()
        ta, pa, aa = np.array(tl), np.array(pl), np.array(al)
        pos_fn = lambda tt: np.c_[np.interp(tt, ta, pa[:, 0]), np.interp(tt, ta, pa[:, 1])]
        att_fn = lambda tt: np.c_[np.interp(tt, ta, aa[:, 0]), np.interp(tt, ta, aa[:, 1]), np.interp(tt, ta, aa[:, 2])]
        img = X.render(tf, pos_fn, att_fn, lambda tt: np.full(len(tt), H0), ground, rays=rays, noise=5.0)
        if mirror:
            img = cv2.flip(img, 1)
        e = hold.process(img, tf + S.FRAME_T_ERR + stamp_err, att_s, hgt_s)
        if e.state == "lost":
            lost += 1
            hold.reanchor(p_est)
        # EKF stand-in: predict with the IMU over one frame period, then correct with the fix
        a_imu = (v - v_prev) * X.FPS + acc_bias + rng.normal(0, 0.05, 2)
        v_prev = v.copy()
        p_est = p_est + v_est / X.FPS + 0.5 * a_imu / X.FPS ** 2
        v_est = v_est + a_imu / X.FPS
        if e.var is not None:
            sent += 1
            innov = e.pos - (p_est - v_est * (t - e.t))      # fix is (t - e.t) old: delay-compensated
            p_est = p_est + 0.25 * innov
            v_est = v_est + 0.6 * innov
        v_sp = np.clip(0.95 * (0 - p_est), -5, 5)
        a_cmd = np.clip(1.8 * (v_sp - v_est), -G * math.tan(math.radians(20)), G * math.tan(math.radians(20)))
        maxd = max(maxd, float(np.linalg.norm(p)))
        if k % 150 == 149:
            log_every.append(f"{tf + 1 / X.FPS:4.0f}s:{np.linalg.norm(p):6.1f}")
    n = int(T * X.FPS)
    verdict = "FLY-AWAY (>20 m)" if maxd > 20 else ("poor (>5 m)" if maxd > 5 else "holds")
    print(f"  {name:38s} max distance {maxd:6.1f} m, final {np.linalg.norm(p):6.1f} m, fixes sent {sent / n:4.0%}, "
          f"lost frames {lost / n:4.0%} -> {verdict}\n      distance over time: {' '.join(log_every)}", flush=True)


def main():
    g = X.Ground("ottawa2022", H0)
    print(f"closed loop, {H0:.0f} m, {T:.0f} s, steady wind push 0.36 m/s^2")
    fly("nominal", ground=g)
    fly("frame stamp 20 ms early", stamp_err=-0.020, ground=g)
    fly("frame stamp 40 ms late", stamp_err=0.040, ground=g)
    fly("picture turned 180 deg", R_bc_true=X.R_BC_NADIR @ X.rot_z(math.pi), ground=g)
    fly("picture mirrored left-right", mirror=True, ground=g)
    fly("picture turned 90 deg", R_bc_true=X.R_BC_NADIR @ X.rot_z(math.pi / 2), ground=g)


if __name__ == "__main__":
    main()
