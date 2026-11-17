#!/usr/bin/env python3
"""Check 6: frame-to-frame jitter of the sent position, its spectrum, and how much a flight
controller that fuses it would shake.

1) ottawa2019fall 100 m and 200 m, 3 seeds x 20 s. Jitter = std of the first difference of the
   error, per axis. Welch spectrum of the error -> e6_jitter_psd.png.
2) Closed loop, one axis, 250 Hz: steady Kalman filter (IMU-driven [p, v], accel noise 0.35 m/s^2
   like PX4 EKF2_ACC_NOISE) fuses the 30 Hz fixes with R = sent per-axis variance; PX4-style
   cascade (MPC_XY_P 0.95, MPC_XY_VEL_P_ACC 1.8, I 0.4), 80 ms attitude lag. The fix error
   sequence from the sim (mean removed) drives it. Output: RMS of the TRUE position wander and of
   the tilt it commands = the shake the judges would see from estimator noise alone.
3) Smallest pre-filters on the 30 Hz fixes: EMA a = 0.5, 0.3; moving average of 3 and 5.
   Their jitter cut and delay (group delay at low frequency).
Approximations: no wind, no IMU noise, KF steady-state; the EKF2 innovation gate is ignored.
Run: ~/.venvs/vio/bin/python e6_jitter.py      (about 40 s)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

FPS = C.S.FPS


def job(h0, seed):
    rec, _ = C.scenario("ottawa2019fall", h0, T=20.0, seed=seed)
    s = rec.sent
    return dict(h=h0, seed=seed, ex=rec.ex[s].tolist(), ey=rec.ey[s].tolist(), var=rec.var[s].tolist())


def closed_loop(noise, R, fs_cam=FPS, fs=250.0, lag=0.08, sigma_a=0.35, ki=0.4):
    """noise: per-frame measurement error (m). Returns RMS true-position wander and RMS tilt (deg)."""
    dt = 1 / fs
    F = np.array([[1, dt], [0, 1]])
    Q = sigma_a ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
    x_hat, P = np.zeros(2), np.eye(2) * 0.1
    p = v = a = integ = 0.0
    n_steps = int(len(noise) / fs_cam * fs)
    ps, acc = [], []
    k_cam, next_cam = 0, 0.0
    for i in range(n_steps):
        t = i * dt
        # controller on the estimate (setpoint 0)
        v_sp = float(np.clip(0.95 * (0 - x_hat[0]), -3, 3))
        ev = v_sp - x_hat[1]
        integ += ki * ev * dt
        a_sp = 1.8 * ev + integ
        a += (a_sp - a) * dt / lag                 # attitude loop lag
        v += a * dt
        p += v * dt
        # KF predict with the (perfect) IMU acceleration
        x_hat = F @ x_hat + np.array([0.5 * dt * dt, dt]) * a
        P = F @ P @ F.T + Q
        if t >= next_cam and k_cam < len(noise):
            z = p + noise[k_cam]
            S_ = P[0, 0] + R
            K = P[:, 0] / S_
            x_hat = x_hat + K * (z - x_hat[0])
            P = P - np.outer(K, P[0, :])
            k_cam += 1
            next_cam += 1 / fs_cam
        ps.append(p)
        acc.append(a)
    ps, acc = np.array(ps[int(2 * fs):]), np.array(acc[int(2 * fs):])
    return float(np.std(ps)), float(np.degrees(np.std(acc) / 9.81))


def ema(x, a):
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def ma(x, n):
    y = np.convolve(x, np.ones(n) / n, mode="full")[:len(x)]
    y[:n] = x[:n]
    return y


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy_free_welch import welch  # local helper (no scipy in the venv)

    res = C.pmap(job, [dict(h0=h, seed=s) for h in (100, 200) for s in (1, 2, 3)])
    fig, ax = plt.subplots(figsize=(7, 4))
    print("1) jitter = std of the frame-to-frame change of the fix error, per axis")
    print(" h(m)  jitter(m)  error std(m)  ratio jitter/(sqrt2*std)  [1.0 = white noise, 0 = slow drift]")
    filt_rows = []
    for h, col in ((100, "#1f6feb"), (200, "#d1242f")):
        rs = [r for r in res if r["h"] == h]
        d = np.concatenate([np.diff(r["ex"]) for r in rs] + [np.diff(r["ey"]) for r in rs])
        e = np.concatenate([np.asarray(r["ex"]) - np.mean(r["ex"]) for r in rs] +
                           [np.asarray(r["ey"]) - np.mean(r["ey"]) for r in rs])
        print(f" {h:4d}   {d.std():6.3f}      {e.std():6.3f}        {d.std() / (math.sqrt(2) * e.std()):.2f}")
        f, pxx = welch(np.concatenate([np.asarray(rs[0]["ex"]) - np.mean(rs[0]["ex"])]), FPS, 128)
        ps = [welch(np.asarray(r[k]) - np.mean(r[k]), FPS, 128)[1] for r in rs for k in ("ex", "ey")]
        ax.semilogy(f, np.mean(ps, axis=0), color=col, label=f"{h} m")
        # closed loop
        R_old = np.mean([np.mean(r["var"]) for r in rs]) / 2
        R_hon = (2 * (math.radians(0.13) * h) ** 2) + 0.004
        shake = {}
        for name, R in (("Hold's var", R_old), ("honest var", R_hon)):
            out = [closed_loop(np.asarray(r[k]) - np.mean(r[k]), R) for r in rs for k in ("ex", "ey")]
            shake[name] = (np.mean([o[0] for o in out]), np.mean([o[1] for o in out]))
        for name, fn, delay in (("none", lambda x: x, 0.0), ("EMA a=0.5", lambda x: ema(x, 0.5), 1.0),
                                ("EMA a=0.3", lambda x: ema(x, 0.3), 7 / 3), ("MA-3", lambda x: ma(x, 3), 1.0),
                                ("MA-5", lambda x: ma(x, 5), 2.0)):
            dd = [np.diff(fn(np.asarray(r[k]))) for r in rs for k in ("ex", "ey")]
            ee = [fn(np.asarray(r[k]) - np.mean(r[k])) for r in rs for k in ("ex", "ey")]
            cl = [closed_loop(x, R_hon) for x in ee]
            filt_rows.append((h, name, np.concatenate(dd).std(), np.concatenate(ee).std(), delay * 1000 / FPS,
                              np.mean([c[0] for c in cl]), np.mean([c[1] for c in cl])))
        so = shake["Hold's var"]
        print(f"   closed loop at {h} m, R = Hold's var ({math.sqrt(R_old):.2f} m/axis): true wander {so[0]:.3f} m,"
              f" tilt {so[1]:.3f} deg;  R = honest ({math.sqrt(R_hon):.2f} m/axis): wander "
              f"{shake['honest var'][0]:.3f} m, tilt {shake['honest var'][1]:.3f} deg")
    ax.set(xlabel="frequency (Hz)", ylabel="PSD of fix error per axis (m^2/Hz)",
           title="Fix error spectrum, ottawa2019fall (sim, 3 seeds x 20 s)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(C.HERE, "e6_jitter_psd.png"), dpi=110)
    print("\n3) pre-filters on the 30 Hz fix (closed loop with the honest R)")
    print(" h(m)  filter      jitter(m)  error std(m)  delay(ms)  true wander(m)  tilt(deg)")
    for r in filt_rows:
        print(f" {r[0]:4d}  {r[1]:10s}  {r[2]:6.3f}     {r[3]:6.3f}       {r[4]:5.0f}      {r[5]:6.3f}        {r[6]:.3f}")
    print("wrote e6_jitter_psd.png")
