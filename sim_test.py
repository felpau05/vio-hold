#!/usr/bin/env python3
"""Synthetic test: a rolling-shutter nadir camera over textured ground, scored against truth.

Run:  python sim_test.py
Prints hold error per altitude (rolling-shutter model on/off), runs camera-blackout tests (analog
static of std 5, 30 and 60; the hold must recover by itself), a timing-calibration test, asserts the
basics, and writes sim_results.png.

shortcut: the ground texture scales with height, so pixel tracking is equally good at every
altitude; real ground loses contrast and detail with height. Fly a height ladder to find that limit.
"""
import math
import os

import cv2
import numpy as np

from hold import Hold, Series, load_lens, euler_R, R_BC_NADIR

HERE = os.path.dirname(os.path.abspath(__file__))
W, H, FPS = 360, 240, 30.0        # one analog field, width halved: what apb_main feeds Hold
READOUT, LATENCY = 0.016, 0.060   # true row readout time and capture latency in the sim
ATT_BIAS = np.radians([0.5, -0.3, 1.0])
ATT_NOISE, ATT_LAG, FRAME_T_ERR = math.radians(0.1), 0.003, 0.002
rng = np.random.default_rng(1)
K, D = load_lens(os.path.join(HERE, "justice_lens_profiles.json"), "Round 1 independent k2", W, H)

uv = np.stack(np.meshgrid(np.arange(W), np.arange(H)), -1).reshape(-1, 1, 2).astype(np.float64)
RAYS_B = np.c_[cv2.fisheye.undistortPoints(uv, K, D).reshape(-1, 2), np.ones(W * H)] @ R_BC_NADIR.T
RAYS_B = RAYS_B.reshape(H, W, 3)


def make_texture(n=4096):
    t = np.zeros((n, n), np.float32)
    for k in range(3, 11):        # 8 .. 1024 cells: detail at every scale, like real ground
        t += cv2.resize(rng.standard_normal((2 ** k, 2 ** k)).astype(np.float32), (n, n),
                        interpolation=cv2.INTER_CUBIC)
    lo, hi = np.percentile(t, [1, 99])
    return cv2.GaussianBlur(np.clip((t - lo) / (hi - lo) * 255, 0, 255), (0, 0), 1.0)


TEX = make_texture()


def render(t0, pos_fn, att_fn, h_fn, span_m):
    """One rolling-shutter frame: row r is exposed at t0 + r/H * READOUT."""
    tr = t0 + np.arange(H) / H * READOUT
    b = np.einsum("rij,rwj->rwi", euler_R(*att_fn(tr).T), RAYS_B)
    P = pos_fn(tr)[:, None, :] + (h_fn(tr)[:, None] / b[..., 2])[..., None] * b[..., :2]
    m = TEX.shape[0] / span_m
    img = cv2.remap(TEX, (P[..., 1] * m + TEX.shape[1] / 2).astype(np.float32),
                    (-P[..., 0] * m + TEX.shape[0] / 2).astype(np.float32), cv2.INTER_LINEAR)
    img = cv2.GaussianBlur(img, (3, 3), 0.8) + rng.normal(0, 3, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def fc_feeds(att_fn, h_fn, T):
    """What the flight controller would send: biased, noisy, slightly late attitude; noisy baro."""
    att, hgt = Series(1e9), Series(1e9)
    for t in np.arange(-1, T + 1, 0.01):                         # ATTITUDE at 100 Hz
        att.add(t, *(att_fn(np.array([t - ATT_LAG]))[0] + ATT_BIAS + rng.normal(0, ATT_NOISE, 3)))
    for t in np.arange(-1, T + 1, 0.1):                          # baro height at 10 Hz
        hgt.add(t, h_fn(np.array([t]))[0] + 0.5 + rng.normal(0, 0.15))
    return att, hgt


def hover(h0):
    s = np.sin
    pos = lambda t: np.c_[1.5 * s(0.21 * t) + 0.5 * s(0.9 * t + 1), 1.5 * s(0.17 * t + 2) + 0.5 * s(1.1 * t)]
    att = lambda t: np.radians(np.c_[
        4 * s(2 * np.pi * 0.7 * t) + 2 * s(2 * np.pi * 1.9 * t + 1) + 0.2 * s(2 * np.pi * 15 * t),
        4 * s(2 * np.pi * 0.6 * t + 2) + 2 * s(2 * np.pi * 2.3 * t) + 0.2 * s(2 * np.pi * 13 * t + 1),
        10 * s(2 * np.pi * 0.05 * t)])
    return pos, att, lambda t: h0 + 1.0 * s(0.3 * t)


def run(frames, att, hgt, readout_model, pos_fn, reanchor_err=None):
    """Per frame (t, state, error m, Mahalanobis^2 of the error under cov, keyframes confirmed so far)."""
    hold = Hold(K, D, R_BC_NADIR, readout_model)
    hold.reanchor(pos_fn(np.array([frames[0][0]]))[0])   # the EKF origin is wherever we start
    out, kfs = [], set()
    for t0, img in frames:
        t_in = t0 + FRAME_T_ERR + (READOUT - readout_model) / 2   # global-shutter model stamps mid-frame
        e = hold.process(img, t_in, att, hgt)
        if e.state == "lost" and reanchor_err is not None:
            hold.reanchor(pos_fn(np.array([e.t]))[0] + reanchor_err)   # stands in for the EKF estimate
        if hold.kf is not None and not hold.kf.get("pending"):
            kfs.add(id(hold.kf))
        if e.var is None:
            out.append((e.t, e.state, np.nan, np.nan, len(kfs)))
            continue
        d = e.pos - pos_fn(np.array([e.t]))[0]
        cov = e.cov if getattr(e, "cov", None) is not None else np.eye(2) * e.var / 2
        out.append((e.t, e.state, float(np.linalg.norm(d)), float(d @ np.linalg.solve(cov, d)), len(kfs)))
    return out


def altitude_sweep(heights=(10, 25, 50, 100, 150, 200, 300), T=20.0):
    rows = []
    for h0 in heights:
        pos, att_fn, h_fn = hover(h0)
        frames = [(t0, render(t0, pos, att_fn, h_fn, 8 * h0)) for t0 in np.arange(0, T, 1 / FPS)]
        att, hgt = fc_feeds(att_fn, h_fn, T)
        res = {}
        for name, ro in (("rolling-shutter model", READOUT), ("global-shutter model", 0.0)):
            out = np.array([r[2:4] for r in run(frames, att, hgt, ro, pos)], float)
            ok = ~np.isnan(out[:, 0])
            e, m2 = out[ok, 0], out[ok, 1]
            # "confident but wrong": error outside the 99 % bound of the covariance we send the EKF
            false_fix = np.mean(m2 > 9.21)
            res[name] = (math.sqrt(np.mean(e ** 2)), np.percentile(e, 95), ok.mean(), false_fix)
        rows.append((h0, res))
        print(f"{h0:4d} m  " + "  ".join(f"{k}: rms {v[0]:.2f} m, p95 {v[1]:.2f} m, used {v[2]:.0%}, "
                                          f"false-fix {v[3]:.1%}" for k, v in res.items()))
    return rows


def blackout_test(h0=100, T=20.0, dark=(10.0, 12.0), static_std=(5.0, 30.0, 60.0)):
    """The camera shows analog static for 2 s. Expect: no keyframe and no fix from the static, and a
    recovery without outside help (no re-anchor call) within 1 s of the picture returning."""
    pos, att_fn, h_fn = hover(h0)
    clean = [(t0, render(t0, pos, att_fn, h_fn, 8 * h0)) for t0 in np.arange(0, T, 1 / FPS)]
    att, hgt = fc_feeds(att_fn, h_fn, T)
    res = {}
    for sd in static_std:
        frames = [(t0, np.clip(90 + rng.normal(0, sd, img.shape), 0, 255).astype(np.uint8)
                   if dark[0] <= t0 < dark[1] else img) for t0, img in clean]
        out = run(frames, att, hgt, READOUT, pos)
        lost = [t for t, st, *_ in out if st == "lost" and t > dark[0]]
        assert lost and min(lost) < dark[0] + 0.5, f"static std {sd}: blackout not detected fast"
        sent_dark = [t for t, st, err, *_ in out if dark[0] + 0.5 < t < dark[1] and not np.isnan(err)]
        assert not sent_dark, f"static std {sd}: {len(sent_dark)} positions sent while the camera saw only static"
        kf_dark = max(r[4] for r in out if r[0] < dark[1]) - max(r[4] for r in out if r[0] < dark[0] + 0.5)
        assert kf_dark == 0, f"static std {sd}: {kf_dark} keyframes made on static"
        back = [t for t, st, err, *_ in out if t > dark[1] and not np.isnan(err)]
        assert back and back[0] < dark[1] + 1.0, f"static std {sd}: no fix within 1 s after the picture returned"
        print(f"blackout to static std {sd:2.0f}: lost after {min(lost) - dark[0]:.2f} s, 0 fixes and 0 keyframes "
              f"from the static, fixes again {back[0] - dark[1]:.2f} s after the picture returned (no re-anchor)")
        res[sd] = out
    return res[static_std[-1]]


def timing_test(T=10.0):
    """Hand-rotation bench test at 1.5 m; the replay sweep must find the true latency and readout."""
    import apb_main
    pos = lambda t: np.zeros((len(t), 2))
    att_fn = lambda t: np.radians(np.c_[15 * np.sin(2 * np.pi * 0.5 * t), 12 * np.sin(2 * np.pi * 0.4 * t + 1),
                                        20 * np.sin(2 * np.pi * 0.3 * t)])
    h_fn = lambda t: np.full(len(t), 1.5)
    t0s = np.arange(0, T, 1 / FPS)
    imgs = np.array([render(t0, pos, att_fn, h_fn, 12.0) for t0 in t0s])
    att, _ = fc_feeds(att_fn, h_fn, T)
    path = os.path.join(HERE, "sim_recording.npz")
    np.savez_compressed(path, imgs=imgs, t_grab=t0s + LATENCY, att=att.array(),
                        hgt=np.array([[-1, 1.5], [T + 1, 1.5]]), K=K, D=D)
    lat, ro = apb_main.sweep(path, R_BC_NADIR)
    want = LATENCY - ATT_LAG      # attitude arrives 3 ms late, so the best match is 3 ms less latency
    print(f"timing sweep found latency {lat * 1e3:.0f} ms (expected {want * 1e3:.0f}), "
          f"readout {ro * 1e3:.0f} ms (true {READOUT * 1e3:.0f})")
    assert abs(lat - want) < 0.011 and abs(ro - READOUT) < 0.0045, "timing sweep missed"


def plot(rows, blk):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.5))
    hs = [r[0] for r in rows]
    for name, c in (("rolling-shutter model", "#1f6feb"), ("global-shutter model", "#d1242f")):
        a.plot(hs, [r[1][name][0] for r in rows], "o-", color=c, label=f"{name} (RMS)")
        a.plot(hs, [r[1][name][1] for r in rows], "o--", color=c, alpha=0.5, label=f"{name} (95th pct)")
    a.set(xlabel="hover height (m)", ylabel="position error (m)", title="Hold error vs height (sim)")
    a.grid(alpha=0.3)
    a.legend(fontsize=8)
    t = np.array([r[0] for r in blk])
    e = np.array([r[2] for r in blk], float)
    b.plot(t, e, color="#1f6feb", lw=1)
    lost = np.array([r[1] == "lost" for r in blk])
    b.fill_between(t, 0, np.nanmax(e) * 1.1, where=lost, color="#d1242f", alpha=0.2, label="lost (nothing sent)")
    b.set(xlabel="time (s)", ylabel="position error (m)", title="2 s of analog static (std 60) at 100 m")
    b.grid(alpha=0.3)
    b.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "sim_results.png"), dpi=120)


if __name__ == "__main__":
    rows = altitude_sweep()
    blk = blackout_test()
    plot(rows, blk)
    try:
        timing_test()
    except (ModuleNotFoundError, AttributeError) as e:      # apb_main.py has no sweep() (yet)
        print(f"timing test skipped: {e}")
    print("checks done; wrote sim_results.png")
