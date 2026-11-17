#!/usr/bin/env python3
"""Honest simulator: real aerial photos as the ground, at their true size.

Unlike sim_test.py, ground detail now shrinks with height. It also adds motor vibration that the
flight controller's 100 Hz attitude never sees, and analog video noise measured on real video (real/).

Run:  python fetch_ground.py ground   (once: downloads the photos)
      python sim_ground.py [site ...] [--heights 100,200] [--seeds 6] [--video]
          -> prints a table pooled over the seeds, writes sim_ground.png, sim_ground_view.png
             and, with --video, videos/ (seed 1 of each run, plus all heights per site)

shortcut: the photos hold about 0.6 m of detail per pixel, so below ~100 m the ground is blurrier
than real ground (results there are too pessimistic). Orthophotos are flat: no trees or roofs
sticking up. The real camera's contrast is unknown; the photos may have more.
"""
import json
import math
import os
import subprocess
import sys

import cv2
import imageio_ffmpeg
import numpy as np

import sim_test as S
from hold import Hold

GROUND = os.path.join(S.HERE, "ground")
VIDEOS = os.path.join(S.HERE, "videos")
FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
COLORS = {"ok": (80, 200, 80), "degraded": (0, 200, 255), "init": (255, 200, 0), "lost": (60, 60, 230)}
SITES = json.load(open(os.path.join(GROUND, "sites.json")))
VIB_DEG, VIB_HZ = 0.2, (140.0, 97.0)   # motor vibration on roll and pitch (assumed); FC never sees it
FPS = 25.0                             # PAL (real/numbers_clip-1.json: 24.96 fps)
NOISE = 3.7                            # analog video noise, grey levels (real clip-1: 3.68 when still)
NOISE_CORR_X = 0.58                    # lag-1 correlation of that noise along x (real clip-1: 0.583)
HEIGHTS = (50, 100, 150, 200, 300, 400)
CHI2_99 = 9.21                         # 99 % bound of a 2-D Gaussian


def video_noise(shape):
    """Noise of std NOISE whose neighbours along x correlate by NOISE_CORR_X (kernel [a, 1, a])."""
    c = NOISE_CORR_X
    a = (1 - math.sqrt(1 - 2 * c * c)) / (2 * c) if c > 0 else 0.0     # 2a / (1 + 2a^2) = c
    k = np.array([[a, 1.0, a]], np.float32) / math.sqrt(1 + 2 * a * a)
    return cv2.filter2D(S.rng.normal(0, NOISE, shape).astype(np.float32), -1, k, borderType=cv2.BORDER_REFLECT)


def load(name):
    g = cv2.cvtColor(cv2.imread(os.path.join(GROUND, name + ".jpg")), cv2.COLOR_BGR2GRAY)
    pyr = [g.astype(np.float32)]
    while pyr[-1].shape[0] > 256:
        pyr.append(cv2.pyrDown(pyr[-1]))
    return pyr, SITES[name]["m_per_px"]


def render(t0, pos_fn, att_fn, h_fn, pyr, mpp):
    """One rolling-shutter frame of the photo ground; the pyramid level matches the ground pixel size."""
    tr = t0 + np.arange(S.H) / S.H * S.READOUT
    b = np.einsum("rij,rwj->rwi", S.euler_R(*att_fn(tr).T), S.RAYS_B)
    h = h_fn(tr)
    P = pos_fn(tr)[:, None, :] + (h[:, None] / b[..., 2])[..., None] * b[..., :2]
    gsd = h.mean() / S.K[0, 0]
    lvl = min(len(pyr) - 1, max(0, int(math.floor(math.log2(gsd / mpp)))))
    tex, m = pyr[lvl], 1.0 / (mpp * 2 ** lvl)
    img = cv2.remap(tex, (P[..., 1] * m + tex.shape[1] / 2).astype(np.float32),
                    (-P[..., 0] * m + tex.shape[0] / 2).astype(np.float32), cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REFLECT101)
    img = cv2.GaussianBlur(img, (3, 3), 0.8) + video_noise(img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def with_vibration(att_fn):
    def f(t):
        a = np.array(att_fn(t), float)
        a[:, 0] += math.radians(VIB_DEG) * np.sin(2 * np.pi * VIB_HZ[0] * t)
        a[:, 1] += math.radians(VIB_DEG) * np.sin(2 * np.pi * VIB_HZ[1] * t + 1)
        return a
    return f


def video_writer(path, w, h):
    return subprocess.Popen([FFMPEG, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                             "-s", f"{w}x{h}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                             "-pix_fmt", "yuv420p", "-crf", "26", path], stdin=subprocess.PIPE)


def annotate(img, hold, e, err, label):
    """2x camera frame with the tracked points, the hold state and the true error."""
    v = cv2.resize(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    col = COLORS[e.state]
    if hold.kf is not None and e.state != "lost":
        for x, y in hold.prev_px.reshape(-1, 2):
            cv2.circle(v, (int(2 * x), int(2 * y)), 2, col, -1)
    txt = f"{label}  {e.state}  " + (f"error {err:.2f} m" if not np.isnan(err) else "nothing sent")
    cv2.rectangle(v, (0, 0), (v.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(v, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1, cv2.LINE_AA)
    return v


def run_recorded(frames, att, hgt, pos_fn, path=None, label=""):
    """sim_test.run, with a video when path is given. No re-anchor help after a loss.

    Returns per frame (t, state, error vector or None, cov or None).
    """
    hold = Hold(S.K, S.D, S.R_BC_NADIR, S.READOUT)
    hold.reanchor(pos_fn(np.array([frames[0][0]]))[0])
    vid, out = (video_writer(path, 2 * S.W, 2 * S.H) if path else None), []
    for t0, img in frames:
        e = hold.process(img, t0 + S.FRAME_T_ERR, att, hgt)
        d = e.pos - pos_fn(np.array([e.t]))[0] if e.var is not None else None
        cov = getattr(e, "cov", None)
        if e.var is not None and cov is None:
            cov = np.eye(2) * e.var / 2          # version 1: var is the 2-axis total
        out.append((e.t, e.state, d, cov))
        if vid:
            vid.stdin.write(annotate(img, hold, e, np.linalg.norm(d) if d is not None else np.nan, label).tobytes())
    if vid:
        vid.stdin.close()
        vid.wait()
    return out


def montage(paths, out_path, cols=3):
    """All heights of one site side by side, at 480 x 320 per tile."""
    n = len(paths)
    tiles = "".join(f"[{i}:v]scale=480:320[t{i}];" for i in range(n))
    layout = "|".join(f"{480 * (i % cols)}_{320 * (i // cols)}" for i in range(n))
    args = [FFMPEG, "-y", "-loglevel", "error"]
    for p in paths:
        args += ["-i", p]
    args += ["-filter_complex", tiles + "".join(f"[t{i}]" for i in range(n)) + f"xstack=inputs={n}:layout={layout}",
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28", out_path]
    subprocess.run(args, check=True)


def one(name, h0, seed, T=20.0, video=False):
    """One 20 s hover at one height with one noise seed. Returns the per-frame errors and covariances."""
    S.rng = np.random.default_rng(seed)
    pyr, mpp = load(name)
    pos, att_fn, h_fn = S.hover(h0)
    seen = with_vibration(att_fn)                     # what the camera feels
    frames = [(t0, render(t0, pos, seen, h_fn, pyr, mpp)) for t0 in np.arange(0, T, 1 / FPS)]
    att, hgt = S.fc_feeds(att_fn, h_fn, T)            # what the flight controller reports
    path = os.path.join(VIDEOS, f"{name}_{h0}m.mp4") if video else None
    out = run_recorded(frames, att, hgt, pos, path, f"{name} {h0} m")
    sent = [r for r in out if r[2] is not None]
    e = np.array([r[2] for r in sent]).reshape(-1, 2)
    m2 = np.array([d @ np.linalg.solve(c, d) for _, _, d, c in sent])      # squared Mahalanobis distance
    return dict(name=name, h=h0, seed=seed, frames=len(out), lost=sum(r[1] == "lost" for r in out),
                e2=np.sum(e ** 2, axis=1), m2=m2), frames[len(frames) // 2][1]


def pooled(runs):
    """Pooled numbers over seeds: sent share, RMS, p95, worst, share outside the 99 % bound of cov."""
    e2 = np.concatenate([r["e2"] for r in runs])
    m2 = np.concatenate([r["m2"] for r in runs])
    rms_seed = [math.sqrt(np.mean(r["e2"])) if len(r["e2"]) else math.nan for r in runs]
    if not len(e2):
        return dict(used=0.0, lost=1.0, rms=math.nan, p95=math.nan, worst=math.nan, false_fix=math.nan, out50=math.nan,
                    rms_seed=rms_seed)
    return dict(used=len(e2) / sum(r["frames"] for r in runs), lost=sum(r["lost"] for r in runs) / sum(r["frames"] for r in runs),
                rms=math.sqrt(np.mean(e2)), p95=float(np.sqrt(np.percentile(e2, 95))), worst=float(np.sqrt(e2.max())),
                false_fix=float(np.mean(m2 > CHI2_99)), out50=float(np.mean(m2 > 1.386)), rms_seed=rms_seed)


def _job(args):
    import cv2 as _cv2
    _cv2.setNumThreads(1)
    return one(*args)


def main():
    import argparse
    import multiprocessing as mp

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ap = argparse.ArgumentParser()
    ap.add_argument("sites", nargs="*")
    ap.add_argument("--heights", default=",".join(map(str, HEIGHTS)))
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--video", action="store_true", help="write videos of seed 1")
    a = ap.parse_args()
    names, heights = a.sites or list(SITES), [int(h) for h in a.heights.split(",")]
    os.makedirs(VIDEOS, exist_ok=True)
    tag = "_" + "_".join(a.sites) if a.sites else ""
    jobs = [(n, h, seed, 20.0, a.video and seed == 1) for n in names for h in heights for seed in range(1, a.seeds + 1)]
    with mp.get_context("spawn").Pool(max(1, min(len(jobs), (os.cpu_count() or 2) - 2))) as pool:
        res = pool.map(_job, jobs)
    print(f"{a.seeds} seeds x 20 s per row, PAL {FPS:g} fps, video noise {NOISE} grey (x-correlation {NOISE_CORR_X}); "
          "false-fix / out-50 = share of sent fixes outside the 99 % / 50 % bound of cov (ideal 1 % / 50 %)")
    table, views = {}, {}
    for name in names:
        for h0 in heights:
            runs = [r for (r, _), j in zip(res, jobs) if j[0] == name and j[1] == h0]
            views[name, h0] = next(v for (r, v), j in zip(res, jobs) if j[0] == name and j[1] == h0)
            r = table[name, h0] = pooled(runs)
            print(f"{name:15s} {h0:4d} m  fixes sent {r['used']:5.1%}  lost {r['lost']:5.1%}  rms {r['rms']:6.2f} m  "
                  f"p95 {r['p95']:6.2f} m  worst {r['worst']:6.2f} m  false-fix {r['false_fix']:5.1%}  out-50 {r['out50']:5.1%}  "
                  f"rms per seed {' '.join(f'{x:.2f}' for x in r['rms_seed'])}", flush=True)
        if a.video:
            montage([os.path.join(VIDEOS, f"{name}_{h}m.mp4") for h in heights],
                    os.path.join(VIDEOS, f"{name}_all_heights.mp4"))

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(12, 4.5))
    for name, c in zip(names, ("#2ea043", "#b08800", "#1f6feb", "#8250df", "#d1242f")):
        ax_a.plot(heights, [table[name, h]["rms"] for h in heights], "o-", color=c, label=name)
        ax_b.plot(heights, [100 * table[name, h]["used"] for h in heights], "o-", color=c, label=name)
    floor = np.array(heights) * math.radians(0.1)
    ax_a.plot(heights, floor, "k--", lw=1, label="best possible at 0.1° tilt error")
    ax_a.set(xlabel="hover height (m)", ylabel="RMS position error of sent fixes (m)",
             title=f"Hold error on real aerial photos (sim, {a.seeds} seeds)")
    ax_b.set(xlabel="hover height (m)", ylabel="frames with a position sent (%)", ylim=(0, 105),
             title="How often the hold has a position")
    for ax in (ax_a, ax_b):
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(S.HERE, f"sim_ground{tag}.png"), dpi=120)

    fig, ax = plt.subplots(len(names), len(heights), figsize=(2.6 * len(heights), 2.0 * len(names)), squeeze=False)
    for i, name in enumerate(names):
        for j, h0 in enumerate(heights):
            ax[i, j].imshow(views[name, h0], cmap="gray", vmin=0, vmax=255)
            ax[i, j].set_xticks([])
            ax[i, j].set_yticks([])
            if i == 0:
                ax[i, j].set_title(f"{h0} m")
        ax[i, 0].set_ylabel(name)
    fig.suptitle("What the camera sees on real ground (one frame per run)", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(S.HERE, f"sim_ground_view{tag}.png"), dpi=110)
    print(f"wrote sim_ground{tag}.png and sim_ground_view{tag}.png")


if __name__ == "__main__":
    main()
