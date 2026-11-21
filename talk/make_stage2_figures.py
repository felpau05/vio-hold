"""Stage 2 talk figures from data measured on the event aircraft (2026-11-21). Re-run after new walks or flights.

  ~/.venvs/vio/bin/python talk/make_stage2_figures.py      -> talk/stage2/*.png

Inputs (local, not in git): logs/rec/ceiling_123134_*.npz (gyro + frames, hand rotation under the ceiling),
logs/rec/desk_check_105007_*.npz (frame stamps), justice_lens_profiles.json, numbers measured today (cited inline).
"""
import glob
import json
import math
import os

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "talk", "stage2")
REC = os.path.join(ROOT, "logs", "rec")
# reference palette (dataviz skill): categorical slots in fixed order, text inks, surface
C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF, "font.size": 13,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2, "text.color": INK,
    "axes.grid": True, "axes.axisbelow": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "axes.titleweight": "bold", "axes.titlesize": 15, "axes.titlelocation": "left",
    "lines.linewidth": 2.0, "legend.frameon": False,
})


def save(fig, name, source):
    fig.text(0.01, 0.01, source, fontsize=9, color=INK2, ha="left", va="bottom")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    os.makedirs(OUT, exist_ok=True)
    fig.savefig(os.path.join(OUT, name), dpi=160)
    plt.close(fig)
    print("wrote", name)


# ---------------------------------------------------------------- 1. gyro-camera time offset (ceiling rotation)
def fig_offset():
    paths = sorted(glob.glob(os.path.join(REC, "ceiling_123134_*.npz")))
    if not paths:
        print("skip offset: no ceiling recording")
        return
    imgs, ts, gy = [], [], []
    for p in paths:
        d = np.load(p)
        imgs += list(d["imgs"])
        ts += list(d["t_frame"])
        gy.append(d["gyro"])
    ts = np.array(ts)
    keep = (ts - ts[0] >= 2) & (ts - ts[0] <= 33)
    imgs = [im for im, k in zip(imgs, keep) if k]
    ts = ts[keep]
    g = np.concatenate(gy)
    g = g[np.argsort(g[:, 0])]
    g = g[np.r_[True, np.diff(g[:, 0]) > 0]]
    rate = np.linalg.norm(g[:, 1:], axis=1)
    bias = g[rate < np.percentile(rate, 10), 1:].mean(0)
    cum = np.r_[np.zeros((1, 3)), np.cumsum((g[1:, 1:] + g[:-1, 1:]) / 2 * np.diff(g[:, 0])[:, None], 0)] \
        - (g[:, 0] - g[0, 0])[:, None] * bias
    yy, xx = np.mgrid[0:540, 0:960]
    mask = (((xx - 464) ** 2 + (yy - 269) ** 2) < 110 ** 2).astype(np.uint8) * 255
    shifts = []
    for a, b in zip(imgs[:-1], imgs[1:]):
        p = cv2.goodFeaturesToTrack(a, 80, 0.01, 8, mask=mask)
        if p is None or len(p) < 12:
            shifts.append(None)
            continue
        q, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p, None, winSize=(21, 21), maxLevel=3)
        d = (q - p).reshape(-1, 2)[st.ravel() == 1]
        shifts.append(np.median(d, 0) if len(d) >= 10 else None)
    offs, res = np.arange(-0.06, 0.0601, 0.002), []
    for off in offs:
        X, Y = [], []
        for k, s in enumerate(shifts):
            if s is None:
                continue
            dth = np.array([np.interp(ts[k + 1] + off, g[:, 0], cum[:, i]) - np.interp(ts[k] + off, g[:, 0], cum[:, i])
                            for i in range(3)])
            if np.linalg.norm(dth) < math.radians(0.15):
                continue
            X.append(dth)
            Y.append(s)
        X, Y = np.array(X), np.array(Y)
        M, *_ = np.linalg.lstsq(X, Y, rcond=None)
        res.append(float(np.sqrt(np.mean((X @ M - Y) ** 2))))
    res = np.array(res)
    k = int(np.argmin(res))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(offs * 1e3, res, color=C1)
    ax.plot([offs[k] * 1e3], [res[k]], "o", ms=9, color=C1, mec=SURF, mew=2)
    ax.annotate(f"best {offs[k] * 1e3:+.0f} ms: {res[k]:.2f} px", (offs[k] * 1e3, res[k]), xytext=(18, 30),
                textcoords="offset points", color=INK, arrowprops=dict(arrowstyle="-", color=INK2))
    ax.set_xlabel("gyro time shift against the frame stamps (ms)")
    ax.set_ylabel("image-motion fit error (px)")
    ax.set_title("Camera and gyro clocks line up within 12 ms")
    save(fig, "01_gyro_camera_offset.png",
         f"Hand rotation under the ceiling, {len(Y)} frame pairs; rotation predicts the motion of central features")


# ---------------------------------------------------------------- 2. frame stamps
def fig_stamps():
    paths = sorted(glob.glob(os.path.join(REC, "desk_check_105007_*.npz")))
    if not paths:
        print("skip stamps")
        return
    t = np.concatenate([np.load(p)["t_frame"] for p in paths])
    s = np.concatenate([np.load(p)["seq"] for p in paths])
    dt = np.diff(t) / np.maximum(np.diff(s), 1) * 1e3          # per driver frame
    dt = dt[np.isfinite(dt) & (np.diff(s) > 0)]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.hist(dt, bins=np.linspace(16.4, 16.9, 51), color=C1, edgecolor=SURF, linewidth=1)
    ax.set_xlabel("time between driver frames (ms)")
    ax.set_ylabel("frame pairs")
    ax.set_title(f"Frame stamps: {np.median(dt):.3f} ms apart, spread {np.std(dt) * 1e3:.0f} µs")
    save(fig, "02_frame_stamps.png", f"Justice D on /dev/video2, driver stamps (CLOCK_MONOTONIC, end of frame), "
                                     f"{len(dt)} frame pairs from the desk check")


# ---------------------------------------------------------------- 3. lens: spec vs measured
def fig_lens():
    prof = {m["name"]: m for m in json.load(open(os.path.join(ROOT, "justice_lens_profiles.json")))["models"]}
    screen = prof["Justice D 1080p board 2026-11-21"]
    nominal = dict(prof["Justice D nominal (spec FOV, uncalibrated)"])
    nominal["fx"] *= 960 / nominal["image_size"][0]           # same picture extent at 960 px width
    paper = dict(fx=442.68, k1=-0.07162, k2=0.05930, k3=-0.07267, k4=0.02782)   # second round (paper, not flat)

    def ppd(m, th):
        t = np.radians(th)
        d = lambda x: x * (1 + m["k1"] * x**2 + m["k2"] * x**4 + m["k3"] * x**6 + m["k4"] * x**8)  # noqa: E731
        return m["fx"] * (d(t + 1e-4) - d(t - 1e-4)) / 2e-4 * math.pi / 180
    th = np.linspace(0, 60, 121)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(th, [ppd(nominal, x) for x in th], color=C2, label="datasheet (114 deg)")
    ax.plot(th, [ppd(screen, x) for x in th], color=C1, label="checkerboard, flat screen (used)")
    ax.plot(th, [ppd(paper, x) for x in th], color=C4, label="checkerboard, paper (not flat)")
    ax.errorbar([4], [(392.4 + 401.0) / 2 * math.pi / 180], yerr=[[0.08], [0.08]], fmt="o", ms=9, color=C3,
                mec=SURF, mew=2, capsize=4, label="gyro rotation (no target)")
    ax.set_xlabel("angle from the lens axis (deg)")
    ax.set_ylabel("pixels per degree")
    ax.set_ylim(4.5, 9)
    ax.legend(loc="lower left")
    ax.set_title("Datasheet lens 17 % off; the gyro sides with the flat board")
    save(fig, "03_lens_measured.png", "960x540 picture. Screen board: 0.17 px error on held-out views. "
                                     "Gyro: pure rotation under the ceiling, central features")


# ---------------------------------------------------------------- 4. first walk
def fig_walk():
    fig, ax = plt.subplots(figsize=(9, 4.5))
    legs = ["S to F\n(1.00 m)", "S to R\n(1.50 m)"]
    truth, raw, hold = [1.00, 1.50], [0.84, 0.72], [0.46, 0.17]
    x, w = np.arange(2), 0.26
    for i, (v, c, lab) in enumerate(((truth, INK2, "tape measure"), (raw, C3, "raw floor tracking"),
                                     (hold, C1, "estimator"))):
        b = ax.bar(x + (i - 1) * w, v, w * 0.92, color=c, label=lab)
        ax.bar_label(b, fmt="%.2f", padding=3, color=INK, fontsize=11)
    ax.set_xticks(x, legs)
    ax.set_ylabel("distance (m)")
    ax.set_ylim(0, 2.0)
    ax.legend(loc="upper left", ncol=3)
    ax.set_title("First walk: the motion is in the pictures, the estimator lost it")
    save(fig, "04_first_walk.png", "Hand-carried at ~1 m, plain glossy floor, fixed 1.0 m height. READY: 15 % live, "
                                   "75 % on replay without the analog-only checks")


# ---------------------------------------------------------------- 5. compute on the A53
def fig_compute():
    fig, ax = plt.subplots(figsize=(8, 4.5))
    items = [("picture checks + sensors,\nfirst version", 27.0, C2), ("picture checks + sensors,\noptimized", 13.0, C1),
             ("estimator (tracking, fit)", 47.0, C3)]
    y = np.arange(len(items))[::-1]
    b = ax.barh(y, [v for _, v, _ in items], 0.6, color=[c for _, _, c in items])
    ax.bar_label(b, fmt="%.0f ms", padding=4, color=INK)
    ax.set_yticks(y, [n for n, _, _ in items])
    ax.set_xlabel("time per frame on the APB's A53 (ms)")
    ax.set_xlim(0, 60)
    ax.set_title("Where the time goes on the APB")
    save(fig, "05_compute.png", "Measured on the board. Camera copy (uncached buffers, 12 ms) runs in its own "
                                "thread; 13-16 position fixes per second")


if __name__ == "__main__":
    fig_stamps()
    fig_lens()
    fig_walk()
    fig_compute()
    fig_offset()
