"""Rolling-shutter readout from a hand rotation recorded on the APB (apb_rec format).

Method: split the picture into bands of rows. For each band, find the gyro time shift that best predicts
the image motion of that band (as in talk/make_stage2_figures.py fig_offset, but per band). With a rolling
shutter, each band is exposed at a different time, so the best shift grows linearly with the row.
The slope of that line, times the picture height, is the readout time (first row to last row).

  python rs_readout.py <prefix of npz chunks> [out.json]
"""
import glob
import json
import math
import sys

import cv2
import numpy as np

H, W = 540, 960
NB = 6                       # row bands
COLS = (280, 680)            # central columns only (fisheye map is close to linear there)


def load(prefix):
    imgs, ts, gy = [], [], []
    for p in sorted(glob.glob(prefix + "_*.npz")):
        d = np.load(p)
        imgs += list(d["imgs"])
        ts += list(d["t_frame"])
        gy.append(d["gyro"])
    g = np.concatenate(gy)
    g = g[np.argsort(g[:, 0])]
    g = g[np.r_[True, np.diff(g[:, 0]) > 0]]
    return imgs, np.array(ts), g


def integrate(g):
    rate = np.linalg.norm(g[:, 1:], axis=1)
    bias = g[rate < np.percentile(rate, 10), 1:].mean(0)
    w = g[:, 1:] - bias
    cum = np.r_[np.zeros((1, 3)), np.cumsum((w[1:] + w[:-1]) / 2 * np.diff(g[:, 0])[:, None], 0)]
    return g[:, 0], cum, bias


def track(imgs):
    """Per frame pair: points (row, col) in frame k and their motion. Forward-backward check."""
    out = []
    mask = np.zeros((H, W), np.uint8)
    mask[:, COLS[0]:COLS[1]] = 255
    for a, b in zip(imgs[:-1], imgs[1:]):
        p = cv2.goodFeaturesToTrack(a, 400, 0.01, 7, mask=mask)
        if p is None or len(p) < 40:
            out.append(None)
            continue
        q, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p, None, winSize=(21, 21), maxLevel=4)
        r, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, q, None, winSize=(21, 21), maxLevel=4)
        ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((r - p).reshape(-1, 2), axis=1) < 0.7)
        p, q = p.reshape(-1, 2)[ok], q.reshape(-1, 2)[ok]
        out.append((p, q) if len(p) >= 40 else None)
    return out


def band_shifts(tr):
    """Median motion of each row band, for each pair (None where a band has too few points)."""
    edges = np.linspace(0, H, NB + 1)
    S = np.full((len(tr), NB, 2), np.nan)
    for k, t in enumerate(tr):
        if t is None:
            continue
        p, q = t
        row = (p[:, 1] + q[:, 1]) / 2
        for j in range(NB):
            m = (row >= edges[j]) & (row < edges[j + 1])
            if m.sum() >= 8:
                S[k, j] = np.median(q[m] - p[m], 0)
    return S, (edges[:-1] + edges[1:]) / 2


def fit_band(S, ts, tg, cum, pairs, offs):
    """Fit error against gyro time shift for one band. S: (pairs, 2) motion."""
    res = []
    for off in offs:
        a = np.stack([np.interp(ts[pairs] + off, tg, cum[:, i]) for i in range(3)], 1)
        b = np.stack([np.interp(ts[pairs + 1] + off, tg, cum[:, i]) for i in range(3)], 1)
        X, Y = b - a, S
        M, *_ = np.linalg.lstsq(X, Y, rcond=None)
        e = np.linalg.norm(X @ M - Y, axis=1)
        e = e[e <= np.percentile(e, 90)]          # drop the worst 10 % (translation, mistracks)
        res.append(float(np.sqrt(np.mean(e ** 2))))
    res = np.array(res)
    k = int(np.argmin(res))
    if 0 < k < len(res) - 1:                       # parabola through the three best points
        y0, y1, y2 = res[k - 1:k + 2]
        d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
        best = offs[k] + d * (offs[1] - offs[0])
    else:
        best = offs[k]
    return best, res


def readout(S, centres, ts, tg, cum, pairs, offs):
    best = []
    curves = []
    for j in range(NB):
        pj = pairs[~np.isnan(S[pairs, j, 0])]
        b, r = fit_band(S[pj, j], ts, tg, cum, pj, offs)
        best.append(b)
        curves.append(r)
    best = np.array(best)
    slope, icpt = np.polyfit(centres, best, 1)
    return slope * H, icpt, best, curves


def main():
    prefix = sys.argv[1]
    imgs, ts, g = load(prefix)
    tg, cum, bias = integrate(g)
    keep = np.arange(len(imgs) - 1)
    print(f"{len(imgs)} frames, {ts[-1] - ts[0]:.1f} s, gyro bias {np.degrees(bias).round(2)} deg/s")
    tr = track(imgs)
    S, centres = band_shifts(tr)
    # pairs with a clear rotation (> 1 deg between frames) and every band measured
    dth = np.array([np.interp(ts[k + 1], tg, cum[:, 0]) for k in keep])  # placeholder for shape
    rot = np.array([np.linalg.norm([np.interp(ts[k + 1], tg, cum[:, i]) - np.interp(ts[k], tg, cum[:, i])
                                    for i in range(3)]) for k in keep])
    good = keep[(rot > math.radians(1.0)) & (~np.isnan(S[keep, :, 0]).any(1))]
    print(f"pairs with > 1 deg rotation and all {NB} bands measured: {len(good)}")
    offs = np.arange(-0.060, 0.0601, 0.001)
    ro, icpt, best, curves = readout(S, centres, ts, tg, cum, good, offs)
    print("best shift per band (ms):", (best * 1e3).round(1), " rows", centres.round(0))
    print(f"READOUT (first to last row): {ro * 1e3:.1f} ms   shift at row 0: {icpt * 1e3:.1f} ms")
    # independent halves and bootstrap
    halves = []
    for part in (good[good < len(imgs) // 2], good[good >= len(imgs) // 2]):
        halves.append(readout(S, centres, ts, tg, cum, part, offs)[0])
    print("halves (ms):", [round(h * 1e3, 1) for h in halves])
    rng = np.random.default_rng(1)
    boot = [readout(S, centres, ts, tg, cum, np.sort(rng.choice(good, len(good))), offs)[0] for _ in range(40)]
    lo, hi = np.percentile(boot, [5, 95])
    print(f"bootstrap 90 %: {lo * 1e3:.1f} to {hi * 1e3:.1f} ms")
    out = dict(readout_ms=ro * 1e3, shift_row0_ms=icpt * 1e3, band_rows=centres.tolist(),
               band_best_ms=(best * 1e3).tolist(), halves_ms=[h * 1e3 for h in halves],
               boot90_ms=[lo * 1e3, hi * 1e3], pairs=int(len(good)), offs_ms=(offs * 1e3).tolist(),
               curves=[c.tolist() for c in curves], frames=len(imgs))
    if len(sys.argv) > 2:
        json.dump(out, open(sys.argv[2], "w"), indent=1)


if __name__ == "__main__":
    main()
