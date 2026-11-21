#!/usr/bin/env python3
"""Replay apb_rec / apb_live --rec chunks through the floor track (apb_lift) and find the camera mount tilt.

  python3 tools/lift_replay.py /path/b_try3_*.npz [--height 1.0] [--tilt R,P] [--sweep]

Why: on the real walks of 2026-11-21 (b_try3) the floor track's height change grew about 0.3 m per metre of forward
walk and stayed near 0 for sideways walks. A simulated camera that sits 10 deg off straight down, while the program
assumes straight down, gives +0.34 m per metre along the tilt and about 0 across it, and reads distances about 13 %
long along it. Thus the mount tilt (settings tilt_deg: roll, pitch of the camera against straight down) is measurable
from any walk at a steady height: with the right tilt the height change does not follow the horizontal move.

Without --sweep: one run with tilt_deg from settings.json (or --tilt), then the walks (moves between still moments)
with their forward, right and height change. With --sweep: the fit h = c + a * forward + b * right for each tilt;
the best tilt makes a and b smallest. Write the result to settings.json tilt_deg by hand (it changes Hold too).
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import apb_est as E          # noqa: E402
import apb_lift as LF        # noqa: E402
import settings as S         # noqa: E402
from apb_main import lens    # noqa: E402
from hold import Series, attitude_R, make_R_bc   # noqa: E402


def attitude(s, paths):
    paths, gyro, accel, _ = E.load_rec(paths)
    att_f, att = E.ApbAtt(s), Series(1e9)
    for r in att_f.update(gyro, accel):
        att.add(*r)
    if not att.rows:
        raise SystemExit("no still moment for the gyro bias in these chunks: add the chunk where the drone was still")
    return paths, att


def frames(paths):
    for p in paths:
        d = np.load(p)
        for img, tf in zip(d["imgs"], d["t_frame"]):
            yield img, float(tf)


def run(s, paths, att, tilt, h0):
    """-> rows (t, forward, right, height change, points, gaps) in the axes of the first yaw."""
    R_bc = make_R_bc(s["top"], s["mirror"], tuple(tilt), s["twist_deg"])
    t_att0 = att.rows[0][0]
    ls, yaw0, out = None, None, []
    for img, tf in frames(paths):
        t = tf - s["video_delay_s"] + 0.5 * s["readout_s"]
        if t < t_att0:
            continue
        if ls is None:
            ls = LF.LiftScale(*lens(s, img.shape), img.shape, h0=h0)
        ls.update(img, attitude_R(att, np.array([t]))[0] @ R_bc)
        if ls.disp is None:
            continue
        if yaw0 is None:
            yaw0 = float(np.interp(t, *np.array(att.rows)[:, [0, 3]].T))
        c, sn = math.cos(yaw0), math.sin(yaw0)
        d = ls.disp
        out.append((t, d[0] * c + d[1] * sn, -d[0] * sn + d[1] * c, h0 * (ls.ratio - 1), ls.n, ls.gaps))
    return np.array(out)


def fit(rows):
    """h = c + a * forward + b * right (least squares) -> (a, b, residual sd)."""
    A = np.c_[rows[:, 1], rows[:, 2], np.ones(len(rows))]
    coef, *_ = np.linalg.lstsq(A, rows[:, 3], rcond=None)
    return float(coef[0]), float(coef[1]), float(np.std(rows[:, 3] - A @ coef))


def walks(rows, still_mps=0.02, min_m=0.2, still_s=0.7):
    """Moves between still moments: (t_start, t_end, d_forward, d_right, d_height)."""
    t, p = rows[:, 0], rows[:, 1:3]
    sp = np.r_[0, np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)]
    still = np.convolve(sp < still_mps, np.ones(5) / 5, "same") > 0.99
    out, i0, k = [], None, 0
    while k < len(t):
        if still[k]:
            j = k
            while j < len(t) and still[j]:
                j += 1
            if i0 is not None and t[j - 1] - t[k] >= still_s * 0 and np.linalg.norm(p[k] - p[i0]) > min_m:
                out.append((t[i0], t[k], *(rows[k, 1:4] - rows[i0, 1:4])))
            i0, k = j - 1, j
        else:
            k += 1
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("chunks", nargs="+")
    ap.add_argument("--height", type=float, default=1.0, help="lens height (m) of the walk")
    ap.add_argument("--tilt", help="roll,pitch (deg) instead of settings tilt_deg")
    ap.add_argument("--sweep", action="store_true", help="find the tilt: pitch then roll, coarse then fine")
    a = ap.parse_args()
    s = E.bench_settings(S.load())
    paths, att = attitude(s, a.chunks)
    tilt = [float(x) for x in a.tilt.split(",")] if a.tilt else list(s["tilt_deg"])
    if a.sweep:
        def score(tl):
            r = run(s, paths, att, tl, a.height)
            ca, cb, sd = fit(r)
            print(f"  tilt roll {tl[0]:+5.1f} pitch {tl[1]:+5.1f}: dh per m forward {ca:+.3f}, per m right {cb:+.3f}, "
                  f"residual sd {sd:.3f} m, {int(r[-1, 5])} gaps", flush=True)
            return ca, cb
        for axis, k in (("pitch", 1), ("roll", 0)):
            for step in (4.0, 1.0):
                grid = [tilt[k] + step * i for i in range(-3, 4)] if step == 1.0 else [step * i for i in range(-5, 6)]
                best = None
                for v in grid:
                    tl = list(tilt)
                    tl[k] = v
                    ca, cb = score(tl)
                    m = abs(ca) + abs(cb)
                    if best is None or m < best[0]:
                        best = (m, v)
                tilt[k] = best[1]
            print(f"best {axis}: {tilt[k]:+.1f} deg")
        print(f"best tilt_deg: [{tilt[0]:.1f}, {tilt[1]:.1f}]")
    r = run(s, paths, att, tilt, a.height)
    ca, cb, sd = fit(r)
    print(f"tilt {tilt}: dh per m forward {ca:+.3f}, per m right {cb:+.3f}, residual sd {sd:.3f} m, "
          f"{len(r)} frames with values, {int(r[-1, 5])} gaps")
    t0 = r[0, 0]
    for ws, we, df, dr, dh in walks(r):
        print(f"  walk {ws - t0:6.1f}-{we - t0:6.1f} s: forward {df:+.2f} right {dr:+.2f} distance {math.hypot(df, dr):.2f} m,"
              f" height change {dh:+.2f} m")


if __name__ == "__main__":
    main()
