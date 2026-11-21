#!/usr/bin/env python3
"""Lens calibration with a checkerboard, for the development area (not the event day).

Input: a folder of pictures, a video file, or a .npz recording with imgs (as apb_main records them).
A view counts when it shows the whole board. Output: an equidistant fisheye profile (cv2.fisheye:
fx, fy, cx, cy, k1-k4) at the picture size, added to the lens file under --name. With --settings: the
lens goes into that settings file as lens_measured, marked "by": "target", and focal_scale becomes 1.0.
apb_main.py calib then fits only one focal scale against it (FAIL outside 0.98 to 1.02).

Every fifth usable view is held out: the fit does not use it, and its error shows whether the lens
fits views it has not seen. The tool refuses a result when fewer than MIN_VIEWS views are usable or
when the corners reach less than MIN_COVER_DEG from the optical axis (then k1-k4 rest on the
picture centre only). The square size does not change the lens, so the board uses square = 1.
Run: ~/.venvs/vio/bin/python tools/lens_calib.py <folder | video | rec.npz> --name "Justice D board 2026-11-18" [--board 9x6]
     ~/.venvs/vio/bin/python tools/lens_calib.py <folder | video | rec.npz> --settings settings.json
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJ)
from calib import add_profile    # noqa: E402
import settings as S             # noqa: E402

MIN_VIEWS = 10
MIN_COVER_DEG = 40.0          # assumed: k1-k4 need corners this far off axis (the picture corner is 60-75 deg)
HOLD_OUT = 5                  # every fifth usable view is held out
PICTURES = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


def pictures(src, step):
    """Grey pictures from a folder, a video file (every step-th frame) or a .npz recording."""
    if os.path.isdir(src):
        imgs = [cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in sorted(glob.glob(os.path.join(src, "*")))
                if f.lower().endswith(PICTURES)]
    elif src.endswith(".npz"):
        imgs = list(np.load(src)["imgs"])
    else:
        cap, imgs, i = cv2.VideoCapture(src), [], 0
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            if i % step == 0:
                imgs.append(fr)
            i += 1
    return [cv2.cvtColor(g, cv2.COLOR_BGR2GRAY) if g.ndim == 3 else g for g in imgs if g is not None]


def main(argv=None, say=print):
    ap = argparse.ArgumentParser(description="Equidistant fisheye lens profile from checkerboard views.")
    ap.add_argument("source", help="folder of pictures, video file, or .npz recording with imgs")
    ap.add_argument("--name", help="name of the new profile in the lens file (not with --settings)")
    ap.add_argument("--settings", help="write the lens into this settings file as lens_measured (by target), not into the lens file")
    ap.add_argument("--board", default="9x6", help="inner corners across x down (default 9x6)")
    ap.add_argument("--lens-file", default=os.path.join(PROJ, "justice_lens_profiles.json"))
    ap.add_argument("--step", type=int, default=15, help="video only: use every n-th frame (default 15)")
    a = ap.parse_args(argv)
    if bool(a.name) == bool(a.settings):
        ap.error("give --name (a profile in the lens file) or --settings (lens_measured in a settings file), not both")
    board = tuple(int(x) for x in a.board.lower().split("x"))
    imgs = pictures(a.source, a.step)
    if not imgs or len({g.shape for g in imgs}) != 1:
        say(f"FAIL {a.source} gives {len(imgs)} pictures in {len({g.shape for g in imgs})} sizes. Give a folder, a video "
            "file or a .npz recording with pictures of one size. The tool wrote no profile.")
        return 1
    h, w = imgs[0].shape
    found = []
    for g in imgs:
        ok, c = cv2.findChessboardCornersSB(g, board, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            found.append(c.reshape(1, -1, 2).astype(np.float64))      # cv2.fisheye.calibrate takes 1 x N only
    say(f"INFO {len(imgs)} pictures {w}x{h}. The whole {board[0]}x{board[1]} board is in {len(found)} of them.")
    if len(found) < MIN_VIEWS:
        say(f"FAIL only {len(found)} views show the whole board. The fit needs {MIN_VIEWS}. Record more views with the "
            "whole board in the picture. The tool wrote no profile.")
        return 1
    obj = np.c_[np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2), np.zeros(board[0] * board[1])].reshape(1, -1, 3)
    test = [c for i, c in enumerate(found) if i % HOLD_OUT == HOLD_OUT - 1]
    fit = [c for i, c in enumerate(found) if i % HOLD_OUT != HOLD_OUT - 1]
    try:
        _, K, D, rvs, tvs = cv2.fisheye.calibrate([obj] * len(fit), fit, (w, h), np.zeros((3, 3)), np.zeros(4),
                                                 flags=cv2.CALIB_RECOMPUTE_EXTRINSIC | cv2.CALIB_FIX_SKEW,
                                                 criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-12))
    except cv2.error as e:
        say(f"FAIL the fit did not converge ({str(e).strip().splitlines()[-1]}). Record views with the board at "
            "several angles and distances. The tool wrote no profile.")
        return 1
    D = D.ravel()

    def err2(c, rv, tv):
        return np.sum((cv2.fisheye.projectPoints(obj, rv, tv, K, D)[0] - c) ** 2, axis=2).ravel()

    rms = float(np.sqrt(np.mean(np.concatenate([err2(c, rv, tv) for c, rv, tv in zip(fit, rvs, tvs)]))))
    held = float(np.sqrt(np.mean(np.concatenate([err2(c, *cv2.fisheye.solvePnP(obj, c, K, D)[1:]) for c in test]))))
    n = cv2.fisheye.undistortPoints(np.concatenate(found, axis=1), K, D).reshape(-1, 2)
    cover = float(np.degrees(np.arctan(np.hypot(n[:, 0], n[:, 1])).max()))
    say(f"INFO lens at {w}x{h}: fx {K[0, 0]:.2f} fy {K[1, 1]:.2f} cx {K[0, 2]:.2f} cy {K[1, 2]:.2f} "
        f"k1 {D[0]:+.5f} k2 {D[1]:+.5f} k3 {D[2]:+.5f} k4 {D[3]:+.5f}")
    say(f"INFO RMS error {rms:.3f} px on {len(fit)} fit views, {held:.3f} px on {len(test)} views that the fit did not use")
    say(f"INFO corners reach {cover:.1f} deg from the optical axis")
    if cover < MIN_COVER_DEG:
        say(f"FAIL the corners reach only {cover:.0f} deg from the optical axis. The fit needs {MIN_COVER_DEG:.0f} deg. "
            "Put the board near the picture edges and corners too. Then run the tool again. The tool wrote no profile.")
        return 1
    p = dict(name=a.name, projection="equidistant_fisheye", image_size=[w, h], fx=float(K[0, 0]), fy=float(K[1, 1]),
             cx=float(K[0, 2]), cy=float(K[1, 2]), b2=0.0, p1=0.0, p2=0.0,
             **{f"k{i + 1}": float(D[i]) for i in range(4)}, radial_turnover_deg_through_80=None,
             note=f"tools/lens_calib.py {time.strftime('%Y-%m-%d %H:%M')}: {len(fit)} fit views, RMS {rms:.3f} px, "
                  f"{len(test)} held-out views {held:.3f} px, corners to {cover:.0f} deg")
    if a.settings:
        try:
            s = S.load(a.settings)
            s["lens_measured"] = dict(by="target", image_size=[w, h], **{k: p[k] for k in S.LENS_KEYS}, note=p["note"])
            s["focal_scale"] = 1.0           # a focal scale measured against the old lens does not fit this one
            S.save(s, a.settings)
        except (ValueError, OSError) as e:
            say(f"FAIL {e} The tool wrote no lens.")
            return 1
        say(f"PASS lens written to {a.settings} as lens_measured (measured with a target). focal_scale is 1.0. Copy the "
            "settings to the board. Then run apb_main.py calib: it fits one focal scale against this lens.")
        return 0
    try:
        add_profile(a.lens_file, p)
    except ValueError as e:
        say(f"FAIL {e} The tool wrote no profile.")
        return 1
    say(f"PASS profile '{a.name}' added to {a.lens_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
