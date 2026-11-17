#!/usr/bin/env python3
"""The forward-backward check in Hold.process starts the backward LK AT the keyframe point it is
supposed to return to (nextPts = kf["px"], OPTFLOW_USE_INITIAL_FLOW). LK then only refines a
guess that is already the answer, so the 0.5 px test passes almost whatever p2 is.

Test on three picture pairs (keyframe -> current), 300 corners each:
  ground   two sim_ground frames 1 s apart at 100 m (true matches)
  wrong    the same, but every forward result p2 moved 3 px (all matches wrong)
  static   two independent frames of analog static, std 10 (nothing to match)
For each: share passing Hold's check (backward started at kf px) and a standard check
(backward started at p2 itself, i.e. no hint).
Run: ~/.venvs/vio/bin/python e4b_fb_check.py      (about 10 s)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import common as C  # noqa: E402

LK = C.Hm.LK
S = C.S


def checks(kf, cur, px, shift=0.0):
    p1, s1, _ = cv2.calcOpticalFlowPyrLK(kf, cur, px, None, **LK)
    p2 = p1 + np.float32(shift)
    # Hold: backward started at the keyframe points
    pb, s3, _ = cv2.calcOpticalFlowPyrLK(cur, kf, p2, px.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **LK)
    hold_ok = (s3.ravel() == 1) & (np.linalg.norm((pb - px).reshape(-1, 2), axis=1) < 0.5)
    # standard: backward started at p2 (no hint)
    pb2, s4, _ = cv2.calcOpticalFlowPyrLK(cur, kf, p2, None, **LK)
    std_ok = (s4.ravel() == 1) & (np.linalg.norm((pb2 - px).reshape(-1, 2), axis=1) < 0.5)
    fwd = s1.ravel() == 1
    return fwd.mean(), (fwd & hold_ok).mean(), (fwd & std_ok).mean()


if __name__ == "__main__":
    C.reseed(7)
    pyr, mpp = C.site("ottawa2019fall")
    pos, att, h = S.hover(100)
    seen = C.with_vibration(att, 0.2)
    a = C.render(2.0, pos, seen, h, pyr, mpp, 5.0)
    b = C.render(3.0, pos, seen, h, pyr, mpp, 5.0)
    px = cv2.goodFeaturesToTrack(a, 300, 0.01, 7)
    st1 = np.clip(90 + S.rng.normal(0, 10, a.shape), 0, 255).astype(np.uint8)
    st2 = np.clip(90 + S.rng.normal(0, 10, a.shape), 0, 255).astype(np.uint8)
    pxs = cv2.goodFeaturesToTrack(st1, 300, 0.01, 7)
    _, ss, _ = cv2.calcOpticalFlowPyrLK(st1, st1, pxs, None, **LK)
    pxs = pxs[ss.ravel() == 1]
    print(f"static std 10: {len(pxs)} of 300 corners pass Hold's texture self-test (minEigThreshold 3e-3)")
    print(" pair              forward LK ok   pass Hold's back check   pass standard back check")
    for name, (k, c, p, sh) in {"ground (true)": (a, b, px, 0.0), "ground, p2 +3 px": (a, b, px, 3.0),
                                "ground, p2 +1.5 px": (a, b, px, 1.5),
                                "static std 10": (st1, st2, pxs, 0.0)}.items():
        f, ho, so = checks(k, c, p, sh)
        print(f" {name:18s}   {f:6.1%}            {ho:6.1%}                    {so:6.1%}")

    # LK's third output (mean abs patch difference per pixel), which Hold drops, and patch NCC
    def ncc(k, c, p0, p1, r=5):
        out = []
        for (x0, y0), (x1, y1) in zip(p0.reshape(-1, 2), p1.reshape(-1, 2)):
            a_ = cv2.getRectSubPix(k, (2 * r + 1, 2 * r + 1), (float(x0), float(y0))).astype(float)
            b_ = cv2.getRectSubPix(c, (2 * r + 1, 2 * r + 1), (float(x1), float(y1))).astype(float)
            a_, b_ = a_ - a_.mean(), b_ - b_.mean()
            out.append((a_ * b_).sum() / (np.sqrt((a_ ** 2).sum() * (b_ ** 2).sum()) + 1e-9))
        return np.array(out)
    print("\n LK err (mean |patch diff|, grey levels) and patch NCC of the forward match (median, 5th-95th pct):")
    st3 = np.clip(90 + S.rng.normal(0, 30, a.shape), 0, 255).astype(np.uint8)
    st4 = np.clip(90 + S.rng.normal(0, 30, a.shape), 0, 255).astype(np.uint8)
    pxs3 = cv2.goodFeaturesToTrack(st3, 300, 0.01, 7)
    low = (128 + 0.1 * (a.astype(float) - 128) + S.rng.normal(0, 5, a.shape)).clip(0, 255).astype(np.uint8)
    lowb = (128 + 0.1 * (b.astype(float) - 128) + S.rng.normal(0, 5, a.shape)).clip(0, 255).astype(np.uint8)
    pxl = cv2.goodFeaturesToTrack(low, 300, 0.01, 7)
    for name, (k, c, p) in {"ground 100 %": (a, b, px), "ground 10 % contrast": (low, lowb, pxl),
                            "static std 10": (st1, st2, pxs), "static std 30": (st3, st4, pxs3)}.items():
        p1, s1, err = cv2.calcOpticalFlowPyrLK(k, c, p, None, **LK)
        ok = s1.ravel() == 1
        e = err.ravel()[ok]
        nc = ncc(k, c, p[ok], p1[ok])
        print(f"   {name:22s} err {np.median(e):5.1f} ({np.percentile(e, 5):4.1f}-{np.percentile(e, 95):4.1f})   "
              f"NCC {np.median(nc):5.2f} ({np.percentile(nc, 5):5.2f}-{np.percentile(nc, 95):5.2f})")
