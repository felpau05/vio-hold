#!/usr/bin/env python3
"""Suggested fix for the yaw/baro lever: solve position, yaw and scale together (2-D similarity fit
of the keyframe points G_i to the current offsets g_i) instead of the mean of C_i = G_i - g_i.

HoldSim below is a copy of Hold.process with only the position solve changed. Same scenario as
t5_yaw_baro_lever.py at 300 m with water on the west half.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t5b_similarity_fix.py
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                     # noqa: E402
import common_hover as CH             # noqa: E402
import hold as HM                     # noqa: E402


def similarity(A, B):
    """C, s, phi minimising sum |A_i - (C + s R(phi) B_i)|^2 (2-D Umeyama)."""
    ma, mb = A.mean(0), B.mean(0)
    a, b = A - ma, B - mb
    phi = math.atan2(np.sum(b[:, 0] * a[:, 1] - b[:, 1] * a[:, 0]), np.sum(b[:, 0] * a[:, 0] + b[:, 1] * a[:, 1]))
    R = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
    s = np.sum(a * (b @ R.T)) / np.sum(b * b)
    return ma - s * R @ mb, s, phi


class HoldSim(HM.Hold):
    def process(self, img, t0, att, hgt):
        tref = t0 + self.readout / 2
        why = self._check_inputs(t0, att, hgt)
        if self.kf is None:
            if why:
                return self._fail(tref, why)
            if not self._new_kf(img, t0, att, hgt):
                return self._fail(tref, "not enough texture")
            return HM.Est(tref, self.pos.copy(), None, self.kf["n0"], "init", False, None)
        kf = self.kf
        p1, s1, _ = cv2.calcOpticalFlowPyrLK(self.prev_img, img, self.prev_px, None, **HM.LK)
        p2, s2, _ = cv2.calcOpticalFlowPyrLK(kf["img"], img, kf["px"], p1.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **HM.LK)
        pb, s3, _ = cv2.calcOpticalFlowPyrLK(img, kf["img"], p2, kf["px"].copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **HM.LK)
        good = (s1 & s2 & s3).ravel().astype(bool)
        good &= np.linalg.norm((pb - kf["px"]).reshape(-1, 2), axis=1) < 0.5
        self.prev_img, self.prev_px = img, p2
        if why:
            return self._fail(tref, why)
        if good.sum() < self.MIN_PTS:
            return self._fail(tref, "few")
        g, t, ok, h = self._rays(p2.reshape(-1, 2)[good], t0, img.shape[0], att, hgt)
        A = (kf["G"][good] + self.vel * (tref - t)[:, None])[ok]
        B = g[ok]
        inl = np.ones(len(A), bool)
        for _ in range(2):                                   # fit, drop outliers, refit
            Cc, s, phi = similarity(A[inl], B[inl])
            R = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
            r = np.linalg.norm(A - (Cc + s * B @ R.T), axis=1)
            inl = r < max(3 * 1.4826 * np.median(r), 0.003 * float(np.mean(h)) + 0.05)
        n = int(inl.sum())
        if n < self.MIN_PTS:
            return self._fail(tref, "few consistent")
        pos = Cc
        if self.t is not None:
            dt = tref - self.t
            if dt > 0:
                self.vel = 0.7 * self.vel + 0.3 * (pos - self.pos) / dt
        self.pos, self.t = pos, tref
        self.last_fit = (s, phi)
        if n < 2 * self.MIN_PTS or (n < 0.5 * kf["n0"] and self.rate < self.KF_RATE):
            self._new_kf(img, t0, att, hgt)
        return HM.Est(tref, pos, 1.0, n, "ok", False, None)


TEX_SHORE = I.make_texture(seed=5, water_half="west")
pos, att, hf = CH.hover_path(300, still=True)
ramp = lambda a: (lambda t: np.clip(t / 10.0, 0, 1) * a)
cases = (("no FC error", {}),
         ("yaw error 0->1 deg", dict(att_err_fn=lambda t: np.c_[0 * t, 0 * t, ramp(math.radians(1.0))(t)])),
         ("baro error 0->1 m", dict(h_err_fn=ramp(1.0))))
for label, kw in cases:
    res = []
    for cls in (HM.Hold, HoldSim):
        CH.Hold = cls
        out, hld = CH.run(300, pos, att, hf, tex=TEX_SHORE, **kw)
        e = np.array([[o[2], o[3]] for o in out if o[0] > 9.5], float)
        res.append(np.nanmean(np.linalg.norm(e, axis=1)))
    print(f"  h 300 m shore, {label:20s} error at end: mean of C_i {res[0]:5.2f} m | similarity fit {res[1]:5.2f} m")
CH.Hold = HM.Hold
