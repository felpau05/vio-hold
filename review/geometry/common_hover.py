"""Shared open-loop hover runner for the review scripts (own renderer, project Hold)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I          # noqa: E402
from hold import Hold      # noqa: E402

s = np.sin


def hover_path(h0, wobble_deg=1.0, yaw_amp_deg=0.0, still=False):
    """Small drift and a tilt wobble like a windy hover (own numbers, similar size to sim_test)."""
    if still:
        pos = lambda t: np.zeros((len(t), 2))
    else:
        pos = lambda t: np.c_[1.0 * s(0.21 * t) + 0.3 * s(0.9 * t + 1), 1.0 * s(0.17 * t + 2) + 0.3 * s(1.1 * t)]
    w = math.radians(wobble_deg)
    att = lambda t: np.c_[w * (4 * s(2 * np.pi * 0.7 * t) + 2 * s(2 * np.pi * 1.9 * t + 1) + 0.2 * s(2 * np.pi * 15 * t)),
                          w * (4 * s(2 * np.pi * 0.6 * t + 2) + 2 * s(2 * np.pi * 2.3 * t) + 0.2 * s(2 * np.pi * 13 * t + 1)),
                          math.radians(yaw_amp_deg) * s(2 * np.pi * 0.05 * t)]
    hf = lambda t: np.full(len(t), float(h0))
    return pos, att, hf


def run(h0, pos, att_fn, h_fn, T=10.0, K_true=None, D_true=None, K_est=None, D_est=None,
        ro_true=0.016, ro_est=None, reverse=False, t0_shift=0.0, tex=None, R_true=None,
        att_err_fn=None, h_err_fn=None, cy_alt=0.0, kf_at=None, span=None, offset=(0.0, 0.0), seed=5):
    """Render T seconds, run Hold, return per-frame (t, state, err_N, err_E) and the Hold."""
    K_true = I.K_FIELD if K_true is None else K_true
    D_true = I.D_FIELD if D_true is None else D_true
    K_est = K_true if K_est is None else K_est
    D_est = D_true if D_est is None else D_est
    ro_est = ro_true if ro_est is None else ro_est
    R_true = I.mount((1, 0, 0)) if R_true is None else R_true
    tex = I.make_texture(seed=seed) if tex is None else tex
    span = 8 * h0 if span is None else span
    off = np.asarray(offset, float)
    posw = lambda t: pos(t) + off
    rnds = {}
    for dc in {0.0, cy_alt, -cy_alt}:
        Kt = K_true.copy()
        Kt[1, 2] += dc
        rnds[dc] = I.Renderer(Kt, D_true, R_true, I.W, I.H, tex, span, seed=seed + 1)
    att, hgt = I.feeds(att_fn, h_fn, T, att_err_fn=att_err_fn, h_err_fn=h_err_fn)
    hold = Hold(K_est, D_est, I.mount((1, 0, 0)), ro_est)
    hold.reanchor(posw(np.array([0.0]))[0])
    out = []
    for k, t0 in enumerate(np.arange(0, T, 1 / I.FPS)):
        dc = cy_alt if k % 2 else -cy_alt
        img = rnds[dc].frame(t0, ro_true, posw, att_fn, h_fn, reverse=reverse)
        e = hold.process(img, t0 + t0_shift, att, hgt)
        if kf_at is not None and abs(t0 - kf_at) < 0.5 / I.FPS and e.var is not None:
            hold._new_kf(img, t0 + t0_shift, att, hgt)        # the same call process() makes at line 218
        err = (e.pos - posw(np.array([e.t]))[0]) if e.var is not None else np.array([np.nan, np.nan])
        out.append((e.t, e.state, *err))
    return out, hold


def rms(out, skip=0.0):
    e = np.array([[r[2], r[3]] for r in out if r[0] >= skip], float)
    ok = ~np.isnan(e[:, 0])
    return math.sqrt(np.mean(np.sum(e[ok] ** 2, 1))), ok.mean(), np.nanmax(np.linalg.norm(e, axis=1))
