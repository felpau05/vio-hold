"""Shared code for the hold.py version 2 tests.

Scenes on sim_ground's photo ground (PAL 25 fps, correlated noise 3.7 grey, unseen vibration), the
FC feeds of sim_test, and a runner that logs what Hold sends: error, Mahalanobis distance under
the sent covariance, and how many keyframes were made (confirmed, not pending).
"""
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
if PROJ not in sys.path:
    sys.path.insert(0, PROJ)

import cv2                   # noqa: E402
import numpy as np           # noqa: E402

import sim_ground as G       # noqa: E402
import sim_test as S         # noqa: E402
from hold import Hold        # noqa: E402

# HOLD_CFG='{"row_norm": true}': run the tests with other Hold defaults (also in the spawned workers)
for _k, _v in __import__("json").loads(os.environ.get("HOLD_CFG", "{}")).items():
    setattr(Hold, _k.upper(), _v)

CHI2_99 = 9.21
_SITES = {}


def site(name):
    if name not in _SITES:
        _SITES[name] = G.load(name)
    return _SITES[name]


def render(t0, pos_fn, att_fn, h_fn, pyr, mpp, ground_fx=None):
    """sim_ground.render with a hook: ground_fx(img, P, tr) edits the picture in ground coordinates
    (P: north/east of each pixel, tr: row times) before blur and noise.
    shortcut: copies sim_ground.render's ray maths; keep the two the same."""
    tr = t0 + np.arange(S.H) / S.H * S.READOUT
    b = np.einsum("rij,rwj->rwi", S.euler_R(*att_fn(tr).T), S.RAYS_B)
    h = h_fn(tr)
    P = pos_fn(tr)[:, None, :] + (h[:, None] / b[..., 2])[..., None] * b[..., :2]
    lvl = min(len(pyr) - 1, max(0, int(math.floor(math.log2(h.mean() / S.K[0, 0] / mpp)))))
    tex, m = pyr[lvl], 1.0 / (mpp * 2 ** lvl)
    img = cv2.remap(tex, (P[..., 1] * m + tex.shape[1] / 2).astype(np.float32),
                    (-P[..., 0] * m + tex.shape[0] / 2).astype(np.float32), cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REFLECT101)
    if ground_fx is not None:
        img = ground_fx(img, P, tr)
    img = cv2.GaussianBlur(img, (3, 3), 0.8) + G.video_noise(img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def scene(name="ottawa2019fall", h0=100, T=15.0, seed=1, pos_fn=None, att_fn=None, h_fn=None, h_baro_fn=None,
          ground_fx=None, img_mod=None, fps=G.FPS):
    """Frames, FC feeds and the true path of one hover. img_mod(t0, img) -> picture actually delivered."""
    S.rng = np.random.default_rng(seed)
    p, a, hf = S.hover(h0)
    pos_fn, att_fn, h_fn = pos_fn or p, att_fn or a, h_fn or hf
    pyr, mpp = site(name)
    seen = G.with_vibration(att_fn)
    frames = []
    for t0 in np.arange(0, T, 1 / fps):
        img = render(t0, pos_fn, seen, h_fn, pyr, mpp, ground_fx)
        frames.append((t0, img if img_mod is None else img_mod(t0, img)))
    att, hgt = S.fc_feeds(att_fn, h_baro_fn or h_fn, T)
    return frames, att, hgt, pos_fn


def run(frames, att, hgt, pos_fn, hold=None, reanchor_err=None, t_err=S.FRAME_T_ERR):
    """Feed the frames to Hold. reanchor_err: None = no outside help after 'lost' (the default)."""
    hold = hold or Hold(S.K, S.D, S.R_BC_NADIR, S.READOUT)
    hold.reanchor(pos_fn(np.array([frames[0][0]]))[0])
    rows, kfs = [], set()
    for t0, img in frames:
        e = hold.process(img, t0 + t_err, att, hgt)
        if e.state == "lost" and reanchor_err is not None:
            hold.reanchor(pos_fn(np.array([e.t]))[0] + reanchor_err)
        kf = getattr(hold, "kf", None)
        if kf is not None and not kf.get("pending", False):
            kfs.add(id(kf))
        d = e.pos - pos_fn(np.array([e.t]))[0]
        cov = getattr(e, "cov", None)
        if e.var is not None and cov is None:
            cov = np.eye(2) * e.var / 2
        rows.append((e.t, e.state, e.why or "", e.var is not None, d[0], d[1],
                     float(d @ np.linalg.solve(cov, d)) if e.var is not None else np.nan, e.n, len(kfs),
                     float(np.trace(cov)) if e.var is not None else np.nan))
    return Rec(rows)


class Rec:
    def __init__(self, rows):
        cols = list(zip(*rows))
        self.t = np.array(cols[0])
        self.state, self.why = np.array(cols[1], object), np.array(cols[2], object)
        self.sent = np.array(cols[3], bool)
        self.ex, self.ey = np.array(cols[4]), np.array(cols[5])
        self.err = np.where(self.sent, np.hypot(self.ex, self.ey), np.nan)
        self.m2, self.n, self.kfs, self.var = np.array(cols[6]), np.array(cols[7]), np.array(cols[8]), np.array(cols[9])

    def stats(self, sel=None):
        sel = np.ones(len(self.t), bool) if sel is None else sel
        s = sel & self.sent
        out = dict(frames=int(sel.sum()), sent=int(s.sum()), sent_share=float(s.sum() / max(sel.sum(), 1)))
        if s.any():
            out.update(rms=float(np.sqrt(np.mean(self.err[s] ** 2))), worst=float(self.err[s].max()),
                       false_fix=float(np.mean(self.m2[s] > CHI2_99)),
                       conf_wrong=int(np.sum(s & (self.err > 2.0) & (self.m2 > CHI2_99))))
        return out


def _init():
    cv2.setNumThreads(1)


def pmap(fn, jobs):
    """fn(**job) for each job in worker processes; fn must be a module-level function."""
    with mp.get_context("spawn").Pool(max(1, min(len(jobs), (os.cpu_count() or 2) - 2)), initializer=_init) as pool:
        return pool.starmap(_call, [(fn, j) for j in jobs])


def _call(fn, job):
    return fn(**job)


def pooled(recs, sel_fn=None):
    """Stats over several runs; sel_fn(rec) -> frame selection."""
    s = [r.stats(sel_fn(r) if sel_fn else None) for r in recs]
    frames, sent = sum(x["frames"] for x in s), sum(x["sent"] for x in s)
    out = dict(frames=frames, sent=sent, sent_share=sent / max(frames, 1))
    err = np.concatenate([r.err[(sel_fn(r) if sel_fn else True) & r.sent] for r in recs])
    m2 = np.concatenate([r.m2[(sel_fn(r) if sel_fn else True) & r.sent] for r in recs])
    if len(err):
        out.update(rms=float(np.sqrt(np.mean(err ** 2))), worst=float(err.max()), false_fix=float(np.mean(m2 > CHI2_99)),
                   conf_wrong=int(np.sum((err > 2.0) & (m2 > CHI2_99))))
    return out
