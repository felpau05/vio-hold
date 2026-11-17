"""Shared code for the hardware-gap review scripts.

Reuses the project's simulator (sim_test, sim_ground) and estimator (hold) unchanged, but lets each
script break one hardware assumption at a time: the true camera (lens, mounting, readout, field
timing) can differ from the model the estimator is given, and the flight-controller feeds can be
slow, late or biased. Runs from any directory.
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, "..", ".."))
if PROJ not in sys.path:
    sys.path.insert(0, PROJ)

import sim_test as S                       # noqa: E402  (builds the synthetic texture)
from hold import Hold, Series, euler_R, R_BC_NADIR, load_lens, rot_z   # noqa: E402

LENS = os.path.join(PROJ, "justice_lens_profiles.json")
W, H, FPS = S.W, S.H, S.FPS
READOUT = S.READOUT
K0, D0 = S.K, S.D


def lens(name="Round 1 independent k2", w=W, h=H):
    return load_lens(LENS, name, w, h)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1., 0], [-s, 0, c]])


def body_rays(K, D, w, h, R_bc):
    uv = np.stack(np.meshgrid(np.arange(w), np.arange(h)), -1).reshape(-1, 1, 2).astype(np.float64)
    n = cv2.fisheye.undistortPoints(uv, K, D).reshape(-1, 2)
    return (np.c_[n, np.ones(w * h)] @ R_bc.T).reshape(h, w, 3)


RAYS0 = body_rays(K0, D0, W, H, R_BC_NADIR)

_PHOTO = {}


class Ground:
    """'synth': sim_test texture spread over 8*h0 metres. Otherwise a photo site from ground/."""

    def __init__(self, kind="synth", h0=100.0):
        self.kind, self.h0 = kind, h0
        if kind != "synth":
            if kind not in _PHOTO:
                import json
                sites = json.load(open(os.path.join(PROJ, "ground", "sites.json")))
                g = cv2.cvtColor(cv2.imread(os.path.join(PROJ, "ground", kind + ".jpg")), cv2.COLOR_BGR2GRAY)
                pyr = [g.astype(np.float32)]
                while pyr[-1].shape[0] > 256:
                    pyr.append(cv2.pyrDown(pyr[-1]))
                _PHOTO[kind] = (pyr, sites[kind]["m_per_px"])
            self.pyr, self.mpp = _PHOTO[kind]

    def sample(self, Pn, Pe, gsd):
        if self.kind == "synth":
            tex = S.TEX
            m = tex.shape[0] / (8 * self.h0)
            border = cv2.BORDER_CONSTANT
        else:
            lvl = min(len(self.pyr) - 1, max(0, int(math.floor(math.log2(max(gsd, 1e-9) / self.mpp)))))
            tex, m = self.pyr[lvl], 1.0 / (self.mpp * 2 ** lvl)
            border = cv2.BORDER_REFLECT101
        return cv2.remap(tex, (Pe * m + tex.shape[1] / 2).astype(np.float32),
                         (-Pn * m + tex.shape[0] / 2).astype(np.float32), cv2.INTER_LINEAR, borderMode=border)


def row_times(t0, h, readout):
    """Readout time of each image row. Negative readout = rows read bottom to top."""
    r = np.arange(h) / h
    return t0 + (r * readout if readout >= 0 else (1 - 1.0 / h - r) * -readout)


def render(t0, pos_fn, att_fn, h_fn, ground, rays=RAYS0, readout=READOUT, exposure=0.0, n_sub=1,
           noise=3.0, fx=None):
    """Rolling-shutter frame. Each row integrates light over [t_row - exposure, t_row]."""
    h, w = rays.shape[:2]
    tr = row_times(t0, h, readout)
    fx = fx or K0[0, 0]
    acc = np.zeros((h, w), np.float32)
    subs = [0.0] if exposure <= 0 else [-(k + 0.5) / n_sub * exposure for k in range(n_sub)]
    for dt in subs:
        t = tr + dt
        b = np.einsum("rij,rwj->rwi", euler_R(*att_fn(t).T), rays)
        hh = h_fn(t)
        P = pos_fn(t)[:, None, :] + (hh[:, None] / b[..., 2])[..., None] * b[..., :2]
        acc += ground.sample(P[..., 0], P[..., 1], hh.mean() / fx)
    img = acc / len(subs)
    img = cv2.GaussianBlur(img, (3, 3), 0.8) + S.rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def fc_feeds(att_fn, h_fn, T, att_hz=100.0, att_lag=S.ATT_LAG, bias=S.ATT_BIAS, noise=S.ATT_NOISE,
             jitter_s=0.0, quant_ms=False, hgt_hz=10.0, hgt_err=None):
    """What the flight controller sends. hgt_err(t, h) -> extra height error (m)."""
    att, hgt = Series(1e9), Series(1e9)
    ts = np.arange(-1, T + 1, 1.0 / att_hz)
    for t in ts:
        stamp = t + (S.rng.uniform(0, jitter_s) if jitter_s else 0.0)
        if quant_ms:
            stamp = math.floor(stamp * 1000) / 1000
        att.add(stamp, *(att_fn(np.array([t - att_lag]))[0] + bias + S.rng.normal(0, noise, 3)))
    if jitter_s:       # stamps can now be out of order; Series assumes they are not
        att.rows.sort(key=lambda r: r[0])
    for t in np.arange(-1, T + 1, 1.0 / hgt_hz):
        h = h_fn(np.array([t]))[0]
        e = hgt_err(t, h) if hgt_err else 0.0
        hgt.add(t, h + 0.5 + e + S.rng.normal(0, 0.15))
    return att, hgt


def hover(h0, wander=2.0, yaw_deg=10.0):
    """sim_test.hover with a chosen wander amplitude (m) and yaw swing (deg)."""
    s = np.sin
    k = wander / 2.0
    pos = lambda t: k * np.c_[1.5 * s(0.21 * t) + 0.5 * s(0.9 * t + 1), 1.5 * s(0.17 * t + 2) + 0.5 * s(1.1 * t)]
    att = lambda t: np.radians(np.c_[
        4 * s(2 * np.pi * 0.7 * t) + 2 * s(2 * np.pi * 1.9 * t + 1) + 0.2 * s(2 * np.pi * 15 * t),
        4 * s(2 * np.pi * 0.6 * t + 2) + 2 * s(2 * np.pi * 2.3 * t) + 0.2 * s(2 * np.pi * 13 * t + 1),
        yaw_deg * s(2 * np.pi * 0.05 * t)])
    return pos, att, lambda t: h0 + 1.0 * s(0.3 * t)


def frames_for(pos, att_fn, h_fn, ground, T=20.0, **kw):
    return [(t0, render(t0, pos, att_fn, h_fn, ground, **kw)) for t0 in np.arange(0, T, 1 / FPS)]


def run(frames, att, hgt, pos_fn, K=K0, D=D0, R_bc=R_BC_NADIR, readout=READOUT, t_off=S.FRAME_T_ERR,
        reanchor_err=np.array([0.4, -0.3]), t_off_fn=None):
    """Feed frames to Hold. t_off: error added to the true first-row time (s)."""
    hold = Hold(K, D, R_bc, readout)
    hold.reanchor(pos_fn(np.array([frames[0][0]]))[0])
    rows = []
    for t0, img in frames:
        off = t_off_fn(t0) if t_off_fn else t_off
        e = hold.process(img, t0 + off, att, hgt)
        if e.state == "lost" and reanchor_err is not None:
            hold.reanchor(pos_fn(np.array([e.t]))[0] + reanchor_err)
        truth = pos_fn(np.array([e.t]))[0]
        rows.append((e.t, e.state, e.var, e.pos.copy(), truth))
    return rows


def score(rows):
    sent = [r for r in rows if r[2] is not None]
    n = len(rows)
    if not sent:
        return dict(used=0.0, rms=math.nan, p95=math.nan, worst=math.nan, false_fix=math.nan, lost=1.0)
    e = np.array([np.linalg.norm(r[3] - r[4]) for r in sent])
    var = np.array([r[2] for r in sent])
    return dict(used=len(sent) / n, rms=float(math.sqrt(np.mean(e ** 2))), p95=float(np.percentile(e, 95)),
                worst=float(e.max()), false_fix=float(np.mean(2 * e ** 2 / var > 9.21)),
                lost=sum(r[1] == "lost" for r in rows) / n)


def fmt(r):
    return (f"sent {r['used']:5.0%}  rms {r['rms']:6.2f} m  p95 {r['p95']:6.2f} m  worst {r['worst']:6.2f} m  "
            f"false-fix {r['false_fix']:5.1%}")
