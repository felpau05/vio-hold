"""Shared harness for the estimator review. Imports the project's own simulators unchanged.

Every experiment script does `import common as C` after putting this folder on sys.path, so
the scripts run from any directory. Nothing in the project is edited.
"""
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))          # /home/paul/Videos/vio-hold-attempt
if PROJ not in sys.path:
    sys.path.insert(0, PROJ)

import cv2                    # noqa: E402
import numpy as np            # noqa: E402

import hold as Hm             # noqa: E402
import sim_test as S          # noqa: E402
import sim_ground as G        # noqa: E402
from hold import Hold         # noqa: E402

CHI2_99_2DOF = 9.21
_PYR = {}


def site(name):
    if name not in _PYR:
        _PYR[name] = G.load(name)
    return _PYR[name]


def reseed(seed):
    """sim_test and sim_ground draw all noise from S.rng; reseed it for repeatable runs."""
    S.rng = np.random.default_rng(seed)


class Cfg:
    """Error sources of the sim, each one switchable. Defaults = sim_ground as written."""

    def __init__(self, **kw):
        self.att_noise = S.ATT_NOISE
        self.att_bias = S.ATT_BIAS.copy()
        self.att_lag = S.ATT_LAG
        self.frame_t_err = S.FRAME_T_ERR
        self.vib_deg = G.VIB_DEG
        self.video_noise = G.NOISE
        self.baro_bias = 0.5
        self.baro_noise = 0.15
        self.__dict__.update(kw)


def fc_feeds(att_fn, h_fn, T, cfg, att_bias_fn=None):
    """Same as sim_test.fc_feeds with every error source taken from cfg.
    att_bias_fn(t) -> extra (roll, pitch, yaw) bias in rad (slow bias wander)."""
    att, hgt = Hm.Series(1e9), Hm.Series(1e9)
    for t in np.arange(-1, T + 1, 0.01):
        a = att_fn(np.array([t - cfg.att_lag]))[0] + cfg.att_bias + S.rng.normal(0, cfg.att_noise, 3)
        if att_bias_fn is not None:
            a = a + att_bias_fn(t)
        att.add(t, *a)
    for t in np.arange(-1, T + 1, 0.1):
        hgt.add(t, h_fn(np.array([t]))[0] + cfg.baro_bias + S.rng.normal(0, cfg.baro_noise))
    return att, hgt


def with_vibration(att_fn, deg):
    def f(t):
        a = np.array(att_fn(t), float)
        a[:, 0] += math.radians(deg) * np.sin(2 * np.pi * G.VIB_HZ[0] * t)
        a[:, 1] += math.radians(deg) * np.sin(2 * np.pi * G.VIB_HZ[1] * t + 1)
        return a
    return f


def render(t0, pos_fn, att_fn, h_fn, pyr, mpp, noise):
    old = G.NOISE
    G.NOISE = noise
    try:
        return G.render(t0, pos_fn, att_fn, h_fn, pyr, mpp)
    finally:
        G.NOISE = old


def make_frames(name, pos_fn, att_fn, h_fn, T, cfg, img_mod=None, fps=S.FPS):
    """img_mod(t0, img_float_or_uint8) -> uint8: optional picture change (shadow, car, fade ...)."""
    pyr, mpp = site(name)
    seen = with_vibration(att_fn, cfg.vib_deg)
    frames = []
    for t0 in np.arange(0, T, 1 / fps):
        img = render(t0, pos_fn, seen, h_fn, pyr, mpp, cfg.video_noise)
        if img_mod is not None:
            img = img_mod(t0, img)
        frames.append((t0, img))
    return frames


class Rec:
    """Per-frame log of one run."""

    def __init__(self, rows):
        self.rows = rows
        for k in rows[0]:
            setattr(self, k, np.array([r[k] for r in rows], dtype=object if k in ("state", "why") else float))

    @property
    def sent(self):
        return ~np.isnan(self.var)

    def summary(self):
        s = self.sent
        e = self.err[s]
        out = dict(frames=len(self.rows), sent=s.mean(), lost=np.mean(self.state == "lost"),
                   kf_switches=int(np.nansum(self.kf_new)) - 1)
        if s.any():
            out.update(rms=math.sqrt(np.mean(e ** 2)), p95=float(np.percentile(e, 95)), worst=float(e.max()),
                       false_fix=float(np.mean(2 * e ** 2 / self.var[s] > CHI2_99_2DOF)))
        return out


def run(frames, att, hgt, pos_fn, cfg, readout=S.READOUT, hold=None, reanchor_on_lost=None, hook=None,
        post=None):
    """Feed frames to Hold like sim_ground.run_recorded (no video). Logs truth, kf changes, vel.
    reanchor_on_lost: None (sim_ground behaviour) or an offset (m) to add to truth (sim_test behaviour)."""
    hold = hold or Hold(S.K, S.D, S.R_BC_NADIR, readout)
    hold.reanchor(pos_fn(np.array([frames[0][0]]))[0])
    rows, last_kf = [], None
    for i, (t0, img) in enumerate(frames):
        if hook is not None:
            hook(i, hold)
        c0 = time.perf_counter()
        e = hold.process(img, t0 + cfg.frame_t_err, att, hgt)
        dt = time.perf_counter() - c0
        if e.state == "lost" and reanchor_on_lost is not None:
            hold.reanchor(pos_fn(np.array([e.t]))[0] + reanchor_on_lost)
        truth = pos_fn(np.array([e.t]))[0]
        sent = e.var is not None
        kf_new = hold.kf is not None and hold.kf is not last_kf
        d = e.pos - truth
        n0_before = last_kf["n0"] if last_kf is not None else np.nan
        if post is not None:
            post(i, hold, e, img, t0 + cfg.frame_t_err, att, hgt)
            kf_new = hold.kf is not None and hold.kf is not last_kf
        last_kf = hold.kf
        rows.append(dict(t=e.t, state=e.state, why=e.why or "", n=e.n, var=e.var if sent else np.nan, n0_prev=n0_before,
                         ex=d[0] if sent else np.nan, ey=d[1] if sent else np.nan,
                         err=float(np.linalg.norm(d)) if sent else np.nan,
                         px=e.pos[0], py=e.pos[1], tx=truth[0], ty=truth[1],
                         kf_new=float(kf_new), kf_var=hold.kf["var"] if hold.kf is not None else np.nan,
                         vx=hold.vel[0], vy=hold.vel[1], rate=hold.rate, cpu=dt,
                         var_old=(getattr(hold, "old_var", None) or np.nan) if sent else np.nan))
    return Rec(rows)


def scenario(name, h0, T=20.0, cfg=None, pos_fn=None, att_fn=None, h_fn=None, img_mod=None, seed=1,
             att_bias_fn=None, **run_kw):
    """The sim_ground hover (or a given path) over a site, one call."""
    cfg = cfg or Cfg()
    reseed(seed)
    p, a, h = S.hover(h0)
    pos_fn, att_fn, h_fn = pos_fn or p, att_fn or a, h_fn or h
    frames = make_frames(name, pos_fn, att_fn, h_fn, T, cfg, img_mod)
    att, hgt = fc_feeds(att_fn, h_fn, T, cfg, att_bias_fn)
    return run(frames, att, hgt, pos_fn, cfg, **run_kw), frames


def fmt(d):
    keys = ("sent", "lost", "rms", "p95", "worst", "false_fix", "kf_switches")
    out = []
    for k in keys:
        if k not in d:
            continue
        v = d[k]
        out.append(f"{k} {v:5.0%}" if k in ("sent", "lost") else
                   f"{k} {v:5.1%}" if k == "false_fix" else f"{k} {v}" if k == "kf_switches" else f"{k} {v:6.2f}")
    return "  ".join(out)


def _init_worker():
    cv2.setNumThreads(1)


def pmap(fn, jobs, procs=None):
    """Run fn(**job) for each job dict in worker processes (spawn: no fork of OpenCV threads).
    fn must be a module-level function of an importable module (the calling script)."""
    import multiprocessing as mp
    procs = procs or max(1, min(len(jobs), (os.cpu_count() or 2) - 2))
    with mp.get_context("spawn").Pool(procs, initializer=_init_worker) as pool:
        return pool.starmap(_call, [(fn, j) for j in jobs])


def _call(fn, job):
    return fn(**job)


def run_stats(rec, h0):
    """Numbers one run contributes to a multi-seed table."""
    s = rec.sent
    ex, ey = rec.ex[s], rec.ey[s]
    bx, by = ex.mean(), ey.mean()
    d = rec.summary()
    d.update(h=h0, bias=math.hypot(bx, by), jit=math.sqrt(np.mean((ex - bx) ** 2 + (ey - by) ** 2)),
             first=float(rec.err[s][0]) if s.any() else np.nan, var_mean=float(np.mean(rec.var[s])),
             e2=list(map(float, ex ** 2 + ey ** 2)))
    return d


def render_fx(t0, pos_fn, att_fn, h_fn, pyr, mpp, noise, ground_fx=None):
    """sim_ground.render with a hook: ground_fx(img, P, tr) edits the picture in ground
    coordinates (P: H x W x 2 north/east metres of each pixel, tr: row times) before blur + noise."""
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
    if ground_fx is not None:
        img = ground_fx(img, P, tr)
    img = cv2.GaussianBlur(img, (3, 3), 0.8) + S.rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def scenario2(name, h0, T, pos_fn, att_fn, h_fn, h_baro_fn=None, ground_fx=None, img_mod=None, cfg=None,
              seed=1, hold=None, **run_kw):
    """Like scenario(), with ground-space effects, a separate baro height, and picture edits."""
    cfg = cfg or Cfg()
    reseed(seed)
    pyr, mpp = site(name)
    seen = with_vibration(att_fn, cfg.vib_deg)
    frames = []
    for t0 in np.arange(0, T, 1 / S.FPS):
        img = render_fx(t0, pos_fn, seen, h_fn, pyr, mpp, cfg.video_noise, ground_fx)
        if img_mod is not None:
            img = img_mod(t0, img)
        frames.append((t0, img))
    att, hgt = fc_feeds(att_fn, h_baro_fn or h_fn, T, cfg)
    return run(frames, att, hgt, pos_fn, cfg, hold=hold, **run_kw), frames
