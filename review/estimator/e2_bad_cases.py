#!/usr/bin/env python3
"""Check 2: the two bad cases.

A) ottawa2019fall at 400 m (claimed RMS 2.6 m, 53 % false-fix). 6 seeds: is it tracking,
   keyframes, or one unlucky draw? Compares each run's constant offset with the attitude error
   the FC reported for the anchor frame (frame 0).
B) prairie at 50 m (claimed RMS 1.8 m). Counts what happens to the tracked points (LK status
   counters wrapped around hold's cv2, nothing edited) and the keyframe changes, and compares
   with controls: prairie at 100 m (photo detail ~ camera pixel), ottawa2019fall at 50 m
   (2x finer photo), and the same prairie run with the photo sharpened by a 2x finer
   source-texture stand-in (the 4096 synthetic texture of sim_test, same blur/noise).
Run: ~/.venvs/vio/bin/python e2_bad_cases.py      (about 60 s)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402


class LKCount:
    """Proxy for hold.cv2. Hold.process calls LK three times per frame: prev->cur, kf->cur, and
    back cur->kf started at the keyframe points. From those it rebuilds Hold's 'good' mask."""

    def __init__(self, real):
        self._real, self.last, self.good = real, [], []

    def __getattr__(self, k):
        return getattr(self._real, k)

    def calcOpticalFlowPyrLK(self, *a, **kw):
        p, s, e = self._real.calcOpticalFlowPyrLK(*a, **kw)
        self.last = (self.last + [(a, p, s)])[-3:]
        if len(self.last) == 3 and "flags" in kw and self.last[1][0][0] is a[1]:   # this is the back call
            (_, _, s1), (_, _, s2), (_, pb, s3) = self.last
            fb = np.linalg.norm((pb - a[3]).reshape(-1, 2), axis=1) < 0.5
            st = (s1 & s2 & s3).ravel().astype(bool)
            self.good.append((len(st), int(st.sum()), int((st & fb).sum())))
        return p, s, e


def run_counted(site, h0, seed, T, pyr_override=None):
    proxy = LKCount(C.Hm.cv2)
    C.Hm.cv2 = proxy
    kf_n0 = []
    if pyr_override is not None:
        C._PYR[site + "*"] = pyr_override
        site = site + "*"

    def hook(i, hold):
        if hold.kf is not None and (not kf_n0 or kf_n0[-1][0] is not hold.kf):
            kf_n0.append((hold.kf, hold.kf["n0"]))
    try:
        rec, _ = C.scenario(site, h0, T=T, seed=seed, hook=hook)
    finally:
        C.Hm.cv2 = proxy._real
    return rec, [n for _, n in kf_n0], proxy.good


def job_A(seed):
    h0 = 400
    rec, _ = C.scenario("ottawa2019fall", h0, T=20.0, seed=seed)
    # FC attitude error for the anchor frame: rebuild the same feed (same seed => same draws)
    C.reseed(seed)
    cfg = C.Cfg()
    p, a, h = C.S.hover(h0)
    _ = C.make_frames("ottawa2019fall", p, a, h, 20.0, cfg)          # consume the same video-noise draws
    att, hgt = C.fc_feeds(a, h, 20.0, cfg)
    seen = C.with_vibration(a, cfg.vib_deg)
    tr = np.linspace(0, C.S.READOUT, 9) + cfg.frame_t_err
    A = att.array()
    m = np.c_[[np.interp(tr, A[:, 0], A[:, k]) for k in (1, 2)]].T
    truth = seen(tr - cfg.frame_t_err)[:, :2]
    # mean roll/pitch error over the anchor frame's rows, without the constant bias (it cancels)
    err = (m - truth).mean(axis=0) - cfg.att_bias[:2]
    s = rec.sent
    bx, by = rec.ex[s].mean(), rec.ey[s].mean()
    return dict(seed=seed, rms=rec.summary()["rms"], ff=rec.summary()["false_fix"], lost=rec.summary()["lost"],
                sw=rec.summary()["kf_switches"], off=math.hypot(bx, by),
                pred=h0 * math.hypot(*err), err_deg=np.degrees(err).tolist())


def job_B(site, h0, seed, T, synth=False):
    pyr = None
    if synth:      # stand-in with detail at every scale (sim_test's texture), at the photo's pixel size
        import cv2
        tex = C.S.TEX.astype(np.float32)
        tex = cv2.resize(tex, (8192, 8192), interpolation=cv2.INTER_CUBIC)
        pyr = [tex]
        while pyr[-1].shape[0] > 256:
            pyr.append(cv2.pyrDown(pyr[-1]))
        pyr = (pyr, 0.29296875 / 2)
    rec, n0s, good = run_counted(site, h0, seed, T, pyr)
    s = rec.sent
    g = np.array(good, float)
    # spread of the per-point fixes, from Hold's own variance terms (frames with no keyframe change)
    hm = h0
    loc = rec.var - rec.kf_var - (C.Hold.ATT_NOISE * hm) ** 2
    keep = s & (rec.kf_new == 0) & (loc > 0)
    spread = float(np.median(np.sqrt(loc[keep] * rec.n[keep]))) if keep.any() else np.nan
    return dict(site=site + (" (synthetic detail)" if synth else ""), h=h0, seed=seed,
                rms=rec.summary()["rms"], sw=rec.summary()["kf_switches"], lost=rec.summary()["lost"],
                n0=float(np.median(n0s)) if n0s else 0, n=float(np.median(rec.n[s])),
                deg=float(np.mean(rec.state[s] == "degraded")),
                end_err=float(np.nanmean(rec.err[-30:])), start_err=float(np.nanmean(rec.err[1:31])),
                lk=float(np.median(g[:, 1])), fbk=float(np.median(g[:, 2])), spread=spread,
                floor=0.003 * h0 + 0.05)


if __name__ == "__main__":
    jobs = [dict(seed=k) for k in range(1, 7)]
    jobsB = [dict(site="prairie", h0=50, seed=k, T=20.0) for k in (1, 3, 5)]
    jobsB += [dict(site="prairie", h0=100, seed=k, T=20.0) for k in (1, 3, 5)]
    jobsB += [dict(site="ottawa2019fall", h0=50, seed=k, T=20.0) for k in (1, 3, 5)]
    jobsB += [dict(site="prairie", h0=50, seed=k, T=20.0, synth=True) for k in (1, 3, 5)]
    A = C.pmap(job_A, jobs)
    B = C.pmap(job_B, jobsB)
    print("A) ottawa2019fall 400 m, 20 s, sim_ground settings")
    print(" seed  RMS(m) false-fix lost kf-switches | constant offset(m)  h x FC roll/pitch noise at anchor frame(m)  "
          "(roll, pitch noise deg)")
    for r in A:
        print(f"  {r['seed']}    {r['rms']:5.2f}   {r['ff']:5.1%}  {r['lost']:4.0%}   {r['sw']:3d}       |   {r['off']:5.2f}"
              f"              {r['pred']:5.2f}                                  ({r['err_deg'][0]:+.3f}, {r['err_deg'][1]:+.3f})")
    c = np.corrcoef([r["off"] for r in A], [r["pred"] for r in A])[0, 1]
    print(f"  correlation of offset with anchor-frame attitude error: {c:.2f}")
    print("\nB) low height / low detail. n0 = points in a new keyframe (median), n = inliers per fix (median),")
    print("   LK ok = points with all 3 LK status bits (median), +FB = also pass the 0.5 px back check,")
    print("   spread = scatter of per-point fixes (m), floor = inlier floor 0.003 h + 0.05 (m), deg = share 'degraded'")
    print(" site                         h  seed  RMS(m) err first 1 s -> last 1 s  kf-switches  n0  LK ok  +FB  inliers"
          "  spread floor  deg")
    for r in B:
        print(f" {r['site']:28s} {r['h']:3d}  {r['seed']}    {r['rms']:5.2f}   {r['start_err']:5.2f} -> {r['end_err']:5.2f}"
              f"          {r['sw']:4d}   {r['n0']:4.0f} {r['lk']:5.0f} {r['fbk']:5.0f} {r['n']:5.0f}"
              f"    {r['spread']:5.2f}  {r['floor']:4.2f} {r['deg']:4.0%}")
