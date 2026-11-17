#!/usr/bin/env python3
"""Check 8: texture loss.

A) Ground contrast 100/50/25/10/5 % (photo blended toward its grey mean), video noise std 5 and
   10, ottawa2019fall at 100 m and 200 m, 10 s. Share of frames with a fix, RMS, false-fix.
B) Fade: contrast falls linearly from 100 % at t = 0 to 0 % at t = 15 s (haze, low sun, dusk).
   What is sent in the last 2 s before the hold stops: error vs sent sigma.
C) Camera blackout to analog static (t = 5..10 s) with static std 3 (sim_test's value), 5, 10, 30,
   and the caller re-anchoring after 'lost' (sim_test behaviour). Does Hold anchor on static and
   send positions while the camera sees nothing?
Run: ~/.venvs/vio/bin/python e8_texture.py      (about 90 s on 16 threads)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S


def contrast_fx(c_fn):
    def fx(img, P, tr):
        c = c_fn(tr)[:, None]
        return 128.0 + c * (img - 128.0)
    return fx


def job_A(h0, c, noise):
    pos, att, h = S.hover(h0)
    cfg = C.Cfg(video_noise=noise)
    rec, _ = C.scenario2("ottawa2019fall", h0, 10.0, pos_fn=pos, att_fn=att, h_fn=h, cfg=cfg,
                         ground_fx=contrast_fx(lambda tr: np.full_like(tr, c)))
    s = rec.sent
    q = 2 * rec.err[s] ** 2 / rec.var[s]
    return dict(h=h0, c=c, noise=noise, sent=float(s.mean()), lost=float(np.mean(rec.state == "lost")),
                rms=float(np.sqrt(np.mean(rec.err[s] ** 2))) if s.any() else np.nan,
                worst=float(np.max(rec.err[s])) if s.any() else np.nan,
                ff=float(np.mean(q > 9.21)) if s.any() else np.nan, n=float(np.median(rec.n[s])) if s.any() else 0,
                sw=int(np.sum(rec.kf_new)) - 1)


def job_B(h0, noise):
    pos, att, h = S.hover(h0)
    cfg = C.Cfg(video_noise=noise)
    rec, _ = C.scenario2("ottawa2019fall", h0, 20.0, pos_fn=pos, att_fn=att, h_fn=h, cfg=cfg,
                         ground_fx=contrast_fx(lambda tr: np.clip(1 - tr / 15.0, 0, 1)))
    s = rec.sent
    t_stop = rec.t[s][-1] if s.any() else np.nan
    last = s & (rec.t > t_stop - 2.0)
    q = 2 * rec.err ** 2 / rec.var
    sent_after_dark = int(np.sum(s & (rec.t > 15.0)))
    return dict(h=h0, noise=noise, t_stop=float(t_stop), c_stop=float(max(0, 1 - t_stop / 15)),
                err_last=float(np.mean(rec.err[last])), max_last=float(np.max(rec.err[last])),
                sig_last=float(np.mean(np.sqrt(rec.var[last] / 2))), ff_last=float(np.mean(q[last] > 9.21)),
                cw=int(np.sum(last & (q > 9.21) & (rec.err > 1.0))), after=sent_after_dark,
                n_last=float(np.median(rec.n[last])), sw=int(np.sum(rec.kf_new)) - 1,
                rms_c50=float(np.sqrt(np.nanmean(rec.err[s & (rec.t < 7.5)] ** 2))))


def job_C(h0, sstd):
    pos, att, h = S.hover(h0)
    dark = (5.0, 10.0)

    def mod(t0, img):
        if dark[0] <= t0 < dark[1]:
            return np.clip(np.full_like(img, 90) + S.rng.normal(0, sstd, img.shape), 0, 255).astype(np.uint8)
        return img
    rec, _ = C.scenario2("ottawa2019fall", h0, 14.0, pos_fn=pos, att_fn=att, h_fn=h, img_mod=mod,
                         reanchor_on_lost=np.array([0.4, -0.3]))
    d = (rec.t > dark[0] + 0.5) & (rec.t < dark[1])
    s = rec.sent
    q = 2 * rec.err ** 2 / rec.var
    return dict(h=h0, sstd=sstd, sent_dark=int(np.sum(s & d)), frames_dark=int(d.sum()),
                inits_dark=int(np.sum(d & (rec.state == "init"))),
                err_dark=float(np.max(rec.err[s & d])) if (s & d).any() else np.nan,
                sig_dark=float(np.median(np.sqrt(rec.var[s & d] / 2))) if (s & d).any() else np.nan,
                ff_dark=float(np.mean(q[s & d] > 9.21)) if (s & d).any() else np.nan,
                ok_dark=int(np.sum(d & (rec.state == "ok"))))


if __name__ == "__main__":
    A = C.pmap(job_A, [dict(h0=h, c=c, noise=n) for h in (100, 200) for c in (1.0, 0.5, 0.25, 0.1, 0.05)
                       for n in (5.0, 10.0)])
    B = C.pmap(job_B, [dict(h0=h, noise=n) for h in (100, 200) for n in (5.0, 10.0)])
    Cc = C.pmap(job_C, [dict(h0=100, sstd=s) for s in (3.0, 5.0, 10.0, 30.0)])
    print("A) fixed ground contrast, 10 s")
    print(" h(m)  contrast  noise  fixes sent  lost   RMS(m)  worst(m)  false-fix  inliers  kf changes")
    for r in A:
        print(f" {r['h']:4d}   {r['c']:5.0%}    {r['noise']:3.0f}     {r['sent']:5.0%}    {r['lost']:4.0%}   {r['rms']:6.2f}"
              f"   {r['worst']:6.2f}    {r['ff']:5.1%}    {r['n']:5.0f}    {r['kf'] if 'kf' in r else r['sw']:4d}")
    print("\nB) contrast fades 100 % -> 0 % over 15 s. Last 2 s of fixes before the hold stops:")
    print(" h(m)  noise  stops at t(s) (contrast)  RMS at c>50%  mean err  max err  mean sent sigma/axis  "
          "false-fix  confident-wrong(>1 m)  inliers  kf changes  fixes sent after c=0")
    for r in B:
        print(f" {r['h']:4d}   {r['noise']:3.0f}     {r['t_stop']:5.2f} ({r['c_stop']:4.0%})            {r['rms_c50']:5.2f}"
              f"      {r['err_last']:5.2f}    {r['max_last']:5.2f}       {r['sig_last']:5.2f}             {r['ff_last']:5.1%}"
              f"        {r['cw']:4d}             {r['n_last']:4.0f}     {r['sw']:4d}        {r['after']}")
    print("\nC) 5 s blackout to analog static at 100 m, caller re-anchors after 'lost' (sim_test behaviour)")
    print(" static std  frames in dark  keyframes made on static  fixes sent in dark  'ok' fixes  max err(m)  "
          "median sent sigma/axis(m)  false-fix")
    for r in Cc:
        print(f"   {r['sstd']:4.0f}        {r['frames_dark']:4d}             {r['inits_dark']:4d}                  {r['sent_dark']:4d}"
              f"          {r['ok_dark']:4d}      {r['err_dark']:6.2f}        {r['sig_dark']:6.2f}               {r['ff_dark']:5.1%}")
