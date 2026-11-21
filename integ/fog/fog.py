#!/usr/bin/env python3
"""Fog: at which visibility and height does the hold stop, and does it send wrong fixes first?

Fog model (Koschmieder): each pixel = ground * t + A * (1 - t), t = exp(-3.912 * d / V).
d = slant distance from the camera to the ground point of the pixel (m). V = meteorological
visibility (m): under 1000 m is fog, under 200 m is dense fog. A = airlight grey level.
The fog goes in before the camera blur and noise, as in a real camera. The camera noise does not
change with fog. ASSUMED: no auto gain that stretches the contrast again (the Justice D has no
exposure control that we can set; its own auto gain is unknown).

Hover (sim_test.hover) at the event levels 10, 20, 35, 50, 75 m. Sites: ottawa2019fall (event site,
fall leaves), prairie (low texture). Camera = the sim's analog Justice model, not the Justice D.
Columns: share of frames sent, RMS and worst error of sent fixes, false-fix (2 err^2/var > 9.21),
'no ground' refusals.
Run: ~/.venvs/vio/bin/python integ/fog/fog.py [--quick]
"""
import argparse
import math
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "review", "estimator"))
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.S
HEIGHTS = (10, 20, 35, 50, 75)
VIS = (math.inf, 1000, 500, 300, 200, 150, 100, 75, 50)
AIRLIGHT = 230.0                 # thick white fog (21 Nov 2026, Ottawa)
T = 10.0


def fog_fx(pos_fn, h_fn, vis):
    def fx(img, P, tr):
        if math.isinf(vis):
            return img
        dpos = P - pos_fn(tr)[:, None, :]
        d = np.sqrt(h_fn(tr)[:, None] ** 2 + np.sum(dpos ** 2, axis=-1))
        t = np.exp(-3.912 * d / vis)
        return img * t + AIRLIGHT * (1 - t)
    return fx


def job(args):
    site, h0, vis, noise, seed = args
    pos, att, h = S.hover(h0)
    rec, frames = C.scenario2(site, h0, T, pos_fn=pos, att_fn=att, h_fn=h, cfg=C.Cfg(video_noise=noise),
                              ground_fx=fog_fx(pos, h, vis), seed=seed)
    s = rec.sent
    q = 2 * rec.err ** 2 / rec.var
    img = frames[len(frames) // 2][1].astype(float)
    why = [str(x) for x in getattr(rec, "why", [])]
    return dict(site=site, h=h0, vis=vis, noise=noise, seed=seed, sent=float(s.mean()),
                rms=float(np.sqrt(np.mean(rec.err[s] ** 2))) if s.any() else math.nan,
                worst=float(np.max(rec.err[s])) if s.any() else math.nan,
                ff=float(np.mean(q[s] > C.CHI2_99_2DOF)) if s.any() else math.nan,
                cw=int(np.sum(s & (q > C.CHI2_99_2DOF) & (rec.err > 1.0))),
                img_std=float(img.std()), t_centre=math.exp(-3.912 * h0 / vis))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="one job, for timing")
    ap.add_argument("--sites", default="ottawa2019fall,prairie")
    ap.add_argument("--noise", default=f"{C.G.NOISE}")
    ap.add_argument("--seeds", type=int, default=2)
    a = ap.parse_args()
    if a.quick:
        print(job(("ottawa2019fall", 50, 300, C.G.NOISE, 1)))
        return
    jobs = [(st, h, v, float(n), sd) for st in a.sites.split(",") for n in a.noise.split(",")
            for h in HEIGHTS for v in VIS for sd in range(1, a.seeds + 1)]
    with Pool(min(int(os.environ.get("FOG_PROCS", 8)), os.cpu_count() or 4)) as p:   # about 1 GB each (Ottawa photo)
        res = p.map(job, jobs)
    print(__doc__.split("Run:")[0])
    print(f"airlight {AIRLIGHT:.0f}, {T:.0f} s per run, {a.seeds} seeds; values = mean over seeds\n")
    for st in a.sites.split(","):
        for n in a.noise.split(","):
            print(f"site {st}, camera noise {float(n):.1f} grey")
            print("  h(m)  vis(m)  contrast  img std  sent %   RMS(m)  worst(m)  false-fix %  confident-wrong")
            for h in HEIGHTS:
                for v in VIS:
                    r = [x for x in res if x["site"] == st and x["noise"] == float(n) and x["h"] == h and x["vis"] == v]
                    m = lambda k: float(np.nanmean([x[k] for x in r])) if not all(math.isnan(x[k]) for x in r) else math.nan  # noqa: E731
                    print(f"  {h:4d}  {'clear' if math.isinf(v) else int(v):>6}  {100 * r[0]['t_centre']:6.0f} %  {m('img_std'):7.1f}"
                          f"  {100 * m('sent'):6.0f}  {m('rms'):7.2f}  {m('worst'):8.2f}  {100 * m('ff'):11.1f}"
                          f"  {sum(x['cw'] for x in r):15d}")
            print()


if __name__ == "__main__":
    main()
