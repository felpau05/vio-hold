#!/usr/bin/env python3
"""Check 4: inputs that break the outlier logic of Hold.process.

ottawa2019fall at 100 m, sim_ground settings, 15 s; the event starts at t = 5 s.
  baseline        nothing happens
  cloud shadow    soft disc of 60 m radius, 55 % darker, crosses under the drone at 8 m/s
  shadow edge     half-plane shadow edge sweeps across at 8 m/s, then stays (sun behind cloud)
  big object 30%  textured object (2x contrast of the ground) covering ~30 % of the picture,
                  in view from t = 0 (so in the keyframe), moving east at 3 m/s
  repeated grid   ground = 3 m grid of bright dots (solar farm / parking) over the photo at 30 %
  climb 10 m/s    real climb 100 -> 150 m from t = 5 to 10 s
  baro glitch     baro reads +10 m from t = 5 to 7 s; real height unchanged
  yaw turn 90     yaw ramp 1.5 rad/s (below MAX_RATE 2.0) for 90 deg at t = 5 s
  frozen frame    capture repeats the t = 5 s frame; the drone drifts north at 0.5 m/s
  static OSD      OSD text + crosshair burned into the picture, ground at 40 % contrast;
                  the drone drifts north at 0.5 m/s from t = 5 s
"confident wrong" = sent fixes with error > 2 m that are outside the 99 % bound of the sent var.
Run: ~/.venvs/vio/bin/python e4_outliers.py    (about 60 s on 16 threads)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import common as C  # noqa: E402

H0, T, TE = 100, 15.0, 5.0
S = C.S


def base_fns():
    return S.hover(H0)


def drift(pos):
    return lambda t: pos(t) + np.c_[0.5 * np.clip(np.asarray(t) - TE, 0, None), np.zeros(len(np.atleast_1d(t)))]


def soft_disc(P, c, R, edge):
    d = np.hypot(P[..., 0] - c[:, None, 0], P[..., 1] - c[:, None, 1])
    return np.clip((R - d) / edge, 0, 1)


def make(name):
    pos, att, h = base_fns()
    kw = dict(pos_fn=pos, att_fn=att, h_fn=h)
    if name == "cloud shadow":
        def fx(img, P, tr):
            c = np.c_[np.zeros_like(tr), 8.0 * (tr - 8.0)]            # passes overhead at t = 8 s
            return img * (1 - 0.55 * soft_disc(P, c, 60.0, 10.0))
        kw["ground_fx"] = fx
    elif name == "shadow edge":
        def fx(img, P, tr):
            x = 8.0 * (tr - 8.0)                                        # edge at drone at t = 8 s
            s = np.clip((x[:, None] - P[..., 1]) / 10.0, 0, 1)          # ground east of ... is shaded
            return img * (1 - 0.55 * s)
        kw["ground_fx"] = fx
    elif name == "big object 30%":
        pyr2, mpp2 = C.site("suburb")
        tex2 = pyr2[1]

        def fx(img, P, tr):
            c = np.c_[np.zeros_like(tr) - 10.0, -20.0 + 3.0 * tr]       # centre (north, east), m
            dn, de = P[..., 0] - c[:, None, 0], P[..., 1] - c[:, None, 1]
            m = ((np.abs(dn) < 45) & (np.abs(de) < 45)).astype(np.float32)
            k = 1.0 / (mpp2 * 2)
            v = cv2.remap(tex2, (de * k + 900).astype(np.float32), (-dn * k + 900).astype(np.float32),
                          cv2.INTER_LINEAR)
            v = np.clip(128 + 2.0 * (v - v.mean()), 0, 255)
            return img * (1 - m) + v * m
        kw["ground_fx"] = fx
    elif name == "repeated grid":
        def fx(img, P, tr):
            p = 3.0
            g = (np.cos(2 * np.pi * P[..., 0] / p) * np.cos(2 * np.pi * P[..., 1] / p) > 0.6) * 200.0 + 30.0
            return 0.3 * img + 0.7 * g
        kw["ground_fx"] = fx
    elif name == "climb 10 m/s":
        kw["h_fn"] = lambda t: h(t) + 10.0 * np.clip(np.asarray(t) - TE, 0, 5.0)
    elif name == "baro glitch":
        kw["h_baro_fn"] = lambda t: h(t) + 10.0 * ((np.asarray(t) >= TE) & (np.asarray(t) < TE + 2))
    elif name == "yaw turn 90":
        def att2(t):
            a = att(t)
            a[:, 2] += np.clip(1.5 * (np.asarray(t) - TE), 0, math.pi / 2)
            return a
        kw["att_fn"] = att2
    elif name == "frozen frame":
        kw["pos_fn"] = drift(pos)
        frozen = {}

        def mod(t0, img):
            if t0 >= TE:
                frozen.setdefault("img", img)
                return frozen["img"]
            return img
        kw["img_mod"] = mod
    elif name == "static OSD":
        kw["pos_fn"] = drift(pos)
        osd = np.zeros((S.H, S.W), np.uint8)
        mask = np.zeros((S.H, S.W), np.uint8)
        for txt, org in (("ALT 100M  12.4V  LQ 98", (8, 18)), ("HDG 270  SPD 0.0  00:42", (8, 230)),
                         ("REC  GPS 0", (250, 18))):
            cv2.putText(mask, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 255, 3, cv2.LINE_AA)
            cv2.putText(osd, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 255, 1, cv2.LINE_AA)
        cv2.line(osd, (165, 120), (195, 120), 255, 1)
        cv2.line(osd, (180, 105), (180, 135), 255, 1)
        cv2.line(mask, (163, 120), (197, 120), 255, 3)
        cv2.line(mask, (180, 103), (180, 137), 255, 3)
        m = mask > 0

        def fx(img, P, tr):
            return 128 + 0.4 * (img - 128)

        def mod(t0, img):
            out = img.copy()
            out[m] = osd[m]
            return out
        kw["ground_fx"], kw["img_mod"] = fx, mod
    return kw


class Bucketed:
    """Proxy for hold.cv2: goodFeaturesToTrack picks corners per cell of a 6 x 4 grid, so one
    textured region cannot own most of the keyframe's points. (Proposed fix, tested here.)"""

    def __init__(self, real):
        self._real = real

    def __getattr__(self, k):
        return getattr(self._real, k)

    def goodFeaturesToTrack(self, img, n, q, d):
        out = []
        hh, ww = img.shape
        for i in range(4):
            for j in range(6):
                y0, x0 = i * hh // 4, j * ww // 6
                sub = img[y0:(i + 1) * hh // 4, x0:(j + 1) * ww // 6]
                p = self._real.goodFeaturesToTrack(np.ascontiguousarray(sub), max(1, n // 24), q, d)
                if p is not None:
                    out.append(p + np.array([x0, y0], np.float32))
        return np.concatenate(out) if out else None


def job(name):
    bucket = name.endswith("+ grid corners")
    kw = make(name.replace(" + grid corners", ""))
    if bucket:
        C.Hm.cv2 = Bucketed(C.Hm.cv2)
    try:
        rec, frames = C.scenario2("ottawa2019fall", H0, T, **kw)
    finally:
        if bucket:
            C.Hm.cv2 = C.Hm.cv2._real
    s = rec.sent
    pre, post = s & (rec.t < TE), s & (rec.t >= TE)
    q = 2 * rec.err ** 2 / rec.var
    bad = post & (rec.err > 2.0) & (q > 9.21)
    states = {k: int(np.sum((rec.state == k) & (rec.t >= TE))) for k in ("ok", "degraded", "lost", "init")}
    trace = [(round(float(t)), round(float(e), 2) if not np.isnan(e) else None, st,
              round(float(math.sqrt(v / 2)), 2) if not np.isnan(v) else None)
             for t, e, st, v in zip(rec.t[::30], rec.err[::30], rec.state[::30], rec.var[::30])]
    whys = {}
    for w in rec.why[rec.t >= TE]:
        if w:
            key = w if not w[0].isdigit() else w.split(" ", 1)[1]
            whys[key] = whys.get(key, 0) + 1
    if name in ("frozen frame", "static OSD", "big object 30%"):
        cv2.imwrite(os.path.join(C.HERE, f"e4_{name.split()[0]}_{name.split()[1]}.png"), frames[int(8 * S.FPS)][1])
    return dict(name=name, rms_pre=float(np.sqrt(np.mean(rec.err[pre] ** 2))) if pre.any() else np.nan,
                rms_post=float(np.sqrt(np.mean(rec.err[post] ** 2))) if post.any() else np.nan,
                max_post=float(np.max(rec.err[post])) if post.any() else np.nan,
                sent_post=float(np.mean(s[rec.t >= TE])), states=states, conf_wrong=int(bad.sum()),
                ff=float(np.mean(q[post] > 9.21)) if post.any() else np.nan,
                sw=int(np.sum(rec.kf_new[rec.t >= TE])), end=float(rec.err[-1]), trace=trace, whys=whys)


NAMES = ["baseline", "cloud shadow", "shadow edge", "big object 30%", "repeated grid", "climb 10 m/s",
         "baro glitch", "yaw turn 90", "frozen frame", "static OSD",
         "baseline + grid corners", "big object 30% + grid corners", "static OSD + grid corners"]

if __name__ == "__main__":
    res = C.pmap(job, [dict(name=n) for n in NAMES])
    print(f"ottawa2019fall {H0} m, event at t = {TE:.0f} s. Stats over t >= {TE:.0f} s.")
    print(" scenario                        RMS before  RMS after  max after  sent after  kf changes  false-fix  confident-wrong  "
          "states after (ok/deg/lost/init)   why not sent")
    for r in res:
        st = r["states"]
        print(f" {r['name']:30s}  {r['rms_pre']:6.2f}     {r['rms_post']:6.2f}    {r['max_post']:6.2f}     "
              f"{r['sent_post']:5.0%}      {r['sw']:4d}       {r['ff']:5.1%}       {r['conf_wrong']:4d}          "
              f"{st['ok']}/{st['degraded']}/{st['lost']}/{st['init']}      {r['whys']}")
    print("\nerror trace (t s, error m, state, sent sigma per axis m), every 1 s:")
    for r in res:
        print(f" {r['name']:30s}", " ".join(f"{t}:{e}" for t, e, st, sg in r["trace"]))
    for r in res:
        if r["name"] in ("frozen frame", "static OSD"):
            print(f"\n {r['name']}: per-second (t, err, state, sigma):", r["trace"])
