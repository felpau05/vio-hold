#!/usr/bin/env python3
"""Static overlay (scope 2; E2, real video): OSD text burned into 20 % of the picture.

ottawa2019fall at 100 m, sim_ground settings, 15 s, the drone drifts 0.5 m/s north from 5 s (a fix
stuck on the overlay would stop following it). The overlay: white text with a black edge in the
top and bottom 24 rows (2 x 24 / 240 = 20 % of the picture) plus a crosshair, like a real OSD.
  no overlay            reference
  overlay + band mask   mask = the overlay region
  overlay + built mask  mask = hold.build_static_mask() of the first 5 s of video
  overlay, no mask      only the static-point gate and the robust fit protect the fix
  no overlay / overlay, no mask, ground 40 % contrast   e4_outliers.py 'static OSD' (version 1: RMS 8.07 m)
6 seeds per case. Run: ~/.venvs/vio/bin/python tests/overlay.py  (about 60 s) -> tests/overlay.txt
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2                  # noqa: E402
import numpy as np          # noqa: E402

import harness as X         # noqa: E402
from hold import Hold, build_static_mask   # noqa: E402

T, TE = 15.0, 5.0
CASES = ["no overlay", "overlay + band mask", "overlay + built mask", "overlay, no mask",
         "no overlay, ground 40 % contrast", "overlay, no mask, ground 40 % contrast"]


def osd():
    """Overlay picture, where it is drawn, and the band mask (0 = not ground)."""
    W, H = X.S.W, X.S.H
    pic, drawn = np.zeros((H, W), np.uint8), np.zeros((H, W), np.uint8)
    for txt, org in (("ALT 100M  12.4V  LQ 98  REC", (6, 17)), ("HDG 270  SPD 0.0  00:42  3D", (6, H - 7))):
        cv2.putText(drawn, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 3, cv2.LINE_AA)
        cv2.putText(pic, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1, cv2.LINE_AA)
    for (x0, y0), (x1, y1) in (((W // 2 - 15, H // 2), (W // 2 + 15, H // 2)), ((W // 2, H // 2 - 15), (W // 2, H // 2 + 15))):
        cv2.line(drawn, (x0, y0), (x1, y1), 255, 3)
        cv2.line(pic, (x0, y0), (x1, y1), 255, 1)
    mask = np.full((H, W), 255, np.uint8)
    mask[:24], mask[-24:] = 0, 0
    mask[H // 2 - 18:H // 2 + 19, W // 2 - 18:W // 2 + 19] = 0
    return pic, drawn > 0, mask


def job(case, seed):
    pic, drawn, band_mask = osd()
    p, a, hf = X.S.hover(100)
    pos = lambda t: p(t) + np.c_[0.5 * np.clip(np.asarray(t) - TE, 0, None), np.zeros(len(t))]
    over = not case.startswith("no overlay")
    fx = (lambda img, P, tr: 128 + 0.4 * (img - 128)) if "40 %" in case else None

    def mod(t0, img):
        if not over:
            return img
        out = img.copy()
        out[drawn] = pic[drawn]         # white text on a black edge, burned in
        return out

    frames, att, hgt, pos = X.scene(h0=100, T=T, seed=seed, pos_fn=pos, ground_fx=fx, img_mod=mod)
    mask = band_mask if case == "overlay + band mask" else (
        build_static_mask([f for t, f in frames if t < TE]) if case == "overlay + built mask" else None)
    rec = X.run(frames, att, hgt, pos, hold=Hold(X.S.K, X.S.D, X.S.R_BC_NADIR, X.S.READOUT, mask=mask))
    st, late = rec.stats(), rec.stats(rec.t >= TE)
    cover = None if mask is None else (float(np.mean(drawn & (mask > 0))), float(np.mean(mask == 0)))
    return dict(case=case, seed=seed, rec=rec, rms=st.get("rms", np.nan), sent=st["sent_share"],
                rms_late=late.get("rms", np.nan), cw=st.get("conf_wrong", 0), ff=st.get("false_fix", np.nan), cover=cover)


def summary(res):
    out = {}
    for c in CASES:
        rs = [r for r in res if r["case"] == c]
        out[c] = dict(X.pooled([r["rec"] for r in rs]), rms_late=X.pooled([r["rec"] for r in rs], lambda r: r.t >= TE)["rms"],
                      rms_seed=[r["rms"] for r in rs], cover=[r["cover"] for r in rs])
    return out


def main():
    res = X.pmap(job, [dict(case=c, seed=s) for c in CASES for s in range(1, 7)])
    sm = summary(res)
    ref = sm["no overlay"]["rms"]
    lines = ["Static OSD overlay on 20 % of the picture, 100 m, drift 0.5 m/s north from 5 s, 6 seeds pooled",
             " case                                    sent    RMS (m)  vs no overlay  RMS after 5 s  false-fix  confident-wrong  RMS per seed"]
    for c in CASES:
        s = sm[c]
        lines.append(f" {c:38s}  {s['sent_share']:6.1%}  {s['rms']:6.3f}   {s['rms'] / ref - 1:+7.1%}        {s['rms_late']:6.3f}"
                     f"      {s['false_fix']:5.1%}       {s['conf_wrong']:3d}          {' '.join(f'{x:.2f}' for x in s['rms_seed'])}")
    cov = [c for c in sm["overlay + built mask"]["cover"] if c]
    lines.append(f"built mask: overlay pixels left as ground {np.mean([c[0] for c in cov]):.2%} of the picture; "
                 f"masked area {np.mean([c[1] for c in cov]):.1%} of the picture")
    out = "\n".join(lines)
    print(out)
    open(os.path.join(X.HERE, "overlay.txt"), "w").write(out + "\n")


if __name__ == "__main__":
    main()
