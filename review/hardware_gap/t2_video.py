#!/usr/bin/env python3
"""Item 2: interlaced analog frames, field order, and OSD text burned into the picture.

The sim feeds Hold one 360x240 field. A V4L2 analog grabber gives woven 720x480 frames (two fields
1/60 s apart) unless the code splits them. The APB flight controller also runs an analog OSD chip
("atxxxx start -s" in boards/orqa/apb/init/rc.board_extras), so the picture may carry fixed text.

Part A (100 m, hover): field-only (sim assumption) vs woven frame fed as one picture, vs the wrong
field, vs a blend-deinterlaced frame.
Part B (300 m): the drone drifts 0.25 m/s east (open loop). With OSD text and black capture borders
in the picture, does the hold still see the drift?

Run: ~/.venvs/vio/bin/python t2_video.py       (about 2.5 minutes)
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S
FIELD_DT = 1 / 60.0


def part_a(T=12.0):
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    pos, att_fn, h_fn = X.hover(h0)
    Kf, Df = X.lens(w=720, h=480)
    rays = X.body_rays(Kf, Df, 720, 480, X.R_BC_NADIR)
    fa, fb, t0s = [], [], np.arange(0, T, 1 / X.FPS)
    for t0 in t0s:
        fa.append(X.render(t0, pos, att_fn, h_fn, g, rays=rays[0::2], noise=5.0, fx=Kf[0, 0]))
        fb.append(X.render(t0 + FIELD_DT + S.READOUT / 480, pos, att_fn, h_fn, g, rays=rays[1::2], noise=5.0,
                           fx=Kf[0, 0]))
    att, hgt = X.fc_feeds(att_fn, h_fn, T)
    half = lambda im: cv2.resize(im, (360, 240), interpolation=cv2.INTER_AREA)

    def weave(a, b):
        w = np.empty((480, 720), np.uint8)
        w[0::2], w[1::2] = a, b
        return w

    def blend(a, b):
        up = lambda im, sh: cv2.warpAffine(im.astype(np.float32), np.float32([[1, 0, 0], [0, 2, sh]]), (720, 480),
                                           flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        return np.clip((up(a, 0.0) + up(b, 1.0)) / 2, 0, 255).astype(np.uint8)

    cases = [
        ("field A, width halved (sim assumption)", [(t, half(a)) for t, a in zip(t0s, fa)], X.K0, X.D0, S.READOUT, 0.0),
        ("woven 720x480 fed as one picture, readout 16 ms", [(t, weave(a, b)) for t, a, b in zip(t0s, fa, fb)],
         Kf, Df, S.READOUT, 0.0),
        ("woven 720x480 fed as one picture, readout 33 ms", [(t, weave(a, b)) for t, a, b in zip(t0s, fa, fb)],
         Kf, Df, 0.033, 0.0),
        ("driver blend-deinterlace, stamped mid-way", [(t, blend(a, b)) for t, a, b in zip(t0s, fa, fb)],
         Kf, Df, S.READOUT, FIELD_DT / 2),
        ("field B used, stamped as field A (field order swapped)", [(t, half(b)) for t, b in zip(t0s, fb)],
         X.K0, X.D0, S.READOUT, -FIELD_DT),
    ]
    print(f"A. interlace, {h0} m hover, Ottawa photo, {T:.0f} s")
    for name, frames, K, D, ro, extra in cases:
        # extra: stamp error relative to the true first-row time of the picture that is used
        fr = [(t, im) for t, im in frames]
        if "field B used" in name:
            fr = [(t + FIELD_DT, im) for t, im in frames]       # true first-row time of field B
        res = X.score(X.run(fr, att, hgt, pos, K=K, D=D, readout=ro, t_off=S.FRAME_T_ERR + extra))
        print(f"  {name:55s} {X.fmt(res)}", flush=True)


def osd(img):
    """PX4 atxxxx-style text (white, black outline) plus the black side borders of a 720-wide capture."""
    im = img.copy()
    for txt, org in (("16.4V", (6, 16)), ("02:31", (300, 16)), ("POSCTL", (150, 230)), ("ARMED", (6, 230)),
                     ("ALT 300", (290, 230))):
        cv2.putText(im, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 0, 3, cv2.LINE_AA)
        cv2.putText(im, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 255, 1, cv2.LINE_AA)
    im[:, :5] = 16
    im[:, -5:] = 16
    return im


def part_b(T=20.0):
    print("B. OSD and capture borders, open loop: the drone drifts 0.25 m/s east")
    for h0 in (200, 300):
        g = X.Ground("ottawa2022", h0)
        hp, att_fn, h_fn = X.hover(h0, wander=1.0)
        pos = lambda t: hp(t) + np.c_[np.zeros(len(t)), 0.25 * t]
        frames = X.frames_for(pos, att_fn, h_fn, g, T=T, noise=5.0)
        att, hgt = X.fc_feeds(att_fn, h_fn, T)
        for name, fr in (("clean picture", frames), ("OSD text + borders", [(t, osd(im)) for t, im in frames])):
            rows = X.run(fr, att, hgt, pos)
            res = X.score(rows)
            sent = [r for r in rows if r[2] is not None]
            tt = np.array([r[0] for r in sent])
            est_e = np.array([r[3][1] for r in sent])
            true_e = np.array([r[4][1] for r in sent])
            seen = np.polyfit(tt, est_e, 1)[0] if len(tt) > 10 else float("nan")
            true = np.polyfit(tt, true_e, 1)[0] if len(tt) > 10 else float("nan")
            print(f"  {h0} m {name:20s} {X.fmt(res)}  east drift seen {seen:+.3f} m/s (true {true:+.3f})",
                  flush=True)


if __name__ == "__main__":
    part_a()
    part_b()
