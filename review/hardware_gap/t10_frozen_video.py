#!/usr/bin/env python3
"""Item 9: video stops but frames keep coming (frozen frame, or a grabber's "NO SIGNAL" screen).

Analog USB grabbers and some V4L2 drivers keep delivering the last frame, or a blue/black screen
with text, when the analog signal drops. Hold has no check for a picture that does not move.
Open loop, Ottawa photo, 100 m: the drone drifts 0.5 m/s east; from 5 s to 10 s the video is bad.
Does Hold stop sending, or does it send 'ok' fixes that miss the drift?

Run: ~/.venvs/vio/bin/python t10_frozen_video.py      (about 40 s)
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S
T, BAD = 12.0, (5.0, 10.0)


def no_signal():
    im = np.full((X.H, X.W), 60, np.uint8)            # grey-blue field as luma
    cv2.putText(im, "NO SIGNAL", (95, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 235, 2, cv2.LINE_AA)
    cv2.putText(im, "AV1  NTSC", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, 235, 1, cv2.LINE_AA)
    return im


def main():
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    hp, att_fn, h_fn = X.hover(h0, wander=1.0)
    pos = lambda t: hp(t) + np.c_[np.zeros(len(t)), 0.5 * np.asarray(t)]
    frames = X.frames_for(pos, att_fn, h_fn, g, T=T, noise=5.0)
    att, hgt = X.fc_feeds(att_fn, h_fn, T)
    frozen_img = [im for t, im in frames if t >= BAD[0]][0]
    ns = no_signal()
    rng = np.random.default_rng(2)
    cases = {
        "frozen frame (same buffer repeated)": [(t, frozen_img if BAD[0] <= t < BAD[1] else im) for t, im in frames],
        "frozen frame + fresh sensor noise": [(t, np.clip(frozen_img + rng.normal(0, 3, frozen_img.shape), 0, 255)
                                                .astype(np.uint8) if BAD[0] <= t < BAD[1] else im) for t, im in frames],
        "grabber NO SIGNAL screen": [(t, np.clip(ns + rng.normal(0, 2, ns.shape), 0, 255).astype(np.uint8)
                                      if BAD[0] <= t < BAD[1] else im) for t, im in frames],
    }
    print(f"{h0} m, drone drifts 0.5 m/s east; video bad from {BAD[0]:.0f} s to {BAD[1]:.0f} s")
    for name, fr in cases.items():
        rows = X.run(fr, att, hgt, pos, reanchor_err=None)
        bad = [r for r in rows if BAD[0] + 0.5 <= r[0] < BAD[1]]
        sent = [r for r in bad if r[2] is not None]
        states = [r[1] for r in bad]
        err_end = [np.linalg.norm(r[3] - r[4]) for r in bad if r[2] is not None][-1:] or [float("nan")]
        print(f"  {name:38s} during the bad video: fixes sent {len(sent) / len(bad):4.0%} "
              f"('ok' {states.count('ok') / len(bad):4.0%}, lost {states.count('lost') / len(bad):4.0%}); "
              f"error of the last fix sent {err_end[0]:.2f} m (drift in 5 s = 2.5 m)", flush=True)


if __name__ == "__main__":
    main()
