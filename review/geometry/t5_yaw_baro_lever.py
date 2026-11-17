#!/usr/bin/env python3
"""High finding: a change in FC yaw error (or baro error) since the keyframe is multiplied by the
distance from the camera to the CENTROID of the tracked ground points, not by the hold displacement.

    error ~= d_yaw * |centroid|            (perpendicular to the centroid direction)
    error ~= (d_h / h) * |centroid|        (along the centroid direction)

With features on one side only (the river, a car park, shadow) the centroid is ~1/4 of the footprint
width, so at 300 m a 1 deg yaw wander moves the fix by metres. The covariance Hold sends does not
contain this term, and both simulators have a constant yaw bias, so neither can show it.

With a keyframe change the leftover error is (R(d2) - I)(c2 - c1): it cancels when the new keyframe
sees the same features (still hover), and stays when the centroid moved.

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t5_yaw_baro_lever.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I                                # noqa: E402
from common_hover import hover_path, run         # noqa: E402

T = 6.0
TEX_FULL = I.make_texture(seed=5)
TEX_SHORE = I.make_texture(seed=5, water_half="west")


def ramp(amount, t_end=T):
    return lambda t: np.clip(t / t_end, 0, 1) * amount


def ramp_up_down(amount):
    return lambda t: np.interp(t, [0, T / 2, T], [0, amount, 0])


def final_err(out, at):
    rows = [o for o in out if abs(o[0] - at) < 0.25 and not np.isnan(o[2])]
    return np.mean([[o[2], o[3]] for o in rows], 0)


def centroid(hold):
    return np.linalg.norm((hold.kf["G"] - hold.kf["pos"]).mean(0))


print("Still hover, +/-6 deg tilt wobble, exact everything except the injected FC error.")
for h0 in (100, 300):
    pos, att, hf = hover_path(h0, still=True)
    for ground, tex, off in (("textured everywhere", TEX_FULL, (0, 0)),
                             ("water on the west half (shore under the drone)", TEX_SHORE, (0, 0))):
        base, hb = run(h0, pos, att, hf, tex=tex, offset=off, T=T)
        cen = centroid(hb)
        y_out, _ = run(h0, pos, att, hf, tex=tex, offset=off, T=T,
                       att_err_fn=lambda t: np.c_[0 * t, 0 * t, ramp(math.radians(1.0))(t)])
        b_out, _ = run(h0, pos, att, hf, tex=tex, offset=off, T=T, h_err_fn=ramp(1.0))
        e0 = np.linalg.norm(final_err(base, T - 0.3))
        ey = np.linalg.norm(final_err(y_out, T - 0.3))
        eb = np.linalg.norm(final_err(b_out, T - 0.3))
        print(f"  h {h0:3d} m  {ground:47s} |centroid of tracked points| {cen:6.1f} m")
        print(f"         error at end: no FC error {e0:5.2f} m | yaw error 0->1 deg {ey:5.2f} m "
              f"(predicted {math.radians(1.0) * cen:5.2f}) | baro error 0->1 m {eb:5.2f} m "
              f"(predicted {1.0 / h0 * cen:5.2f})")

print(f"\nKeyframe change (h 300 m, shore): yaw error 0 -> 1 deg at {T/2:.0f} s -> 0 deg at {T:.0f} s")
pos, att, hf = hover_path(300, still=True)
yerr = lambda t: np.c_[0 * t, 0 * t, ramp_up_down(math.radians(1.0))(t)]
no_kf, _ = run(300, pos, att, hf, tex=TEX_SHORE, T=T, att_err_fn=yerr)
with_kf, _ = run(300, pos, att, hf, tex=TEX_SHORE, T=T, att_err_fn=yerr, kf_at=T / 2)
for label, out in (("no keyframe change", no_kf), (f"keyframe change at {T/2:.0f} s", with_kf)):
    print(f"  {label:24s} error at {T/2:.0f} s {np.linalg.norm(final_err(out, T / 2)):5.2f} m, "
          f"at {T:.0f} s (yaw error back to 0) {np.linalg.norm(final_err(out, T - 0.3)):5.2f} m")

print("\nConstant yaw error (no change since keyframe) only rotates the move from the anchor:")
pos_move = lambda t: np.c_[np.clip(t / (T / 2), 0, 1) * 5.0, 0 * t]
_, att, hf = hover_path(100)
for dpsi in (5.0, 15.0):
    out, _ = run(100, pos_move, att, hf, tex=TEX_FULL, T=T,
                 att_err_fn=lambda t, d=dpsi: np.c_[0 * t, 0 * t, np.full(len(t), math.radians(d))])
    print(f"  yaw error {dpsi:4.1f} deg, camera moved 5 m north: error {np.linalg.norm(final_err(out, T - 0.3)):5.2f} m "
          f"(predicted 2*sin(d/2)*5 m = {2 * math.sin(math.radians(dpsi) / 2) * 5:4.2f} m)")
