"""Render the indoor tuning floors (and scene options) from a still pose, save a montage, count corners.
Hold-out floors are only built and rendered to check that the code runs: no picture and no number is kept.
Run: ~/.venvs/vio/bin/python integ/indoor/render_check.py"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "px4sim"))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
import camera as C  # noqa: E402
import sitl  # noqa: E402


def pose_at(h):
    def pose(t):
        t = np.atleast_1d(t)
        q = np.tile([1.0, 0, 0, 0], (len(t), 1))
        return np.zeros(len(t)), np.zeros(len(t)), np.full(len(t), h), q
    return pose


def render(ground, h, **kw):
    c = dict(sitl.DEFAULTS, exposure_s=0.004, **kw)
    cam = C.Camera(c, ground)
    return cam.finish(cam.render(1.0, pose_at(h)))


if __name__ == "__main__":
    rows, lines = [], []
    for name in ("floor_concrete", "floor_tiles", "floor_carpet", "floor_wood", "floor_shiny"):
        g = C.make_ground(name)
        row = []
        for h in (2, 5, 10, 15):
            img = render(g, h, reflect=name.endswith("shiny"))
            n = cv2.goodFeaturesToTrack(img, 600, 0.005, 7)
            n = 0 if n is None else len(n)
            q = cv2.resize(img, (45, 36), interpolation=cv2.INTER_AREA).astype(float)
            lines.append(f"{name:16s} h {h:2d} m  corners {n:3d}  std {img.std():5.1f}  MAD(1/8) {np.median(np.abs(q - np.median(q))):4.1f}")
            row.append(img)
        rows.append(np.hstack(row))
    g = C.make_ground("floor_concrete")
    extra = [render(g, 5, flicker_hz=120, seed=1), render(g, 5, people=3, seed=1), render(g, 2, people=3, seed=2),
             render(g, 10, props=True, seed=1)]
    rows.append(np.hstack(extra))
    cv2.imwrite(os.path.join(HERE, "render_check.png"), np.vstack(rows))
    for name in ("holdout_floor_a", "holdout_floor_b"):      # runs, nothing kept
        img = render(C.make_ground(name), 5)
        lines.append(f"{name}: renders ({img.shape[1]}x{img.shape[0]}); not looked at")
    open(os.path.join(HERE, "render_check.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
