#!/usr/bin/env python3
"""Rays of the calibrated camera (calib_loop_d.py: new lens profile + mount) against the true camera of camera.py.

calib.solve does not fit the lens centre, and near the optical axis a centre offset looks like a mount tilt, so the
mount alone is not the right score. This compares the whole camera model: for every 4th pixel of the 640x360
picture, the body-frame ray of the found model (hold.load_lens + hold.make_R_bc, as the onboard program builds it)
against the true ray (camera.py's own lens and mount). Also the same for the start settings (full sensor profile,
mount front). Run: ~/.venvs/vio/bin/python integ/justice_d/calib_rays.py <calib_loop_d_settings_*.json>
"""
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), HERE]
import camera as C            # noqa: E402
import hold                   # noqa: E402
from calib_loop_d import TRUE   # noqa: E402

W, H = 640, 360


def rays_found(lens_file, lens_name, top, mirror, tilt, twist):
    K, D = hold.load_lens(lens_file, lens_name, W, H)
    uu, vv = np.meshgrid(np.arange(0, W, 4.0), np.arange(0, H, 4.0))
    n = cv2.fisheye.undistortPoints(np.c_[uu.ravel(), vv.ravel()].reshape(-1, 1, 2), K, D).reshape(-1, 2)
    f = np.c_[n, np.ones(len(n))]
    return (f / np.linalg.norm(f, axis=1, keepdims=True)) @ hold.make_R_bc(top, mirror, tuple(tilt), twist).T


def rays_true():
    K, D = C.lens(TRUE["lens"], W, H)
    uu, vv = np.meshgrid(np.arange(0, W, 4.0), np.arange(0, H, 4.0))
    return C.unproject(uu.ravel(), vv.ravel(), K, D) @ C.mount(TRUE["top"], TRUE["mirror"], TRUE["tilt"], TRUE["twist"]).T


def score(rf, rt):
    ang = np.degrees(np.arccos(np.clip(np.sum(rf * rt, axis=1), -1, 1)))
    off = np.degrees(np.arccos(np.clip(rt[:, 2], -1, 1)))         # off nadir (body down)
    m = off < 55                                                    # Hold's MAX_OFF_NADIR
    return dict(median_deg=round(float(np.median(ang[m])), 3), p95_deg=round(float(np.percentile(ang[m], 95)), 3),
                max_deg=round(float(ang[m].max()), 3), centre_deg=round(float(ang[np.argmin(off)]), 3))


def main(path):
    s = json.load(open(path))
    rt = rays_true()
    found = score(rays_found(os.path.join(PROJ, s["lens_file"]), s["lens_name"], s["top"], s["mirror"], s["tilt_deg"],
                             s["twist_deg"]), rt)
    start = score(rays_found(C.LENS_JSON, "Justice D full sensor (assumed)", "front", False, (0, 0), 0), rt)
    start_mount_true = score(rays_found(C.LENS_JSON, "Justice D full sensor (assumed)", TRUE["top"], False, TRUE["tilt"],
                                        TRUE["twist"]), rt)
    out = dict(settings=os.path.relpath(path, PROJ), rays_within_55deg_of_nadir=dict(
        calibrated=found, start_settings=start, start_lens_true_mount=start_mount_true))
    print(json.dumps(out, indent=1))
    open(path.replace("_settings_", "_rays_").replace(".json", ".txt"), "w").write(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main(sys.argv[1])
