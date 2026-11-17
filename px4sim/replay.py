#!/usr/bin/env python3
"""Replay a logged closed-loop run open loop, to find where the estimate error comes from.

Re-renders the camera from the logged truth and runs Hold three times on the same frames:
  px4  - with PX4's logged ATTITUDE (what an 'ours' run used)
  gyro - with the gyro-propagated attitude computed from that log (what an 'ours_gyro' run used)
  true - with the true attitude, turned by PX4's mean yaw offset (a perfect flight controller)
and prints the error of each against the truth in PX4's heading frame.
Run: ~/.venvs/vio/bin/python px4sim/replay.py px4sim/runs/<case>.npz [seconds]
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import camera as C  # noqa: E402
import hold  # noqa: E402
import sitl  # noqa: E402
from sitl import DEFAULTS  # noqa: E402


def main(path, secs=20.0):
    d = np.load(path)
    tr, att, ekf, te = d["truth"], d["att"], d["ekf"], float(d["t_engage"])
    cfg = dict(DEFAULTS)
    side = os.path.splitext(path)[0] + ".json"
    if os.path.exists(side):
        cfg.update(json.load(open(side)).get("cfg", {}))
    cam = C.Camera(cfg, C.photo_ground(cfg["ground"]))

    def pose(t):
        i = np.clip(np.searchsorted(tr[:, 0], t) - 1, 0, len(tr) - 2)
        f = np.clip((t - tr[i, 0]) / (tr[i + 1, 0] - tr[i, 0]), 0, 1)[:, None]
        v = tr[i, 1:] * (1 - f) + tr[i + 1, 1:] * f
        return v[:, 0], v[:, 1], v[:, 2], v[:, 3:] / np.linalg.norm(v[:, 3:], axis=1)[:, None]

    ty = C.q_yaw(tr[:, 4:])
    dy = float(np.angle(np.mean(np.exp(1j * (np.interp(tr[:, 0], att[:, 0], np.unwrap(att[:, 3])) - ty)))))
    r, p = C.q_roll_pitch(tr[:, 4:])
    att_true = np.c_[tr[:, 0], r, p, ty + dy]
    hgt = hold.Series(1e9)
    z0 = ekf[0, 3]
    for row in ekf:
        hgt.add(row[0], z0 - row[3])
    R = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])
    K, D = hold.load_lens(C.LENS_JSON, cfg["model_lens"], cam.W, cam.H)
    t0s = np.arange(math.ceil((te - 2) * cam.fps), math.floor((te + secs) * cam.fps)) / cam.fps
    frames = [(t0 + cam.exposure / 2, cam.finish(cam.render(t0 + cam.exposure / 2, pose))) for t0 in t0s]
    out = {}
    L = sitl.Link.__new__(sitl.Link)                 # the gyro-propagated attitude, as the closed loop computed it
    L.c, L.q_f = dict(cfg, att_source="gyro"), None
    att_gyro = np.array([L._gyro_att(tuple(r))[:4] for r in att])
    for name, rows in (("px4", att[:, :4]), ("gyro", att_gyro), ("true", att_true[::2])):
        A = hold.Series(1e9)
        for row in rows:
            A.add(*row)
        H = hold.Hold(K, D, hold.make_R_bc(), cfg["readout_s"])
        H.reanchor(np.zeros(2))
        errs, base = [], None
        for t0, img in frames:
            e = H.process(img, t0, A, hgt)
            if e.var is None:
                continue
            N, E, _, _ = pose(np.array([e.t]))
            P = R @ np.array([N[0], E[0]])
            if base is None:
                base = e.pos - P
            errs.append((e.t - te, *(e.pos - P - base), e.var))
        errs = np.array(errs)
        rms = np.sqrt(np.mean(errs[:, 1] ** 2 + errs[:, 2] ** 2))
        out[name] = errs
        print(f"{name:5s} attitude: fixes {len(errs)}  error RMS {rms:.2f} m  max {np.hypot(errs[:, 1], errs[:, 2]).max():.2f} m  "
              f"median sent sigma {np.sqrt(np.median(errs[:, 3])):.2f} m")
    return out


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 20.0)
