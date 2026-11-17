#!/usr/bin/env python3
"""Open-loop check: the independent renderer (camera.py) against hold.py, with exact inputs.

Each case moves the drone 2 m north in 1 s at 50 m and asks Hold for the move. If the two
geometries agree, Hold reports (+2, 0) for every matched mount and pose; a mismatched mount must
show up as a wrong move. Writes px4sim/out/check_camera.txt.
Run: ~/.venvs/vio/bin/python px4sim/check_camera.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import camera as C  # noqa: E402
import hold  # noqa: E402
from sitl import DEFAULTS  # noqa: E402

OUT = os.path.join(HERE, "out")


def euler_of(q):
    r, p = C.q_roll_pitch(q)
    return r, p, C.q_yaw(q)


def case(name, yaw=0.0, roll=0.0, pitch=0.0, cam=None, est=None, move=(2.0, 0.0), h0=50.0, ground=None):
    c = dict(DEFAULTS, **(cam or {}))
    cam_ = C.Camera(c, ground)
    est = est or {}
    q0 = C.qmul(C.q_axis(C.E_D, math.radians(yaw)),
                C.qmul(C.q_axis(C.E_E, math.radians(pitch)), C.q_axis(C.E_N, math.radians(roll))))

    def pose(t):
        s = np.clip(np.asarray(t) / 1.0, 0, 1)
        return move[0] * s, move[1] * s, np.full(len(t), h0), np.broadcast_to(q0, (len(t), 4))
    att, hgt = hold.Series(1e9), hold.Series(1e9)
    r, p, y = euler_of(q0)
    for t in np.arange(-1, 3, 0.01):
        att.add(t, float(r), float(p), float(y), 0.0, 0.0, 0.0)
        hgt.add(t, h0)
    K, D = hold.load_lens(C.LENS_JSON, c["model_lens"], cam_.W, cam_.H)
    R_bc = hold.make_R_bc(top=est.get("top", c["cam_top"]), mirror=est.get("mirror", c["cam_mirror"]),
                          tilt_deg=est.get("tilt", c["cam_tilt"]), twist_deg=est.get("twist", c["cam_twist"]))
    H = hold.Hold(K, D, R_bc, c["readout_s"])
    H.reanchor(np.zeros(2))
    e = None
    for k in range(int(1.4 * cam_.fps)):
        t0 = k / cam_.fps
        e = H.process(cam_.finish(cam_.render(t0, pose)), t0, att, hgt)
    got = e.pos if e.var is not None else np.array([np.nan, np.nan])
    err = float(np.hypot(*(got - np.array(move))))
    return f"{name:42s} est move N {got[0]:+6.2f} E {got[1]:+6.2f}  (true {move[0]:+.1f}, {move[1]:+.1f})  err {err:5.2f} m  state {e.state}", err


def main():
    os.makedirs(OUT, exist_ok=True)
    g = C.photo_ground("ottawa2019fall")
    exact = dict(true_lens=DEFAULTS["model_lens"], vib_deg=0.0)
    rows = [
        case("nose north", cam=exact, ground=g),
        case("nose east (yaw 90)", yaw=90, cam=exact, ground=g),
        case("nose south-west (yaw -135), move east", yaw=-135, move=(0, 2), cam=exact, ground=g),
        case("pitch +10 deg", pitch=10, cam=exact, ground=g),
        case("roll +10 deg, yaw 30", roll=10, yaw=30, cam=exact, ground=g),
        case("picture top -> right wing (both)", cam=dict(exact, cam_top="right"), ground=g),
        case("picture mirrored (both)", cam=dict(exact, cam_mirror=True), ground=g),
        case("mount tilt (4, -3) twist 8, pitch 8 (both)", pitch=8,
             cam=dict(exact, cam_tilt=(4.0, -3.0), cam_twist=8.0), ground=g),
        case("readout -16 ms (both)", pitch=5, cam=dict(exact, readout_s=-0.0184), ground=g),
        case("default true lens differs from model", ground=g),
        case("MISMATCH: true top rear, Hold assumes front", cam=dict(exact, cam_top="rear"),
             est=dict(top="front"), ground=g),
    ]
    lines = [r for r, _ in rows]
    ok = all(e < 0.15 for _, e in rows[:9]) and rows[-1][1] > 1.0
    lines.append("PASS" if ok else "FAIL")
    txt = "\n".join(lines)
    print(txt)
    open(os.path.join(OUT, "check_camera.txt"), "w").write(txt + "\n")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
