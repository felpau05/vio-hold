#!/usr/bin/env python3
"""Lever of a yaw error: how far one fix moves when the attitude yaw Hold gets is off by +1 deg (against the yaw the
keyframe was made with). Two Hold copies make the same keyframe from frame A (FC attitude); frame B (5 frames = 1/12 s
later) is processed with the FC attitude and with the FC yaw + 1 deg. lever = fix difference / (1 deg in rad), in m.
'dense' forces the whole-picture path on the keyframe (kf['dense'] = True); 'corner' keeps the corner path.
Frames of a recorded flight (default TUNING: lowap_apb_10m_w20g40_s1 on ottawa2019fall, seed 1), FC ATTITUDE as sent.
Run: ~/.venvs/vio/bin/python integ/diag10/probe_yaw.py --hold base=integ/diag10/hold_base.py [--hold ...] [--height H]
Writes integ/diag10/results/probe_yaw_<flight>[_<H>m].json"""
import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np
import cv2

cv2.setNumThreads(1)
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(PROJ, "px4sim")]
sys.dont_write_bytecode = True
import camera as C   # noqa: E402
import replay as RP  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--hold", action="append", required=True)
ap.add_argument("--flight", default="lowap_apb_10m_w20g40_s1")
ap.add_argument("--ground", default="ottawa2019fall")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--height", type=float, default=None)
ap.add_argument("--times", default="6,14,22,30")
a = ap.parse_args()
hm0 = RP.load_hold(os.path.join(HERE, "hold_base.py"), "hb0")
pose, att, ekf, dh, dy, te, tr = RP.flight(a.flight, a.height, hm0.Series)
c = dict(RP.JD, seed=a.seed)
cam = C.Camera(c, C.make_ground(a.ground))


def series(dyaw):
    S = hm0.Series(1e9)
    for r in att[(att[:, 0] > te - 5) & (att[:, 0] < te + 45)][::2]:
        r = r.copy()
        r[3] += dyaw
        S.add(*r)
    return S


A0, A1 = series(0.0), series(math.radians(1.0))
H = hm0.Series(1e9)
for r in ekf[(ekf[:, 0] > te - 5) & (ekf[:, 0] < te + 45)][::3]:
    H.add(r[0], -r[3] + dh)
OKW = (None, "waiting for image motion that proves the picture is live")
out = {}
frames = {}
for ts in [float(x) for x in a.times.split(",")]:
    tA = te + ts + cam.exposure / 2
    tB = tA + 5 / cam.fps
    frames[ts] = (tA, tB, cam.finish(cam.render(tA, pose)), cam.finish(cam.render(tB, pose)))
for spec in a.hold:
    name, path = spec.split("=", 1)
    hm = RP.load_hold(os.path.abspath(path), "h_" + name)
    m = C.calib_lens(c["true_lens"])
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "lens.json")
        json.dump({"models": [m]}, open(p, "w"))
        K, D = hm.load_lens(p, m["name"], cam.W, cam.H)
    for mode in ("dense", "corner"):
        rows = []
        for ts, (tA, tB, imA, imB) in frames.items():
            res = []
            for S in (A0, A1):
                h = hm.Hold(K, D, hm.make_R_bc(), c["readout_s"], **RP.HOLD_CFG)
                h.reanchor(np.zeros(2))
                h.process(imA, tA, A0, H)
                if h.kf is None:
                    break
                h.kf["dense"] = mode == "dense"
                e1 = h.process(imB, tB, S, H)
                res.append((e1.pos.copy(), e1.why, e1.n))
            if len(res) < 2 or res[0][1] not in OKW or res[1][1] not in OKW:
                rows.append(dict(t=ts, fail=[r[1] for r in res]))
                continue
            dpos = (res[1][0] - res[0][0]) / math.radians(1.0)
            _, _, _, q = pose(tA)
            r_, p_ = C.q_roll_pitch(q)
            rows.append(dict(t=ts, lever_N=round(float(dpos[0]), 2), lever_E=round(float(dpos[1]), 2),
                             lever=round(float(np.linalg.norm(dpos)), 2), n=res[0][2],
                             roll=round(math.degrees(float(r_[0])), 1), pitch=round(math.degrees(float(p_[0])), 1)))
        L = [r["lever"] for r in rows if "lever" in r]
        print(f"{name} {mode}: median lever {np.median(L) if L else None} m per rad of yaw error; {rows}", flush=True)
        out[f"{name}_{mode}"] = rows
tag = "" if a.height is None else f"_{a.height:g}m"
json.dump(out, open(os.path.join(HERE, "results", f"probe_yaw_{a.flight}{tag}.json"), "w"), indent=0)
