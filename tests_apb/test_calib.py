#!/usr/bin/env python3
"""calib.solve on synthetic hand-rotation recordings: all 8 picture orientations, with a known video
delay, readout (both signs), focal scale and mount tilt. Then apb_main calib --rec end to end
(settings written, calibrated set), and a recording with too little rotation (refused).

Renders with the reviewer's harness (review/hardware_gap/harness.py, read only): the true camera
differs from the lens profile and from the default mount. Far scene (60 m), 0.2 m hand pivot.
Run: ~/.venvs/vio/bin/python tests_apb/test_calib.py      (about 2 minutes)
"""
import json
import math
import os
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "review", "hardware_gap"))
import harness as X          # noqa: E402
import apb_main              # noqa: E402
import calib                 # noqa: E402
import settings as S         # noqa: E402
from hold import make_R_bc   # noqa: E402

T = 10.0


def hand_att(t, k=1.0):
    return np.radians(k * np.c_[15 * np.sin(2 * np.pi * 0.5 * t), 12 * np.sin(2 * np.pi * 0.4 * t + 1),
                                20 * np.sin(2 * np.pi * 0.3 * t)])


def recording(R_true, fscale, latency, readout, h0=60.0, lever=0.2, k=1.0):
    K = X.K0.copy()
    K[0, 0] *= fscale
    K[1, 1] *= fscale
    rays = X.body_rays(K, X.D0, X.W, X.H, R_true)
    a = lambda t: hand_att(t, k)                                               # noqa: E731
    pos = lambda t: lever * np.c_[np.sin(a(t)[:, 1]), -np.sin(a(t)[:, 0])]    # noqa: E731
    hf = lambda t: np.full(len(t), float(h0))                                  # noqa: E731
    g = X.Ground("synth", h0 * 1.5)
    t0s = np.arange(0, T, 1 / X.FPS)
    imgs = np.array([X.render(t0, pos, a, hf, g, rays=rays, readout=readout) for t0 in t0s])
    att, _ = X.fc_feeds(a, hf, T)
    return dict(imgs=imgs, t_grab=t0s + latency, att=att.array(), K=X.K0, D=X.D0,
                hgt=np.array([[-1, h0], [T + 1, h0]]), mode=np.array("sim"))


def expected_delay(latency, readout):
    # harness t0 is the first row READ; for a bottom-to-top sensor that is the bottom row, while
    # Hold's t0 is row 0 (the top row), read (1 - 1/H) * |readout| later. Attitude is 3 ms late.
    return latency - X.S.ATT_LAG - (abs(readout) * (1 - 1 / X.H) if readout < 0 else 0.0)


def main():
    t_start = time.time()
    s = S.load("/nonexistent")
    fails = 0
    # The sim FC attitude has a roll/pitch bias (sim_test.ATT_BIAS); it acts as mount tilt (H8), so the
    # calibration must find mount tilt minus that bias.
    br, bp = np.degrees(X.S.ATT_BIAS[:2])
    print(f"A. 8 orientations; true mount tilt roll +2.0 pitch -1.5 deg, twist +1.0 deg; FC bias {br:+.1f} {bp:+.1f} deg")
    for i, (top, mirror) in enumerate(calib.ORIENTATIONS):
        fscale = (0.96, 1.03)[i % 2]
        latency = (0.075, 0.040, 0.120, 0.060)[i % 4]
        readout = (0.016, -0.016, 0.020, -0.012)[(i // 2) % 4]
        rec = recording(make_R_bc(top, mirror, (2.0, -1.5), 1.0), fscale, latency, readout)
        r = calib.solve(rec, s)
        want = expected_delay(latency, readout)
        ok = (r["ok"] and r["top"] == top and r["mirror"] == mirror and abs(r["video_delay_s"] - want) < 0.005
              and abs(r["readout_s"] - readout) < 0.0045 and abs(r["focal_scale"] - fscale) < 0.01
              and abs(r["tilt_deg"][0] - (2.0 - br)) < 0.7 and abs(r["tilt_deg"][1] - (-1.5 - bp)) < 0.7
              and abs(r["twist_deg"] - 1.0) < 0.7)
        fails += not ok
        print(f"  {'PASS' if ok else 'FAIL'} true {top:5s} mirror {str(mirror):5s} -> found {r.get('top')} "
              f"mirror {r.get('mirror')}; delay {r.get('video_delay_s', math.nan) * 1e3:.0f} ms (want {want * 1e3:.0f}), "
              f"readout {r.get('readout_s', math.nan) * 1e3:+.0f} ms (true {readout * 1e3:+.0f}), focal "
              f"{r.get('focal_scale', math.nan):.3f} (true {fscale}), tilt {np.round(r.get('tilt_deg', [math.nan] * 2), 2)} "
              f"(want [{2.0 - br:.2f} {-1.5 - bp:.2f}]) "
              f"twist {r.get('twist_deg', math.nan):+.2f}; resid {r.get('resid_deg', math.nan):.3f} deg vs next "
              f"{r.get('runner_up', {}).get('resid_deg', math.nan):.2f} ({r.get('ratio', 0):.0f}x) "
              f"{r.get('seconds')} s {r.get('why') or ''}", flush=True)

    print("B. apb_main calib --rec: writes settings and the calibrated flag")
    rec = recording(make_R_bc("rear", False), 1.0, 0.070, 0.016)
    with tempfile.TemporaryDirectory() as d:
        path, sp = os.path.join(d, "rec.npz"), os.path.join(d, "settings.json")
        np.savez(path, **rec)
        S.save(dict(s, log_dir=d), sp)
        lines = []
        rc = apb_main.mode_calib(S.load(sp), SimpleNamespace(rec=path, seconds=None, settings=sp), say=lines.append)
        w = json.load(open(sp))
        ok = rc == 0 and w["calibrated"] and w["top"] == "rear" and not w["mirror"] and w["calib_capture"] == "sim"
        fails += not ok
        print(f"  {'PASS' if ok else 'FAIL'} rc {rc}, settings top={w['top']} mirror={w['mirror']} calibrated="
              f"{w['calibrated']} delay {w['video_delay_s'] * 1e3:.0f} ms; output: {' | '.join(lines[-4:])}")
        # Second run on the same recording: the delay must repeat (event-day rule: within 3 ms).
        lines = []
        apb_main.mode_calib(S.load(sp), SimpleNamespace(rec=path, seconds=None, settings=sp), say=lines.append)
        rep = [x for x in lines if "previous calibration" in x]
        ok = bool(rep) and rep[0].startswith("PASS")
        fails += not ok
        print(f"  {'PASS' if ok else 'FAIL'} repeat: {rep[0] if rep else 'no repeat line'}")

    print("C. too little rotation is refused")
    rec = recording(make_R_bc(), 1.0, 0.070, 0.016, k=0.2)
    r = calib.solve(rec, s)
    ok = not r["ok"] and "turned too little" in r["why"]
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} ok={r['ok']} why: {r.get('why')}")
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t_start:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
