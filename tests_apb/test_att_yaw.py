#!/usr/bin/env python3
"""The attitude feed option att_source "gyro_fcyaw" (apb_main.GyroAtt with fc_yaw): roll and pitch from the
integrated gyro rates as in "gyro", yaw as the flight controller (FC) sends it.
Y1 the default "gyro" gives the same numbers as the code before the option (a copy of the old class in this file);
Y2 "gyro_fcyaw": the yaw equals the FC yaw in each row, and roll and pitch stay within 0.01 deg of "gyro" while
   the FC yaw does not jump;
Y3 an FC yaw jump of 20 deg (ArduCopter's in-flight yaw alignment after take-off): "gyro" still has a yaw error of
   more than 10 deg after 2 s, "gyro_fcyaw" has none, and its roll and pitch do not move by more than 0.05 deg;
Y4 att_filter and the settings accept the value, and the default stays "gyro";
Y5 the interlock: Hold's option dense_turn with any other att_source stops the start (apb_main.turn_problems).
Run: ~/.venvs/vio/bin/python tests_apb/test_att_yaw.py      (no simulator, about 2 s)
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import apb_main            # noqa: E402
import settings            # noqa: E402
from hold import euler_R   # noqa: E402

fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}")


class OldGyroAtt:
    """apb_main.GyroAtt as it was on 2026-11-21 02:08 (the tested bundle), copied here as the reference of Y1."""

    def __init__(self, tau_s):
        self.tau, self.R, self.t, self.w = tau_s, None, None, None

    def __call__(self, row):
        if len(row) < 7:
            return row
        t, w = row[0], np.asarray(row[4:7], float)
        R_fc = euler_R(*row[1:4])
        dt = math.inf if self.t is None else t - self.t
        if dt > 0.5:
            self.R = R_fc
        elif dt > 0:
            self.R = self.R @ cv2.Rodrigues((w + self.w) / 2 * dt)[0]
            e = cv2.Rodrigues(R_fc @ self.R.T)[0]
            self.R = cv2.Rodrigues(e * min(1.0, dt / self.tau))[0] @ self.R
            u, _, vt = np.linalg.svd(self.R)
            self.R = u @ vt
        self.t, self.w = t, w
        R = self.R
        return (t, math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
                math.atan2(R[1, 0], R[0, 0]), *row[4:])


def flight(n=600, hz=100.0, jump_at=None, jump_deg=20.0, seed=3):
    """ATTITUDE rows (t, roll, pitch, yaw, p, q, r) of a hover with small true motion. The FC attitude is the truth
    plus a slow tilt wander (the bias that the option must not let in) and, from jump_at, a yaw step."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / hz
    roll = np.radians(3.0) * np.sin(2 * math.pi * 0.4 * t)
    pitch = np.radians(2.0) * np.sin(2 * math.pi * 0.3 * t + 1.0)
    yaw = np.radians(40.0) + np.radians(1.5) * np.sin(2 * math.pi * 0.1 * t)
    rows = []
    for i in range(n):
        j = min(i + 1, n - 1)
        Ri, Rj = euler_R(roll[i], pitch[i], yaw[i]), euler_R(roll[j], pitch[j], yaw[j])
        w = cv2.Rodrigues(Ri.T @ Rj)[0].ravel() * hz + rng.normal(0, 0.002, 3)       # body rates with gyro noise
        fc_yaw = yaw[i] + (math.radians(jump_deg) if jump_at is not None and t[i] >= jump_at else 0.0)
        rows.append((t[i], roll[i] + np.radians(0.5) * math.sin(0.2 * t[i]), pitch[i], fc_yaw, *w))
    return rows, np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def wrap(deg):
    return (np.asarray(deg) + 180.0) % 360.0 - 180.0


def main():
    print("Y1 the default feed is the code of the tested bundle")
    rows, *_ = flight()
    new, old = apb_main.GyroAtt(20.0), OldGyroAtt(20.0)
    a, b = [new(r) for r in rows], [old(r) for r in rows]
    check(a == b, f"GyroAtt(20) equals the old class on {len(rows)} rows, bit for bit")

    print("Y2 gyro_fcyaw: the FC yaw, our roll and pitch")
    f = apb_main.GyroAtt(20.0, fc_yaw=True)
    c = [f(r) for r in rows]
    dy = max(abs(x[3] - r[3]) for x, r in zip(c, rows))
    check(dy == 0.0, f"yaw equals the FC yaw in each row (largest difference {dy:g} rad)")
    drp = max(max(abs(x[1] - y[1]), abs(x[2] - y[2])) for x, y in zip(c, a))
    check(math.degrees(drp) < 0.01, f"roll and pitch within 0.01 deg of the default (largest {math.degrees(drp):.4f} deg)")

    print("Y3 an FC yaw jump of 20 deg at 2.0 s")
    rows, roll, pitch, yaw = flight(jump_at=2.0)
    g, f = apb_main.GyroAtt(20.0), apb_main.GyroAtt(20.0, fc_yaw=True)
    a, c = np.array([g(r)[:4] for r in rows]), np.array([f(r)[:4] for r in rows])
    fc = np.array([r[3] for r in rows])
    i = int(4.0 * 100)                                                    # 2 s after the jump
    e_gyro = abs(float(wrap(np.degrees(a[i, 3] - fc[i]))))
    e_fcyaw = abs(float(wrap(np.degrees(c[i, 3] - fc[i]))))
    check(e_gyro > 10.0, f"the default still differs from the FC yaw by {e_gyro:.1f} deg 2 s after the jump (the cause)")
    check(e_fcyaw == 0.0, f"gyro_fcyaw differs by {e_fcyaw:g} deg")
    j = slice(int(1.9 * 100), int(2.3 * 100))                              # across the jump
    step = max(float(np.abs(np.diff(np.degrees(c[j, 1]) - roll[j])).max()),
               float(np.abs(np.diff(np.degrees(c[j, 2]) - pitch[j])).max()))
    check(step < 0.05, f"roll and pitch error move by {step:.4f} deg or less in each step across the jump (limit 0.05)")
    tilt = max(float(np.abs(np.degrees(c[300:, 1]) - roll[300:]).max()), float(np.abs(np.degrees(c[300:, 2]) - pitch[300:]).max()))
    check(tilt < 0.6, f"largest roll or pitch error after the jump {tilt:.3f} deg (the FC tilt wander is 0.5 deg)")

    print("Y4 the setting")
    s = settings.load("/nonexistent")
    check(s["att_source"] == "gyro", f"default att_source is {s['att_source']!r}")
    check(type(apb_main.att_filter(s)) is apb_main.GyroAtt and not apb_main.att_filter(s).fc_yaw, "att_filter: gyro -> GyroAtt, FC yaw off")
    s2 = dict(s, att_source="gyro_fcyaw")
    check(apb_main.att_filter(s2).fc_yaw is True, "att_filter: gyro_fcyaw -> GyroAtt with the FC yaw")
    check("gyro_fcyaw" in settings.CHOICES["att_source"], "settings accepts att_source gyro_fcyaw")

    print("Y5 the interlock for hold_cfg dense_turn")
    from types import SimpleNamespace
    check(apb_main.turn_problems(s) == [], "no dense_turn: no problem")
    s3 = dict(s, hold_cfg=dict(s["hold_cfg"], dense_turn=True))
    bad = apb_main.turn_problems(s3)
    check(len(bad) == 1 and "gyro_fcyaw" in bad[0], f"dense_turn with att_source gyro: {bad}")
    check(apb_main.turn_problems(dict(s3, att_source="gyro_fcyaw")) == [], "dense_turn with gyro_fcyaw: no problem")
    why = apb_main.start_problems(s3, SimpleNamespace(firmware="ardupilot"), SimpleNamespace(info={}), (360, 640))
    check(any("dense_turn needs att_source gyro_fcyaw" in w for w in why), "start_problems stops the run for it")
    print("ALL PASS" if not fails else f"{fails} FAIL")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
