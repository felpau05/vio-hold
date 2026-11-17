#!/usr/bin/env python3
"""apb_main.WindAssist (experiment, off by default): it acts only in GUIDED, moves the target upwind of the
hold point by the change of the push / (Kp Kv), hands back to LOITER on a fault and stays off until GUIDED is selected again.
The FC here is a stand-in object; the closed-loop test is apsim (labels c_wa_*)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import apb_main  # noqa: E402
import settings as S  # noqa: E402
from hold import Series  # noqa: E402

fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'} {text}")


class FakeFC:
    def __init__(self):
        self.mode, self.targets, self.modes_sent = 5, [], []
        self.att, self.imu, self.hgt = Series(1e9), Series(1e9), Series(1e9)
        self.pos = np.array([10.0, -4.0])

    def age(self, kind):
        return 0.0

    def local_pos(self):
        return self.pos.copy()

    def set_mode(self, m):
        self.modes_sent.append(m)
        self.mode = m

    def send_target(self, n, e, d, yaw):
        self.targets.append((n, e, d, yaw))


class E:
    def __init__(self, pos):
        self.pos = np.array(pos, float)


def main():
    s = S.load("/nonexistent")
    fc, wa = FakeFC(), apb_main.WindAssist(s)
    yaw = math.radians(90.0)                          # nose east: body x = east, body y = south
    t = 0.0
    for k in range(200):                              # 2 s of data; wind pushes east at 1.2 m/s^2
        t = k * 0.01
        fc.att.add(t, 0.0, 0.0, yaw, 0.0, 0.0, 0.0)
        fc.imu.add(t, 1.2, 0.0, -9.8)
        fc.hgt.add(t, 50.0)
    wa.step(fc, E([10.0, -4.0]), True, t)
    check(not fc.targets, "LOITER: no position target is sent")
    fc.mode = 4
    wa.step(fc, E([10.0, -4.0]), True, t)
    n0, e0, _, _ = fc.targets[-1]
    check(abs(n0 - 10.0) < 1e-6 and abs(e0 + 4.0) < 1e-6, f"GUIDED entry: the first target is the hold point ({n0:.2f} {e0:.2f})")
    for k in range(60):                               # 0.6 s of fixes; a gust: the push grows from 1.2 to 2.4
        t += 0.01
        fc.imu.add(t, 2.4, 0.0, -9.8)
        fc.att.add(t, 0.0, 0.0, yaw, 0.0, 0.0, 0.0)
        wa.step(fc, E([10.0, -4.0]), True, t)
    n, e, d, y = fc.targets[-1]
    check(abs(n - 10.0) < 0.05 and -4.0 - 0.6 < e < -4.0 - 0.1 and abs(d + 50.0) < 1e-6,
          f"GUIDED: the target moves upwind (west) of the hold point: {n:.2f} {e:.2f} {d:.1f} (hold 10.00 -4.00 -50)")
    check(abs(y - yaw) < 1e-9, "GUIDED: the target keeps the heading at GUIDED entry")
    t += 0.6                                          # no fix for 0.6 s
    fc.imu.add(t, 1.2, 0.0, -9.8)
    nt = len(fc.targets)
    note = wa.step(fc, None, False, t)
    check(fc.modes_sent == [5] and note and "LOITER" in note, f"no fix for 0.6 s: LOITER is sent ({note})")
    fc.mode = 4                                       # the FC still reports GUIDED for a moment
    wa.step(fc, E([10.0, -4.0]), True, t + 0.01)
    check(len(fc.targets) == nt and fc.modes_sent == [5], "after a fault: no new target until GUIDED is selected again")
    fc.mode = 5
    wa.step(fc, E([10.0, -4.0]), True, t + 0.02)
    fc.mode = 4
    wa.step(fc, E([10.0, -4.0]), True, t + 0.03)
    check(len(fc.targets) == nt + 1, "GUIDED selected again: targets again")
    fc.pos = np.array([14.0, -4.0])                   # the fix and the FC position 4 m apart
    wa.step(fc, E([10.0, -4.0]), True, t + 0.04)
    check(fc.modes_sent == [5, 5], "a fix 4 m from the FC position: LOITER")
    print("ALL PASS" if fails == 0 else f"{fails} FAIL")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
