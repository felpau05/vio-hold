#!/usr/bin/env python3
"""apb_main bench mode (event-day ground tests G1/H3) in the simulator: the lens 1.5 m over a floor,
hand-held wobble of +-3 deg, nose at 0.3 rad; the drone is carried 1 m toward the nose, then 1 m to
the right. The printed moves must read forward 1.0 m, then right 1.0 m, each within 0.1 m.
The fake FC reports 0.2 m height (on the ground), so the fixed bench height must be used.
Run: ~/.venvs/vio/bin/python tests_apb/test_bench.py      (about 25 s)
"""
import os
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import apb_main            # noqa: E402
import capture             # noqa: E402
import settings as S       # noqa: E402
import sim_test            # noqa: E402
from test_run import LAT, SimCamera   # noqa: E402

PORT = 14680


def main():
    t_begin = time.time()
    d = tempfile.mkdtemp()
    t_start = time.monotonic()
    fake = subprocess.Popen([sys.executable, os.path.join(HERE, "fake_fc.py"), "--port", str(PORT), "--t-start",
                             repr(t_start), "--log", os.path.join(d, "fake.jsonl"), "--scenario", "bench",
                             "--seconds", "60"])
    cv2.imwrite(os.path.join(d, "mask.png"), np.full((sim_test.H, sim_test.W), 255, np.uint8))
    s = dict(S.load("/nonexistent"), fc_device=f"udpin:127.0.0.1:{PORT}", msp_device="/nonexistent", calibrated=True,
             video_delay_s=LAT, readout_s=sim_test.READOUT, field="frame", log_dir=d,
             video_kind="analog", lens_name="Round 1 independent k2", mask_file=os.path.join(d, "mask.png"))
    s["calib_capture"] = capture.capture_mode(s, {}, (sim_test.H, sim_test.W))
    src = capture.CallableSource(SimCamera(t_start, "bench", 12.0), s)
    apb_main.open_video = lambda s: src
    lines = []
    say = lambda x: lines.append((time.monotonic() - t_start, x))      # noqa: E731
    try:
        rc = apb_main.mode_bench(s, SimpleNamespace(height=1.5, seconds=None), say=say, seconds=14.0 - (time.monotonic() - t_start))
    finally:
        fake.terminate()
        fake.wait()
    moves = [(t, float(x.split()[1]), float(x.split()[4])) for t, x in lines if x.startswith("forward")]
    for t, x in lines[::3]:
        print(f"  t {t:5.1f} s  {x}")
    at = lambda t0: min(moves, key=lambda m: abs(m[0] - t0))                # noqa: E731
    m8, m13 = at(8.0), at(13.5)
    ok1 = abs(m8[1] - 1.0) < 0.1 and abs(m8[2]) < 0.1
    ok2 = abs(m13[1] - 1.0) < 0.1 and abs(m13[2] - 1.0) < 0.1
    wob = [m for m in moves if m[0] < 4.0]
    ok0 = bool(wob) and max(abs(m[1]) + abs(m[2]) for m in wob) < 0.1
    print(f"  {'PASS' if ok0 else 'FAIL'} before the move (wobble +-3 deg only): largest move "
          f"{max((abs(m[1]) + abs(m[2]) for m in wob), default=float('nan')):.2f} m (H3 tilt test: under 0.10 m)")
    print(f"  {'PASS' if ok1 else 'FAIL'} after 1 m toward the nose: forward {m8[1]:+.2f} right {m8[2]:+.2f} m")
    print(f"  {'PASS' if ok2 else 'FAIL'} after 1 m to the right: forward {m13[1]:+.2f} right {m13[2]:+.2f} m")
    fails = (not ok0) + (not ok1) + (not ok2) + (rc != 0)
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t_begin:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
