"""Check that the analog camera of px4sim/camera.py gives the same pictures as before the Justice D change.

The old camera.py is read from git (HEAD) into a temporary module; both render the same frames with the same
seeds (PAL and NTSC, with and without flicker, propellers, people, vibration), and every byte must agree.
Run: ~/.venvs/vio/bin/python integ/justice_d/check_analog_same.py   (writes integ/justice_d/check_analog_same.txt)
"""
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim")]
import camera as NEW   # noqa: E402
import sitl            # noqa: E402

with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "camera_old.py")
    open(p, "w").write(subprocess.run(["git", "-C", PROJ, "show", "HEAD:px4sim/camera.py"], capture_output=True,
                                      text=True, check=True).stdout)
    spec = importlib.util.spec_from_file_location("camera_old", p)
    OLD = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(OLD)
OLD.LENS_JSON = NEW.LENS_JSON


def pose(t):
    t = np.atleast_1d(t)
    q = np.stack([np.cos(0.05 * np.sin(t)), 0.03 * np.sin(3 * t), 0.02 * np.cos(2 * t), 0.1 * np.sin(0.3 * t)], -1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return 0.4 * np.sin(t), 0.3 * np.cos(t), 80 + np.sin(t), q


cases = [dict(), dict(video="NTSC"), dict(flicker_hz=100.0), dict(props=True), dict(people=2, ground="floor_tiles"),
         dict(vib_deg=0.0, readout_s=-0.018), dict(exposure_s=0.02, noise=10, ground="night_lights")]
lines, bad = [], 0
for case in cases:
    c = dict(sitl.DEFAULTS, **case)
    g = NEW.make_ground(c["ground"])
    a, b = OLD.Camera(c, g), NEW.Camera(c, g)
    same = True
    for k in range(6):
        t0 = 1.0 + k / a.fps
        ia, ib = a.finish(a.render(t0, pose)), b.finish(b.render(t0, pose))
        same &= ia.shape == ib.shape and np.array_equal(ia, ib)
        same &= np.array_equal(a.breakup(ia), b.breakup(ib))
    same &= np.array_equal(a.centre_b, b.centre_b) and a.pixel_of((3.0, 2.0), 0.1, 0.2, 80.0, np.array([1, 0, 0, 0.])) == \
        b.pixel_of((3.0, 2.0), 0.1, 0.2, 80.0, np.array([1, 0, 0, 0.]))
    bad += not same
    lines.append(f"{'SAME' if same else 'DIFFERENT'} {case or 'defaults (PAL)'}: 6 frames {a.W}x{a.H}, break-up, centre ray, pixel_of")
lines.append("ALL SAME" if not bad else f"{bad} DIFFERENT")
print("\n".join(lines))
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_analog_same.txt"), "w").write("\n".join(lines) + "\n")
sys.exit(bad)
