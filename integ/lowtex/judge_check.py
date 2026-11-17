#!/usr/bin/env python3
"""The capture judge (capture._Judge) on flat pictures: snow with tarps and lights at night must pass; flat
no-signal screens, a frozen (repeated) picture and plain snow must not. Pictures from px4sim/camera.py.
Run: ~/.venvs/vio/bin/python integ/lowtex/judge_check.py > integ/lowtex/judge_check.txt"""
import os
import sys

import numpy as np

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), os.path.join(PROJ, "tests")]
import camera as C  # noqa: E402
import capture  # noqa: E402
import settings as S  # noqa: E402
import sitl  # noqa: E402
from bad_pictures import no_signal  # noqa: E402


def render(ground, h, n=2, **case):
    c = dict(sitl.DEFAULTS, **case)
    cam = C.Camera(c, C.make_ground(ground))

    def pose(t):
        t = np.asarray(t, float)
        return 0.3 * t, 0.2 * t, np.full(len(t), h), np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    return [cam.finish(cam.render(k / 25, pose)) for k in range(n)]


rng = np.random.default_rng(1)
flat = lambda g, sd: [np.clip(g + rng.normal(0, sd, (288, 360)), 0, 255).astype(np.uint8) for _ in range(2)]  # noqa: E731
ns = no_signal()
cases = [("snow + tarps + pad 50 m", render("snow_tarps_pad", 50.0), True),
         ("snow + tarps 50 m", render("snow_tarps", 50.0), True),
         ("night lights 50 m", render("night_lights", 50.0, noise=10, exposure_s=0.02), True),
         ("night lights 100 m", render("night_lights", 100.0, noise=10, exposure_s=0.02), True),
         ("plain snow 50 m", render("snow", 50.0), False),
         ("flat grey 60 + noise 2", flat(60, 2), False),
         ("black + noise 10 (no lights)", flat(6, 10), False),
         ("NO SIGNAL screen, first frame", [ns], False),
         ("NO SIGNAL screen, repeated", [ns, ns.copy()], False),
         ("snow + tarps, frozen (repeated)", (lambda f: [f, f.copy()])(render("snow_tarps_pad", 50.0, n=1)[0]), False)]
s = S.DEFAULTS
bad = 0
for name, frames, want in cases:
    j = capture._Judge(s)
    whys = [j(f) for f in frames]
    got = whys[-1] is None
    bad += got != want
    print(f"{name:34s} marks {capture.strong_marks(frames[-1]):2d}  last frame: {whys[-1] or 'accepted':28s} "
          f"{'PASS' if got == want else 'FAIL'} (want {'accepted' if want else 'refused'})")
print("ALL PASS" if not bad else f"{bad} FAIL")
sys.exit(1 if bad else 0)
