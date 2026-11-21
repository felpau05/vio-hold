#!/usr/bin/env python3
"""Justice D (spec Q3: digital MIPI CSI, 1280x720, 60 fps, 114 x 86 deg) support: settings defaults, lens profile,
downscale, flat-picture judge without the analog "marks" exemption, capture mode string.
Run: ~/.venvs/vio/bin/python tests_apb/test_justice_d.py
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import apb_main            # noqa: E402
import capture             # noqa: E402
import settings as S       # noqa: E402

fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}")


s = S.load("/nonexistent")
check(s["video_kind"] == "csi" and s["field"] == "frame" and s["video_size"] == [1280, 720] and s["video_fps"] == 60.0,
      "defaults describe the Justice D (csi, whole frame, 1280x720, 60 fps)")
_names = {m["name"] for m in json.load(open(os.path.join(os.path.dirname(HERE), s["lens_file"])))["models"]}
_lj = S.load(S.PATH)["lens_name"]
check(s["lens_name"] == "Justice D nominal (spec FOV, uncalibrated)" and _lj.startswith("Justice D") and _lj in _names,
      f"defaults use the nominal Justice D lens; settings.json a Justice D profile in the lens file ({_lj})")

# lens at the scaled picture (640 x 360): the same field of view as the 1280 x 720 profile
K, D = apb_main.lens(s, (360, 640))
hfov = 2 * math.degrees(640 / 2 / K[0, 0])
check(abs(hfov - 114.0) < 0.5 and abs(K[0, 2] - 319.5) < 0.6 and abs(K[1, 2] - 179.5) < 0.6,
      f"lens at 640x360: horizontal FOV {hfov:.1f} deg (spec 114), centre ({K[0, 2]:.1f}, {K[1, 2]:.1f})")
check(apb_main.make_hold(s, (360, 640)) is not None, "Hold builds at 640x360")

g = (np.random.default_rng(1).random((720, 1280)) * 255).astype(np.uint8)
sc = capture.scale_img(g, s["video_scale"])
check(sc.shape == (360, 640) and capture.scale_img(g, 1.0) is g, "video_scale 0.5 gives 640x360; 1.0 passes the frame through")

# flat picture with three bright marks: analog accepts it (night lights), csi must not (no markers in the solution)
flat = np.full((360, 640), 90, np.uint8)
rng = np.random.default_rng(2)
flat = np.clip(flat + rng.normal(0, 3.7, flat.shape), 0, 255).astype(np.uint8)
for x in (100, 300, 500):
    flat[170:190, x:x + 20] = 250
a, c = dict(s, video_kind="analog"), dict(s, video_kind="csi")
check(capture._Judge(a)(flat) is None, "analog: flat picture with 3 marks and sensor noise is passed on to Hold")
check(capture._Judge(c)(flat) == "flat picture (no signal?)", "csi: the same picture is refused (no mark exemption)")
tex = (rng.random((360, 640)) * 255).astype(np.uint8)
j = capture._Judge(c)
check(j(tex) is None and j(tex) == "repeated frame", "csi: textured picture passes, an identical next frame is a repeat")

check(capture.capture_mode(s, {}, (360, 640)) == "csi 640x360 (scale 0.5)", "capture mode string for csi")
try:
    capture.V4L2Source(dict(s, video_device="/dev/nonexistent"))
    check(False, "opening a missing device raises")
except RuntimeError as e:
    check("video_kind" in str(e), "opening a missing device raises a clear error")

print("ALL PASS" if not fails else f"{fails} FAIL")
sys.exit(fails)
