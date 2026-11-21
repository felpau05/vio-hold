#!/usr/bin/env python3
"""Compute time on this laptop, one thread (cv2.setNumThreads(1)): Hold.process per frame at 320x180, 640x360 and
1280x720, and capture.luma plus the INTER_AREA shrink of one 1280x720 YUYV buffer (as capture.DigitalSource.newest).

Pictures: the Justice D of px4sim/camera.py ("sim Justice D near crop"), 1280x720 sensor frames at 60 fps, rendered
first (not timed) from the recorded flight of open_loop.py (100 m, wind 20 gusting 40, seed 11), then shrunk by 1, 2
and 4. Each size gets its own Hold with the calib-like lens at that size and the overlay's hold_cfg; every frame is
processed. These are laptop numbers, not drone numbers: no factor to the APB is applied.

Run: ~/.venvs/vio/bin/python integ/justice_d/compute.py [frames]   (writes integ/justice_d/compute_<time>.json)
"""
import json
import math
import os
import platform
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), HERE]
import camera as C      # noqa: E402
import hold             # noqa: E402
import open_loop as O   # noqa: E402
import sitl             # noqa: E402


def cpu_model():
    for line in subprocess.run(["lscpu"], capture_output=True, text=True).stdout.splitlines():
        if line.startswith("Model name"):
            return line.split(":", 1)[1].strip()
    return platform.processor()


def main(n=400, seed=11, height=100):
    cv2.setNumThreads(1)
    case = dict(video="JUSTICE_D", true_lens="sim Justice D near crop", ground=O.GROUND, seed=seed)
    c = {**sitl.DEFAULTS, **sitl.video_defaults(case), **case}
    cam = C.Camera(c, C.photo_ground(O.GROUND))
    cam.W, cam.H = cam.SW, cam.SH                 # finish() then keeps the full 1280x720 sensor picture
    pose, att, hgt, dy, te, src = O.flight(seed, height)
    t0s = [te + k / cam.fps + cam.exposure / 2 for k in range(n)]
    full = [cam.finish(cam.render(t0, pose)) for t0 in t0s]
    m = C.calib_lens(c["true_lens"])
    cfg = json.load(open(sitl.JUSTICE_D_OVERLAY)).get("hold_cfg", {})
    out = dict(cpu=cpu_model(), threads="cv2.setNumThreads(1); numpy single-threaded", cv2=cv2.__version__,
               numpy=np.__version__, frames=n, skip_first=5, height=height, seed=seed, flight=src,
               hold_py_md5=sitl.HOLD_MD5, started=time.strftime("%Y-%m-%d %H:%M:%S"),
               load_1min_at_start=round(os.getloadavg()[0], 2), sizes={})
    with tempfile.TemporaryDirectory() as d:
        lens_path = os.path.join(d, "lens.json")
        json.dump({"models": [m]}, open(lens_path, "w"))
        for k in (4, 2, 1):
            w, h = cam.SW // k, cam.SH // k
            imgs = [cv2.resize(f, (w, h), interpolation=cv2.INTER_AREA) if k > 1 else f for f in full]
            K, D = hold.load_lens(lens_path, m["name"], w, h)
            H = hold.Hold(K, D, hold.make_R_bc(), c["readout_s"], **cfg)
            N, E, _, _ = pose(te)
            H.reanchor(np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]]) @ np.array([N[0], E[0]]))
            ms, states = [], {}
            for t0, img in zip(t0s, imgs):
                a = time.perf_counter()
                e = H.process(img, t0, att, hgt)
                ms.append(1e3 * (time.perf_counter() - a))
                states[e.state] = states.get(e.state, 0) + 1
            x = np.array(ms[5:])
            out["sizes"][f"{w}x{h}"] = dict(median_ms=round(float(np.median(x)), 2), p95_ms=round(float(np.percentile(x, 95)), 2),
                                            mean_ms=round(float(x.mean()), 2), max_ms=round(float(x.max()), 2),
                                            frames_timed=len(x), states=states)
            print(f"Hold.process {w}x{h}: median {np.median(x):.2f} ms, p95 {np.percentile(x, 95):.2f} ms over {len(x)} frames",
                  flush=True)
    base = out["sizes"]["640x360"]["median_ms"]
    out["ratio_median_to_640x360"] = {s: round(v["median_ms"] / base, 2) for s, v in out["sizes"].items()}
    import capture                                  # luma + shrink, as capture.DigitalSource.newest
    buf = np.empty((720, 1280, 2), np.uint8)
    buf[:, :, 0] = full[0]
    buf[:, :, 1] = 128
    ms = []
    for _ in range(1005):
        a = time.perf_counter()
        g = capture.luma(buf, "YUYV")
        img = cv2.resize(g, (g.shape[1] // 2, g.shape[0] // 2), interpolation=cv2.INTER_AREA)
        ms.append(1e3 * (time.perf_counter() - a))
    assert img.shape == (360, 640)
    x = np.array(ms[5:])
    out["luma_reduce_1280x720_yuyv"] = dict(median_ms=round(float(np.median(x)), 3), p95_ms=round(float(np.percentile(x, 95)), 3),
                                            runs=len(x), capture_py_mtime=time.ctime(os.path.getmtime(capture.__file__)))
    print(f"capture.luma + INTER_AREA 1280x720 YUYV -> 640x360: median {np.median(x):.3f} ms, p95 {np.percentile(x, 95):.3f} ms")
    out["load_1min_at_end"] = round(os.getloadavg()[0], 2)
    path = os.path.join(HERE, f"compute_{time.strftime('%Y%m%d_%H%M%S')}.json")
    json.dump(out, open(path, "w"), indent=1)
    print("wrote", path, "CPU", out["cpu"])


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 400)
